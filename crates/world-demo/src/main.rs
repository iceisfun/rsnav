//! Interactive multi-tile world demo.
//!
//! - **Spawn** built-in navmesh tiles from the palette.
//! - **Drag** tiles around the world (the Move tool); seams re-stitch live, so
//!   sliding two tiles edge-to-edge connects them.
//! - **Path** across tiles: right-click a source, left-click a goal.
//! - **Line of sight**: the Vis tool draws a ray from the source to the cursor,
//!   green when clear, red where a wall (or an unlinked seam) blocks it.
//!
//! Links are created by *placement*: there is no manual weld — line two tiles
//! up and `stitch_all` finds the overlapping border edges.

use eframe::egui;
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};

use rsnav_common::{Aabb, Vertex};
use rsnav_navigation::{LineOfSightResult, TileId, TiledWorld};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
use rsnav_triangle::{carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

const STITCH_TOL: f64 = 1e-6;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 750.0])
            .with_title("rsnav — multi-tile world demo"),
        ..Default::default()
    };
    eframe::run_native(
        "rsnav-world-demo",
        native_options,
        Box::new(|_cc| Ok(Box::<WorldDemo>::default())),
    )
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum Tool {
    Move,
    Path,
    Vis,
}

struct WorldDemo {
    world: TiledWorld,
    tile_count: usize,
    next_spawn: Vertex,

    tool: Tool,
    show_links: bool,

    // path / vis
    src: Option<Vertex>,
    goal: Option<Vertex>,
    path: Option<Vec<Vertex>>,

    // interaction
    hover: Option<Vertex>,
    drag: Option<(TileId, Vertex)>, // (tile, cursor-to-offset delta at grab)

    view: ViewTransform,
    request_fit: bool,
    status: String,
}

impl Default for WorldDemo {
    fn default() -> Self {
        let mut demo = Self {
            world: TiledWorld::new(),
            tile_count: 0,
            next_spawn: Vertex::new(0.0, 0.0),
            tool: Tool::Move,
            show_links: true,
            src: None,
            goal: None,
            path: None,
            hover: None,
            drag: None,
            view: ViewTransform::default(),
            request_fit: true,
            status: "Spawn tiles, drag them edge-to-edge, then path across.".into(),
        };
        // Seed a small scene: two tiles already touching.
        demo.spawn(TileKind::Open);
        demo.spawn(TileKind::Holed);
        demo
    }
}

#[derive(Copy, Clone)]
enum TileKind {
    Open,
    Holed,
    Pillars,
}

impl WorldDemo {
    fn spawn(&mut self, kind: TileKind) {
        let nav = match kind {
            TileKind::Open => open_tile(),
            TileKind::Holed => holed_tile(),
            TileKind::Pillars => pillars_tile(),
        };
        self.world.add_tile(nav, self.next_spawn);
        self.tile_count += 1;
        // Cascade the next spawn to the right so tiles don't stack.
        self.next_spawn = Vertex::new(self.next_spawn.x + 10.0, self.next_spawn.y);
        self.restitch();
        self.request_fit = true;
    }

    fn restitch(&mut self) {
        self.world.stitch_all(STITCH_TOL);
        self.replan();
    }

    fn replan(&mut self) {
        self.path = match (self.src, self.goal) {
            (Some(s), Some(g)) => self.world.find_path(s, g),
            _ => None,
        };
    }

    fn world_aabb(&self) -> Option<Aabb> {
        if self.tile_count == 0 {
            return None;
        }
        let mut aabb = Aabb::EMPTY;
        for i in 0..self.tile_count {
            let b = self.world.tile_world_aabb(TileId(i as u32));
            aabb.extend(b.min);
            aabb.extend(b.max);
        }
        Some(aabb)
    }
}

impl eframe::App for WorldDemo {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("tools")
            .resizable(false)
            .default_width(220.0)
            .show(ctx, |ui| self.tool_panel(ui));
        egui::CentralPanel::default().show(ctx, |ui| self.canvas(ui));
    }
}

impl WorldDemo {
    fn tool_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("World demo");
        ui.separator();

        ui.label("Spawn tile");
        ui.horizontal(|ui| {
            if ui.button("Open").clicked() {
                self.spawn(TileKind::Open);
            }
            if ui.button("Holed").clicked() {
                self.spawn(TileKind::Holed);
            }
            if ui.button("Pillars").clicked() {
                self.spawn(TileKind::Pillars);
            }
        });
        ui.label(format!("{} tile(s), {} link(s)", self.tile_count, self.world.links().len()));

        ui.add_space(8.0);
        ui.label("Tool");
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.tool, Tool::Move, "Move");
            ui.selectable_value(&mut self.tool, Tool::Path, "Path");
            ui.selectable_value(&mut self.tool, Tool::Vis, "Vis");
        });
        match self.tool {
            Tool::Move => ui.label("Drag a tile to reposition it. Seams stitch live."),
            Tool::Path => ui.label("Right-click: source · Left-click: goal."),
            Tool::Vis => ui.label("Right-click: source · move cursor to test LOS."),
        };

        ui.add_space(8.0);
        ui.checkbox(&mut self.show_links, "show links (green seams)");

        ui.add_space(8.0);
        if ui.button("Clear path / source").clicked() {
            self.src = None;
            self.goal = None;
            self.path = None;
        }
        if ui.button("Fit view").clicked() {
            self.request_fit = true;
        }
        if ui.button("Reset world").clicked() {
            *self = WorldDemo::default();
        }

        ui.add_space(12.0);
        ui.separator();
        ui.label(&self.status);
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        painter.rect_filled(rect, 0.0, Color32::from_gray(26));

        if self.request_fit {
            self.request_fit = false;
            if let Some(world) = self.world_aabb() {
                self.view = ViewTransform::fit(world, rect.size(), 40.0);
            }
        }

        self.hover = response.hover_pos().map(|p| self.view.screen_to_world(rect, p));
        self.handle_input(&response, rect);
        self.draw(&painter, rect);
    }

    fn handle_input(&mut self, response: &egui::Response, rect: Rect) {
        let pointer_world = response
            .interact_pointer_pos()
            .map(|p| self.view.screen_to_world(rect, p));

        match self.tool {
            Tool::Move => {
                if response.drag_started() {
                    if let Some(w) = pointer_world {
                        if let Some(tile) = self.tile_at(w) {
                            let off = self.world.tile_offset(tile);
                            self.drag = Some((tile, Vertex::new(w.x - off.x, w.y - off.y)));
                        }
                    }
                }
                if response.dragged() {
                    if let (Some((tile, grab)), Some(w)) = (self.drag, pointer_world) {
                        self.world
                            .set_tile_offset(tile, Vertex::new(w.x - grab.x, w.y - grab.y));
                        self.restitch();
                    }
                }
                if response.drag_stopped() {
                    self.drag = None;
                }
            }
            Tool::Path => {
                if response.secondary_clicked() {
                    self.src = pointer_world;
                    self.path = None;
                } else if response.clicked() {
                    self.goal = pointer_world;
                    self.replan();
                }
            }
            Tool::Vis => {
                if response.secondary_clicked() {
                    self.src = pointer_world;
                }
            }
        }
    }

    fn tile_at(&self, p: Vertex) -> Option<TileId> {
        // Topmost tile whose world bounds contain p.
        (0..self.tile_count)
            .rev()
            .map(|i| TileId(i as u32))
            .find(|&t| self.world.tile_world_aabb(t).contains(p))
    }

    fn draw(&self, painter: &egui::Painter, rect: Rect) {
        // Tiles: fill triangles tinted per tile, wall edges in muted red.
        for i in 0..self.tile_count {
            let tile = TileId(i as u32);
            let off = self.world.tile_offset(tile);
            let nav = self.world.tile_nav(tile);
            let fill = tile_color(i).gamma_multiply(0.16);
            for tri in &nav.triangles {
                let pts: Vec<Pos2> = (0..3)
                    .map(|k| self.w2s(rect, nav.vertex(tri.vertices[k]), off))
                    .collect();
                painter.add(Shape::convex_polygon(
                    pts,
                    fill,
                    Stroke::new(0.6, Color32::from_gray(70)),
                ));
                for e in 0..3 {
                    if tri.edge_markers[e] != 0 || !tri.neighbors[e].is_valid() {
                        let a = self.w2s(rect, nav.vertex(tri.vertices[(e + 1) % 3]), off);
                        let b = self.w2s(rect, nav.vertex(tri.vertices[(e + 2) % 3]), off);
                        painter
                            .line_segment([a, b], Stroke::new(1.5, Color32::from_rgb(170, 70, 70)));
                    }
                }
            }
            // Tile bounds + label.
            let b = self.world.tile_world_aabb(tile);
            let r = Rect::from_min_max(
                self.w2s(rect, b.min, Vertex::new(0.0, 0.0)),
                self.w2s(rect, b.max, Vertex::new(0.0, 0.0)),
            );
            painter.rect_stroke(r, 0.0, Stroke::new(1.0, tile_color(i).gamma_multiply(0.7)));
            painter.text(
                r.left_top() + Vec2::new(3.0, 1.0),
                egui::Align2::LEFT_TOP,
                format!("#{i}"),
                egui::FontId::proportional(12.0),
                tile_color(i),
            );
        }

        // Links: bright green seam segments.
        if self.show_links {
            for l in self.world.links() {
                let a = self.w2s(rect, l.portal.0, Vertex::new(0.0, 0.0));
                let b = self.w2s(rect, l.portal.1, Vertex::new(0.0, 0.0));
                painter.line_segment([a, b], Stroke::new(3.0, Color32::from_rgb(90, 220, 120)));
            }
        }

        // Source marker.
        if let Some(s) = self.src {
            painter.circle_filled(
                self.w2s(rect, s, Vertex::new(0.0, 0.0)),
                5.0,
                Color32::from_rgb(80, 220, 80),
            );
        }

        // Path polyline.
        if let Some(path) = &self.path {
            let pts: Vec<Pos2> = path
                .iter()
                .map(|v| self.w2s(rect, *v, Vertex::new(0.0, 0.0)))
                .collect();
            for w in pts.windows(2) {
                painter.line_segment([w[0], w[1]], Stroke::new(3.0, Color32::from_rgb(80, 220, 80)));
            }
            if let Some(last) = pts.last() {
                painter.circle_filled(*last, 5.0, Color32::from_rgb(80, 220, 80));
            }
        }

        // Vis ray: source → cursor, colored by LOS.
        if self.tool == Tool::Vis {
            if let (Some(s), Some(h)) = (self.src, self.hover) {
                let (color, end) = match self.world.line_of_sight(s, h) {
                    LineOfSightResult::Clear => (Color32::from_rgb(120, 220, 120), h),
                    LineOfSightResult::Blocked { point } => (Color32::from_rgb(225, 90, 90), point),
                    LineOfSightResult::SourceOutsideMesh => (Color32::from_gray(120), h),
                    LineOfSightResult::Indeterminate => (Color32::from_rgb(225, 190, 90), h),
                };
                painter.line_segment(
                    [
                        self.w2s(rect, s, Vertex::new(0.0, 0.0)),
                        self.w2s(rect, end, Vertex::new(0.0, 0.0)),
                    ],
                    Stroke::new(2.0, color),
                );
                painter.circle_stroke(
                    self.w2s(rect, end, Vertex::new(0.0, 0.0)),
                    4.0,
                    Stroke::new(1.5, color),
                );
            }
        }
    }

    /// World-local point (`local + offset`) to screen.
    #[inline]
    fn w2s(&self, rect: Rect, local: Vertex, offset: Vertex) -> Pos2 {
        self.view
            .world_to_screen(rect, Vertex::new(local.x + offset.x, local.y + offset.y))
    }
}

// =========================================================================
// Built-in tiles (all 10×10 so they stitch on a grid)
// =========================================================================

/// A hole ring (edge index pairs) plus an interior point marking it.
type Hole<'a> = (&'a [(usize, usize)], (f64, f64));

fn build(pts: &[(f64, f64)], outer: &[(usize, usize)], holes: &[Hole]) -> NavMesh {
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    for &(x, y) in pts {
        cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
    }
    for &(a, b) in outer {
        pslg.segments.push(PslgSegment { a: a as u32, b: b as u32, marker: 1 });
    }
    for (ring, inside) in holes {
        for &(a, b) in *ring {
            pslg.segments.push(PslgSegment { a: a as u32, b: b as u32, marker: 2 });
        }
        pslg.holes.push(PslgHole { point: Vertex::new(inside.0, inside.1) });
    }
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    if !holes.is_empty() {
        carve_holes(&mut cdt, &pslg, false);
    }
    build_from_cdt(&cdt)
}

fn open_tile() -> NavMesh {
    build(
        &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        &[(0, 1), (1, 2), (2, 3), (3, 0)],
        &[],
    )
}

fn holed_tile() -> NavMesh {
    build(
        &[
            (0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0),
            (3.5, 3.5), (6.5, 3.5), (6.5, 6.5), (3.5, 6.5),
        ],
        &[(0, 1), (1, 2), (2, 3), (3, 0)],
        &[(&[(4, 5), (5, 6), (6, 7), (7, 4)], (5.0, 5.0))],
    )
}

fn pillars_tile() -> NavMesh {
    build(
        &[
            (0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0),
            (2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0), // lower-left pillar
            (6.0, 6.0), (8.0, 6.0), (8.0, 8.0), (6.0, 8.0), // upper-right pillar
        ],
        &[(0, 1), (1, 2), (2, 3), (3, 0)],
        &[
            (&[(4, 5), (5, 6), (6, 7), (7, 4)], (3.0, 3.0)),
            (&[(8, 9), (9, 10), (10, 11), (11, 8)], (7.0, 7.0)),
        ],
    )
}

fn tile_color(i: usize) -> Color32 {
    const PALETTE: [Color32; 6] = [
        Color32::from_rgb(120, 170, 230),
        Color32::from_rgb(230, 170, 110),
        Color32::from_rgb(150, 210, 150),
        Color32::from_rgb(210, 150, 200),
        Color32::from_rgb(220, 210, 130),
        Color32::from_rgb(150, 200, 210),
    ];
    PALETTE[i % PALETTE.len()]
}

// =========================================================================
// View transform (world ↔ screen)
// =========================================================================

#[derive(Copy, Clone)]
struct ViewTransform {
    scale: f32,
    offset: Vec2, // canvas-local pixel position of world origin
}

impl Default for ViewTransform {
    fn default() -> Self {
        Self { scale: 20.0, offset: Vec2::new(60.0, 60.0) }
    }
}

impl ViewTransform {
    fn fit(world: Aabb, canvas: Vec2, pad: f32) -> Self {
        let w = (world.max.x - world.min.x).max(1e-6);
        let h = (world.max.y - world.min.y).max(1e-6);
        let sx = (canvas.x - 2.0 * pad) as f64 / w;
        let sy = (canvas.y - 2.0 * pad) as f64 / h;
        let scale = sx.min(sy).max(1e-3) as f32;
        let cx = ((world.min.x + world.max.x) * 0.5) as f32;
        let cy = ((world.min.y + world.max.y) * 0.5) as f32;
        Self {
            scale,
            offset: Vec2::new(canvas.x * 0.5 - cx * scale, canvas.y * 0.5 - cy * scale),
        }
    }

    fn world_to_screen(&self, rect: Rect, v: Vertex) -> Pos2 {
        Pos2::new(
            rect.min.x + v.x as f32 * self.scale + self.offset.x,
            rect.min.y + v.y as f32 * self.scale + self.offset.y,
        )
    }

    fn screen_to_world(&self, rect: Rect, p: Pos2) -> Vertex {
        Vertex::new(
            ((p.x - rect.min.x - self.offset.x) / self.scale) as f64,
            ((p.y - rect.min.y - self.offset.y) / self.scale) as f64,
        )
    }
}
