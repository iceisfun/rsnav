//! Interactive demo: author a PSLG with the mouse, hit Create, then probe
//! the resulting navmesh.
//!
//! Authoring tools:
//!   - Add perimeter: left-click drops vertices; right-click (or "Close")
//!     closes the polygon.
//!   - Add hole: same flow, but the closed polygon becomes a hole.
//!   - Create: runs delaunay → form_skeleton → carve_holes → build_from_cdt
//!     → Bsp::build, then switches to exploration.
//!
//! Exploration tools:
//!   - Right-click: pick path source (snapped to navmesh).
//!   - Left-click: pick path destination → A* + funnel path drawn between.
//!   - Hover: shows nearest-point marker, triangle info, and (when a
//!     source is set) a line-of-sight indicator (green clear / red blocked
//!     at the wall hit).

use eframe::egui;
use egui::{Color32, Pos2, Sense, Shape, Stroke, Vec2};

use rsnav_bsp::Bsp;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navigation::{find_path, line_of_sight, nearest_point, LineOfSightResult, PathOptions};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

// =========================================================================
// Entry point
// =========================================================================

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 750.0])
            .with_title("rsnav2 — navmesh demo"),
        ..Default::default()
    };
    eframe::run_native(
        "rsnav-demo",
        native_options,
        Box::new(|_cc| Ok(Box::new(DemoApp::default()))),
    )
}

// =========================================================================
// App state
// =========================================================================

#[derive(Default)]
struct DemoApp {
    // Authoring
    perimeters: Vec<Polygon>,
    holes: Vec<Polygon>,
    drawing: Option<Drawing>,
    next_marker: i32,

    // Build artefacts
    navmesh: Option<NavMesh>,
    bsp: Option<Bsp>,
    last_build_error: Option<String>,

    // Exploration probe state
    path_src: Option<Vertex>,
    last_path: Option<Vec<Vertex>>,
    path_distance_from_wall: f64,
    hover_canvas: Option<Vertex>,
}

#[derive(Clone)]
struct Polygon {
    verts: Vec<Vertex>,
    marker: i32,
}

#[derive(Clone)]
struct Drawing {
    kind: DrawingKind,
    verts: Vec<Vertex>,
    marker: i32,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum DrawingKind {
    Perimeter,
    Hole,
}

impl DemoApp {
    fn in_exploration(&self) -> bool {
        self.navmesh.is_some()
    }

    fn reset(&mut self) {
        *self = DemoApp::default();
    }

    fn cancel_drawing(&mut self) {
        self.drawing = None;
    }

    fn start_drawing(&mut self, kind: DrawingKind) {
        self.next_marker += 1;
        self.drawing = Some(Drawing {
            kind,
            verts: Vec::new(),
            marker: self.next_marker,
        });
    }

    fn close_drawing(&mut self) {
        if let Some(d) = self.drawing.take() {
            if d.verts.len() < 3 {
                // Discard degenerate polygons silently — easier than a
                // modal error popup for a demo.
                return;
            }
            let polygon = Polygon {
                verts: d.verts,
                marker: d.marker,
            };
            match d.kind {
                DrawingKind::Perimeter => self.perimeters.push(polygon),
                DrawingKind::Hole => self.holes.push(polygon),
            }
        }
    }

    fn try_build(&mut self) {
        self.last_build_error = None;
        if self.perimeters.is_empty() {
            self.last_build_error =
                Some("No perimeter yet — add at least one before Create.".into());
            return;
        }

        // Compose a Pslg out of all authored polygons.
        let mut pslg = Pslg::new();
        let mut next_idx = 0u32;
        for poly in self.perimeters.iter().chain(self.holes.iter()) {
            let start = next_idx;
            for v in &poly.verts {
                pslg.vertices.push(PslgVertex::new(*v));
                next_idx += 1;
            }
            // Close the ring with segments.
            let n = poly.verts.len() as u32;
            for i in 0..n {
                let a = start + i;
                let b = start + (i + 1) % n;
                pslg.segments.push(PslgSegment {
                    a,
                    b,
                    marker: poly.marker,
                });
            }
        }

        // Hole seed points — use each hole polygon's centroid.
        for hole in &self.holes {
            let cx: f64 = hole.verts.iter().map(|v| v.x).sum::<f64>() / hole.verts.len() as f64;
            let cy: f64 = hole.verts.iter().map(|v| v.y).sum::<f64>() / hole.verts.len() as f64;
            pslg.holes.push(PslgHole {
                point: Vertex::new(cx, cy),
            });
        }

        // Build pipeline.
        let mut cdt = CdtMesh::new();
        for v in &pslg.vertices {
            cdt.push_vertex(VertexSlot::new(v.position, 0));
        }
        delaunay(&mut cdt, DivConqOptions::default());
        form_skeleton(&mut cdt, &pslg, None);
        carve_holes(&mut cdt, &pslg, false);
        let nav = build_from_cdt(&cdt);
        let bsp = Bsp::build(&nav);

        if nav.triangle_count() == 0 {
            self.last_build_error = Some(
                "Build produced 0 triangles — check your polygon winding and hole placement."
                    .into(),
            );
            return;
        }

        self.navmesh = Some(nav);
        self.bsp = Some(bsp);
        self.drawing = None;
        self.path_src = None;
        self.last_path = None;
    }

    fn compute_path_to(&mut self, goal: Vertex) {
        let (Some(nav), Some(bsp), Some(src)) =
            (&self.navmesh, &self.bsp, self.path_src)
        else {
            return;
        };
        let opts = PathOptions {
            distance_from_wall: self.path_distance_from_wall,
        };
        match find_path(nav, bsp, src, goal, &opts) {
            Ok(res) => self.last_path = Some(res.points),
            Err(_) => self.last_path = None,
        }
    }
}

// =========================================================================
// UI
// =========================================================================

impl eframe::App for DemoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("tools")
            .resizable(false)
            .default_width(220.0)
            .show(ctx, |ui| {
                self.tool_panel(ui);
            });
        egui::CentralPanel::default().show(ctx, |ui| {
            self.canvas_panel(ui);
        });
    }
}

impl DemoApp {
    fn tool_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("rsnav2 demo");
        ui.separator();

        if !self.in_exploration() {
            ui.label("Authoring");
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.button("Add perimeter").clicked() {
                    self.start_drawing(DrawingKind::Perimeter);
                }
                if ui.button("Add hole").clicked() {
                    self.start_drawing(DrawingKind::Hole);
                }
            });
            ui.horizontal(|ui| {
                let close_enabled = self.drawing.as_ref().map_or(false, |d| d.verts.len() >= 3);
                if ui
                    .add_enabled(close_enabled, egui::Button::new("Close polygon"))
                    .clicked()
                {
                    self.close_drawing();
                }
                if ui
                    .add_enabled(self.drawing.is_some(), egui::Button::new("Cancel"))
                    .clicked()
                {
                    self.cancel_drawing();
                }
            });

            ui.add_space(8.0);
            let stats = format!(
                "{} perimeter(s), {} hole(s)",
                self.perimeters.len(),
                self.holes.len()
            );
            ui.label(stats);
            if let Some(d) = &self.drawing {
                ui.label(format!(
                    "Drawing {}: {} vertex/vertices",
                    match d.kind {
                        DrawingKind::Perimeter => "perimeter",
                        DrawingKind::Hole => "hole",
                    },
                    d.verts.len()
                ));
            }

            ui.add_space(12.0);
            let create_enabled = !self.perimeters.is_empty() && self.drawing.is_none();
            if ui
                .add_enabled(
                    create_enabled,
                    egui::Button::new(egui::RichText::new("Create navmesh").strong()),
                )
                .clicked()
            {
                self.try_build();
            }
            if let Some(err) = &self.last_build_error {
                ui.colored_label(Color32::from_rgb(220, 80, 80), err);
            }
        } else {
            ui.label("Exploring");
            ui.add_space(4.0);
            let nav = self.navmesh.as_ref().unwrap();
            ui.label(format!(
                "{} triangle(s), {} region(s)",
                nav.triangle_count(),
                nav.region_count
            ));

            ui.add_space(8.0);
            ui.label("Path");
            ui.add(
                egui::Slider::new(&mut self.path_distance_from_wall, 0.0..=40.0)
                    .text("distance from wall")
                    .step_by(1.0),
            );
            if self.path_src.is_some() {
                ui.label("Right-click: change source · Left-click: pick goal");
            } else {
                ui.label("Right-click on a triangle to set the path source.");
            }
            if ui.button("Clear path").clicked() {
                self.path_src = None;
                self.last_path = None;
            }
        }

        ui.add_space(16.0);
        ui.separator();
        if ui.button("Reset everything").clicked() {
            self.reset();
        }

        ui.add_space(12.0);
        ui.label("Tip: left-click to drop vertices.");
        ui.label("Polygons close on the 'Close' button.");
    }

    fn canvas_panel(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let (response, painter) = ui.allocate_painter(available, Sense::click_and_drag());
        let rect = response.rect;

        // Background
        painter.rect_filled(rect, 0.0, Color32::from_gray(28));

        // Update hover position (cursor in world coords).
        self.hover_canvas = response
            .hover_pos()
            .map(|p| Vertex::new((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64));

        // Mouse handlers depending on mode.
        if !self.in_exploration() {
            self.handle_authoring_mouse(&response, rect);
            self.draw_authoring(&painter, rect);
        } else {
            self.handle_exploration_mouse(&response, rect);
            self.draw_navmesh(&painter, rect);
            self.draw_exploration_overlays(&painter, rect);
        }
    }

    // -- coordinate conversions ----------------------------------------

    fn world_to_screen(rect: egui::Rect, v: Vertex) -> Pos2 {
        Pos2::new(rect.min.x + v.x as f32, rect.min.y + v.y as f32)
    }

    fn screen_to_world(rect: egui::Rect, p: Pos2) -> Vertex {
        Vertex::new((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64)
    }

    // -- authoring -----------------------------------------------------

    fn handle_authoring_mouse(&mut self, response: &egui::Response, rect: egui::Rect) {
        // Only react to clicks inside the canvas.
        if !response.clicked() && !response.secondary_clicked() {
            return;
        }
        let Some(pos) = response.interact_pointer_pos() else { return };
        let world = Self::screen_to_world(rect, pos);

        if response.clicked() {
            if let Some(d) = &mut self.drawing {
                d.verts.push(world);
            }
        }
        if response.secondary_clicked() {
            if self.drawing.as_ref().map_or(false, |d| d.verts.len() >= 3) {
                self.close_drawing();
            }
        }
    }

    fn draw_authoring(&self, painter: &egui::Painter, rect: egui::Rect) {
        // Finalized perimeters (light blue) and holes (orange).
        for p in &self.perimeters {
            self.draw_polygon(painter, rect, &p.verts, Color32::from_rgb(80, 160, 230), true);
        }
        for h in &self.holes {
            self.draw_polygon(painter, rect, &h.verts, Color32::from_rgb(230, 140, 80), true);
        }

        // In-progress polygon (yellow).
        if let Some(d) = &self.drawing {
            let color = match d.kind {
                DrawingKind::Perimeter => Color32::from_rgb(220, 220, 60),
                DrawingKind::Hole => Color32::from_rgb(220, 120, 60),
            };
            self.draw_polygon(painter, rect, &d.verts, color, false);

            // Rubber-band to the cursor.
            if let (Some(last), Some(hover)) = (d.verts.last(), self.hover_canvas) {
                painter.line_segment(
                    [
                        Self::world_to_screen(rect, *last),
                        Self::world_to_screen(rect, hover),
                    ],
                    Stroke::new(1.0, color.gamma_multiply(0.6)),
                );
            }
        }
    }

    fn draw_polygon(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        verts: &[Vertex],
        color: Color32,
        closed: bool,
    ) {
        if verts.is_empty() {
            return;
        }
        let pts: Vec<Pos2> = verts.iter().map(|v| Self::world_to_screen(rect, *v)).collect();
        let stroke = Stroke::new(2.0, color);
        for w in pts.windows(2) {
            painter.line_segment([w[0], w[1]], stroke);
        }
        if closed && pts.len() >= 2 {
            painter.line_segment([*pts.last().unwrap(), pts[0]], stroke);
        }
        for p in &pts {
            painter.circle_filled(*p, 3.0, color);
        }
    }

    // -- exploration ---------------------------------------------------

    fn handle_exploration_mouse(&mut self, response: &egui::Response, rect: egui::Rect) {
        if response.secondary_clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let world = Self::screen_to_world(rect, pos);
                let snapped = self
                    .bsp
                    .as_ref()
                    .zip(self.navmesh.as_ref())
                    .and_then(|(b, n)| nearest_point(n, b, world))
                    .map(|np| np.point)
                    .unwrap_or(world);
                self.path_src = Some(snapped);
                self.last_path = None;
            }
        }
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let world = Self::screen_to_world(rect, pos);
                let snapped = self
                    .bsp
                    .as_ref()
                    .zip(self.navmesh.as_ref())
                    .and_then(|(b, n)| nearest_point(n, b, world))
                    .map(|np| np.point)
                    .unwrap_or(world);
                if self.path_src.is_some() {
                    self.compute_path_to(snapped);
                } else {
                    // No source set yet → first left-click becomes the
                    // source for convenience.
                    self.path_src = Some(snapped);
                }
            }
        }
    }

    fn draw_navmesh(&self, painter: &egui::Painter, rect: egui::Rect) {
        let nav = self.navmesh.as_ref().unwrap();
        let region_count = nav.region_count.max(1);

        for (i, tri) in nav.triangles.iter().enumerate() {
            let v0 = nav.vertex(tri.vertices[0]);
            let v1 = nav.vertex(tri.vertices[1]);
            let v2 = nav.vertex(tri.vertices[2]);
            let pts = [
                Self::world_to_screen(rect, v0),
                Self::world_to_screen(rect, v1),
                Self::world_to_screen(rect, v2),
            ];
            let fill = region_color(tri.region, region_count).gamma_multiply(0.25);
            painter.add(Shape::convex_polygon(
                pts.to_vec(),
                fill,
                Stroke::new(0.8, Color32::from_gray(80)),
            ));
            // Constrained edges drawn in red on top.
            for edge in 0..3 {
                if tri.edge_markers[edge] != 0 || !tri.neighbors[edge].is_valid() {
                    let (a_id, b_id) = (
                        tri.vertices[(edge + 1) % 3],
                        tri.vertices[(edge + 2) % 3],
                    );
                    let pa = Self::world_to_screen(rect, nav.vertex(a_id));
                    let pb = Self::world_to_screen(rect, nav.vertex(b_id));
                    painter.line_segment([pa, pb], Stroke::new(2.0, Color32::from_rgb(220, 70, 70)));
                }
            }
            // Suppress the unused i — keep around in case we want
            // hover-highlight in a future iteration.
            let _ = i;
        }
    }

    fn draw_exploration_overlays(&self, painter: &egui::Painter, rect: egui::Rect) {
        let (Some(nav), Some(bsp)) = (&self.navmesh, &self.bsp) else { return };

        // Path source (if set) — green dot.
        if let Some(src) = self.path_src {
            painter.circle_filled(
                Self::world_to_screen(rect, src),
                5.0,
                Color32::from_rgb(80, 220, 80),
            );
        }

        // Path polyline — bright green.
        if let Some(path) = &self.last_path {
            let pts: Vec<Pos2> = path
                .iter()
                .map(|v| Self::world_to_screen(rect, *v))
                .collect();
            for w in pts.windows(2) {
                painter.line_segment([w[0], w[1]], Stroke::new(3.0, Color32::from_rgb(80, 220, 80)));
            }
            if let Some(last) = pts.last() {
                painter.circle_filled(*last, 5.0, Color32::from_rgb(80, 220, 80));
            }
        }

        // Hover overlays: nearest-point snap marker and LOS line.
        if let Some(h) = self.hover_canvas {
            if let Some(np) = nearest_point(nav, bsp, h) {
                // Snap marker — pale cyan.
                painter.circle_stroke(
                    Self::world_to_screen(rect, np.point),
                    4.0,
                    Stroke::new(1.5, Color32::from_rgb(120, 200, 220)),
                );
                if let Some(src) = self.path_src {
                    if let Some(src_tri) = bsp.locate(nav, src) {
                        let los = line_of_sight(nav, src_tri, src, np.point);
                        let (color, end) = match los {
                            LineOfSightResult::Clear => {
                                (Color32::from_rgb(120, 200, 120), np.point)
                            }
                            LineOfSightResult::Blocked { point } => {
                                (Color32::from_rgb(220, 100, 100), point)
                            }
                            LineOfSightResult::SourceOutsideMesh => {
                                (Color32::from_gray(120), np.point)
                            }
                        };
                        painter.line_segment(
                            [
                                Self::world_to_screen(rect, src),
                                Self::world_to_screen(rect, end),
                            ],
                            Stroke::new(1.5, color),
                        );
                        if let LineOfSightResult::Blocked { point } = los {
                            painter.circle_filled(
                                Self::world_to_screen(rect, point),
                                3.5,
                                color,
                            );
                        }
                    }
                }
                // Hovered triangle info (top-left status).
                if let Some(tri_id) = bsp.locate(nav, h) {
                    let _ = self.draw_tri_info(painter, rect, nav, tri_id);
                }
            }
        }
    }

    fn draw_tri_info(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        nav: &NavMesh,
        tri_id: TriangleId,
    ) -> () {
        let tri = nav.triangle(tri_id);
        let text = format!(
            "tri {} · region {} · area {:.1}",
            tri_id.get(),
            tri.region,
            tri.area
        );
        painter.text(
            rect.min + Vec2::new(8.0, 6.0),
            egui::Align2::LEFT_TOP,
            text,
            egui::FontId::monospace(13.0),
            Color32::from_gray(220),
        );
    }
}

// =========================================================================
// Helpers
// =========================================================================

/// Deterministic per-region color so disconnected regions are visually
/// distinguishable.
fn region_color(region: u32, total: u32) -> Color32 {
    if total <= 1 {
        return Color32::from_rgb(160, 180, 220);
    }
    let golden = 0.61803398875_f32;
    let h = (region as f32 * golden) % 1.0;
    hsv_to_rgb(h, 0.5, 0.95)
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> Color32 {
    let i = (h * 6.0).floor();
    let f = h * 6.0 - i;
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);
    let (r, g, b) = match i as i32 % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    };
    Color32::from_rgb((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
}
