//! `rsnav-smoothing-demo` — side-by-side comparison of the bitfield-edge
//! smoothing options.
//!
//! The same bitfield is built into two navmeshes:
//!
//! - **Left**: baseline. `diagonal_smoothing = false`, `clip_ears_max_area = 0`.
//! - **Right**: with the current option settings applied.
//!
//! Both meshes are rendered with the same scale; constrained edges (walls)
//! are drawn on top. The side panel shows triangle and vertex counts so
//! the effect of each option is visible at a glance.
//!
//! Several preset bitfields highlight different artifact patterns: a clean
//! 45° triangle, a multi-arm pinwheel, a jagged blob, and an L-corridor.
//! Click any walkable cell on the left canvas to flip it to wall (and vice
//! versa) — both navmeshes rebuild live.

use eframe::egui;
use egui::{Color32, Pos2, Rect, Sense, Stroke};

use rsnav_common::Vertex;
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions, NavBuild};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

// =========================================================================
// World constants
// =========================================================================

const GRID_W: u32 = 48;
const GRID_H: u32 = 32;

// =========================================================================
// Presets
// =========================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Preset {
    DiagonalTriangle,
    Pinwheel,
    JaggedBlob,
    LCorridor,
}

impl Preset {
    fn all() -> &'static [Preset] {
        &[
            Preset::DiagonalTriangle,
            Preset::Pinwheel,
            Preset::JaggedBlob,
            Preset::LCorridor,
        ]
    }

    fn label(self) -> &'static str {
        match self {
            Preset::DiagonalTriangle => "45° triangle",
            Preset::Pinwheel => "Pinwheel",
            Preset::JaggedBlob => "Jagged blob",
            Preset::LCorridor => "L-corridor",
        }
    }

    fn fill(self, cells: &mut [bool]) {
        cells.fill(false);
        match self {
            Preset::DiagonalTriangle => {
                // Cells (col, row) walkable when col + row < some threshold.
                let limit = GRID_W.min(GRID_H);
                for row in 0..GRID_H {
                    for col in 0..GRID_W {
                        if col + row < limit {
                            cells[idx(col, row)] = true;
                        }
                    }
                }
            }
            Preset::Pinwheel => {
                // Four stair-arm pinwheel emanating from the center.
                let cx = GRID_W as i32 / 2;
                let cy = GRID_H as i32 / 2;
                for row in 0..GRID_H as i32 {
                    for col in 0..GRID_W as i32 {
                        let dx = col - cx;
                        let dy = row - cy;
                        // Two crossed stair-bands; thickness varies with distance.
                        let on_band = (dx + dy).abs() <= 4 || (dx - dy).abs() <= 4;
                        let in_circle = dx * dx + dy * dy <= 12 * 12;
                        if on_band && in_circle {
                            cells[idx(col as u32, row as u32)] = true;
                        }
                    }
                }
            }
            Preset::JaggedBlob => {
                // Pseudo-random blob: union of two skewed ellipses sampled
                // through a hand-rolled hash so each cell is independent and
                // creates a noisy 1-cell-deep boundary.
                let cx = GRID_W as f64 * 0.5;
                let cy = GRID_H as f64 * 0.5;
                for row in 0..GRID_H {
                    for col in 0..GRID_W {
                        let fx = col as f64 + 0.5 - cx;
                        let fy = row as f64 + 0.5 - cy;
                        // Tilted ellipse.
                        let ang = std::f64::consts::FRAC_PI_4;
                        let (s, c) = (ang.sin(), ang.cos());
                        let ex = c * fx - s * fy;
                        let ey = s * fx + c * fy;
                        let inside = (ex / 14.0).powi(2) + (ey / 9.0).powi(2) < 1.0;
                        let nudge = (hash2(col, row) as f64 / u32::MAX as f64) - 0.5;
                        let inside = if inside {
                            (ex / 14.0).powi(2) + (ey / 9.0).powi(2) + 0.12 * nudge < 1.0
                        } else {
                            (ex / 14.0).powi(2) + (ey / 9.0).powi(2) - 0.18 * nudge < 0.85
                        };
                        if inside {
                            cells[idx(col, row)] = true;
                        }
                    }
                }
            }
            Preset::LCorridor => {
                // L-shaped walkable corridor: 6-wide horizontal arm at the
                // bottom, 6-wide vertical arm on the right, meeting at the
                // corner. Then carve an angled stair across one of the
                // arms for variety.
                for col in 0..GRID_W {
                    for row in 0..6 {
                        cells[idx(col, row)] = true;
                    }
                }
                for row in 0..GRID_H {
                    for col in (GRID_W - 6)..GRID_W {
                        cells[idx(col, row)] = true;
                    }
                }
                // Angled stair: carve a slim diagonal cut into the horizontal arm.
                for k in 0..16i32 {
                    let col = 4 + k;
                    let row = 5 - (k / 2);
                    if col >= 0 && col < GRID_W as i32 && row >= 0 && row < GRID_H as i32 {
                        cells[idx(col as u32, row as u32)] = false;
                    }
                }
            }
        }
    }
}

fn hash2(x: u32, y: u32) -> u32 {
    let mut h = x.wrapping_mul(0x9E37_79B1).wrapping_add(y.wrapping_mul(0x85EB_CA77));
    h ^= h >> 16;
    h = h.wrapping_mul(0x27D4_EB2F);
    h ^= h >> 15;
    h.wrapping_mul(0x1656_67B1)
}

#[inline]
fn idx(col: u32, row: u32) -> usize {
    (row * GRID_W + col) as usize
}

// =========================================================================
// Build pipeline wrapper
// =========================================================================

fn build(cells: &[bool], opts: &BuildOptions) -> Option<NavBuild> {
    let bf = Bitfield::new(GRID_W, GRID_H, cells.to_vec()).expect("dims");
    build_navmesh_from_bitfield(&bf, opts).ok()
}

// =========================================================================
// App
// =========================================================================

struct SmoothingDemoApp {
    cells: Vec<bool>,
    preset: Preset,
    diagonal_smoothing: bool,
    clip_ears_max_area: f64,

    baseline: Option<NavBuild>,
    tuned: Option<NavBuild>,
    dirty: bool,
}

impl SmoothingDemoApp {
    fn new() -> Self {
        let mut cells = vec![false; (GRID_W * GRID_H) as usize];
        Preset::DiagonalTriangle.fill(&mut cells);
        // Initial sliders match BuildOptions::default() — both passes on.
        let mut s = Self {
            cells,
            preset: Preset::DiagonalTriangle,
            diagonal_smoothing: true,
            clip_ears_max_area: 0.6,
            baseline: None,
            tuned: None,
            dirty: true,
        };
        s.rebuild();
        s
    }

    fn rebuild(&mut self) {
        let mut baseline_opts = BuildOptions::default();
        baseline_opts.extract.diagonal_smoothing = false;
        baseline_opts.clip_ears_max_area = 0.0;
        self.baseline = build(&self.cells, &baseline_opts);

        let mut tuned_opts = BuildOptions::default();
        tuned_opts.extract.diagonal_smoothing = self.diagonal_smoothing;
        tuned_opts.clip_ears_max_area = self.clip_ears_max_area;
        self.tuned = build(&self.cells, &tuned_opts);

        self.dirty = false;
    }

    fn set_preset(&mut self, p: Preset) {
        if self.preset == p {
            return;
        }
        self.preset = p;
        p.fill(&mut self.cells);
        self.dirty = true;
    }

    fn toggle_cell(&mut self, world: Vertex) {
        let col = world.x.floor();
        let row = world.y.floor();
        if col < 0.0 || row < 0.0 || col >= GRID_W as f64 || row >= GRID_H as f64 {
            return;
        }
        let i = idx(col as u32, row as u32);
        self.cells[i] = !self.cells[i];
        self.dirty = true;
    }

    fn set_cell(&mut self, world: Vertex, walkable: bool) {
        let col = world.x.floor();
        let row = world.y.floor();
        if col < 0.0 || row < 0.0 || col >= GRID_W as f64 || row >= GRID_H as f64 {
            return;
        }
        let i = idx(col as u32, row as u32);
        if self.cells[i] != walkable {
            self.cells[i] = walkable;
            self.dirty = true;
        }
    }

    // ----- UI -----

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Smoothing demo");
        ui.label("Compare bitfield → CDT artifacts with and without the smoothing passes.");
        ui.add_space(8.0);

        ui.label("Preset");
        let mut chosen = self.preset;
        for &p in Preset::all() {
            if ui.radio_value(&mut chosen, p, p.label()).clicked() {
                // handled below
            }
        }
        if chosen != self.preset {
            self.set_preset(chosen);
        }

        ui.add_space(12.0);
        ui.separator();
        ui.add_space(6.0);

        ui.label("Options (right canvas)");
        if ui
            .checkbox(&mut self.diagonal_smoothing, "diagonal_smoothing")
            .changed()
        {
            self.dirty = true;
        }
        ui.label("Polygon-stage: collapse unit-step stair runs to diagonals.");

        ui.add_space(6.0);
        if ui
            .add(
                egui::Slider::new(&mut self.clip_ears_max_area, 0.0..=1.5)
                    .text("clip_ears_max_area")
                    .step_by(0.05),
            )
            .changed()
        {
            self.dirty = true;
        }
        ui.label("CDT-stage: prune ear triangles below this area (0 = off).");

        ui.add_space(12.0);
        ui.separator();
        ui.add_space(6.0);

        ui.label("Counts");
        let (b_tri, b_vert) = count(&self.baseline);
        let (t_tri, t_vert) = count(&self.tuned);
        egui::Grid::new("counts").striped(true).num_columns(3).show(ui, |ui| {
            ui.label("");
            ui.label("baseline");
            ui.label("tuned");
            ui.end_row();
            ui.label("triangles");
            ui.label(format!("{b_tri}"));
            ui.label(format!("{t_tri}"));
            ui.end_row();
            ui.label("vertices");
            ui.label(format!("{b_vert}"));
            ui.label(format!("{t_vert}"));
            ui.end_row();
            ui.label("Δ triangles");
            ui.label("");
            ui.label(format!("{}", t_tri as i64 - b_tri as i64));
            ui.end_row();
        });

        ui.add_space(12.0);
        ui.separator();
        ui.add_space(6.0);

        ui.label("Click a cell on either canvas to toggle it.");
        ui.label("Drag with LMB to paint walls, RMB to paint walkable.");

        if ui.button("Reset preset").clicked() {
            self.preset.fill(&mut self.cells);
            self.dirty = true;
        }
    }

    fn split_canvas(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size_before_wrap();
        let (response, painter) = ui.allocate_painter(available, Sense::click_and_drag());
        let outer = response.rect;

        // Two equal sub-rects with a 2px gutter.
        let half_w = (outer.width() - 2.0) * 0.5;
        let left = Rect::from_min_size(outer.min, egui::vec2(half_w, outer.height()));
        let right = Rect::from_min_size(
            Pos2::new(outer.min.x + half_w + 2.0, outer.min.y),
            egui::vec2(half_w, outer.height()),
        );

        // Editing — click toggles a single cell; drag with the left button
        // paints walls (so you can drag-fill).
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                if left.contains(pos) {
                    self.toggle_cell(screen_to_world(left, pos));
                } else if right.contains(pos) {
                    self.toggle_cell(screen_to_world(right, pos));
                }
            }
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            if let Some(pos) = response.interact_pointer_pos() {
                if left.contains(pos) {
                    self.set_cell(screen_to_world(left, pos), false);
                } else if right.contains(pos) {
                    self.set_cell(screen_to_world(right, pos), false);
                }
            }
        }
        if response.dragged_by(egui::PointerButton::Secondary) {
            if let Some(pos) = response.interact_pointer_pos() {
                if left.contains(pos) {
                    self.set_cell(screen_to_world(left, pos), true);
                } else if right.contains(pos) {
                    self.set_cell(screen_to_world(right, pos), true);
                }
            }
        }

        Self::paint_panel(&painter, left, &self.cells, self.baseline.as_ref(), "baseline");
        Self::paint_panel(&painter, right, &self.cells, self.tuned.as_ref(), "tuned");

        if self.dirty {
            self.rebuild();
        }
    }

    fn paint_panel(
        painter: &egui::Painter,
        rect: Rect,
        cells: &[bool],
        build: Option<&NavBuild>,
        title: &str,
    ) {
        painter.rect_filled(rect, 0.0, Color32::from_gray(22));
        painter.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::from_gray(60)));

        // Wall cells (the 'false' cells).
        for row in 0..GRID_H {
            for col in 0..GRID_W {
                if cells[idx(col, row)] {
                    continue;
                }
                let v0 = world_to_screen(rect, Vertex::new(col as f64, row as f64));
                let v1 = world_to_screen(rect, Vertex::new(col as f64 + 1.0, row as f64 + 1.0));
                painter.rect_filled(
                    Rect::from_two_pos(v0, v1),
                    0.0,
                    Color32::from_rgb(70, 70, 80),
                );
            }
        }

        // Navmesh fill + edges.
        if let Some(b) = build {
            let nav: &NavMesh = &b.navmesh;
            for tri in &nav.triangles {
                let pa = world_to_screen(rect, nav.vertex(tri.vertices[0]));
                let pb = world_to_screen(rect, nav.vertex(tri.vertices[1]));
                let pc = world_to_screen(rect, nav.vertex(tri.vertices[2]));
                painter.add(egui::Shape::convex_polygon(
                    vec![pa, pb, pc],
                    Color32::from_rgba_unmultiplied(80, 160, 110, 70),
                    Stroke::new(0.6, Color32::from_rgba_unmultiplied(180, 200, 200, 110)),
                ));
            }
            // Constrained edges on top.
            for tri in &nav.triangles {
                for edge in 0..3 {
                    if tri.edge_markers[edge] == 0 {
                        continue;
                    }
                    let (a, b_id) = tri.edge_vertices(edge);
                    let pa = world_to_screen(rect, nav.vertex(a));
                    let pb = world_to_screen(rect, nav.vertex(b_id));
                    painter.line_segment(
                        [pa, pb],
                        Stroke::new(1.6, Color32::from_rgb(230, 200, 90)),
                    );
                }
            }
        } else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "no walkable region",
                egui::FontId::proportional(14.0),
                Color32::from_gray(180),
            );
        }

        // Title pill.
        painter.text(
            Pos2::new(rect.min.x + 8.0, rect.min.y + 6.0),
            egui::Align2::LEFT_TOP,
            title,
            egui::FontId::proportional(13.0),
            Color32::from_rgb(220, 230, 220),
        );
    }
}

fn count(build: &Option<NavBuild>) -> (usize, usize) {
    match build {
        Some(b) => (b.navmesh.triangle_count(), b.navmesh.vertices.len()),
        None => (0, 0),
    }
}

// =========================================================================
// Coordinate transforms
// =========================================================================

fn cell_size_px(rect: Rect) -> f32 {
    (rect.width() / GRID_W as f32).min(rect.height() / GRID_H as f32)
}

fn world_to_screen(rect: Rect, v: Vertex) -> Pos2 {
    let s = cell_size_px(rect);
    let total_w = s * GRID_W as f32;
    let total_h = s * GRID_H as f32;
    let ox = rect.min.x + (rect.width() - total_w) * 0.5;
    let oy = rect.min.y + (rect.height() - total_h) * 0.5;
    Pos2::new(ox + v.x as f32 * s, oy + total_h - v.y as f32 * s)
}

fn screen_to_world(rect: Rect, p: Pos2) -> Vertex {
    let s = cell_size_px(rect);
    let total_w = s * GRID_W as f32;
    let total_h = s * GRID_H as f32;
    let ox = rect.min.x + (rect.width() - total_w) * 0.5;
    let oy = rect.min.y + (rect.height() - total_h) * 0.5;
    Vertex::new(
        ((p.x - ox) / s) as f64,
        ((oy + total_h - p.y) / s) as f64,
    )
}

// =========================================================================
// eframe glue
// =========================================================================

impl eframe::App for SmoothingDemoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("controls")
            .resizable(false)
            .min_width(280.0)
            .show(ctx, |ui| {
                self.side_panel(ui);
            });
        egui::CentralPanel::default().show(ctx, |ui| {
            self.split_canvas(ui);
        });
    }
}

fn main() -> eframe::Result<()> {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 720.0])
            .with_title("rsnav-smoothing-demo — diagonal_smoothing + clip_ears"),
        ..Default::default()
    };
    eframe::run_native(
        "rsnav-smoothing-demo",
        opts,
        Box::new(|_cc| Ok(Box::new(SmoothingDemoApp::new()))),
    )
}
