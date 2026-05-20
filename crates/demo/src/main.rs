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

use std::path::{Path, PathBuf};

use eframe::egui;
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use serde::{Deserialize, Serialize};

use rsnav_bsp::Bsp;
use rsnav_common::{Aabb, Polygon as CommonPolygon, TriangleId, Vertex};
use rsnav_navigation::{
    find_path, line_of_sight, nearest_point, visibility_region, LineOfSightResult, PathOptions,
};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

/// File the Save / Load buttons read and write. Kept in CWD so it's easy
/// to find next to the binary; rename interesting captures to keep them.
const DEBUG_FILE: &str = "rsnav-debug.json";

/// Default directory for the "fixtures" browser. The text input under the
/// Fixtures section is pre-filled with this and is editable, so you can
/// point at any directory of *.json files.
const DEFAULT_FIXTURES_DIR: &str = "./testdata";

// =========================================================================
// Entry point
// =========================================================================

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 750.0])
            .with_title("rsnav — navmesh demo"),
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
    show_visibility: bool,
    visibility_radius: f64,
    visibility_samples: usize,

    // Status line shown under the tool panel (Save / Load / Build feedback).
    status: Option<String>,

    // Fixture browser: which directory to scan and the last-scanned listing
    // so we don't read the filesystem every frame.
    fixtures_dir: String,
    fixture_listing: Vec<PathBuf>,
    fixtures_scanned_from: Option<String>,

    // Canvas view: world ↔ canvas-pixel transform. `request_fit = true`
    // makes the next `canvas_panel` recompute fit from the current
    // geometry — set this after Load/Create or when the user clicks
    // "Fit view".
    view: ViewTransform,
    request_fit: bool,
}

/// Affine transform from world coords to canvas-local pixel coords.
///
/// Screen coords add `rect.min` on top — see [`DemoApp::world_to_screen`].
#[derive(Copy, Clone, Debug)]
struct ViewTransform {
    /// World units per canvas pixel.
    scale: f32,
    /// Canvas-local pixel position of the world origin (0, 0).
    offset: Vec2,
}

impl Default for ViewTransform {
    fn default() -> Self {
        Self { scale: 1.0, offset: Vec2::ZERO }
    }
}

impl ViewTransform {
    /// Compute a uniform-scale transform that maps `world` to fill
    /// `canvas_size`, leaving `padding` pixels around all sides.
    fn fit(world: Aabb, canvas_size: Vec2, padding: f32) -> Self {
        let w = (world.max.x - world.min.x).max(1e-9);
        let h = (world.max.y - world.min.y).max(1e-9);
        let avail_w = (canvas_size.x - 2.0 * padding).max(1.0) as f64;
        let avail_h = (canvas_size.y - 2.0 * padding).max(1.0) as f64;
        // Uniform scale — keep the aspect ratio. The smaller of the two
        // available-axis ratios wins.
        let scale = ((avail_w / w).min(avail_h / h)) as f32;
        let world_cx = ((world.min.x + world.max.x) * 0.5) as f32;
        let world_cy = ((world.min.y + world.max.y) * 0.5) as f32;
        let canvas_cx = canvas_size.x * 0.5;
        let canvas_cy = canvas_size.y * 0.5;
        Self {
            scale,
            offset: Vec2::new(canvas_cx - world_cx * scale, canvas_cy - world_cy * scale),
        }
    }

    fn world_to_canvas(&self, v: Vertex) -> Pos2 {
        Pos2::new(
            (v.x as f32) * self.scale + self.offset.x,
            (v.y as f32) * self.scale + self.offset.y,
        )
    }

    fn canvas_to_world(&self, p: Pos2) -> Vertex {
        Vertex::new(
            ((p.x - self.offset.x) / self.scale) as f64,
            ((p.y - self.offset.y) / self.scale) as f64,
        )
    }
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

impl Default for DemoApp {
    fn default() -> Self {
        Self {
            perimeters: Vec::new(),
            holes: Vec::new(),
            drawing: None,
            next_marker: 0,
            navmesh: None,
            bsp: None,
            last_build_error: None,
            path_src: None,
            last_path: None,
            path_distance_from_wall: 0.0,
            hover_canvas: None,
            show_visibility: false,
            visibility_radius: 200.0,
            visibility_samples: 180,
            status: None,
            fixtures_dir: DEFAULT_FIXTURES_DIR.to_string(),
            fixture_listing: Vec::new(),
            fixtures_scanned_from: None,
            view: ViewTransform::default(),
            request_fit: false,
        }
    }
}

impl DemoApp {
    fn in_exploration(&self) -> bool {
        self.navmesh.is_some()
    }

    fn reset(&mut self) {
        *self = DemoApp::default();
    }

    /// Tear down the built navmesh and any exploration state, but keep
    /// the authored polygons so the user can edit and re-Create.
    fn back_to_authoring(&mut self) {
        self.navmesh = None;
        self.bsp = None;
        self.path_src = None;
        self.last_path = None;
        self.last_build_error = None;
        self.drawing = None;
    }

    /// Serialize the current authored polygons to JSON and write to
    /// [`DEBUG_FILE`]. Updates `status`.
    fn save_debug(&mut self) {
        let file = SaveFile {
            version: 1,
            perimeters: self
                .perimeters
                .iter()
                .map(|p| p.verts.iter().map(Point::from).collect())
                .collect(),
            outer_polygon: None,
            holes: self
                .holes
                .iter()
                .map(|p| p.verts.iter().map(Point::from).collect())
                .collect(),
        };
        match serde_json::to_string_pretty(&file) {
            Ok(json) => {
                let path = save_path();
                match std::fs::write(&path, json) {
                    Ok(()) => self.status = Some(format!("saved → {}", path.display())),
                    Err(e) => self.status = Some(format!("save failed: {e}")),
                }
            }
            Err(e) => self.status = Some(format!("serialize failed: {e}")),
        }
    }

    /// Load polygons from [`DEBUG_FILE`]. Accepts both our own format
    /// (`perimeters` array) and the gonav fixture format
    /// (`outer_polygon`, single perimeter).
    fn load_debug(&mut self) {
        let path = save_path();
        self.load_from_path(&path);
    }

    /// Read and adopt polygons from any JSON file. Used by both the
    /// debug Save/Load flow and the fixture browser.
    fn load_from_path(&mut self, path: &Path) {
        let text = match std::fs::read_to_string(path) {
            Ok(s) => s,
            Err(e) => {
                self.status = Some(format!("load failed: {e}"));
                return;
            }
        };
        let parsed: SaveFile = match serde_json::from_str(&text) {
            Ok(p) => p,
            Err(e) => {
                self.status = Some(format!("parse failed: {e}"));
                return;
            }
        };
        let mut perimeters = parsed.perimeters;
        if let Some(single) = parsed.outer_polygon {
            perimeters.insert(0, single);
        }
        if perimeters.is_empty() {
            self.status = Some(format!(
                "load: no perimeter in {} (fixture probably has only `holes`)",
                path.display()
            ));
            return;
        }
        // Preserve fixture-browser state across the reset so the user
        // can keep clicking through fixtures.
        let fixtures_dir = self.fixtures_dir.clone();
        let fixture_listing = self.fixture_listing.clone();
        let fixtures_scanned_from = self.fixtures_scanned_from.clone();
        let mut new_app = DemoApp::default();
        new_app.perimeters = perimeters
            .into_iter()
            .enumerate()
            .map(|(i, verts)| Polygon {
                verts: verts.into_iter().map(Into::into).collect(),
                marker: (i as i32 + 1) * 10,
            })
            .collect();
        new_app.holes = parsed
            .holes
            .into_iter()
            .enumerate()
            .map(|(i, verts)| Polygon {
                verts: verts.into_iter().map(Into::into).collect(),
                marker: 1000 + i as i32 * 10,
            })
            .collect();
        new_app.next_marker = 2000;
        new_app.fixtures_dir = fixtures_dir;
        new_app.fixture_listing = fixture_listing;
        new_app.fixtures_scanned_from = fixtures_scanned_from;
        new_app.request_fit = true;
        new_app.status = Some(format!("loaded ← {}", path.display()));
        *self = new_app;
    }

    /// Refresh the fixture browser list from `self.fixtures_dir`. Expands
    /// a leading `~` to `$HOME`. Sorts results so the order is stable.
    fn refresh_fixture_listing(&mut self) {
        let dir = expand_tilde(&self.fixtures_dir);
        match std::fs::read_dir(&dir) {
            Ok(entries) => {
                let mut files: Vec<PathBuf> = entries
                    .filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| {
                        p.extension()
                            .and_then(|x| x.to_str())
                            .map_or(false, |x| x.eq_ignore_ascii_case("json"))
                    })
                    .collect();
                files.sort();
                self.fixture_listing = files;
                self.fixtures_scanned_from = Some(self.fixtures_dir.clone());
                self.status = Some(format!(
                    "scanned {}: {} .json file(s)",
                    dir.display(),
                    self.fixture_listing.len()
                ));
            }
            Err(e) => {
                self.fixture_listing.clear();
                self.fixtures_scanned_from = None;
                self.status = Some(format!("scan failed {}: {}", dir.display(), e));
            }
        }
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

        // Hole seed points — must be STRICTLY INSIDE each hole polygon.
        // The arithmetic centroid is NOT safe for concave polygons (e.g. a
        // C-shape's centroid falls outside the polygon), so use the
        // ear-based interior-point finder instead. If a hole turns out to
        // be degenerate we just skip it; carve_holes silently ignores
        // missing seeds.
        for hole in &self.holes {
            let cp = CommonPolygon::from_vertices(hole.verts.clone());
            if let Some(seed) = cp.interior_point() {
                pslg.holes.push(PslgHole { point: seed });
            }
        }

        // Build pipeline. form_skeleton auto-handles duplicate-position
        // vertices internally; no need to pre-dedupe here.
        let mut cdt = CdtMesh::new();
        for v in &pslg.vertices {
            cdt.push_vertex(VertexSlot::new(v.position, 0));
        }
        delaunay(&mut cdt, DivConqOptions::default());
        if let Err(e) = form_skeleton(&mut cdt, &pslg, None) {
            self.last_build_error = Some(format!("Segment insertion failed: {e}"));
            return;
        }
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
        self.request_fit = true;
    }

    /// AABB of everything currently authored or loaded — perimeters,
    /// holes, in-progress drawing, and the built navmesh. `None` if
    /// nothing has been placed yet.
    fn current_world_aabb(&self) -> Option<Aabb> {
        let mut any = false;
        let mut aabb = Aabb::EMPTY;
        for p in &self.perimeters {
            for v in &p.verts {
                aabb.extend(*v);
                any = true;
            }
        }
        for h in &self.holes {
            for v in &h.verts {
                aabb.extend(*v);
                any = true;
            }
        }
        if let Some(d) = &self.drawing {
            for v in &d.verts {
                aabb.extend(*v);
                any = true;
            }
        }
        if let Some(nav) = &self.navmesh {
            for v in &nav.vertices {
                aabb.extend(*v);
                any = true;
            }
        }
        any.then_some(aabb)
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
        ui.heading("rsnav demo");
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
            if ui
                .add(egui::Button::new("← Back to edit"))
                .on_hover_text("Tear down the navmesh and edit the polygons. Geometry is preserved.")
                .clicked()
            {
                self.back_to_authoring();
            }

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

            ui.add_space(8.0);
            ui.label("Visibility");
            ui.checkbox(&mut self.show_visibility, "show from cursor");
            if self.show_visibility {
                ui.add(
                    egui::Slider::new(&mut self.visibility_radius, 10.0..=2000.0)
                        .text("radius")
                        .logarithmic(true),
                );
                let mut samples_i = self.visibility_samples as i32;
                if ui
                    .add(egui::Slider::new(&mut samples_i, 32..=720).text("samples"))
                    .changed()
                {
                    self.visibility_samples = samples_i.max(8) as usize;
                }
            }
        }

        // Save / Load are available in both modes — capturing a confusing
        // build is exactly the case where you want to write to disk.
        ui.add_space(16.0);
        ui.separator();
        ui.label(format!("Debug file: {}", DEBUG_FILE));
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    !self.perimeters.is_empty() || !self.holes.is_empty(),
                    egui::Button::new("Save"),
                )
                .on_hover_text("Write all perimeters + holes to rsnav-debug.json in the current directory.")
                .clicked()
            {
                self.save_debug();
            }
            if ui
                .button("Load")
                .on_hover_text("Replace the current authoring state with rsnav-debug.json.")
                .clicked()
            {
                self.load_debug();
            }
        });

        ui.add_space(12.0);
        ui.separator();
        ui.collapsing("Fixtures", |ui| {
            ui.horizontal(|ui| {
                ui.label("dir:");
                ui.add(egui::TextEdit::singleline(&mut self.fixtures_dir).desired_width(160.0));
            });
            if ui.button("Scan").clicked() {
                self.refresh_fixture_listing();
            }
            // Auto-scan on first open of this collapser so the list
            // doesn't appear empty after launch.
            if self.fixtures_scanned_from.as_deref() != Some(self.fixtures_dir.as_str()) {
                self.refresh_fixture_listing();
            }
            if self.fixture_listing.is_empty() {
                ui.label("no .json files");
            } else {
                let listing = self.fixture_listing.clone();
                egui::ScrollArea::vertical()
                    .max_height(200.0)
                    .show(ui, |ui| {
                        for path in &listing {
                            let name = path
                                .file_name()
                                .map(|n| n.to_string_lossy().into_owned())
                                .unwrap_or_else(|| path.display().to_string());
                            if ui
                                .button(&name)
                                .on_hover_text(path.display().to_string())
                                .clicked()
                            {
                                self.load_from_path(path);
                            }
                        }
                    });
            }
        });

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            if ui
                .button("Fit view")
                .on_hover_text("Center & scale the canvas to fit all current geometry.")
                .clicked()
            {
                self.request_fit = true;
            }
            if ui.button("Reset everything").clicked() {
                self.reset();
            }
        });

        if let Some(s) = &self.status {
            ui.add_space(8.0);
            ui.colored_label(Color32::from_rgb(160, 200, 220), s);
        }

        ui.add_space(12.0);
        ui.label("Tip: left-click to drop vertices.");
        ui.label("Right-click while drawing closes the polygon.");
    }

    fn canvas_panel(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let (response, painter) = ui.allocate_painter(available, Sense::click_and_drag());
        let rect = response.rect;

        // Background
        painter.rect_filled(rect, 0.0, Color32::from_gray(28));

        // Recompute the world→canvas fit on demand. Triggered after Load
        // and after Create, plus by the "Fit view" button.
        if self.request_fit {
            self.request_fit = false;
            if let Some(world) = self.current_world_aabb() {
                self.view = ViewTransform::fit(world, rect.size(), 16.0);
            } else {
                self.view = ViewTransform::default();
            }
        }

        // Update hover position (cursor in world coords).
        self.hover_canvas = response
            .hover_pos()
            .map(|p| self.screen_to_world(rect, p));

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

    fn world_to_screen(&self, rect: Rect, v: Vertex) -> Pos2 {
        let local = self.view.world_to_canvas(v);
        Pos2::new(rect.min.x + local.x, rect.min.y + local.y)
    }

    fn screen_to_world(&self, rect: Rect, p: Pos2) -> Vertex {
        let local = Pos2::new(p.x - rect.min.x, p.y - rect.min.y);
        self.view.canvas_to_world(local)
    }

    // -- authoring -----------------------------------------------------

    fn handle_authoring_mouse(&mut self, response: &egui::Response, rect: egui::Rect) {
        // Only react to clicks inside the canvas.
        if !response.clicked() && !response.secondary_clicked() {
            return;
        }
        let Some(pos) = response.interact_pointer_pos() else { return };
        let world = self.screen_to_world(rect, pos);

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
                        self.world_to_screen(rect, *last),
                        self.world_to_screen(rect, hover),
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
        let pts: Vec<Pos2> = verts.iter().map(|v| self.world_to_screen(rect, *v)).collect();
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
                let world = self.screen_to_world(rect, pos);
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
                let world = self.screen_to_world(rect, pos);
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
                self.world_to_screen(rect, v0),
                self.world_to_screen(rect, v1),
                self.world_to_screen(rect, v2),
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
                    let pa = self.world_to_screen(rect, nav.vertex(a_id));
                    let pb = self.world_to_screen(rect, nav.vertex(b_id));
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

        // Visibility overlay — drawn FIRST so other overlays sit on top.
        //
        // The region is star-shaped from the cursor, so the natural
        // triangulation is a fan from the source through consecutive
        // boundary points. We build it as a single egui `Mesh` (one
        // vertex per source + boundary point, indices forming the fan)
        // rather than N separate `convex_polygon` calls — that way the
        // renderer sees each radial spoke as an *interior* edge between
        // two indexed triangles and skips its anti-aliasing pass, so
        // there's no visible spoke artifact from alpha-blended overlap.
        if self.show_visibility {
            if let Some(h) = self.hover_canvas {
                if let Some(vr) = visibility_region(
                    nav,
                    bsp,
                    h,
                    self.visibility_radius,
                    self.visibility_samples,
                ) {
                    let fill = Color32::from_rgba_unmultiplied(255, 230, 130, 36);
                    let stroke =
                        Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 220, 110, 160));

                    let mut mesh = egui::epaint::Mesh::default();
                    // Vertex 0 = source. Vertices 1..=N = boundary points.
                    mesh.colored_vertex(self.world_to_screen(rect, vr.source), fill);
                    for v in &vr.boundary {
                        mesh.colored_vertex(self.world_to_screen(rect, *v), fill);
                    }
                    let n = vr.boundary.len() as u32;
                    for i in 0..n {
                        let a = 1 + i;
                        let b = 1 + (i + 1) % n;
                        mesh.add_triangle(0, a, b);
                    }
                    painter.add(Shape::Mesh(mesh));

                    // Stroke the OUTER boundary as one closed polyline.
                    // The fan triangulation has no exterior at the spokes,
                    // so this only paints around the perimeter — no radial
                    // lines.
                    let pts: Vec<Pos2> = vr
                        .boundary
                        .iter()
                        .map(|v| self.world_to_screen(rect, *v))
                        .collect();
                    for i in 0..pts.len() {
                        painter.line_segment([pts[i], pts[(i + 1) % pts.len()]], stroke);
                    }
                }
            }
        }

        // Path source (if set) — green dot.
        if let Some(src) = self.path_src {
            painter.circle_filled(
                self.world_to_screen(rect, src),
                5.0,
                Color32::from_rgb(80, 220, 80),
            );
        }

        // Path polyline — bright green.
        if let Some(path) = &self.last_path {
            let pts: Vec<Pos2> = path
                .iter()
                .map(|v| self.world_to_screen(rect, *v))
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
                    self.world_to_screen(rect, np.point),
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
                            // Degenerate walk — amber, treat as uncertain.
                            LineOfSightResult::Indeterminate => {
                                (Color32::from_rgb(220, 180, 80), np.point)
                            }
                        };
                        painter.line_segment(
                            [
                                self.world_to_screen(rect, src),
                                self.world_to_screen(rect, end),
                            ],
                            Stroke::new(1.5, color),
                        );
                        if let LineOfSightResult::Blocked { point } = los {
                            painter.circle_filled(
                                self.world_to_screen(rect, point),
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

// --- Save / Load JSON schema --------------------------------------------

#[derive(Serialize, Deserialize)]
struct SaveFile {
    version: u32,
    #[serde(default)]
    perimeters: Vec<Vec<Point>>,
    /// gonav-fixture compatibility: a single perimeter under this key gets
    /// merged into `perimeters` on load. Never written on save.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    outer_polygon: Option<Vec<Point>>,
    #[serde(default)]
    holes: Vec<Vec<Point>>,
}

#[derive(Copy, Clone, Serialize, Deserialize)]
struct Point {
    x: f64,
    y: f64,
}

impl From<&Vertex> for Point {
    fn from(v: &Vertex) -> Self {
        Self { x: v.x, y: v.y }
    }
}
impl From<Point> for Vertex {
    fn from(p: Point) -> Self {
        Vertex::new(p.x, p.y)
    }
}

fn save_path() -> PathBuf {
    std::env::current_dir()
        .unwrap_or_else(|_| PathBuf::from("."))
        .join(DEBUG_FILE)
}

/// Expand a leading `~` to `$HOME`. Leaves everything else alone.
fn expand_tilde(s: &str) -> PathBuf {
    if let Some(rest) = s.strip_prefix("~") {
        if let Ok(home) = std::env::var("HOME") {
            let mut p = PathBuf::from(home);
            let rest = rest.strip_prefix('/').unwrap_or(rest);
            if !rest.is_empty() {
                p.push(rest);
            }
            return p;
        }
    }
    PathBuf::from(s)
}

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
