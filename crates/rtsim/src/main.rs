//! RTS-style dynamic-obstacle testbed.
//!
//! Owns a 128×128 cell bitfield as ground truth. Mouse tools modify the
//! grid (paint walls, clear, harvest forest cells); a background
//! `NavWorker` keeps the navmesh in sync, and a handful of agents path
//! between random walkable points while the mesh churns.
//!
//! Coordinate system: world coords are cell-aligned. Cell `(col, row)`
//! occupies `[col, col+1] × [row, row+1]`; row 0 is at the *bottom* of
//! the world (math-up). Egui draws with Y growing downward, so we flip
//! Y in [`RtsimApp::world_to_screen`].

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use eframe::egui;
use egui::{Color32, Pos2, Rect, Sense, Stroke};

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_dynamic::{BuildOptions, NavBuild, NavEvent, NavListener, NavWorker};
use rsnav_navigation::{find_path, PathOptions};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

const GRID_W: u32 = 128;
const GRID_H: u32 = 128;
const N_AGENTS: usize = 10;
const N_FOREST_BLOBS: usize = 6;
const FOREST_RADIUS: i32 = 9;
const AGENT_SPEED: f64 = 12.0; // world units / sec
const WAYPOINT_EPS: f64 = 0.25;
/// How many recent telemetry events to display in the HUD.
const EVENT_LOG_CAP: usize = 12;

fn main() -> eframe::Result<()> {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 800.0])
            .with_title("rsnav-rtsim — dynamic-obstacles testbed"),
        ..Default::default()
    };
    eframe::run_native(
        "rsnav-rtsim",
        opts,
        Box::new(|_cc| Ok(Box::new(RtsimApp::new()))),
    )
}

// =========================================================================
// Telemetry: a NavListener that pushes owned events into a ring buffer
// so the side panel can show recent worker activity.
// =========================================================================

#[derive(Clone, Debug)]
enum OwnedNavEvent {
    BuildStarted {
        generation: u64,
    },
    BuildCompleted {
        generation: u64,
        build_ms: f64,
        triangles: usize,
        regions: u32,
    },
    BuildFailed {
        generation: u64,
        error: String,
    },
}

struct EventLog {
    /// (time the listener received the event, owned event).
    events: Mutex<VecDeque<(Instant, OwnedNavEvent)>>,
}

impl EventLog {
    fn new() -> Self {
        Self {
            events: Mutex::new(VecDeque::with_capacity(EVENT_LOG_CAP)),
        }
    }
    fn snapshot(&self) -> Vec<(Instant, OwnedNavEvent)> {
        self.events
            .lock()
            .expect("event log")
            .iter()
            .cloned()
            .collect()
    }
}

impl NavListener for EventLog {
    fn on_event(&self, ev: &NavEvent<'_>) {
        let owned = match ev {
            NavEvent::BuildStarted { generation } => OwnedNavEvent::BuildStarted {
                generation: *generation,
            },
            NavEvent::BuildCompleted {
                generation,
                build_ms,
                triangles,
                regions,
            } => OwnedNavEvent::BuildCompleted {
                generation: *generation,
                build_ms: *build_ms,
                triangles: *triangles,
                regions: *regions,
            },
            NavEvent::BuildFailed { generation, error } => OwnedNavEvent::BuildFailed {
                generation: *generation,
                error: format!("{error}"),
            },
        };
        let mut guard = self.events.lock().expect("event log");
        if guard.len() >= EVENT_LOG_CAP {
            guard.pop_front();
        }
        guard.push_back((Instant::now(), owned));
    }
}

// =========================================================================
// Cell grid
// =========================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Cell {
    Walkable,
    Wall,
    Forest,
}

impl Cell {
    fn walkable(self) -> bool {
        matches!(self, Cell::Walkable)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Tool {
    PaintWall,
    Clear,
    Harvest,
}

// =========================================================================
// Agents
// =========================================================================

#[derive(Clone, Debug)]
struct Agent {
    pos: Vertex,
    goal: Vertex,
    path: Vec<Vertex>,
    /// Index of the next waypoint in `path` to walk toward.
    waypoint: usize,
    /// Generation of the build the path was computed against. -1 = no
    /// valid path.
    path_gen: i64,
    /// Number of consecutive replan failures. After several, we pick a
    /// brand new random goal.
    failed_replans: u32,
}

impl Agent {
    fn new(pos: Vertex, goal: Vertex) -> Self {
        Self {
            pos,
            goal,
            path: vec![pos],
            waypoint: 1,
            path_gen: -1,
            failed_replans: 0,
        }
    }

    fn at_goal(&self) -> bool {
        self.pos.distance(self.goal) <= WAYPOINT_EPS
    }
}

// =========================================================================
// Tiny LCG so we don't need the `rand` crate.
// =========================================================================

struct Rng {
    state: u64,
}
impl Rng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed | 1, // avoid 0
        }
    }
    fn next_u32(&mut self) -> u32 {
        // splitmix64
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        ((z ^ (z >> 31)) >> 32) as u32
    }
    fn next_in(&mut self, bound: u32) -> u32 {
        if bound == 0 {
            0
        } else {
            self.next_u32() % bound
        }
    }
    fn unit_f32(&mut self) -> f32 {
        (self.next_u32() as f32) / (u32::MAX as f32)
    }
}

// =========================================================================
// App
// =========================================================================

struct RtsimApp {
    cells: Vec<Cell>,
    cell_w: u32,
    cell_h: u32,

    worker: NavWorker,
    /// Shared with the worker via `spawn_with_listener`; the side panel
    /// drains a copy each frame for the HUD.
    event_log: Arc<EventLog>,
    /// Wall-clock at app start. Event log times shown relative to it.
    app_started: Instant,
    /// Set when the user has just edited the grid; cleared after we
    /// submit a snapshot to the worker.
    dirty: bool,
    /// Throttle: don't submit more often than this.
    last_submit: Instant,
    submit_min_interval: std::time::Duration,

    /// The build we're using for queries this frame.
    current_build: Option<Arc<NavBuild>>,

    agents: Vec<Agent>,
    rng: Rng,
    tool: Tool,
    paint_radius: i32,

    /// Real-time clock for agent movement.
    last_tick: Instant,

    show_navmesh: bool,
    show_regions: bool,
    show_agents: bool,
    show_paths: bool,
    show_bitfield: bool,
}

impl RtsimApp {
    fn new() -> Self {
        let cells = vec![Cell::Walkable; (GRID_W * GRID_H) as usize];
        let event_log = Arc::new(EventLog::new());
        let worker = NavWorker::spawn_with_listener(
            BuildOptions::default(),
            event_log.clone() as Arc<dyn NavListener>,
        );
        let mut app = Self {
            cells,
            cell_w: GRID_W,
            cell_h: GRID_H,
            worker,
            event_log,
            app_started: Instant::now(),
            dirty: false,
            last_submit: Instant::now() - std::time::Duration::from_secs(1),
            submit_min_interval: std::time::Duration::from_millis(60),
            current_build: None,
            agents: Vec::new(),
            rng: Rng::new(0xC0FFEE),
            tool: Tool::PaintWall,
            paint_radius: 1,
            last_tick: Instant::now(),
            show_navmesh: true,
            show_regions: true,
            show_agents: true,
            show_paths: true,
            show_bitfield: true,
        };

        app.seed_forests();
        // Kick off the first build immediately.
        app.submit_snapshot();
        // Wait briefly so the first frame has something to draw.
        let deadline = Instant::now() + std::time::Duration::from_millis(500);
        loop {
            if app.worker.poll_swap() {
                break;
            }
            if Instant::now() > deadline {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        app.current_build = app.worker.current();
        app.spawn_agents();
        app
    }

    fn seed_forests(&mut self) {
        for _ in 0..N_FOREST_BLOBS {
            let cx = self.rng.next_in(self.cell_w) as i32;
            let cy = self.rng.next_in(self.cell_h) as i32;
            let r = FOREST_RADIUS;
            for dy in -r..=r {
                for dx in -r..=r {
                    let x = cx + dx;
                    let y = cy + dy;
                    if x < 0 || y < 0 || (x as u32) >= self.cell_w || (y as u32) >= self.cell_h {
                        continue;
                    }
                    let d2 = dx * dx + dy * dy;
                    // Soft edge — randomly drop cells near the rim to make a blob.
                    if d2 > r * r {
                        continue;
                    }
                    let p_keep = 1.0 - (d2 as f32) / ((r * r) as f32);
                    if self.rng.unit_f32() < p_keep * 0.95 + 0.05 {
                        self.set_cell(x as u32, y as u32, Cell::Forest);
                    }
                }
            }
        }
        self.dirty = true;
    }

    fn spawn_agents(&mut self) {
        let Some(build) = self.current_build.clone() else {
            return;
        };
        for _ in 0..N_AGENTS {
            let pos = self
                .random_walkable_point(&build.navmesh, &build.bsp)
                .unwrap_or(Vertex::new(GRID_W as f64 / 2.0, GRID_H as f64 / 2.0));
            let goal = self
                .random_walkable_point(&build.navmesh, &build.bsp)
                .unwrap_or(pos);
            self.agents.push(Agent::new(pos, goal));
        }
    }

    fn random_walkable_point(&mut self, nav: &NavMesh, bsp: &Bsp) -> Option<Vertex> {
        for _ in 0..256 {
            let x = (self.rng.next_in(self.cell_w) as f64) + self.rng.unit_f32() as f64;
            let y = (self.rng.next_in(self.cell_h) as f64) + self.rng.unit_f32() as f64;
            let v = Vertex::new(x, y);
            if bsp.locate(nav, v).is_some() {
                return Some(v);
            }
        }
        None
    }

    fn idx(&self, col: u32, row: u32) -> usize {
        (row * self.cell_w + col) as usize
    }

    fn get_cell(&self, col: u32, row: u32) -> Cell {
        self.cells[self.idx(col, row)]
    }

    fn set_cell(&mut self, col: u32, row: u32, kind: Cell) {
        let i = self.idx(col, row);
        if self.cells[i] != kind {
            self.cells[i] = kind;
            self.dirty = true;
        }
    }

    fn apply_tool_at_cell(&mut self, col: u32, row: u32) {
        match self.tool {
            Tool::PaintWall => self.paint_disk(col, row, Cell::Wall),
            Tool::Clear => self.paint_disk(col, row, Cell::Walkable),
            Tool::Harvest => {
                // Single-cell, only if the cell is currently a forest.
                if matches!(self.get_cell(col, row), Cell::Forest) {
                    self.set_cell(col, row, Cell::Walkable);
                }
            }
        }
    }

    fn paint_disk(&mut self, cx: u32, cy: u32, kind: Cell) {
        let r = self.paint_radius;
        for dy in -r..=r {
            for dx in -r..=r {
                let x = cx as i32 + dx;
                let y = cy as i32 + dy;
                if x < 0 || y < 0 || (x as u32) >= self.cell_w || (y as u32) >= self.cell_h {
                    continue;
                }
                if dx * dx + dy * dy <= r * r {
                    self.set_cell(x as u32, y as u32, kind);
                }
            }
        }
    }

    fn submit_snapshot(&mut self) {
        let data: Vec<bool> = self.cells.iter().map(|c| c.walkable()).collect();
        let bf = Bitfield::new(self.cell_w, self.cell_h, data)
            .expect("cells.len() == cell_w * cell_h");
        self.worker.submit_snapshot(Arc::new(bf));
        self.last_submit = Instant::now();
        self.dirty = false;
    }

    fn replan_agent(&mut self, idx: usize, nav: &NavMesh, bsp: &Bsp) {
        let pos = self.agents[idx].pos;
        let goal = self.agents[idx].goal;

        // If goal is no longer walkable (a wall just dropped on top of
        // it), pick a new one.
        let goal = if bsp.locate(nav, goal).is_some() {
            goal
        } else {
            match self.random_walkable_point(nav, bsp) {
                Some(g) => g,
                None => return, // mesh has nowhere to go right now
            }
        };
        self.agents[idx].goal = goal;

        // If pos isn't on the mesh either (a wall enclosed the agent),
        // bail and let the next swap try again.
        if bsp.locate(nav, pos).is_none() {
            self.agents[idx].path = Vec::new();
            self.agents[idx].waypoint = 0;
            self.agents[idx].failed_replans = self.agents[idx].failed_replans.saturating_add(1);
            return;
        }

        match find_path(nav, bsp, pos, goal, &PathOptions::default()) {
            Ok(result) => {
                self.agents[idx].path = result.points;
                self.agents[idx].waypoint = 1.min(self.agents[idx].path.len().saturating_sub(1));
                if self.agents[idx].path.len() < 2 {
                    self.agents[idx].waypoint = 0;
                }
                self.agents[idx].failed_replans = 0;
            }
            Err(_) => {
                self.agents[idx].path = Vec::new();
                self.agents[idx].failed_replans = self.agents[idx].failed_replans.saturating_add(1);
                // After a few failures, try a fresh goal next tick.
                if self.agents[idx].failed_replans >= 3 {
                    if let Some(g) = self.random_walkable_point(nav, bsp) {
                        self.agents[idx].goal = g;
                    }
                    self.agents[idx].failed_replans = 0;
                }
            }
        }
    }

    fn tick_agents(&mut self, dt: f64) {
        let Some(build) = self.current_build.clone() else {
            return;
        };
        let nav = &build.navmesh;
        let bsp = &build.bsp;

        // Did the navmesh just swap? If so, every agent re-plans.
        let mesh_gen = build.generation as i64;
        for i in 0..self.agents.len() {
            if self.agents[i].path_gen != mesh_gen {
                self.replan_agent(i, nav, bsp);
                self.agents[i].path_gen = mesh_gen;
            }
        }

        for i in 0..self.agents.len() {
            if self.agents[i].path.is_empty() {
                continue;
            }
            if self.agents[i].at_goal() {
                // Pick a new goal.
                if let Some(g) = self.random_walkable_point(nav, bsp) {
                    self.agents[i].goal = g;
                    self.replan_agent(i, nav, bsp);
                }
                continue;
            }

            let mut remaining = AGENT_SPEED * dt;
            while remaining > 0.0 {
                let wp_idx = self.agents[i].waypoint;
                if wp_idx >= self.agents[i].path.len() {
                    break;
                }
                let target = self.agents[i].path[wp_idx];
                let to_target = target - self.agents[i].pos;
                let dist = to_target.length();
                if dist <= remaining {
                    self.agents[i].pos = target;
                    remaining -= dist;
                    self.agents[i].waypoint += 1;
                    if self.agents[i].waypoint >= self.agents[i].path.len() {
                        break;
                    }
                } else {
                    let dir = to_target * (1.0 / dist);
                    self.agents[i].pos = self.agents[i].pos + dir * remaining;
                    remaining = 0.0;
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Drawing
    // -----------------------------------------------------------------

    fn cell_size_px(&self, rect: Rect) -> f32 {
        let sx = rect.width() / self.cell_w as f32;
        let sy = rect.height() / self.cell_h as f32;
        sx.min(sy)
    }

    fn world_to_screen(&self, rect: Rect, v: Vertex) -> Pos2 {
        let s = self.cell_size_px(rect);
        let total_w = s * self.cell_w as f32;
        let total_h = s * self.cell_h as f32;
        let ox = rect.min.x + (rect.width() - total_w) * 0.5;
        let oy = rect.min.y + (rect.height() - total_h) * 0.5;
        // Flip Y (world Y up → screen Y down).
        Pos2::new(ox + v.x as f32 * s, oy + total_h - v.y as f32 * s)
    }

    fn screen_to_cell(&self, rect: Rect, p: Pos2) -> Option<(u32, u32)> {
        let s = self.cell_size_px(rect);
        let total_w = s * self.cell_w as f32;
        let total_h = s * self.cell_h as f32;
        let ox = rect.min.x + (rect.width() - total_w) * 0.5;
        let oy = rect.min.y + (rect.height() - total_h) * 0.5;
        let cx = (p.x - ox) / s;
        let cy_screen = (p.y - oy) / s;
        // Flip back to math-up coords.
        let cy = self.cell_h as f32 - cy_screen;
        if cx < 0.0 || cy < 0.0 {
            return None;
        }
        let col = cx as u32;
        let row = cy as u32;
        if col >= self.cell_w || row >= self.cell_h {
            return None;
        }
        Some((col, row))
    }

    fn region_color(&self, region: u32) -> Color32 {
        // 8 pastel hues cycled by region id.
        const PALETTE: [Color32; 8] = [
            Color32::from_rgb(180, 210, 255),
            Color32::from_rgb(190, 255, 200),
            Color32::from_rgb(255, 220, 190),
            Color32::from_rgb(245, 200, 245),
            Color32::from_rgb(255, 245, 180),
            Color32::from_rgb(200, 240, 240),
            Color32::from_rgb(220, 220, 255),
            Color32::from_rgb(245, 215, 215),
        ];
        PALETTE[(region as usize) % PALETTE.len()]
    }

    fn canvas_panel(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size_before_wrap();
        let (response, painter) = ui.allocate_painter(available, Sense::click_and_drag());
        let rect = response.rect;

        painter.rect_filled(rect, 0.0, Color32::from_gray(28));

        // Input → grid edits.
        if response.dragged() || response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                if let Some((col, row)) = self.screen_to_cell(rect, pos) {
                    self.apply_tool_at_cell(col, row);
                }
            }
        }

        let s = self.cell_size_px(rect);

        // Bitfield underlay: walls + forests as filled cells.
        if self.show_bitfield {
            for row in 0..self.cell_h {
                for col in 0..self.cell_w {
                    let c = self.get_cell(col, row);
                    let color = match c {
                        Cell::Walkable => continue,
                        Cell::Wall => Color32::from_rgb(70, 70, 80),
                        Cell::Forest => Color32::from_rgb(50, 110, 60),
                    };
                    let v0 = self.world_to_screen(rect, Vertex::new(col as f64, row as f64));
                    let v1 =
                        self.world_to_screen(rect, Vertex::new(col as f64 + 1.0, row as f64 + 1.0));
                    let r = Rect::from_two_pos(v0, v1);
                    painter.rect_filled(r, 0.0, color);
                }
            }
        }

        // Navmesh tris (over the bitfield, under the agents).
        if let Some(build) = self.current_build.clone() {
            let nav = &build.navmesh;
            if self.show_navmesh {
                for tri in &nav.triangles {
                    let pa = self.world_to_screen(rect, nav.vertex(tri.vertices[0]));
                    let pb = self.world_to_screen(rect, nav.vertex(tri.vertices[1]));
                    let pc = self.world_to_screen(rect, nav.vertex(tri.vertices[2]));
                    let fill = if self.show_regions {
                        self.region_color(tri.region).gamma_multiply(0.55)
                    } else {
                        Color32::from_rgba_unmultiplied(200, 220, 200, 100)
                    };
                    painter.add(egui::Shape::convex_polygon(
                        vec![pa, pb, pc],
                        fill,
                        Stroke::NONE,
                    ));
                }
                // Constraint edges in a darker tone over the fills.
                for tri in &nav.triangles {
                    for edge in 0..3 {
                        if tri.edge_markers[edge] == 0 {
                            continue;
                        }
                        let (a, b) = tri.edge_vertices(edge);
                        let pa = self.world_to_screen(rect, nav.vertex(a));
                        let pb = self.world_to_screen(rect, nav.vertex(b));
                        painter.line_segment(
                            [pa, pb],
                            Stroke::new(1.2, Color32::from_rgb(20, 20, 30)),
                        );
                    }
                }
            }

            // Agent paths.
            if self.show_paths {
                for agent in &self.agents {
                    if agent.path.len() < 2 {
                        continue;
                    }
                    let pts: Vec<Pos2> = agent
                        .path
                        .iter()
                        .map(|v| self.world_to_screen(rect, *v))
                        .collect();
                    painter.add(egui::Shape::line(
                        pts,
                        Stroke::new(1.5, Color32::from_rgba_unmultiplied(255, 80, 80, 180)),
                    ));
                }
            }

            // Agents themselves.
            if self.show_agents {
                let r_px = (s * 0.45).max(2.0);
                for agent in &self.agents {
                    let p = self.world_to_screen(rect, agent.pos);
                    let body_color = if agent.path.len() >= 2 {
                        Color32::from_rgb(220, 60, 60)
                    } else {
                        Color32::from_rgb(120, 120, 120)
                    };
                    painter.circle_filled(p, r_px, body_color);
                    painter.circle_stroke(p, r_px, Stroke::new(1.0, Color32::BLACK));
                    let g = self.world_to_screen(rect, agent.goal);
                    painter.circle_stroke(
                        g,
                        r_px * 0.7,
                        Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 200, 60, 200)),
                    );
                }
            }
        }

        // Outline of the world.
        let p0 = self.world_to_screen(rect, Vertex::new(0.0, 0.0));
        let p1 =
            self.world_to_screen(rect, Vertex::new(self.cell_w as f64, self.cell_h as f64));
        painter.rect_stroke(
            Rect::from_two_pos(p0, p1),
            0.0,
            Stroke::new(1.0, Color32::from_gray(120)),
        );
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("rsnav-rtsim");
        ui.add_space(4.0);

        ui.label("Tool (mouse drag on the canvas)");
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.tool, Tool::PaintWall, "Wall");
            ui.selectable_value(&mut self.tool, Tool::Clear, "Clear");
            ui.selectable_value(&mut self.tool, Tool::Harvest, "Harvest");
        });
        ui.add(egui::Slider::new(&mut self.paint_radius, 0..=6).text("brush radius"));
        ui.label(match self.tool {
            Tool::PaintWall => "Drag: paint walls (a disk per cell).",
            Tool::Clear => "Drag: clear cells (walls AND forest → walkable).",
            Tool::Harvest => "Click/drag: remove ONE forest cell at a time (radius ignored).",
        });

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Display");
        ui.checkbox(&mut self.show_bitfield, "bitfield underlay");
        ui.checkbox(&mut self.show_navmesh, "navmesh triangles");
        ui.checkbox(&mut self.show_regions, "region coloring");
        ui.checkbox(&mut self.show_agents, "agents");
        ui.checkbox(&mut self.show_paths, "agent paths");

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Current build");
        match &self.current_build {
            Some(b) => {
                ui.label(format!("gen   : {}", b.generation));
                ui.label(format!("tris  : {}", b.navmesh.triangle_count()));
                ui.label(format!("regs  : {}", b.navmesh.region_count));
                ui.label(format!("build : {:.2} ms", b.build_ms));
            }
            None => {
                ui.label("(no build yet)");
            }
        }
        if let Some(e) = self.worker.last_error() {
            ui.colored_label(Color32::from_rgb(255, 120, 120), format!("last error: {e}"));
        }

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Worker stats");
        let s = self.worker.stats();
        let in_flight = s
            .snapshots_submitted
            .saturating_sub(s.builds_completed + s.builds_failed + s.snapshots_coalesced);
        let avg_ms = if s.builds_completed > 0 {
            s.total_build_ms / s.builds_completed as f64
        } else {
            0.0
        };
        ui.label(format!("submitted    : {}", s.snapshots_submitted));
        ui.label(format!("coalesced    : {}", s.snapshots_coalesced));
        ui.label(format!("in flight    : {in_flight}"));
        ui.label(format!("completed    : {}", s.builds_completed));
        if s.builds_failed > 0 {
            ui.colored_label(
                Color32::from_rgb(255, 120, 120),
                format!("failed       : {}", s.builds_failed),
            );
        } else {
            ui.label(format!("failed       : {}", s.builds_failed));
        }
        ui.label(format!(
            "build ms     : {:.2} avg / {:.2} max / {:.2} last",
            avg_ms, s.max_build_ms, s.last_build_ms
        ));

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Recent events");
        let events = self.event_log.snapshot();
        if events.is_empty() {
            ui.label("(no events yet)");
        } else {
            for (when, ev) in events.iter().rev() {
                let t = when.saturating_duration_since(self.app_started).as_secs_f64();
                let (color, text) = match ev {
                    OwnedNavEvent::BuildStarted { generation } => (
                        Color32::from_gray(180),
                        format!("[{t:>6.2}s] start gen {generation}"),
                    ),
                    OwnedNavEvent::BuildCompleted {
                        generation,
                        build_ms,
                        triangles,
                        regions,
                    } => (
                        Color32::from_rgb(150, 230, 160),
                        format!(
                            "[{t:>6.2}s] done  gen {generation}: {build_ms:.2}ms  {triangles}t  {regions}r"
                        ),
                    ),
                    OwnedNavEvent::BuildFailed { generation, error } => (
                        Color32::from_rgb(255, 130, 130),
                        format!("[{t:>6.2}s] FAIL  gen {generation}: {error}"),
                    ),
                };
                ui.colored_label(
                    color,
                    egui::RichText::new(text).monospace().size(11.0),
                );
            }
        }

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Reset");
        if ui.button("clear all (walkable everywhere)").clicked() {
            self.cells.fill(Cell::Walkable);
            self.dirty = true;
        }
        if ui.button("respawn forests").clicked() {
            for c in self.cells.iter_mut() {
                if matches!(c, Cell::Forest) {
                    *c = Cell::Walkable;
                }
            }
            self.seed_forests();
        }
        if ui.button("re-roll agents").clicked() {
            self.agents.clear();
            self.spawn_agents();
        }

        ui.add_space(8.0);
        ui.label(egui::RichText::new(
            "Coordinates: 128×128 cells, 1.0 world unit per cell, math-up Y."
        ).small().color(Color32::from_gray(150)));
    }
}

// =========================================================================
// eframe::App glue
// =========================================================================

impl eframe::App for RtsimApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // -- Frame start: swap if the worker has a newer build.
        let swapped = self.worker.poll_swap();
        if swapped {
            self.current_build = self.worker.current();
        }

        // -- Submit if dirty and we're not still inside the throttle window.
        if self.dirty && self.last_submit.elapsed() >= self.submit_min_interval {
            self.submit_snapshot();
        }

        // -- Tick agents at real-time, clamped to 100ms so a stalled frame
        // can't teleport them.
        let now = Instant::now();
        let mut dt = (now - self.last_tick).as_secs_f64();
        self.last_tick = now;
        if dt > 0.1 {
            dt = 0.1;
        }
        self.tick_agents(dt);

        egui::SidePanel::left("tools")
            .resizable(false)
            .min_width(260.0)
            .show(ctx, |ui| {
                self.side_panel(ui);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.canvas_panel(ui);
        });

        // Continuous repaint: agents are moving and the worker may be
        // publishing builds in the background.
        ctx.request_repaint();
    }
}

