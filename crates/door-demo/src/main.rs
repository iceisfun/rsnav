//! `rsnav-door-demo` — a focused testbed for **doors**.
//!
//! A small grid world is split into four rooms by a cross of walls.
//! Each wall has two [`Door`]s. A door is a pure *Option A* obstacle:
//! when **open** its cells are walkable; when **closed** they are
//! carved out of the bitfield, so the background [`NavWorker`] rebuilds
//! the navmesh without the gap.
//!
//! A handful of actors patrol back and forth between two fixed points
//! (`home` ⇄ `away`) on top of a shared [`Crowd`]. Click a door on the
//! map — or use the side-panel checkboxes — to open or close it: the
//! bitfield is resubmitted, the navmesh rebuilds, and
//! [`Crowd::set_nav`] revalidates every actor's *remaining* path by
//! line-of-sight. Actors whose corridor the door just blocked drop
//! their stale route and replan; unaffected actors keep walking.
//!
//! That last point is the headline: a path is planned against one
//! navmesh generation, but the world keeps changing under it. When the
//! path generation no longer matches the live navmesh, the still-valid
//! tail of the route is kept and the blocked remainder is replanned —
//! no global replan storm, no actor frozen on a stale corridor.
//!
//! Conventions: 1 world unit = 1 cell; math-up Y (row 0 at the bottom),
//! flipped for egui in [`world_to_screen`].

use std::sync::Arc;
use std::time::{Duration, Instant};

use eframe::egui;
use egui::{Color32, Pos2, Rect, Sense, Stroke};

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_crowd::{Agent, AgentId, Crowd, CrowdConfig, Goal};
use rsnav_dynamic::{BuildOptions, NavBuild, NavWorker};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

// =========================================================================
// World constants
// =========================================================================

const GRID_W: u32 = 76;
const GRID_H: u32 = 48;

/// Column of the full-height wall that splits the map left/right.
const VWALL_COL: u32 = 37;
/// Row of the full-width wall that splits the map top/bottom.
const HWALL_ROW: u32 = 23;

const N_ACTORS_INITIAL: usize = 8;
const ACTOR_RADIUS_MIN: f64 = 0.34;
const ACTOR_RADIUS_MAX: f64 = 0.50;
const ACTOR_SPEED: f64 = 7.0;
const ARRIVE_RADIUS: f64 = 0.6;
/// A patrol's two endpoints must be at least this far apart, so every
/// actor actually crosses a wall (and a door) on each leg.
const MIN_PATROL_SPAN: f64 = 28.0;

/// Click within this many world units of a door to toggle it. Doors
/// are one cell thick, so a little slack makes them easy to hit.
const DOOR_CLICK_SLACK: f64 = 1.5;

// =========================================================================
// fn main
// =========================================================================

fn main() -> eframe::Result<()> {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 760.0])
            .with_title("rsnav-door-demo — doors + patrols"),
        ..Default::default()
    };
    eframe::run_native(
        "rsnav-door-demo",
        opts,
        Box::new(|_cc| Ok(Box::new(DoorDemoApp::new()))),
    )
}

// =========================================================================
// Cell grid
// =========================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Cell {
    Walkable,
    /// A doorway cell. Its *base* state is walkable (the door is open);
    /// [`build_bitfield`] carves it shut for any door currently closed.
    DoorWay,
    /// A permanent wall.
    Wall,
}

impl Cell {
    /// Whether the navmesh considers this cell walkable in its base
    /// (door-open) state.
    fn walkable(self) -> bool {
        matches!(self, Cell::Walkable | Cell::DoorWay)
    }
}

fn idx(col: u32, row: u32) -> usize {
    (row * GRID_W + col) as usize
}

// =========================================================================
// Doors
// =========================================================================

/// A togglable gap in a wall.
///
/// A door is an inclusive cell rectangle plus an open/closed flag.
/// Toggling it is just the `bool` flip plus a bitfield resubmit — no
/// mesh-specific code (this is the "Option A" approach: a door is an
/// ordinary obstacle that comes and goes).
///
/// [`Door::rect`] is the general constructor; [`Door::horizontal`] and
/// [`Door::vertical`] are the common one-cell-thick spans for a gap in
/// a straight wall.
#[derive(Clone, Debug)]
struct Door {
    /// Inclusive cell rect `(col0, row0, col1, row1)`.
    rect: (u32, u32, u32, u32),
    /// Open doors are walkable; closed doors block.
    open: bool,
}

impl Door {
    /// A door covering an inclusive cell rectangle. Corners are
    /// normalized, so argument order does not matter.
    fn rect(c0: u32, r0: u32, c1: u32, r1: u32, open: bool) -> Self {
        Self {
            rect: (c0.min(c1), r0.min(r1), c0.max(c1), r0.max(r1)),
            open,
        }
    }

    /// A one-cell-tall door on `row`, spanning columns `c0..=c1` — a
    /// gap in a horizontal wall.
    fn horizontal(row: u32, c0: u32, c1: u32, open: bool) -> Self {
        Self::rect(c0, row, c1, row, open)
    }

    /// A one-cell-wide door on `col`, spanning rows `r0..=r1` — a gap
    /// in a vertical wall.
    fn vertical(col: u32, r0: u32, r1: u32, open: bool) -> Self {
        Self::rect(col, r0, col, r1, open)
    }

    /// Iterate the `(col, row)` cells the door occupies.
    fn cells(&self) -> impl Iterator<Item = (u32, u32)> {
        let (c0, r0, c1, r1) = self.rect;
        (r0..=r1).flat_map(move |r| (c0..=c1).map(move |c| (c, r)))
    }

    /// Distance from a world point to the door's footprint. Zero when
    /// the point is inside the door's cell rectangle.
    fn distance_to(&self, p: Vertex) -> f64 {
        let (c0, r0, c1, r1) = self.rect;
        let dx = (c0 as f64 - p.x).max(0.0).max(p.x - (c1 + 1) as f64);
        let dy = (r0 as f64 - p.y).max(0.0).max(p.y - (r1 + 1) as f64);
        (dx * dx + dy * dy).sqrt()
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
        Self { state: seed | 1 }
    }
    fn next_u32(&mut self) -> u32 {
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
    fn unit_f64(&mut self) -> f64 {
        (self.next_u32() as f64) / (u32::MAX as f64)
    }
}

// =========================================================================
// Actors
// =========================================================================

/// A patrolling actor: it walks `home → away → home → away → …`
/// forever. The two endpoints are fixed at spawn; only the current
/// destination flips.
struct Actor {
    id: AgentId,
    home: Vertex,
    away: Vertex,
    /// `true` while the current goal is `away`.
    heading_away: bool,
}

// =========================================================================
// App
// =========================================================================

struct DoorDemoApp {
    cells: Vec<Cell>,
    doors: Vec<Door>,

    worker: NavWorker,
    current_build: Option<Arc<NavBuild>>,

    crowd: Crowd,
    actors: Vec<Actor>,

    rng: Rng,
    dirty: bool,
    last_submit: Instant,
    submit_min_interval: Duration,
    last_tick: Instant,

    show_navmesh: bool,
    show_regions: bool,
    show_walls: bool,
    show_paths: bool,
    show_agents: bool,
    show_velocity: bool,
}

impl DoorDemoApp {
    fn new() -> Self {
        // 1) Paint the walled world. Walls go down first; the door
        //    cells are then carved back out as `DoorWay`.
        let mut cells = vec![Cell::Walkable; (GRID_W * GRID_H) as usize];
        paint_walls(&mut cells);
        let doors = vec![
            Door::vertical(VWALL_COL, 5, 9, true),
            Door::vertical(VWALL_COL, 36, 40, true),
            Door::horizontal(HWALL_ROW, 11, 15, true),
            Door::horizontal(HWALL_ROW, 56, 60, true),
        ];
        for door in &doors {
            for (c, r) in door.cells() {
                cells[idx(c, r)] = Cell::DoorWay;
            }
        }

        // 2) Spin up the worker and submit the first bitfield.
        let mut worker = NavWorker::spawn(BuildOptions::default());
        worker.submit_snapshot(Arc::new(build_bitfield(&cells, &doors)));
        let deadline = Instant::now() + Duration::from_millis(1500);
        loop {
            if worker.poll_swap() {
                break;
            }
            if Instant::now() > deadline {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        let current_build = worker
            .current()
            .expect("first navmesh build must succeed");

        // 3) Build the crowd.
        let crowd = Crowd::new(current_build.clone(), CrowdConfig::default());

        let mut app = Self {
            cells,
            doors,
            worker,
            current_build: Some(current_build),
            crowd,
            actors: Vec::new(),
            rng: Rng::new(0xD00D_F00D_CAFE),
            dirty: false,
            last_submit: Instant::now() - Duration::from_secs(1),
            submit_min_interval: Duration::from_millis(80),
            last_tick: Instant::now(),
            show_navmesh: true,
            show_regions: false,
            show_walls: true,
            show_paths: true,
            show_agents: true,
            show_velocity: true,
        };
        for _ in 0..N_ACTORS_INITIAL {
            app.spawn_actor();
        }
        app
    }

    // -- Spawn / remove ---------------------------------------------------

    fn spawn_actor(&mut self) {
        let Some(build) = self.current_build.clone() else {
            return;
        };
        let nav = &build.navmesh;
        let bsp = &build.bsp;
        let Some(home) = self.random_room_point(nav, bsp) else {
            return;
        };
        // Pick an `away` endpoint a good distance off, so the actor
        // crosses a wall every leg.
        let mut away = home;
        for _ in 0..64 {
            let Some(p) = self.random_room_point(nav, bsp) else {
                continue;
            };
            if (p - home).length() >= MIN_PATROL_SPAN {
                away = p;
                break;
            }
        }
        let radius =
            ACTOR_RADIUS_MIN + (ACTOR_RADIUS_MAX - ACTOR_RADIUS_MIN) * self.rng.unit_f64();
        let id = self.crowd.add_agent(Agent::new(home, radius, ACTOR_SPEED));
        self.crowd.set_goal(
            id,
            Some(Goal {
                target: away,
                arrive_radius: ARRIVE_RADIUS,
            }),
        );
        self.actors.push(Actor {
            id,
            home,
            away,
            heading_away: true,
        });
    }

    fn remove_last_actor(&mut self) {
        if let Some(actor) = self.actors.pop() {
            self.crowd.remove_agent(actor.id);
        }
    }

    fn clear_actors(&mut self) {
        for actor in self.actors.drain(..) {
            self.crowd.remove_agent(actor.id);
        }
    }

    /// A random point that lands inside a *room* (a plain `Walkable`
    /// cell — never a wall or a doorway), snapped onto the navmesh.
    /// Keeping endpoints out of doorways means a closing door never
    /// strands a patrol target off-mesh.
    fn random_room_point(&mut self, nav: &NavMesh, bsp: &Bsp) -> Option<Vertex> {
        for _ in 0..256 {
            let col = self.rng.next_in(GRID_W);
            let row = self.rng.next_in(GRID_H);
            if !matches!(self.cells[idx(col, row)], Cell::Walkable) {
                continue;
            }
            let v = Vertex::new(
                col as f64 + self.rng.unit_f64(),
                row as f64 + self.rng.unit_f64(),
            );
            if bsp.locate(nav, v).is_some() {
                return Some(v);
            }
        }
        None
    }

    // -- Patrol -----------------------------------------------------------

    /// Flip any actor that has reached its destination onto the next
    /// leg. The `Crowd` clears `agent.goal` on arrival, so a `None`
    /// goal means "arrived" — and an actor blocked by a shut door
    /// keeps its goal (and so keeps retrying) until the door reopens.
    fn tick_actors(&mut self) {
        for i in 0..self.actors.len() {
            let id = self.actors[i].id;
            let arrived = self
                .crowd
                .agent(id)
                .map(|a| a.goal.is_none())
                .unwrap_or(false);
            if arrived {
                self.actors[i].heading_away = !self.actors[i].heading_away;
                let target = if self.actors[i].heading_away {
                    self.actors[i].away
                } else {
                    self.actors[i].home
                };
                self.crowd.set_goal(
                    id,
                    Some(Goal {
                        target,
                        arrive_radius: ARRIVE_RADIUS,
                    }),
                );
            }
        }
    }

    /// Count actors whose last replan failed — i.e. the door(s) on
    /// every route to their goal are shut.
    fn blocked_count(&self) -> usize {
        self.actors
            .iter()
            .filter(|a| self.crowd.plan_failed(a.id))
            .count()
    }

    // -- Doors ------------------------------------------------------------

    /// Toggle the door nearest to a world-space point, if one is close
    /// enough to count as a click on it.
    fn toggle_door_near(&mut self, p: Vertex) {
        let mut best: Option<(usize, f64)> = None;
        for (i, door) in self.doors.iter().enumerate() {
            let d = door.distance_to(p);
            if d <= DOOR_CLICK_SLACK && best.map_or(true, |(_, bd)| d < bd) {
                best = Some((i, d));
            }
        }
        if let Some((i, _)) = best {
            self.doors[i].open = !self.doors[i].open;
            self.dirty = true;
        }
    }

    // -- Bitfield submission ---------------------------------------------

    fn maybe_submit(&mut self) {
        if !self.dirty {
            return;
        }
        if self.last_submit.elapsed() < self.submit_min_interval {
            return;
        }
        self.worker
            .submit_snapshot(Arc::new(build_bitfield(&self.cells, &self.doors)));
        self.last_submit = Instant::now();
        self.dirty = false;
    }

    // -- Rendering --------------------------------------------------------

    fn actor_color(i: usize) -> Color32 {
        const PALETTE: [Color32; 8] = [
            Color32::from_rgb(235, 150, 70),
            Color32::from_rgb(90, 190, 120),
            Color32::from_rgb(110, 165, 240),
            Color32::from_rgb(225, 110, 160),
            Color32::from_rgb(220, 205, 90),
            Color32::from_rgb(120, 210, 210),
            Color32::from_rgb(180, 130, 235),
            Color32::from_rgb(225, 130, 110),
        ];
        PALETTE[i % PALETTE.len()]
    }

    fn region_color(region: u32) -> Color32 {
        const PALETTE: [Color32; 6] = [
            Color32::from_rgb(180, 210, 255),
            Color32::from_rgb(190, 255, 200),
            Color32::from_rgb(255, 220, 190),
            Color32::from_rgb(245, 200, 245),
            Color32::from_rgb(255, 245, 180),
            Color32::from_rgb(200, 240, 240),
        ];
        PALETTE[(region as usize) % PALETTE.len()]
    }

    fn canvas_panel(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size_before_wrap();
        let (response, painter) = ui.allocate_painter(available, Sense::click());
        let rect = response.rect;

        // Click a door to toggle it.
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                self.toggle_door_near(screen_to_world(rect, pos));
            }
        }

        painter.rect_filled(rect, 0.0, Color32::from_gray(28));

        // Walls.
        if self.show_walls {
            for row in 0..GRID_H {
                for col in 0..GRID_W {
                    if !matches!(self.cells[idx(col, row)], Cell::Wall) {
                        continue;
                    }
                    let v0 = world_to_screen(rect, Vertex::new(col as f64, row as f64));
                    let v1 = world_to_screen(
                        rect,
                        Vertex::new(col as f64 + 1.0, row as f64 + 1.0),
                    );
                    painter.rect_filled(
                        Rect::from_two_pos(v0, v1),
                        0.0,
                        Color32::from_rgb(95, 95, 105),
                    );
                }
            }
        }

        // Navmesh.
        if let Some(build) = self.current_build.clone() {
            let nav = &build.navmesh;
            if self.show_navmesh {
                for tri in &nav.triangles {
                    let pa = world_to_screen(rect, nav.vertex(tri.vertices[0]));
                    let pb = world_to_screen(rect, nav.vertex(tri.vertices[1]));
                    let pc = world_to_screen(rect, nav.vertex(tri.vertices[2]));
                    let fill = if self.show_regions {
                        Self::region_color(tri.region).gamma_multiply(0.4)
                    } else {
                        Color32::from_rgba_unmultiplied(200, 220, 200, 45)
                    };
                    painter.add(egui::Shape::convex_polygon(
                        vec![pa, pb, pc],
                        fill,
                        Stroke::new(0.5, Color32::from_rgba_unmultiplied(40, 50, 55, 130)),
                    ));
                }
                for tri in &nav.triangles {
                    for edge in 0..3 {
                        if tri.edge_markers[edge] == 0 {
                            continue;
                        }
                        let (a, b) = tri.edge_vertices(edge);
                        let pa = world_to_screen(rect, nav.vertex(a));
                        let pb = world_to_screen(rect, nav.vertex(b));
                        painter.line_segment(
                            [pa, pb],
                            Stroke::new(1.0, Color32::from_rgb(20, 20, 30)),
                        );
                    }
                }
            }
        }

        // Doors — over the navmesh so their state always reads. An
        // open door is a green frame around the gap; a closed door is
        // a solid barrier.
        for door in &self.doors {
            let (c0, r0, c1, r1) = door.rect;
            let v0 = world_to_screen(rect, Vertex::new(c0 as f64, r0 as f64));
            let v1 = world_to_screen(
                rect,
                Vertex::new((c1 + 1) as f64, (r1 + 1) as f64),
            );
            let dr = Rect::from_two_pos(v0, v1);
            if door.open {
                painter.rect_stroke(dr, 0.0, Stroke::new(2.0, Color32::from_rgb(110, 200, 120)));
            } else {
                painter.rect_filled(dr, 0.0, Color32::from_rgb(170, 95, 50));
                painter.rect_stroke(dr, 0.0, Stroke::new(1.0, Color32::from_rgb(90, 50, 25)));
            }
        }

        // Actor paths.
        if self.show_paths {
            for (i, actor) in self.actors.iter().enumerate() {
                let Some(agent) = self.crowd.agent(actor.id) else {
                    continue;
                };
                let path = self.crowd.path(actor.id);
                if path.is_empty() {
                    continue;
                }
                let cursor = self.crowd.path_cursor(actor.id).unwrap_or(0);
                let remaining = path.get(cursor..).unwrap_or(&[]);
                if remaining.is_empty() {
                    continue;
                }
                let mut pts: Vec<Pos2> = Vec::with_capacity(remaining.len() + 1);
                pts.push(world_to_screen(rect, agent.pos));
                for v in remaining {
                    pts.push(world_to_screen(rect, *v));
                }
                let c = Self::actor_color(i);
                let line = Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), 120);
                painter.add(egui::Shape::line(pts, Stroke::new(1.4, line)));
            }
        }

        // Actors.
        if self.show_agents {
            let s = cell_size_px(rect);
            for (i, actor) in self.actors.iter().enumerate() {
                let Some(agent) = self.crowd.agent(actor.id) else {
                    continue;
                };
                let c = Self::actor_color(i);

                // Faint patrol-endpoint markers.
                for ep in [actor.home, actor.away] {
                    let p = world_to_screen(rect, ep);
                    painter.circle_stroke(
                        p,
                        (s * 0.35).max(2.0),
                        Stroke::new(1.0, Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), 90)),
                    );
                }

                let p = world_to_screen(rect, agent.pos);
                let r_px = (s * agent.radius as f32).max(2.5);
                // A blocked actor (every route shut) gets a red ring.
                let outline = if self.crowd.plan_failed(actor.id) {
                    Color32::from_rgb(230, 70, 70)
                } else {
                    Color32::BLACK
                };
                painter.circle_filled(p, r_px, c);
                painter.circle_stroke(p, r_px, Stroke::new(1.4, outline));

                if self.show_velocity && agent.vel.length_sq() > 1e-6 {
                    let tip = world_to_screen(rect, agent.pos + agent.vel * 0.25);
                    painter.line_segment(
                        [p, tip],
                        Stroke::new(1.5, Color32::from_rgb(20, 20, 20)),
                    );
                }
                if let Some(g) = agent.goal {
                    let gp = world_to_screen(rect, g.target);
                    painter.circle_stroke(
                        gp,
                        r_px * 0.8,
                        Stroke::new(1.2, c),
                    );
                }
            }
        }

        // World outline.
        let p0 = world_to_screen(rect, Vertex::new(0.0, 0.0));
        let p1 = world_to_screen(rect, Vertex::new(GRID_W as f64, GRID_H as f64));
        painter.rect_stroke(
            Rect::from_two_pos(p0, p1),
            0.0,
            Stroke::new(1.0, Color32::from_gray(120)),
        );
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("rsnav-door-demo");
        ui.add_space(2.0);
        ui.label(
            egui::RichText::new("toggle doors, watch actors repath")
                .small()
                .color(Color32::from_gray(150)),
        );

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Actors");
        ui.label(format!("count   : {}", self.actors.len()));
        let blocked = self.blocked_count();
        let blocked_color = if blocked > 0 {
            Color32::from_rgb(230, 120, 120)
        } else {
            Color32::from_gray(170)
        };
        ui.colored_label(blocked_color, format!("blocked : {blocked}"));
        ui.horizontal(|ui| {
            if ui.button("+1").clicked() {
                self.spawn_actor();
            }
            if ui.button("+5").clicked() {
                for _ in 0..5 {
                    self.spawn_actor();
                }
            }
            if ui.button("−1").clicked() {
                self.remove_last_actor();
            }
            if ui.button("clear").clicked() {
                self.clear_actors();
            }
        });

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Doors");
        ui.label(
            egui::RichText::new("click a door on the map, or toggle here")
                .small()
                .color(Color32::from_gray(150)),
        );
        let mut toggled = false;
        for (i, door) in self.doors.iter_mut().enumerate() {
            let state = if door.open { "open" } else { "closed" };
            if ui
                .checkbox(&mut door.open, format!("door {} — {state}", i + 1))
                .changed()
            {
                toggled = true;
            }
        }
        if toggled {
            self.dirty = true;
        }

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Display");
        ui.checkbox(&mut self.show_walls, "walls");
        ui.checkbox(&mut self.show_navmesh, "navmesh triangles");
        ui.checkbox(&mut self.show_regions, "region coloring");
        ui.checkbox(&mut self.show_paths, "actor paths");
        ui.checkbox(&mut self.show_agents, "actors");
        ui.checkbox(&mut self.show_velocity, "velocity heading");

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
            ui.colored_label(Color32::from_rgb(255, 120, 120), format!("error: {e}"));
        }

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Worker stats");
        let s = self.worker.stats();
        ui.label(format!("submitted : {}", s.snapshots_submitted));
        ui.label(format!("coalesced : {}", s.snapshots_coalesced));
        ui.label(format!("completed : {}", s.builds_completed));
        ui.label(format!("failed    : {}", s.builds_failed));
        ui.label(format!(
            "build ms  : {:.2} max / {:.2} last",
            s.max_build_ms, s.last_build_ms
        ));

        ui.add_space(10.0);
        ui.separator();
        ui.label(
            egui::RichText::new(
                "Closing a door carves the navmesh; Crowd::set_nav \
                 revalidates each actor's remaining path by \
                 line-of-sight and replans only the blocked ones.",
            )
            .small()
            .color(Color32::from_gray(140)),
        );
    }
}

// =========================================================================
// World painting + bitfield (free fns)
// =========================================================================

/// Paint the cross of full-extent walls. Door cells are carved back to
/// `DoorWay` by the caller.
fn paint_walls(cells: &mut [Cell]) {
    for row in 0..GRID_H {
        cells[idx(VWALL_COL, row)] = Cell::Wall;
    }
    for col in 0..GRID_W {
        cells[idx(col, HWALL_ROW)] = Cell::Wall;
    }
}

/// Build the bitfield the worker triangulates: base cell walkability,
/// then every *closed* door's cells forced shut. Open doors leave
/// their `DoorWay` cells walkable.
fn build_bitfield(cells: &[Cell], doors: &[Door]) -> Bitfield {
    let mut data: Vec<bool> = cells.iter().map(|c| c.walkable()).collect();
    for door in doors {
        if door.open {
            continue;
        }
        for (c, r) in door.cells() {
            data[idx(c, r)] = false;
        }
    }
    Bitfield::new(GRID_W, GRID_H, data).expect("dims match")
}

// =========================================================================
// Coordinate transforms (free fns)
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
// eframe::App glue
// =========================================================================

impl eframe::App for DoorDemoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Pull a fresh navmesh, if any. `set_nav` is where the
        // generation gap is closed: each actor's still-unwalked route
        // is revalidated by line-of-sight against the new mesh.
        if self.worker.poll_swap() {
            if let Some(b) = self.worker.current() {
                self.current_build = Some(b.clone());
                self.crowd.set_nav(b.clone());
            }
        }

        let now = Instant::now();
        let mut dt = (now - self.last_tick).as_secs_f64();
        self.last_tick = now;
        if dt > 0.1 {
            dt = 0.1;
        }

        self.tick_actors();
        self.maybe_submit();
        self.crowd.tick(dt);

        egui::SidePanel::left("controls")
            .resizable(false)
            .min_width(260.0)
            .show(ctx, |ui| {
                self.side_panel(ui);
            });
        egui::CentralPanel::default().show(ctx, |ui| {
            self.canvas_panel(ui);
        });

        ctx.request_repaint();
    }
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rect_normalizes_corner_order() {
        let d = Door::rect(10, 2, 4, 8, true);
        assert_eq!(d.rect, (4, 2, 10, 8));
    }

    #[test]
    fn vertical_door_is_one_column() {
        let d = Door::vertical(VWALL_COL, 5, 8, true);
        let cells: Vec<_> = d.cells().collect();
        assert_eq!(
            cells,
            vec![
                (VWALL_COL, 5),
                (VWALL_COL, 6),
                (VWALL_COL, 7),
                (VWALL_COL, 8),
            ]
        );
    }

    #[test]
    fn horizontal_door_is_one_row() {
        let d = Door::horizontal(HWALL_ROW, 11, 13, false);
        let cells: Vec<_> = d.cells().collect();
        assert_eq!(
            cells,
            vec![(11, HWALL_ROW), (12, HWALL_ROW), (13, HWALL_ROW)]
        );
    }

    #[test]
    fn closed_door_carves_the_bitfield_open_leaves_it() {
        let cells = vec![Cell::Walkable; (GRID_W * GRID_H) as usize];
        let shut = build_bitfield(&cells, &[Door::vertical(VWALL_COL, 10, 12, false)]);
        let open = build_bitfield(&cells, &[Door::vertical(VWALL_COL, 10, 12, true)]);
        assert!(!shut.at(VWALL_COL as i64, 11));
        assert!(open.at(VWALL_COL as i64, 11));
    }

    #[test]
    fn distance_to_zero_inside_grows_outside() {
        let d = Door::vertical(37, 10, 14, true);
        assert!(d.distance_to(Vertex::new(37.5, 12.0)) < 1e-9);
        assert!((d.distance_to(Vertex::new(34.0, 12.0)) - 3.0).abs() < 1e-9);
    }
}
