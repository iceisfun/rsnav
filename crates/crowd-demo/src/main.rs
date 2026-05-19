//! Phase-3 testbed for `rsnav-crowd`.
//!
//! Builds a small RTS world (town hall + mine + forest blob) and runs a
//! mix of peon roles on top of a shared [`Crowd`]:
//!
//! - **Wanderers** pick a random walkable goal and, on arrival, pick a
//!   new one — forever. The original phase-2 lane-forming demo.
//! - **Mine peons** loop `mine ring slot → harvest → hall ring slot →
//!   deposit → repeat`.
//! - **Forest peons** loop `nearest free forest cell → harvest (the
//!   cell flips to walkable) → hall ring slot → deposit → repeat`.
//!
//! When the forest runs dry, a fresh blob is sampled within
//! `{MIN_FOREST_DIST, MAX_FOREST_DIST}` cells of the town hall (and far
//! enough from the mine) and painted into the bitfield — the navmesh
//! follows automatically through the worker.
//!
//! Conventions match `rsnav-rtsim`:
//!   - 1 world unit = 1 cell.
//!   - Math-up Y (row 0 is at the bottom); egui draws Y-down, so we flip
//!     in [`CrowdDemoApp::world_to_screen`].
//!   - A background `NavWorker` owns the navmesh; we `poll_swap` once
//!     per frame and hand fresh builds to the `Crowd` via `set_nav`.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use eframe::egui;
use egui::{Color32, Pos2, Rect, Sense, Stroke};

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_crowd::{Agent, AgentId, Crowd, CrowdConfig, Goal};
use rsnav_dynamic::{BuildOptions, NavBuild, NavEvent, NavListener, NavWorker};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

// =========================================================================
// World constants
// =========================================================================

const GRID_W: u32 = 96;
const GRID_H: u32 = 64;

/// Town hall walls: inclusive cell rect (col0, row0, col1, row1).
const HALL_RECT: (u32, u32, u32, u32) = (14, 28, 18, 34);
/// Mine walls.
const MINE_RECT: (u32, u32, u32, u32) = (78, 28, 81, 32);
/// Center of the *initial* forest blob (subsequent blobs are sampled
/// against `MIN_FOREST_DIST` / `MAX_FOREST_DIST`).
const FOREST_INITIAL_CENTER: (i32, i32) = (50, 50);
const FOREST_BLOB_RADIUS: i32 = 6;
const MIN_FOREST_DIST: f64 = 16.0;
const MAX_FOREST_DIST: f64 = 36.0;
const MINE_KEEPOUT: f64 = 10.0; // min distance between a new blob center and the mine
/// Chance that a forest respawn drops a *second* blob as well.
const SECOND_FOREST_CHANCE: f64 = 0.15;

/// Per-role initial spawn counts.
const N_WANDERERS_INITIAL: usize = 3;
const N_MINE_PEONS_INITIAL: usize = 5;
const N_FOREST_PEONS_INITIAL: usize = 5;

const RADIUS_MIN: f64 = 0.30;
const RADIUS_MAX: f64 = 0.45;
const SPEED_DEFAULT: f64 = 8.0;
const ARRIVE_RADIUS: f64 = 0.5;
/// When every slot at a building is taken, a peon waits this far out
/// (world units, measured from the building center) instead of piling
/// against the wall — far enough to clear the slot-ring congestion,
/// close enough to dash in the moment a slot frees.
const STAGING_RADIUS: f64 = 11.0;
/// How many candidate loiter points to evaluate around the staging
/// ring; the least-congested one wins.
const STAGING_CANDIDATES: usize = 12;
/// Agents within this radius of a candidate count toward its
/// contention score.
const STAGING_OCCUPANCY_RADIUS: f64 = 4.5;
/// Per-world-unit penalty added to a candidate's contention score, so
/// a peon won't trek across the map for a marginally quieter spot.
/// At `0.06`, an extra ~17 units of walk costs as much as one agent.
const STAGING_DIST_WEIGHT: f64 = 0.06;

/// How long a peon harvests at a mine slot before it gets cargo.
const MINE_HARVEST_SECS: f64 = 1.0;
/// Harvest-seconds that must accumulate before a forest cell falls.
const FOREST_HARVEST_SECS: f64 = 0.6;
/// Up to this many peons can chop the same tree at once. Harvest
/// progress accumulates from every active chopper, so a full crew of
/// three fells a tree three times faster than a lone peon.
const MAX_FOREST_HARVESTERS: usize = 3;
/// How long a peon takes to drop cargo at the hall.
const DEPOSIT_SECS: f64 = 0.3;
/// Safety: if a peon spends more than this in a single FSM step, release
/// its slot and try again.
const STEP_TIMEOUT_SECS: f64 = 25.0;

const EVENT_LOG_CAP: usize = 10;

// =========================================================================
// fn main
// =========================================================================

fn main() -> eframe::Result<()> {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("rsnav-crowd-demo — peons + forest"),
        ..Default::default()
    };
    eframe::run_native(
        "rsnav-crowd-demo",
        opts,
        Box::new(|_cc| Ok(Box::new(CrowdDemoApp::new()))),
    )
}

// =========================================================================
// Cell grid
// =========================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Cell {
    Walkable,
    Forest,
    Hall,
    Mine,
}

impl Cell {
    /// Whether the navmesh should consider this cell walkable.
    ///
    /// Forest cells are **obstacles** — peons can't walk through them.
    /// A forest peon's harvest position is the closest walkable cell
    /// adjacent to its target tree; harvesting flips the tree cell to
    /// walkable, which is a real bool change in the bitfield and
    /// triggers a navmesh rebuild.
    fn walkable(self) -> bool {
        matches!(self, Cell::Walkable)
    }
}

// =========================================================================
// Tiny LCG so we don't need the `rand` crate.
// =========================================================================

struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self { Self { state: seed | 1 } }
    fn next_u32(&mut self) -> u32 {
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        ((z ^ (z >> 31)) >> 32) as u32
    }
    fn next_in(&mut self, bound: u32) -> u32 {
        if bound == 0 { 0 } else { self.next_u32() % bound }
    }
    fn unit_f64(&mut self) -> f64 {
        (self.next_u32() as f64) / (u32::MAX as f64)
    }
}

// =========================================================================
// Telemetry: drain `NavWorker` events into a small ring for the HUD.
// =========================================================================

#[derive(Clone, Debug)]
enum OwnedNavEvent {
    BuildStarted { generation: u64 },
    BuildCompleted {
        generation: u64,
        build_ms: f64,
        triangles: usize,
        regions: u32,
    },
    BuildFailed { generation: u64, error: String },
}

struct EventLog {
    events: Mutex<VecDeque<(Instant, OwnedNavEvent)>>,
}

impl EventLog {
    fn new() -> Self {
        Self {
            events: Mutex::new(VecDeque::with_capacity(EVENT_LOG_CAP)),
        }
    }
    fn snapshot(&self) -> Vec<(Instant, OwnedNavEvent)> {
        self.events.lock().expect("event log").iter().cloned().collect()
    }
}

impl NavListener for EventLog {
    fn on_event(&self, ev: &NavEvent<'_>) {
        let owned = match ev {
            NavEvent::BuildStarted { generation } => OwnedNavEvent::BuildStarted { generation: *generation },
            NavEvent::BuildCompleted { generation, build_ms, triangles, regions } => {
                OwnedNavEvent::BuildCompleted {
                    generation: *generation,
                    build_ms: *build_ms,
                    triangles: *triangles,
                    regions: *regions,
                }
            }
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
// Peon roles, slots, FSM
// =========================================================================

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum PeonRole {
    Wanderer,
    MinePeon,
    ForestPeon,
}

#[derive(Copy, Clone, Debug)]
enum ResourceSlot {
    Mine(usize),
    Forest(usize),
}

#[derive(Clone, Debug)]
enum PeonStep {
    /// Wanderer-only: pick a random walkable goal, repeat on arrival.
    Wander,
    /// Idle peons need to claim a resource slot and begin a cycle.
    Idle,
    /// Every resource slot was taken — heading to / idling at a staging
    /// point near the resource, retrying the claim from there.
    WaitingResourceSlot { started: Instant },
    GoingToResource { slot: ResourceSlot, started: Instant },
    Harvesting { slot: ResourceSlot, until: Instant },
    /// Carrying cargo but every hall drop-off was taken — heading to /
    /// idling at a staging point near the hall, retrying from there.
    WaitingHallSlot { started: Instant },
    GoingToHall { slot: usize, started: Instant },
    Depositing { slot: usize, until: Instant },
}

struct Peon {
    id: AgentId,
    role: PeonRole,
    step: PeonStep,
    deposits: u32,
}

#[derive(Debug)]
struct RingSlot {
    pos: Vertex,
    owner: Option<AgentId>,
}

#[derive(Debug)]
struct ForestSlot {
    col: u32,
    row: u32,
    /// Peons that have claimed this tree (capped at
    /// `MAX_FOREST_HARVESTERS`). Includes peons still walking to it;
    /// `progress` only advances from the ones in the `Harvesting` step.
    harvesters: Vec<AgentId>,
    /// Accumulated harvest time in seconds; the tree falls at
    /// `FOREST_HARVEST_SECS`. Advances by `active-choppers * dt`.
    progress: f64,
}

#[derive(Debug)]
struct ResourceMgr {
    mine_slots: Vec<RingSlot>,
    hall_slots: Vec<RingSlot>,
    /// Sparse — consumed cells become `None` so existing peons can keep
    /// referring to their owned-slot index without surprises.
    forest_cells: Vec<Option<ForestSlot>>,
}

impl ResourceMgr {
    fn new(hall_rect: (u32, u32, u32, u32), mine_rect: (u32, u32, u32, u32)) -> Self {
        Self {
            mine_slots: build_ring_slots(mine_rect),
            hall_slots: build_ring_slots(hall_rect),
            forest_cells: Vec::new(),
        }
    }

    fn forest_count(&self) -> usize {
        self.forest_cells.iter().filter(|s| s.is_some()).count()
    }

    /// Mean position of all remaining forest cells — used as the
    /// staging anchor for forest peons that can't claim a tree.
    /// `None` once the forest is fully harvested.
    fn forest_centroid(&self) -> Option<Vertex> {
        let mut acc = Vertex::ZERO;
        let mut n = 0u32;
        for s in self.forest_cells.iter().flatten() {
            acc = acc + Vertex::new(s.col as f64 + 0.5, s.row as f64 + 0.5);
            n += 1;
        }
        (n > 0).then(|| acc * (1.0 / n as f64))
    }

    fn add_forest(&mut self, col: u32, row: u32) {
        self.forest_cells.push(Some(ForestSlot {
            col,
            row,
            harvesters: Vec::new(),
            progress: 0.0,
        }));
    }

    fn claim_nearest_mine(&mut self, from: Vertex, agent: AgentId) -> Option<(usize, Vertex)> {
        claim_nearest_ring(&mut self.mine_slots, from, agent)
    }

    fn claim_nearest_hall(&mut self, from: Vertex, agent: AgentId) -> Option<(usize, Vertex)> {
        claim_nearest_ring(&mut self.hall_slots, from, agent)
    }

    fn release_mine(&mut self, idx: usize) {
        if let Some(s) = self.mine_slots.get_mut(idx) { s.owner = None; }
    }

    fn release_hall(&mut self, idx: usize) {
        if let Some(s) = self.hall_slots.get_mut(idx) { s.owner = None; }
    }

    /// Find the closest forest cell that still has chopper capacity
    /// (`harvesters.len() < MAX_FOREST_HARVESTERS`) and a walkable
    /// cardinal neighbor on the navmesh; the returned position is that
    /// *approach cell*, not the tree itself.
    ///
    /// Interior forest cells (surrounded by other forest cells) are
    /// skipped — they become claimable only after a perimeter neighbor
    /// has fallen.
    fn claim_nearest_forest(
        &mut self,
        from: Vertex,
        agent: AgentId,
        nav: &NavBuild,
    ) -> Option<(usize, Vertex)> {
        let mut best: Option<(usize, Vertex, f64)> = None;
        for (i, slot_opt) in self.forest_cells.iter().enumerate() {
            let Some(s) = slot_opt else { continue; };
            if s.harvesters.len() >= MAX_FOREST_HARVESTERS {
                continue;
            }
            if s.harvesters.contains(&agent) {
                continue;
            }
            let Some(approach) = closest_walkable_neighbor(s.col, s.row, nav, from)
            else { continue; };
            let d2 = (approach - from).length_sq();
            if best.map_or(true, |(_, _, bd)| d2 < bd) {
                best = Some((i, approach, d2));
            }
        }
        let (i, approach, _) = best?;
        if let Some(s) = self.forest_cells[i].as_mut() {
            s.harvesters.push(agent);
            return Some((i, approach));
        }
        None
    }

    fn release_forest(&mut self, idx: usize, agent: AgentId) {
        if let Some(Some(s)) = self.forest_cells.get_mut(idx) {
            s.harvesters.retain(|&a| a != agent);
        }
    }

    /// Remove the slot from the manager and return its grid coordinates.
    fn consume_forest(&mut self, idx: usize) -> Option<(u32, u32)> {
        let Some(slot_opt) = self.forest_cells.get_mut(idx) else { return None; };
        let Some(slot) = slot_opt.take() else { return None; };
        Some((slot.col, slot.row))
    }

    /// Release every slot currently owned by `agent` (used when an
    /// agent is removed mid-cycle).
    fn release_all_for(&mut self, agent: AgentId) {
        for s in &mut self.mine_slots {
            if s.owner == Some(agent) { s.owner = None; }
        }
        for s in &mut self.hall_slots {
            if s.owner == Some(agent) { s.owner = None; }
        }
        for slot_opt in self.forest_cells.iter_mut() {
            if let Some(s) = slot_opt {
                s.harvesters.retain(|&a| a != agent);
            }
        }
    }
}

fn build_ring_slots(rect: (u32, u32, u32, u32)) -> Vec<RingSlot> {
    let (c0, r0, c1, r1) = rect;
    let mut out = Vec::new();
    // Top row (r1 + 1).
    if r1 + 1 < GRID_H {
        for col in c0..=c1 {
            out.push(RingSlot {
                pos: Vertex::new(col as f64 + 0.5, (r1 + 1) as f64 + 0.5),
                owner: None,
            });
        }
    }
    // Bottom row (r0 - 1).
    if r0 > 0 {
        for col in c0..=c1 {
            out.push(RingSlot {
                pos: Vertex::new(col as f64 + 0.5, (r0 - 1) as f64 + 0.5),
                owner: None,
            });
        }
    }
    // Left col (c0 - 1).
    if c0 > 0 {
        for row in r0..=r1 {
            out.push(RingSlot {
                pos: Vertex::new((c0 - 1) as f64 + 0.5, row as f64 + 0.5),
                owner: None,
            });
        }
    }
    // Right col (c1 + 1).
    if c1 + 1 < GRID_W {
        for row in r0..=r1 {
            out.push(RingSlot {
                pos: Vertex::new((c1 + 1) as f64 + 0.5, row as f64 + 0.5),
                owner: None,
            });
        }
    }
    out
}

/// Return the center of the closest cardinal neighbor of cell
/// `(col, row)` that is currently walkable on the *navmesh* (so e.g.
/// just-cleared cells that haven't been re-extracted yet don't count
/// — `bsp.locate` is the source of truth). Closeness is measured to
/// `near`; ties are broken implicitly by iteration order.
///
/// Used to compute a forest peon's actual harvest position: peons
/// can't enter a tree cell, but they can stand next to it.
fn closest_walkable_neighbor(
    col: u32,
    row: u32,
    nav: &NavBuild,
    near: Vertex,
) -> Option<Vertex> {
    let candidates: [(i32, i32); 4] = [
        (col as i32 - 1, row as i32),
        (col as i32 + 1, row as i32),
        (col as i32, row as i32 - 1),
        (col as i32, row as i32 + 1),
    ];
    let mut best: Option<(Vertex, f64)> = None;
    for (nc, nr) in candidates {
        if nc < 0 || nr < 0 { continue; }
        if (nc as u32) >= GRID_W || (nr as u32) >= GRID_H { continue; }
        let v = Vertex::new(nc as f64 + 0.5, nr as f64 + 0.5);
        if nav.bsp.locate(&nav.navmesh, v).is_some() {
            let d2 = (v - near).length_sq();
            if best.map_or(true, |(_, bd)| d2 < bd) {
                best = Some((v, d2));
            }
        }
    }
    best.map(|(v, _)| v)
}

fn claim_nearest_ring(
    ring: &mut [RingSlot],
    from: Vertex,
    agent: AgentId,
) -> Option<(usize, Vertex)> {
    let mut best: Option<(usize, f64)> = None;
    for (i, s) in ring.iter().enumerate() {
        if s.owner.is_some() { continue; }
        let d2 = (s.pos - from).length_sq();
        if best.map_or(true, |(_, bd)| d2 < bd) {
            best = Some((i, d2));
        }
    }
    let (i, _) = best?;
    ring[i].owner = Some(agent);
    Some((i, ring[i].pos))
}

/// Hysteresis (world units) for slot stealing: a candidate slot must
/// be this much closer to me than my current claim — and, if
/// already owned, this much closer to me than to its current owner —
/// before I'll swap. Suppresses ping-pong between near-equidistant slots.
const REBALANCE_HYST: f64 = 1.5;

/// Look up the *best alternate* ring slot for `me_pos`.
///
/// Returns `Some((new_idx, new_pos, evicted))` where:
///   - `new_idx` / `new_pos` describe the slot to swap into;
///   - `evicted` is `Some(other_agent)` if the new slot was owned by
///     another peon (who must be reset by the caller), or `None` if it
///     was already free.
///
/// "Best" = strictly closer to me than my current slot, with
/// [`REBALANCE_HYST`] of margin. For owned candidates, we require that
/// the owner is in transit (not already harvesting/depositing) AND
/// strictly farther from the slot than I am, again with margin.
fn find_better_ring_slot(
    ring: &[RingSlot],
    me_pos: Vertex,
    cur_idx: usize,
    me_id: AgentId,
    pos_of: &std::collections::HashMap<AgentId, Vertex>,
    in_transit_of: &std::collections::HashMap<AgentId, bool>,
) -> Option<(usize, Vertex, Option<AgentId>)> {
    let cur_d = (ring.get(cur_idx)?.pos - me_pos).length();
    let mut best: Option<(usize, f64, Option<AgentId>)> = None;
    for (i, s) in ring.iter().enumerate() {
        if i == cur_idx { continue; }
        let d = (s.pos - me_pos).length();
        if d + REBALANCE_HYST >= cur_d { continue; }
        let evict = match s.owner {
            None => None,
            Some(o) if o == me_id => continue,
            Some(o) => {
                if !*in_transit_of.get(&o).unwrap_or(&false) { continue; }
                let Some(owner_pos) = pos_of.get(&o).copied() else { continue; };
                let owner_d = (s.pos - owner_pos).length();
                if d + REBALANCE_HYST < owner_d { Some(o) } else { continue }
            }
        };
        if best.map_or(true, |(_, bd, _)| d < bd) {
            best = Some((i, d, evict));
        }
    }
    let (i, _, evicted) = best?;
    Some((i, ring[i].pos, evicted))
}

// =========================================================================
// App
// =========================================================================

struct CrowdDemoApp {
    cells: Vec<Cell>,
    cell_w: u32,
    cell_h: u32,

    worker: NavWorker,
    event_log: Arc<EventLog>,
    app_started: Instant,

    current_build: Option<Arc<NavBuild>>,

    crowd: Crowd,
    peons: Vec<Peon>,
    resources: ResourceMgr,

    /// Bitfield has been edited (forest harvested / blob respawned)
    /// since the last submission to the worker.
    dirty: bool,
    last_submit: Instant,
    submit_min_interval: Duration,

    rng: Rng,
    last_tick: Instant,

    new_agent_speed: f64,

    /// Cumulative number of times the rebalancer kicked a peon off its
    /// claimed slot because a closer-to-the-slot peon took it.
    eviction_count: u32,

    show_navmesh: bool,
    show_regions: bool,
    show_bitfield: bool,
    show_slots: bool,
    show_paths: bool,
    show_agents: bool,
    show_velocity: bool,
}

impl CrowdDemoApp {
    fn new() -> Self {
        // 1) Paint the starting world.
        let mut cells = vec![Cell::Walkable; (GRID_W * GRID_H) as usize];
        paint_buildings(&mut cells);
        // Initial forest blob: collect cells into a list before flipping
        // them to Forest so we can register them with the manager later.
        let initial_forest = collect_forest_blob(
            &cells,
            FOREST_INITIAL_CENTER,
            FOREST_BLOB_RADIUS,
        );
        for (c, r) in &initial_forest {
            cells[(*r * GRID_W + *c) as usize] = Cell::Forest;
        }

        // 2) Spin up the worker and submit the first bitfield.
        let event_log = Arc::new(EventLog::new());
        let mut worker = NavWorker::spawn_with_listener(
            BuildOptions::default(),
            event_log.clone() as Arc<dyn NavListener>,
        );
        let data: Vec<bool> = cells.iter().map(|c| c.walkable()).collect();
        let bf = Bitfield::new(GRID_W, GRID_H, data).expect("dims match");
        worker.submit_snapshot(Arc::new(bf));

        let deadline = Instant::now() + Duration::from_millis(1500);
        loop {
            if worker.poll_swap() { break; }
            if Instant::now() > deadline { break; }
            std::thread::sleep(Duration::from_millis(2));
        }
        let current_build = worker
            .current()
            .expect("first navmesh build must succeed");

        // 3) Build the crowd and resource manager.
        let crowd = Crowd::new(current_build.clone(), CrowdConfig::default());
        let mut resources = ResourceMgr::new(HALL_RECT, MINE_RECT);
        for (c, r) in initial_forest {
            resources.add_forest(c, r);
        }

        let mut app = Self {
            cells,
            cell_w: GRID_W,
            cell_h: GRID_H,
            worker,
            event_log,
            app_started: Instant::now(),
            current_build: Some(current_build),
            crowd,
            peons: Vec::new(),
            resources,
            dirty: false,
            last_submit: Instant::now() - Duration::from_secs(1),
            submit_min_interval: Duration::from_millis(80),
            rng: Rng::new(0xCAFEC0DE),
            last_tick: Instant::now(),
            new_agent_speed: SPEED_DEFAULT,
            eviction_count: 0,
            show_navmesh: true,
            show_regions: false,
            show_bitfield: true,
            show_slots: true,
            show_paths: true,
            show_agents: true,
            show_velocity: true,
        };
        for _ in 0..N_WANDERERS_INITIAL { app.spawn_peon(PeonRole::Wanderer); }
        for _ in 0..N_MINE_PEONS_INITIAL { app.spawn_peon(PeonRole::MinePeon); }
        for _ in 0..N_FOREST_PEONS_INITIAL { app.spawn_peon(PeonRole::ForestPeon); }
        app
    }

    // -- Spawn / remove ---------------------------------------------------

    fn spawn_peon(&mut self, role: PeonRole) {
        let Some(build) = self.current_build.clone() else { return; };
        let radius = RADIUS_MIN + (RADIUS_MAX - RADIUS_MIN) * self.rng.unit_f64();
        let Some(pos) = self.random_walkable_point(&build.navmesh, &build.bsp) else { return; };
        let id = self.crowd.add_agent(Agent::new(pos, radius, self.new_agent_speed));
        let step = match role {
            PeonRole::Wanderer => PeonStep::Wander,
            _ => PeonStep::Idle,
        };
        self.peons.push(Peon { id, role, step, deposits: 0 });
    }

    fn remove_last_of(&mut self, role: PeonRole) {
        // Find the last peon with this role.
        let idx = self.peons.iter().rposition(|p| p.role == role);
        let Some(idx) = idx else { return; };
        let peon = self.peons.swap_remove(idx);
        self.resources.release_all_for(peon.id);
        self.crowd.remove_agent(peon.id);
    }

    fn clear_all_peons(&mut self) {
        for peon in self.peons.drain(..) {
            self.resources.release_all_for(peon.id);
            self.crowd.remove_agent(peon.id);
        }
    }

    fn random_walkable_point(&mut self, nav: &NavMesh, bsp: &Bsp) -> Option<Vertex> {
        for _ in 0..256 {
            let x = self.rng.next_in(self.cell_w) as f64 + self.rng.unit_f64();
            let y = self.rng.next_in(self.cell_h) as f64 + self.rng.unit_f64();
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

    // -- Peon FSM ---------------------------------------------------------

    fn tick_peons(&mut self, now: Instant, dt: f64) {
        let Some(build) = self.current_build.clone() else { return; };
        let nav = &build.navmesh;
        let bsp = &build.bsp;

        // Shared forest harvest: every peon actively in the Harvesting
        // step on a tree adds `dt` to that tree's progress, so three
        // choppers fell it three times faster. Cells that reach the
        // threshold are consumed here; their harvesters notice next.
        let mut chop: std::collections::HashMap<usize, u32> = std::collections::HashMap::new();
        for peon in &self.peons {
            if let PeonStep::Harvesting { slot: ResourceSlot::Forest(idx), .. } = peon.step {
                *chop.entry(idx).or_default() += 1;
            }
        }
        for (idx, count) in chop {
            if let Some(Some(cell)) = self.resources.forest_cells.get_mut(idx) {
                cell.progress += count as f64 * dt;
            }
        }
        let mut harvested_forest: Vec<(u32, u32)> = Vec::new();
        for idx in 0..self.resources.forest_cells.len() {
            let done = matches!(
                self.resources.forest_cells.get(idx),
                Some(Some(c)) if c.progress >= FOREST_HARVEST_SECS,
            );
            if done {
                if let Some((col, row)) = self.resources.consume_forest(idx) {
                    harvested_forest.push((col, row));
                }
            }
        }

        let n = self.peons.len();
        for i in 0..n {
            // Snapshot the agent's state before mutating crowd / resources.
            let (pos, has_goal) = match self.crowd.agent(self.peons[i].id) {
                Some(a) => (a.pos, a.goal.is_some()),
                None => continue,
            };
            let plan_failed = self.crowd.plan_failed(self.peons[i].id);
            let role = self.peons[i].role;

            // Timeout safety: anything but Idle / Wander that's been
            // running too long releases its slot and resets.
            let timed_out = match self.peons[i].step {
                PeonStep::GoingToResource { started, .. }
                | PeonStep::WaitingResourceSlot { started }
                | PeonStep::WaitingHallSlot { started }
                | PeonStep::GoingToHall { started, .. } => {
                    now.saturating_duration_since(started).as_secs_f64() > STEP_TIMEOUT_SECS
                }
                _ => false,
            };

            if timed_out {
                self.release_current_slot(i);
                self.peons[i].step = match role {
                    PeonRole::Wanderer => PeonStep::Wander,
                    _ => PeonStep::Idle,
                };
                self.crowd.set_goal(self.peons[i].id, None);
                continue;
            }

            match role {
                PeonRole::Wanderer => {
                    if !has_goal {
                        if let Some(g) = self.random_walkable_point(nav, bsp) {
                            self.crowd.set_goal(
                                self.peons[i].id,
                                Some(Goal { target: g, arrive_radius: ARRIVE_RADIUS }),
                            );
                        }
                    }
                }
                PeonRole::MinePeon | PeonRole::ForestPeon => {
                    self.advance_worker_peon(i, pos, has_goal, plan_failed, now, &build);
                }
            }
        }

        // Apply forest harvests: flip cells to walkable and mark the
        // bitfield dirty so the next throttle window submits.
        for (col, row) in harvested_forest {
            let idx = self.idx(col, row);
            if matches!(self.cells[idx], Cell::Forest) {
                self.cells[idx] = Cell::Walkable;
                self.dirty = true;
            }
        }
    }

    fn advance_worker_peon(
        &mut self,
        i: usize,
        pos: Vertex,
        has_goal: bool,
        plan_failed: bool,
        now: Instant,
        nav: &NavBuild,
    ) {
        let role = self.peons[i].role;

        // Plan failure → release current slot and reset.
        if plan_failed {
            self.release_current_slot(i);
            self.peons[i].step = PeonStep::Idle;
            self.crowd.set_goal(self.peons[i].id, None);
            return;
        }

        let id = self.peons[i].id;

        match self.peons[i].step.clone() {
            PeonStep::Wander => {} // unreachable for worker peons
            PeonStep::Idle => {
                if !self.try_claim_resource(i, pos, now, nav) {
                    // Every resource slot is taken — go wait at a
                    // staging point near the resource rather than
                    // loitering at the hall exit.
                    if let Some(center) = self.resource_staging_center(role) {
                        let stage = self.staging_point(center, pos);
                        self.crowd.set_goal(
                            id,
                            Some(Goal {
                                target: stage,
                                arrive_radius: ARRIVE_RADIUS * 2.0,
                            }),
                        );
                        self.peons[i].step =
                            PeonStep::WaitingResourceSlot { started: now };
                    }
                    // else: forest fully depleted (no centroid) — stay
                    // Idle; the blob respawns within a frame or two.
                }
            }
            PeonStep::WaitingResourceSlot { .. } => {
                // Retry from the staging point; on success the helper
                // redirects us into GoingToResource, otherwise we keep
                // heading to / idling at the staging point.
                let _ = self.try_claim_resource(i, pos, now, nav);
            }
            PeonStep::GoingToResource { slot, .. } => {
                if !has_goal {
                    // Mine harvest is a fixed per-peon timer; forest
                    // harvest is shared (cell progress), so `until` is
                    // unused there — parked at `now`.
                    let until = match slot {
                        ResourceSlot::Mine(_) => {
                            now + Duration::from_secs_f64(MINE_HARVEST_SECS)
                        }
                        ResourceSlot::Forest(_) => now,
                    };
                    self.peons[i].step = PeonStep::Harvesting { slot, until };
                    // Stand still while harvesting (crowd already has
                    // no goal; explicit None makes intent obvious).
                    self.crowd.set_goal(id, None);
                }
            }
            PeonStep::Harvesting { slot, until } => match slot {
                ResourceSlot::Mine(idx) => {
                    if now >= until {
                        self.resources.release_mine(idx);
                        self.try_claim_hall(i, pos, now);
                    }
                }
                ResourceSlot::Forest(idx) => {
                    // Shared harvest: tick_peons advances the cell's
                    // progress and consumes it when full. We're done
                    // the moment our tree is gone.
                    let cell_gone = self
                        .resources
                        .forest_cells
                        .get(idx)
                        .map_or(true, |c| c.is_none());
                    if cell_gone {
                        self.resources.release_forest(idx, id);
                        self.try_claim_hall(i, pos, now);
                    }
                }
            },
            PeonStep::WaitingHallSlot { .. } => {
                // Retry the hall claim only; on success head straight
                // in. Otherwise keep heading to / idling at the staging
                // point set when we entered this state — don't re-stage
                // every tick.
                if let Some((idx, target)) = self.resources.claim_nearest_hall(pos, id) {
                    self.crowd.set_goal(
                        id,
                        Some(Goal { target, arrive_radius: ARRIVE_RADIUS }),
                    );
                    self.peons[i].step = PeonStep::GoingToHall { slot: idx, started: now };
                }
            }
            PeonStep::GoingToHall { slot, .. } => {
                if !has_goal {
                    self.peons[i].step = PeonStep::Depositing {
                        slot,
                        until: now + Duration::from_secs_f64(DEPOSIT_SECS),
                    };
                    self.crowd.set_goal(id, None);
                }
            }
            PeonStep::Depositing { slot, until } => {
                if now >= until {
                    self.resources.release_hall(slot);
                    self.peons[i].deposits = self.peons[i].deposits.saturating_add(1);
                    self.peons[i].step = PeonStep::Idle;
                }
            }
        }
    }

    fn try_claim_hall(&mut self, i: usize, pos: Vertex, now: Instant) {
        let id = self.peons[i].id;
        if let Some((idx, target)) = self.resources.claim_nearest_hall(pos, id) {
            self.crowd.set_goal(
                id,
                Some(Goal { target, arrive_radius: ARRIVE_RADIUS }),
            );
            self.peons[i].step = PeonStep::GoingToHall { slot: idx, started: now };
        } else {
            // Every drop-off is taken — stage at a low-contention point
            // near the hall and retry from there, instead of piling
            // against the wall.
            let stage = self.staging_point(rect_center(HALL_RECT), pos);
            self.crowd.set_goal(
                id,
                Some(Goal {
                    target: stage,
                    arrive_radius: ARRIVE_RADIUS * 2.0,
                }),
            );
            self.peons[i].step = PeonStep::WaitingHallSlot { started: now };
        }
    }

    /// Attempt to claim this peon's resource slot (mine or forest by
    /// role). On success sets the goal + `GoingToResource` and returns
    /// `true`; on failure leaves the peon's step untouched.
    fn try_claim_resource(
        &mut self,
        i: usize,
        pos: Vertex,
        now: Instant,
        nav: &NavBuild,
    ) -> bool {
        let id = self.peons[i].id;
        let claimed = match self.peons[i].role {
            PeonRole::MinePeon => self
                .resources
                .claim_nearest_mine(pos, id)
                .map(|(idx, p)| (ResourceSlot::Mine(idx), p)),
            PeonRole::ForestPeon => self
                .resources
                .claim_nearest_forest(pos, id, nav)
                .map(|(idx, p)| (ResourceSlot::Forest(idx), p)),
            PeonRole::Wanderer => None,
        };
        if let Some((slot, target)) = claimed {
            self.crowd.set_goal(
                id,
                Some(Goal { target, arrive_radius: ARRIVE_RADIUS }),
            );
            self.peons[i].step = PeonStep::GoingToResource { slot, started: now };
            true
        } else {
            false
        }
    }

    /// The anchor a peon of `role` stages near when it can't claim a
    /// resource slot. `None` for wanderers, and for forest peons while
    /// the forest is fully depleted.
    fn resource_staging_center(&self, role: PeonRole) -> Option<Vertex> {
        match role {
            PeonRole::MinePeon => Some(rect_center(MINE_RECT)),
            PeonRole::ForestPeon => self.resources.forest_centroid(),
            PeonRole::Wanderer => None,
        }
    }

    /// A waiting spot on a `STAGING_RADIUS` ring around `center`.
    ///
    /// Rather than aiming straight down the approach line (which is
    /// exactly the busy inbound/outbound lane), this evaluates
    /// `STAGING_CANDIDATES` points spread all the way around the
    /// building and picks the one with the lowest contention score:
    /// `agents-nearby + STAGING_DIST_WEIGHT * walk-distance`. So peons
    /// fan out to the *quiet* sides of the building and the choice
    /// self-balances as the crowd shifts. Each candidate is snapped
    /// onto the navmesh.
    fn staging_point(&self, center: Vertex, from: Vertex) -> Vertex {
        let Some(build) = &self.current_build else {
            // No navmesh yet — fall back to a simple radial point.
            let dir = (from - center).normalize_or_zero();
            let dir = if dir.length_sq() < 0.5 {
                Vertex::new(1.0, 0.0)
            } else {
                dir
            };
            return center + dir * STAGING_RADIUS;
        };

        let mut best: Option<(Vertex, f64)> = None;
        for k in 0..STAGING_CANDIDATES {
            let theta = std::f64::consts::TAU * (k as f64) / (STAGING_CANDIDATES as f64);
            let (sin, cos) = theta.sin_cos();
            let raw = center + Vertex::new(cos * STAGING_RADIUS, sin * STAGING_RADIUS);
            // Snap onto the navmesh.
            let pt = if build.bsp.locate(&build.navmesh, raw).is_some() {
                raw
            } else {
                match build.bsp.nearest(&build.navmesh, raw) {
                    Some(n) => n.point,
                    None => continue,
                }
            };
            let occ = self.occupancy_near(pt, STAGING_OCCUPANCY_RADIUS);
            let score = occ as f64 + STAGING_DIST_WEIGHT * pt.distance(from);
            if best.map_or(true, |(_, bs)| score < bs) {
                best = Some((pt, score));
            }
        }
        best.map(|(p, _)| p).unwrap_or(from)
    }

    /// Count the agents whose position lies within `radius` of `p` —
    /// the contention proxy for staging-point selection.
    fn occupancy_near(&self, p: Vertex, radius: f64) -> usize {
        let r2 = radius * radius;
        self.peons
            .iter()
            .filter(|peon| {
                self.crowd
                    .agent(peon.id)
                    .is_some_and(|a| (a.pos - p).length_sq() <= r2)
            })
            .count()
    }

    /// Release whatever slot (if any) the peon currently owns.
    fn release_current_slot(&mut self, i: usize) {
        let id = self.peons[i].id;
        match self.peons[i].step.clone() {
            PeonStep::GoingToResource { slot, .. } | PeonStep::Harvesting { slot, .. } => {
                match slot {
                    ResourceSlot::Mine(idx) => self.resources.release_mine(idx),
                    ResourceSlot::Forest(idx) => self.resources.release_forest(idx, id),
                }
            }
            PeonStep::GoingToHall { slot, .. } => self.resources.release_hall(slot),
            PeonStep::Depositing { slot, .. } => self.resources.release_hall(slot),
            _ => {}
        }
    }

    // -- Opportunistic slot stealing --------------------------------------

    /// For every peon currently in transit (`GoingToResource` /
    /// `GoingToHall`), see if there's a slot of the same kind that's
    /// strictly better — either free + closer to me, or owned by a
    /// peon who is farther from the slot than I am. If so, swap claims
    /// and reset the evicted peon so its FSM re-claims fresh next tick.
    fn rebalance_slots(&mut self, now: Instant) {
        use std::collections::HashMap;
        let pos_of: HashMap<AgentId, Vertex> = self
            .peons
            .iter()
            .filter_map(|p| self.crowd.agent(p.id).map(|a| (p.id, a.pos)))
            .collect();
        let in_transit_of: HashMap<AgentId, bool> = self
            .peons
            .iter()
            .map(|p| {
                (
                    p.id,
                    matches!(
                        p.step,
                        PeonStep::GoingToResource { .. } | PeonStep::GoingToHall { .. }
                    ),
                )
            })
            .collect();

        for i in 0..self.peons.len() {
            let me_id = self.peons[i].id;
            let Some(me_pos) = pos_of.get(&me_id).copied() else { continue; };

            match self.peons[i].step.clone() {
                PeonStep::GoingToResource { slot: ResourceSlot::Mine(cur_idx), started } => {
                    if let Some((new_idx, new_pos, evicted)) = find_better_ring_slot(
                        &self.resources.mine_slots,
                        me_pos,
                        cur_idx,
                        me_id,
                        &pos_of,
                        &in_transit_of,
                    ) {
                        self.resources.release_mine(cur_idx);
                        if let Some(eid) = evicted {
                            self.evict_to_idle(eid);
                            self.eviction_count = self.eviction_count.saturating_add(1);
                        }
                        self.resources.mine_slots[new_idx].owner = Some(me_id);
                        self.crowd.set_goal(
                            me_id,
                            Some(Goal { target: new_pos, arrive_radius: ARRIVE_RADIUS }),
                        );
                        self.peons[i].step = PeonStep::GoingToResource {
                            slot: ResourceSlot::Mine(new_idx),
                            started,
                        };
                    }
                }
                // Forest cells are plentiful and shared (up to
                // MAX_FOREST_HARVESTERS choppers each) — no rebalancing.
                PeonStep::GoingToResource { slot: ResourceSlot::Forest(_), .. } => {}
                PeonStep::GoingToHall { slot: cur_idx, started } => {
                    let swap = find_better_ring_slot(
                        &self.resources.hall_slots,
                        me_pos,
                        cur_idx,
                        me_id,
                        &pos_of,
                        &in_transit_of,
                    );
                    if let Some((new_idx, new_pos, evicted)) = swap {
                        self.resources.release_hall(cur_idx);
                        if let Some(eid) = evicted {
                            // Hall-evicted peon is still carrying cargo;
                            // park them in WaitingHallSlot so their FSM
                            // re-claims next tick.
                            if let Some(j) =
                                self.peons.iter().position(|p| p.id == eid)
                            {
                                self.peons[j].step =
                                    PeonStep::WaitingHallSlot { started: now };
                                self.crowd.set_goal(eid, None);
                            }
                            self.eviction_count = self.eviction_count.saturating_add(1);
                        }
                        self.resources.hall_slots[new_idx].owner = Some(me_id);
                        self.crowd.set_goal(
                            me_id,
                            Some(Goal { target: new_pos, arrive_radius: ARRIVE_RADIUS }),
                        );
                        self.peons[i].step =
                            PeonStep::GoingToHall { slot: new_idx, started };
                    }
                }
                _ => {}
            }
        }
    }

    fn evict_to_idle(&mut self, id: AgentId) {
        let Some(j) = self.peons.iter().position(|p| p.id == id) else { return };
        self.peons[j].step = match self.peons[j].role {
            PeonRole::Wanderer => PeonStep::Wander,
            _ => PeonStep::Idle,
        };
        self.crowd.set_goal(id, None);
    }

    // -- Forest respawn ---------------------------------------------------

    fn maybe_respawn_forest(&mut self) {
        if self.resources.forest_count() > 0 {
            return;
        }
        if self.spawn_one_forest_blob() {
            self.dirty = true;
            // Occasionally drop a second blob in the same respawn.
            if self.rng.unit_f64() < SECOND_FOREST_CHANCE {
                self.spawn_one_forest_blob();
            }
        }
    }

    /// Sample a blob center within `MIN_FOREST_DIST..MAX_FOREST_DIST`
    /// of the town hall (and clear of the mine) and paint it. Returns
    /// `true` once a blob with at least one new cell lands.
    fn spawn_one_forest_blob(&mut self) -> bool {
        let hall = rect_center(HALL_RECT);
        let mine = rect_center(MINE_RECT);
        for _ in 0..64 {
            let angle = self.rng.unit_f64() * std::f64::consts::TAU;
            let dist =
                MIN_FOREST_DIST + (MAX_FOREST_DIST - MIN_FOREST_DIST) * self.rng.unit_f64();
            let cx = (hall.x + angle.cos() * dist).round() as i32;
            let cy = (hall.y + angle.sin() * dist).round() as i32;
            if cx < 0 || cy < 0 {
                continue;
            }
            if cx as u32 >= GRID_W || cy as u32 >= GRID_H {
                continue;
            }
            let dx = cx as f64 - mine.x;
            let dy = cy as f64 - mine.y;
            if (dx * dx + dy * dy).sqrt() < MINE_KEEPOUT {
                continue;
            }
            if self.paint_forest_blob_at(cx, cy, FOREST_BLOB_RADIUS) > 0 {
                return true;
            }
        }
        false
    }

    fn paint_forest_blob_at(&mut self, cx: i32, cy: i32, radius: i32) -> usize {
        let r2 = radius * radius;
        let mut placed = 0;
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy > r2 { continue; }
                let x = cx + dx;
                let y = cy + dy;
                if x < 0 || y < 0 { continue; }
                if x as u32 >= GRID_W || y as u32 >= GRID_H { continue; }
                let idx = self.idx(x as u32, y as u32);
                if matches!(self.cells[idx], Cell::Walkable) {
                    self.cells[idx] = Cell::Forest;
                    self.resources.add_forest(x as u32, y as u32);
                    placed += 1;
                }
            }
        }
        placed
    }

    // -- Bitfield submission ----------------------------------------------

    fn maybe_submit(&mut self) {
        if !self.dirty { return; }
        if self.last_submit.elapsed() < self.submit_min_interval { return; }
        let data: Vec<bool> = self.cells.iter().map(|c| c.walkable()).collect();
        let bf = Bitfield::new(GRID_W, GRID_H, data).expect("dims match");
        self.worker.submit_snapshot(Arc::new(bf));
        self.last_submit = Instant::now();
        self.dirty = false;
    }

    // -- After a navmesh swap: reassign goals that are now off-mesh -------

    fn reassign_after_swap(&mut self, build: &NavBuild) {
        let nav = &build.navmesh;
        let bsp = &build.bsp;
        for i in 0..self.peons.len() {
            let id = self.peons[i].id;
            let Some(agent) = self.crowd.agent(id) else { continue; };
            let goal_ok = agent
                .goal
                .map(|g| bsp.locate(nav, g.target).is_some())
                .unwrap_or(true);
            if !goal_ok {
                // Drop the goal; the FSM will re-derive one on the next
                // tick (which may also release the underlying slot).
                self.release_current_slot(i);
                self.peons[i].step = match self.peons[i].role {
                    PeonRole::Wanderer => PeonStep::Wander,
                    _ => PeonStep::Idle,
                };
                self.crowd.set_goal(id, None);
            }
        }
    }

    // -- Coordinate transforms --------------------------------------------

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
        Pos2::new(ox + v.x as f32 * s, oy + total_h - v.y as f32 * s)
    }

    // -- Rendering ---------------------------------------------------------

    fn region_color(&self, region: u32) -> Color32 {
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

    fn role_color(role: PeonRole) -> Color32 {
        match role {
            PeonRole::Wanderer => Color32::from_rgb(170, 170, 180),
            PeonRole::MinePeon => Color32::from_rgb(232, 145, 60),
            PeonRole::ForestPeon => Color32::from_rgb(80, 190, 110),
        }
    }

    fn canvas_panel(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size_before_wrap();
        let (response, painter) = ui.allocate_painter(available, Sense::hover());
        let rect = response.rect;
        painter.rect_filled(rect, 0.0, Color32::from_gray(28));
        let s = self.cell_size_px(rect);

        // Bitfield underlay.
        if self.show_bitfield {
            for row in 0..self.cell_h {
                for col in 0..self.cell_w {
                    let c = self.get_cell(col, row);
                    let color = match c {
                        Cell::Walkable => continue,
                        Cell::Forest => Color32::from_rgb(50, 110, 60),
                        Cell::Hall => Color32::from_rgb(70, 110, 200),
                        Cell::Mine => Color32::from_rgb(200, 170, 60),
                    };
                    let v0 = self.world_to_screen(rect, Vertex::new(col as f64, row as f64));
                    let v1 = self.world_to_screen(
                        rect,
                        Vertex::new(col as f64 + 1.0, row as f64 + 1.0),
                    );
                    let r = Rect::from_two_pos(v0, v1);
                    painter.rect_filled(r, 0.0, color);
                }
            }
        }

        // Navmesh.
        if let Some(build) = self.current_build.clone() {
            let nav = &build.navmesh;
            if self.show_navmesh {
                for tri in &nav.triangles {
                    let pa = self.world_to_screen(rect, nav.vertex(tri.vertices[0]));
                    let pb = self.world_to_screen(rect, nav.vertex(tri.vertices[1]));
                    let pc = self.world_to_screen(rect, nav.vertex(tri.vertices[2]));
                    let fill = if self.show_regions {
                        self.region_color(tri.region).gamma_multiply(0.4)
                    } else {
                        Color32::from_rgba_unmultiplied(200, 220, 200, 50)
                    };
                    // Stroke the triangle so the full triangulation —
                    // including interior subdivision edges — is visible
                    // and you can watch it re-mesh as the world changes.
                    painter.add(egui::Shape::convex_polygon(
                        vec![pa, pb, pc],
                        fill,
                        Stroke::new(0.5, Color32::from_rgba_unmultiplied(40, 50, 55, 130)),
                    ));
                }
                // Constraint / boundary edges over the wireframe, in a
                // darker, thicker tone so walls read clearly.
                for tri in &nav.triangles {
                    for edge in 0..3 {
                        if tri.edge_markers[edge] == 0 { continue; }
                        let (a, b) = tri.edge_vertices(edge);
                        let pa = self.world_to_screen(rect, nav.vertex(a));
                        let pb = self.world_to_screen(rect, nav.vertex(b));
                        painter.line_segment(
                            [pa, pb],
                            Stroke::new(1.0, Color32::from_rgb(20, 20, 30)),
                        );
                    }
                }
            }
        }

        // Resource slots (small open rings at the slot world positions).
        if self.show_slots {
            for slot in &self.resources.mine_slots {
                let p = self.world_to_screen(rect, slot.pos);
                let outline = if slot.owner.is_some() {
                    Color32::from_rgba_unmultiplied(232, 145, 60, 230)
                } else {
                    Color32::from_rgba_unmultiplied(232, 145, 60, 120)
                };
                painter.circle_stroke(p, (s * 0.25).max(2.0), Stroke::new(1.2, outline));
            }
            for slot in &self.resources.hall_slots {
                let p = self.world_to_screen(rect, slot.pos);
                let outline = if slot.owner.is_some() {
                    Color32::from_rgba_unmultiplied(110, 160, 240, 230)
                } else {
                    Color32::from_rgba_unmultiplied(110, 160, 240, 120)
                };
                painter.circle_stroke(p, (s * 0.25).max(2.0), Stroke::new(1.2, outline));
            }
        }

        // Agent paths.
        if self.show_paths {
            for peon in &self.peons {
                let id = peon.id;
                let Some(agent) = self.crowd.agent(id) else { continue; };
                let path = self.crowd.path(id);
                if path.is_empty() { continue; }
                let cursor = self.crowd.path_cursor(id).unwrap_or(0);
                let remaining = path.get(cursor..).unwrap_or(&[]);
                if remaining.is_empty() { continue; }
                let mut pts: Vec<Pos2> = Vec::with_capacity(remaining.len() + 1);
                pts.push(self.world_to_screen(rect, agent.pos));
                for v in remaining {
                    pts.push(self.world_to_screen(rect, *v));
                }
                let c = Self::role_color(peon.role);
                let line_c = Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), 110);
                painter.add(egui::Shape::line(pts, Stroke::new(1.2, line_c)));
            }
        }

        // Agents.
        if self.show_agents {
            for peon in &self.peons {
                let Some(agent) = self.crowd.agent(peon.id) else { continue; };
                let p = self.world_to_screen(rect, agent.pos);
                let r_px = (s * agent.radius as f32).max(2.5);
                let body = Self::role_color(peon.role);
                painter.circle_filled(p, r_px, body);
                painter.circle_stroke(p, r_px, Stroke::new(1.0, Color32::BLACK));

                // Cargo dot.
                let carrying = matches!(
                    peon.step,
                    PeonStep::WaitingHallSlot { .. }
                        | PeonStep::GoingToHall { .. }
                        | PeonStep::Depositing { .. }
                );
                if carrying {
                    painter.circle_filled(p, (r_px * 0.4).max(1.5), Color32::WHITE);
                }

                // Velocity heading.
                if self.show_velocity {
                    let v = agent.vel;
                    if v.length_sq() > 1e-6 {
                        let tip = agent.pos + v * 0.25;
                        let pt = self.world_to_screen(rect, tip);
                        painter.line_segment(
                            [p, pt],
                            Stroke::new(1.5, Color32::from_rgb(20, 20, 20)),
                        );
                    }
                }

                if let Some(g) = agent.goal {
                    let gp = self.world_to_screen(rect, g.target);
                    painter.circle_stroke(
                        gp,
                        r_px * 0.7,
                        Stroke::new(1.0, Color32::from_rgba_unmultiplied(body.r(), body.g(), body.b(), 180)),
                    );
                }
            }
        }

        // World outline.
        let p0 = self.world_to_screen(rect, Vertex::new(0.0, 0.0));
        let p1 = self.world_to_screen(rect, Vertex::new(self.cell_w as f64, self.cell_h as f64));
        painter.rect_stroke(
            Rect::from_two_pos(p0, p1),
            0.0,
            Stroke::new(1.0, Color32::from_gray(120)),
        );
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("rsnav-crowd-demo");
        ui.add_space(2.0);
        ui.label(
            egui::RichText::new("phase 3: peons + resources + forest respawn")
                .small()
                .color(Color32::from_gray(150)),
        );
        ui.add_space(8.0);
        ui.separator();
        ui.heading("Peons");
        let (n_w, n_m, n_f, deposits) = self.peon_totals();
        ui.label(format!("wanderers     : {n_w}"));
        ui.horizontal(|ui| {
            if ui.button("+1").clicked() { self.spawn_peon(PeonRole::Wanderer); }
            if ui.button("+5").clicked() {
                for _ in 0..5 { self.spawn_peon(PeonRole::Wanderer); }
            }
            if ui.button("−1").clicked() { self.remove_last_of(PeonRole::Wanderer); }
        });
        ui.label(format!("mine peons    : {n_m}"));
        ui.horizontal(|ui| {
            if ui.button("+1").clicked() { self.spawn_peon(PeonRole::MinePeon); }
            if ui.button("+5").clicked() {
                for _ in 0..5 { self.spawn_peon(PeonRole::MinePeon); }
            }
            if ui.button("−1").clicked() { self.remove_last_of(PeonRole::MinePeon); }
        });
        ui.label(format!("forest peons  : {n_f}"));
        ui.horizontal(|ui| {
            if ui.button("+1").clicked() { self.spawn_peon(PeonRole::ForestPeon); }
            if ui.button("+5").clicked() {
                for _ in 0..5 { self.spawn_peon(PeonRole::ForestPeon); }
            }
            if ui.button("−1").clicked() { self.remove_last_of(PeonRole::ForestPeon); }
        });
        if ui.button("clear all peons").clicked() {
            self.clear_all_peons();
        }
        ui.label(format!("total deposits: {deposits}"));
        ui.add(
            egui::Slider::new(&mut self.new_agent_speed, 1.0..=20.0)
                .text("new-agent max speed"),
        );

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Resources");
        let mine_used = self.resources.mine_slots.iter().filter(|s| s.owner.is_some()).count();
        let hall_used = self.resources.hall_slots.iter().filter(|s| s.owner.is_some()).count();
        ui.label(format!("mine slots    : {}/{} used", mine_used, self.resources.mine_slots.len()));
        ui.label(format!("hall slots    : {}/{} used", hall_used, self.resources.hall_slots.len()));
        ui.label(format!("forest cells  : {} remaining", self.resources.forest_count()));
        ui.label(format!("evictions     : {}", self.eviction_count));

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Display");
        ui.checkbox(&mut self.show_bitfield, "bitfield underlay");
        ui.checkbox(&mut self.show_navmesh, "navmesh triangles");
        ui.checkbox(&mut self.show_regions, "region coloring");
        ui.checkbox(&mut self.show_slots, "resource slot markers");
        ui.checkbox(&mut self.show_agents, "agents");
        ui.checkbox(&mut self.show_paths, "agent paths");
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
        ui.label(format!("failed       : {}", s.builds_failed));
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
                ui.colored_label(color, egui::RichText::new(text).monospace().size(11.0));
            }
        }
    }

    fn peon_totals(&self) -> (usize, usize, usize, u32) {
        let mut n_w = 0;
        let mut n_m = 0;
        let mut n_f = 0;
        let mut deposits = 0u32;
        for p in &self.peons {
            match p.role {
                PeonRole::Wanderer => n_w += 1,
                PeonRole::MinePeon => n_m += 1,
                PeonRole::ForestPeon => n_f += 1,
            }
            deposits = deposits.saturating_add(p.deposits);
        }
        (n_w, n_m, n_f, deposits)
    }
}

// =========================================================================
// World painting (free fns)
// =========================================================================

/// World-space center of an inclusive cell rect.
fn rect_center(rect: (u32, u32, u32, u32)) -> Vertex {
    let (c0, r0, c1, r1) = rect;
    Vertex::new(
        (c0 as f64 + c1 as f64) * 0.5 + 0.5,
        (r0 as f64 + r1 as f64) * 0.5 + 0.5,
    )
}

fn paint_buildings(cells: &mut [Cell]) {
    paint_rect(cells, HALL_RECT, Cell::Hall);
    paint_rect(cells, MINE_RECT, Cell::Mine);
}

fn paint_rect(cells: &mut [Cell], rect: (u32, u32, u32, u32), kind: Cell) {
    let (c0, r0, c1, r1) = rect;
    for row in r0..=r1 {
        for col in c0..=c1 {
            if col >= GRID_W || row >= GRID_H { continue; }
            cells[(row * GRID_W + col) as usize] = kind;
        }
    }
}

fn collect_forest_blob(cells: &[Cell], center: (i32, i32), radius: i32) -> Vec<(u32, u32)> {
    let (cx, cy) = center;
    let r2 = radius * radius;
    let mut out = Vec::new();
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            if dx * dx + dy * dy > r2 { continue; }
            let x = cx + dx;
            let y = cy + dy;
            if x < 0 || y < 0 { continue; }
            if x as u32 >= GRID_W || y as u32 >= GRID_H { continue; }
            let idx = (y as u32 * GRID_W + x as u32) as usize;
            if matches!(cells[idx], Cell::Walkable) {
                out.push((x as u32, y as u32));
            }
        }
    }
    out
}

// =========================================================================
// eframe::App glue
// =========================================================================

impl eframe::App for CrowdDemoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Pull a fresh build, if any, and let the crowd + FSM react.
        if self.worker.poll_swap() {
            if let Some(b) = self.worker.current() {
                self.current_build = Some(b.clone());
                self.crowd.set_nav(b.clone());
                self.reassign_after_swap(&b);
            }
        }

        let now = Instant::now();
        let mut dt = (now - self.last_tick).as_secs_f64();
        self.last_tick = now;
        if dt > 0.1 { dt = 0.1; }

        self.rebalance_slots(now);
        self.tick_peons(now, dt);
        self.maybe_respawn_forest();
        self.maybe_submit();
        self.crowd.tick(dt);

        egui::SidePanel::left("controls")
            .resizable(false)
            .min_width(280.0)
            .show(ctx, |ui| {
                self.side_panel(ui);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.canvas_panel(ui);
        });

        ctx.request_repaint();
    }
}
