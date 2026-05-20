//! Multi-agent crowd simulation over an [`rsnav_dynamic::NavBuild`].
//!
//! `rsnav-crowd` is the Detour-Crowd analogue for this workspace: each
//! [`Agent`] gets its own funnel-pulled path corridor through a shared
//! navmesh, and a sampled velocity-obstacle solver picks a per-tick
//! velocity that follows the corridor while side-stepping other agents.
//!
//! ## Quick start
//!
//! ```no_run
//! use std::sync::Arc;
//! use rsnav_common::Vertex;
//! use rsnav_crowd::{Agent, Crowd, CrowdConfig, Goal};
//! use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
//! use rsnav_polygon_extract::Bitfield;
//!
//! let bf = Bitfield::new(16, 16, vec![true; 16 * 16]).unwrap();
//! let nav = Arc::new(
//!     build_navmesh_from_bitfield(&bf, &BuildOptions::default()).unwrap(),
//! );
//! let mut crowd = Crowd::new(nav, CrowdConfig::default());
//! let id = crowd.add_agent(Agent::new(Vertex::new(2.0, 8.0), 0.3, 2.0));
//! crowd.set_goal(
//!     id,
//!     Some(Goal { target: Vertex::new(14.0, 8.0), arrive_radius: 0.5 }),
//! );
//! for _ in 0..1_000 {
//!     crowd.tick(1.0 / 60.0);
//!     if crowd.agent(id).unwrap().goal.is_none() { break; }
//! }
//! ```
//!
//! ## Per-tick pipeline
//!
//! [`Crowd::tick`] runs four passes:
//!
//! 1. **Replan / arrive.** For each agent with a goal, check whether
//!    the goal is reached (clear it) or whether a new path is needed
//!    (path is empty, or `stuck` ≥ `stuck_ticks`). Replans go through
//!    [`rsnav_navigation::find_path`] with `distance_from_wall` set to
//!    the agent's `radius`, so each agent gets a corridor sized for
//!    its body.
//! 2. **Rebuild spatial hash.** A simple hash grid over all live
//!    agents, sized to the configured `neighbor_radius`.
//! 3. **Choose velocities.** For each agent: compute a preferred
//!    velocity toward the next corridor corner; generate `vo_samples`
//!    candidate velocities spread ±π around the preferred direction
//!    (plus a zero-velocity "brake" sample); score each by alignment
//!    minus a time-to-collision penalty against neighbors; pick the
//!    winner.
//! 4. **Integrate.** Advance positions, advance the corridor cursor
//!    when within `arrive_eps` of the current corner, and update each
//!    agent's `stuck` counter.
//!
//! ## Limitations (v0)
//!
//! - No formation / goal-slot logic — give every agent a unique goal
//!   target if you don't want them stacking on a single point.
//! - Two agents that are *perfectly* head-on with identical radii,
//!   speeds, and [`priority`](Agent::priority) can fail to pick a side;
//!   supply a small positional offset, different goals, or different
//!   priorities to break the symmetry.
//! - Avoidance does not consult the navmesh directly: it trusts the
//!   path corridor for wall clearance. If avoidance briefly pushes an
//!   agent off-corridor, the `stuck` counter triggers a replan.
//! - Use [`Crowd::set_nav`] whenever the navmesh is hot-swapped
//!   (e.g. after [`rsnav_dynamic::NavWorker::poll_swap`]); all paths
//!   are invalidated and replanned on the next tick.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

use std::collections::HashMap;
use std::sync::Arc;

use rsnav_common::Vertex;
use rsnav_dynamic::NavBuild;
use rsnav_navigation::{find_path, path_clear, PathOptions};

// =========================================================================
// Public types
// =========================================================================

/// Opaque handle for an agent in a [`Crowd`].
///
/// Indices are stable across removals: removing an agent leaves a hole
/// that the next [`Crowd::add_agent`] may reuse, but ids of other agents
/// never shift.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct AgentId(pub u32);

/// A goal an agent is currently trying to reach.
#[derive(Copy, Clone, Debug)]
pub struct Goal {
    /// World-space target point.
    pub target: Vertex,
    /// The agent's goal is cleared once `pos.distance(target) <= arrive_radius`.
    pub arrive_radius: f64,
}

/// Snapshot of a single agent's externally visible state.
///
/// Internal fields (current path, cursor, stuck counter) are owned by
/// the [`Crowd`] and not exposed.
#[derive(Copy, Clone, Debug)]
pub struct Agent {
    pub pos: Vertex,
    pub vel: Vertex,
    /// Body radius. Drives both planning clearance and disc-disc avoidance.
    pub radius: f64,
    /// Maximum desired speed, in world-units per second.
    pub max_speed: f64,
    /// User-defined right-of-way. Higher-priority agents hold their line
    /// while lower-priority neighbors yield: in the collision penalty,
    /// the predicted-collision term is scaled by `2^((other - me)/2)`,
    /// clamped so the gap maps into a `[0.25, 4.0]` factor. A crowd that
    /// leaves every `priority` at the default `0.0` is unaffected. (A
    /// hard, already-overlapping contact is never discounted — even a
    /// top-priority agent still separates from one.)
    pub priority: f32,
    /// Current goal, if any. `None` ⇒ the agent is idle and brakes.
    pub goal: Option<Goal>,
}

impl Agent {
    /// Construct an idle agent at `pos` with the given body radius and
    /// max speed. Velocity is zero and there is no goal yet.
    pub fn new(pos: Vertex, radius: f64, max_speed: f64) -> Self {
        Self {
            pos,
            vel: Vertex::ZERO,
            radius,
            max_speed,
            priority: 0.0,
            goal: None,
        }
    }
}

/// Tunables for the per-tick solver.
#[derive(Copy, Clone, Debug)]
pub struct CrowdConfig {
    /// Number of angular candidate velocities to test per agent per tick
    /// (in addition to a zero-velocity "brake" sample). Spread evenly
    /// over ±π around the preferred direction. Default `16`.
    pub vo_samples: u32,
    /// World-space radius for neighbor queries. Agents farther than this
    /// from one another are ignored by avoidance. Default `6.0`.
    pub neighbor_radius: f64,
    /// Time-to-collision horizon. Candidates whose TTC against any
    /// neighbor falls below this value are penalised, linearly more so
    /// the closer the collision is to now. Default `1.5`s.
    pub time_horizon: f64,
    /// Replan trigger: if the agent has moved less than ~10% of its
    /// expected per-tick distance for this many consecutive ticks, the
    /// corridor is discarded and rebuilt. Default `60` (≈1 s at 60 Hz).
    pub stuck_ticks: u32,
    /// Weight applied to the alignment term (preferred-velocity dot
    /// candidate, normalised). Default `1.0`.
    pub align_weight: f64,
    /// Weight applied to the time-to-collision penalty. Default `2.0`.
    /// Heavier values make agents more cautious; lighter values let them
    /// crowd more tightly but increase the chance of brushing contact.
    pub avoid_weight: f64,
    /// How close (in world units) the agent must come to the current
    /// corridor corner before the cursor advances. Default `0.25`.
    pub arrive_eps: f64,
    /// Speed, as a fraction of the agent's `max_speed`, at which a
    /// *goal-less* agent nudges itself out of the way when a neighbor
    /// closes in. Keeps a parked agent "soft" — it yields instead of
    /// acting as an immovable wall — without it wandering far from
    /// where it stopped. Default `0.5`. Set to `0.0` to make idle
    /// agents fully immovable (the pre-soft-hold behavior).
    pub hold_speed_frac: f64,
}

impl Default for CrowdConfig {
    fn default() -> Self {
        Self {
            vo_samples: 16,
            neighbor_radius: 6.0,
            time_horizon: 1.5,
            stuck_ticks: 60,
            align_weight: 1.0,
            avoid_weight: 2.0,
            arrive_eps: 0.25,
            hold_speed_frac: 0.5,
        }
    }
}

// =========================================================================
// Internal slot
// =========================================================================

#[derive(Debug)]
struct Slot {
    agent: Agent,
    /// Funnel-pulled corridor from current pos to goal. `path[0]` is the
    /// start (snapshot at plan time) and `path.last()` is the goal.
    path: Vec<Vertex>,
    /// Index of the corner the agent is currently steering toward.
    cursor: usize,
    /// Consecutive ticks of near-zero progress while a goal is active.
    stuck: u32,
    /// Position recorded at the end of the previous tick, for `stuck`
    /// detection.
    last_pos: Vertex,
    /// Set when `find_path` last returned `Err`. Suppresses further
    /// replan attempts until the goal or nav is replaced.
    plan_failed: bool,
}

impl Slot {
    fn new(agent: Agent) -> Self {
        let pos = agent.pos;
        Self {
            agent,
            path: Vec::new(),
            cursor: 0,
            stuck: 0,
            last_pos: pos,
            plan_failed: false,
        }
    }

    /// Drop the current corridor and reset planning state.
    fn invalidate_path(&mut self) {
        self.path.clear();
        self.cursor = 0;
        self.stuck = 0;
        self.plan_failed = false;
    }
}

// =========================================================================
// Spatial hash (uniform grid)
// =========================================================================

#[derive(Debug)]
struct SpatialHash {
    cell: f64,
    bins: HashMap<(i32, i32), Vec<usize>>,
}

impl SpatialHash {
    fn new(cell: f64) -> Self {
        Self {
            cell: cell.max(0.01),
            bins: HashMap::new(),
        }
    }

    fn key(&self, p: Vertex) -> (i32, i32) {
        (
            (p.x / self.cell).floor() as i32,
            (p.y / self.cell).floor() as i32,
        )
    }

    fn clear(&mut self) {
        for v in self.bins.values_mut() {
            v.clear();
        }
    }

    fn insert(&mut self, idx: usize, p: Vertex) {
        let k = self.key(p);
        self.bins.entry(k).or_default().push(idx);
    }

    fn for_neighbors<F: FnMut(usize)>(&self, p: Vertex, r: f64, mut f: F) {
        let span = ((r / self.cell).ceil() as i32).max(0);
        let (cx, cy) = self.key(p);
        for dy in -span..=span {
            for dx in -span..=span {
                if let Some(v) = self.bins.get(&(cx + dx, cy + dy)) {
                    for &i in v {
                        f(i);
                    }
                }
            }
        }
    }
}

// =========================================================================
// Crowd
// =========================================================================

/// The simulation owner.
///
/// Hosts the agent slab, a shared [`Arc<NavBuild>`] used for planning,
/// and the per-tick spatial hash + velocity scratch buffer. See the
/// [crate-level docs](crate) for the per-tick pipeline.
#[derive(Debug)]
pub struct Crowd {
    nav: Arc<NavBuild>,
    slots: Vec<Option<Slot>>,
    config: CrowdConfig,
    hash: SpatialHash,
    next_vels: Vec<Vertex>,
}

impl Crowd {
    /// Construct an empty crowd against the given navmesh build.
    pub fn new(nav: Arc<NavBuild>, config: CrowdConfig) -> Self {
        let hash = SpatialHash::new(config.neighbor_radius.max(1.0));
        Self {
            nav,
            slots: Vec::new(),
            config,
            hash,
            next_vels: Vec::new(),
        }
    }

    pub fn config(&self) -> &CrowdConfig {
        &self.config
    }

    /// Borrow the current navmesh build.
    pub fn nav(&self) -> &NavBuild {
        &self.nav
    }

    /// Swap to a freshly-published navmesh (e.g. after a `NavWorker`
    /// `poll_swap`). Each agent's *remaining route* — from where it is
    /// now, through every corridor corner it has yet to reach — is
    /// revalidated against the new mesh with segment line-of-sight
    /// ([`rsnav_navigation::path_clear`]). Routes that are still clear
    /// are kept; only the ones an obstacle has actually broken are
    /// cleared, to be replanned on the next [`Crowd::tick`].
    ///
    /// Segment validation (not a corner-only on-mesh test) matters: a
    /// building or forest can spawn *between* two still-on-mesh corners
    /// and block the straight leg between them — a corner check would
    /// miss it and the agent would walk through the new obstacle.
    ///
    /// Keeping still-valid routes avoids a global replan storm when a
    /// mesh regeneration is cosmetic or strictly additive (a wall was
    /// removed — existing corridors stay valid; agents just won't take
    /// the new shortcut until their next natural replan).
    ///
    /// Revalidation is geometric, not clearance-aware. [`path_clear`] is
    /// a zero-width line-of-sight test: it confirms a leg still does not
    /// *cross* a wall, but does not re-verify the agent's radius of
    /// clearance along it. If a rebuild narrows a corridor below the
    /// agent's body width without severing the leg's centerline, the
    /// route is kept — the agent then fails to make progress against
    /// the new wall and its `stuck` counter forces a full,
    /// clearance-aware replan (`find_path` with `distance_from_wall`)
    /// within `stuck_ticks`. A precise swept-disc revalidation here is a
    /// deliberate non-goal: it would cost more than the occasional
    /// late replan it would save.
    pub fn set_nav(&mut self, nav: Arc<NavBuild>) {
        self.nav = nav.clone();
        for slot in self.slots.iter_mut().flatten() {
            if slot.path.is_empty() {
                continue;
            }
            // Revalidate [agent.pos, remaining corners..]. The corners
            // already traversed are irrelevant; the agent's current
            // position is the true start of what's left to walk.
            let start = slot.cursor.min(slot.path.len());
            let mut route = Vec::with_capacity(slot.path.len() - start + 1);
            route.push(slot.agent.pos);
            route.extend_from_slice(&slot.path[start..]);
            if !path_clear(&nav.navmesh, &nav.bsp, &route) {
                slot.invalidate_path();
            }
        }
    }

    /// Insert a new agent and return its handle.
    pub fn add_agent(&mut self, agent: Agent) -> AgentId {
        for (i, s) in self.slots.iter_mut().enumerate() {
            if s.is_none() {
                *s = Some(Slot::new(agent));
                return AgentId(i as u32);
            }
        }
        let id = self.slots.len() as u32;
        self.slots.push(Some(Slot::new(agent)));
        AgentId(id)
    }

    /// Remove an agent, returning its last snapshot. Returns `None` if
    /// the id is already gone or out of range.
    pub fn remove_agent(&mut self, id: AgentId) -> Option<Agent> {
        let slot = self.slots.get_mut(id.0 as usize)?;
        slot.take().map(|s| s.agent)
    }

    /// Borrow an agent's externally visible state.
    pub fn agent(&self, id: AgentId) -> Option<&Agent> {
        self.slots.get(id.0 as usize)?.as_ref().map(|s| &s.agent)
    }

    /// Iterate over every live agent.
    pub fn agents(&self) -> impl Iterator<Item = (AgentId, &Agent)> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.as_ref().map(|s| (AgentId(i as u32), &s.agent)))
    }

    /// Number of live (non-removed) agents.
    pub fn agent_count(&self) -> usize {
        self.slots.iter().filter(|s| s.is_some()).count()
    }

    /// Read-only access to an agent's current corridor (start →
    /// funnel-pulled corners → goal). Returns an empty slice if no path
    /// is currently planned. Useful for debug rendering.
    pub fn path(&self, id: AgentId) -> &[Vertex] {
        match self.slots.get(id.0 as usize).and_then(|s| s.as_ref()) {
            Some(s) => &s.path,
            None => &[],
        }
    }

    /// Index into [`Crowd::path`] of the corner the agent is currently
    /// steering toward. Combined with [`Agent::pos`], this lets a
    /// renderer draw the *remaining* corridor — i.e. a polyline that
    /// begins at the agent and never includes the already-traversed
    /// leg from the original start point.
    pub fn path_cursor(&self, id: AgentId) -> Option<usize> {
        self.slots
            .get(id.0 as usize)
            .and_then(|s| s.as_ref())
            .map(|s| s.cursor)
    }

    /// `true` when the agent's last replan attempt returned an error
    /// (start or goal off-mesh, or no portal wide enough for its
    /// radius). Callers can use this to release a claimed slot and
    /// pick a different one rather than waiting on an unreachable
    /// destination.
    pub fn plan_failed(&self, id: AgentId) -> bool {
        self.slots
            .get(id.0 as usize)
            .and_then(|s| s.as_ref())
            .map(|s| s.plan_failed)
            .unwrap_or(false)
    }

    // ---- mutators ---------------------------------------------------------

    /// Assign or clear the agent's goal. Always invalidates the path.
    pub fn set_goal(&mut self, id: AgentId, goal: Option<Goal>) {
        if let Some(Some(slot)) = self.slots.get_mut(id.0 as usize) {
            slot.agent.goal = goal;
            slot.invalidate_path();
            if goal.is_none() {
                slot.agent.vel = Vertex::ZERO;
            }
        }
    }

    /// Teleport the agent to `pos`. Invalidates the path.
    pub fn set_pos(&mut self, id: AgentId, pos: Vertex) {
        if let Some(Some(slot)) = self.slots.get_mut(id.0 as usize) {
            slot.agent.pos = pos;
            slot.last_pos = pos;
            slot.invalidate_path();
        }
    }

    /// Change the agent's body radius. Invalidates the path because the
    /// planner uses radius as wall clearance.
    pub fn set_radius(&mut self, id: AgentId, radius: f64) {
        if let Some(Some(slot)) = self.slots.get_mut(id.0 as usize) {
            slot.agent.radius = radius;
            slot.invalidate_path();
        }
    }

    /// Change the agent's max speed. Cheap; does not invalidate the path.
    pub fn set_max_speed(&mut self, id: AgentId, max_speed: f64) {
        if let Some(Some(slot)) = self.slots.get_mut(id.0 as usize) {
            slot.agent.max_speed = max_speed;
        }
    }

    /// Set the agent's right-of-way. See [`Agent::priority`]. Cheap;
    /// does not invalidate the path.
    pub fn set_priority(&mut self, id: AgentId, priority: f32) {
        if let Some(Some(slot)) = self.slots.get_mut(id.0 as usize) {
            slot.agent.priority = priority;
        }
    }

    // ---- main tick --------------------------------------------------------

    /// Advance the simulation by `dt` seconds.
    pub fn tick(&mut self, dt: f64) {
        self.replan_and_arrive();
        self.rebuild_hash();
        self.choose_all_velocities();
        self.integrate(dt);
    }

    fn replan_and_arrive(&mut self) {
        let nav = self.nav.clone();
        for slot_opt in self.slots.iter_mut() {
            let Some(slot) = slot_opt else {
                continue;
            };

            // Defensive snap: if avoidance or a navmesh swap left the
            // agent off-mesh, pull it to the nearest mesh point so
            // planning can succeed. Without this, find_path returns
            // StartOutsideMesh and the agent latches in place.
            if nav.bsp.locate(&nav.navmesh, slot.agent.pos).is_none() {
                if let Some(n) = nav.bsp.nearest(&nav.navmesh, slot.agent.pos) {
                    slot.agent.pos = n.point;
                    slot.last_pos = n.point;
                    // Old path is now misleading; force a rebuild.
                    slot.path.clear();
                    slot.cursor = 0;
                }
            }

            let Some(goal) = slot.agent.goal else {
                slot.path.clear();
                slot.cursor = 0;
                slot.agent.vel = Vertex::ZERO;
                continue;
            };
            if slot.agent.pos.distance(goal.target) <= goal.arrive_radius {
                slot.agent.goal = None;
                slot.path.clear();
                slot.cursor = 0;
                slot.agent.vel = Vertex::ZERO;
                continue;
            }

            // Replan when: we have no path yet, or we've been stuck long
            // enough. `plan_failed` does NOT latch — after `stuck_ticks`
            // ticks of no progress we always retry, in case the world
            // (or our position) has changed in our favor.
            let needs_replan =
                slot.path.is_empty() || slot.stuck >= self.config.stuck_ticks;
            if needs_replan {
                let opts = PathOptions {
                    distance_from_wall: slot.agent.radius,
                };
                match find_path(&nav.navmesh, &nav.bsp, slot.agent.pos, goal.target, &opts) {
                    Ok(p) => {
                        slot.path = p.points;
                        slot.cursor = if slot.path.len() >= 2 { 1 } else { 0 };
                        slot.stuck = 0;
                        slot.plan_failed = false;
                    }
                    Err(_) => {
                        slot.plan_failed = true;
                        slot.path.clear();
                        slot.cursor = 0;
                        slot.stuck = 0; // reset so we wait another window before retrying
                    }
                }
            }
        }
    }

    fn rebuild_hash(&mut self) {
        self.hash.clear();
        for (i, slot_opt) in self.slots.iter().enumerate() {
            if let Some(slot) = slot_opt {
                self.hash.insert(i, slot.agent.pos);
            }
        }
    }

    fn choose_all_velocities(&mut self) {
        let n = self.slots.len();
        self.next_vels.clear();
        self.next_vels.resize(n, Vertex::ZERO);
        for i in 0..n {
            if let Some(slot) = self.slots.get(i).and_then(|s| s.as_ref()) {
                self.next_vels[i] = self.choose_velocity(i, slot);
            }
        }
    }

    fn integrate(&mut self, dt: f64) {
        let arrive_eps = self.config.arrive_eps;
        let nav = self.nav.clone();
        for (i, slot_opt) in self.slots.iter_mut().enumerate() {
            let Some(slot) = slot_opt else {
                continue;
            };
            let v = self.next_vels[i];
            slot.agent.vel = v;
            let proposed = slot.agent.pos + v * dt;

            // Clamp to the navmesh: if avoidance would push the agent
            // off-mesh, snap to the nearest valid point. This keeps the
            // invariant "agents are always on (or on the boundary of)
            // the navmesh" and stops them from getting visually trapped
            // outside buildings.
            slot.agent.pos = match nav.bsp.locate(&nav.navmesh, proposed) {
                Some(_) => proposed,
                None => nav
                    .bsp
                    .nearest(&nav.navmesh, proposed)
                    .map(|n| n.point)
                    .unwrap_or(slot.agent.pos),
            };

            while slot.cursor < slot.path.len() {
                let target = slot.path[slot.cursor];
                if slot.agent.pos.distance(target) <= arrive_eps {
                    slot.cursor += 1;
                } else {
                    break;
                }
            }

            let moved = slot.agent.pos.distance(slot.last_pos);
            let expected = slot.agent.max_speed * dt * 0.1;
            if slot.agent.goal.is_some() && moved < expected {
                slot.stuck = slot.stuck.saturating_add(1);
            } else {
                slot.stuck = 0;
            }
            slot.last_pos = slot.agent.pos;
        }
    }

    // ---- per-agent velocity choice ---------------------------------------

    fn choose_velocity(&self, idx: usize, slot: &Slot) -> Vertex {
        // An agent with no reachable goal still participates in
        // collision avoidance: it holds position but yields when a
        // neighbor pushes in, so a parked agent (idle, waiting for a
        // slot, mid-deposit) never becomes an immovable wall. That wall
        // is what turns ordinary drop-off congestion into a hard
        // deadlock.
        if slot.agent.goal.is_none() || slot.plan_failed {
            return self.choose_hold_velocity(idx, slot);
        }
        let v_pref = self.preferred_velocity(slot);
        if v_pref.length_sq() < 1e-12 {
            return self.choose_hold_velocity(idx, slot);
        }
        let max_speed = slot.agent.max_speed.max(1e-6);
        let pref_norm_sq = max_speed * max_speed;

        let mut best = v_pref;
        let mut best_score = f64::NEG_INFINITY;

        let mut consider = |v: Vertex,
                            avoid_w: f64,
                            align_w: f64,
                            ttc_penalty: &dyn Fn(Vertex) -> f64| {
            let align = v.dot(v_pref) / pref_norm_sq;
            let pen = ttc_penalty(v);
            let score = align_w * align - avoid_w * pen;
            if score > best_score {
                best_score = score;
                best = v;
            }
        };

        let penalty = |v: Vertex| self.collision_penalty(idx, slot, v);
        let avoid_w = self.config.avoid_weight;
        let align_w = self.config.align_weight;

        // Zero-velocity (brake) sample.
        consider(Vertex::ZERO, avoid_w, align_w, &penalty);

        let n = self.config.vo_samples.max(2) as usize;
        let half = n / 2;
        let pref_dir_len = v_pref.length();
        let inv = 1.0 / pref_dir_len;
        let cos0 = v_pref.x * inv;
        let sin0 = v_pref.y * inv;
        let max_angle = std::f64::consts::PI;

        for k in 0..=half {
            let frac = if half == 0 { 0.0 } else { k as f64 / half as f64 };
            let theta = max_angle * frac;
            let (sin_t, cos_t) = theta.sin_cos();
            let cx_p = cos0 * cos_t - sin0 * sin_t;
            let cy_p = sin0 * cos_t + cos0 * sin_t;
            consider(
                Vertex::new(cx_p * max_speed, cy_p * max_speed),
                avoid_w,
                align_w,
                &penalty,
            );
            if k > 0 && k < half {
                let cx_m = cos0 * cos_t + sin0 * sin_t;
                let cy_m = sin0 * cos_t - cos0 * sin_t;
                consider(
                    Vertex::new(cx_m * max_speed, cy_m * max_speed),
                    avoid_w,
                    align_w,
                    &penalty,
                );
            }
        }

        best
    }

    /// Velocity for an agent with no (reachable) goal.
    ///
    /// Holds position when nothing is pressing on it, but stays "soft":
    ///
    /// 1. If it currently overlaps neighbors, it moves directly along
    ///    the penetration-depth-weighted separation vector — a pile of
    ///    parked agents decompresses instead of staying jammed.
    /// 2. Otherwise, if a neighbor's motion will collide with it soon,
    ///    it takes the lowest-penalty nudge from a full circle of
    ///    candidate velocities.
    /// 3. Otherwise it stays put.
    ///
    /// A parked agent therefore yields to pressure rather than acting
    /// as an immovable wall, while still settling to a stop once the
    /// crowd around it clears.
    fn choose_hold_velocity(&self, idx: usize, slot: &Slot) -> Vertex {
        let nudge = slot.agent.max_speed * self.config.hold_speed_frac;
        if nudge <= 0.0 {
            return Vertex::ZERO;
        }
        let me_pos = slot.agent.pos;
        let me_r = slot.agent.radius;
        let search = self.config.neighbor_radius + me_r;

        // (1) Direct decompression out of any current overlaps.
        let mut push = Vertex::ZERO;
        self.hash.for_neighbors(me_pos, search, |j| {
            if j == idx {
                return;
            }
            let Some(Some(other)) = self.slots.get(j) else {
                return;
            };
            let away = me_pos - other.agent.pos;
            let dist = away.length();
            let r = me_r + other.agent.radius;
            if dist < r {
                let depth = r - dist;
                let dir = if dist > 1e-6 {
                    away * (1.0 / dist)
                } else {
                    // Exactly coincident — fan agents apart by index
                    // (golden angle) so they don't pick the same way.
                    let a = idx as f64 * 2.399_963_229_728_653;
                    Vertex::new(a.cos(), a.sin())
                };
                push = push + dir * depth;
            }
        });
        if push.length_sq() > 1e-12 {
            return push.normalize_or_zero() * nudge;
        }

        // (2) Not overlapping: hold unless a neighbor is closing in.
        let zero_pen = self.collision_penalty(idx, slot, Vertex::ZERO);
        if zero_pen <= 0.0 {
            return Vertex::ZERO;
        }
        let n = self.config.vo_samples.max(4) as usize;
        let mut best = Vertex::ZERO;
        let mut best_pen = zero_pen;
        for k in 0..n {
            let theta = std::f64::consts::TAU * (k as f64) / (n as f64);
            let (sin, cos) = theta.sin_cos();
            let v = Vertex::new(cos * nudge, sin * nudge);
            let pen = self.collision_penalty(idx, slot, v);
            if pen < best_pen {
                best_pen = pen;
                best = v;
            }
        }
        best
    }

    fn preferred_velocity(&self, slot: &Slot) -> Vertex {
        let Some(goal) = slot.agent.goal else {
            return Vertex::ZERO;
        };
        let target = if slot.cursor < slot.path.len() {
            slot.path[slot.cursor]
        } else {
            goal.target
        };
        let dir = (target - slot.agent.pos).normalize_or_zero();
        dir * slot.agent.max_speed
    }

    /// Compute a [0, 1] collision penalty for trying velocity `v` from
    /// agent `idx`. Worst (largest) penalty across all neighbors in
    /// `neighbor_radius` is returned. Penalty of 1.0 ⇒ already
    /// overlapping; 0.0 ⇒ no collision predicted within `time_horizon`.
    fn collision_penalty(&self, idx: usize, slot: &Slot, v: Vertex) -> f64 {
        let me_pos = slot.agent.pos;
        let me_r = slot.agent.radius;
        let horizon = self.config.time_horizon.max(1e-6);
        let search = self.config.neighbor_radius + me_r;

        let mut worst: f64 = 0.0;
        self.hash.for_neighbors(me_pos, search, |j| {
            if j == idx {
                return;
            }
            let Some(Some(other)) = self.slots.get(j) else {
                return;
            };
            // Frame where j is stationary: I'm at d = me - j moving at
            // v - vel_j. Collision when |d + (v - vel_j) * t| <= r_sum.
            let d = me_pos - other.agent.pos;
            let v_rel = v - other.agent.vel;
            let r = me_r + other.agent.radius;
            let c = d.dot(d) - r * r;
            if c <= 0.0 {
                worst = 1.0;
                return;
            }
            let a = v_rel.dot(v_rel);
            if a <= 1e-12 {
                return;
            }
            let b = 2.0 * d.dot(v_rel);
            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return;
            }
            let t = (-b - disc.sqrt()) / (2.0 * a);
            if t > 0.0 && t < horizon {
                // Predicted (not yet overlapping) collision: scale by
                // relative priority so a lower-priority agent yields
                // harder and a higher-priority one holds its line.
                let pen = (1.0 - t / horizon)
                    * priority_factor(slot.agent.priority, other.agent.priority);
                if pen > worst {
                    worst = pen;
                }
            }
        });
        worst
    }
}

/// Per-neighbor avoidance scaling derived from the two agents'
/// [`Agent::priority`] values.
///
/// `diff = other - me`. At `diff == 0` the factor is exactly `1.0`, so
/// a crowd that never touches `priority` is unaffected. A neighbor that
/// outranks me (`diff > 0`) pushes the factor above 1 — I yield harder;
/// a neighbor I outrank (`diff < 0`) pulls it below 1 — I hold my line
/// and expect them to step aside. The gap is clamped to ±4, bounding
/// the factor to `[0.25, 4.0]`.
fn priority_factor(me: f32, other: f32) -> f64 {
    let diff = (other - me).clamp(-4.0, 4.0) as f64;
    2.0_f64.powf(diff * 0.5)
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
    use rsnav_polygon_extract::Bitfield;

    fn open_arena(w: u32, h: u32) -> Arc<NavBuild> {
        let data = vec![true; (w as usize) * (h as usize)];
        let bf = Bitfield::new(w, h, data).expect("dims match data");
        Arc::new(
            build_navmesh_from_bitfield(&bf, &BuildOptions::default())
                .expect("walkable arena builds"),
        )
    }

    /// Open arena with one unwalkable rectangle (inclusive cell rect).
    fn arena_with_block(w: u32, h: u32, block: (u32, u32, u32, u32)) -> Arc<NavBuild> {
        let (c0, r0, c1, r1) = block;
        let mut data = vec![true; (w as usize) * (h as usize)];
        for row in r0..=r1 {
            for col in c0..=c1 {
                data[(row * w + col) as usize] = false;
            }
        }
        let bf = Bitfield::new(w, h, data).expect("dims match data");
        Arc::new(
            build_navmesh_from_bitfield(&bf, &BuildOptions::default())
                .expect("arena with a block builds"),
        )
    }

    #[test]
    fn single_agent_walks_to_goal_on_open_arena() {
        let nav = open_arena(20, 20);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        let id = crowd.add_agent(Agent::new(Vertex::new(2.0, 10.0), 0.3, 2.0));
        crowd.set_goal(
            id,
            Some(Goal {
                target: Vertex::new(18.0, 10.0),
                arrive_radius: 0.5,
            }),
        );
        for _ in 0..1_200 {
            crowd.tick(1.0 / 60.0);
            if crowd.agent(id).unwrap().goal.is_none() {
                break;
            }
        }
        let a = crowd.agent(id).unwrap();
        assert!(a.goal.is_none(), "agent didn't arrive; ended at {:?}", a.pos);
    }

    #[test]
    fn two_agents_head_on_pass_without_collision() {
        let nav = open_arena(24, 10);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        // Small y-offset to break perfect head-on symmetry.
        let a = crowd.add_agent(Agent::new(Vertex::new(3.0, 5.05), 0.4, 1.5));
        let b = crowd.add_agent(Agent::new(Vertex::new(21.0, 4.95), 0.4, 1.5));
        crowd.set_goal(
            a,
            Some(Goal {
                target: Vertex::new(21.0, 5.0),
                arrive_radius: 0.6,
            }),
        );
        crowd.set_goal(
            b,
            Some(Goal {
                target: Vertex::new(3.0, 5.0),
                arrive_radius: 0.6,
            }),
        );

        let sum_r = 0.8;
        let mut min_dist = f64::INFINITY;
        let mut steps = 0;
        for _ in 0..1_500 {
            crowd.tick(1.0 / 60.0);
            let pa = crowd.agent(a).unwrap();
            let pb = crowd.agent(b).unwrap();
            let d = pa.pos.distance(pb.pos);
            if d < min_dist {
                min_dist = d;
            }
            steps += 1;
            if pa.goal.is_none() && pb.goal.is_none() {
                break;
            }
        }

        // No appreciable disc overlap during the encounter.
        assert!(
            min_dist >= sum_r - 0.05,
            "agents collided: min_dist={:.3} sum_r={:.3} (after {} ticks)",
            min_dist,
            sum_r,
            steps,
        );
        assert!(
            crowd.agent(a).unwrap().goal.is_none(),
            "agent A didn't reach its goal (pos={:?})",
            crowd.agent(a).unwrap().pos,
        );
        assert!(
            crowd.agent(b).unwrap().goal.is_none(),
            "agent B didn't reach its goal (pos={:?})",
            crowd.agent(b).unwrap().pos,
        );
    }

    #[test]
    fn parked_agents_decompress() {
        // Two goal-less agents placed overlapping must push apart —
        // a parked agent stays "soft", it does not freeze into a wall.
        let nav = open_arena(16, 16);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        let a = crowd.add_agent(Agent::new(Vertex::new(8.0, 8.0), 0.5, 2.0));
        let b = crowd.add_agent(Agent::new(Vertex::new(8.4, 8.0), 0.5, 2.0));
        // Neither agent is ever given a goal.
        for _ in 0..600 {
            crowd.tick(1.0 / 60.0);
        }
        let pa = crowd.agent(a).unwrap().pos;
        let pb = crowd.agent(b).unwrap().pos;
        let sum_r = 1.0;
        assert!(
            pa.distance(pb) >= sum_r - 0.05,
            "parked agents stayed overlapped: dist={:.3}",
            pa.distance(pb),
        );
    }

    #[test]
    fn idle_agent_stays_put() {
        let nav = open_arena(8, 8);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        let id = crowd.add_agent(Agent::new(Vertex::new(4.0, 4.0), 0.3, 2.0));
        let start = crowd.agent(id).unwrap().pos;
        for _ in 0..120 {
            crowd.tick(1.0 / 60.0);
        }
        let end = crowd.agent(id).unwrap().pos;
        assert!(start.approx_eq(end, 1e-9), "idle agent drifted: {:?}", end);
    }

    #[test]
    fn set_nav_keeps_still_valid_paths() {
        // Swapping to an identical-topology build should NOT invalidate
        // a corridor whose waypoints and goal still locate on the mesh.
        let nav = open_arena(12, 12);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        let id = crowd.add_agent(Agent::new(Vertex::new(2.0, 6.0), 0.3, 2.0));
        crowd.set_goal(
            id,
            Some(Goal {
                target: Vertex::new(10.0, 6.0),
                arrive_radius: 0.5,
            }),
        );
        crowd.tick(1.0 / 60.0);
        let path_len_before = crowd.path(id).len();
        assert!(path_len_before >= 2);

        crowd.set_nav(open_arena(12, 12));
        assert_eq!(
            crowd.path(id).len(),
            path_len_before,
            "identical mesh swap should preserve the corridor",
        );
    }

    #[test]
    fn set_nav_clears_paths_whose_goal_left_the_mesh() {
        // Swapping to a build where the goal is outside the new mesh
        // bounds should clear the path so the agent replans (and the
        // demo can pick a fresh goal).
        let nav = open_arena(20, 20);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        let id = crowd.add_agent(Agent::new(Vertex::new(2.0, 10.0), 0.3, 2.0));
        crowd.set_goal(
            id,
            Some(Goal {
                target: Vertex::new(18.0, 10.0),
                arrive_radius: 0.5,
            }),
        );
        crowd.tick(1.0 / 60.0);
        assert!(!crowd.path(id).is_empty());

        // Shrink the world to 8×8: the previous goal at (18, 10) is now
        // well outside.
        crowd.set_nav(open_arena(8, 8));
        assert!(
            crowd.path(id).is_empty(),
            "path with off-mesh goal should be invalidated",
        );
    }

    #[test]
    fn set_nav_invalidates_path_blocked_mid_segment() {
        // The hard case: a new obstacle lands *between* two corridor
        // corners that both still locate on-mesh. A corner-only check
        // would keep the path and walk the agent through the obstacle;
        // segment line-of-sight must catch it.
        let nav = open_arena(20, 8);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());
        let id = crowd.add_agent(Agent::new(Vertex::new(2.0, 4.0), 0.3, 2.0));
        crowd.set_goal(
            id,
            Some(Goal {
                target: Vertex::new(18.0, 4.0),
                arrive_radius: 0.5,
            }),
        );
        crowd.tick(1.0 / 60.0);
        assert!(!crowd.path(id).is_empty(), "path should be planned");

        // Block cols 9..=11, rows 2..=5 — straddles the straight leg
        // from (2, 4) to (18, 4); both endpoints stay walkable.
        crowd.set_nav(arena_with_block(20, 8, (9, 2, 11, 5)));
        assert!(
            crowd.path(id).is_empty(),
            "path crossing the freshly-spawned block should be invalidated",
        );
    }

    #[test]
    fn priority_factor_is_neutral_at_equal_priority() {
        assert!((priority_factor(0.0, 0.0) - 1.0).abs() < 1e-12);
        assert!((priority_factor(2.5, 2.5) - 1.0).abs() < 1e-12);
        // A neighbor that outranks me ⇒ I yield harder ⇒ factor > 1.
        assert!(priority_factor(0.0, 2.0) > 1.0);
        // A neighbor I outrank ⇒ I hold ⇒ factor < 1.
        assert!(priority_factor(2.0, 0.0) < 1.0);
        // Clamped to [0.25, 4.0].
        assert!(priority_factor(0.0, 100.0) <= 4.0 + 1e-9);
        assert!(priority_factor(100.0, 0.0) >= 0.25 - 1e-9);
    }

    #[test]
    fn higher_priority_agent_yields_less() {
        // Head-on pass with a tiny y-offset to fix which side each takes.
        // The high-priority agent should hold a straighter line; the
        // low-priority one does most of the avoiding and so travels a
        // longer total path.
        let nav = open_arena(28, 10);
        let mut crowd = Crowd::new(nav, CrowdConfig::default());

        let mut hi = Agent::new(Vertex::new(4.0, 5.05), 0.4, 1.6);
        hi.priority = 3.0;
        let mut lo = Agent::new(Vertex::new(24.0, 4.95), 0.4, 1.6);
        lo.priority = 0.0;
        let hi = crowd.add_agent(hi);
        let lo = crowd.add_agent(lo);
        crowd.set_goal(
            hi,
            Some(Goal { target: Vertex::new(24.0, 5.0), arrive_radius: 0.6 }),
        );
        crowd.set_goal(
            lo,
            Some(Goal { target: Vertex::new(4.0, 5.0), arrive_radius: 0.6 }),
        );

        let mut prev_hi = crowd.agent(hi).unwrap().pos;
        let mut prev_lo = crowd.agent(lo).unwrap().pos;
        let mut dist_hi = 0.0;
        let mut dist_lo = 0.0;
        let mut min_dist = f64::INFINITY;
        for _ in 0..2_000 {
            crowd.tick(1.0 / 60.0);
            let a = crowd.agent(hi).unwrap();
            let b = crowd.agent(lo).unwrap();
            dist_hi += a.pos.distance(prev_hi);
            dist_lo += b.pos.distance(prev_lo);
            prev_hi = a.pos;
            prev_lo = b.pos;
            min_dist = min_dist.min(a.pos.distance(b.pos));
            if a.goal.is_none() && b.goal.is_none() {
                break;
            }
        }

        assert!(min_dist >= 0.8 - 0.05, "agents collided: min_dist={min_dist:.3}");
        assert!(crowd.agent(hi).unwrap().goal.is_none(), "hi didn't arrive");
        assert!(crowd.agent(lo).unwrap().goal.is_none(), "lo didn't arrive");
        assert!(
            dist_lo > dist_hi,
            "low-priority agent should travel farther (it yields more): \
             lo={dist_lo:.2} hi={dist_hi:.2}",
        );
    }
}
