# From polyline to movement

Prerequisites: a working `find_path` call ([01-quickstart.md](01-quickstart.md)) and the
clearance decision already made ([06-clearance.md](06-clearance.md)). This page does not
re-open that decision; it shows the runtime half of it.

`find_path` returns a `PathResult`. A `PathResult` is not movement. It is a polyline whose
first point is literally your `start` and whose last point is literally your `goal`, with
nothing in between adjusted for agent size, turn radius, or the fact that your character
accelerates. Turning that into a character requires two separate mechanisms, and most
"the agent looks wrong" bugs are a confusion between them:

- **Following a plan** — `rsnav_pathing::PathFollower` converts the polyline into a steering
  target each tick.
- **Free movement** — WASD, a steering controller, knockback, a shove from a scripted event.
  None of it goes through the planner, so `PathOptions::distance_from_wall` is not involved
  at all. `rsnav_navigation::WallClearance` is the runtime equivalent.

The runnable reference for both halves together is
[`crates/navigation/examples/walk_the_path.rs`](../crates/navigation/examples/walk_the_path.rs):

```
cargo run --release -p rsnav-navigation --example walk_the_path
```

It builds a navmesh from an ASCII bitfield, plans across it, walks the plan with a
`PathFollower` at a fixed 20 Hz step while asserting every visited position still
`Bsp::locate`s onto the mesh, then hand-nudges the same agent into a wall.

---

## PathFollower

```rust
pub fn new(points: Vec<Vertex>) -> Result<Self, PathFollowerError>
pub fn target(&mut self, agent_pos: Vertex, opts: &FollowerOptions) -> Vertex
pub fn total_length(&self) -> f64
pub fn arc_length(&self) -> f64
pub fn progress(&self) -> f64
pub fn at_end(&self) -> bool
```

([`crates/pathing/src/lib.rs:77`](../crates/pathing/src/lib.rs) onward.)

The model is arc length. The follower stores the polyline plus a cumulative-length table,
and keeps one piece of state: `arc`, the agent's projected position along the path. Each
call to `target` projects `agent_pos` onto the path, advances `arc`, and returns the point
`lookahead` arc-length further on. You steer toward that point; you do not teleport to it.

`target` takes `&mut self` — it mutates the stored arc. Call it once per agent per tick.

### Progress is monotone, and that has a sharp edge

`project_forward` finishes with `best_arc.max(self.arc)`
([`pathing/src/lib.rs:203`](../crates/pathing/src/lib.rs)). The projection can never move
backward. That is what stops a path that doubles back on itself from re-snapping the agent
to an earlier segment when the two passes come within a lookahead of each other.

The cost: an agent that *genuinely* backtracks — knocked back, rerouted by avoidance, pushed
by a script — does not regain the path it lost. Its arc stays where it was and `target`
keeps aiming forward. For a small displacement this is what you want. For a large one, the
correct response is to replan and construct a new `PathFollower`, not to hope the old one
recovers.

One related fact: `project_forward`'s search window is one segment backward and **unbounded
forward** (`for k in from..n - 1`, [`pathing/src/lib.rs:185`](../crates/pathing/src/lib.rs)),
so cost is O(remaining segments) per call — fine for a funnel polyline, worth knowing if you
feed it something much longer. Its doc comment claims a bounded "small window"; the
implementation does not do that, and the monotone `max` is what actually provides the
no-re-snap guarantee.

### FollowerOptions

```rust
pub struct FollowerOptions {
    pub lookahead: f64,
    pub corner_avoidance: f64,
    pub corner_angle_threshold: f64,
}
```

Defaults ([`pathing/src/lib.rs:36`](../crates/pathing/src/lib.rs)): `lookahead: 1.0`,
`corner_avoidance: 0.0`, `corner_angle_threshold: 0.1` radians (about 5.7 degrees). Note
that anti-shortcutting is **off** unless you ask for it.

**lookahead** is in world units and is the only knob that matters for most projects. Too
small and the agent oscillates: the target sits inside the distance it covers in one tick, so
it overshoots and corrects every frame. Too large and it cuts corners, because the target
jumps past the bend before the agent reaches it. Start around `speed * dt * 1.5`, raise it
until the motion stops jittering, lower it until corners stop being shaved. An agent with a
slow turn rate needs more than one that can pivot in place. `walk_the_path.rs` runs
`speed = 4.0`, `dt = 0.05`, `lookahead = 1.0` — five ticks of travel.

**corner_avoidance** is the anti-shortcut bias. At each interior vertex whose turn angle
exceeds `corner_angle_threshold`, the target is displaced perpendicular to the corner
bisector, away from the inside of the turn, at full `corner_avoidance` magnitude at the
corner and fading linearly to zero at `corner_avoidance` arc-distance away
([`pathing/src/lib.rs:216-260`](../crates/pathing/src/lib.rs)). The value serves two roles
at once: it is both the size of the influence window and the peak magnitude of the bias.

The contrast is easiest to see by running
[`crates/pathing/examples/follow_path.rs`](../crates/pathing/examples/follow_path.rs):

```
cargo run --release -p rsnav-pathing --example follow_path
```

On an L-shaped path with a 90-degree left turn at `(5, 0)`, step 5 with
`corner_avoidance = 0.0` gives a steering target of `(5.00, 0.00)` — dead at the corner,
which is exactly the point the agent will cut across. With `corner_avoidance = 0.8` the same
step gives `(5.56, -0.57)`: pushed outside the turn, so the agent swings wide instead of
clipping the inside wall.

**corner_angle_threshold** exists to stop the follower twitching on numerical-noise corners
along what should be a straight run. Leave it at 0.1 unless your polylines come from
somewhere other than the funnel.

### What the follower does not know

`rsnav-pathing` depends only on `rsnav-common`
([`crates/pathing/Cargo.toml`](../crates/pathing/Cargo.toml)). No navmesh, no `Bsp`, no
`WallInfo`. This is a deliberate scope choice and it has consequences you must handle:

- The corner bias **can place the steering target outside walkable space.** The follower has
  no way to detect this. It is a steering target, not a position, so a target briefly inside
  a wall is usually harmless — the agent steers toward it and gets clamped by whatever moves
  it. It stops being harmless if you teleport the agent to the target.
- The bias **accumulates across corners.** `apply_corner_avoidance` loops over every interior
  vertex and adds each in-range corner's contribution to the same target
  (`shifted = shifted + outside * ...`, [`pathing/src/lib.rs:257`](../crates/pathing/src/lib.rs)).
  On a zig-zag where several corners fall within `corner_avoidance` arc-distance, the biases
  stack and the target can be displaced by a multiple of `corner_avoidance`. Keep
  `corner_avoidance` well under the spacing between corners on your typical path.
- The window test is `(target_arc - v_arc).abs() < corner_avoidance`
  ([`pathing/src/lib.rs:226-227`](../crates/pathing/src/lib.rs)) — an *absolute* arc distance.
  A corner the agent has already passed keeps pulling the target outward until it falls out
  of range — the bias does not switch off once the turn is behind you, it fades with
  distance in both directions.

One degenerate input worth knowing: a single-point polyline is accepted. `total_length` is 0,
`at_end()` is immediately true, and `progress()` returns `0.0` because it special-cases
zero-length paths ([`pathing/src/lib.rs:110-116`](../crates/pathing/src/lib.rs)). The two
disagree. Treat `at_end()` as authoritative.

---

## Free movement: WallClearance

The funnel already holds a *planned* path off walls, subject to the limits
[06-clearance.md](06-clearance.md) states. It does nothing for movement that never went
through the planner. `WallClearance` is the free-movement analogue: it precomputes the
mesh's wall segments once, and `clamp` pushes a proposed position back out so the agent's
centre sits at least `radius` from every wall.

```rust
pub fn from_navmesh(nav: &NavMesh) -> Self
pub fn from_navmesh_with_doors(nav: &NavMesh, doors: &DoorSet) -> Self
pub fn from_walls(nav: &NavMesh, walls: &WallInfo) -> Self
pub fn segment_count(&self) -> usize
pub fn clamp(&self, pos: Vertex, radius: f64) -> Vertex
```

([`crates/navigation/src/wall_clearance.rs:94-149`](../crates/navigation/src/wall_clearance.rs).)

A "wall" here is exactly what `is_wall_edge_local` reports: a constrained edge
(`edge_markers[i] != 0`) or a boundary edge with no triangle on the far side — the same set
A* and the funnel treat as impassable — plus, when you build via `from_navmesh_with_doors`,
every edge cut by a currently-closed door (`WallInfo::is_wall_edge`,
[`wall.rs:103`](../crates/navigation/src/wall.rs)). Interior constrained edges are stored
once, not twice, via canonical-pair dedup.

`radius` is a **per-call argument**, not construction state. One `WallClearance` serves
agents of every size; you do not need a mesh per radius. `radius <= 0.0` is a no-op.

### The order rule

Snap onto the mesh, **then** push off the wall:

```rust
let on_mesh = bsp.nearest(nav, proposed).map(|n| n.point).unwrap_or(proposed);
let safe = clearance.clamp(on_mesh, agent_radius);
```

`clamp` keeps the agent's centre off walls. It does not keep the agent on the mesh, and it
cannot: it only knows about segments, not about which side of them is walkable. From
`walk_the_path.rs`, with `agent_radius = 0.4` and a wall block spanning x in [9, 13]:

```
    overshoot into the wall    (10.500, 5.500)  on_mesh=false wall_dist= 1.500 VIOLATION
      clamp alone (WRONG)      (10.500, 5.500)  on_mesh=false wall_dist= 1.500 VIOLATION
      nearest (step 1)         ( 9.000, 5.500)  on_mesh=true  wall_dist= 0.000 VIOLATION
      then clamp (step 2)      ( 8.600, 5.500)  on_mesh=true  wall_dist= 0.400 ok
```

The overshoot lands 1.5 units *inside* a wall block. That is further than `radius` from every
wall segment, so `clamp` alone considers it clear and returns it unchanged — the agent stays
inside the wall. Only `Bsp::nearest` knows the point is off the mesh. Run the two in the other
order and you end up flush on the wall face at distance 0.

Note also the first case in that output: a nudge that stays on the mesh but ends up 0.05 from
the wall is corrected to exactly 0.4. That is the ordinary path, and it is the one `clamp` is
built for.

### Costs and limits, unhedged

- **`clamp` is a linear scan over every wall segment in the mesh, run `RELAX_ITERS = 4` times,
  per call** ([`wall_clearance.rs:70`, `:155-181`](../crates/navigation/src/wall_clearance.rs)).
  There is no spatial index. The pass loop does break early as soon as a pass adjusts nothing
  (`:178-180`), so a position already clear of every wall costs one scan, not four; only a
  position actually being pushed pays up to four. On the toy map in `walk_the_path.rs` that is
  8 segments. On a town-scale mesh it is tens of thousands per scan, per agent per frame. Budget it:
  call it only for agents that actually moved outside the planner, or cache a local segment
  subset yourself.
- **It does not guarantee convergence.** Four relaxation passes settle a flat wall and the
  ordinary concave corner (pinned by the test `pushes_out_of_a_concave_corner`,
  [`wall_clearance.rs:292`](../crates/navigation/src/wall_clearance.rs) — test code, not a
  runnable example). A tight multi-wall pocket can leave the result still within `radius` of a
  wall, with no error signal. The method doc's phrase "the nearest position whose distance to
  every wall is at least `radius`" is what a fixed-iteration relaxation aims at, not what it
  promises.
- **In a corridor narrower than `2 * radius` no clear position exists.** The agent is pinned
  toward the channel centre, consistent with A* refusing such a portal.
- **Rebuild it whenever the mesh changes or any door state changes.** Build is `O(triangles)`;
  the lifecycle is the same as `Bsp` and `WallInfo`. Use `from_navmesh_with_doors` so a shut
  door holds a hand-moved agent exactly as a static wall does. `NavWorld` does **not** own a
  `WallClearance` — you maintain it alongside ([09-doors-and-navworld.md](09-doors-and-navworld.md)).

### Double counting

If the mesh was built with a baked erosion of `r`, the walls already sit `r` inside the true
geometry. Pass `max(0.0, agent_radius - r)` to `clamp`, not the raw radius, or the clearance
is applied twice. [06-clearance.md](06-clearance.md) owns that arithmetic, including the
`diagonal_smoothing` correction; the module header at
[`wall_clearance.rs:18-37`](../crates/navigation/src/wall_clearance.rs) states it too.

---

## The loop

```rust
// once per mesh
let clearance = WallClearance::from_navmesh(&nav);
let walls = WallInfo::from_navmesh(&nav);

// on a new goal. PathError implements neither Display nor std::error::Error,
// so it will not compose with `?` into a boxed error — match it (07).
let path = match find_path_with_walls(&nav, &bsp, &walls, pos, goal, &opts) {
    Ok(p) => p,
    Err(e) => return handle_no_path(e),
};
let mut follower = PathFollower::new(path.points.clone()).expect("funnel never returns empty");

// per tick
let target = follower.target(pos, &follow_opts);
let dir = (target - pos).normalize_or_zero();
let proposed = pos + dir * (speed * dt);
let on_mesh = bsp.nearest(&nav, proposed).map(|n| n.point).unwrap_or(proposed);
pos = clearance.clamp(on_mesh, effective_radius);
if follower.at_end() { /* arrived */ }
```

Four triggers should discard the follower and replan, and nothing detects them for you:

1. **The goal moved.** Cheapest and most common.
2. **`path_clear` returned false.** The remaining route — `[pos, remaining corners...]` — no
   longer survives a leg-by-leg walk. Note it is a zero-width test that ignores clearance
   entirely; see [07-paths-and-queries.md](07-paths-and-queries.md).
3. **The navmesh was swapped.** Every `TriangleId`, the `Bsp`, the `WallInfo` and the
   `WallClearance` all die with the old mesh. See
   [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md).
4. **No progress for N ticks.** Watch `follower.arc_length()`: if it has not advanced by a
   meaningful fraction of `speed * dt` for N consecutive ticks, the agent is wedged — against
   geometry, against another agent, or against the monotone-arc behaviour above. Replanning is
   the only recovery. `rsnav-crowd` implements exactly this with `CrowdConfig::stuck_ticks`
   ([11-crowds.md](11-crowds.md)).

For many agents at once, stop here and read [11-crowds.md](11-crowds.md): `Crowd` already owns
the plan/follow/replan loop plus local avoidance, and re-implementing it per agent on top of
`PathFollower` will not converge on the same behaviour.
