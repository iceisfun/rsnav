# rsnav2 — Skill Reference

Pure-Rust 2D navigation stack: constrained Delaunay triangulator (Shewchuk
*Triangle* port) plus a navmesh runtime — A* + funnel pathing, BVH point
queries, line of sight, visibility region, and a steering-target path
follower. No FFI, no C deps, `f64` throughout, `#![forbid(unsafe_code)]`
on every runtime crate.

This file is for AI assistants integrating against the crates. It covers
the public API surface, the standard data flow, idiomatic recipes, and
the pitfalls you can't infer from signatures.

---

## When to reach for this stack

Use rsnav2 when you need any of:

- A constrained Delaunay triangulation of a planar straight-line graph
  (PSLG) — outer ring(s), holes, internal walls.
- A bitfield → polygons converter (true/false grid → outer + holes), e.g.
  for converting a tile or occupancy grid into a navmesh input.
- A serializable runtime navmesh with O(log n) point-location, A* +
  funnel pathing with optional wall clearance, line of sight, and
  visibility-region queries.
- A small steering helper that turns a polyline into a lookahead target
  for an agent and resists corner shortcutting.

Not in scope for v1:

- Steiner-point quality refinement (Triangle's `-q`). No facility for it.
- Self-intersecting PSLG segments (Triangle's `segmentintersection`). The
  segment-insertion functions return `Err(SegmentInsertError::SelfIntersection)`
  instead — see [Pitfalls](#pitfalls).
- Conforming-Delaunay midpoint splitting (Triangle's `conformingedge`).

---

## Workspace layout

```
crates/
  common/          rsnav-common          shared geometry primitives
  triangle/        rsnav-triangle        CDT builder (Shewchuk port)
  polygon-extract/ rsnav-polygon-extract bitfield -> polygons + holes
  navmesh/         rsnav-navmesh         runtime mesh + binary format
  bsp/             rsnav-bsp             BVH over a NavMesh
  navigation/      rsnav-navigation      A* + funnel + LOS + visibility
  pathing/         rsnav-pathing         polyline follower (no nav dep)
  dynamic/         rsnav-dynamic         background-thread navmesh worker
                                         (Bitfield -> NavMesh) + typed
                                         telemetry events + stats counters
  demo/            rsnav-demo            egui authoring + probing app
  fixtures/        rsnav-fixtures        CLI runner for JSON fixtures
  rtsim/           rsnav-rtsim           RTS-style dynamic-obstacles testbed
```

Every library crate ships a runnable example in `crates/<name>/examples/`.
Run `cargo run -p <crate> --example <name>`.

---

## The pipeline

```
  Bitfield                                     PSLG
  --------                                     ----
  rsnav-polygon-extract::extract  ────────►   Pslg { vertices, segments, holes }
                                                       │
                                                       ▼
                                              CdtMesh (rsnav-triangle)
                                                       │
                                              delaunay(cdt, ..)
                                              form_skeleton(cdt, pslg, ..)
                                              carve_holes(cdt, pslg, ..)
                                                       │
                                                       ▼
                                              build_from_cdt(cdt)  ──► NavMesh
                                                       │
                                              Bsp::build(nav)      ──► Bsp
                                                       │
                                                       ▼
                                              find_path / line_of_sight /
                                              visibility_region / nearest_point
                                                       │
                                                       ▼
                                              PathFollower (rsnav-pathing)
```

You always go in this order. `form_skeleton` requires the Delaunay
triangulation to already be there; `carve_holes` requires the segments
to already be inserted; `build_from_cdt` requires both. Skipping any
step yields a mesh with garbage region IDs or an empty walkable area.

---

## Core types (cheat sheet)

All under `rsnav_common`:

| Type | Notes |
| --- | --- |
| `Vertex { x: f64, y: f64 }` | 2D point. `+ - * /`, `dot`, `cross`, `distance`, `normalize_or_zero`, `lerp`. |
| `Aabb { min, max }` | `from_points`, `contains`, `intersects`, `union`, `width/height/center`. |
| `Polygon { vertices: Vec<Vertex> }` | `signed_area`, `winding`, `contains`, `ensure_winding`, `remove_collinear`, `interior_point` (use this — not centroid! — for hole seeds on concave shapes). |
| `PolygonWithHoles { outer, holes }` | `contains`, `area`, `aabb`. |
| `Triangle { v: [VertexId; 3] }` | `positions`, `signed_area`, `centroid`, `contains`, `barycentric`. |
| `Mesh2d { vertices, triangles }` | Unstructured CDT output; usually go through `NavMesh` instead. |
| `VertexId(u32) / TriangleId(u32) / PolygonId(u32)` | Newtype indices; `INVALID = u32::MAX`; `.index()` for `usize`. |
| `geom::{orient2d, incircle, signed_area2, segments_intersect, nearest_point_on_segment}` | Non-robust predicate helpers. Robust versions live in `rsnav_triangle::predicates`. |

PSLG (the CDT input), `rsnav_triangle::pslg`:

| Type | Notes |
| --- | --- |
| `Pslg { vertices, segments, holes }` | Mutable; build it manually. `form_skeleton` auto-handles bit-exact duplicate positions, so a separate `deduplicate()` pass is no longer required. `deduplicate()` still exists if you want a canonicalised PSLG for some other reason (caching, hashing). |
| `PslgVertex { position, attributes, marker }` | `marker == 0` = unmarked. |
| `PslgSegment { a, b, marker }` | `a`/`b` are u32 indices into `Pslg::vertices`. |
| `PslgHole { point }` | Seed point inside the hole — the carve flood-fills from here. |

Runtime mesh, `rsnav_navmesh`:

| Type | Notes |
| --- | --- |
| `NavMesh { vertices, triangles, aabb, region_count }` | Flat parallel arrays. `to_bytes`/`from_bytes` round-trip exact. Use `nav.reachable(a, b)` as a cheap O(1) "do A* at all?" pre-check. |
| `NavTriangle { vertices, neighbors, edge_markers, area, centroid, region }` | CCW order. `neighbors[i] == TriangleId::INVALID` ⇒ boundary. `edge_markers[i] != 0` ⇒ constrained edge (the PSLG marker is preserved). `region` is the connected component under "non-wall neighbor". `edge_vertices(i)`, `is_edge_constrained(i)`, `is_edge_boundary(i)`. |

BVH, `rsnav_bsp`:

| Type | Notes |
| --- | --- |
| `Bsp` | `Bsp::build(&NavMesh)` is `O(n log n)`; queries are average `O(log n)`. Cheap to rebuild; rebuild whenever the underlying NavMesh changes. |
| `Bsp::locate(&nav, p) -> Option<TriangleId>` | Point-in-mesh. `None` outside the mesh / inside a hole. |
| `Bsp::nearest(&nav, p) -> Option<Nearest>` | Snap to nearest surface point. Always succeeds for non-empty mesh. |
| `Nearest { triangle, point, distance }` | Distance is 0 when `p` is interior. |

Pathing + queries, `rsnav_navigation`:

| Type / fn | Notes |
| --- | --- |
| `find_path(&nav, &bsp, start, goal, &PathOptions) -> Result<PathResult, PathError>` | A* + funnel. `start`/`goal` must already be inside the mesh — `nearest_point` first if you might be off-mesh. |
| `PathOptions { distance_from_wall }` | `0.0` = point agent. `> 0`: A* rejects portals shorter than this, and funnel pulls portal endpoints on wall vertices inward by this amount. Models an agent radius. |
| `PathResult { points: Vec<Vertex>, triangles: Vec<TriangleId> }` | Polyline includes `start` and `goal`; `triangles` is the A* corridor. |
| `PathError::{StartOutsideMesh, GoalOutsideMesh, Unreachable}` | `Unreachable` covers both "different region" and "every connecting portal too narrow". |
| `line_of_sight(&nav, start_tri, from, to) -> LineOfSightResult` | Walks the segment triangle-by-triangle. `start_tri` must contain `from`. Returns `Clear`, `Blocked { point }`, or `SourceOutsideMesh`. |
| `nearest_point(&nav, &bsp, p) -> Option<NearestPoint>` | Convenience wrapper over `Bsp::nearest`. |
| `visibility_region(&nav, &bsp, source, max_radius, samples) -> Option<VisibilityRegion>` | Ray-cast `samples` directions (clamped ≥ 8; 180 is a good default). Boundary is in CCW angular order; draw as a fan from `source`. |

Polyline follower, `rsnav_pathing` (zero dependency on the navmesh):

| Type / fn | Notes |
| --- | --- |
| `PathFollower::new(points: Vec<Vertex>) -> Result<Self, PathFollowerError>` | Returns `Err(EmptyPath)` on empty input. Owns the path; arc-length tracked internally. |
| `FollowerOptions { lookahead, corner_avoidance, corner_angle_threshold }` | `corner_avoidance = 0.0` disables anti-shortcut bias. Threshold in radians (~0.1 = 5.7°). |
| `target(agent_pos, &opts) -> Vertex` | Projects agent forward (monotone — never backtracks), returns lookahead steering target with optional corner bias. |
| `progress()`, `arc_length()`, `at_end()`, `total_length()` | State accessors. |

Bitfield → polygons, `rsnav_polygon_extract`:

| Type / fn | Notes |
| --- | --- |
| `Bitfield { width, height, data: Vec<bool> }` | Row-major, `true` = walkable. Cell (col, row) covers `[col, col+1] × [row, row+1]` with y-up (row 0 at bottom). Construct via `Bitfield::new(w, h, data) -> Result<Self, BitfieldError>` (returns `BadDataLength` if `data.len() != w * h`) or the infallible `Bitfield::empty(w, h)`. |
| `ExtractOptions { min_area, remove_collinear, diagonal_smoothing }` | Defaults: keep all, strip collinear vertices, no smoothing. |
| `extract(&bits, &opts) -> Vec<PolygonWithHoles>` | Outer rings CCW, holes CW. 4-connectivity (diagonal-only touch = disconnected). |

Dynamic obstacles + telemetry, `rsnav_dynamic`:

| Type / fn | Notes |
| --- | --- |
| `NavWorker` | Owns a background thread that turns `Arc<Bitfield>` snapshots into `Arc<NavBuild>`. `spawn(BuildOptions)` for no-telemetry; `spawn_with_listener(opts, Arc<dyn NavListener>)` for typed events. `Drop` joins the thread cleanly; `shutdown()` joins explicitly. |
| `BuildOptions { extract: ExtractOptions, perimeter_marker, hole_marker }` | Knobs forwarded to the per-snapshot pipeline. Defaults: extract defaults, marker 1 / 2. |
| `NavBuild { navmesh, bsp, build_ms, generation }` | One successful build. `generation` increases monotonically per worker. The first published build is `generation = 1`. |
| `BuildError::{NoPerimeter, SegmentInsertion(SegmentInsertError), EmptyMesh}` | Why a rebuild failed. Worker keeps the previous published build intact and reports via `last_error()` / `NavEvent::BuildFailed`. |
| `submit_snapshot(Arc<Bitfield>)` | Non-blocking. If another snapshot is already queued, the worker silently keeps only the newest one (counted in `NavStats::snapshots_coalesced`). |
| `poll_swap() -> bool` | Call once per frame, before any system reads `current()`. Returns true if a newer build was atomically swapped in this call. |
| `current() -> Option<Arc<NavBuild>>` | The build presented to game systems this frame. `None` until the first build publishes. |
| `latest_published() -> Option<Arc<NavBuild>>` | The freshest build the worker has put out, regardless of `poll_swap` — useful for tests and one-shot waits. |
| `stats() -> NavStats` | Cheap snapshot of running counters. Safe every frame. |
| `last_error() -> Option<String>` | Last build's error text; cleared when a subsequent build succeeds. |
| `NavStats { snapshots_submitted, snapshots_coalesced, builds_completed, builds_failed, last_completed_generation, last_build_ms, max_build_ms, total_build_ms }` | Plain `Copy` struct. Caller derives averages itself (`total / completed`). |
| `NavEvent<'a>::{BuildStarted, BuildCompleted, BuildFailed}` | Typed events emitted by the worker. `BuildFailed` borrows the `&BuildError`; listeners that want to retain events must convert to an owned form themselves. |
| `NavListener` trait | `fn on_event(&self, event: &NavEvent<'_>)`. Send + Sync + 'static. Blanket impl for `Fn(&NavEvent<'_>)` closures — pass `Arc::new(|ev: &NavEvent<'_>\| { ... }) as Arc<dyn NavListener>`. Invoked synchronously on the worker thread; keep handlers cheap. |
| `build_navmesh_from_bitfield(&Bitfield, &BuildOptions) -> Result<NavBuild, BuildError>` | Synchronous one-shot pipeline. Same routine the worker calls internally; useful for tests, batch jobs, or any caller that doesn't want a thread. |

---

## Common recipes

### 1. Build a navmesh from a hand-coded PSLG

```rust
use rsnav_common::Vertex;
use rsnav_navmesh::build_from_cdt;
use rsnav_triangle::{
    carve_holes, delaunay, form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

let outer = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
let hole  = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)];

let mut cdt = CdtMesh::new();
let mut pslg = Pslg::new();
for (x, y) in outer.iter().chain(hole.iter()) {
    cdt.push_vertex(VertexSlot::new(Vertex::new(*x, *y), 0));
    pslg.vertices.push(PslgVertex::new(Vertex::new(*x, *y)));
}
// Outer ring (marker 1 = "wall")
for &(a, b) in &[(0u32, 1), (1, 2), (2, 3), (3, 0)] {
    pslg.segments.push(PslgSegment { a, b, marker: 1 });
}
// Inner ring around the hole (marker 2)
for &(a, b) in &[(4u32, 5), (5, 6), (6, 7), (7, 4)] {
    pslg.segments.push(PslgSegment { a, b, marker: 2 });
}
// Seed point INSIDE the hole. For concave holes, use
// Polygon::interior_point — NOT the arithmetic centroid.
pslg.holes.push(PslgHole { point: Vertex::new(5.0, 5.0) });

delaunay(&mut cdt, DivConqOptions::default());
form_skeleton(&mut cdt, &pslg, /* mark_hull_with */ None)
    .expect("PSLG is non-self-intersecting");
carve_holes(&mut cdt, &pslg, /* convex outer? */ false);
let nav = build_from_cdt(&cdt);
```

`form_skeleton` returns `Result<(), SegmentInsertError>`. On
`Err(SelfIntersection { endpoint1, endpoint2 })` the CDT is left in a
valid state — discard the bad segment from the PSLG and retry, or bail.

### 2. Build a navmesh from a bitfield (occupancy grid)

```rust
use rsnav_polygon_extract::{extract, Bitfield, ExtractOptions};
use rsnav_common::Polygon;

let bits = Bitfield::new(w, h, data).expect("data length == w * h");
let regions = extract(&bits, &ExtractOptions::default());

// Then for each region, push outer + holes into a Pslg, choose hole seed
// points via Polygon::interior_point() (NOT centroid — concave holes will
// place the centroid outside the polygon and carve the wrong region),
// and run the same delaunay → form_skeleton → carve_holes → build_from_cdt
// pipeline as in recipe 1.
let seed_for_hole = |h: &Polygon| h.interior_point()
    .expect("hole degenerate after extraction");
```

### 3. Persist a navmesh

```rust
let bytes = nav.to_bytes();              // Vec<u8>
std::fs::write("level.navmesh", &bytes)?;

let loaded = rsnav_navmesh::NavMesh::from_bytes(&bytes)?;
// Round-trip is exact: vertex/triangle order and all derived fields
// (adjacency, edge markers, regions) match what was built.
```

Format spec: `crates/navmesh/FORMAT.md`. Little-endian, no compression,
no varints, no alignment tricks. Magic `RSNAVMSH`, version `1`. Unknown
section types are skipped (forward compat).

### 4. Path search

```rust
use rsnav_bsp::Bsp;
use rsnav_navigation::{find_path, nearest_point, PathError, PathOptions};

let bsp = Bsp::build(&nav);  // O(n log n); cache for the life of the mesh

let opts = PathOptions { distance_from_wall: 0.3 }; // agent radius
let start = Vertex::new(1.0, 1.0);
let goal  = Vertex::new(9.0, 9.0);

// Off-mesh inputs are an error. If you want "snap to mesh" semantics,
// project first:
let start = nearest_point(&nav, &bsp, start).unwrap().point;
let goal  = nearest_point(&nav, &bsp, goal).unwrap().point;

match find_path(&nav, &bsp, start, goal, &opts) {
    Ok(path) => { /* path.points is the polyline; path.triangles is the corridor */ }
    Err(PathError::Unreachable)       => { /* different region or all portals too narrow */ }
    Err(PathError::StartOutsideMesh)  => { /* didn't snap, or snap failed */ }
    Err(PathError::GoalOutsideMesh)   => { /* same */ }
}
```

Cheap pre-check before any pathing: `nav.reachable(start_tri, goal_tri)`
returns `false` immediately when the two triangles' `region` IDs differ.

### 5. Line of sight

```rust
use rsnav_navigation::{line_of_sight, LineOfSightResult};

let start_tri = bsp.locate(&nav, from).unwrap();  // `from` must be in-mesh
match line_of_sight(&nav, start_tri, from, to) {
    LineOfSightResult::Clear              => { /* visible */ }
    LineOfSightResult::Blocked { point }  => { /* wall at `point` */ }
    LineOfSightResult::SourceOutsideMesh  => unreachable!("we just located"),
}
```

`line_of_sight` tolerates a `from` that lies exactly on an edge of
`start_tri` — useful when the source came from snapping or from a
visibility-region sweep landing on a triangulation vertex.

### 6. Visibility region (visibility polygon)

```rust
use rsnav_navigation::visibility_region;

let vr = visibility_region(&nav, &bsp, source, /* max_radius */ 50.0, /* samples */ 180)?;
// vr.boundary is CCW around vr.source. Draw as a triangle fan from
// source through consecutive boundary points (wrap last back to first).
```

180 samples (2° each) is a good default for hover rendering; bump higher
for tight corners or zoomed-in screenshots.

### 7. Steering an agent along a path

```rust
use rsnav_pathing::{FollowerOptions, PathFollower};

let mut follower = PathFollower::new(path.points)   // from find_path
    .expect("path is non-empty");
let opts = FollowerOptions {
    lookahead: 1.5,
    corner_avoidance: 0.4,
    corner_angle_threshold: 0.1, // radians
};

loop {
    let target = follower.target(agent_pos, &opts);
    // ... apply your steering controller toward `target` ...
    if follower.at_end() { break; }
}
```

The follower is path-only (no navmesh ref). Reuse it across multiple
agents only with care — it owns one agent's arc-length state.

### 8. Dynamic obstacles in a game loop

When the world can change while the game is running (buildings, harvested
forests, doodads spawning/despawning), keep the navmesh on a background
thread and let game systems read whatever the worker has most recently
published. The `Bitfield` is the ground truth; "add an obstacle" and
"remove an obstacle" are both just bitfield edits + a new snapshot.

```rust
use std::sync::Arc;
use rsnav_dynamic::{BuildOptions, NavWorker};
use rsnav_polygon_extract::Bitfield;

// Game-startup wiring.
let world_w = 128;
let world_h = 128;
let mut grid = vec![true; (world_w * world_h) as usize]; // all walkable
let mut worker = NavWorker::spawn(BuildOptions::default());
worker.submit_snapshot(Arc::new(
    Bitfield::new(world_w, world_h, grid.clone()).expect("dims"),
));

// When the player places a 4x4 building at (col, row):
fn paint_rect(grid: &mut [bool], w: u32, col: u32, row: u32, dw: u32, dh: u32, walkable: bool) {
    for dy in 0..dh {
        for dx in 0..dw {
            grid[((row + dy) * w + (col + dx)) as usize] = walkable;
        }
    }
}
paint_rect(&mut grid, world_w, 30, 40, 4, 4, false);
worker.submit_snapshot(Arc::new(
    Bitfield::new(world_w, world_h, grid.clone()).expect("dims"),
));

// When the building is destroyed: flip the same cells back.
paint_rect(&mut grid, world_w, 30, 40, 4, 4, true);
worker.submit_snapshot(Arc::new(
    Bitfield::new(world_w, world_h, grid.clone()).expect("dims"),
));

// In your game loop (every frame):
loop {
    if worker.poll_swap() {
        // A new build is now visible to queries. Invalidate cached
        // routes, reset agent paths, etc.
    }
    if let Some(build) = worker.current() {
        // Use build.navmesh / build.bsp for path queries this frame.
        let _ = (build.navmesh.triangle_count(), build.bsp.is_empty());
    }
    break; // (in a real game this is the frame boundary)
}
```

Submit-rate doesn't need throttling — the worker coalesces, so a burst
of edits in one frame results in at most one extra rebuild. Rebuilds
happen off-thread; `poll_swap` is the cooperative point at which game
state observes them.

### 9. Telemetry (typed events + stats)

```rust
use std::sync::Arc;
use rsnav_dynamic::{BuildOptions, NavEvent, NavListener, NavWorker};

// A closure is enough — there's a blanket NavListener impl for Fn.
let listener: Arc<dyn NavListener> = Arc::new(|ev: &NavEvent<'_>| match ev {
    NavEvent::BuildStarted { generation } => log_my_engine(format!("nav start g{generation}")),
    NavEvent::BuildCompleted { generation, build_ms, triangles, regions } => {
        log_my_engine(format!("nav done  g{generation}: {build_ms:.2}ms {triangles}t {regions}r"));
    }
    NavEvent::BuildFailed { generation, error } => {
        log_my_engine(format!("nav FAIL  g{generation}: {error}"));
    }
});
let worker = NavWorker::spawn_with_listener(BuildOptions::default(), listener);

// In your HUD or dashboard each frame:
let stats = worker.stats();
let avg_ms = if stats.builds_completed > 0 {
    stats.total_build_ms / stats.builds_completed as f64
} else { 0.0 };
my_hud.write(format!(
    "nav: {} builds  avg {:.1}ms  max {:.1}ms  coalesced {}",
    stats.builds_completed, avg_ms, stats.max_build_ms, stats.snapshots_coalesced,
));
```

A custom struct implementing `NavListener` is the right choice when the
listener needs to own state (event ring buffer, atomic counters, channel
sender). `rtsim` does this for the in-app event log — see
`crates/rtsim/src/main.rs` (`EventLog`).

---

## Gotchas and idioms

- **Use `Polygon::interior_point` for hole seed points**, not the
  arithmetic centroid. The centroid of a concave polygon (C / L / U
  shape) routinely falls outside the polygon, which causes `carve_holes`
  to flood-fill the wrong region — silently. `interior_point` runs an
  ear-find and returns a guaranteed-inside point.

- **Duplicate-position vertices are handled automatically.** The D&C
  Delaunay drops bit-exact duplicate positions, so a segment referencing
  a dropped ID would otherwise crash insertion. `form_skeleton` now
  builds a position → first-occurrence-ID remap from the mesh's vertex
  pool and rewrites segment endpoints through it before inserting. No
  separate `Pslg::deduplicate` call is required. (The standalone
  `deduplicate` method still exists if you want a canonicalised PSLG
  for some other reason.)

- **Segment insertion can fail.** `form_skeleton` returns
  `Result<(), SegmentInsertError>`. The two variants:
  - `SelfIntersection { endpoint1, endpoint2 }` — the segment would
    cross an existing constrained subsegment. v1 doesn't support self-
    intersecting PSLG input; the CDT is left in a valid state with the
    bad segment NOT inserted.
  - `VertexNotInTriangulation { vertex }` — a segment endpoint isn't a
    corner of any live triangle. With the auto-remap above this only
    fires for genuinely missing vertices — a segment endpoint not in
    the CDT input at all.

- **The pipeline order is non-negotiable.** `delaunay → form_skeleton →
  carve_holes → build_from_cdt`. Skipping `carve_holes` leaves
  concavities and hole interiors as walkable triangles; skipping
  `form_skeleton` means edge markers and region splitting won't work.

- **`build_from_cdt` re-numbers everything.** Vertex and triangle indices
  in the resulting `NavMesh` do **not** match `CdtMesh` indices. Don't
  carry CDT indices into the runtime.

- **`distance_from_wall` matters in two places.** A* rejects portals
  whose edge length is `≤ distance_from_wall`; funnel pushes portal
  endpoints that are wall vertices inward by `distance_from_wall`.
  Pass it on `PathOptions` and it's wired through both.

- **`Bsp::locate` returns `None` inside holes.** Holes are unwalkable
  carved-away regions; nothing in the navmesh exists there. Use
  `Bsp::nearest` (or `nearest_point`) for "snap to nearest mesh point".

- **`find_path` does not snap.** Off-mesh `start`/`goal` give
  `StartOutsideMesh` / `GoalOutsideMesh`. Either snap up front via
  `nearest_point`, or treat those errors as "agent is off-map" and
  handle separately.

- **Boundary check before A*:** `nav.reachable(a, b)` (O(1) region-ID
  compare) avoids running the full search across an unreachable region.

- **`Bitfield` is y-up.** Row 0 is the bottom row. Most image / tile-map
  formats are y-down — flip when you load.

- **CCW everywhere.** Outer rings, navmesh triangles, visibility-region
  boundaries: all CCW. Holes are CW. If you author polygons by hand, use
  `Polygon::ensure_winding(Winding::CounterClockwise)` (or `Clockwise`
  for holes) before handing off.

- **`#![forbid(unsafe_code)]`** on every runtime crate. No FFI surface
  to worry about.

- **Tests are authoritative.** Each module has a `#[cfg(test)] mod tests`
  with end-to-end fixtures. When unsure how an API is meant to be
  invoked, the tests in the relevant module file are the best reference.

- **`NavListener` callbacks run on the worker thread.** They fire
  synchronously between builds, before the next snapshot is processed.
  Keep handlers cheap: push to a channel, increment an atomic, format
  one line of log. If you do real work in the callback (network I/O,
  big formatting, file writes), it directly delays the next build.

- **The worker coalesces submissions.** Submitting two `Bitfield`
  snapshots while the worker is busy means only the *newest* gets built;
  the older ones are dropped and counted in `NavStats::snapshots_coalesced`.
  This is intentional — you can spam `submit_snapshot` from the main
  thread without throttling. Game systems still see *every* completed
  build through `poll_swap`, but they don't see every submission.

- **`poll_swap` is the cooperative swap point.** The worker publishes
  builds whenever they finish, but `current()` only updates when the
  game thread calls `poll_swap()`. Call it at frame start, before any
  system reads the navmesh, to guarantee every system sees the same
  build all frame.

---

## Demo and CLI

- `cargo run -p rsnav-demo --release` — egui authoring app. Right-click
  sets source, left-click sets goal, "Create navmesh" rebuilds. Left
  panel has a fixture browser; the directory field is pre-filled with
  `./testdata` and is editable.
- `cargo run -p rsnav-fixtures --release -- --testdata <PATH>` — batch
  runner over a `.json` file or a directory of them (status table, exits
  non-zero on failure; drop-in for CI). `--testdata` defaults to
  `./testdata` if omitted. Add `-v` for per-hole diagnostics.
- `cargo run -p rsnav-rtsim --release` — RTS-style testbed for the
  `NavWorker` flow. 128×128 cell bitfield, mouse tools (paint walls,
  clear, harvest forest cells one at a time), ~10 agents pathing
  between random walkable points through the live mesh. Side panel
  shows `NavStats` counters and a scrolling recent-events log via the
  typed `NavListener` API — useful for seeing the coalescing /
  in-flight / build-ms cadence interactively.

---

## Runnable examples (one per library crate)

| Crate | Example | What it shows |
| --- | --- | --- |
| `rsnav-triangle` | `triangulate_pslg` | Hand-coded PSLG, carve a hole, list live triangles. |
| `rsnav-polygon-extract` | `grid_to_polygons` | ASCII-art bitfield → polygons + holes. |
| `rsnav-navmesh` | `save_and_load` | Build → `to_bytes` → `from_bytes` → exact match. |
| `rsnav-bsp` | `locate_and_nearest` | `locate` vs `nearest` on the donut fixture. |
| `rsnav-navigation` | `find_path` | A* + funnel with and without `distance_from_wall`. |
| `rsnav-navigation` | `visibility_region` | 180-sample sweep from a room with a pillar. |
| `rsnav-pathing` | `follow_path` | L-shape walk with anti-shortcut on/off. |
| `rsnav-dynamic` | `live_worker` | Spawn a `NavWorker` with a printing `NavListener`, place a building, demolish it; print stats at the end. |

All run as `cargo run -p <crate> --example <name>`.

---

## Reading the source when in doubt

- `crates/common/src/{vertex,polygon,triangle,mesh2d,aabb,geom}.rs` —
  data primitives.
- `crates/triangle/src/lib.rs` — re-exports the user-facing surface
  (`delaunay`, `form_skeleton`, `carve_holes`, the `Pslg` types).
- `crates/navmesh/src/{navmesh,build,binary}.rs` — runtime mesh, CDT
  conversion, serialization.
- `crates/navmesh/FORMAT.md` — normative binary spec.
- `crates/navigation/src/{path,los,visibility,astar,funnel,wall}.rs` —
  pathing + queries.
- `crates/bsp/src/lib.rs` — BVH index.
- `crates/pathing/src/lib.rs` — steering follower.

Tests in each file cover the canonical use shape end-to-end and are
small enough to read top-to-bottom.
