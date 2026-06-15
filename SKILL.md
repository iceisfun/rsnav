# rsnav — Skill Reference

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

Use rsnav when you need any of:

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
  crowd/           rsnav-crowd           multi-agent crowd primitive
                                         (per-agent corridor + sampled-VO
                                         local avoidance + spatial hash)
  demo/            rsnav-demo            egui authoring + probing app
  fixtures/        rsnav-fixtures        CLI runner for JSON fixtures
  rtsim/           rsnav-rtsim           RTS-style dynamic-obstacles testbed
  crowd-demo/      rsnav-crowd-demo      multi-agent peon-economy testbed
                                         (FSM + slot reservation + forest)
  door-demo/       rsnav-door-demo       togglable-doors testbed (bitfield
                                         obstacle doors via NavWorker rebuild)
  world-demo/      rsnav-world-demo      multi-tile world editor (place +
                                         drag tiles, stitch seams, path / LOS)
```

`rsnav-navigation` also ships the in-mesh runtime features used above:
**doors** (`DoorSet` edge-cuts + `WallInfo` oracle — no rebuild, distinct
from the bitfield-rebuild `door-demo`), **`NavWorld<M>`** (owning container +
`NavMetadata`), and **`TiledWorld`** (multi-tile worlds).

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
                                              clip_ears(cdt, max_area)   (optional)
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

`clip_ears` is an optional post-carve cleanup pass that prunes small
"ear" triangles (two wall edges + one interior neighbor) — typically
half-cell stair-step artifacts left behind when a bitfield with a
diagonal-ish boundary is triangulated. It promotes the previously-
interior edge on the surviving neighbor to a constraint (inheriting the
smaller nonzero wall marker), and iterates until no more ears qualify.
Cost: a sliver of walkable area per clipped ear, bounded by the
threshold. Skip it for hand-authored PSLGs where small ears are
intentional geometry.

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
| `NavMesh::region_triangles(id)` / `region_area(id)` / `region_centroid(id)` / `region_bounds(id)` | Per-region views over the connected-component `region` field: a `TriangleId` iterator, the summed area, the area-weighted centroid, and an `Aabb`. An out-of-range region id is graceful — empty iterator / `0.0` / `None` / `None`. |
| `NavMesh::random_point(rng)` / `random_point_in_region(region, rng)` | Uniform *area-weighted* random point inside the whole mesh / one region — ideal for spawn placement. `rng` is any `impl FnMut() -> f64` yielding `[0, 1)` (no `rand` dependency); `O(n)` per call. `None` when the (region's) area is zero. |
| `NavMesh::boundary_edges() -> impl Iterator<Item = BoundaryEdge>` | Every no-neighbor edge — outer rim + hole rims — as `BoundaryEdge { triangle, from, to, marker }`, CCW so the walkable interior is left of `from → to`. For rendering the playable outline / exporting to a PSLG. |

BVH, `rsnav_bsp`:

| Type | Notes |
| --- | --- |
| `Bsp` | `Bsp::build(&NavMesh)` is `O(n log n)`; queries are average `O(log n)`. Cheap to rebuild; rebuild whenever the underlying NavMesh changes. |
| `Bsp::locate(&nav, p) -> Option<TriangleId>` | Point-in-mesh. `None` outside the mesh / inside a hole. |
| `Bsp::nearest(&nav, p) -> Option<Nearest>` | Snap to nearest surface point. Always succeeds for non-empty mesh. |
| `Nearest { triangle, point, distance }` | Distance is 0 when `p` is interior. |
| `Bsp::query_aabb(aabb, \|tri\| ...)` | Broad-phase range query — visits every triangle whose stored AABB intersects `aabb`, average `O(log n + k)`. Reports a *superset* (a thin triangle has a fat AABB); refine inside the visitor for exact overlap. Doesn't touch the `NavMesh`. For AoE scans, render culling, box-selection. |

Pathing + queries, `rsnav_navigation`:

| Type / fn | Notes |
| --- | --- |
| `find_path(&nav, &bsp, start, goal, &PathOptions) -> Result<PathResult, PathError>` | A* + funnel. `start`/`goal` must already be inside the mesh — `nearest_point` first if you might be off-mesh. Builds a door-free `WallInfo` internally each call. |
| `find_path_with_walls(&nav, &bsp, &walls, start, goal, &PathOptions) -> Result<PathResult, PathError>` | Same, against a caller-provided `WallInfo`. Pass one built with `WallInfo::from_navmesh_with_doors` to route around closed doors; or reuse one `WallInfo` across many queries instead of rebuilding it per call. |
| `PathOptions { distance_from_wall }` | `0.0` = point agent. `> 0`: A* rejects portals shorter than this, and funnel pulls portal endpoints on wall vertices inward by this amount. Models an agent radius. |
| `PathResult { points: Vec<Vertex>, triangles: Vec<TriangleId> }` | Polyline includes `start` and `goal`; `triangles` is the A* corridor. |
| `PathError::{StartOutsideMesh, GoalOutsideMesh, Unreachable}` | `Unreachable` covers both "different region" and "every connecting portal too narrow". |
| `line_of_sight(&nav, &walls, start_tri, from, to) -> LineOfSightResult` | Walks the segment triangle-by-triangle, stopping at the first wall the `&WallInfo` reports (static walls, plus closed doors when `walls` was built with them). `start_tri` must contain `from`. Returns `Clear`, `Blocked { point }`, `SourceOutsideMesh`, or `Indeterminate` (a numerical-degeneracy walk — treat as "not clear"). |
| `path_clear(&nav, &bsp, &walls, &[Vertex]) -> bool` | Segment-by-segment line-of-sight check over a polyline — `true` if every leg can be walked on the current mesh. The cheap way to revalidate a planned path after the navmesh (or a door) changed: pass `[agent_pos, remaining_corners..]`; `false` ⇒ replan. Catches a new obstacle that landed *between* two still-on-mesh corners, which a corner-only test misses. |
| `nearest_point(&nav, &bsp, p) -> Option<NearestPoint>` | Convenience wrapper over `Bsp::nearest`. |
| `visibility_region(&nav, &bsp, &walls, source, max_radius, samples) -> Option<VisibilityRegion>` | Ray-cast `samples` directions (clamped ≥ 8; 180 is a good default). Boundary is in CCW angular order; draw as a fan from `source`. Rays are occluded by closed doors when `walls` carries them. |

> **Signature note.** `line_of_sight`, `path_clear`, and `visibility_region` each take a `&WallInfo` — the shared "what is impassable?" oracle. Build it once per mesh+door-state: `WallInfo::from_navmesh(&nav)` (no doors) or `WallInfo::from_navmesh_with_doors(&nav, &doors)`. `find_path` is the exception — it builds a door-free one internally; use `find_path_with_walls` to make pathing door-aware.

Doors — runtime edge-cuts, `rsnav_navigation` (no mesh or BSP rebuild):

| Type / fn | Notes |
| --- | --- |
| `WallInfo` | The shared wall oracle consumed by A*, the funnel, LOS, visibility, and `WallClearance`. `WallInfo::from_navmesh(&nav)` = static walls only; `WallInfo::from_navmesh_with_doors(&nav, &doors)` folds every *closed* door's edges in. `O(triangles)`; rebuild whenever the mesh or any door state changes. |
| `DoorSet` | The doors in a world, and the source of truth for which edges are gated shut. `add(&nav, &bsp, a, b, state)` resolves an authoring **segment** to the internal portal edges it crosses; `add_edge(&nav, va, vb, state)` gates one named edge (unambiguous — use with `nearest_portal_edge`). `open/close/toggle/set_state(id)`, `remove(id)`, `clear()`. `generation()` bumps on every change — your repath signal. |
| `DoorState::{Open, Closed}` | Closed = the cut edges behave as walls; open = ordinary portals. |
| `Door { id, line, state, .. }` | `is_closed()`, `edge_count()` (0 ⇒ the segment cut no portal — drawn off-mesh or only over walls). |
| `nearest_portal_edge(&nav, &bsp, p) -> Option<(VertexId, VertexId)>` | The internal portal edge nearest a world point — for "click the edge under the cursor" door authoring. Never returns a wall/boundary edge. |
| `resolve_door_edges(&nav, &bsp, a, b) -> Vec<(u32,u32)>` | The internal portal edges a segment crosses, as canonical vertex-pair keys. The resolver `DoorSet::add` uses internally; exposed for inspection. |

A door is **not** a region of triangles or a geometry patch — it's a set of internal portal *edges* a closed door promotes to walls. The mesh, triangle IDs, and BSP never change; opening/closing only flips which edges the traversal code treats as impassable. Insetting (`distance_from_wall`) keeps working: the door endpoints are wall vertices while closed, so the funnel insets around them. Existing paths are never mutated — clients compare `DoorSet::generation()` and repath when it differs.

`NavWorld<M>` — the owning container, `rsnav_navigation`:

| Type / fn | Notes |
| --- | --- |
| `NavWorld<M = NoMetadata>` | Owns `(NavMesh, Bsp, DoorSet, WallInfo)` + your metadata `M`. `new(nav, meta)` builds the BSP and oracle; `without_metadata(nav)` for `M = NoMetadata`. Removes the "rebuild `walls` when a door changes" footgun — every door mutator does it for you. |
| `add_door / add_door_edge / open_door / close_door / toggle_door / set_door / remove_door / clear_doors` | Door authoring + toggling; each rebuilds the wall oracle internally. |
| `find_path / line_of_sight / path_clear / visibility / nearest_point / locate` | Door-aware queries against the owned oracle. `line_of_sight(from, to)` locates the start triangle for you. |
| `generation()` | Mirrors the door generation — a path planned at `g` is stale once this differs. |
| `meta() / meta_mut()` | Access your metadata store. |
| `metadata_at(p) -> Option<&M::Value>` / `zone_at(p) -> Option<M::Zone>` | Point queries: locate the triangle, ask the metadata. |
| `zone_crossings(&path) -> Vec<ZoneCrossing<M::Zone>>` | Zone boundaries crossed along a `PathResult` — "entered/left zone" events. Each `ZoneCrossing { point, from, into }`. |
| `NavMetadata` trait | Implement on your own store: `type Zone: Clone + PartialEq; type Value; fn zone(&self, tri) -> Option<Zone>; fn value_at(&self, tri, p) -> Option<&Value>;`. rsnav only hands you a `TriangleId`. `NoMetadata` is the no-op default. |
| `zone_crossings(&nav, &path, zone_of)` (free fn) | The same walk without a `NavWorld` — pass any `Fn(TriangleId) -> Option<Z>`. |

`TiledWorld` — multi-tile worlds, `rsnav_navigation`:

| Type / fn | Notes |
| --- | --- |
| `TiledWorld` | A set of independent navmesh tiles placed in one world space, plus links stitched between them. Tiles are never merged or re-triangulated. |
| `add_tile(nav, offset) -> TileId` | Emplace `nav` at a world translation. Call `stitch_all` afterward. |
| `stitch_all(tol)` | (Re)build every cross-tile link by finding boundary edges that are **collinear and overlap** in world space. Vertices need not match — one long edge links to the several short ones it overlaps. `tol` is the world-space slack (e.g. `1e-6` for exact grids). |
| `set_tile_offset(tile, offset)` | Move a tile; invalidates links — re-`stitch_all`. |
| `find_path(start, goal) -> Option<Vec<Vertex>>` | Cross-tile A* (intra-tile adjacency + links) in world space, then world-space funnel. |
| `line_of_sight(from, to) -> LineOfSightResult` | Walks across tiles, crossing open seam links, stopping at the first wall or *unlinked* boundary. |
| `locate(p) -> Option<GlobalTri>` | Which tile + triangle a world point lands in. |
| `tile_count() / links() / tile_world_aabb / tile_nav / tile_offset` | Inspection + rendering accessors. |
| `GlobalTri { tile: TileId, tri: TriangleId }`, `Link { a, b, portal }` | Namespaced triangle handle; a cross-tile connection with its world-space portal segment. |

v1 limits: translation-only offsets, links always open (no per-seam doors yet), no cross-seam clearance, and a slight funnel soft-corner where one edge links to two (aligned grids are exact). A door is conceptually "a link you can close" — the planned unification of the two overlays.

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
| `ExtractOptions { min_area, remove_collinear, diagonal_smoothing }` | Defaults: keep all, strip collinear vertices, smoothing **on**. `diagonal_smoothing` iterates to a fixed point and collapses *any* run of unit-perpendicular zigzag corners whose flanking direction is preserved — handles multi-step stairs adjacent to longer straight runs. Set `diagonal_smoothing = false` to keep the exact cell-aligned boundary. |
| `extract(&bits, &opts) -> Vec<PolygonWithHoles>` | Outer rings CCW, holes CW. 4-connectivity (diagonal-only touch = disconnected). |

Dynamic obstacles + telemetry, `rsnav_dynamic`:

| Type / fn | Notes |
| --- | --- |
| `NavWorker` | Owns a background thread that turns `Arc<Bitfield>` snapshots into `Arc<NavBuild>`. `spawn(BuildOptions)` for no-telemetry; `spawn_with_listener(opts, Arc<dyn NavListener>)` for typed events. `Drop` joins the thread cleanly; `shutdown()` joins explicitly. |
| `BuildOptions { extract: ExtractOptions, perimeter_marker, hole_marker, clip_ears_max_area }` | Knobs forwarded to the per-snapshot pipeline. Defaults: extract defaults (smoothing on), marker 1 / 2, `clip_ears_max_area = 0.6` (catches half-cell stair ears on unit-cell bitfields). Set `clip_ears_max_area = 0.0` to disable the pass for hand-authored PSLGs where small ears are intentional. |
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

Multi-agent crowds, `rsnav_crowd`:

| Type / fn | Notes |
| --- | --- |
| `Agent { pos, vel, radius, max_speed, priority, goal }` + `Agent::new(pos, radius, max_speed)` | Plain `Copy` snapshot of one agent's externally visible state. `priority` is right-of-way: higher-priority agents hold their line while lower-priority neighbors yield (default `0.0` everywhere ⇒ no effect). `goal == None` ⇒ agent idles and brakes. |
| `AgentId(u32)` | Opaque handle, stable across removals (the slab reuses freed indices for new agents but does not shift other ids). |
| `Goal { target, arrive_radius }` | Agent's goal is cleared automatically once `pos.distance(target) <= arrive_radius`. |
| `CrowdConfig` | Defaults: `vo_samples = 16`, `neighbor_radius = 6.0`, `time_horizon = 1.5 s`, `stuck_ticks = 60`, `align_weight = 1.0`, `avoid_weight = 2.0`, `arrive_eps = 0.25`. Lower `avoid_weight` for denser crowds, raise it for more cautious agents. |
| `Crowd::new(Arc<NavBuild>, CrowdConfig)` | Builds an empty crowd. The `Arc<NavBuild>` is what every replan uses; swap it later with `set_nav`. |
| `Crowd::add_agent(Agent) -> AgentId` / `remove_agent(id)` / `agent(id)` / `agents()` / `agent_count()` / `path(id)` / `path_cursor(id)` / `plan_failed(id)` | Read/iteration surface. `path` returns the funnel-pulled corridor `[planned_start, c1, …, goal]`; `path_cursor` is the index of the corner the agent is currently steering toward (use `path[cursor..]` plus `agent.pos` to render the remaining leg). |
| `Crowd::set_goal(id, Option<Goal>)` / `set_pos(id, Vertex)` / `set_radius(id, f64)` / `set_max_speed(id, f64)` / `set_priority(id, f32)` | Mutators. `set_goal`, `set_pos`, and `set_radius` invalidate the path (radius drives planning clearance via `PathOptions::distance_from_wall`). |
| `Crowd::set_nav(Arc<NavBuild>)` | Swap to a freshly-published build. Each agent's remaining route (`[agent.pos, remaining corners..]`) is revalidated with segment line-of-sight (`path_clear`); routes still clear are **kept**, only genuinely-broken ones are cleared. Catches an obstacle that spawned *between* two corners. Designed for the typical `NavWorker::poll_swap` flow. |
| `Crowd::tick(dt: f64)` | One simulation step. Four passes: (1) replan / arrive, (2) rebuild spatial hash, (3) per-agent sampled-VO velocity choice, (4) integrate + snap-to-mesh + advance corridor cursor + update stuck counter. |

The per-tick pipeline is described in detail in `crates/crowd/src/lib.rs`
crate docs. Key behaviors worth knowing without reading them:

- **Per-agent radius drives planning AND avoidance.** Planning calls
  `find_path` with `distance_from_wall = radius`; avoidance treats each
  agent as a disc of its own radius. Different radii in the same crowd
  work as expected — a wider ballista plans through wider corridors and
  gets a bigger personal-space bubble.
- **Snap-to-mesh is automatic.** After integrating, if avoidance would
  carry an agent off-mesh, `Bsp::nearest` snaps it back. Agents will
  never permanently slip outside a building or into a wall — at worst
  they pin against the boundary for a tick or two while their stuck
  counter ticks up toward a replan.
- **`plan_failed` does NOT latch.** After `stuck_ticks` ticks of no
  progress the planner is retried. This is what lets a peon whose
  approach cell was briefly off-mesh recover automatically without the
  caller polling.

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
use rsnav_navigation::{line_of_sight, LineOfSightResult, WallInfo};

let walls = WallInfo::from_navmesh(&nav);          // or _with_doors(&nav, &doors)
let start_tri = bsp.locate(&nav, from).unwrap();   // `from` must be in-mesh
match line_of_sight(&nav, &walls, start_tri, from, to) {
    LineOfSightResult::Clear              => { /* visible */ }
    LineOfSightResult::Blocked { point }  => { /* wall (or closed door) at `point` */ }
    LineOfSightResult::SourceOutsideMesh  => unreachable!("we just located"),
    LineOfSightResult::Indeterminate      => { /* degenerate walk — treat as not clear */ }
}
```

`line_of_sight` tolerates a `from` that lies exactly on an edge of
`start_tri` — useful when the source came from snapping or from a
visibility-region sweep landing on a triangulation vertex.

### 6. Visibility region (visibility polygon)

```rust
use rsnav_navigation::{visibility_region, WallInfo};

let walls = WallInfo::from_navmesh(&nav);   // closed doors occlude rays if built _with_doors
let vr = visibility_region(&nav, &bsp, &walls, source, /* max_radius */ 50.0, /* samples */ 180)?;
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

### 10. Multi-agent crowd loop

```rust
use std::sync::Arc;
use rsnav_common::Vertex;
use rsnav_crowd::{Agent, Crowd, CrowdConfig, Goal};
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_polygon_extract::Bitfield;

let bf = Bitfield::new(32, 16, vec![true; 32 * 16]).expect("dims");
let nav = Arc::new(
    build_navmesh_from_bitfield(&bf, &BuildOptions::default()).expect("walkable"),
);
let mut crowd = Crowd::new(nav, CrowdConfig::default());

let a = crowd.add_agent(Agent::new(Vertex::new( 4.0, 8.0), /*r*/ 0.4, /*v_max*/ 2.0));
let b = crowd.add_agent(Agent::new(Vertex::new(28.0, 8.0), 0.4, 2.0));
crowd.set_goal(a, Some(Goal { target: Vertex::new(28.0, 8.0), arrive_radius: 0.5 }));
crowd.set_goal(b, Some(Goal { target: Vertex::new( 4.0, 8.0), arrive_radius: 0.5 }));

let dt = 1.0 / 60.0;
for _ in 0..600 {
    crowd.tick(dt);
    // Optional: read each agent's state for rendering / metrics.
    for (id, agent) in crowd.agents() {
        let _ = (id, agent.pos, agent.vel, agent.goal.is_some());
    }
    // ...optionally hand-off to the NavWorker flow:
    //   if worker.poll_swap() { crowd.set_nav(worker.current().unwrap()); }
}
```

Integration with the worker is the same one-line `crowd.set_nav(...)`
after `poll_swap` — `Crowd::set_nav` only invalidates the corridors the
new mesh actually breaks, so you can call it on every swap without
fearing a replan storm.

`rsnav-crowd` ships only the per-agent simulation primitive. Anything
above it (FSM, formations, goal-slot reservation, role-aware spawning,
priority bias for chokepoints, resource gathering loops) is application
concern — `rsnav-crowd-demo` shows one full implementation of that
layer on top of the primitive.

### 11. Runtime doors (no rebuild)

A door gates one or more internal portal *edges*; closing it makes A*/LOS/
visibility treat them as walls, with no mesh or BSP rebuild. Build the
wall oracle from the current door state and pass it to the door-aware
query functions.

```rust
use rsnav_navigation::{find_path_with_walls, nearest_portal_edge, DoorSet, DoorState, PathOptions, WallInfo};

let bsp = rsnav_bsp::Bsp::build(&nav);
let mut doors = DoorSet::new();

// Author a door across a passage (segment), or on a specific edge:
let d1 = doors.add(&nav, &bsp, Vertex::new(5.0, -1.0), Vertex::new(5.0, 5.0), DoorState::Closed);
if let Some((va, vb)) = nearest_portal_edge(&nav, &bsp, Vertex::new(5.0, 5.0)) {
    let _d2 = doors.add_edge(&nav, va, vb, DoorState::Open);
}

// Rebuild the oracle whenever a door changes, then query through it.
let walls = WallInfo::from_navmesh_with_doors(&nav, &doors);
let _ = find_path_with_walls(&nav, &bsp, &walls, start, goal, &PathOptions::default());

doors.open(d1);                          // toggle; generation() bumped
let walls = WallInfo::from_navmesh_with_doors(&nav, &doors);  // rebuild ($O(tris)$)
// clients holding a path compare doors.generation() and repath when it differs
```

### 12. NavWorld + metadata ("now leaving town")

`NavWorld<M>` owns the mesh, BSP, doors, and oracle so you stop juggling
them, and resolves world-position metadata through your `NavMetadata`.

```rust
use rsnav_common::{TriangleId, Vertex};
use rsnav_navigation::{NavMetadata, NavWorld, PathOptions};

struct Zones { centroids: Vec<Vertex> }   // your store, indexed by triangle
impl NavMetadata for Zones {
    type Zone = &'static str;
    type Value = &'static str;
    fn zone(&self, t: TriangleId) -> Option<&'static str> {
        Some(if self.centroids[t.index()].x < 10.0 { "town" } else { "wild" })
    }
    fn value_at(&self, t: TriangleId, _p: Vertex) -> Option<&&'static str> { self.zone(t).map(|_| &"x") }
}

let meta = Zones { centroids: nav.triangles.iter().map(|t| t.centroid).collect() };
let mut world = NavWorld::new(nav, meta);          // builds BSP + oracle
let _door = world.add_door(Vertex::new(10.0, -1.0), Vertex::new(10.0, 5.0), rsnav_navigation::DoorState::Open);

let path = world.find_path(start, goal, &PathOptions::default()).unwrap();
for c in world.zone_crossings(&path) {             // emit "left town / entered wild"
    println!("at {:?}: {:?} -> {:?}", c.point, c.from, c.into);
}
let here = world.metadata_at(start);               // point query
```

### 13. Multi-tile world (place + stitch + path across)

Place independent navmeshes in one world space and let `stitch_all`
connect overlapping borders — no manual welds, no merged mega-mesh.

```rust
use rsnav_common::Vertex;
use rsnav_navigation::TiledWorld;

let mut world = TiledWorld::new();
world.add_tile(tile_a, Vertex::new(0.0, 0.0));
world.add_tile(tile_b, Vertex::new(10.0, 0.0));   // east neighbor — borders touch at x=10
world.stitch_all(1e-6);                            // links the overlapping seam edges

let path = world.find_path(Vertex::new(3.0, 5.0), Vertex::new(17.0, 5.0));  // crosses the seam
let los  = world.line_of_sight(Vertex::new(3.0, 5.0), Vertex::new(17.0, 5.0));
// stream a tile in/out: add_tile / drop + re-stitch_all — never a full mesh rebuild
```

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

- **`clip_ears` is opt-in and goes between `carve_holes` and
  `build_from_cdt`.** It deletes "ear" triangles (two wall edges + one
  interior neighbor) under an area threshold and promotes the surviving
  neighbor's interior edge to a constraint with the smaller of the two
  parent wall markers. Cascading ears are resolved in a fixed-point
  loop. Two ears sharing their only interior edge ("bowtie") are left
  intact to avoid stranding a single isolated triangle. Each clip
  shrinks the walkable area by at most the threshold; pick the threshold
  relative to your world scale (≈0.6 for unit-cell bitfields).

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
  Note this is a *static* check — it does not know about closed doors, so a
  door that fully separates two areas still reports `reachable == true` and
  A* runs and returns `Unreachable`. Correct result, just no fast-fail.

- **`find_path` ignores doors; `find_path_with_walls` respects them.** The
  bare `find_path` builds a door-free `WallInfo` internally. To route around
  closed doors, build `WallInfo::from_navmesh_with_doors(&nav, &doors)` and
  call `find_path_with_walls` (the same applies to `line_of_sight`,
  `path_clear`, `visibility_region` — all take a `&WallInfo`). `NavWorld`
  does this plumbing for you.

- **Rebuild `WallInfo` after any door change.** It's a snapshot of the
  current door state, `O(triangles)`. Forgetting means stale routing. Raw
  `DoorSet` users rebuild manually; `NavWorld`'s door mutators rebuild
  automatically. Compare `DoorSet::generation()` / `NavWorld::generation()`
  to decide when to repath — doors never mutate paths you already hold.

- **A door cuts *edges*, not a passage.** Gating one internal portal edge
  separates a two-triangle doorway, but a *wide* passage has walkable
  triangles flanking that edge — A* just routes around it. Use `nearest_portal_edge`
  to author by the highlighted edge, and gate every portal across a wide gap
  (or draw a segment with `DoorSet::add`, which cuts all the edges it crosses).

- **`TiledWorld`: re-`stitch_all` after `add_tile` / `set_tile_offset`.**
  Links are derived from current tile positions; moving or adding a tile
  invalidates them. Re-stitching is cheap (no mesh rebuild) and the BSPs are
  per-tile and untouched. Border edges only link where they *overlap* in
  world space within `tol`.

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

- **Sampled-VO is not formally collision-free.** `rsnav-crowd` uses a
  Detour-Crowd-style sampled velocity-obstacle solver, not ORCA. It
  scores 1 brake + N angular candidate velocities (default 16) by
  `align_weight · alignment − avoid_weight · TTC_penalty` and picks the
  winner. In practice this produces clean lane-forming, side-stepping,
  and head-on resolution; in adversarial cases (dense chokepoints with
  many agents trying to go opposite directions) agents can brush each
  other or stall briefly. Use [`Agent::priority`] to bias who yields:
  the TTC penalty against a neighbor is scaled by `2^((other−me)/2)`
  (clamped to a `[0.25, 4.0]` factor), so a higher-priority unit holds
  its line and lower-priority ones step aside. Equal priority is the
  neutral default. A hard, already-overlapping contact is never
  discounted regardless of priority.

- **`Crowd::set_nav` validates instead of nuking.** It revalidates
  each agent's remaining route — `[agent.pos, remaining corners..]` —
  with segment line-of-sight (`path_clear`), not a corner-only on-mesh
  test. Routes still clear are kept; only genuinely-broken ones are
  cleared and rebuilt on the next tick. The segment check is what
  catches a building or forest that spawned *between* two corners that
  both still locate fine — a corner-only test would keep that path and
  walk the agent straight through the new obstacle. So a cosmetic mesh
  swap costs nothing, and a destructive one replans only the affected
  agents on the next tick.

- **Slot reservation / FSMs live in user code.** `rsnav-crowd` doesn't
  know about resources, drop-off rings, formation goals, or who-can-take-
  whose-slot. The demo (`crates/crowd-demo/src/main.rs`) implements one
  full version of that layer — `ResourceMgr`, `PeonStep`, mine + hall
  rings, forest cells, opportunistic slot stealing — that's a useful
  reference but **not** a generic library. Bring your own.

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
- `cargo run -p rsnav-crowd-demo --release` — peon-economy testbed for
  the `rsnav-crowd` flow. 96×64 cell bitfield with a town hall + mine
  + (depleting / respawning) forest blob. Three agent roles: **mine
  peons** (`mine ring slot → harvest → hall ring slot → deposit`),
  **forest peons** (`nearest tree's walkable neighbor → harvest, cell
  flips walkable → hall slot → deposit`), and **wanderers** (random
  walkable goals). Side panel: per-role spawn buttons, mine / hall
  slot usage, forest cells remaining, eviction counter, `NavStats` +
  event log. The peon FSM, ring-slot reservation, and opportunistic
  slot stealing live in the demo's `main.rs` and are not part of the
  `rsnav-crowd` library.
- `cargo run -p rsnav-door-demo --release` — togglable-doors testbed.
  76×48 cell bitfield split into four rooms by a cross of walls; each
  wall holds two **doors**. A door is a pure obstacle — open cells are
  walkable, closed cells are carved out of the bitfield so the
  `NavWorker` rebuilds the mesh without the gap (the "Option A" door:
  no mesh-specific code, just a `bool` flip and a resubmit). Actors
  patrol `home ⇄ away`; click a door (or use the checkboxes) to toggle
  it. The point of interest: when a door changes mid-route the path
  generation no longer matches the live mesh, so `Crowd::set_nav`
  revalidates each actor's remaining `[pos, corners…]` route by
  line-of-sight and replans only the blocked ones. `Door::rect` /
  `horizontal` / `vertical` author the doors; all of this lives in the
  demo's `main.rs`, not the library. (This is the *bitfield-rebuild* door;
  the in-mesh edge-cut `DoorSet` in `rsnav-navigation` is the other approach
  — no rebuild, demoed inside `rsnav-demo`'s exploration mode.)
- `cargo run -p rsnav-world-demo --release` — multi-tile world editor.
  Spawn navmesh tiles (Open / Holed / Pillars), drag them edge-to-edge
  with the **Move** tool (seams re-stitch live, links drawn green), then
  **Path** (right-click source, left-click goal → cross-tile route) or
  **Vis** (right-click source, move cursor → cross-tile line-of-sight,
  green clear / red blocked). Links are placement-driven — no manual weld.
  Built on `rsnav_navigation::TiledWorld`.

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
| `rsnav-navigation` | `tiled_world` | 3 tiles in a row, stitch seams, path across all of them around an obstacle. |
| `rsnav-pathing` | `follow_path` | L-shape walk with anti-shortcut on/off. |
| `rsnav-dynamic` | `live_worker` | Spawn a `NavWorker` with a printing `NavListener`, place a building, demolish it; print stats at the end. |
| `rsnav-crowd` | `two_agents_pass` | Two agents head-on on an open arena; print positions every 30 ticks and a final summary verifying the discs never overlapped. |

All run as `cargo run -p <crate> --example <name>`.

---

## Reading the source when in doubt

- `crates/common/src/{vertex,polygon,triangle,mesh2d,aabb}.rs` —
  data primitives.
- `crates/common/src/geom.rs` — reusable pure-function geometry toolkit:
  orient/incircle predicates, segment intersection, point-in-triangle,
  nearest point on segment/triangle.
- `crates/triangle/src/lib.rs` — re-exports the user-facing surface
  (`delaunay`, `form_skeleton`, `carve_holes`, `clip_ears`, the `Pslg`
  types).
- `crates/triangle/src/clip.rs` — `clip_ears` ear-removal post-pass.
- `crates/navmesh/src/{navmesh,build,binary}.rs` — runtime mesh, CDT
  conversion, serialization.
- `crates/navmesh/FORMAT.md` — normative binary spec.
- `crates/navigation/src/{path,los,visibility,astar,funnel,wall}.rs` —
  pathing + queries. `wall.rs` is the `WallInfo` oracle (the single
  "is this edge impassable?" chokepoint all traversal routes through).
- `crates/navigation/src/doors.rs` — `DoorSet`, the segment/edge resolver,
  `nearest_portal_edge`. Runtime edge-cut doors, no rebuild.
- `crates/navigation/src/world.rs` — `NavWorld<M>` owning container,
  `NavMetadata` trait, `zone_crossings`.
- `crates/navigation/src/tiled.rs` — `TiledWorld`: tiles, auto-stitch link
  discovery, cross-tile A* + world-space funnel + LOS.
- `crates/bsp/src/lib.rs` — BVH index.
- `crates/pathing/src/lib.rs` — steering follower.
- `crates/crowd/src/lib.rs` — `Agent` / `Crowd` / `CrowdConfig`,
  spatial hash, sampled-VO solver, replan + integrate passes.
- `crates/crowd-demo/src/main.rs` — a worked example of the
  application layer (FSM, ring slots, forest harvest, slot stealing)
  on top of `rsnav-crowd`.
- `crates/door-demo/src/main.rs` — togglable doors as bitfield
  obstacles, patrolling actors, and path revalidation across navmesh
  hot-swaps.

Tests in each file cover the canonical use shape end-to-end and are
small enough to read top-to-bottom.
