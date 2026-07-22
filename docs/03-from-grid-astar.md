# Coming from grid A*: what transfers and what will mislead you

You already have working pathfinding — A* over a grid, a flow field, a waypoint
graph, or Recast/Detour. This page is a translation layer. It does not explain
what a navmesh is from first principles ([02-concepts.md](02-concepts.md) does
that, and you can skip it); it states what each of your existing pieces maps
onto, and then names the habits that will produce plausible-looking wrong
results here.

Nothing on this page is a tutorial. Every name is a pointer into the page that
teaches it.

Two facts to absorb before the tables, because they invalidate designs rather
than just requiring adjustment:

- **There is no per-triangle traversal cost.** No cost hook, no weighted
  region, no query filter, no area-type multiplier. See
  [Table 4](#table-4-change-over-time) and the note under it.
- **Agent radius is not one mechanism.** Where you inflated the obstacle grid
  by the agent radius, rsnav offers three unrelated mechanisms with different
  units, different costs and different guarantees. Choosing wrong fails
  silently. [06-clearance.md](06-clearance.md) owns that decision entirely;
  this page will not summarise it.

---

## Table 1: vocabulary

| Your grid | rsnav | Notes |
|---|---|---|
| Grid cell | [`NavTriangle`](../crates/navmesh/src/navmesh.rs) | Variable size. Triangle count tracks **boundary complexity**, not area — a 4.2M-cell open field can be a handful of triangles. |
| `(x, y)` cell index | `TriangleId` | A `u32` newtype (`TriangleId(pub u32)`), not a coordinate. **Not portable** across meshes, and not across a rebuild of the same map. `NavMesh::vertex` / `::triangle` panic on an out-of-range id (navmesh.rs:91, :98). |
| 4- or 8-neighbours | `NavTriangle::neighbors[i]` | Exactly three slots. `neighbors[i]` shares the edge *opposite* `vertices[i]`; `TriangleId::INVALID` means that edge is on the mesh boundary. |
| `walkable: bool` per cell | A triangle existing at all | Non-walkable space is absent from the mesh, not flagged in it. Walls within the mesh are `edge_markers[i] != 0`; see [04-units-and-conventions.md](04-units-and-conventions.md) for marker values. |
| `if (x,y) in bounds` | [`Bsp::locate`](../crates/bsp/src/lib.rs) returning `Option<TriangleId>` | `None` means "outside every triangle". Average `O(log n)`, not `O(1)`. |
| Flood-fill to test connectivity | `NavTriangle::region` + `NavMesh::reachable(a, b)` | Precomputed at build time. `reachable` is a `region == region` comparison (navmesh.rs:105). See the caveat below. |
| Scanning the array for what's near a point | `Bsp::locate` / `Bsp::nearest` / `Bsp::query_aabb` | Three distinct queries; `query_aabb` is broad-phase only. [07-paths-and-queries.md](07-paths-and-queries.md). |
| "The map" as one array you index | `NavMesh` + `Bsp` + `WallInfo`, three objects that must agree | The `Bsp` and `WallInfo` are *derived*. Both go stale on a rebuild; `WallInfo` also goes stale on a door toggle. [09-doors-and-navworld.md](09-doors-and-navworld.md) covers `NavWorld<M>`, which owns all three so they cannot drift. |

**Region caveat.** `region` is computed from *constrained edges only* and is
fixed at build time. It knows nothing about doors, so a goal sealed behind a
closed door still passes `reachable()` (the pre-check is astar.rs:92) and only
fails after a full A* exhausts the open set (astar.rs:178). "Region" also does not mean room or zone — for game
data attached to triangles see the `NavMetadata` trait in
[09-doors-and-navworld.md](09-doors-and-navworld.md).

For a concrete look at what replaced your cell array, run:

```
cargo run -p rsnav-polygon-extract --example grid_to_polygons
```

which shows the intermediate step — a grid of booleans traced into outer rings
and holes — before triangulation. See
[`grid_to_polygons.rs`](../crates/polygon-extract/examples/grid_to_polygons.rs).

---

## Table 2: the search

| Your grid A* | rsnav | Notes |
|---|---|---|
| Open/closed lists over a cell array | A* over triangle adjacency | Same algorithm, different graph. `BinaryHeap` frontier, `closed: Vec<bool>` indexed by `TriangleId` (astar.rs:102-103). |
| Octile / Manhattan heuristic | Plain Euclidean to the goal **point** | `start_point.distance(goal_point)` (astar.rs:109). No grid-motion correction, because motion is not grid-constrained. |
| Uniform step cost (1, or √2 diagonally) | **Portal-crossing cost** | See below. |
| Path = list of cells | `PathResult::triangles` **and** `PathResult::points` | Two different things of two different lengths. |
| Blocked cell check inside the loop | `WallInfo::is_wall_edge` | One oracle folding static walls and closed doors together, shared by A*, the funnel, LOS and visibility. |
| — | Portal width rejection | With `PathOptions::distance_from_wall > 0`, A* refuses portals too narrow for the body. [06-clearance.md](06-clearance.md). |

### Portal-crossing cost

Each triangle records the point at which the route enters it: the closest point
on the shared portal edge to the *predecessor's* entry point. A step costs the
straight-line distance between consecutive entry points (astar.rs:151-165).

This exists because the funnel only ever produces the shortest path *within*
the channel A* commits to. A centroid-to-centroid metric over-estimates the
funnelled length by a different amount per channel, so it can rank a channel
that wraps around an obstacle below the tight straight-then-turn channel — and
the funnel then faithfully renders the detour. That failure is pinned by the
regression test `path_commits_to_shorter_channel_around_offset_hole` at
[`path.rs:512`](../crates/navigation/src/path.rs).

Cost and heuristic stay consistent (triangle inequality holds), so the closed
set remains valid. But entry points are chosen greedily per triangle, so this
is a close approximation of the funnelled length, **not a proof of optimality**.
The source says so in its own words (astar.rs:63-67). If you are used to grid
A* being exactly optimal for the graph it searches, that property is gone here
— and it was never the property you wanted, since the graph is a channel
selector, not the path.

### `triangles` and `points` are not parallel

`PathResult::triangles` is the raw A* channel including the start and goal
triangles. `PathResult::points` is the string-pulled polyline. They have
unrelated lengths — a hundred-triangle corridor across an open room
string-pulls to two points. **Never zip them.** Anything that wants a
triangle-per-point correspondence has to re-locate each point.

---

## Table 3: smoothing and radius

| Your grid A* | rsnav | Notes |
|---|---|---|
| Bresenham / raycast post-smoothing pass | The funnel — **already ran** | `find_path` is `astar` then `funnel` (path.rs:95-105). The polyline you receive is string-pulled. |
| 8-way movement staircase artifacts | Gone by construction | Paths are any-angle; there is no movement quantisation to correct. |
| Path is a sequence of cell centres | `points[0]` is your literal `start`, `points.last()` your literal `goal` | The funnel pushes degenerate portals at both ends (funnel.rs:37, :45). Endpoints are **never** adjusted for clearance — a start already touching a wall stays touching it. |
| "String-pulled path may cut corners, so inflate obstacles" | Three separate mechanisms | Baked contour inset, grid erosion, and query-time clearance. Different units, different costs, mutually double-counting. [06-clearance.md](06-clearance.md). |
| Inflate the obstacle grid by the agent radius | Closest analogue is `Bitfield::eroded` | It is also the one whose radii are cell-quantized. Do not reach for it on the assumption that it is the familiar option. [06-clearance.md](06-clearance.md). |
| Post-hoc smoothing knobs (tension, subdivision) | None | Corner *softening* for movement is the follower's job, not the planner's. [08-moving-agents.md](08-moving-agents.md). |

---

## Table 4: change over time

| Your grid A* | rsnav | Notes |
|---|---|---|
| Flip a cell's walkable flag | Edit the `Bitfield` and rebuild | Submit an `Arc<Bitfield>` snapshot to a background `NavWorker`. [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md). |
| Flip a cell to open/shut a gate | `DoorSet` — no rebuild | A door is a set of *edges*. Mesh, triangle ids and `Bsp` are untouched; only the `WallInfo` is rebuilt. [09-doors-and-navworld.md](09-doors-and-navworld.md). |
| Invalidate a cached path by re-walking its cells | [`path_clear`](../crates/navigation/src/path.rs) | Leg-by-leg LOS revalidation. Zero-width — see habit (c). |
| Per-cell terrain cost multipliers (mud, road, danger) | **Not supported** | See below. |
| Multiple grids for multiple agent sizes | Multiple navmeshes, or one query-time radius | `ClearanceField` computed once and thresholded at several radii gives small/medium/large meshes for roughly one transform. [06-clearance.md](06-clearance.md). |

### There is no traversal cost model

A*'s step cost is pure geometric distance and its heuristic is
`start_point.distance(goal_point)`. There is no cost callback, no weighted
region, no per-triangle multiplier, and no query-filter equivalent anywhere in
the workspace. `PathOptions` has exactly one field, `distance_from_wall`
(path.rs:14-41).

If your design needs slow-mud, road-preference, danger-avoidance or
faction-restricted terrain, you must write your own search over `NavMesh`
adjacency. The pieces are public and sufficient: `NavTriangle::neighbors`,
`edge_markers`, `centroid`, `area`, `WallInfo::is_wall_edge`, and
`funnel::funnel` to string-pull whatever channel you produce (it is
`pub mod funnel`, though not re-exported at the crate root — see
[17-api-map.md](17-api-map.md)). Attach your cost data per triangle through the
`NavMetadata` trait or your own side table.

This is stated here, early, so it does not surprise you in week three.

---

## If you are not coming from a grid

The tables above are written against a cell grid because that is the most
common starting point. Two other backgrounds map on cleanly:

**From a waypoint / navgraph.** Your nodes were hand-placed and your edges were
hand-authored or line-of-sight tested; coverage was sampled, so a point between
waypoints had no representation. Here coverage is *exhaustive* — every walkable
point is inside exactly one triangle (or on a shared edge, see habit (g)), and
`Bsp::locate` maps an arbitrary world point into the graph. The consequence
worth internalising is that you no longer snap an agent to the nearest node
before planning: `find_path` takes raw world coordinates, and
`nearest_point` exists to handle off-mesh input (a click outside the geometry),
not to quantise on-mesh input. Edges are also not authored — adjacency is
implied by triangles sharing an unconstrained edge, so you cannot delete a
single link. The mechanism for "this connection is now shut" is a `DoorSet`
([09-doors-and-navworld.md](09-doors-and-navworld.md)).

**From a flow field.** You computed one field per destination and every agent
read it for free; cost scaled with map size and destination count, not agent
count. rsnav has no field: cost is per query, per agent, and there is no shared
structure amortising many agents onto one goal. If your design depends on
thousands of agents converging on a handful of destinations, that amortisation
is something you would have to build on top — plan one path and share the
polyline, or write your own multi-source search over `NavMesh` adjacency, as in
the no-cost-model note above. What rsnav gives you instead is per-agent local
avoidance around individually planned corridors
([11-crowds.md](11-crowds.md)), with the honest caveat that `rsnav-crowd` is
single-threaded throughout.

**From Recast/Detour.** The concepts transfer nearly one-for-one: a polygon
mesh, a BVH for point location, A* over polygon adjacency, and a funnel. The
differences that matter are that rsnav triangulates (three neighbours, not up
to six), that there is no query-filter or area-cost equivalent, that clearance
is a build-time or query-time choice rather than a single agent-radius voxel
parameter ([06-clearance.md](06-clearance.md)), and that off-mesh connections,
tile-local rebuilds and height/3D layers do not exist. Tiling exists but is
narrow — read [12-large-worlds.md](12-large-worlds.md) before committing to it.

---

## Habits that will actively mislead you

**(a) Do not re-smooth the returned path.** The funnel has already run inside
`find_path`. A second smoothing pass over `points` — Bresenham shortcutting,
spline fitting, corner cutting — destroys whatever clearance the portal shrink
bought, because that clearance lives in exactly the vertex positions your pass
would move. If the path still looks too tight, the fix is
[06-clearance.md](06-clearance.md), not a post-pass.

**(b) Do not cache a `TriangleId` across a rebuild.** Ids are minted per
`NavMesh` instance and are dense indices into that instance's arrays.
`NavMesh::vertex` and `::triangle` panic on an out-of-range id, and — worse —
an in-range id from a different mesh returns a plausible wrong triangle with no
error at all. This applies to a rebuilt mesh of the *same* map: nothing is
stable. The same rule kills `DoorSet` edge keys across a rebuild
([09-doors-and-navworld.md](09-doors-and-navworld.md)) and invalidates every
`Bsp`, `WallInfo` and `WallClearance`
([10-dynamic-rebuilds.md](10-dynamic-rebuilds.md)). Cache world coordinates, not
ids.

**(c) Do not step the path cell-by-cell to revalidate.** There are no cells,
and re-locating every corner is not sufficient anyway — a new obstacle can land
*between* two corners that both still locate fine. `path_clear` walks each leg
with `line_of_sight` and catches that (path.rs:136). Know its blind spot: it is
a **zero-width** test. A path planned with `distance_from_wall = 0.5`
revalidates as clear after a change that leaves it flush against a new wall.
`path_clear` answers "is this route still geometrically walkable", never "is it
still walkable by *this* agent".

**(d) More triangles is not more accurate.** On a grid, raising resolution
strictly improves the fit of walkable space and strictly costs more. Here the
triangulation is exact for the polygon it was given: extra triangles add search
nodes and memory and buy nothing. `ExtractOptions::remove_collinear` (default
`true`) exists precisely to delete them — the raw trace emits a vertex at every
unit cell corner, and turning it off takes a 3×3 walkable block from 4 vertices
to 12. Where the fit *does* change is upstream, in the grid resolution you
authored and in `diagonal_smoothing`, which is not area-preserving.
[05-building-navmeshes.md](05-building-navmeshes.md).

**(e) Do not assume a rebuild is catastrophic.** Grid pathfinders usually treat
"rebuild the whole structure" as a last resort. Here it is the normal path for
geometry changes: `NavWorker` builds on a background thread and *coalesces*
queued snapshots so it cannot fall behind, and you swap in the result at a
frame boundary. And for the case people reach for most — a gate that opens and
shuts — no rebuild happens at all, because a `DoorSet` toggle touches only the
wall oracle. [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md),
[09-doors-and-navworld.md](09-doors-and-navworld.md).

**(f) Line of sight is three-valued, not two.** `LineOfSightResult` is `Clear`,
`Blocked { point }`, `SourceOutsideMesh`, and `Indeterminate` (los.rs:13-32).
`Indeterminate` means the walk hit a numerical degeneracy — the segment ran
collinear with an edge or grazed a vertex — or exhausted its step cap, and the
answer is genuinely unknown. It is **common**, not exotic, on grid-derived
meshes: `segment_intersection` returns `None` for collinear pairs, and a
bitfield-extracted mesh is full of axis-aligned edges that an axis-aligned ray
runs straight along. The two in-tree consumers resolve it in *opposite*
directions — `path_clear` fails the leg (conservative, at the cost of spurious
replans), `visibility_region` keeps the ray at full length (visibility.rs:72-77,
which can report an open direction through what may be a wall). Any new caller
must pick a direction deliberately. Also note the free `line_of_sight` never
returns `SourceOutsideMesh`: `start_tri` is an unvalidated caller obligation, so
only the `NavWorld` and `TiledWorld` wrappers produce that variant.

**(g) Point location is boundary-inclusive and winding-agnostic.** A grid cell
lookup is a total function with exactly one answer. `Bsp::locate` is not:
`point_in_triangle` returns `true` on edges and vertices (geom.rs:143-145), so a
point exactly on a shared edge is inside *both* neighbours, and `locate` returns
whichever the left-then-right tree descent reaches first (bsp/src/lib.rs:131-139).
That is deterministic for a given tree but not a geometrically meaningful
choice. Do not build logic that depends on which side of an edge a point
"belongs" to. The same closed-set convention runs through `Polygon::contains`,
`Aabb::contains` and `Aabb::intersects` — see
[04-units-and-conventions.md](04-units-and-conventions.md).

---

## Where to go next

You have translated the model. The two pages that will actually save you time
are, in order:

1. [04-units-and-conventions.md](04-units-and-conventions.md) — `false` is
   **wall**, row 0 is the **bottom** row, `BuildOptions::inset` is in **cells**
   while `PathOptions::distance_from_wall` is in **world units**. Read this
   before you set a single option.
2. [06-clearance.md](06-clearance.md) — the three-way agent-radius decision,
   which is the one place your grid instincts have no correct analogue.

Then [05-building-navmeshes.md](05-building-navmeshes.md) for getting your real
map in, and [07-paths-and-queries.md](07-paths-and-queries.md) for the rest of
the runtime surface — starting with the fact that `find_path` rebuilds an
`O(triangles)` `WallInfo` on every single call.
