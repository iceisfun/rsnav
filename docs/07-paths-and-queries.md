# Paths and runtime queries

Prerequisites: a built `NavMesh` and its `Bsp` (see [building navmeshes](05-building-navmeshes.md)),
and the conventions in [units and conventions](04-units-and-conventions.md). Clearance —
what `distance_from_wall` actually guarantees and which of the three mechanisms you should
be using — belongs entirely to [clearance](06-clearance.md); this page names the field and
links.

This is the single-mesh runtime surface: plan a path, revalidate it, locate a point, snap a
point, ask what is visible, scan a box. Everything here operates on an immutable `NavMesh`
plus a `Bsp` built from it. Nothing on this page rebuilds or mutates geometry.

---

## Paths

### `find_path` versus `find_path_with_walls`

Lead with the cost, because it decides which one you call.
[`find_path`](../crates/navigation/src/path.rs) (path.rs:70) has a two-line body:

```rust
let walls = WallInfo::from_navmesh(nav);
find_path_with_walls(nav, bsp, &walls, start, goal, opts)
```

`WallInfo::from_navmesh` is `O(triangles)`: it allocates a `Vec<bool>` of `vertex_count`
and scans all three edges of every triangle (wall.rs:71-84). `find_path` therefore pays a
full-mesh sweep and an allocation on **every single call**. For a one-off click-to-walk that
is irrelevant. For a crowd, a per-frame replan, or a benchmark it is the dominant per-query
cost.

Anything repeated hoists the oracle:

```rust
let walls = WallInfo::from_navmesh(&nav);           // once, per mesh
for agent in &mut agents {
    let r = find_path_with_walls(&nav, &bsp, &walls, agent.pos, agent.goal, &opts);
}
```

Rebuild `walls` whenever the mesh changes **or** any door state changes — the same lifecycle
as the `Bsp`. If you have doors at all, build it with `WallInfo::from_navmesh_with_doors`, or
let [`NavWorld`](09-doors-and-navworld.md) own it (see the closing section).

Both functions take `&PathOptions`, whose only field is `distance_from_wall`. It is passed
unchanged to two stages — A*'s `min_portal_width` and the funnel's inward portal shift — so
A* never commits to a corridor the funnel would collapse. What that number does and does not
guarantee is [clearance](06-clearance.md)'s subject, not this page's. Units are **world
units**, not bitfield cells; see [units](04-units-and-conventions.md).

Runnable reference: [`crates/navigation/examples/find_path.rs`](../crates/navigation/examples/find_path.rs),
a 4x4 square with a 1x1 hole, run at `distance_from_wall` 0.0 and 0.3.

```
cargo run -p rsnav-navigation --example find_path
```

```
distance_from_wall = 0.0  →  2 segments
    (0.500, 0.500)
    (1.500, 2.500)          <- the raw hole corner
    (3.500, 3.500)
    total length: 4.472

distance_from_wall = 0.3  →  2 segments
    (0.500, 0.500)
    (1.288, 2.712)          <- shifted along the portal, not along the wall normal
    (3.500, 3.500)
    total length: 4.696
```

### Reading `PathResult`

```rust
pub struct PathResult {
    pub points: Vec<Vertex>,       // the string-pulled polyline
    pub triangles: Vec<TriangleId>,// the A* channel
}
```
(path.rs:56)

The two fields have **different lengths and different meanings**. `triangles` is the raw
corridor A* selected, including the start and goal triangles; `points` is what the funnel
pulled out of it. There is no index correspondence between them. Zipping them is a bug.
`triangles` is for diagnostics, debug rendering, and per-triangle lookups such as
[zone crossings](09-doors-and-navworld.md).

`points` always begins at the literal `start` you passed and ends at the literal `goal`
(funnel.rs pushes degenerate portals `(start, start)` and `(goal, goal)` at both ends).
Neither endpoint is ever adjusted for clearance. An agent standing flush against a wall when
you plan gets a path whose first point is flush against that wall. If the start must be
legal, make it legal before planning — snap with `nearest_point` and, if you use it, clamp
with [`WallClearance`](08-moving-agents.md).

### `PathError` and the information it loses

```rust
pub enum PathError { StartOutsideMesh, GoalOutsideMesh, Unreachable }
```
(path.rs:45)

`StartOutsideMesh` and `GoalOutsideMesh` come from `bsp.locate` returning `None` for the
respective endpoint (path.rs:95-96). Order matters: the start is tested first, so a call with
both endpoints off-mesh reports `StartOutsideMesh`.

`Unreachable` is a lossy collapse. A* has two distinct failures —
`AstarError::UnreachableRegion` (the cheap `nav.reachable` pre-check found the two triangles
in different regions) and `AstarError::Unreachable` (the open set was exhausted, typically
because every connecting portal was rejected by `min_portal_width`) — and path.rs:99-101 maps
both to the same variant. From the outside you cannot tell "the goal is in a disconnected
region, do not retry" from "every portal is narrower than this agent's body, a smaller agent
would make it".

The diagnostic is to replan at zero clearance and compare:

```rust
match find_path_with_walls(&nav, &bsp, &walls, s, g, &opts) {
    Err(PathError::Unreachable) => {
        let bare = PathOptions { distance_from_wall: 0.0 };
        if find_path_with_walls(&nav, &bsp, &walls, s, g, &bare).is_ok() {
            // geometry connects; this agent is too wide for the pinch
        } else {
            // genuinely disconnected
        }
    }
    _ => {}
}
```

The 2r body-width rule that produces the first case is pinned by
`distance_from_wall_blocks_portal_narrower_than_body` (path.rs:405, `#[cfg(test)]` code, not
a runnable example): a 0.7-wide pinch with `distance_from_wall = 0.4` is wider than one radius
but narrower than the 0.8 span the funnel reserves, so A* correctly reports the goal
unreachable rather than routing a doomed path into it.

Note that `nav.reachable` compares `NavTriangle::region`, which is derived from constrained
edges only and knows nothing about doors. A goal sealed off entirely by closed doors passes
the pre-check and only fails after a full search exhausts the open set — correct, but not
cheap. See [doors](09-doors-and-navworld.md).

`PathError` derives `Copy, Clone, Debug, Eq, PartialEq` and implements **neither `Display`
nor `std::error::Error`**. It will not compose with `?` into a `Box<dyn Error>` or an
`anyhow::Error`; you must map it yourself. (`AstarError` is the same. Of the error types in
this workspace these two are the exception — see [api map](17-api-map.md).)

### Revalidating a plan: `path_clear`

```rust
pub fn path_clear(nav: &NavMesh, bsp: &Bsp, walls: &WallInfo, points: &[Vertex]) -> bool
```
(path.rs:136)

Walks each consecutive pair of `points` with `line_of_sight`, re-locating the start of every
leg. `false` means replan. Pass only the part of the route the agent still has to traverse,
typically `[agent_pos, remaining_corners..]`.

A corner-only on-mesh test is not sufficient and this is why the function exists: a new
obstacle can land squarely between two corners that both still locate fine, leaving the
straight leg between them blocked. `path_clear` walks the segments, so it catches that.

Only `LineOfSightResult::Clear` passes a leg. `Blocked`, `SourceOutsideMesh` and
`Indeterminate` all fail it — an uncertain check conservatively triggers a replan rather than
trusting a stale corridor. Empty and single-point slices return `true` (nothing to walk).

**The blind spot: `path_clear` ignores clearance entirely.** It is a zero-width line-of-sight
test. A path planned with `distance_from_wall = 0.5` still validates as clear after a world
change that leaves it running flush against a newly-built wall, because the centerline is
still geometrically walkable. There is no swept-disc variant. If your world change can narrow
a corridor without severing it, `path_clear` will not tell you; recovery has to come from a
no-progress timer (which is exactly what `rsnav-crowd` does — see [crowds](11-crowds.md)).

Cost is `O(legs * (log n + walk length))`: one `bsp.locate` plus one full triangle walk per
leg.

Spec: `path_clear_accepts_open_routes_and_rejects_blocked` (path.rs:556, `#[cfg(test)]`) covers
open legs, a multi-segment detour, a leg across a hole, a leg leaving the mesh, and the two
degenerate slices.

### Calling `astar` and `funnel` directly

`rsnav_navigation::astar` and `rsnav_navigation::funnel` are public modules but the functions
are **not re-exported at the crate root** — you must write `rsnav_navigation::astar::astar`
and `rsnav_navigation::funnel::funnel`. Reach for them only if you are building a search that
`find_path` cannot express. One hazard: `funnel` handed an empty triangle slice returns
`vec![start, goal]` unconditionally (funnel.rs:29-31), a straight line ignoring all geometry.
`astar` never returns an empty channel, so `find_path` cannot hit this, but a direct caller
can.

---

## Point queries

The mesh-reading `Bsp` methods take the mesh as a parameter rather than borrowing it at
construction (`query_aabb` is the exception — it needs no mesh at all):

```rust
pub fn locate(&self, mesh: &NavMesh, p: Vertex) -> Option<TriangleId>       // bsp/src/lib.rs:127
pub fn nearest(&self, mesh: &NavMesh, p: Vertex) -> Option<Nearest>         // bsp/src/lib.rs:166
pub fn query_aabb(&self, query: Aabb, mut visit: impl FnMut(TriangleId))    // bsp/src/lib.rs:251
```

Nothing enforces that you pass the mesh the tree was built from. Passing a different one
yields silently wrong `TriangleId`s or an index panic inside `NavMesh::vertex`. Keep the pair
together — this is one of the reasons `NavWorld` exists.

Runnable reference: [`crates/bsp/examples/locate_and_nearest.rs`](../crates/bsp/examples/locate_and_nearest.rs).

```
cargo run -p rsnav-bsp --example locate_and_nearest
```

### `locate`

Point-in-triangle lookup, average `O(log n)`, `None` when `p` is outside every triangle.

The containment test is `rsnav_common::geom::point_in_triangle`, which is
**boundary-inclusive and winding-agnostic**. A point lying exactly on a shared edge or on a
vertex is inside several triangles, and `locate` returns whichever the left-then-right tree
descent reaches first (bsp/src/lib.rs:137-138). That is deterministic for a given tree but it
is not a meaningful geometric choice — do not build logic on which triangle you get.

### `nearest` and `nearest_point`

`Bsp::nearest` returns the closest point on the mesh surface, its owning triangle, and the
Euclidean distance. `None` only when the BVH is empty. For a point already inside a triangle,
`distance == 0.0` and `point == p` exactly.

`rsnav_navigation::nearest_point` (path.rs:163) is a field-for-field rewrap of `Nearest` as
`NearestPoint`, with no added logic. Use whichever type your call site already has in scope.

The trap: **`nearest` snaps into hole boundaries as readily as outer ones.** A point in the
middle of a carved-out building snaps to that building's inner ring, not to the nearest open
floor outside it. From the example's donut, the point at the centre of the hole:

```
  center of the hole        (  2.0,   2.0)  →  tri 0, snapped to (1.50, 2.00), dist 0.500
  far outside the mesh      ( -5.0,   5.0)  →  tri 2, snapped to (0.00, 4.00), dist 5.099
```

The first result is a legal mesh point, but if you were snapping a click that landed inside a
building it puts the agent's destination hard against the building's wall. Pinned by
`nearest_in_hole_snaps_to_inner_boundary` (bsp/src/lib.rs:426, `#[cfg(test)]`).

When two triangles are equidistant the returned triangle is arbitrary; the crate's own test
asserts only that distances match brute force, explicitly not the triangle.

### `query_aabb`

Range query for area-of-effect scans, render culling and box selection. Average
`O(log n + k)`, unspecified visit order, and it does not touch the `NavMesh` at all — the
per-triangle AABBs captured at build time are enough.

**Broad-phase only.** It reports every triangle whose *bounding box* overlaps the query, a
strict superset of the triangles whose area overlaps: a thin diagonal triangle has a fat
AABB. If you need precision, run the exact test inside the closure:

```rust
let mut hits = Vec::new();
bsp.query_aabb(area, |t| {
    let tri = nav.triangle(t);
    // exact triangle-vs-box test here before accepting `t`
    hits.push(t);
});
```

An empty query `Aabb` early-returns. A degenerate point `Aabb` (`Aabb::from_point`) is not
empty and works. `Aabb::intersects` counts touching as overlap.

---

## Line of sight

```rust
pub fn line_of_sight(
    nav: &NavMesh, walls: &WallInfo, start_tri: TriangleId, from: Vertex, to: Vertex,
) -> LineOfSightResult
```
(los.rs:47)

Walks the directed segment triangle by triangle, stopping at the first wall edge crossed. The
wall test is `walls.is_wall_edge`, so a **closed door blocks the ray exactly as a static wall
does** when you pass a door-aware `WallInfo`.

`start_tri` is an **unvalidated caller obligation**. The function never checks that `from` is
inside it, and consequently it can never return `SourceOutsideMesh` — the only producers of
that variant in the whole workspace are `NavWorld::line_of_sight` (world.rs:314) and
`TiledWorld::line_of_sight` (tiled.rs:311), both of which locate the source for you. Matching
`SourceOutsideMesh` on the free function is dead code. Its own doc comment (los.rs:20-22)
describes behaviour the free function does not have.

The obligation in practice:

```rust
let Some(start_tri) = bsp.locate(&nav, from) else { /* off-mesh: your call */ };
match line_of_sight(&nav, &walls, start_tri, from, to) { .. }
```

### The result is three-valued, and the third value is common

```rust
pub enum LineOfSightResult {
    Clear,
    Blocked { point: Vertex },
    SourceOutsideMesh,
    Indeterminate,
}
```
(los.rs:13)

`Indeterminate` means the walk hit a numerical degeneracy — the segment grazed a vertex or ran
collinear with a triangle edge, so no exit edge could be identified (los.rs:97-104) — or it
exhausted its step cap of `triangle_count * 2 + 4`. The answer is genuinely unknown. It exists
as its own variant precisely so an uncertain walk cannot masquerade as a verified-clear one.

On grid-derived meshes this is not exotic. `geom::segment_intersection` returns `None` for
parallel and collinear segment pairs, and a bitfield-extracted navmesh is full of
axis-aligned edges at integer coordinates — an axis-aligned ray along an integer coordinate is
collinear with them by construction. (The frequency is not quantified anywhere in-tree; the
mechanism is certain, the rate is not.)

**The two in-tree consumers resolve it in opposite directions**, deliberately:

| Caller | `Indeterminate` treated as | Why |
|---|---|---|
| `path_clear` (path.rs:142-147) | not clear → replan | a stale corridor is worse than a spurious replan |
| `visibility_region` (visibility.rs:72-77) | ray runs to full length | a spurious *short* ray punches a visible notch in the polygon; a spurious long ray only over-reports an open direction |

Neither is the default. Every new caller must pick a direction deliberately, and should write
down which one it picked and why. Note the consequence of the visibility choice: a visibility
region can report an open direction through what may in fact be a wall.

All four variants are matched exhaustively in
[`crates/navigation/examples/find_path.rs`](../crates/navigation/examples/find_path.rs)
(lines 58-68); that run exercises the `Blocked` arm.

---

## Visibility regions

```rust
pub fn visibility_region(
    nav: &NavMesh, bsp: &Bsp, walls: &WallInfo,
    source: Vertex, max_radius: f64, samples: usize,
) -> Option<VisibilityRegion>
```
(visibility.rs:49)

Casts `samples` rays at uniform angular intervals around `source` and records each ray's first
wall hit, or `max_radius` if it hits nothing. Returns `None` if and only if
`bsp.locate(source)` is `None` — there is no snapping, so pre-snap with `Bsp::nearest` if you
want that.

The region is star-shaped about `source` by definition, so every consecutive boundary pair
`(b[i], b[i+1])` forms a triangle `(source, b[i], b[i+1])` entirely inside it. Render it as a
triangle fan; there is no triangulation step and no convexity requirement.

Two facts about `samples`:

- It is clamped **upward** to a minimum of 8 (`let n = samples.max(8)`, visibility.rs:58).
  Requesting 4 gets you 8 boundary points. The struct field's own doc (visibility.rs:31)
  claims `boundary` "always contains exactly `samples` points", which is wrong for
  `samples < 8`. See the errata list in [troubleshooting](16-troubleshooting.md).
- Cost is `samples` multiplied by one full LOS triangle walk, each bounded only by
  `2 * triangle_count`. On a large mesh with `max_radius` big enough to cross it, a
  360-sample query walks a lot of triangles. The module header (visibility.rs:8) describes
  this as running "trivially per frame"; treat that as true for a small room and false for a
  town, and measure before putting it in a per-frame path for many sources.

Runnable reference:
[`crates/navigation/examples/visibility_region.rs`](../crates/navigation/examples/visibility_region.rs).

```
cargo run -p rsnav-navigation --example visibility_region
```

```
10x10 room with a 2x2 hole centered at (5, 5)

  center of north corridor    ( 5.0,  8.0)  bbox [0.0..10.0]x[0.0..10.0]  visible area ~ 68.9
  south-west corner           ( 1.0,  1.0)  bbox [-0.0..10.0]x[0.0..10.0] visible area ~ 71.7
  right next to the pillar    ( 3.9,  5.0)  bbox [-0.0..4.4]x[-0.0..10.0] visible area ~ 41.6
```

The third source sits beside the pillar and its bounding box is clipped at x = 4.4: the pillar
occludes everything east of it.

---

## Next: stop carrying four values around

Every function on this page takes some subset of `nav`, `bsp`, `walls`. Keeping them
consistent by hand is the source of the two silent failures named above — a `Bsp` queried
against the wrong mesh, and a `WallInfo` that is stale with respect to door state.

[`NavWorld<M>`](09-doors-and-navworld.md) owns the `NavMesh`, its `Bsp`, a `DoorSet` and the
derived `WallInfo` in one value, rebuilds the wall oracle on every door mutation, and exposes
`locate`, `nearest_point`, `find_path`, `line_of_sight`, `path_clear` and `visibility` as
door-aware methods that thread the oracle for you. It also removes `find_path`'s per-call
`WallInfo` rebuild, since it holds one. It does **not** own a `WallClearance` — you build and
maintain that yourself alongside it (see [moving agents](08-moving-agents.md)).

For a world made of several stitched meshes, `TiledWorld` has its own, materially different
query surface — different signatures, no clearance model. See
[large worlds](12-large-worlds.md).
