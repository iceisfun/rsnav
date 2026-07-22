# It's not working

Symptom, cause, how to check, where to read more. Blocks are ordered by roughly how
often they bite, not by which crate is at fault, because almost every failure in this
library is **silent** — you get wrong geometry, not an error.

Nothing here teaches. If a block does not resolve it in five lines, follow the link.
If the answer turns out to be a clearance *choice* rather than a bug, the link goes to
[06-clearance.md](06-clearance.md) and stops there.

Two lists at the end: [Tools](#tools) and
[Known doc/code contradictions](#known-doccode-contradictions).

---

## Build fails

### `BuildError::NoPerimeter` on a map that is obviously walkable

`false` is **wall**, not floor, so a default-initialized `Vec<bool>` and
`Bitfield::empty(w, h)` are both a solid block with zero walkable cells.
Check: `bf.data.iter().filter(|c| **c).count()`. If it is 0, invert your fill.
Second cause: `ExtractOptions::min_area` culled every region — it is compared against
`outer.area()` only. See [04-units-and-conventions.md](04-units-and-conventions.md),
[05-building-navmeshes.md](05-building-navmeshes.md). Also errata (a) and (b) below.

### `BuildError::EmptyMesh`

Extraction found regions but the CDT produced zero live triangles. Usual cause is an
inset that ate everything — full erosion is `Ok`-and-empty at the triangle level
(`build_cdt_with_inset`, [inset.rs](../crates/triangle/src/inset.rs)), and only
`build_navmesh_from_bitfield` turns the empty result into this error.
Check: rebuild with `inset: None`. See [06-clearance.md](06-clearance.md).

### Panic instead of an error on a bad inset or `snap_cell`

`build_cdt_with_inset` uses `assert!(inset.is_finite() && inset >= 0.0)`
([inset.rs:126](../crates/triangle/src/inset.rs)), and
`SnapGrid::from_target` asserts finite, positive and non-subnormal
([planarize.rs:47](../crates/common/src/planarize.rs)). Neither is an `Err`.
Only `build_navmesh_from_bitfield` validates first and returns `BuildError::InvalidInset`.
Validate before you call the triangle-level API. See [13-authored-geometry.md](13-authored-geometry.md).

### `form_skeleton` returns `SegmentInsertError::SelfIntersection`

The legacy path cannot insert a constraint that crosses an existing one, at all.
An authored scene where a hole ring crosses a perimeter ring is unbuildable on it.
Check: this is reproduced deliberately in
[`inset_hole_cross_regression.rs`](../crates/navmesh/tests/inset_hole_cross_regression.rs) (test code),
which then builds the same scene through `build_cdt_with_inset`.
Fix: use the inset path, at `inset: Some(0.0)` if you want no erosion.
See [13-authored-geometry.md](13-authored-geometry.md).

---

## The world is in the wrong place

### Everything is mirrored top-to-bottom

Row 0 is the **bottom** row and the y-axis points up. PBM, PNG and most tile editors
put row 0 at the top. Check: the P4 reader in
[`erode_vs_inset.rs`](../crates/dynamic/examples/erode_vs_inset.rs) flips on load
with `cells[(h - 1 - row) * w + col]` — that line is the canonical instance.
See [04-units-and-conventions.md](04-units-and-conventions.md).

### `PathError::StartOutsideMesh` for a point that is visibly inside

Three causes, in order of likelihood: the row flip above; erosion or inset removed the
floor under the spawn point; or you are locating against a different `NavMesh` than the
`Bsp` was built from. Check: `bsp.nearest(&nav, p)` — a small non-zero distance means
erosion, a large one means a coordinate-space error.
See [07-paths-and-queries.md](07-paths-and-queries.md).

### Walkable floor disappeared along the map border

Everything outside the grid reads as wall (`Bitfield::at` takes `i64` and returns
`false` out of range), so any erosion radius > 0 removes the outermost ring of cells.
This is correct, and visible whenever walkable area runs to the grid edge.
Fix: pad the bitfield by at least the radius before eroding.
See [06-clearance.md](06-clearance.md).

### Two areas that touch at a corner are not connected

Regions are traced with 4-connectivity. Corner-touching cells become separate polygons,
separate regions, and `NavMesh::reachable` returns false between them.
Check: `nav.region_count`. Fix: widen the join to share an edge.
See [04-units-and-conventions.md](04-units-and-conventions.md).

---

## Pathing behaves wrongly

### `PathError::Unreachable` when the route is plainly there

`AstarError::UnreachableRegion` and `AstarError::Unreachable` both collapse into this
one variant ([path.rs:99](../crates/navigation/src/path.rs)), so "disconnected" and
"every portal narrower than the agent" are indistinguishable from the outside.
Check: retry with `PathOptions { distance_from_wall: 0.0 }`. If it now succeeds, it was
portal width, not connectivity. See [06-clearance.md](06-clearance.md), [07-paths-and-queries.md](07-paths-and-queries.md).

### A closed door does not block the path until A* has fully searched

`astar` pre-checks `nav.reachable(start, goal)`, which compares `NavTriangle::region`.
Regions are derived from constrained edges only and know nothing about doors, so a
door-sealed goal passes the cheap check and fails only after the open set is exhausted.
Correct result, worst-case cost. See [09-doors-and-navworld.md](09-doors-and-navworld.md).

### The path hugs or clips the wall despite `distance_from_wall`

`distance_from_wall` is not a Euclidean guarantee. The funnel shifts each wall-vertex
portal endpoint **along the portal**, not along the wall normal
([funnel.rs:115](../crates/navigation/src/funnel.rs)), so a portal meeting the wall
at angle θ buys roughly `r·sin(θ)`. Separately, `points[0]` and `points.last()` are
your literal start and goal and are never adjusted at all.
Fix: `WallClearance::clamp` is the only true distance-to-segment invariant. See [06-clearance.md](06-clearance.md).

### Clearance feels doubled — the agent walks down the middle of a corridor

You baked an inset or erosion of `r` and then also passed the full agent radius to
`PathOptions::distance_from_wall` or `WallClearance::clamp`.
Fix: pass `max(0, agent_radius - r)`. With `diagonal_smoothing` on, substitute
`max(0, r - 0.708)` for `r` first. See [06-clearance.md](06-clearance.md).

### `path_clear` says a route is fine when it obviously is not walkable at width

`path_clear` is a zero-width line-of-sight test per leg. It never re-checks clearance,
so a path planned at `distance_from_wall = 0.5` still validates after a change leaves it
flush against a new wall. Check: replan instead of revalidating when width matters.
See [07-paths-and-queries.md](07-paths-and-queries.md).

---

## Clearance is not what you asked for

### Erosion removed far more than the radius you requested

Grid erosion is cell-quantized. Achievable clearances are exactly
`{0, 1, √2, 2, √5, …}`, so every radius in `(0, 1]` is one full 8-connected peel: a
0.128 request becomes 1.0, roughly 7.8× the erosion asked for.
Check: the guarantee suite in
[`erode_adversarial.rs`](../crates/polygon-extract/tests/erode_adversarial.rs) (test code).
Fix: sub-cell radii belong to the contour inset. See [06-clearance.md](06-clearance.md).

### Erosion applied, agents still clip corners

`ExtractOptions::diagonal_smoothing` defaults to `true`, runs *after* erosion inside
`extract`, and is not area-preserving — at reflex corners it bulges up to
`√2/2 ≈ 0.708` cells into the wall. Guaranteed clearance drops to `max(0, r - 0.708)`
([lib.rs:868](../crates/polygon-extract/src/lib.rs)).
Fix: set `diagonal_smoothing = false`, or erode by `r + 0.708`. See [06-clearance.md](06-clearance.md).

### `ClearanceField::threshold(0.0)` returned a grid with the walls still in it

Intended. The field cannot distinguish a wall cell from a wall-adjacent walkable one —
both have `sq == 0` — so a threshold of zero is vacuously true everywhere
([lib.rs:758](../crates/polygon-extract/src/lib.rs)).
Fix: use the original bitfield, or `Bitfield::eroded` with radius 0, which clones.

---

## Geometry silently missing or wrong

### A region is walkable that should be a carved-out hole

`carve_holes` needs a seed point strictly inside every hole. `rsnav-dynamic` gets it
from `Polygon::interior_point()`, which returns `Option` and is skipped when `None`
([dynamic/src/lib.rs:346](../crates/dynamic/src/lib.rs)) — a hole with no seed is
never carved and stays walkable, with no diagnostic.
Check: count `pslg.holes` against your hole ring count. See [13-authored-geometry.md](13-authored-geometry.md).

### Input geometry vanished with no error anywhere

Three independent silent drops. `extract` discards holes with no enclosing outer ring
([lib.rs:198](../crates/polygon-extract/src/lib.rs)). `InsetBuild::skipped_rings` is
the only signal a ring was dropped at entry, and `rsnav-dynamic` discards it. And a
perimeter dropped as fully eroded is `continue`d without being recorded in
`skipped_rings` at all, so diffing counts against it will mislead.
See [05-building-navmeshes.md](05-building-navmeshes.md), [13-authored-geometry.md](13-authored-geometry.md).

### Phantom walls or a region split in two, only in release builds

`drop_interior_constraints` requires `soup_markers` **sorted and deduplicated** —
membership is a binary search guarded only by a `debug_assert!`
([winding.rs:374](../crates/triangle/src/winding.rs)). An unsorted slice silently
misses markers in release. Check: run the same input in debug and watch for the assert.
See [13-authored-geometry.md](13-authored-geometry.md).

### `carve_by_winding` deleted the whole mesh

It must receive the **original, un-planarized** soup, already carrying natural
orientation (perimeter CCW, hole CW). `winding_number` does no normalization, so a CW
perimeter inverts the keep rule. The keep rule is `winding >= 1`, not `== 1`.
See [13-authored-geometry.md](13-authored-geometry.md), [04-units-and-conventions.md](04-units-and-conventions.md).

---

## Large worlds and rebuilds

### Two meshes that touch geometrically are not connected

`NavMesh::append` assumes the inputs are disjoint: no vertex dedup, no adjacency across
the seam, region ids offset. Touching triangles get no neighbour link and land in
different regions, so `reachable()` is false across a physically walkable join.
Fix: use `TiledWorld` + `stitch_all`, or build both sides as one bitfield.
See [12-large-worlds.md](12-large-worlds.md).

### No path across a tile seam

Check first: `world.links().is_empty()`. If true, nothing stitched. Causes, in order:
`BuildOptions::inset` was set (per-tile contour erosion pulls boundary edges off the tile
line and `stitch_all` matches nothing); a tile was eroded instead of the global grid;
`min_area` or `clip_ears_max_area` removed a seam fragment in one tile only.
Both directions are pinned in [`tiled_erosion_seams.rs`](../crates/navigation/tests/tiled_erosion_seams.rs) (test code);
its `seam_edges` helper is the diagnostic. See [12-large-worlds.md](12-large-worlds.md).

### `find_path` crosses a seam that `line_of_sight` refuses to

Two tolerances, one configurable. `stitch_all(tol)` controls matching, but
`link_across` hardcodes `1e-6` ([tiled.rs:365](../crates/navigation/src/tiled.rs)).
Stitching noisy borders with a large `tol` produces links LOS will not cross.
See [12-large-worlds.md](12-large-worlds.md).

### Paths route through portals that are no longer there

`set_tile_offset` does not clear `links` or `link_index`; the stale `Link::portal`
segments keep their old world-space coordinates and remain fully routable.
Fix: call `stitch_all` after every move. Same for `add_tile` — the new tile is unlinked
until you re-stitch. Errata (h). See [12-large-worlds.md](12-large-worlds.md).

### Everything broke immediately after a navmesh rebuild

`TriangleId` and `VertexId` are per-instance, and `NavMesh::vertex`/`triangle` **panic**
on an out-of-range id. A swap invalidates every cached id, the `Bsp`, every `WallInfo`
and `WallClearance`, every `DoorSet`'s resolved edges, and every in-flight path.
Fix: rebuild all of them; revalidate paths with `path_clear`.
See [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md), [04-units-and-conventions.md](04-units-and-conventions.md).

### A door does nothing

`DoorSet::add` returns a `DoorId` even when the authoring segment cut no toggleable
portal — off-mesh, collinear with an edge (`segment_intersection` returns `None`), or
across a wall, since `resolve_door_edges` skips constrained edges.
Check: `doors.get(id).unwrap().edge_count() == 0` means inert.
Fix: `nearest_portal_edge` + `add_edge`. See [09-doors-and-navworld.md](09-doors-and-navworld.md).

---

## Runtime queries return surprising values

### `LineOfSightResult::Indeterminate` constantly

`segment_intersection` returns `None` for parallel and collinear pairs, and
bitfield-derived meshes are full of axis-aligned edges, so an axis-aligned ray along an
integer coordinate is collinear with them. The two in-tree consumers resolve it in
**opposite** directions: `path_clear` treats it as blocked, `visibility_region` as clear.
Pick a direction deliberately. All four variants are matched in
[`find_path.rs`](../crates/navigation/examples/find_path.rs). See [07-paths-and-queries.md](07-paths-and-queries.md).

### Visibility reports open ground through a wall

That is the other half of the block above: `visibility_region` keeps an `Indeterminate`
ray at full length ([visibility.rs:73](../crates/navigation/src/visibility.rs)).
It over-reports rather than under-reports, by design.
See [07-paths-and-queries.md](07-paths-and-queries.md).

### `nearest_point` snapped into the middle of a building

`Bsp::nearest` treats hole boundaries exactly like outer boundaries, so a point inside a
carved-out building snaps to that building's inner ring, not to open floor.
Pinned by `nearest_in_hole_snaps_to_inner_boundary`
([bsp/src/lib.rs:426](../crates/bsp/src/lib.rs), test code). See [07-paths-and-queries.md](07-paths-and-queries.md).

### `query_aabb` reported triangles that are not in the box

Broad-phase only: it visits every triangle whose stored AABB overlaps the query, a
strict superset. A thin diagonal triangle has a fat AABB.
Fix: run an exact test inside the visit closure. See [07-paths-and-queries.md](07-paths-and-queries.md).

### `line_of_sight` never returns `SourceOutsideMesh`

The free function trusts the caller-supplied `start_tri` and never validates it; only
the `NavWorld` and `TiledWorld` wrappers produce that variant. Matching on it against
the free function is dead code, despite the variant's own doc comment.
See [07-paths-and-queries.md](07-paths-and-queries.md).

---

## Performance

### Inset builds are unbearably slow

`carve_by_winding` runs a full brute-force `winding_number` cross-check for **every**
triangle under `debug_assert_eq!`
([winding.rs:336](../crates/triangle/src/winding.rs)). It is free in release and
crippling in debug. Never benchmark or demo the inset path in a debug build.
See [15-performance-and-determinism.md](15-performance-and-determinism.md).

### Per-frame stall proportional to mesh size

Two usual suspects. `find_path` rebuilds an `O(triangles)` `WallInfo` on every call
([path.rs:77](../crates/navigation/src/path.rs)) — hoist one and use
`find_path_with_walls`, or use `NavWorld`. And `WallClearance::clamp` linearly scans
every wall segment in the mesh, four relaxation passes, per call, with no spatial index.
See [07-paths-and-queries.md](07-paths-and-queries.md), [08-moving-agents.md](08-moving-agents.md).

### One agent pegs a core

An agent whose goal is unreachable runs a full A* **every tick, forever**: the failure
arm clears the path, and the next tick's guard is `path.is_empty() || stuck >= stuck_ticks`
([crowd/src/lib.rs:547](../crates/crowd/src/lib.rs)). `plan_failed` does not latch or
suppress anything. Fix: read `plan_failed()` yourself and clear the goal. Errata (d).
See [11-crowds.md](11-crowds.md).

### `visibility_region` is not the cheap per-frame call the header implies

Cost is `samples × one full LOS walk`, each bounded only by `2 × triangle_count`.
Also, `samples` is clamped **up** to 8, so asking for 4 gets you 8.
See [07-paths-and-queries.md](07-paths-and-queries.md).

---

## Agents and the worker

### An agent teleports, or `pos != pos + vel*dt`

Deliberate. Crowd avoidance never consults the navmesh, so `integrate` snaps a proposed
off-mesh position to `bsp.nearest` ([crowd/src/lib.rs:607](../crates/crowd/src/lib.rs)),
and `replan_and_arrive` re-snaps an off-mesh agent at the top of the tick.
A position read between ticks is not an integration of the last velocity.
See [11-crowds.md](11-crowds.md).

### Two agents meet head-on and freeze

Identical radius, speed and priority give both the same score surface and neither picks
a side. Every test and example in the crate offsets the two agents by ±0.05 in y to break it
([`two_agents_pass.rs`](../crates/crowd/examples/two_agents_pass.rs)).
Fix: a positional offset, different goals, or different priorities. See [11-crowds.md](11-crowds.md).

### An `AgentId` addresses the wrong agent

Slots are recycled: `remove_agent` leaves a `None` hole and the next `add_agent` fills
the first hole it finds. There is no generation tag. Also, all five setters silently
no-op on an unknown id — no `Result`, no `bool`. See [11-crowds.md](11-crowds.md).

### The worker stopped and nothing said so

`submit_snapshot` discards the send result (`let _ = self.tx.send(...)`), so snapshots
vanish silently once the worker thread is gone.
Check: `worker.is_running()`. See [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md).

### A `NavListener` produces no output at all

Every listener call is wrapped in `catch_unwind` and the panic is swallowed with no
diagnostic ([dynamic/src/lib.rs:687](../crates/dynamic/src/lib.rs)).
Check: put a bare `println!` first in the handler.
Also note `last_error()` is a formatted `String`; the typed `BuildError` is only visible
via `NavEvent::BuildFailed`. See [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md).

### Build output differs between runs or thread counts

This should be impossible for the navmesh pipeline. Reproduce with `par_bench`, which
builds at 1/2/4/8/16/32/auto threads and compares `to_bytes()` against the first run,
printing `ok` or `DIVERGED`. If it prints `DIVERGED`, that is a bug worth filing.
`Crowd` determinism is *not* tested and must not be assumed.
See [15-performance-and-determinism.md](15-performance-and-determinism.md).

---

## Tools

```
cargo run --release -p rsnav-dynamic --example stage_bench -- --digest [testdata_dir]
```
Per-stage timings plus an FNV-1a digest of each serialized `NavMesh` — the bit-identity
gate for refactors.

```
cargo run --release -p rsnav-dynamic --example par_bench -- [testdata_dir]
```
Cross-thread byte-identity check. `--inset <r>` runs the same check over the
offset/planarize/winding path.

- `TiledWorld::links().is_empty()` — the canonical "my seams failed to stitch" signal.
- `seam_edges` in [`tiled_erosion_seams.rs`](../crates/navigation/tests/tiled_erosion_seams.rs)
  — counts boundary edges actually lying on the seam line, which tells you whether the
  problem is the geometry or the matching.
- `InsetBuild::soup` — the offset contours the winding cull classified against, kept for
  debug rendering.
- `verify_planar` ([planarize.rs:191](../crates/common/src/planarize.rs)) — `O(n²)`,
  tests and debug only; confirms a segment set really is non-crossing.
- A debug build. Several invariants in this codebase are `debug_assert`s
  (`carve_by_winding`'s differential oracle, `drop_interior_constraints`' sortedness).
  A wrong-output bug that only appears in release is often one of them.

---

## Known doc/code contradictions

Verified in-tree. If you tripped over one of these, the code is right and the comment is
wrong — you are not misreading it.

- **(a)** The `no_run` crate doctest at
  [dynamic/src/lib.rs:19](../crates/dynamic/src/lib.rs) labels `Bitfield::empty(32, 32)`
  "Some 32x32 walkable map with a central wall". It is a solid block of wall; building
  from it returns `NoPerimeter`. The block is `no_run`, so it never executes.
- **(b)** `BuildError::NoPerimeter`'s doc names `min_polygon_area`. No such field exists
  anywhere in the workspace; the real one is `ExtractOptions::min_area`.
- **(c)** *Fixed in tree.* The header comment in
  [`grid_to_polygons.rs`](../crates/polygon-extract/examples/grid_to_polygons.rs)
  previously claimed the defaults included "no smoothing"; it now states
  `diagonal_smoothing = true` correctly and compares against `false`.
- **(d)** `Slot::plan_failed`'s doc says it "suppresses further replan attempts". It does
  not — see the pegged-core block above. The tick-loop comment saying it does not latch is
  the accurate one.
- **(e)** `VisibilityRegion::boundary`'s field doc promises "exactly `samples` points",
  contradicting `let n = samples.max(8)` in the function it comes from.
- **(f)** `Door::line`'s doc says it is kept "so the door can be re-resolved if the mesh
  is rebuilt". There is no re-resolve API. After a rebuild you must reconstruct the
  `DoorSet` from scratch.
- **(g)** `tiled.rs`'s module header describes "streaming a tile in or out" as an
  `add_tile` / re-stitch. There is no `remove_tile`; streaming out means rebuilding the
  `TiledWorld`.
- **(h)** `set_tile_offset`'s doc says it "invalidates the links". Nothing is cleared —
  stale portals stay live and routable until the next `stitch_all`.
- **(i)** Two doc comments call `0.128` the *default* inset —
  [polygon-extract/src/lib.rs:677](../crates/polygon-extract/src/lib.rs) ("`BuildOptions::inset`
  defaults to 0.128 cells") and [dynamic/src/lib.rs:111](../crates/dynamic/src/lib.rs) ("the
  0.128 default"). `BuildOptions::default()` sets `inset: None`. 0.128 is a *typical* agent
  radius and the demo's value, not a default; `BuildOptions::inset`'s own doc words it
  correctly as "e.g. 0.128". [04](04-units-and-conventions.md) and [06](06-clearance.md) say
  `None`, and they are right.
- **(j)** `LineOfSightResult::Indeterminate`'s doc tells callers to treat the result
  conservatively, naming "stop a visibility ray short" as an example. `visibility_region`
  does the opposite — it keeps the ray at full length, deliberately and with its own
  rationale ([visibility.rs:73](../crates/navigation/src/visibility.rs)), because a spurious
  *short* ray punches a false notch in the polygon. Both behaviours are defensible; the two
  comments contradict each other. See
  [§`Indeterminate` constantly](#lineofsightresultindeterminate-constantly).
