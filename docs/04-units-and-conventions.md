# Units, coordinates, winding, and markers

This page is the contract. Every convention in rsnav is defined here once, and
every other page links here instead of restating it. Read it after your first
path works ([01-quickstart](01-quickstart.md)) and before you touch any option,
because most of what follows fails *silently* when you get it wrong: you do not
get an error, you get a world that is upside down, a region that vanished, or an
agent whose clearance is counted twice.

Nothing here is a decision. Which clearance mechanism to use is
[06-clearance](06-clearance.md); this page only fixes what the numbers *mean*.

---

## The grid

The normative statement lives in the module header of
[`crates/polygon-extract/src/lib.rs`](../crates/polygon-extract/src/lib.rs)
lines 3-16. Restated:

| Rule | Consequence |
|---|---|
| Row-major: `data[row * width + col]` | Indexing is by row first, always. |
| `true` = walkable, `false` = **wall** | `Bitfield::empty()` is a solid block, not an open map. It builds to `BuildError::NoPerimeter`. |
| Cell `(col, row)` occupies `[col, col+1] x [row, row+1]` | Cell centers sit at half-integers. A cell's corners are integers. |
| y-axis points **up**; `row = 0` is the **bottom** row | Most image and tile formats put row 0 at the top. You must flip. |
| **4-connectivity** | Two cells touching only at a corner are two separate regions, never one. |
| One cell is exactly **1.0 world unit** | The grid and world share a coordinate system with no scale factor. |
| Everything outside the grid reads as wall | See [`Bitfield::at`](../crates/polygon-extract/src/lib.rs) — it takes `i64` on purpose and returns `false` out of range. |

Two of these have teeth.

**The row flip.** PBM, PNG, and nearly every tilemap put row 0 at the top. The
canonical instance is the P4 reader in
[`crates/dynamic/examples/erode_vs_inset.rs`](../crates/dynamic/examples/erode_vs_inset.rs):

```rust
cells[(h as usize - 1 - row) * w as usize + col] = (byte >> (7 - (col % 8))) & 1 == 1;
```

`h - 1 - row` is the whole convention. The ASCII `grid()` helper in
[`crates/polygon-extract/examples/grid_to_polygons.rs`](../crates/polygon-extract/examples/grid_to_polygons.rs)
does the same thing (`let math_row = height - 1 - i;`) so its map literals can
be read top-down. If you skip the flip nothing errors; your world is mirrored
vertically and every path looks plausible until you compare it against your
renderer.

**Outside is wall.** Because `at()` returns `false` beyond the bounds, any grid
erosion with `radius > 0` removes the outermost ring of cells — the border is
treated as a wall it must stand clear of. This is correct and it is a visible
regression for maps whose walkable area runs to the grid edge. Pad the bitfield
by `ceil(radius)` cells and it disappears.

`extract` with default options on a 2x2 grid of two corner-touching walkable
cells returns **2** regions, which the example prints:

```
corner-touching pair: 2 region(s)
```

---

## The scale contract

One cell is 1.0 world unit. There is no `cells_per_metre` anywhere in the
library, so the mapping from your game's units to rsnav's is entirely decided by
how finely you rasterize. That single choice determines which clearance
mechanisms are even *available* to you, because grid erosion cannot resolve
below one cell.

Let `s` be cells per world metre and `R` the agent radius in metres. The agent
radius in rsnav units is `R * s`.

| `R * s` | What you can use |
|---|---|
| `< 1.0` | Contour inset only (`BuildOptions::inset`), or query-time clearance. Grid erosion quantizes your request up to a full 1.0 peel. |
| `>= 1.0`, near an achievable step | Grid erosion works, and is the only option compatible with `TiledWorld`. |
| `>= 1.0`, between steps | Grid-erode the integer part, contour-inset the remainder; they compose additively. |

Rule of thumb, and the one thing on this page that is *not* read off the source:
rasterize at roughly **2 to 4 cells per agent radius** if you intend to use grid
erosion, and accept whatever `s` your source art dictates if you intend to use
the contour inset. Nothing in the tree measures or validates that range — it
follows only from wanting `R * s` to land at or above the first achievable
erosion step without paying `O(cells)` on a grid finer than you need. Treat it as
a starting point to measure from, not as a contract like the rest of this page.
A 0.4 m agent at 5 cells/m is 2.0 cells and
lands exactly on an achievable erosion step; the same agent at 1 cell/m is 0.4
cells and grid erosion is off the table.

Achievable grid clearances are exactly `{ sqrt(a^2 + b^2) : a, b in N }` =
`{0, 1, sqrt2, 2, sqrt5, sqrt8, 3, ...}`. See
[06-clearance](06-clearance.md) for why, and for the cost of getting it wrong.

---

## The units table

This is the reason this page exists. Two different numbers in two different
structs mean two different things, and nothing converts between them.

| Setting | Unit | Notes |
|---|---|---|
| `BuildOptions::inset` | **cells** | `Option<f64>`, default `None`. A typical agent radius here is well below 1.0. |
| `ErodeOptions::radius` | **cells** | Default `0.0`. Quantized to whole cells regardless of what you pass. |
| `PathOptions::distance_from_wall` | **world units** | Default `0.0`. Not a Euclidean guarantee — see [06](06-clearance.md). |
| `WallClearance::clamp(pos, radius)` | **world units** | Per-call argument, so one instance serves every agent size. |
| `Agent::radius` (rsnav-crowd) | **world units** | Doubles as the planner's `distance_from_wall` *and* the avoidance disc radius. |
| `ExtractOptions::min_area` | **area**, cell units (one cell = 1.0) | Measured on `outer.area()` only; hole area is not subtracted. |
| `BuildOptions::clip_ears_max_area` | **area**, in whatever coordinate scale the input uses | Default `0.6`, tuned for unit cells (a half-cell stair artifact has area exactly 0.5). |
| `TiledWorld::stitch_all(tol)` | **world units** | Slack for "collinear and overlapping". |
| The egui demo's `inset` slider (0.0..=40.0) | **world units**, on authored polygons | Not comparable to the bitfield inset. |

Because one cell is 1.0 world unit, cells and world units *are* the same
quantity for grid-derived meshes. The distinction still matters for two reasons.
First, on authored (non-grid) geometry there is no cell, so anything documented
in cells is meaningless and anything documented as an area is at your coordinate
scale. Second, the two ends of a pipeline are configured in different structs in
different crates, and mixing a build-time radius with a query-time radius
double-counts — the arithmetic for that lives in
[06-clearance](06-clearance.md).

`clip_ears_max_area` is the one to watch on authored input. Its 0.6 default is
a unit-cell number. Fed a PSLG — a Planar Straight Line Graph, the vertices-plus-
segments form that authored vector geometry takes, as opposed to a grid; see
[13-authored-geometry.md](13-authored-geometry.md) — drawn at, say, 100 units per tile, 0.6 does
nothing; fed one drawn at 0.01 units per tile, it eats real geometry. Rescale it
in proportion or set it to `0.0`. See
[13-authored-geometry](13-authored-geometry.md).

The demo's slider is world units on hand-drawn polygons and its own hover text
says so. Its 0..=40 range and the bitfield pipeline's typical 0.128 are numbers
from two unrelated coordinate systems that happen to feed the same parameter
name.

---

## Winding

| Ring | Winding | Sign of signed area |
|---|---|---|
| Outer / perimeter | counter-clockwise | positive |
| Hole | clockwise | negative |

`extract` produces rings in this orientation, and the CDT — the constrained
Delaunay triangulation, the stage that turns rings into triangles while forcing
your walls to survive as actual triangle edges — expects it. It is a
**producer/consumer contract, not an invariant** — nothing in the type system
or in `PolygonWithHoles` enforces or normalizes it. Four consequences, all in
[`crates/common/src/polygon.rs`](../crates/common/src/polygon.rs):

- The closing edge from `vertices.last()` back to `vertices.first()` is
  **implicit**. Do not repeat the first vertex. A ring with `first == last`
  produces a zero-length edge in `Polygon::edges()` and an extra vertex
  everywhere downstream. (`offset_ring_left` and the inset pipeline's
  `normalize_ring` defensively strip trailing repeats; `Polygon`'s own methods
  do not.)
- `Winding::Degenerate` is returned for *any* ring whose `signed_area2()` is
  exactly `0.0`, which includes rings with fewer than 3 vertices —
  `signed_area2` early-returns 0.0 for `n < 3`. Degenerate therefore conflates
  "zero area" with "not a ring at all".
- `Polygon::ensure_winding(target)` **silently does nothing** on a Degenerate
  ring. Code that assumes "after `ensure_winding(CounterClockwise)` this ring is
  CCW" is wrong for that case.
- `PolygonWithHoles::aabb()` returns only the *outer* ring's AABB. Correct for
  a well-formed region, but not a bound over all stored geometry if a hole
  straddles the perimeter — which the inset pipeline explicitly tolerates as
  input.

`InsetRing` is the one place input winding does not matter: normalization is
internal, and `RingKind` is passed structurally rather than inferred.

---

## Markers

A marker is an `i32` carried from a PSLG segment onto the navmesh edge it
became.

| Value | Meaning |
|---|---|
| `0` | interior / unconstrained edge |
| non-zero | a wall — the value is the originating segment's marker |

Defaults from `BuildOptions`: `perimeter_marker: 1`, `hole_marker: 2`. Those are
*your* choices; any downstream code that classifies walls by marker value
depends on what you set at build time. Both build paths carry the same two
values — `build_navmesh_from_bitfield` passes `perimeter_marker` /
`hole_marker` into the legacy PSLG and into `InsetRing::marker` alike — but
`build_cdt_with_inset` called directly takes a marker per ring from the caller,
and numbering there is entirely caller-specific. Nothing is ever inferred *from*
a marker: `RingKind` is what distinguishes perimeter from hole.

The indexing convention, from
[`crates/navmesh/src/navmesh.rs`](../crates/navmesh/src/navmesh.rs):

> `neighbors[i]` is the triangle sharing the edge opposite `vertices[i]`, or
> `TriangleId::INVALID` if that edge is on the mesh boundary.

So **edge `i` is the edge opposite `vertices[i]`**, and
`edge_vertices(i)` returns `(vertices[(i+1)%3], vertices[(i+2)%3])`. Two
predicates read it: `is_edge_constrained(i)` is `edge_markers[i] != 0`, and
`is_edge_boundary(i)` is `!neighbors[i].is_valid()`. A wall is either.

`NavTriangle::vertices` is in CCW order (positive signed area).

`NavTriangle::region` is a connected-component id under the relation "two
triangles connect iff their shared edge is not constrained". It is not a zone,
a room, or a game concept, and it knows nothing about doors.

---

## Boundary semantics

Every containment test in the library is **closed** — the boundary counts as
inside. No predicate in `rsnav-common` is half-open.

| Predicate | Behavior |
|---|---|
| `geom::point_in_triangle` | boundary-inclusive, winding-agnostic |
| `Polygon::contains` | boundary-inclusive (explicit on-edge test first) |
| `Aabb::contains` | inclusive on all four sides |
| `Aabb::intersects` | touching counts as intersecting |

The consequence that bites: a point exactly on a shared edge is inside *both*
neighbouring triangles. `Bsp::locate` descends left-then-right and returns
whichever it reaches first
([`crates/bsp/src/lib.rs`](../crates/bsp/src/lib.rs), `locate_in`). That is
deterministic for a given tree, and it is not a geometrically meaningful choice.
Do not build logic on which of two triangles you get.

The one deliberate exception sits one crate over, in `rsnav-triangle`:
`winding::winding_number` is half-open in y specifically so a ray through a
vertex is never double-counted, which makes "on the contour" resolve
deterministically rather than inclusively.

---

## IDs

`VertexId`, `TriangleId` and `PolygonId` are `u32` newtypes with
`INVALID = u32::MAX`. `is_valid()` is exactly `!= u32::MAX`.

They are **indices into one particular mesh's arrays**, not handles. From
`NavMesh::vertex`:

> **Panics** if `id` is out of range — most commonly because the ID was issued
> by a different mesh. NavMesh IDs are not portable across instances; pass IDs
> only back to the same NavMesh that produced them.

"A different mesh" includes the same logical mesh after a rebuild. A
`TriangleId` cached across a `NavWorker` swap is at best pointing at unrelated
geometry and at worst panics. The same applies to every derived structure keyed
by id: a `DoorSet`'s resolved edges are vertex-index pairs into a specific mesh
and must be rebuilt from scratch after any rebuild
([09-doors-and-navworld](09-doors-and-navworld.md),
[10-dynamic-rebuilds](10-dynamic-rebuilds.md)).

`Vertex` is `PartialEq` but not `Eq` and not `Hash`. Any deduplication keyed on
position must go through `f64::to_bits`, which is what the planarizer's vertex
pool does.

---

## Where to go next

- Choosing a clearance mechanism, and the arithmetic for combining them:
  [06-clearance](06-clearance.md)
- What `extract` and the CDT actually do with these conventions:
  [05-building-navmeshes](05-building-navmeshes.md)
- Authored (non-grid) input, where the cell conventions do not apply:
  [13-authored-geometry](13-authored-geometry.md)
- Something is wrong and you suspect a convention:
  [16-troubleshooting](16-troubleshooting.md)
