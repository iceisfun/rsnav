# Building a navmesh from a grid

You have run [the quickstart](01-quickstart.md) against a toy map. Your real map is a
tilemap, a PNG, a heightmap threshold, or procedural output, and now the build has to be
correct rather than merely present. This page is about that: how the grid becomes a mesh,
which options change the answer, which errors mean what, and — the part worth reading twice
— which failures produce wrong geometry instead of an error.

Prerequisites: [01-quickstart](01-quickstart.md) for the shape of the call, and
[04-units-and-conventions](04-units-and-conventions.md) for the grid conventions this page
assumes and does not restate (`true` = walkable, row 0 is the bottom row, one cell = 1.0
world unit, 4-connectivity, CCW outer rings and CW holes).

Agent radius and clearance are deliberately absent here. `BuildOptions::inset` exists and it
bakes an agent radius into the mesh; it is one of three mechanisms and choosing between them
is a decision with silent failure modes, so it lives entirely in
[06-clearance](06-clearance.md). Authored polygon input rather than a grid is
[13-authored-geometry](13-authored-geometry.md). Threads and timings are
[15-performance-and-determinism](15-performance-and-determinism.md).

## The pipeline

[`build_navmesh_from_bitfield`](../crates/dynamic/src/lib.rs) is the whole thing in one
call. Its stages, in the order the source runs them:

```
Bitfield
  │  extract()                      trace walkable cells into rings
  ▼
Vec<PolygonWithHoles>               outer ring CCW + hole rings CW, per region
  │  per region, independently:
  │    region_pslg()                rings -> PSLG vertices + segments + hole seeds
  │    delaunay()                   divide-and-conquer Delaunay of the ring vertices
  │    form_skeleton()              force the ring segments in as constrained edges
  │    carve_holes()                flood-fill from each hole seed, delete what it reaches
  │    clip_ears()                  drop stair-step ear triangles
  │    build_from_cdt()             -> a NavMesh for this region alone
  ▼
NavMesh::append() in extraction order
  ▼
Bsp::build()
  ▼
NavBuild { navmesh, bsp, build_ms, generation }
```

Two structural facts follow from this shape and matter later.

Regions are built **independently**, because extraction guarantees they are geometrically
disjoint. That is what makes the per-region CDT parallelisable, and it is why the merge is a
plain `append` with no vertex dedup and no adjacency across the join — see
[12-large-worlds](12-large-worlds.md) for the consequences when you append meshes yourself.

The merge runs in **extraction order** regardless of which thread finished which region, so
the output bytes do not depend on the thread count. `extract` emits regions in border-edge
discovery order (row-major over cells), and that ordering is the determinism anchor for the
whole library ([15](15-performance-and-determinism.md)).

`build_navmesh_from_bitfield` returns a `NavBuild`; `generation` is always `0` for direct
callers and is only filled in by `NavWorker` ([10-dynamic-rebuilds](10-dynamic-rebuilds.md)).
For a stage-by-stage timing decomposition against a real corpus, the written-out version of
this pipeline is [`stage_bench.rs`](../crates/dynamic/examples/stage_bench.rs).

## Getting your world into a Bitfield

`Bitfield` is three public fields — `width`, `height`, `data: Vec<bool>` — and one checked
constructor. `Bitfield::new` returns `BitfieldError::BadDataLength` when
`data.len() != width * height`. Nothing enforces that invariant afterwards: the fields are
public, so a later `data.push(..)` produces a bitfield that will read garbage. Construct
through `new`, then treat the fields as read-only or go through `set`.

`Bitfield::at(col: i64, row: i64)` takes signed coordinates on purpose and returns `false`
out of range — everything outside the grid is wall. `Bitfield::set(col: u32, row: u32, v)` is
unchecked and panics on an out-of-range index.

**A tile enum array.** The common case. One `match` per cell, and the row flip if your array
is stored top-down:

```rust
let mut data = vec![false; (w as usize) * (h as usize)];
for row in 0..h as usize {
    for col in 0..w as usize {
        // tiles[] is top-down; the Bitfield's row 0 is the BOTTOM row.
        let tile = tiles[(h as usize - 1 - row) * w as usize + col];
        data[row * w as usize + col] = matches!(tile, Tile::Floor | Tile::Grass);
    }
}
let bf = Bitfield::new(w, h, data)?;
```

**An image or PBM.** Same flip, because image row 0 is the top row.
[`erode_vs_inset.rs`](../crates/dynamic/examples/erode_vs_inset.rs) carries a compact
self-contained P4 PBM reader that does exactly this
(`cells[(h - 1 - row) * w + col]`); it is the canonical in-tree instance of the convention.

**Procedural.** Threshold whatever field you generate. The thing to watch is speck count: a
noise threshold typically produces hundreds of one- and two-cell islands, each of which
becomes its own region with its own two triangles. `ExtractOptions::min_area` is the cull.

For a small ASCII fixture — the fastest way to reproduce a bug — the `grid()` helper in
[`grid_to_polygons.rs`](../crates/polygon-extract/examples/grid_to_polygons.rs) and in
[`mesh_anatomy.rs`](../crates/dynamic/examples/mesh_anatomy.rs) parses `#`/`.` rows written
top-down and flips them. `grid_to_polygons.rs` also contrasts the default against
`diagonal_smoothing = false` on a staircase, which is the only input where the two differ.

## ExtractOptions

Four fields, defined at
[`polygon-extract/src/lib.rs:112`](../crates/polygon-extract/src/lib.rs) with `Default` at
`:131`. Only the first three change the geometry.

**`min_area: f64`, default `0.0`.** Drop regions whose outer polygon area is strictly less
than this. Three details decide whether it does what you want:

- It is measured **after** `diagonal_smoothing`, so a speck's area is its smoothed area, not
  its cell count.
- It tests `outer.area()` **only**. Hole area is not subtracted, so a large donut with a
  large hole is never culled on the strength of its small net walkable area.
- A culled region takes **its holes with it**, which is correct — they were only ever holes
  in that region.

`min_area <= 0.0` disables the pass entirely (the check is `if opts.min_area > 0.0`), so a
negative value is a silent no-op rather than an error. Set it to somewhere between 2.0 and
8.0 to kill single-cell noise; leave it at `0.0` for tiled builds, where culling a
seam-adjacent fragment in one tile and not its neighbour breaks stitching
([12](12-large-worlds.md)).

**`remove_collinear: bool`, default `true`.** Leave it on. The raw trace emits a vertex at
every unit cell corner along the perimeter, so turning this off is not a small change: the
in-crate test at `lib.rs:1306` takes a 3x3 walkable block from 4 vertices to 12. Every one of
those vertices reaches the CDT. The only reason to turn it off is if you need the exact
cell-aligned boundary for something other than pathfinding.

**`diagonal_smoothing: bool`, default `true`.** Replaces stair-step zigzags along
axis-aligned edges with single diagonals, iterating to a fixed point. On a diagonal wall this
is a large triangle-count win and it is why it defaults on.

It is **not area-preserving**. At a reflex corner it replaces a stair pair with a diagonal
that cuts into the wall by up to `sqrt(2)/2 ≈ 0.708` cells. If nothing downstream depends on
an exact clearance bound, that is invisible. If you are relying on a guaranteed distance
between the mesh and the wall, it is not — the smoothing runs *after* erosion, inside
`extract`, and eats into the margin you paid for. That is the one reason to set this to
`false`, and the arithmetic is in [06-clearance](06-clearance.md). The trap is that the two
options live in different structs in different crates, so nothing puts them side by side.

**`threads: usize`, default `0`.** `0` = one per available core, `1` = fully serial. Output
is byte-identical at every setting. Left at `0` it inherits `BuildOptions::threads`; set
explicitly it wins. See [15](15-performance-and-determinism.md).

## BuildOptions

Defined at [`dynamic/src/lib.rs:68`](../crates/dynamic/src/lib.rs), `Default` at `:129`.
Note it is `Clone + Debug` but **not** `Copy`, unlike `ExtractOptions`.

| field | default | what to think about |
|---|---|---|
| `extract` | `ExtractOptions::default()` | forwarded, with the `threads` inheritance above |
| `perimeter_marker` | `1` | marker written onto outer-ring constraint edges |
| `hole_marker` | `2` | marker written onto hole-ring constraint edges |
| `threads` | `0` | governs the per-region CDT and, by inheritance, `extract` |
| `clip_ears_max_area` | `0.6` | see below |
| `inset` | `None` | baked agent radius — [06-clearance](06-clearance.md) |

The two markers reach `NavTriangle::edge_markers` and are the only thing distinguishing an
outer wall from a hole wall at runtime. If you classify walls by marker anywhere — rendering,
audio, "is this an exterior wall" — you are depending on values you chose here. Marker
semantics are [04](04-units-and-conventions.md).

**`clip_ears_max_area: f64`, default `0.6`.** After carving, `clip_ears` deletes triangles
with exactly two constrained edges, one interior neighbour, and area below this threshold,
promoting the surviving neighbour's exposed edge to a constraint. It iterates to a fixed
point, since clipping one ear can expose another.

The `0.6` is tuned for **unit-cell bitfield input**: half-cell stair-step artifacts have area
exactly `0.5`, and `0.6` catches them with margin. It is an absolute area in your coordinate
scale, so it does not survive a change of scale. Feed a hand-authored PSLG at a different
scale through `BuildOptions` without rescaling this and it will either silently eat real
geometry or silently do nothing ([13](13-authored-geometry.md) owns that case). `0.0`
disables the pass — which is also what you want for exact area comparisons, and what
[12](12-large-worlds.md) recommends when a tile seam refuses to link.

Clipping only ever removes area, so it can never reduce the distance between the mesh and a
wall.

## Errors

`BuildError` ([`dynamic/src/lib.rs:157`](../crates/dynamic/src/lib.rs)) implements
`Display` and `std::error::Error`.

- **`NoPerimeter`** — `extract` returned zero regions. Either the grid has no walkable cells
  at all (the usual cause: `false` is wall, so `Bitfield::empty()` is a solid block, and a
  bitfield built from an inverted predicate is a solid block too), or `min_area` culled
  everything. Its own doc comment blames a field named `min_polygon_area`, which does not
  exist anywhere in the workspace; the real field is `ExtractOptions::min_area`.
- **`EmptyMesh`** — the pipeline ran but produced zero live triangles after carving. On grid
  input this means every region was fully consumed, which in practice means a baked inset
  large enough to erase the map ([06](06-clearance.md)).
- **`SegmentInsertion(SegmentInsertError)`** — `form_skeleton` rejected a constraint. From
  grid input this should not happen: `extract` produces non-crossing rings. If you see it,
  the input is not what you think it is. Crossing rings are what
  [13](13-authored-geometry.md)'s inset path exists for.
- **`InvalidInset(f64)`** — `inset` was `Some(r)` with `r` negative, NaN, or infinite.
  `build_navmesh_from_bitfield` validates before calling, which is the only reason this is a
  typed error; the triangle-level entry point asserts and panics instead.
- **`Planarize(PlanarizeError)`** — only reachable on the inset path.
- **`Panicked(String)`** — produced only by `NavWorker`, which catches the unwind so one bad
  snapshot degrades to a failed build instead of killing the thread. Direct callers of
  `build_navmesh_from_bitfield` get the panic itself.

## Reading the built mesh

[`mesh_anatomy.rs`](../crates/dynamic/examples/mesh_anatomy.rs) walks a built mesh and
prints every accessor described here:

```
cargo run --release -p rsnav-dynamic --example mesh_anatomy
```

On its 24x12 fixture:

```
bitfield: 24x12 = 288 cells, 161 walkable
navmesh:  16 vertices, 12 triangles, 3 region(s), built in 0.198 ms
regions:
  region 0:   2 tris, area   18.00, bounds (2.0,1.0)..(8.0,4.0), centroid (5.00,2.50)
  region 1:   2 tris, area   27.00, bounds (14.0,1.0)..(23.0,4.0), centroid (18.50,2.50)
  region 2:   8 tris, area  116.00, bounds (1.0,5.0)..(23.0,11.0), centroid (12.41,8.00)
boundary: 16 edges, total length 118.00 (markers seen: [1, 2])
```

**Regions.** `NavMesh::region_count` and `NavTriangle::region` label connected components
under the relation "two triangles connect iff their shared edge is not constrained". A region
is not a room, not a zone, and not anything you authored — it is exactly what
`NavMesh::reachable(a, b)` compares, and it is A*'s O(1) pre-check. Accessors, all on
[`navmesh/src/navmesh.rs`](../crates/navmesh/src/navmesh.rs):

- `region_triangles(r) -> impl Iterator<Item = TriangleId>`
- `region_area(r) -> f64` — sum of triangle areas; `0.0` for an id with no triangles
- `region_bounds(r) -> Option<Aabb>`
- `region_centroid(r) -> Option<Vertex>` — area-weighted, and therefore **not guaranteed to be
  inside the region**; for a non-convex region use `random_point_in_region` if you need an
  interior point

All four are linear scans over the whole triangle array. Fine for setup and diagnostics;
cache the results if you want them per frame.

**Spawn points.** `random_point(rng)` and `random_point_in_region(region, rng)` take any
`FnMut() -> f64` yielding uniform values in `[0, 1)` and consume three per call: one to pick
a triangle weighted by area, two for a barycentric point inside it. The result is uniform
over walkable **area**, not over triangles, which is what you want on a mesh where one
triangle can be a whole courtyard. `O(triangles)` per call, over a linear area CDF — good for
spawn placement, not for a hot loop.

Passing your own closure rather than an rng type is what keeps the crate dependency-free and
lets you reproduce a spawn set exactly; `mesh_anatomy.rs` uses a splitmix64 closure so its
output is identical on every machine.

**Outlines.** `boundary_edges() -> impl Iterator<Item = BoundaryEdge>` yields every edge with
no triangle on the far side — the outer rim of each region plus every hole rim, each exactly
once, directed so the walkable interior is on the left. `BoundaryEdge` carries `triangle`,
`from`, `to`, `marker`. This is the playable-area outline for debug overlays or for exporting
the mesh back to a ring set.

Interior walls with a live triangle on *both* sides are not boundary edges and will not
appear. Grid input does not produce those; hand-authored region-splitting segments do, and
you find them with `NavTriangle::is_edge_constrained(i)`.

**One triangle.** `NavTriangle` is `vertices: [VertexId; 3]` (CCW), `neighbors: [TriangleId; 3]`,
`edge_markers: [i32; 3]`, `area`, `centroid`, `region`. Edge `i` is the edge **opposite**
`vertices[i]`; `edge_vertices(i)` returns `(vertices[(i+1)%3], vertices[(i+2)%3])`;
`neighbors[i]` is `TriangleId::INVALID` on the rim. `NavMesh::vertex` and `NavMesh::triangle`
panic on an out-of-range id, which in practice means an id minted by a different mesh or by
this mesh before a rebuild. IDs are not portable ([04](04-units-and-conventions.md)).

Saving the result to skip the build at runtime is [14-saving-and-loading](14-saving-and-loading.md).

## The silent-failure set

Everything above either works or returns an error. These do neither: they produce a mesh that
builds cleanly, passes every assertion, and is wrong. This is the section to come back to
when the mesh looks fine and the game does not.

**Holes with no enclosing outer ring are dropped without a word.** `extract` parents each hole
to the smallest outer ring containing it; a hole whose parent lookup returns `None` is simply
not pushed anywhere (`polygon-extract/src/lib.rs:195-200`). The trace should never produce
one, so this is a guard rather than a routine path — but it has no diagnostic, so if it ever
fires you will see a hole missing from the mesh and nothing else.

**A hole whose seed point cannot be found is never carved and stays walkable.** `carve_holes`
works by flood-filling outward from a point inside each hole, so its correctness depends
entirely on that point existing. The dynamic path gets it from
`Polygon::interior_point()` (`dynamic/src/lib.rs:346`), which returns `Option` — `None` for
fewer than three vertices or a degenerate winding — and the `if let Some(seed)` around it
means a `None` silently contributes no `PslgHole`. The hole ring is still inserted as
constrained edges, so what you get is a hole outlined by walls with walkable triangles inside
it: agents can path into a building and not out of it.

`interior_point` exists specifically because the arithmetic centroid of a concave C, L, or U
shape often falls outside the polygon, which would flood-fill the wrong region entirely. It
finds a convex vertex whose ear triangle is empty and returns that triangle's centroid
(`common/src/polygon.rs:152`). It is `O(n^2)` worst case and it can, pathologically, find no
ear.

**On the inset path, vanished input geometry has no diagnostic anywhere.**
`build_cdt_with_inset` returns `InsetBuild::skipped_rings` listing rings it dropped at entry,
and its own doc says callers must surface it because a skipped perimeter should be a build
error. `rsnav-dynamic` takes only `built.mesh` (`dynamic/src/lib.rs:397`) and discards it. A
degenerate perimeter through this path therefore becomes an empty contribution to the merge
with no error, no log, and no counter. Worse, `skipped_rings` does not even cover everything:
a perimeter whose bounding box is too small to survive the inset is dropped earlier still and
never recorded ([13](13-authored-geometry.md)).

**Corner-touching areas are separate regions.** 4-connectivity is a convention, not a bug, but
two rooms joined only at a diagonal produce two regions with no route between them and no
indication that they were ever meant to connect. `mesh_anatomy.rs`'s three-region output and
the `corner_touching_cells_are_separate_regions` test at
`polygon-extract/src/lib.rs:1318` are both worth looking at once.

When one of these bites, [16-troubleshooting](16-troubleshooting.md) indexes them by what you
observed rather than by what caused it.
