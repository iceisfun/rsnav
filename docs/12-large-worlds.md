# Big worlds: placement, tiles, and seams

Prerequisites: you can build a navmesh from a grid ([05](05-building-navmeshes.md))
and get a path out of it ([07](07-paths-and-queries.md)). You have read the
clearance decision page ([06](06-clearance.md)) — this page constrains which of
its three mechanisms you are allowed to use.

Two different problems get confused here, and they have different answers:

* **Placement.** You have one mesh and you want it at world coordinates
  `(10000, 50000)`. This is a translation. Use [`NavMesh::translate`](../crates/navmesh/src/navmesh.rs).
* **Tiling.** You have many meshes that together cover a world too large or too
  dynamic to triangulate as one. This needs seams. Use
  [`TiledWorld`](../crates/navigation/src/tiled.rs).

They look similar because both involve an offset. They are not
interchangeable, and combining them for the same placement applies the shift
twice. Pick one per placement.

Out of scope on this page: streaming and level-of-detail architecture, which
this library does not support — see [The honest limits](#the-honest-limits-of-tiledworld-v1)
for why. Erosion *mechanism* belongs to [06](06-clearance.md); this page owns
only the *ordering* constraint. Serialization is [14](14-saving-and-loading.md).
Doors and metadata are [09](09-doors-and-navworld.md) — `TiledWorld` supports
neither, and there is no bridge between `NavWorld` and `TiledWorld`.

---

## Placement: translate once, before the BVH

Triangulate in local coordinates near the origin. That is where the CDT's
predicates are numerically happiest, and it costs nothing — the finished mesh
is what gets placed.

The authoritative statement of the choice is the header of
[`crates/navigation/examples/translate_vs_tiled.rs`](../crates/navigation/examples/translate_vs_tiled.rs),
quoted here rather than paraphrased:

> * `translate` **bakes** the placement: one mesh, placed once, every
>   coordinate in the mesh is a world coordinate afterward. All the
>   single-mesh machinery (`Bsp`, `find_path`, `line_of_sight`,
>   `visibility_region`) works in world space with no wrapper. Translate
>   *before* `Bsp::build` — a BVH built earlier holds stale absolute
>   AABBs.
> * `TiledWorld` is a **view transform**: the mesh stays local and
>   reusable (the same tile can be added at many origins), the offset can
>   change later (`set_tile_offset`), and seams to neighboring tiles can
>   be stitched. The cost is going through the `TiledWorld` query surface.
>
> Pick one per placement — adding a pre-translated mesh as a tile with a
> non-zero offset applies the shift twice.

Run it: `cargo run -p rsnav-navigation --example translate_vs_tiled`. It
places the same local room at `(10000, 50000)` both ways and shows the
polylines match.

### Two order rules, both silent when violated

```rust
let mut nav = build_local_mesh();
nav.translate(WORLD_ORIGIN);   // first
let bsp = Bsp::build(&nav);    // then
```

**Translate before `Bsp::build`.** The BVH stores absolute per-triangle AABBs.
A tree built before the translate indexes coordinates the mesh no longer has.
There is no error: `locate` returns `None` for points that are inside, and
`nearest` returns nonsense. The same hazard applies to `WallClearance`, which
caches vertex *positions*, not ids —
[`WallClearance::from_walls`](../crates/navigation/src/wall_clearance.rs) copies
`nav.vertex(a)` into its segment list at build time. Rebuild both after any
translate.

**Never combine translate with a tile offset for the same placement.**
`TiledWorld` adds `tile_offset` at query time
([`world_vertex`](../crates/navigation/src/tiled.rs)). A pre-translated mesh
added at a non-zero offset is displaced by the sum. The symptom is a world
that is exactly twice as far from the origin as intended, which reads as a
coordinate-system bug rather than a placement bug.

### `NavMesh::append` merges geometry, not connectivity

[`NavMesh::append`](../crates/navmesh/src/navmesh.rs) is the merge primitive
behind per-region parallel building. It remaps vertex ids by the old vertex
count, neighbour ids by the old triangle count (`TriangleId::INVALID` is never
offset), and region ids by the old `region_count`.

Its documented assumption is load-bearing: **the two meshes are disjoint.** No
vertex is deduplicated and no adjacency is created across the join. Two meshes
that touch geometrically — a corridor mesh appended to a room mesh, sharing an
edge exactly — still have no neighbour links across that edge, land in
different regions, and `nav.reachable(a, b)` returns `false` for a join an
agent could physically walk. `find_path` fails with `Unreachable`.

`append` is for combining pieces that do not touch. If your pieces do touch
and must connect, you need either one triangulation covering both, or
`TiledWorld` with a stitched seam.

---

## Tiling: place, then stitch

```rust
let mut world = TiledWorld::new();
world.add_tile(nav_a, Vertex::new(0.0, 0.0));
world.add_tile(nav_b, Vertex::new(32.0, 0.0));
world.stitch_all(1e-9);
assert!(!world.links().is_empty());
```

[`add_tile`](../crates/navigation/src/tiled.rs) takes the mesh by value,
builds its `Bsp` eagerly, and records `world_aabb = nav.aabb + offset`. It does
**not** stitch — a tile added after a previous `stitch_all` stays unlinked
until you stitch again.

[`stitch_all(tol)`](../crates/navigation/src/tiled.rs) rebuilds every link from
scratch. For every pair of tiles it matches boundary edges whose *world-space*
segments are collinear within `tol` and overlap by more than `tol`, emitting one
`Link` per overlapping pair carrying the clipped overlap as its funnel portal.
Vertices need not match: a long border edge in one tile links to each of the
several shorter edges it overlaps in its neighbour.

`tol` is world-space slack. Use `1e-6` for exact grids; the in-tree tests use
`1e-9` because tiles cut on integer cell boundaries match exactly and the
tolerance only has to absorb f64 noise.

**`world.links().is_empty()` after stitching is the canonical "my seams
failed" signal.** There is no error path. A world with zero links routes
within tiles and returns `None` for anything crossing a seam.

When links are zero, the next diagnostic is: did the boundary geometry survive
on the seam line at all? Count the boundary edges that lie exactly on it. This
helper is lifted from
[`crates/navigation/tests/tiled_erosion_seams.rs`](../crates/navigation/tests/tiled_erosion_seams.rs) —
it is `#[cfg(test)]` code there, not public API, so copy it into your own
tooling:

```rust
/// Boundary edges of `nav` lying entirely on the world line `x = SEAM_X`
/// once the tile is placed at `offset_x`.
fn seam_edges(nav: &NavMesh, offset_x: f64) -> usize {
    nav.boundary_edges()
        .filter(|be| {
            let (a, b) = (nav.vertex(be.from), nav.vertex(be.to));
            (a.x + offset_x - SEAM_X).abs() < 1e-9 && (b.x + offset_x - SEAM_X).abs() < 1e-9
        })
        .count()
}
```

If that count is 0 on either side, no tolerance will recover it. The geometry
is gone and the cause is almost always the ordering rule below.

---

## The ordering rule

An agent radius in a tiled world comes from **grid erosion applied globally,
before slicing**. Two calls, and their order is the whole trick:

```rust
let eroded = global.eroded(&ErodeOptions { radius, threads: 0 })?; // 1. globally
let cells  = eroded.subgrid(tx * TILE, ty * TILE, TILE, TILE);     // 2. then slice
```

Slicing an already-eroded grid leaves each tile's boundary exactly on the tile
border line, at identical integer coordinates in both neighbours. `stitch_all`
matches it exactly.

**Never erode a tile.** `Bitfield::at` reads out-of-range cells as `false`,
i.e. everything outside the grid is wall, so eroding a `subgrid` treats the
tile border as a wall and eats `radius` cells at every seam. Both tiles then
have zero boundary edges on the seam line, `stitch_all` matches nothing, and
`find_path` returns `None`.

**`BuildOptions::inset` must stay `None` for tiled builds.** Contour inset is
the same failure by a different route: it recedes each tile's boundary edges
by `r` off the tile line, so there is nothing collinear left to match. This is
pinned as a negative control in
[`tiled_erosion_seams.rs::per_tile_contour_inset_breaks_the_seam`](../crates/navigation/tests/tiled_erosion_seams.rs),
which asserts `(seam_edges_left, seam_edges_right, links) == (0, 0, 0)` for
inset `0.5` and `1.0` and that no cross-seam path exists. The positive
direction is pinned by
`global_erosion_then_subgrid_keeps_seams_linked`, which checks radii `0.0` and
`1.0` against `diagonal_smoothing` both on and off.

Both directions are also demonstrated as runnable output by
[`crates/navigation/examples/tiled_build.rs`](../crates/navigation/examples/tiled_build.rs):

```
cargo run --release -p rsnav-navigation --example tiled_build
```

```
erode globally, then subgrid  <- do this
  tiles            4
  seam edges @x=32 4
  cross-tile links 4
  path             5 points, length 94.96

subgrid, then erode each tile  <- broken, silently
  tiles            4
  seam edges @x=32 0
  cross-tile links 0
  path             None
```

Same grid, same radius, same tile layout. The only difference is which of the
two calls ran first.

### Two build options that act asymmetrically between neighbours

Both of these can drop a fragment in one tile and not its neighbour, which
breaks a seam that both tiles individually look fine on:

* `ExtractOptions::min_area` culls a region below a threshold. A seam-adjacent
  sliver can fall below it on one side of the cut and not the other. **Set
  `min_area = 0.0` for tiled builds.**
* `BuildOptions::clip_ears_max_area` shaves small ear triangles, and the ear
  it shaves may be the one whose wall edge lies *on* the seam. **Drop it to
  `0.0` if a seam ever fails to link.** The `0.6` default is tuned for
  unit-cell stair artifacts, not for seam preservation.

`tiled_build.rs` sets both to `0.0`.

---

## The honest limits of TiledWorld v1

These determine whether `TiledWorld` is usable for your project at all, so
read them before building on it.

**No clearance.** [`TiledWorld::find_path`](../crates/navigation/src/tiled.rs)
takes no `PathOptions` and nothing in `tiled.rs` applies `distance_from_wall`.
The module header scopes it out for v1. Consequence, stated plainly: the only
routes to an agent radius in a tiled world are **global grid erosion before
tiling** and **`WallClearance` applied outside the `TiledWorld` surface**, per
tile against `tile_nav()` in local coordinates. The baked-inset and
`PathOptions` routes from [06](06-clearance.md) are both unreachable here.

**No error detail.** `find_path` returns `Option<Vec<Vertex>>`, so "start
off-mesh", "goal off-mesh" and "unreachable" collapse into one `None`. The
single-mesh `find_path` distinguishes them with `PathError`.

**Query cost scales with the whole world, not the explored region.** A* in
`tiled.rs` allocates four scratch `Vec`s of length `total_tris` across *all*
tiles per call — `g_score`, `came`, `entry`, `closed` — plus the heap. This is
the opposite of what a tiled design usually buys you.

**`locate` is a linear scan.** It walks tiles in insertion order with an AABB
reject and returns the first hit. There is no world-level spatial index over
tiles, and `locate` runs at the head of both `find_path` and `line_of_sight`,
so per-query cost grows with tile count. If two tiles overlap in world space,
which one you get depends on insertion order.

**`stitch_all` is the real scaling wall.** It is O(tiles²) pairs, and for each
surviving pair O(|boundary edges a| × |boundary edges b|) with no spatial index
over edges. Worse, `boundary_world_edges` re-collects a fresh `Vec` of a tile's
full boundary on every pair it participates in, so each tile's boundary is
materialized O(tiles) times per stitch.

**There is no `remove_tile`.** The module header describes streaming a tile in
or out as an `add_tile` / re-stitch, but no removal API exists (errata (g) —
see [16](16-troubleshooting.md)). Streaming a tile *out* means rebuilding the
`TiledWorld` from scratch. Combined with `stitch_all`'s cost, this is why there
is no streaming architecture advice on this page: the primitives to support it
are not here.

**`set_tile_offset` does not clear links.** Its doc says it invalidates them,
but nothing is actually cleared (errata (h)). The stale `Link`s keep their old
world-space `portal` segments and remain fully routable by A*, the funnel and
`line_of_sight` until the next `stitch_all`. Between moving a tile and
re-stitching, the world will plan paths through portals that are no longer
where the geometry is. The same applies to `add_tile`: the new tile is
unlinked, and its neighbours' old links are still live.

**Two tolerances, one configurable.** `stitch_all(tol)` controls matching, but
`link_across` — the LOS crossing test — hardcodes `1e-6` independent of it.
Stitching noisy borders with a large `tol` can therefore produce links that
`find_path` crosses happily and `line_of_sight` refuses to cross.

**No doors.** `tiled.rs` consults `is_wall_edge_local` directly with no
`WallInfo` overlay, so a `DoorSet` is invisible to a `TiledWorld`, and `Link`
carries no open/closed state. See [09](09-doors-and-navworld.md) for what you
give up.

**Translation only.** Tile offsets are translations; there is no rotation.

**Slight bend at T-junction seams.** Where one long edge links to two shorter
ones, the shared vertex is a T-junction the v1 funnel treats as a soft corner,
so the path bends slightly at the seam. Across an aligned grid with matching
seam vertices the path is exact. Exact paths at mismatched seams would need
collinear link portals merged; that is deferred.

---

## Choosing

| You have | Use |
|---|---|
| One mesh, one world position | `NavMesh::translate`, then `Bsp::build` |
| Pieces that do not touch | `NavMesh::append` |
| Pieces that touch and must connect | one triangulation, or `TiledWorld` + `stitch_all` |
| A mesh reused at several origins | `TiledWorld` (the mesh stays local) |
| A world needing doors, metadata, or `distance_from_wall` | one mesh + `NavWorld` ([09](09-doors-and-navworld.md)) — `TiledWorld` has none of these |

For an interactive view of placed and stitched tiles, run the world demo:
`cargo run --release -p rsnav-world-demo`.

## Further reading

* [06 — Clearance](06-clearance.md) for what erosion actually does to your radius.
* [16 — Troubleshooting](16-troubleshooting.md) for the seam symptom index.
* [`crates/navigation/src/tiled.rs`](../crates/navigation/src/tiled.rs) module
  header for the link model in the source's own words.
