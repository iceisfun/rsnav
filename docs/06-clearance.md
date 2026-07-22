# Keeping agents off walls: pick one of three

An agent is a disc, not a point. `find_path` returns a polyline that, left alone,
runs through the exact corner of every obstacle it rounds — a body of any radius
clips the wall. rsnav offers three unrelated mechanisms to fix that, and they are
not interchangeable:

| | mechanism |
|---|---|
| bake it into the geometry, in contour space | [`BuildOptions::inset`](../crates/dynamic/src/lib.rs) |
| bake it into the geometry, in grid space | [`Bitfield::eroded`](../crates/polygon-extract/src/lib.rs) |
| apply it at query time | [`PathOptions::distance_from_wall`](../crates/navigation/src/path.rs), [`WallClearance::clamp`](../crates/navigation/src/wall_clearance.rs) |

This page exists because choosing wrong **fails silently**. A contour inset in a
tiled world does not error — it disconnects your tiles. A grid erosion at 0.128
does not warn — it erodes by 1.0. `distance_from_wall` does not report that the
clearance it achieved was 0.89 rather than the 1.0 you asked for. Nothing in the
type system distinguishes any of these from working correctly.

Units are the other trap, and they are owned by
[04-units-and-conventions.md](04-units-and-conventions.md): the two baked options
are measured in **bitfield cells**, the two query-time options in **world units**.
Nothing converts between them.

---

## The comparison

| | contour inset | grid erosion | `distance_from_wall` | `WallClearance::clamp` |
|---|---|---|---|---|
| **units** | cells | cells | world units | world units |
| **paid** | at build | at build (before extract) | per query | per call |
| **sub-cell radii** | yes (0.128 is typical) | **no** — quantized to `{0, 1, √2, 2, √5, …}` | yes | yes |
| **works with `TiledWorld`** | **no** — breaks seams | **yes**, and it is the only one that does | not reachable through the `TiledWorld` surface | yes, applied outside it |
| **works on authored, non-grid input** | yes, including rings that cross | no — grid only | yes | yes |
| **cost model** | scales with boundary complexity | `O(cells)`, whatever the boundary looks like | per-path, inside A* and the funnel | `O(wall segments)` × 4, per call, no index |
| **rebuild needed to change radius** | yes | yes | no | no |
| **does NOT guarantee** | true `r` clearance at a protruding wall corner (no arc joins) | the radius you asked for; it delivers the next quantized step up | Euclidean clearance at all — it is a portal shift | convergence, or that the agent stays on the mesh |

The last row is the one people are surprised by. Read the sections below before
treating any of these as "the path stays `r` from every wall".

---

## Choosing

1. **Is your world a `TiledWorld`?**
   Yes → **grid erosion**, run once on the global grid *before* slicing into
   tiles. Contour inset breaks stitching and `TiledWorld::find_path` takes no
   `PathOptions` at all. See [12-large-worlds.md](12-large-worlds.md). Stop here.
2. **Is your source authored vector geometry rather than a grid?**
   Yes → **contour inset**. There is no bitfield to erode. See
   [13-authored-geometry.md](13-authored-geometry.md).
3. **Do agents come in more than one size?**
   Yes and you can afford one mesh per size → bake per size; a single
   [`Bitfield::clearance`](../crates/polygon-extract/src/lib.rs) transform
   thresholded at N radii gives you N grids for roughly the price of one.
   Yes and you cannot → **query-time**, since `distance_from_wall` and
   `clamp(pos, radius)` both take the radius per call.
4. **Is your agent radius smaller than one cell?**
   Yes → **contour inset** or query-time. Grid erosion cannot represent it; a
   0.128 request produces a 1.0 peel.
5. **Does the world change at runtime, with rebuilds you would rather not pay for?**
   Yes → **query-time**. Baked clearance is only as fresh as the last build.
6. **Otherwise** → bake it. A baked radius is checked once at build time instead
   of on every query, and it is the only route that shrinks the mesh itself, so
   A* never routes through space the body cannot occupy. Subject to the corner
   shortfall above: contour inset guarantees this along straight walls, and grid
   erosion guarantees it everywhere once `diagonal_smoothing` is accounted for.

Baking and query-time compose. See [combining them](#combining-them) — they also
double-count if you are not careful.

---

## What each one actually delivers

[`crates/navigation/examples/clearance_three_ways.rs`](../crates/navigation/examples/clearance_three_ways.rs)
runs one map and one agent radius through all four mechanisms and **measures** the
minimum distance from the returned polyline to the walls of the un-eroded
reference mesh — exact segment-to-segment distance, not sampling. Run it:

```
cargo run --release -p rsnav-navigation --example clearance_three_ways
cargo run --release -p rsnav-navigation --example clearance_three_ways -- 0.4
```

At `r = 1.0` on that map (the `build ms` column is machine- and run-dependent;
every other column is deterministic):

```
mechanism             build ms   tris   cells   pts   length  min clear   leg
none (baseline)           0.17     15     540     3   35.476     0.0000     0
inset (contour)           0.16     14     540     4   36.671     0.9119     0
eroded (grid)             0.07     15     381     4   36.749     1.0000     1
distance_from_wall        0.17     15     540     4   35.698     0.8944     2
  + clamp corners         0.17     15     540     4   35.698     0.8944     2
```

Four findings, each of which the rest of this page explains:

- **The baseline is exactly 0.** The funnel apexes on wall vertices, so the path
  touches. That is what all of this is fixing.
- **The contour inset delivers 0.9119, not 1.0.** The offset stage never generates
  arcs; a wall corner protruding into walkable space is mitered or beveled, and
  the join cuts inside the true `r`-offset arc.
- **Grid erosion delivers exactly 1.0** — and at `r = 0.4` it still delivers 1.0,
  along with the same drop from 540 to 381 walkable cells.
- **`distance_from_wall` delivers 0.8944 = 2/√5.** That is `r · sin θ` for the
  angle at which the path leaves the shrunk portal. It is not a rounding error;
  it is the mechanism.
- **`WallClearance::clamp` moved zero vertices.** Every polyline vertex was
  already 1.0 clear; the shortfall lies in the *interior* of a leg. `clamp`
  enforces its invariant at the points you hand it and nowhere else, which is
  why it is a free-movement tool and not a path post-process.

---

## Contour inset — `BuildOptions::inset`

```rust
let opts = BuildOptions::default().with_inset(0.128); // cells
let build = build_navmesh_from_bitfield(&bf, &opts)?;
```

Erosion in contour space: every ring is offset inward by `inset` before
triangulation. `Some(0.0)` erodes nothing but still takes the crossing-tolerant
path; `None` (the default) takes the legacy
`delaunay → form_skeleton → carve_holes` path.

**Sub-cell radii genuinely work.** This is the reason it exists. 0.128 cells is a
typical agent radius on a bitfield where one cell is 1.0 world unit, and it is not
representable on the grid at all.

**It accepts authored input, including rings that cross.** The legacy path returns
`SegmentInsertError::SelfIntersection` on the first segment that crosses an
existing constrained subseg; the inset path planarizes first, so a hole ring that
straddles a perimeter builds. That is worth reaching for at inset 0.0 purely for
the crossing tolerance. The regression that pins this is
[`crates/navmesh/tests/inset_hole_cross_regression.rs`](../crates/navmesh/tests/inset_hole_cross_regression.rs)
(test code). The pipeline itself is owned by
[13-authored-geometry.md](13-authored-geometry.md).

**Cost scales with boundary complexity**, not with area — it moves vertices, and a
mostly-open map has few. See [costs](#costs).

**It does not generate arcs.** `offset_ring_left` miters a corner and falls back to
a bevel past `OffsetOptions::miter_limit` (default 2.0). At a wall corner that
protrudes into walkable space the correct offset is a circular arc of radius `r`
about that vertex; a miter or bevel chord cuts inside it. The measured 0.9119
above is exactly this. Along straight walls the inset is exact.

**It panics rather than errors on a bad radius when called at the triangle level.**
[`build_cdt_with_inset`](../crates/triangle/src/inset.rs) opens with
`assert!(inset.is_finite() && inset >= 0.0, ...)`. Only
`build_navmesh_from_bitfield` converts that case into a typed
`BuildError::InvalidInset`, because it validates before calling.

**It cannot be used with `TiledWorld`.** Per-tile contour erosion pulls each tile's
boundary edges `r` inward off the tile border line, and `stitch_all` matches only
boundary edges that are collinear-and-overlapping in world space — so it matches
nothing. Zero seam edges, zero links, and `find_path` across the seam returns
`None` with no error. Pinned as a negative control by
`per_tile_contour_inset_breaks_the_seam` in
[`crates/navigation/tests/tiled_erosion_seams.rs`](../crates/navigation/tests/tiled_erosion_seams.rs)
(test code), which asserts `(0, 0, 0)` for inset 0.5 and 1.0.

Two smaller edges worth knowing, both at the triangle level: full erosion is
`Ok`-and-**empty** (zero live triangles), not an error — `build_navmesh_from_bitfield`
turns that same case into `BuildError::EmptyMesh` once every region has eroded
away; and a perimeter whose bounding-box minimum dimension is
`<= 2 * inset` is dropped before offsetting as provably fully eroded, without
appearing in `InsetBuild::skipped_rings`. Both are covered in
[13-authored-geometry.md](13-authored-geometry.md).

---

## Grid erosion — `Bitfield::eroded`

Lead with the disqualifying property, because it decides the choice:

**Radii are cell-quantized.** Output cells are whole cells, so the achievable
clearances are exactly `{ √(a² + b²) : a, b ∈ ℕ }` = `{0, 1, √2, 2, √5, √8, 3, …}`
and the result is a step function of the radius that jumps only at those values.
Every radius in `(0, 1]` produces the identical one-cell peel.

Concretely, from
[`crates/polygon-extract/examples/erode_and_clearance.rs`](../crates/polygon-extract/examples/erode_and_clearance.rs):

```
         requested radius   walkable
                      0.0        195
    0.128 (typical inset)        103
                      0.5        103  (same grid as the row above)
                      1.0        103  (same grid as the row above)
                      1.4         43
       sqrt2 = 1.41421...         41
                      1.5         41  (same grid as the row above)
                      2.0         41  (same grid as the row above)
```

A request of 0.128 removes the whole first ring of wall-adjacent cells — a
guaranteed clearance of 1.0, **7.8× more erosion than asked for**. This is not a
bug to work around. It is the reason sub-cell radii belong to the contour path
permanently.

Note also that √2 erodes *more* than 1.4 does. The test is `sq >= ceil(radius²)`
in exact integers, and √2 evaluates in f64 to `2.0000000000000004`, so cells at
exactly √2 clearance are dropped. Ties round toward more erosion, deliberately.

### The reason to use it anyway

It runs on the **grid**, so it can run **once, globally, before the grid is sliced
into tiles**. Every tile's seam edge then still lies exactly on the tile border
line, at identical integer coordinates in both neighbours, and `stitch_all` links
them. That makes it the only clearance mechanism compatible with `TiledWorld`
seams.

The ordering is load-bearing: **erode the global grid, then `subgrid` into tiles.**
Never erode a tile — that treats the tile border as wall and eats `radius` cells
at every seam, reproducing exactly the failure the contour inset has. See
[12-large-worlds.md](12-large-worlds.md).

### The rest of the properties

**Exact Euclidean, `O(cells)`, bit-reproducible.** The implementation is a 3×3
Chebyshev dilation of the wall set followed by a Felzenszwalb–Huttenlocher squared
EDT. Squared distances are exact integers — no float ever enters a distance — so
output is identical across platforms and thread counts. `radius <= 1.0` takes an
exact fast path that skips four of the five passes.

**Any `radius > 0` removes the outermost ring of cells**, because everything
outside the grid reads as wall (`Bitfield::at` takes signed coordinates and
returns `false` out of range). Correct, but a visible regression for maps whose
walkable area runs to the grid edge. Pad the bitfield and it disappears —
demonstrated in section 4 of `erode_and_clearance.rs`.

**One transform, many agent sizes.** `Bitfield::clearance(threads)` returns the
squared-clearance field un-thresholded; `ClearanceField::threshold(radius)` slices
it. The transform is the expensive part and thresholding is a linear pass, so
build one field and threshold it at three radii to get small/medium/large agent
grids for roughly one transform:

```
   clearance() transform: 9.3 us
   threshold(1) ->  103 walkable cells, 0.3 us
   threshold(2) ->   41 walkable cells, 0.3 us
   threshold(3) ->    9 walkable cells, 0.3 us
```

`ClearanceField::sq_at(col, row)` is directly readable: squared clearance in
cells², where `√sq_at` is exactly the largest radius an agent may have while
standing anywhere in that cell.

**`threshold(0.0)` keeps every cell, walls included.** A clearance of at least zero
is vacuously true everywhere, and the field cannot distinguish a wall cell from a
wall-adjacent walkable one — both are `sq == 0`. In the example map, 195 walkable
cells become 288, the full grid. If you want the original grid, use the original
grid, or `eroded()` with radius 0, which clones.

### The `diagonal_smoothing` interaction

Easy to miss, because the two options live in different structs in different
crates. `ExtractOptions::diagonal_smoothing` **defaults to `true`**. It runs
*after* erosion, inside `extract`, and it is not area-preserving: at reflex
corners it replaces a stair pair with a single diagonal that bulges up to
√2/2 = 0.7071… cells **into the wall**. This page, and
[`wall_clearance.rs`](../crates/navigation/src/wall_clearance.rs), quote that
constant as **0.708** throughout — rounded *up*, not to nearest, so that every
`r - 0.708` understates the guarantee and every `r + 0.708` over-erodes. Both
errors land on the safe side.

The guarantees stated at
[`crates/polygon-extract/src/lib.rs:857-876`](../crates/polygon-extract/src/lib.rs)
are therefore:

1. Never over-claims: `R ⊆ S ⊖ D_r`.
2. Sandwich bound: `S ⊖ D_(r + √2) ⊆ R ⊆ S ⊖ D_r` — the one-sided error versus
   true erosion is at most one cell diagonal, always conservative.
3. **Conditional on `diagonal_smoothing == false`.** With smoothing left at its
   default of `true`, the guaranteed clearance is `max(0, r - 0.708)`.

So when a hard clearance bound matters: disable smoothing, or erode by
`r + 0.708`, or use `max(0, r - 0.708)` everywhere downstream. `clip_ears` and
`min_area` only ever *remove* area, which can only increase clearance, so they are
safe.

To verify any specific guarantee claim before relying on it, read
[`crates/polygon-extract/tests/erode_adversarial.rs`](../crates/polygon-extract/tests/erode_adversarial.rs)
— 567 lines of the quantization and sandwich-bound cases. It is test code, not
usage.

---

## Query-time — `distance_from_wall` and `WallClearance::clamp`

### `PathOptions::distance_from_wall` is not a Euclidean guarantee

One number, in world units, drives two stages that must agree:

- **A\* rejects a portal** unless its length strictly exceeds the sum of the
  inward shifts the funnel will apply — `distance_from_wall` for *each* endpoint
  that is a wall vertex. A portal flanked by two walls therefore needs **more
  than `2 · distance_from_wall`** of width. This is the body-width rule; it
  exists so A* never commits to a corridor the funnel would collapse.
- **The funnel shifts each wall-vertex portal endpoint inward *along the
  portal*** by `distance_from_wall`. If the two shifts exceed the portal length
  they are scaled by `len / total_raw`, collapsing the portal to a point rather
  than crossing outside it.

The second bullet is the whole caveat. The shift is along the portal direction,
not along the wall normal. A portal meeting the wall at angle θ therefore moves
the path only about `r · sin θ` away from that wall. The measured 0.8944 = 2/√5 in
the table above is exactly this: the funnel placed the apex 1.0 below a corner
vertex along a vertical portal, then the path left at an angle, and its
perpendicular distance to the corner was `r · sin θ`.

The achieved clearance is therefore `<= distance_from_wall`, and can be
substantially less. **Do not document or reason about `distance_from_wall` as
"the path stays `r` from every wall."**

Two further limits: the path endpoints are the literal `start` and `goal` and are
never adjusted for clearance, so a start already hugging a wall stays there; and
[`path_clear`](../crates/navigation/src/path.rs) ignores clearance entirely, so
a path planned at 0.5 still revalidates as clear after a change leaves it flush
against a new wall. See [07-paths-and-queries.md](07-paths-and-queries.md).

### `WallClearance::clamp` is the only true distance-to-segment invariant

```rust
let clearance = WallClearance::from_navmesh(nav); // once per mesh
let safe = clearance.clamp(pos, radius);          // radius is per-call
```

It precomputes the mesh's wall segments once (`O(triangles)`, canonical-pair
deduplicated so each physical wall is stored once) and pushes a proposed position
back out until its distance to every wall segment is at least `radius`. Because
`radius` is a per-call argument, one instance serves agents of every size.

What it does not promise:

- **No convergence guarantee.** `clamp` runs `RELAX_ITERS = 4` fixed relaxation
  passes and stops. Four passes settle a flat wall and typical concave corners; a
  tight multi-wall pocket can leave the result still inside `radius` of a wall,
  with **no error signal**. The method doc's phrasing ("the nearest position whose
  distance to every wall is at least `radius`") describes the intent, not what a
  fixed-iteration relaxation delivers.
- **No spatial index.** Each pass linearly scans every wall segment in the mesh.
  Cost is `O(wall segments) × 4` per call. On a town-scale mesh with tens of
  thousands of wall segments this is not a per-agent-per-frame operation, and
  neither the module doc nor the method doc states it.
- **It does not keep the agent on the mesh.** It can push a position across a hole
  boundary or off the mesh entirely.
- **It applies to the points you give it and nothing between them.** Clamping a
  polyline's corners does not make the polyline clear — the measured example
  above moved zero vertices while the leg interiors stayed at 0.8944.

Rebuild it whenever the mesh **or any door state** changes;
`from_navmesh_with_doors` folds closed-door edges in as walls. The runtime usage
pattern — `bsp.nearest` first, `clamp` second, and why that order is not optional
— belongs to [08-moving-agents.md](08-moving-agents.md).

---

## Combining them

These compose, and they double-count if you let them.

- **Baked `r`, then query-time.** Pass `max(0, agent_radius - r)` to
  `PathOptions::distance_from_wall` and `WallClearance::clamp`, not the raw agent
  radius. The walls already sit `r` inside the true geometry.
  With a *contour* inset that last sentence holds along straight walls only: at a
  protruding wall corner the miter/bevel join cuts inside the true `r`-offset arc
  ([above](#contour-inset--buildoptionsinset), measured at 0.9119 for `r = 1.0`),
  so the subtraction over-credits the bake by the corner shortfall. The shortfall
  depends on the corner angle and `miter_limit`, so there is no closed form to
  substitute the way there is for smoothed grid erosion below. If corner clearance
  is a hard requirement, measure it on your own geometry with
  `clearance_three_ways` rather than trusting the subtraction.
- **Grid erosion with `diagonal_smoothing` on.** Substitute `max(0, r - 0.708)`
  for `r` when computing that subtraction, because that is the clearance the
  erosion actually guarantees.
- **Grid erosion `a` plus contour inset `b`.** They add: the effective baked
  clearance is `a + b`. This is a supported combination, not an accident — erode
  the integer part on the grid and inset the sub-cell remainder in contour space.
  An agent of radius 2.128 cells is `eroded(2.0)` then `inset: Some(0.128)`.

The arithmetic is stated in-source at
[`crates/navigation/src/wall_clearance.rs:18-37`](../crates/navigation/src/wall_clearance.rs).

---

## Costs

[`crates/dynamic/examples/erode_vs_inset.rs`](../crates/dynamic/examples/erode_vs_inset.rs)
times grid erosion plus a legacy build against a contour-inset build for the same
radius, over the whole `testdata/` corpus.

```
cargo run --release -p rsnav-dynamic --example erode_vs_inset -- 1.0
```

> **Re-measure before trusting these.** The figures below were produced on an
> AMD EPYC 7502P (64 threads), rustc 1.97.1, release profile, at radius 1.0.
> They will not transfer to your hardware. Run the command above on your own
> machine before making a decision on them.

```
file                       cells    erode    build    TOTAL |     inset     tris |   ratio
act3-town.pbm              2611k     4.8m    51.6m    56.4m |    175.6m    17181 |   3.11x
act5-spine.pbm             2217k     4.3m    29.4m    33.7m |    109.7m    12813 |   3.25x
synth-pillars-1024.pbm     1048k     3.6m    35.2m    38.8m |     50.8m    23816 |   1.31x
synth-maze-512.pbm          262k     0.6m     2.3m     3.0m |      2.8m      126 |   0.94x
synth-open-2048.pbm        4194k     5.3m     5.7m    11.0m |      5.0m        2 |   0.45x
synth-open-512.pbm          262k     0.8m     1.4m     2.3m |      1.5m        2 |   0.67x
```

`ratio` is inset time over erode+build time; above 1.0 the grid column wins.

The shape is consistent and follows from the two cost models:

- **Grid erosion wins when boundary complexity is high.** `act3-town` produces
  17k triangles from 2.6M cells; the contour path has to offset and planarize
  every one of those boundary vertices, and pays 3.1× for it.
- **Contour inset wins on large, mostly-open grids.** `synth-open-2048` is 4.2M
  cells and two triangles. Erosion pays `O(cells)` to move a handful of vertices;
  the contour path pays nothing it does not use.

There is no single answer, which is why erosion is opt-in at the call site and
never a build option.

Two cost notes that do not appear in that table:

- **Never benchmark or demo the inset path in a debug build.**
  `carve_by_winding` runs a full brute-force `winding_number` cross-check for
  every triangle under `debug_assert`, as a permanent differential oracle. Debug
  runs are dramatically slower by design.
- Erosion is memory-bandwidth-bound and is internally capped at 16 threads; it
  runs serially below 500k cells. Output is byte-identical at every thread count.
  See [15-performance-and-determinism.md](15-performance-and-determinism.md).

---

## Where to verify a claim

| claim | source |
|---|---|
| what each mechanism measurably delivers | [`crates/navigation/examples/clearance_three_ways.rs`](../crates/navigation/examples/clearance_three_ways.rs) |
| quantization, `ClearanceField`, `threshold(0.0)`, the border peel | [`crates/polygon-extract/examples/erode_and_clearance.rs`](../crates/polygon-extract/examples/erode_and_clearance.rs) |
| erode vs inset cost | [`crates/dynamic/examples/erode_vs_inset.rs`](../crates/dynamic/examples/erode_vs_inset.rs) |
| erosion guarantees and the sandwich bound | [`crates/polygon-extract/src/lib.rs`](../crates/polygon-extract/src/lib.rs) lines 857-876; [`crates/polygon-extract/tests/erode_adversarial.rs`](../crates/polygon-extract/tests/erode_adversarial.rs) (test code) |
| inset breaks tile seams; global erosion does not | [`crates/navigation/tests/tiled_erosion_seams.rs`](../crates/navigation/tests/tiled_erosion_seams.rs) (test code) |
| the body-width portal rule | `distance_from_wall_blocks_portal_narrower_than_body`, [`crates/navigation/src/path.rs`](../crates/navigation/src/path.rs) (test code) |
| baked-erosion arithmetic | [`crates/navigation/src/wall_clearance.rs`](../crates/navigation/src/wall_clearance.rs) lines 18-37 |

---

**Next:** [08-moving-agents.md](08-moving-agents.md) — turning the polyline into a
character that moves, and where `WallClearance::clamp` sits in a frame.
Related: [04-units-and-conventions.md](04-units-and-conventions.md) (units),
[05-building-navmeshes.md](05-building-navmeshes.md) (`ExtractOptions`),
[12-large-worlds.md](12-large-worlds.md) (tiling),
[13-authored-geometry.md](13-authored-geometry.md) (the inset pipeline),
[16-troubleshooting.md](16-troubleshooting.md).
