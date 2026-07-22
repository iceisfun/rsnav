# Building from authored polygons instead of a grid

Prerequisites: a working build from the grid path ([05-building-navmeshes.md](05-building-navmeshes.md)) is helpful but not required. You need [04-units-and-conventions.md](04-units-and-conventions.md) for winding and marker rules, which this page uses and does not restate.

Most rsnav users never open this page. The documented pipeline takes a `Bitfield` in and gives a `NavMesh` out. This page is for the other source: vector data. A level editor that stores collision as polylines, a CAD or SVG export, hand-placed contours dropped over a hand-drawn map. There is no `Bitfield` anywhere in that flow and no reason to rasterize one.

`rsnav-triangle` accepts that input directly. Two paths exist, they are not variations on each other, and picking wrong shows up as a build error or as silently wrong geometry.

---

## Which path

| | Legacy: `delaunay` → `form_skeleton` → `carve_holes` | Crossing-tolerant: [`build_cdt_with_inset`](../crates/triangle/src/inset.rs) |
|---|---|---|
| Input shape | `Pslg`: vertices, segments as index pairs, hole seed points | `&[InsetRing]`: point sequences tagged `Perimeter` or `Hole` |
| Input winding | Yours to get right | Any — normalization is internal |
| Rings that cross | **Cannot build.** `SegmentInsertError::SelfIntersection` | Builds |
| Hole seed points | Required, one strictly inside every hole | Not used at all |
| Erosion by agent radius | Not available | Built in, any `inset >= 0.0` |
| Failure mode on bad input | Typed `Err` from `form_skeleton` | `Err` for planarizer failures; **panics** on a bad `inset` |

The rule: **if your authored rings can ever cross each other, or if you want an agent radius baked in, use `build_cdt_with_inset`.** It is worth reaching for at `inset = 0.0` purely for the crossing tolerance — at zero the offset stage is skipped entirely and planarization is the whole value. The legacy path stays because it is the direct port of `triangle.c`, it is what the bitfield pipeline uses when `BuildOptions::inset` is `None`, and it is bit-identical to earlier releases.

---

## The legacy path

### What a PSLG is

A Planar Straight Line Graph is the classical input to a constrained Delaunay triangulation: a set of vertices, a set of straight-line *segments* that the triangulation is required to preserve as edges, and a set of *hole seed points*. See [`crates/triangle/src/pslg.rs`](../crates/triangle/src/pslg.rs):

```rust
pub struct Pslg {
    pub vertices: Vec<PslgVertex>,
    pub segments: Vec<PslgSegment>,   // { a: u32, b: u32, marker: i32 }
    pub holes: Vec<PslgHole>,         // { point: Vertex }
}
```

`PslgSegment::a` and `::b` are **indices, not positions**. A ring is expressed as the segments closing a cycle through its vertex indices; there is no ring type at this layer, and nothing tells the two rings of a room-with-a-pillar apart except which segments you emitted and where you put the hole seed.

`marker` is what becomes `NavTriangle::edge_markers[i]` downstream, and therefore what any wall classification you write later depends on. Marker meanings are yours to choose; the conventions the bitfield pipeline happens to use are in [04-units-and-conventions.md](04-units-and-conventions.md).

### The lockstep invariant

The CDT's vertex pool and the `Pslg`'s vertex list are **separate structures**, and `PslgSegment::a`/`::b` index the *mesh's* pool — `form_skeleton` resolves them through the mesh, not through the `Pslg`. The two lists must therefore be filled in the same order. Every caller in this workspace does it in one loop:

```rust
for (x, y) in pts {
    cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
    pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
}
```

quoted from [`crates/triangle/examples/triangulate_pslg.rs`](../crates/triangle/examples/triangulate_pslg.rs), which is the canonical minimal reference for this whole path — a 4×4 square with a 1×1 hole, printing live triangle counts after each stage. [`crates/navmesh/examples/save_and_load.rs`](../crates/navmesh/examples/save_and_load.rs) shows the equivalent with the `Pslg` built first and the mesh pool filled from it in a second loop, which is the same invariant expressed the other way around.

There is one wrinkle behind this. `delaunay` silently drops bit-exact duplicate vertex positions from its working set, so mesh index does not survive as input index in general. `form_skeleton` compensates by building its own position-to-first-occurrence remap and rewriting each segment's endpoints through it before insertion — but that repair exists for `form_skeleton`'s benefit. Any code of yours that assumes CDT vertex index equals input index after `delaunay` is wrong.

### Stages

```rust
delaunay(&mut cdt, DivConqOptions::default());   // options BY VALUE
form_skeleton(&mut cdt, &pslg, None)?;           // insert constraints
carve_holes(&mut cdt, &pslg, false);             // remove holes + exterior
clip_ears(&mut cdt, max_area);                   // optional cleanup
let nav = build_from_cdt(&cdt);                  // -> NavMesh
```

- `delaunay` takes `DivConqOptions` **by value**, unlike essentially every other options-taking function in the workspace, which take `&opts`. Its single field is `dwyer: bool`, default `true`, matching `triangle.c`. No caller in the workspace sets it to `false`.
- `form_skeleton`'s third argument is `mark_hull_with: Option<i32>`; `Some(1)` mimics `triangle.c`'s default markhull. Every caller in the workspace passes `None`, apart from one unit test in `segment.rs` that exists to cover the `Some` branch.
- `carve_holes`'s third argument is `convex: bool`, which *skips* the hull-concavity infection stage. Every in-workspace caller passes `false`.
- `clip_ears` is discussed under [scale](#the-scale-warning) below. `max_area <= 0.0` disables it and returns 0.
- `build_from_cdt` is infallible and returns a `NavMesh` with zero triangles for an empty or degenerate CDT. Detecting that is the caller's job.

Running the canonical example prints the stage counts, which is the cheapest way to see where geometry goes:

```
$ cargo run --release -p rsnav-triangle --example triangulate_pslg
after delaunay:       10 triangles
after form_skeleton:  10 triangles
after carve_holes:    8 triangles
```

`delaunay` triangulates the convex hull of every pushed point, so the initial count covers the hole and the exterior too. `form_skeleton` changes no counts here because inserting these particular constraints happens to require no splits. `carve_holes` is where the hole disappears. If your own run shows `carve_holes` removing nothing, your seed point is not inside the hole; if it removes everything, your seed is in the walkable region.

### Hole seeds are the whole correctness story

`carve_holes` is a flood fill. It seeds each point in `pslg.holes`, spreads across every edge that is not a constrained subseg, and deletes what it reaches. Its correctness depends entirely on there being a seed **strictly inside** every hole. A hole ring with no seed inside it is never carved and stays walkable — no error, no warning, just a room where a pillar should be.

Do not use the arithmetic centroid. For a concave hole — a C, an L, a U — the centroid frequently falls outside the ring, which makes the flood fill start in the walkable region and delete the map instead of the hole. [`Polygon::interior_point`](../crates/common/src/polygon.rs) exists for exactly this and says so in its own doc comment:

```rust
pub fn interior_point(&self) -> Option<Vertex>
```

It finds a convex vertex whose ear triangle contains no other vertex and returns that triangle's centroid, which is provably interior. `O(n²)` worst case. It returns `None` for fewer than three vertices, for a `Winding::Degenerate` ring, and pathologically when no ear is found. **`None` must be treated as a build failure.** The dynamic pipeline skips a hole whose seed is `None` (`crates/dynamic/src/lib.rs`), which is how a silently uncarved hole reaches a shipped navmesh.

### Choosing markers

Markers are the only channel by which authored intent survives into the `NavMesh`. `build_from_cdt` lifts each constrained subseg's marker onto `NavTriangle::edge_markers[i]`, where `0` means interior/unconstrained and any non-zero value means wall. Beyond that split, the numbers mean whatever you decide.

Two consequences worth planning for. First, `NavTriangle::region` — and therefore `NavMesh::reachable()` — is computed under the relation "two triangles connect iff their shared edge is not constrained", so **any** non-zero marker splits regions, whether you meant it as a wall or as an annotation. There is no such thing as a marker that labels an edge without also sealing it. Second, if you intend to classify walls downstream by marker value (glass versus stone, interior versus exterior), that classification depends entirely on values you assigned here, and nothing records the scheme for you. The demo uses `(i+1)*10` for perimeters, `1000 + i*10` for holes and `>= 2001` for rings authored in-session; `rsnav-dynamic` uses 1 and 2. Neither is a convention you must follow, and no numeric threshold separates the categories in general.

### The hard wall

`form_skeleton` returns

```rust
SegmentInsertError::SelfIntersection { endpoint1, endpoint2 }
```

on the **first** segment that would cross an already-inserted constrained subseg. Not a warning, not a repair — the insertion stops there. The mesh is left valid up to that point, so you can bail or strip the offending segment and retry, but there is no way to build a scene where a hole ring crosses a perimeter ring on this path at all.

This is not an exotic case. A level editor produces it the moment an author drags a hole half out of a room. It is the entire reason the second path exists.

---

## The crossing-tolerant path

[`build_cdt_with_inset`](../crates/triangle/src/inset.rs) **replaces** `delaunay` / `form_skeleton` / `carve_holes` — you do not call them yourself, and no hole seeds are involved:

```rust
pub fn build_cdt_with_inset(
    rings: &[InsetRing<'_>],
    inset: f64,
    opts: &InsetOptions,
) -> Result<InsetBuild, InsetError>

pub struct InsetRing<'a> {
    pub points: &'a [Vertex],   // implicitly closed, ANY winding
    pub kind: RingKind,         // Perimeter | Hole
    pub marker: i32,            // carried onto every derived constraint
}
```

`RingKind` is passed **structurally** and is never inferred from markers, because marker numbering is caller-specific. `InsetRing` borrows `points`, so the vertex buffers must outlive the ring slice. `InsetBuild` gives you `mesh` (ready for `clip_ears` then `build_from_cdt`), `soup` (the offset contours the cull classified against, kept for debug rendering), and `skipped_rings`.

### From polygons you already have

If your authored data is already `PolygonWithHoles` — or anything you can turn into one — the conversion is mechanical, and the borrow is the only thing to watch. This is how `rsnav-dynamic` does it per region (`crates/dynamic/src/lib.rs`):

```rust
let mut rings: Vec<InsetRing<'_>> = Vec::with_capacity(1 + region.holes.len());
rings.push(InsetRing {
    points: &region.outer.vertices,
    kind: RingKind::Perimeter,
    marker: opts.perimeter_marker,
});
for h in &region.holes {
    rings.push(InsetRing {
        points: &h.vertices,
        kind: RingKind::Hole,
        marker: opts.hole_marker,
    });
}
let built = build_cdt_with_inset(&rings, r, &InsetOptions::default())?;
```

`rings` borrows out of `region`, so `region` must outlive the call. If you are building rings from temporaries — points converted from an editor's coordinate type, say — materialize the `Vec<Vertex>` buffers into a named binding first and take slices from that; the borrow checker will tell you, but the fix is not always the obvious one.

Two things this snippet does not need and you do not either: no hole seed points, and no attention to which way round any ring runs. Scene 3 of `inset_rings.rs` feeds the same square perimeter clockwise and counter-clockwise and gets identical area out, because normalization to perimeter-CCW / hole-CW happens in stage 1 regardless of what you sent.

### What the inset actually does

Erosion by radius r shrinks perimeters by r and *grows* holes by r. Those are the same operation — a left-offset — once rings carry their natural orientation, which is the reason the pipeline normalizes before offsetting rather than after. Scene 1 of `inset_rings.rs` pins the arithmetic on a 40×40 perimeter with a 10×10 hole:

```
  inset   0:   8 live triangles, area 1500.000 (analytic 1500.000), skipped 0
  inset   2:   8 live triangles, area 1100.000 (analytic 1100.000), skipped 0
  inset   5:   8 live triangles, area  500.000 (analytic  500.000), skipped 0
```

At inset r the perimeter becomes `[r, 40-r]²` and the hole becomes `(10 + 2r)²`, so the walkable area is `(40 - 2r)² - (10 + 2r)²`. The units are whatever units your input is in — this path has no cell concept at all, which is a meaningful difference from `BuildOptions::inset`. See [04-units-and-conventions.md](04-units-and-conventions.md).

The runnable reference is [`crates/triangle/examples/inset_rings.rs`](../crates/triangle/examples/inset_rings.rs):

```
cargo run --release -p rsnav-triangle --example inset_rings
```

Scene 2 of that example is the important one. A 100×100 perimeter and a 20-tall hole ring straddling its right wall, fed to both paths:

```
  legacy: form_skeleton failed -> PSLG segment (4 → 5) crosses an existing
          constrained subsegment (self-intersecting input is not supported in v1)
  inset    0:   6 live triangles, area  9600.000
  inset    3:   6 live triangles, area  8316.000
```

9600 is `100*100 - 20*20`: the perimeter minus the part of the hole that lies inside it. The hole's outside lobe was exterior to begin with and needed no special handling.

### The four stages, for debugging

You do not reimplement these, but you will read them when output looks wrong. The module header at [`crates/triangle/src/inset.rs`](../crates/triangle/src/inset.rs) is the authority; the derivations belong to `offset.rs` and `planarize.rs`.

1. **Normalize.** Consecutive duplicates removed (closing wrap included), rings with fewer than 3 distinct points or zero area skipped, winding forced to perimeter-CCW and hole-CW. This runs identically at every inset, so the same ring set enters the soup at 0 as at r.
2. **Offset** (skipped entirely at `inset == 0.0`). Every edge is pushed **left** by the inset. Once rings carry their natural orientations, "left" is into the walkable region for both kinds, so one primitive shrinks perimeters and grows holes. The result is allowed to self-intersect and to contain orientation-flipped lobes; nothing detects or repairs them here.
3. **Planarize**, with the **robust** `orient2d` from `rsnav_triangle::predicates`. This is what makes the constraint set reaching `form_skeleton` provably non-self-intersecting, which is what makes crossing input buildable. Snap-rounded onto a power-of-two grid so the result is bit-deterministic.
4. **Cull by signed winding.** `carve_by_winding` kills every live triangle whose **centroid** has winding `< 1` against the original soup. Then `drop_interior_constraints` removes soup constraints left stranded between two kept triangles, which would otherwise split regions and fabricate phantom walls.

Classification in stage 4 is per-triangle and purely local — no flood fill, no seeds — which is why islands, merged holes and regions that split apart all come out right with no extra logic.

**The keep rule is `winding >= 1`, not `== 1`.** Two authored perimeters that overlap produce winding 2 in the overlap and must stay walkable. Anyone who reasons about the cull as even-odd, or as exactly-one, will delete the overlap of two rooms.

### Evidence

These are tests — two integration tests under `crates/navmesh/tests/` and one `#[cfg(test)]` block inside `inset.rs` — not runnable examples, but they are where the claims on this page come from:

- [`crates/navmesh/tests/inset_hole_cross_regression.rs`](../crates/navmesh/tests/inset_hole_cross_regression.rs) — a real captured scene, snapshotted from the demo, where a hole ring crosses the perimeter with two edge crossings. Asserts that the legacy path fails with `SelfIntersection` and that the inset path builds it. This is the load-bearing evidence for the whole crossing claim.
- [`crates/navmesh/tests/inset_pipeline.rs`](../crates/navmesh/tests/inset_pipeline.rs) — end-to-end scenarios asserted through `build_from_cdt` **region counts**: two holes whose dilations merge and then cross the shrunk perimeter, and a dumbbell whose corridor pinches into two regions while a dead-end dog leg disappears. The clearest demonstration of why winding-based culling is needed rather than seeded carving.
- `property_random_star_scenes`, `crates/triangle/src/inset.rs:393` — a seeded 25-scene sweep asserting, per case, that every radius builds, that the result is contained in the input rings, that area is monotone non-increasing in r, that `mesh(r2)` nests inside `mesh(r1)`, and that rebuilds are bit-identical. Every determinism and monotonicity statement about this path traces to here.

---

## Porting a legacy build to the crossing-tolerant path

If you have working legacy code and hit `SelfIntersection`, the migration is small but it is not a drop-in — the input model changes shape.

1. **Stop building a `Pslg`.** Rings become `InsetRing` values with a `RingKind`. You no longer emit segments, and index bookkeeping — the lockstep invariant, the `(i + 1) % n` cycle closing — disappears entirely.
2. **Delete every hole seed point.** `pslg.holes` has no analogue. Classification is per-triangle winding against the soup, not a flood fill, so `Polygon::interior_point` is no longer on the critical path for this build. It remains the right tool anywhere else you need a provably interior point.
3. **Drop `delaunay`, `form_skeleton` and `carve_holes`.** `build_cdt_with_inset` performs all three internally, plus planarization ahead of them and the winding cull after.
4. **Keep `clip_ears` and `build_from_cdt`.** They operate on the returned `InsetBuild::mesh` exactly as before. Re-read [the scale warning](#the-scale-warning) while you are there.
5. **Handle the new failure surface.** `InsetError` has two variants, `Planarize` and `Segment`; the second is documented as unreachable by contract, since the planarizer guarantees a non-crossing segment set, and if it ever fires it is a planarizer bug that must be surfaced rather than swallowed. Separately, a bad `inset` or `snap_cell` **panics** rather than returning `Err`, so validate before calling.
6. **Check `InsetBuild::mesh` for emptiness yourself,** and read [sharp edges](#sharp-edges) on what `skipped_rings` does and does not tell you.

Pass `inset = 0.0` for the first migration step. That changes only the crossing tolerance and leaves geometry otherwise comparable, which makes it possible to diff against your previous output before introducing erosion as a second, separate change.

---

## Sharp edges

Each of these is silent or fatal in a way that is hard to diagnose from the output.

**`build_cdt_with_inset` panics on a bad inset.** `assert!(inset.is_finite() && inset >= 0.0)` at `inset.rs:126`. There is no `InsetError` variant for it. `build_navmesh_from_bitfield` returns a typed `BuildError::InvalidInset` only because it validates *before* calling. Direct callers get an unwind — validate first.

**`InsetOptions::snap_cell = Some(v)` also panics on bad input.** It reaches `SnapGrid::from_target`, which asserts the value is finite, positive and not subnormal ([`crates/common/src/planarize.rs`](../crates/common/src/planarize.rs)). Not an `InsetError`. `None` is the normal choice and picks `SnapGrid::auto` from the soup bounding box and the inset; reach for `Some` only when the auto grid's bounds fight your coordinate scale.

**Full erosion is `Ok` and empty, not an error.** A build that erodes everything away returns a `CdtMesh` with zero live triangles. Scene 4 of `inset_rings.rs` shows a 20×20 perimeter at inset 15 doing exactly this. Check the mesh, not the `Result`.

**`skipped_rings` does not report what you think it reports.** It lists rings that were degenerate **at entry** — fewer than three distinct points after duplicate removal, or zero area. It does **not** list a perimeter dropped as provably fully eroded: `inset.rs:145-150` drops a perimeter whose bbox min-dimension is `<= 2*inset` before offsetting, with a bare `continue` that never touches `skipped`. Scene 4 prints both cases; the fully-eroded one comes back `skipped_rings = []` with zero triangles. Diffing input ring count against `skipped_rings.len()` will mislead you. And `rsnav-dynamic` discards `skipped_rings` entirely, so a degenerate perimeter fed through the dynamic path becomes a silently-empty region contribution with no diagnostic anywhere.

**`drop_interior_constraints` requires `soup_markers` sorted and deduplicated.** Membership is a binary search, and the precondition is guarded only by a `debug_assert!`. An unsorted slice in a release build silently fails to drop constraints, leaving phantom walls and split regions with no error. `build_cdt_with_inset` derives the set itself from the surviving contours and sorts it; if you call the function standalone, do the same. It is never a threshold or a numeric range — marker schemes differ per caller and no cutoff separates soup from non-soup.

**`carve_by_winding` needs the *original* soup, not the planarized segments,** and needs it already carrying natural orientation. It does no normalization of its own. A perimeter passed clockwise inverts the keep rule and kills the entire mesh.

**`delaunay` takes its options by value and silently drops duplicate positions.** Covered above; it bites first-time callers of this layer more than anything else here.

**Debug builds of this path are dramatically slower.** `carve_by_winding` cross-checks every triangle classification against the brute-force `winding_number` under `debug_assert` (`winding.rs:336-340`). That is a permanent differential oracle and it is worth having, but never benchmark or demo the inset path in a debug build.

---

## Debugging a build that came out wrong

Wrong output on this path is almost always wrong *input*, and the pipeline gives you four places to look. In rough order of how often each is the answer:

**Render `InsetBuild::soup`.** It is kept on the result for exactly this. The soup is the offset contour set the cull classified against — self-intersections, flipped lobes and all — so drawing it over your input rings shows immediately whether the offset stage produced what you expected. A hole that grew the wrong way, or a perimeter that turned inside out at a sharp reflex corner, is visible here and nowhere else.

**Check the winding of a point you can reason about.** `winding_number(p, contours) -> i32` is exported at the `rsnav_triangle` root. Pick a point you know should be walkable and one you know should not, and evaluate both against `built.soup`. Keep in mind the rule is `>= 1`, so a point that comes back 2 is correctly kept and a point that comes back 0 or -1 is correctly killed. A perimeter that reads -1 where you expected +1 means orientation went wrong upstream.

**Count live triangles versus ghosts.** The liveness test is two conditions, not one: a slot counts only when it is neither `is_dead()` nor carrying an invalid vertex id (hull-fan ghosts). Slot 0 is the dummy, so every scan starts at 1. Missing the ghost check yields garbage geometry that looks like a pipeline bug and is not; `live_area` and `live_tris` in `inset_rings.rs` are the correct form to copy.

**Run `verify_planar` on the constraint set** if you are calling `planarize` yourself. Covered under [going lower](#going-lower). If you are only calling `build_cdt_with_inset`, the planarizer's contract already covers this and an `InsetError::Segment` would be the signal.

One property that is useful when a build looks unstable: this path is deterministic and area-monotone. Repeated builds of the same input are bit-identical, and area is non-increasing in the inset, with `mesh(r2)` nested inside `mesh(r1)` for `r2 > r1`. Both are asserted by the seeded sweep cited above. If you observe either failing, the input differs between runs — check for iteration over a `HashMap` or `HashSet` upstream of your ring construction.

---

## The scale warning

`clip_ears(&mut mesh, max_area)` deletes triangles with exactly two constrained edges, one interior neighbour, and area below `max_area`. The threshold is a **raw area in whatever coordinate units your input uses**.

`BuildOptions::clip_ears_max_area` defaults to `0.6`, and that number is tuned for one specific thing: unit-cell bitfield input, where half-cell stair-step artifacts have area exactly 0.5, so 0.6 catches them and nothing else. It is not a general default. Copying it into an authored scene whose coordinates run to thousands of world units means it does nothing at all; copying it into a scene whose coordinates run 0..1 means it eats real geometry. Rescale it in proportion to your coordinate scale, or pass `0.0` to disable the pass — which is also what you want for exact area comparisons.

Note that `BuildOptions` is only reachable through `build_navmesh_from_bitfield`, which takes a `Bitfield`. There is no path from authored polygons through `BuildOptions`; on this page `clip_ears` is a function you call yourself, with a threshold you chose.

---

## Going lower

Four things under this layer are public and occasionally the right tool. Each has a module header that owns its explanation; none is a tutorial here.

**[`offset_ring_left`](../crates/common/src/offset.rs)** — `(ring: &Polygon, delta: f64, marker: i32, opts: &OffsetOptions) -> Option<SoupContour>`. The single offset primitive stage 2 uses. Pushes every edge left by `delta`, miters reflex corners with a bevel fallback past `OffsetOptions::miter_limit` (default 2.0, SVG/Clipper convention), generates no arcs, and deliberately emits self-intersecting output for downstream winding math to cancel. `None` for a degenerate ring. Reach for it when you want eroded *contours* rather than an eroded mesh.

**`planarize` versus `planarize_with`** — [`crates/common/src/planarize.rs`](../crates/common/src/planarize.rs). Both turn crossing, overlapping, T-junctioned soup into a segment set that meets only at shared endpoints, with all vertices on a power-of-two snap grid. `planarize` uses `rsnav_common::geom::orient2d`; `planarize_with` takes the predicate as an argument, and the inset pipeline passes `rsnav_triangle::predicates::orient2d`. **The two `orient2d`s share a name and do not share guarantees.** The `common` one is the fast non-adaptive form; the `triangle` one is Shewchuk's adaptive robust version. Snap rounding cannot repair a sign error in a near-degenerate orientation test, which is why the shipped pipeline injects the robust predicate and why you should too if your geometry is near-degenerate. Note also that `rsnav_common::geom` is **not** re-exported at its crate root — the path is `rsnav_common::geom::orient2d`.

**`carve_by_winding` and `drop_interior_constraints` standalone** — [`crates/triangle/src/winding.rs`](../crates/triangle/src/winding.rs), both re-exported at the `rsnav_triangle` root. Useful if you are assembling a constraint set some other way and want winding-based culling over it rather than seeded flood fill. Their preconditions are listed under [sharp edges](#sharp-edges) and are not optional. `winding_number(p, contours) -> i32` is exported too, if you only want to classify points.

**`verify_planar(ps, orient) -> Result<(), String>`** — a debug tool. Checks a `PlanarSegments` for proper crossings, endpoints interior to another segment, collinear overlap beyond shared endpoints, and duplicate or zero-length segments. `O(n²)`, for tests and debug assertions. If a build produces geometry you cannot explain, this is the assertion that tells you whether the constraint set was the problem.

Two surfaces this page deliberately does not cover. `CdtMesh`'s half-edge layer (`Otri`/`Osub`, `bond`/`sym`/`tspivot`, `EncodedTri`) is public and large, and no build path on this page requires touching it — the examples use only `CdtMesh::new`, `push_vertex`, `triangle`, `vertex_pos`, `live_triangle_count`, and iteration over `.triangles`. And `rsnav_triangle::io` is a reader/writer for Triangle's `.node`/`.poly`/`.ele` file formats; it exists, it is public, and no build path in the workspace uses it — the only callers are unit tests that read reference `.poly` fixtures.

---

## Where to go next

- Baked erosion is one of three ways to keep an agent off walls, and the choice between them is not obvious — [06-clearance.md](06-clearance.md) owns that decision. This page owns only the mechanism.
- Once you have a `NavMesh`, everything downstream is identical to the grid path: [07-paths-and-queries.md](07-paths-and-queries.md), [08-moving-agents.md](08-moving-agents.md).
- Placing an authored mesh at world coordinates: [12-large-worlds.md](12-large-worlds.md), and note that baked inset is incompatible with `TiledWorld`.
- Something built and looks wrong: [16-troubleshooting.md](16-troubleshooting.md).
