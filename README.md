# rsnav2

> A Rust constrained-Delaunay triangulator (port of Jonathan Shewchuk's *Triangle*) plus the runtime pieces you actually need to ship navigation: a navmesh binary format, A* + funnel path search with wall-clearance, a BVH for fast point queries, and an authoring/probing demo.

This is a **pure-Rust** reimplementation — no FFI, no C dependencies.
Built around a CDT port faithful enough that the included `A.poly` round-trips
byte-for-byte against the `triangle` binary's `.ele` output (29 triangles).

## At a glance

```
              ┌────────────────────────┐
   bitfield   │                        │   PSLG
   ────────▶  │   polygon-extract      │   ────────┐
              │ (true/false grid →     │           │
              │  polygons + holes)     │           │
              └────────────────────────┘           ▼
                                          ┌──────────────────────┐
                                          │      triangle        │
                                          │  (Shewchuk port,     │
                                          │   CDT_ONLY subset:   │
                                          │   D&C Delaunay,      │
                                          │   constrained edges, │
                                          │   hole carving)      │
                                          └──────────┬───────────┘
                                                     │ CdtMesh
                                                     ▼
                                          ┌──────────────────────┐
                                          │       navmesh        │   binary file
                                          │   (compact, indexed, │   (.navmesh,
                                          │   region-labelled)   │    see FORMAT.md)
                                          └────┬───────────┬─────┘
                                               │           │
                          query index ◀────────┘           └─────────▶ runtime queries
                          ┌──────────┐                       ┌──────────────────────┐
                          │   bsp    │                       │     navigation       │
                          │ (BVH for │ ─────  used by ────▶  │   A* + funnel +      │
                          │  locate, │                       │   LOS + nearest;     │
                          │ nearest) │                       │   distance_from_wall │
                          └──────────┘                       └──────────┬───────────┘
                                                                        │ path polyline
                                                                        ▼
                                                              ┌────────────────────┐
                                                              │      pathing       │
                                                              │  (PathFollower:    │
                                                              │   lookahead +      │
                                                              │   anti-shortcut)   │
                                                              └────────────────────┘
```

## Quick start

### Interactive demo

```
cargo run -p rsnav-demo --release
```

Author polygons with the mouse, press **Create navmesh**, then probe with right-click (set source) / left-click (set goal) for paths. The left panel has a *Fixtures* browser; the path field is pre-filled with `./testdata` and is editable — point it at any directory of `.json` maps.

### Batch-run fixtures from the CLI

```
cargo run -p rsnav-fixtures --release -- --testdata ./testdata
cargo run -p rsnav-fixtures --release -- --testdata ./broken.json -v
```

`--testdata <PATH>` is a file or a directory of `.json` fixtures; if omitted it defaults to `./testdata`. Prints a status table (triangle count, region count, build_ms per fixture) and exits non-zero on any failure — drop-in for CI.

### Programmatic use

Each crate ships a runnable example (`cargo run -p <crate> --example <name>`):

| crate | example | demonstrates |
| --- | --- | --- |
| `rsnav-triangle` | `triangulate_pslg` | Build a CDT from a hand-coded PSLG, carve holes. |
| `rsnav-polygon-extract` | `grid_to_polygons` | Turn a `true/false` bitfield into polygons + holes. |
| `rsnav-navmesh` | `save_and_load` | Build a navmesh, serialize to bytes, reload, compare. |
| `rsnav-bsp` | `locate_and_nearest` | BVH point-in-mesh + nearest-point queries. |
| `rsnav-navigation` | `find_path` | A* + funnel with `distance_from_wall`. |
| `rsnav-navigation` | `visibility_region` | Star-shaped visibility polygon from a point (sampled). |
| `rsnav-pathing` | `follow_path` | Simulated agent walking a polyline with lookahead + anti-shortcut. |

## Crates

| name | what it provides |
| --- | --- |
| `rsnav-common` | `Vertex`, `Polygon`, `Triangle`, `Aabb`, `Mesh2d`, IDs. Geometry helpers (`orient2d`, `incircle`, segment intersection, `Polygon::interior_point` — used for safe hole seeds on concave polygons). |
| `rsnav-triangle` | Constrained Delaunay triangulator. Faithful Rust port of Shewchuk's `triangle.c` restricted to the `-DCDT_ONLY` subset (no Steiner-point quality refinement, no Voronoi). D&C Delaunay, segment insertion, hole carving, `.poly`/`.node`/`.ele` I/O. |
| `rsnav-polygon-extract` | Bitfield → `PolygonWithHoles`. 4-connectivity region detection, optional collinear-vertex removal, optional zigzag → diagonal smoothing, min-area culling. |
| `rsnav-navmesh` | Runtime mesh: flat vertices + triangles, per-triangle adjacency, edge constraint markers, area, centroid, connected-component region IDs. Versioned little-endian binary format ([FORMAT.md](crates/navmesh/FORMAT.md)). |
| `rsnav-bsp` | BVH (AABB-tree) over a `NavMesh`. `locate(point)` and `nearest(point)`, both `O(log n)` average. |
| `rsnav-navigation` | A* across triangle adjacency, Simple Stupid Funnel string-pull, triangle-walk line-of-sight, nearest-point. `distance_from_wall` rejects narrow portals and pulls portal endpoints inward at wall vertices. |
| `rsnav-pathing` | `PathFollower`: lookahead + monotone arc-progress projection + anti-shortcut bias at corners. No navmesh dependency — operates on any polyline. |
| `rsnav-demo` | egui authoring + probing app (the *Quick start* demo above). |
| `rsnav-fixtures` | CLI runner for `.json` PSLG fixtures (the *Batch-run* tool above). |

## File format

`navmesh` v1 is a section-based little-endian binary format. The full normative spec is in [`crates/navmesh/FORMAT.md`](crates/navmesh/FORMAT.md). It's designed to be implementable in any language — fixed-width records, no varints/compression/alignment-tricks, unknown section types silently skipped for forward compatibility.

Required sections: `META`, `VERTICES`, `TRIANGLES`. Optional (recomputed if absent): `ADJACENCY`, `EDGE_MARKERS`, `TRI_INFO`. The minimum portable file is `META + VERTICES + TRIANGLES + EDGE_MARKERS` (the markers can't be re-derived without losing the wall information).

## Status

Working and tested (124 tests pass workspace-wide):

- ✅ CDT round-trip against `triangle.c` reference (`A.poly` → 29 triangles, byte-exact)
- ✅ 200-point random Delaunay stress passes the empty-circumcircle test
- ✅ Wall-distance clearance pathing (rejects narrow portals, pulls funnel endpoints off walls)
- ✅ Real gonav-format fixtures (representative loads: ~1900 tris / 6 regions and ~1250 tris / 23 regions) build clean when pointed at via `--testdata`

Deliberate v1 omissions:

- Steiner point quality refinement (`-q` switch in `triangle`) — no facility for it.
- Self-intersecting PSLG segments (`segmentintersection` in `triangle`) — `form_skeleton` returns `Err(SelfIntersection)` rather than splitting the crossing.
- Conforming-Delaunay midpoint splitting (`conformingedge` in `triangle`).
- Visibility-region exact (Asano/Suri sweep). The shipped `visibility_region` uses uniform angular sampling — exact enough for visualization at typical resolutions.
- Interactive pan/zoom in the demo (only auto-fit + Fit-view button).

## References

- **Triangle**: Jonathan Richard Shewchuk, *"Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator,"* in *Applied Computational Geometry: Towards Geometric Engineering*, vol. 1148 of LNCS, pp. 203–222, Springer, 1996. [`https://www.cs.cmu.edu/~quake/triangle.html`](https://www.cs.cmu.edu/~quake/triangle.html)
- **Robust predicates**: Shewchuk, *"Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates,"* Discrete & Computational Geometry 18:305–363, 1997.
- **Simple Stupid Funnel**: Mikko Mononen, *"Simple Stupid Funnel Algorithm,"* 2010. [`https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html`](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html)

## License

Dual-licensed under either of

- MIT license
- Apache License, Version 2.0

at your option.
