# Where everything lives

A router, not a reference. rustdoc owns every signature; this page owns the question
"which crate, which module, is it re-exported, and which page teaches it". Nothing here
is duplicated from rustdoc, because a duplicated signature is a signature that drifts.

Build the API docs for the whole workspace and read them alongside this page:

```
cargo doc --workspace --no-deps --open
```

## Crates

| Crate | Owns | Entry points (by name) | Taught in |
|---|---|---|---|
| `rsnav-common` | Core geometry types and the shared building blocks every other crate imports. | `Vertex`, `Aabb`, `Polygon`, `PolygonWithHoles`, `Winding`, `Triangle`, `Mesh2d`, `VertexId`/`TriangleId`/`PolygonId`; modules `geom`, `offset`, `planarize`, `par`, `rng` | [04](04-units-and-conventions.md), [13](13-authored-geometry.md), [15](15-performance-and-determinism.md) |
| `rsnav-polygon-extract` | The grid front end: occupancy bitfield in, rings out; plus grid-space erosion. | `Bitfield`, `BitfieldError`, `ExtractOptions`, `extract`, `ErodeOptions`, `ErodeError`, `ClearanceField` | [05](05-building-navmeshes.md), [06](06-clearance.md) |
| `rsnav-triangle` | The CDT itself — a port of Shewchuk's Triangle, plus the crossing-tolerant inset front end. | `delaunay`, `DivConqOptions`, `form_skeleton`, `carve_holes`, `clip_ears`, `Pslg` and friends, `CdtMesh`, `build_cdt_with_inset`, `InsetRing`, `RingKind`, `InsetOptions`, `InsetBuild`, `InsetError`, `carve_by_winding`, `drop_interior_constraints`, `winding_number`, `orient2d`, `incircle` | [13](13-authored-geometry.md) |
| `rsnav-navmesh` | The runtime mesh type and its on-disk format. | `NavMesh`, `NavTriangle`, `BoundaryEdge`, `build_from_cdt`, `MAGIC`, `FORMAT_VERSION`, `SaveError`, `LoadError` | [05](05-building-navmeshes.md), [14](14-saving-and-loading.md) |
| `rsnav-bsp` | The spatial index over a `NavMesh`. Despite the name it is a BVH, not a BSP. | `Bsp`, `Nearest` | [07](07-paths-and-queries.md) |
| `rsnav-navigation` | Everything you ask a built navmesh at runtime. | `find_path`, `find_path_with_walls`, `PathOptions`, `PathResult`, `PathError`, `path_clear`, `nearest_point`, `NearestPoint`, `line_of_sight`, `LineOfSightResult`, `WallInfo`, `WallClearance`, `DoorSet`/`Door`/`DoorId`/`DoorState`, `resolve_door_edges`, `nearest_portal_edge`, `visibility_region`, `VisibilityRegion`, `NavWorld`, `NavMetadata`, `NoMetadata`, `zone_crossings`, `ZoneCrossing`, `TiledWorld`, `TileId`, `GlobalTri`, `Link` | [07](07-paths-and-queries.md), [09](09-doors-and-navworld.md), [12](12-large-worlds.md) |
| `rsnav-pathing` | Turning a returned polyline into agent motion. Depends only on `rsnav-common` — it has no navmesh access at all. | `PathFollower`, `FollowerOptions`, `PathFollowerError` | [08](08-moving-agents.md) |
| `rsnav-dynamic` | The one-shot bitfield-to-navmesh pipeline and the background rebuild worker. | `build_navmesh_from_bitfield`, `BuildOptions`, `NavBuild`, `BuildError`, `NavWorker`, `NavListener`, `NavEvent`, `NavStats` | [05](05-building-navmeshes.md), [10](10-dynamic-rebuilds.md) |
| `rsnav-crowd` | Many agents sharing one navmesh, with local avoidance. | `Crowd`, `CrowdConfig`, `Agent`, `AgentId`, `Goal` | [11](11-crowds.md) |

Demo binaries — `rsnav-demo`, `rsnav-crowd-demo`, `rsnav-door-demo`, `rsnav-world-demo`,
`rsnav-smoothing-demo`, `rsnav-rtsim`, `rsnav-fixtures` — are applications, not API. Run
them; do not depend on them.

## Modules at a glance

Only two crates have a module structure worth navigating; the rest declare everything at
their crate root (`rsnav-bsp`, `rsnav-pathing`, `rsnav-dynamic`, `rsnav-crowd`,
`rsnav-polygon-extract`) and are fully covered by the crate table above. `rsnav-navmesh`
has three modules — `binary`, `build`, `navmesh` — but re-exports all of their public items,
so the crate root is the only path you need.

| `rsnav_common::` | Contains |
|---|---|
| `vertex`, `aabb`, `ids`, `triangle`, `mesh2d`, `polygon` | The core types, all re-exported. One exception: `MeshIndexError` stays at `mesh2d::MeshIndexError`. |
| `geom` | Non-robust predicates and the pure-function geometry toolbox. Module-only. |
| `offset` | Contour offsetting by a signed distance. Re-exported. |
| `planarize` | Snap-rounded planarization and `SnapGrid`. Re-exported. |
| `par` | Hand-rolled scoped-thread primitives. Module-only. See [15](15-performance-and-determinism.md). |
| `rng` | A seeded LCG for tests and property sweeps. Module-only. |

| `rsnav_navigation::` | Contains |
|---|---|
| `path` | `find_path`, `find_path_with_walls`, `path_clear`, `nearest_point`, `PathOptions`, `PathResult`, `PathError`. All re-exported. |
| `astar`, `funnel` | The two search stages `find_path` composes. Public, **not** re-exported. |
| `los`, `visibility` | `line_of_sight`, `visibility_region`. Re-exported. |
| `wall`, `wall_clearance` | `WallInfo` and `WallClearance` are re-exported; `is_wall_edge_local` is not. |
| `doors` | `DoorSet` and its authoring helpers. Re-exported. |
| `world` | `NavWorld`, `NavMetadata`, `zone_crossings`. Re-exported. |
| `tiled` | `TiledWorld` and seam stitching. Re-exported. |

`rsnav_triangle` exposes one module per port stage (`divconq`, `segment`, `holes`, `clip`,
`flip`, `sort`, `mesh`, `predicates`, `pslg`, `inset`, `winding`, `io`) and re-exports the
build-path entry points from each; `io` is the exception and stays module-only.

## Table 1: re-exports, and what is not reachable from a crate root

Several useful items live only in their module. Guessing the short path produces a compile
error, so the exact import path is worth stating.

| Item | Real path | At the crate root? |
|---|---|---|
| `geom::orient2d`, `incircle`, `signed_area2`, `segment_intersection`, `segments_intersect`, `point_in_triangle`, `nearest_point_on_segment`, `nearest_point_on_triangle`, `on_segment_collinear`, `SegmentHit`, `SegmentIntersection` | `rsnav_common::geom::*` | **No.** `rsnav_common` re-exports `aabb`, `ids`, `mesh2d`, `offset`, `planarize`, `polygon`, `triangle`, `vertex` items only ([`lib.rs:21`](../crates/common/src/lib.rs)). |
| `Lcg` | `rsnav_common::rng::Lcg` | **No.** Module-only. |
| `resolve_threads`, `par_map_indexed`, `par_bands_mut` | `rsnav_common::par::*` | **No.** Module-only. |
| `Aabb`, `Mesh2d`, `Polygon`, `PolygonWithHoles`, `Winding`, `Triangle`, `Vertex`, `VertexId`, `TriangleId`, `PolygonId` | `rsnav_common::*` | Yes. |
| `offset_ring_left`, `OffsetOptions`, `SoupContour` | `rsnav_common::*` | Yes (from `offset`). |
| `planarize`, `planarize_with`, `verify_planar`, `PlanarSegments`, `PlanarizeError`, `SnapGrid` | `rsnav_common::*` | Yes (from `planarize`). |
| `astar`, `AstarError` | `rsnav_navigation::astar::*` | **No.** `pub mod astar`, not re-exported. |
| `funnel` | `rsnav_navigation::funnel::funnel` | **No.** `pub mod funnel`, not re-exported. |
| `is_wall_edge_local` | `rsnav_navigation::wall::is_wall_edge_local` | **No.** Only `WallInfo` is re-exported from `wall`. |
| `WindingIndex` | — | **Does not exist as API.** It is a private struct ([`winding.rs:119`](../crates/triangle/src/winding.rs)) named in the crate's own module header. It is a per-call acceleration structure and is not callable from outside. |
| `EdgeKey` | — | **Not nameable downstream.** `pub(crate) type EdgeKey = (u32, u32)` ([`wall.rs:25`](../crates/navigation/src/wall.rs)). `resolve_door_edges`' real public return type is `Vec<(u32, u32)>`. |
| `io::read_poly`, `read_node`, `format_node`, `format_ele`, `format_poly`, `WriteOptions`, `IoError` | `rsnav_triangle::io::*` | **No.** `pub mod io`, not re-exported; see "Undocumented surface" below. |
| `CdtMesh`, `Otri`, `Osub`, `VertexSlot`, `VertexType` | `rsnav_triangle::*` | Yes, but see "Undocumented surface". |

### The two `orient2d`s

`rsnav_common::geom::orient2d` and `rsnav_triangle::orient2d` share a name and do not share
a guarantee.

| | `rsnav_common::geom::orient2d` | `rsnav_triangle::orient2d` |
|---|---|---|
| Path | module-only, `geom::` | re-exported from `predicates` at the crate root |
| Form | fast, non-adaptive, non-robust | Shewchuk's adaptive exact predicate |
| Used by | `planarize()` by default | `planarize_with()` as injected by the inset pipeline ([`inset.rs:190`](../crates/triangle/src/inset.rs)) |

`planarize_with` exists precisely so a caller that owns a robust predicate can supply it.
The shipped inset pipeline does. A caller reaching for the free `planarize()` on
near-degenerate geometry gets the weaker of the two. Same for `incircle`.

The project [README](../README.md) names `orient2d` and `incircle` in the
`rsnav-common` row with no path, which invites `use rsnav_common::orient2d;` — that does
not compile.

## Table 2: options structs and where their defaults live

Read the `Default` impl, not this page, for the values. Each row is a jump target.

| Struct | Crate | Definition | `Default` impl | Taught in |
|---|---|---|---|---|
| `ExtractOptions` | `rsnav-polygon-extract` | [`lib.rs:112`](../crates/polygon-extract/src/lib.rs) | [`lib.rs:131`](../crates/polygon-extract/src/lib.rs) | [05](05-building-navmeshes.md) |
| `ErodeOptions` | `rsnav-polygon-extract` | [`lib.rs:662`](../crates/polygon-extract/src/lib.rs) | [`lib.rs:689`](../crates/polygon-extract/src/lib.rs) | [06](06-clearance.md) |
| `BuildOptions` | `rsnav-dynamic` | [`lib.rs:68`](../crates/dynamic/src/lib.rs) | [`lib.rs:129`](../crates/dynamic/src/lib.rs) | [05](05-building-navmeshes.md) |
| `InsetOptions` | `rsnav-triangle` | [`inset.rs:59`](../crates/triangle/src/inset.rs) | [`inset.rs:68`](../crates/triangle/src/inset.rs) | [13](13-authored-geometry.md) |
| `OffsetOptions` | `rsnav-common` | [`offset.rs:28`](../crates/common/src/offset.rs) | [`offset.rs:36`](../crates/common/src/offset.rs) | [13](13-authored-geometry.md) |
| `DivConqOptions` | `rsnav-triangle` | [`divconq.rs:24`](../crates/triangle/src/divconq.rs) | [`divconq.rs:30`](../crates/triangle/src/divconq.rs) | [13](13-authored-geometry.md) |
| `PathOptions` | `rsnav-navigation` | [`path.rs:14`](../crates/navigation/src/path.rs) | [`path.rs:35`](../crates/navigation/src/path.rs) | [06](06-clearance.md), [07](07-paths-and-queries.md) |
| `FollowerOptions` | `rsnav-pathing` | [`lib.rs:22`](../crates/pathing/src/lib.rs) | [`lib.rs:36`](../crates/pathing/src/lib.rs) | [08](08-moving-agents.md) |
| `CrowdConfig` | `rsnav-crowd` | [`lib.rs:142`](../crates/crowd/src/lib.rs) | [`lib.rs:177`](../crates/crowd/src/lib.rs) | [11](11-crowds.md) |

One signature irregularity worth knowing before you write the call: `delaunay` takes
`DivConqOptions` **by value**. Every other options-taking *function* in the workspace —
`extract`, `eroded`, `build_navmesh_from_bitfield`, `build_cdt_with_inset`,
`offset_ring_left`, `find_path`, `PathFollower::target` — takes `&opts`. The constructors
that store their options are the separate case and take them by value: `NavWorker::spawn`
and `NavWorker::spawn_with_listener` take `BuildOptions`, `Crowd::new` takes `CrowdConfig`.

Things routinely mistaken for options structs but which have no configuration at all:
`Bsp`, `WallInfo`, `WallClearance`, `NavWorld`, `TiledWorld`. `TiledWorld`'s single knob is
the `tol` argument to `stitch_all`; `Bsp`'s `LEAF_THRESHOLD` and `WallClearance`'s
`RELAX_ITERS` are private constants.

## Table 3: error types, and which ones compose with `?`

| Error | Crate / module | `Display` | `std::error::Error` |
|---|---|---|---|
| `PathError` | `rsnav_navigation::path` | **No** | **No** |
| `AstarError` | `rsnav_navigation::astar` | **No** | **No** |
| `BuildError` | `rsnav_dynamic` | Yes ([`lib.rs:179`](../crates/dynamic/src/lib.rs)) | Yes ([`lib.rs:192`](../crates/dynamic/src/lib.rs)) |
| `InsetError` | `rsnav_triangle::inset` | Yes | Yes |
| `SegmentInsertError` | `rsnav_triangle::segment` | Yes | Yes |
| `PlanarizeError` | `rsnav_common::planarize` | Yes | Yes |
| `ErodeError` | `rsnav_polygon_extract` | Yes | Yes |
| `BitfieldError` | `rsnav_polygon_extract` | Yes | Yes |
| `MeshIndexError` | `rsnav_common::mesh2d` | Yes | Yes |
| `PathFollowerError` | `rsnav_pathing` | Yes | Yes |
| `SaveError`, `LoadError` | `rsnav_navmesh::binary` | Yes | Yes |
| `IoError` | `rsnav_triangle::io` | Yes | Yes |

`PathError` and `AstarError` are the two exceptions, and the consequence is concrete: a
function returning `Result<_, Box<dyn std::error::Error>>` cannot `?` on `find_path`. Match
the variants, or map them into your own error type. `PathError` is `Copy + Eq`, so matching
is cheap.

Two failure modes are not errors at all and will not appear in this table:

- `build_cdt_with_inset` **panics** on a non-finite or negative inset
  ([`inset.rs:126`](../crates/triangle/src/inset.rs)); only
  `build_navmesh_from_bitfield` validates first and converts it to
  `BuildError::InvalidInset`.
- `SnapGrid::from_target` and `SnapGrid::auto` **panic** on bad input
  ([`planarize.rs:46`](../crates/common/src/planarize.rs)), so
  `InsetOptions::snap_cell = Some(bad)` is an unwind, not an `InsetError`.

See [13](13-authored-geometry.md).

## Dependencies

`rsnav-dynamic` is the one library crate with an external dependency: `arc-swap`
([`crates/dynamic/Cargo.toml:16`](../crates/dynamic/Cargo.toml) inherits it from the
workspace, where it is pinned as `arc-swap = "1"`). Every other library crate —
`rsnav-common`, `rsnav-polygon-extract`, `rsnav-triangle`, `rsnav-navmesh`, `rsnav-bsp`,
`rsnav-navigation`, `rsnav-pathing`, `rsnav-crowd` — declares only rsnav crates and `std`.

Any blanket "zero external dependencies" phrasing is therefore false for the workspace as a
whole. Note that the transitive picture is smaller than the direct one suggests:
`rsnav-crowd` depends on `rsnav-dynamic`, so it pulls `arc-swap` too and cannot be routed
around. A build is external-dependency-free only if it avoids both `rsnav-dynamic` and
`rsnav-crowd`. The demo binaries additionally pull `eframe`/`egui`, and `rsnav-fixtures`
pulls `serde`/`serde_json`; neither is on any library path.

There is no FFI and no C. There is no `unsafe` block anywhere in the library sources.
Seven of the nine library crates enforce that with `#![forbid(unsafe_code)]`;
`rsnav-triangle` and `rsnav-dynamic` do not carry the attribute, though neither contains
any `unsafe`.

## Deliberately undocumented surface

Three public areas have no page in this set. Each omission is a choice.

- **`CdtMesh`'s half-edge layer** — `Otri`, `Osub`, `bond`/`sym`/`tspivot`, `EncodedTri`,
  `VertexType`, and the `divconq`/`segment` insertion internals. Public because the port
  needed them public, and large. No build path requires touching them: every example in
  the tree uses only `CdtMesh::new`, `push_vertex` (with `VertexSlot::new`),
  `live_triangle_count`, and iteration over `triangles` / `triangle` / `vertex_pos`.
  The module headers in [`crates/triangle/src/mesh.rs`](../crates/triangle/src/mesh.rs)
  are the reference.
- **`rsnav_triangle::io`** — a reader/writer for Triangle's `.node`/`.poly`/`.ele` file
  formats. Public, but not re-exported and used by no build path in the workspace. It
  exists for interoperability with `triangle.c` tooling, which is not a documented
  workflow here.
- **The demo crates' internals** — `rsnav-demo`, `rsnav-crowd-demo`, `rsnav-door-demo`,
  `rsnav-world-demo`, `rsnav-smoothing-demo`, `rsnav-rtsim`. They are things to run and
  read, not architecture to imitate; their egui plumbing has no bearing on library use.

If you know the name but the behaviour is not what you expected, the symptom index in
[README.md](README.md) and [16-troubleshooting.md](16-troubleshooting.md) route by what you
observed rather than by crate; 16 also carries the list of known doc/code contradictions.
Coming from grid A\* or Recast, [03-from-grid-astar.md](03-from-grid-astar.md) names the
assumptions this API will quietly violate.

Also out of scope for the whole doc set, stated once in [docs/README.md](README.md):
algorithm derivations (the module headers own those), a per-item API reference (rustdoc
owns that), and `docs/plan-inset.md`, which is the inset design record and not part of this
set.
