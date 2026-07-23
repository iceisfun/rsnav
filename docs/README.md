# rsnav documentation

This page routes. It teaches nothing; every line below points at the page that owns the
material. The project [README](../README.md) owns the crate graph and the pitch.

## Start here

Pick the door that describes you. All three converge on
[04-units-and-conventions.md](04-units-and-conventions.md) and
[06-clearance.md](06-clearance.md) — the two places this library differs most from what you
will assume — so no door is a branch you cannot leave.

- **I have never written pathfinding.**
  [01](01-quickstart.md) → [02](02-concepts.md) → [04](04-units-and-conventions.md) →
  [06](06-clearance.md) → [08](08-moving-agents.md).
- **I know A\* on a grid (or Recast, or a waypoint graph).**
  [01](01-quickstart.md) → [03](03-from-grid-astar.md) → [04](04-units-and-conventions.md) →
  [06](06-clearance.md) → [08](08-moving-agents.md) → [05](05-building-navmeshes.md) →
  [07](07-paths-and-queries.md). Do not skip [08](08-moving-agents.md) on the grounds that
  you have written a path follower before: the
  [order rule](08-moving-agents.md#the-order-rule) (`Bsp::nearest`, *then*
  `WallClearance::clamp`) has no grid analogue and fails silently reversed.
  Skip [02](02-concepts.md); [03](03-from-grid-astar.md) stands alone and is the page that
  names which of your habits will actively mislead you.
- **I know navmeshes, show me the API.**
  [17](17-api-map.md) → the decision table at the top of [06](06-clearance.md) →
  [04](04-units-and-conventions.md) → whichever task page applies.

## Symptom index

Nearly every failure in this library is silent — wrong geometry, no error. Find what you
observed, not which crate you suspect.

| You see | Go to |
|---|---|
| `NoPerimeter` on a map that is obviously walkable | [16 §NoPerimeter](16-troubleshooting.md#builderrornoperimeter-on-a-map-that-is-obviously-walkable) |
| My whole world is upside down | [04 §The grid](04-units-and-conventions.md#the-grid), [16 §mirrored](16-troubleshooting.md#everything-is-mirrored-top-to-bottom) |
| `StartOutsideMesh` for a point I can see is inside | [16 §off-mesh start](16-troubleshooting.md#patherrorstartoutsidemesh-for-a-point-that-is-visibly-inside) |
| `Unreachable`, but I can see the route | [16 §Unreachable](16-troubleshooting.md#patherrorunreachable-when-the-route-is-plainly-there) |
| The agent shaves the corner / clips the wall | [06 §what it is not](06-clearance.md#pathoptionsdistance_from_wall-is-not-a-euclidean-guarantee) |
| I asked for 0.128 of erosion and lost a whole cell of floor | [06 §grid erosion](06-clearance.md#grid-erosion--bitfielderoded) |
| Walkable floor disappeared along the map edge | [16 §border](16-troubleshooting.md#walkable-floor-disappeared-along-the-map-border) |
| Clearance feels doubled — the agent walks down the middle | [06 §combining](06-clearance.md#combining-them) |
| Two areas that touch are not connected | [16 §no adjacency](16-troubleshooting.md#two-areas-that-touch-at-a-corner-are-not-connected) |
| No path across a tile seam | [16 §seam](16-troubleshooting.md#no-path-across-a-tile-seam), [12 §limits](12-large-worlds.md#the-honest-limits-of-tiledworld-v1) |
| Everything broke after a rebuild | [10 §what a swap invalidates](10-dynamic-rebuilds.md#what-a-swap-invalidates) |
| My door does nothing | [09 §inert doors](09-doors-and-navworld.md#the-inert-door-trap) |
| An agent with an unreachable goal is eating a core | [16 §pegs a core](16-troubleshooting.md#one-agent-pegs-a-core), [11 §surprises](11-crowds.md#behaviours-that-will-surprise-you) |
| The inset build is unbearably slow | [16 §slow inset](16-troubleshooting.md#inset-builds-are-unbearably-slow), [15 §protocol](15-performance-and-determinism.md#6-measurement-protocol) |
| My `NavListener` stopped firing | [16 §no listener output](16-troubleshooting.md#a-navlistener-produces-no-output-at-all) |
| `line_of_sight` says `Indeterminate` constantly | [07 §three-valued](07-paths-and-queries.md#the-result-is-three-valued-and-the-third-value-is-common) |
| Output differs between runs | [15 §what determinism does not promise](15-performance-and-determinism.md#what-determinism-does-not-promise) |
| An area that should be a hole is walkable | [16 §phantom region](16-troubleshooting.md#a-region-is-walkable-that-should-be-a-carved-out-hole) |
| Input geometry vanished with no diagnostic | [16 §vanished](16-troubleshooting.md#input-geometry-vanished-with-no-error-anywhere) |
| `SelfIntersection` from `form_skeleton` | [16 §self-intersection](16-troubleshooting.md#form_skeleton-returns-segmentinserterrorselfintersection) |

## The map

**The spine — 01 → 02 or 03 → 04 → 06 → 08.** Finish it and you can ship a character that
walks to a click and does not clip walls. Everything else is reached because you have that
specific problem.

| | Page | Read it because |
|---|---|---|
| 01 | [Your first path in forty lines](01-quickstart.md) | You want something on screen today. |
| 02 | [What a navmesh is, and why not a grid](02-concepts.md) | You have no pathfinding background. Skip if you do. |
| 03 | [Coming from grid A\*](03-from-grid-astar.md) | You do have one, and some of it will mislead you. Replaces 02. |
| 04 | [Units, coordinates, winding, markers](04-units-and-conventions.md) | Every convention in the library lives here and nowhere else. |
| 06 | [Keeping agents off walls: pick one of three](06-clearance.md) | Three mechanisms; choosing wrong fails silently. |
| 08 | [From polyline to movement](08-moving-agents.md) | A path is not a character that moves. |

| | Page | Trigger | Prereq |
|---|---|---|---|
| 05 | [Building navmeshes from a grid](05-building-navmeshes.md) | Your real map is a tilemap, image, or procedural output | 04 |
| 07 | [Paths and runtime queries](07-paths-and-queries.md) | You need AI perception, spawn points, or revalidation | 01 |
| 09 | [Doors, zones, NavWorld](09-doors-and-navworld.md) | Gates that toggle without a rebuild; per-triangle game data | 07 |
| 10 | [Dynamic rebuilds](10-dynamic-rebuilds.md) | Obstacles change at runtime | 05 |
| 11 | [Running many agents](11-crowds.md) | More than a handful of agents at once | 08 |
| 12 | [Big worlds: placement, tiles, seams](12-large-worlds.md) | The world does not fit in one mesh | 06 |
| 13 | [Building from authored polygons](13-authored-geometry.md) | Your source is vector data, not a grid | 04 |
| 14 | [Saving and loading](14-saving-and-loading.md) | You bake at build time and load at runtime | 05 |
| 15 | [Threads, determinism, and costs](15-performance-and-determinism.md) | Shipping at scale, or you need reproducible output | — |
| 16 | [It's not working](16-troubleshooting.md) | Entered cold, never read linearly | — |
| 17 | [Where everything lives](17-api-map.md) | You know what you want and need the path | — |
| 18 | [Sending a navmesh over the wire](18-interop.md) | A server ships a mesh to clients, or you move one between processes | 14 |

## Elsewhere

- `cargo doc --open -p rsnav-navigation` — every signature. Rustdoc owns those; these pages
  do not repeat them.
- [Project README](../README.md) — crate graph, architecture diagram, the runnable egui demo apps.
- [crates/navmesh/FORMAT.md](../crates/navmesh/FORMAT.md) — normative binary format spec.
- [SKILL.md](../SKILL.md) — AI-assistant recipe index.

Deliberately out of scope, so you stop looking: `CdtMesh`'s half-edge layer (`Otri`/`Osub`,
`bond`/`sym`/`tspivot`, `EncodedTri`) and the divide-and-conquer / segment-insertion
internals; `rsnav_triangle::io` (`.node`/`.poly`/`.ele` — public, unused by any build path);
a per-item API reference (rustdoc owns signatures; duplicating them guarantees drift); the
egui demo crates' internal architecture (things to run, never explained); algorithm
derivations for D&C Delaunay, snap rounding and the Felzenszwalb-Huttenlocher EDT (the
module headers own those, and `docs/plan-inset.md` is the inset design record, not part of
this set); API-level comparisons against Recast/Detour; and any benchmark figure not
re-measured on your own machine.

## Runnable examples

Every example in the workspace. This table supersedes the README's *Programmatic use* table
and is the one thing on this page that must be kept complete as examples are added. Run from
the workspace root — the corpus drivers read `testdata/` relative to the current directory.

To depend on rsnav from a project of your own rather than run these in place, see
[Using rsnav from your own project](01-quickstart.md#using-rsnav-from-your-own-project).
rsnav is not on crates.io; depend on it by git.
Use `--release` for anything touching the corpus or the inset path; debug builds of the inset
path run a per-triangle brute-force cross-check.

| Example | Command |
|---|---|
| [first_path](../crates/navigation/examples/first_path.rs) | `cargo run -p rsnav-navigation --example first_path` |
| [find_path](../crates/navigation/examples/find_path.rs) | `cargo run -p rsnav-navigation --example find_path` |
| [clearance_three_ways](../crates/navigation/examples/clearance_three_ways.rs) | `cargo run --release -p rsnav-navigation --example clearance_three_ways` |
| [walk_the_path](../crates/navigation/examples/walk_the_path.rs) | `cargo run --release -p rsnav-navigation --example walk_the_path` |
| [doors_and_zones](../crates/navigation/examples/doors_and_zones.rs) | `cargo run -p rsnav-navigation --example doors_and_zones` |
| [visibility_region](../crates/navigation/examples/visibility_region.rs) | `cargo run -p rsnav-navigation --example visibility_region` |
| [tiled_world](../crates/navigation/examples/tiled_world.rs) | `cargo run -p rsnav-navigation --example tiled_world` |
| [tiled_build](../crates/navigation/examples/tiled_build.rs) | `cargo run --release -p rsnav-navigation --example tiled_build` |
| [translate_vs_tiled](../crates/navigation/examples/translate_vs_tiled.rs) | `cargo run -p rsnav-navigation --example translate_vs_tiled` |
| [tcp_interop](../crates/navigation/examples/tcp_interop.rs) | `cargo run -p rsnav-navigation --example tcp_interop` |
| [grid_to_polygons](../crates/polygon-extract/examples/grid_to_polygons.rs) | `cargo run -p rsnav-polygon-extract --example grid_to_polygons` |
| [erode_and_clearance](../crates/polygon-extract/examples/erode_and_clearance.rs) | `cargo run --release -p rsnav-polygon-extract --example erode_and_clearance` |
| [triangulate_pslg](../crates/triangle/examples/triangulate_pslg.rs) | `cargo run -p rsnav-triangle --example triangulate_pslg` |
| [inset_rings](../crates/triangle/examples/inset_rings.rs) | `cargo run --release -p rsnav-triangle --example inset_rings` |
| [mesh_anatomy](../crates/dynamic/examples/mesh_anatomy.rs) | `cargo run --release -p rsnav-dynamic --example mesh_anatomy` |
| [live_worker](../crates/dynamic/examples/live_worker.rs) | `cargo run -p rsnav-dynamic --example live_worker` |
| [erode_vs_inset](../crates/dynamic/examples/erode_vs_inset.rs) | `cargo run --release -p rsnav-dynamic --example erode_vs_inset -- 1.0` |
| [pbm_bench](../crates/dynamic/examples/pbm_bench.rs) | `cargo run --release -p rsnav-dynamic --example pbm_bench -- testdata` |
| [stage_bench](../crates/dynamic/examples/stage_bench.rs) | `cargo run --release -p rsnav-dynamic --example stage_bench -- [--digest] [--erode 2.0] testdata` |
| [par_bench](../crates/dynamic/examples/par_bench.rs) | `cargo run --release -p rsnav-dynamic --example par_bench -- [--inset 0.5] testdata` |
| [save_and_load](../crates/navmesh/examples/save_and_load.rs) | `cargo run -p rsnav-navmesh --example save_and_load` |
| [locate_and_nearest](../crates/bsp/examples/locate_and_nearest.rs) | `cargo run -p rsnav-bsp --example locate_and_nearest` |
| [follow_path](../crates/pathing/examples/follow_path.rs) | `cargo run -p rsnav-pathing --example follow_path` |
| [two_agents_pass](../crates/crowd/examples/two_agents_pass.rs) | `cargo run -p rsnav-crowd --example two_agents_pass` |

Interactive egui demos, to run rather than read: `cargo run --release -p rsnav-demo`, and the
same for `rsnav-crowd-demo`, `rsnav-door-demo`, `rsnav-world-demo`, `rsnav-smoothing-demo`,
`rsnav-rtsim`. `rsnav-fixtures` is a headless PSLG corpus driver
(`cargo run --release -p rsnav-fixtures -- --testdata <dir>`); note `testdata/` holds the PBM
corpus and no `.json` fixtures, so point it at a fixture directory of your own.
