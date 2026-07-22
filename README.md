# rsnav

> A Rust constrained-Delaunay triangulator (port of Jonathan Shewchuk's *Triangle*) plus the runtime pieces you actually need to ship navigation: a navmesh binary format, A* + funnel path search with wall-clearance, a BVH for fast point queries, a background worker for dynamic obstacles, a multi-agent crowd with local avoidance, and authoring/probing demos.

This is a **pure-Rust** reimplementation — no FFI, no C dependencies, no `unsafe`.
Every crate but one is also free of external Rust dependencies; `rsnav-dynamic` takes
`arc-swap` for the lock-free `poll_swap` handoff.
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

That diagram is the **legacy path**: polygons go to the CDT as-is. There is a second,
crossing-tolerant path through `triangle`, taken when `BuildOptions::inset` is set — the
contours are first offset inward (`common::offset`), the resulting self- and
mutually-crossing rings are planarized into a non-crossing arrangement
(`common::planarize`), and the interior is selected by winding number rather than by
hole-seed flood fill (`triangle::winding`). It is the only path that accepts input rings
that cross. See [docs/13-authored-geometry.md](docs/13-authored-geometry.md) for when to
use it and [docs/06-clearance.md](docs/06-clearance.md) for what it costs.

For game-loop integration with dynamic obstacles, `rsnav-dynamic` wraps the
pipeline in a background-thread worker that consumes `Bitfield` snapshots
and publishes results lock-free:

```
  game thread                                  worker thread (NavWorker)
  ┌──────────────────────┐                     ┌──────────────────────────────────┐
  │ edit Bitfield        │                     │  polygon-extract → CDT → navmesh │
  │ submit_snapshot ─────┼──── coalesce ─────▶ │  → BSP                           │
  │                      │                     │                                  │
  │ poll_swap()          │ ◀── ArcSwap ─────── │  publish Arc<NavBuild>           │
  │   ↳ Arc<NavBuild>    │                     │  dispatch NavEvent → listener    │
  │ stats() → NavStats   │ ◀── atomics ─────── │  bump counters                   │
  └──────────────────────┘                     └──────────────────────────────────┘
```

For multi-agent simulation, `rsnav-crowd` sits on top of `Arc<NavBuild>`:
each agent owns a funnel-pulled corridor through the shared navmesh and a
sampled velocity-obstacle solver picks per-tick velocities that follow
the corridor while side-stepping other agents.

```
   Arc<NavBuild>  ──▶  Crowd ──▶ tick(dt)
                       │ ├── replan / arrive   (find_path per agent, radius-aware)
                       │ ├── rebuild hash      (spatial grid of agent positions)
                       │ ├── choose velocities (sampled-VO: align minus TTC penalty)
                       │ └── integrate         (advance + snap to mesh)
                       │
                       └── set_nav(Arc<NavBuild>)   ◀── only invalidates corridors
                                                       that the new mesh broke
```

## Documentation

Full prose documentation lives in **[docs/README.md](docs/README.md)**, which routes by who
you are and by what went wrong. Three entry points:

- **Never written pathfinding** — [01-quickstart](docs/01-quickstart.md), then
  [02-concepts](docs/02-concepts.md).
- **You have grid A\*, Recast, or a waypoint graph** — [01-quickstart](docs/01-quickstart.md),
  then [03-from-grid-astar](docs/03-from-grid-astar.md), which names the habits that will
  mislead you here.
- **You know navmeshes and want the API** — [17-api-map](docs/17-api-map.md), then the
  decision table in [06-clearance](docs/06-clearance.md).

All three converge on [04-units-and-conventions](docs/04-units-and-conventions.md) and
[06-clearance](docs/06-clearance.md). Those two pages carry the material that costs people
days: `false` means wall, `BuildOptions::inset` is in **cells** while
`PathOptions::distance_from_wall` is in **world units**, grid erosion quantizes a 0.128
request to a full 1.0 peel, and `distance_from_wall` is a portal shift rather than a
Euclidean clearance guarantee.

One choice is worth making before you write any code, because reversing it later means
rebuilding your whole content pipeline: **`BuildOptions::inset` cannot be used with
`TiledWorld`.** Per-tile contour erosion pulls each tile's boundary off the seam line, so
`stitch_all` matches nothing and cross-seam paths silently fail. If your world will ever be
tiled, clearance has to come from global grid erosion applied *before* slicing, which is
cell-quantized — so sub-cell agent radii and tiling are mutually exclusive in v1. See
[06-clearance](docs/06-clearance.md) and [12-large-worlds](docs/12-large-worlds.md).

Something already broken? [docs/README.md](docs/README.md) opens with a symptom index —
nearly every failure mode here is silent, producing wrong geometry rather than an error —
and [16-troubleshooting](docs/16-troubleshooting.md) is written to be entered cold.

## Quick start

### Interactive demo

```
cargo run -p rsnav-demo --release
```

Author polygons with the mouse, press **Create navmesh**, then probe with right-click (set source) / left-click (set goal) for paths. The left panel has a *Fixtures* browser; the path field is pre-filled with `./testdata` and is editable — point it at any directory of `.json` maps. Also in exploration mode: **Doors** (place a runtime door by clicking the highlighted edge, or by drawing a segment across a passage; toggle open/closed and watch paths/LOS/visibility react with no rebuild) and **Zones** (a `NavMetadata` demo that tints triangles and annotates where a path crosses a zone boundary).

### Batch-run fixtures from the CLI

```
cargo run -p rsnav-fixtures --release -- --testdata ./testdata
cargo run -p rsnav-fixtures --release -- --testdata ./broken.json -v
```

`--testdata <PATH>` is a file or a directory of `.json` fixtures; if omitted it defaults to `./testdata`. Prints a status table (triangle count, region count, build_ms per fixture) and exits non-zero on any failure — drop-in for CI.

### Dynamic-obstacles testbed

```
cargo run -p rsnav-rtsim --release
```

RTS-style harness: a 128×128 cell bitfield is the ground truth; mouse tools paint walls, clear them, or harvest forest cells one at a time. A background `NavWorker` (from `rsnav-dynamic`) keeps the navmesh in sync by re-running the full pipeline on each bitfield snapshot, coalescing rapid changes so it never falls behind. ~10 agents path between random walkable points and re-plan after every navmesh swap, demonstrating that game systems can keep operating while the mesh churns. The side panel surfaces live worker stats (submitted / coalesced / in-flight / completed / build_ms) and a scrolling event log via the typed `NavListener` API.

### Multi-agent crowd testbed

```
cargo run -p rsnav-crowd-demo --release
```

Small RTS world (town hall + mine + forest blob) on a 96×64 bitfield with a mix of peon roles: **mine peons** loop `mine ring slot → harvest → hall slot → deposit → repeat`, **forest peons** loop `nearest tree → harvest from its walkable neighbor → hall slot → deposit → repeat` (each harvest flips one tree cell back to walkable and the navmesh follows), and a few **wanderers** path to random walkable points to keep the avoidance solver under load. Forest blobs respawn when fully chewed. Side panel has per-role spawn buttons, mine/hall slot usage, forest cells remaining, eviction counter, and the standard `NavWorker` stats. The peon FSM and slot-reservation logic live entirely in the demo — `rsnav-crowd` itself ships only the per-agent crowd primitive.

### Doors testbed

```
cargo run -p rsnav-door-demo --release
```

A 76×48 bitfield split into four rooms by a cross of walls, each wall holding two **doors**. A door is a pure obstacle: open, its cells are walkable; closed, they are carved out of the bitfield and the `NavWorker` rebuilds the mesh without the gap. A handful of actors patrol back and forth between two fixed points (`home ⇄ away`); **click a door** on the map — or use the side-panel checkboxes — to open or close it. When a door toggles mid-route, `Crowd::set_nav` revalidates each actor's *remaining* path by line-of-sight: actors the door just blocked drop the stale corridor and replan, unaffected actors keep walking. A fully closed-off actor gets a red ring and retries every tick until a route reopens. `Door::rect` / `Door::horizontal` / `Door::vertical` are the demo's door-authoring helpers.

### Multi-tile world demo

```
cargo run -p rsnav-world-demo --release
```

Place independent navmesh tiles in a shared world and path across the seams. Spawn tiles from the palette (Open / Holed / Pillars), then with the **Move** tool drag them edge-to-edge — seams re-stitch live, links shown as green segments. The **Path** tool (right-click source, left-click goal) routes across tiles; the **Vis** tool (right-click source, move cursor) draws a cross-tile line-of-sight ray, green clear / red blocked. Links are created purely by placement (overlapping border edges auto-connect) — there's no manual weld. Built on `rsnav_navigation::TiledWorld`.

### Programmatic use

Every crate ships runnable examples. The complete, maintained inventory — 23 examples with
their exact `cargo run` lines — lives in
[docs/README.md § Runnable examples](docs/README.md#runnable-examples). Start with:

```
cargo run -p rsnav-navigation --example first_path
```

Bitfield in, path polyline out, in forty lines — walked through in
[docs/01-quickstart.md](docs/01-quickstart.md).

## Crates

| name | what it provides |
| --- | --- |
| `rsnav-common` | `Vertex`, `Polygon`, `Triangle`, `Aabb`, `Mesh2d`, IDs. Geometry helpers (`orient2d`, `incircle`, segment intersection, `Polygon::interior_point` — used for safe hole seeds on concave polygons). Also `offset` (contour offsetting, no arc joins), `planarize` (snap-rounded arrangement of crossing rings), `par` (the hand-rolled work-splitting used by every parallel stage) and `rng` (deterministic sampling). |
| `rsnav-triangle` | Constrained Delaunay triangulator. Faithful Rust port of Shewchuk's `triangle.c` restricted to the `-DCDT_ONLY` subset (no Steiner-point quality refinement, no Voronoi). D&C Delaunay, segment insertion, hole carving, `.poly`/`.node`/`.ele` I/O. |
| `rsnav-polygon-extract` | Bitfield → `PolygonWithHoles`. 4-connectivity region detection, optional collinear-vertex removal, optional zigzag → diagonal smoothing, min-area culling. |
| `rsnav-navmesh` | Runtime mesh: flat vertices + triangles, per-triangle adjacency, edge constraint markers, area, centroid, connected-component region IDs. Per-region accessors (triangles / area / centroid / bounds), area-weighted `random_point` sampling for spawns, `boundary_edges` outline iteration. Versioned little-endian binary format ([FORMAT.md](crates/navmesh/FORMAT.md)). |
| `rsnav-bsp` | BVH (AABB-tree) over a `NavMesh`. `locate(point)` and `nearest(point)`, both `O(log n)` average, plus `query_aabb` broad-phase range queries. |
| `rsnav-navigation` | A* across triangle adjacency, Simple Stupid Funnel string-pull, triangle-walk line-of-sight, nearest-point. `distance_from_wall` rejects narrow portals and pulls portal endpoints inward at wall vertices. **Doors**: runtime edge-cuts (`DoorSet`) — closing a door promotes internal portal edges to walls via one shared `WallInfo` oracle, so A*/LOS/visibility/clearance all react with no mesh or BSP rebuild. **`NavWorld<M>`**: owns `(NavMesh, Bsp, DoorSet, WallInfo)` + user metadata (`NavMetadata` trait, `zone_crossings` for "entered/left zone" path events). **`TiledWorld`**: multi-tile worlds — independent navmeshes placed by world offset, seams auto-stitched by overlapping border edges, cross-tile A* + LOS. |
| `rsnav-pathing` | `PathFollower`: lookahead + monotone arc-progress projection + anti-shortcut bias at corners. No navmesh dependency — operates on any polyline. |
| `rsnav-dynamic` | `NavWorker`: background-thread navmesh updates driven by `Bitfield` snapshots, with lock-free `poll_swap` for game loops. Typed `NavListener` events (`BuildStarted` / `Completed` / `Failed`) and a polling `NavStats` accessor for HUDs and ops dashboards. Coalesces rapid submissions — only the newest snapshot is built. |
| `rsnav-crowd` | Multi-agent crowd primitive: per-agent funnel-pulled corridor + sampled velocity-obstacle local avoidance with per-agent radius. `Crowd::set_nav` keeps still-valid paths across navmesh swaps instead of replanning the whole population. Snaps agents back to the mesh if avoidance pushes them off. No FSM, no formation logic, no slot reservation — those live in user code. |
| `rsnav-demo` | egui authoring + probing app (the *Quick start* demo above). |
| `rsnav-fixtures` | CLI runner for `.json` PSLG fixtures (the *Batch-run* tool above). |
| `rsnav-rtsim` | RTS-style dynamic-obstacles testbed (the *Dynamic-obstacles* app above). |
| `rsnav-crowd-demo` | Multi-agent peon-economy testbed (the *Multi-agent crowd* app above). |
| `rsnav-door-demo` | Togglable-doors testbed (the *Doors* app above). |
| `rsnav-world-demo` | Multi-tile world editor: place navmesh tiles, drag them to stitch seams, path / line-of-sight across tiles (the *Multi-tile world* app below). |

## File format

`navmesh` v1 is a section-based little-endian binary format. The full normative spec is in [`crates/navmesh/FORMAT.md`](crates/navmesh/FORMAT.md). It's designed to be implementable in any language — fixed-width records, no varints/compression/alignment-tricks, unknown section types silently skipped for forward compatibility.

Required sections: `META`, `VERTICES`, `TRIANGLES`. Optional (recomputed if absent): `ADJACENCY`, `EDGE_MARKERS`, `TRI_INFO`. The minimum portable file is `META + VERTICES + TRIANGLES + EDGE_MARKERS` (the markers can't be re-derived without losing the wall information).

## Status

Working and tested (~205 tests pass workspace-wide):

- ✅ CDT round-trip against `triangle.c` reference (`A.poly` → 29 triangles, byte-exact)
- ✅ 200-point random Delaunay stress passes the empty-circumcircle test
- ✅ Wall-distance clearance pathing (rejects narrow portals, pulls funnel endpoints off walls)
- ✅ Runtime doors: closing an edge-cut blocks A*/LOS at the door, opening restores it, mesh & BSP never rebuilt
- ✅ Multi-tile worlds: independent tiles auto-stitch on overlapping borders (incl. mismatched seam triangulations) and path/LOS across seams
- ✅ Real gonav-format fixtures (representative loads: ~1900 tris / 6 regions and ~1250 tris / 23 regions) build clean when pointed at via `--testdata`

Deliberate v1 omissions:

- Steiner point quality refinement (`-q` switch in `triangle`) — no facility for it.
- Self-intersecting PSLG segments (`segmentintersection` in `triangle`) — on the legacy path, `form_skeleton` returns `Err(SelfIntersection)` rather than splitting the crossing. This is no longer the whole story: `build_cdt_with_inset` exists precisely to accept crossing rings, resolving them by snap-rounded planarization plus a winding-number interior test instead of by segment splitting. Crossing input is supported *there*, not in `form_skeleton`. See [docs/13-authored-geometry.md](docs/13-authored-geometry.md).
- Conforming-Delaunay midpoint splitting (`conformingedge` in `triangle`).
- Visibility-region exact (Asano/Suri sweep). The shipped `visibility_region` uses uniform angular sampling — exact enough for visualization at typical resolutions.
- Interactive pan/zoom in the demo (only auto-fit + Fit-view button).
- `TiledWorld` v1: translation-only tile offsets (no rotation), links always open (no per-seam doors yet), no agent clearance across seams, and a slight funnel soft-corner at T-junction seams where one tile's edge links to two of its neighbor's (aligned grids are exact; the fix is merging collinear link portals).

## References

- **Triangle**: Jonathan Richard Shewchuk, *"Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator,"* in *Applied Computational Geometry: Towards Geometric Engineering*, vol. 1148 of LNCS, pp. 203–222, Springer, 1996. [`https://www.cs.cmu.edu/~quake/triangle.html`](https://www.cs.cmu.edu/~quake/triangle.html)
- **Robust predicates**: Shewchuk, *"Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates,"* Discrete & Computational Geometry 18:305–363, 1997.
- **Simple Stupid Funnel**: Mikko Mononen, *"Simple Stupid Funnel Algorithm,"* 2010. [`https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html`](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html)

## License

Dual-licensed under either of

- MIT license
- Apache License, Version 2.0

at your option.
