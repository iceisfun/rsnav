# rsnav

> A Rust constrained-Delaunay triangulator (port of Jonathan Shewchuk's *Triangle*) plus the runtime pieces you actually need to ship navigation: a navmesh binary format, A* + funnel path search with wall-clearance, a BVH for fast point queries, a background worker for dynamic obstacles, a multi-agent crowd with local avoidance, and authoring/probing demos.

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
| `rsnav-dynamic` | `live_worker` | Spawn a `NavWorker`, place + demolish an obstacle, print telemetry events. |
| `rsnav-crowd` | `two_agents_pass` | Two agents head-on on an open arena; print per-tick positions and verify the sampled-VO never overlapped them. |

## Crates

| name | what it provides |
| --- | --- |
| `rsnav-common` | `Vertex`, `Polygon`, `Triangle`, `Aabb`, `Mesh2d`, IDs. Geometry helpers (`orient2d`, `incircle`, segment intersection, `Polygon::interior_point` — used for safe hole seeds on concave polygons). |
| `rsnav-triangle` | Constrained Delaunay triangulator. Faithful Rust port of Shewchuk's `triangle.c` restricted to the `-DCDT_ONLY` subset (no Steiner-point quality refinement, no Voronoi). D&C Delaunay, segment insertion, hole carving, `.poly`/`.node`/`.ele` I/O. |
| `rsnav-polygon-extract` | Bitfield → `PolygonWithHoles`. 4-connectivity region detection, optional collinear-vertex removal, optional zigzag → diagonal smoothing, min-area culling. |
| `rsnav-navmesh` | Runtime mesh: flat vertices + triangles, per-triangle adjacency, edge constraint markers, area, centroid, connected-component region IDs. Per-region accessors (triangles / area / centroid / bounds), area-weighted `random_point` sampling for spawns, `boundary_edges` outline iteration. Versioned little-endian binary format ([FORMAT.md](crates/navmesh/FORMAT.md)). |
| `rsnav-bsp` | BVH (AABB-tree) over a `NavMesh`. `locate(point)` and `nearest(point)`, both `O(log n)` average, plus `query_aabb` broad-phase range queries. |
| `rsnav-navigation` | A* across triangle adjacency, Simple Stupid Funnel string-pull, triangle-walk line-of-sight, nearest-point. `distance_from_wall` rejects narrow portals and pulls portal endpoints inward at wall vertices. |
| `rsnav-pathing` | `PathFollower`: lookahead + monotone arc-progress projection + anti-shortcut bias at corners. No navmesh dependency — operates on any polyline. |
| `rsnav-dynamic` | `NavWorker`: background-thread navmesh updates driven by `Bitfield` snapshots, with lock-free `poll_swap` for game loops. Typed `NavListener` events (`BuildStarted` / `Completed` / `Failed`) and a polling `NavStats` accessor for HUDs and ops dashboards. Coalesces rapid submissions — only the newest snapshot is built. |
| `rsnav-crowd` | Multi-agent crowd primitive: per-agent funnel-pulled corridor + sampled velocity-obstacle local avoidance with per-agent radius. `Crowd::set_nav` keeps still-valid paths across navmesh swaps instead of replanning the whole population. Snaps agents back to the mesh if avoidance pushes them off. No FSM, no formation logic, no slot reservation — those live in user code. |
| `rsnav-demo` | egui authoring + probing app (the *Quick start* demo above). |
| `rsnav-fixtures` | CLI runner for `.json` PSLG fixtures (the *Batch-run* tool above). |
| `rsnav-rtsim` | RTS-style dynamic-obstacles testbed (the *Dynamic-obstacles* app above). |
| `rsnav-crowd-demo` | Multi-agent peon-economy testbed (the *Multi-agent crowd* app above). |
| `rsnav-door-demo` | Togglable-doors testbed (the *Doors* app above). |

## File format

`navmesh` v1 is a section-based little-endian binary format. The full normative spec is in [`crates/navmesh/FORMAT.md`](crates/navmesh/FORMAT.md). It's designed to be implementable in any language — fixed-width records, no varints/compression/alignment-tricks, unknown section types silently skipped for forward compatibility.

Required sections: `META`, `VERTICES`, `TRIANGLES`. Optional (recomputed if absent): `ADJACENCY`, `EDGE_MARKERS`, `TRI_INFO`. The minimum portable file is `META + VERTICES + TRIANGLES + EDGE_MARKERS` (the markers can't be re-derived without losing the wall information).

## Status

Working and tested (~160 tests pass workspace-wide):

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
