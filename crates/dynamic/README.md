# rsnav-dynamic

Build a navmesh from a bitfield, and keep it fresh in a game loop.

The one-shot entry point is `build_navmesh_from_bitfield`, which returns a
`NavBuild` (a `NavMesh` + a `Bsp` point-location index) ready to query:

```rust
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_polygon_extract::Bitfield;

let bf = Bitfield::new(width, height, cells).unwrap();
let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
    .expect("the map has walkable cells");
let nav = &build.navmesh;
let bsp = &build.bsp;
```

`BuildOptions::inset` optionally bakes an agent radius into the mesh boundary in
contour space; `BuildOptions::threads` controls per-region parallelism.

For worlds that change at runtime, `NavWorker` owns a background thread that
turns `Bitfield` snapshots into builds, coalescing submissions so only the most
recent snapshot is processed. The main thread swaps in the latest build at frame
start via `NavWorker::poll_swap` — a lock-free hand-off suitable for a render
loop. See [`docs/10-dynamic-rebuilds.md`](https://github.com/iceisfun/rsnav/blob/master/docs/10-dynamic-rebuilds.md).

Depends on [`arc-swap`](https://crates.io/crates/arc-swap) for the lock-free
swap.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
