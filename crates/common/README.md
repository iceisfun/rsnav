# rsnav-common

Shared 2D geometry primitives for the [rsnav](https://github.com/iceisfun/rsnav)
navigation stack.

This is the common vocabulary every other `rsnav-*` crate speaks: `Vertex`,
`Polygon`, `Triangle`, `Mesh2d`, an axis-aligned `Aabb`, and the typed
`VertexId` / `TriangleId` / `PolygonId` index newtypes. All coordinates are
`f64`; the y-axis points up.

```rust
use rsnav_common::Vertex;

let a = Vertex::new(1.0, 2.0);
let b = Vertex::new(4.0, 6.0);
```

You rarely depend on this crate directly except to name points you hand to
`rsnav-dynamic` (to build) and `rsnav-navigation` (to query).

`#![forbid(unsafe_code)]`. Part of the rsnav workspace — see the
[repository](https://github.com/iceisfun/rsnav) and the [`docs/`](https://github.com/iceisfun/rsnav/tree/master/docs)
learning path for the full guide.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
