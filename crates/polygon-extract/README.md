# rsnav-polygon-extract

Turn a 2D occupancy grid into polygons (outer rings + holes), and optionally
erode it by an agent radius.

Convention: the `Bitfield` is row-major, `true` = walkable, `false` = wall.
Cell `(col, row)` occupies `[col, col+1] × [row, row+1]`; the y-axis points up,
so `row = 0` is the bottom row. Walkable cells connect with 4-connectivity.

```rust
use rsnav_polygon_extract::{Bitfield, ErodeOptions};

// A 4x4 room with a solid wall border.
let cells = vec![
    false, false, false, false,
    false, true,  true,  false,
    false, true,  true,  false,
    false, false, false, false,
];
let bf = Bitfield::new(4, 4, cells).unwrap();

// Grid-space erosion: pull the walkable area in by one cell of clearance.
let eroded = bf.eroded(&ErodeOptions { radius: 1.0, threads: 0 }).unwrap();
```

Erosion is the only clearance strategy that composes with tiling, because it is
purely grid-local. See [`docs/06-clearance.md`](https://github.com/iceisfun/rsnav/blob/master/docs/06-clearance.md)
for how it compares to contour inset and query-time wall clearance.

`#![forbid(unsafe_code)]`. Part of the [rsnav](https://github.com/iceisfun/rsnav)
workspace.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
