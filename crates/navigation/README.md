# rsnav-navigation

Path search and queries over a `NavMesh`. This is the crate you call at runtime.

- `find_path` — A* across triangle adjacency, then funnel / string-pull to a
  polyline. Honors `PathOptions::distance_from_wall`.
- `line_of_sight` — walk a directed segment triangle-by-triangle, stopping at
  the first wall it hits.
- `path_clear` — revalidate a planned polyline against the current mesh; the
  cheap "do I need to replan?" check after the world has changed.
- `nearest_point` — snap an off-mesh world position (a mouse click) onto the
  navmesh surface before pathing to it.

Higher-level containers live here too: `NavWorld` (a mesh + doors you can open
and close at runtime) and `TiledWorld` (many independently-built tiles stitched
into one queryable world).

```rust
use rsnav_navigation::{find_path, nearest_point, PathOptions};
use rsnav_common::Vertex;

// `nav` and `bsp` come from rsnav_dynamic::build_navmesh_from_bitfield.
let start = Vertex::new(3.5, 6.5);
let goal  = Vertex::new(20.5, 6.5);
let path = find_path(&nav, &bsp, start, goal, &PathOptions::default())
    .expect("both endpoints on the mesh and connected");
```

See [`examples/first_path.rs`](examples/first_path.rs) for a complete,
runnable program and [`docs/01-quickstart.md`](https://github.com/iceisfun/rsnav/blob/master/docs/01-quickstart.md)
for the walkthrough.

`#![forbid(unsafe_code)]`. Part of the [rsnav](https://github.com/iceisfun/rsnav)
workspace.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
