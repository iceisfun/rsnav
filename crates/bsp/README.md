# rsnav-bsp

A bounding-volume hierarchy (AABB tree) spatial index over a `NavMesh`'s
triangles. Two queries, both average `O(log n)`:

- `Bsp::locate` — given a point, find the triangle that contains it (or `None`
  if the point is off the mesh).
- `Bsp::nearest` — given a point, find the nearest triangle, the closest point
  on it, and the euclidean distance.

Build is `O(n log n)`: a recursive median split on triangle centroids along the
longest axis of each node's AABB.

Despite the crate name, this is a BVH rather than a classic binary space
partition; the name is historical. `Bsp` is the index `find_path` uses to turn a
world position into a starting triangle.

`#![forbid(unsafe_code)]`. Part of the [rsnav](https://github.com/iceisfun/rsnav)
workspace.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
