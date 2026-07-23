# rsnav-navmesh

The runtime navmesh: a flat, query-friendly view of a constrained Delaunay
triangulation plus its derived adjacency, per-triangle metadata, and
reachability regions.

A `NavMesh` is what path search and point-location run against. It also carries
a versioned, little-endian binary file format for saving and loading a baked
mesh — see `FORMAT.md` in this crate for the on-disk spec, and
[`docs/14-saving-and-loading.md`](https://github.com/iceisfun/rsnav/blob/master/docs/14-saving-and-loading.md)
for usage.

A *region* is a set of triangles reachable from one another without crossing a
wall — not a room or a zone. `NavMesh::translate` bakes a world placement into
the vertices so a mesh built at the origin can be dropped anywhere.

`#![forbid(unsafe_code)]`. Part of the [rsnav](https://github.com/iceisfun/rsnav)
workspace. Most users build one via `rsnav-dynamic` rather than constructing it
directly.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
