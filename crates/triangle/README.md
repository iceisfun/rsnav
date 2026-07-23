# rsnav-triangle

Constrained Delaunay triangulation — an independent Rust implementation of the
CDT algorithms popularized by Jonathan Richard Shewchuk's
[Triangle](https://www.cs.cmu.edu/~quake/triangle.html), restricted to the CDT
subset: no quality refinement, no Steiner-point insertion, no Voronoi output.

It is built from the published algorithms and other implementations and is
tested for agreement against `triangle.c` (the bundled `A.poly` round-trips to
the same 29 triangles), but it is **not** a port, clone, or translation of
Triangle's source code, and is **not** subject to Triangle's license — none of
Triangle's code is used here. Credit to Shewchuk's work is gladly given; see the
[references](https://github.com/iceisfun/rsnav#references) in the root README.
The robust geometric predicates follow Shewchuk's separately public-domain
predicates.

Give it a planar straight-line graph (PSLG) — outer ring(s), holes, and
internal wall segments — and it produces a constrained Delaunay triangulation.
It also offers a contour-space inset path (`build_cdt_with_inset`) that bakes an
agent radius into the mesh boundary.

Self-intersecting input segments are rejected with
`SegmentInsertError::SelfIntersection` rather than being split.

Part of the [rsnav](https://github.com/iceisfun/rsnav) workspace. Most users
reach this through `rsnav-dynamic`'s `build_navmesh_from_bitfield` rather than
calling the triangulator directly; see
[`examples/inset_rings.rs`](examples/inset_rings.rs) for direct use.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
