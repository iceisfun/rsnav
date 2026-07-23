# rsnav-triangle

Constrained Delaunay triangulation — a direct Rust port of Jonathan Shewchuk's
[Triangle](https://www.cs.cmu.edu/~quake/triangle.html) 1.6, restricted to the
CDT subset (`-DCDT_ONLY` build equivalent): no quality refinement, no
Steiner-point insertion, no Voronoi output.

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
