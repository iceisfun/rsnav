//! Regression: the scene captured from the demo where a hole ring
//! crosses the perimeter ring (4 of its 9 vertices lie outside, 2 edge
//! crossings). The legacy pipeline cannot build this: `form_skeleton`
//! refuses crossing constraints with `SegmentInsertError::SelfIntersection`.
//!
//! The inset pipeline (planarize + winding cull) must build it instead:
//! only the hole ∩ perimeter region is carved, the hole's outside lobe
//! is exterior anyway, and no seed points are involved.
//!
//! Coordinates are snapshotted from `rsnav-debug.json` (also committed
//! as `fixtures/scenes/hole-crosses-perimeter.json` for manual replay
//! through the demo's Load).

use rsnav_common::geom::nearest_point_on_segment;
use rsnav_common::{Polygon, Vertex};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
use rsnav_triangle::segment::SegmentInsertError;
use rsnav_triangle::{
    build_cdt_with_inset, delaunay, form_skeleton, CdtMesh, DivConqOptions, InsetOptions,
    InsetRing, RingKind, VertexSlot,
};

const PERIMETER: &[(f64, f64)] = &[
    (291.708984375, 319.86065673828125),
    (601.40625, 206.9029541015625),
    (1065.7698974609375, 165.7528076171875),
    (1359.2867431640625, 317.6336364746094),
    (1440.446533203125, 654.1515502929688),
    (1411.012939453125, 840.80078125),
    (1196.244873046875, 905.8601684570312),
    (887.9534912109375, 924.484375),
    (476.42303466796875, 849.0909423828125),
    (272.90509033203125, 708.6752319335938),
    (177.515625, 536.4886474609375),
    (227.685546875, 391.5384826660156),
];

const HOLE: &[(f64, f64)] = &[
    (581.4324340820312, 375.1197204589844),
    (495.76171875, 351.59271240234375),
    (441.1597900390625, 304.02117919921875),
    (418.80078125, 215.89840698242188),
    (506.1546630859375, 118.7509994506836),
    (596.4187622070312, 113.93620300292969),
    (720.9847412109375, 166.8599090576172),
    (738.1031494140625, 248.88311767578125),
    (647.4683837890625, 376.462158203125),
];

fn to_polygon(pts: &[(f64, f64)]) -> Polygon {
    Polygon::from_vertices(pts.iter().map(|(x, y)| Vertex::new(*x, *y)))
}

/// Sanity: the fixture must actually exercise the failure mode — a hole
/// that genuinely straddles the perimeter. If someone "cleans up" the
/// coordinates the regression stops testing anything.
fn assert_fixture_straddles() {
    let perim = to_polygon(PERIMETER);
    let outside = HOLE
        .iter()
        .filter(|(x, y)| !perim.contains(Vertex::new(*x, *y)))
        .count();
    assert_eq!(
        outside, 4,
        "fixture invariant broken: expected 4 hole vertices outside the \
         perimeter, found {outside}"
    );
}

/// Pin the bug: the legacy delaunay → form_skeleton path must keep
/// failing on this scene with `SelfIntersection`. Doubles as a tripwire
/// if form_skeleton's contract ever changes underneath the inset work.
#[test]
fn legacy_path_still_fails() {
    assert_fixture_straddles();

    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    let mut next_idx = 0u32;
    for ring in [PERIMETER, HOLE] {
        let start = next_idx;
        for (x, y) in ring {
            cdt.push_vertex(VertexSlot::new(Vertex::new(*x, *y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(*x, *y)));
            next_idx += 1;
        }
        let n = ring.len() as u32;
        for i in 0..n {
            pslg.segments.push(PslgSegment {
                a: start + i,
                b: start + (i + 1) % n,
                marker: 1,
            });
        }
    }

    delaunay(&mut cdt, DivConqOptions::default());
    let err = form_skeleton(&mut cdt, &pslg, None)
        .expect_err("legacy path unexpectedly built a hole-crossing scene");
    assert!(
        matches!(err, SegmentInsertError::SelfIntersection { .. }),
        "expected SelfIntersection, got {err:?}"
    );
}

fn to_verts(pts: &[(f64, f64)]) -> Vec<Vertex> {
    pts.iter().map(|&(x, y)| Vertex::new(x, y)).collect()
}

fn build_scene(inset: f64) -> NavMesh {
    let perim = to_verts(PERIMETER);
    let hole = to_verts(HOLE);
    let rings = [
        InsetRing { points: &perim, kind: RingKind::Perimeter, marker: 10 },
        InsetRing { points: &hole, kind: RingKind::Hole, marker: 1000 },
    ];
    let built = build_cdt_with_inset(&rings, inset, &InsetOptions::default())
        .expect("inset path must build the hole-crossing scene");
    assert!(built.skipped_rings.is_empty());
    build_from_cdt(&built.mesh)
}

fn covers(nav: &NavMesh, p: Vertex) -> bool {
    nav.triangles.iter().any(|t| {
        let a = nav.vertex(t.vertices[0]);
        let b = nav.vertex(t.vertices[1]);
        let c = nav.vertex(t.vertices[2]);
        rsnav_common::geom::point_in_triangle(a, b, c, p)
    })
}

fn nav_area(nav: &NavMesh) -> f64 {
    nav.triangles
        .iter()
        .map(|t| {
            let a = nav.vertex(t.vertices[0]);
            let b = nav.vertex(t.vertices[1]);
            let c = nav.vertex(t.vertices[2]);
            rsnav_common::geom::signed_area2(a, b, c).abs() * 0.5
        })
        .sum()
}

/// The bug fix: the same scene the legacy path rejects builds cleanly
/// at inset 0. The hole ∩ perimeter region is carved, the hole's
/// outside lobe stays exterior, no seed points involved. Uses the
/// demo's loaded-scene marker scheme (perimeter 10, hole 1000) so the
/// interior-constraint drop is exercised with high markers.
#[test]
fn inset_zero_builds() {
    assert_fixture_straddles();
    let nav = build_scene(0.0);
    assert!(
        nav.triangle_count() > 10,
        "expected a substantial mesh, got {}",
        nav.triangle_count()
    );
    assert_eq!(nav.region_count, 1, "walkable area must be one region");

    let perim = to_polygon(PERIMETER);
    let hole = to_polygon(HOLE);
    // Deep inside the perimeter, far from the hole: walkable.
    let p = Vertex::new(1000.0, 600.0);
    assert!(perim.contains(p) && !hole.contains(p));
    assert!(covers(&nav, p), "walkable point not covered");
    // Inside hole ∩ perimeter: carved.
    let p = Vertex::new(600.0, 300.0);
    assert!(perim.contains(p) && hole.contains(p));
    assert!(!covers(&nav, p), "hole interior still covered");
    // In the hole's outside lobe (outside the perimeter): exterior.
    let p = Vertex::new(550.0, 140.0);
    assert!(!perim.contains(p) && hole.contains(p));
    assert!(!covers(&nav, p), "hole lobe outside the perimeter covered");
}

/// Erosion on the same scene: smaller area, and every kept vertex keeps
/// its clearance from every input boundary segment.
#[test]
fn inset_fifteen_builds_smaller_with_clearance() {
    let inset = 15.0;
    let nav0 = build_scene(0.0);
    let nav = build_scene(inset);
    assert!(nav.triangle_count() > 0);
    let (a0, a1) = (nav_area(&nav0), nav_area(&nav));
    assert!(a1 < a0, "area must shrink: {a1} !< {a0}");

    // Clearance: kept vertices sit on offset contours, so they are at
    // least `inset` from every input boundary segment (minus snap
    // slack, generously bounded at 1.0 here).
    let mut edges: Vec<(Vertex, Vertex)> = Vec::new();
    for ring in [PERIMETER, HOLE] {
        for i in 0..ring.len() {
            let (ax, ay) = ring[i];
            let (bx, by) = ring[(i + 1) % ring.len()];
            edges.push((Vertex::new(ax, ay), Vertex::new(bx, by)));
        }
    }
    for t in &nav.triangles {
        for &vid in &t.vertices {
            let v = nav.vertex(vid);
            for &(a, b) in &edges {
                let d = nearest_point_on_segment(a, b, v).distance(v);
                assert!(
                    d >= inset - 1.0,
                    "vertex {v:?} within {d} of an input wall (inset {inset})"
                );
            }
        }
    }
}
