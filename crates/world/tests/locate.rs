//! z-disambiguated point location over stacked layers.

use rsnav_common::Vertex;
use rsnav_navmesh::{build_from_cdt, connection_marker, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};
use rsnav_world::World;

fn build_layer(pts: &[(f64, f64)], markers: &[i32], z: impl Fn(Vertex) -> f64) -> NavMesh {
    let mut mesh = CdtMesh::new();
    for &(x, y) in pts {
        mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
    }
    delaunay(&mut mesh, DivConqOptions::default());
    let n = pts.len() as u32;
    let pslg = Pslg {
        vertices: pts
            .iter()
            .map(|&(x, y)| PslgVertex::new(Vertex::new(x, y)))
            .collect(),
        segments: (0..n)
            .map(|i| PslgSegment { a: i, b: (i + 1) % n, marker: markers[i as usize] })
            .collect(),
        holes: Vec::new(),
    };
    form_skeleton(&mut mesh, &pslg, None).unwrap();
    let mut nav = build_from_cdt(&mesh);
    nav.assign_vertex_z(z);
    nav
}

/// Ground at z = 0 and a flat upper floor at z = 6 sharing the same
/// footprint, joined by a seam at x = 10 (heights meet at the seam via
/// the upper layer's edge ramp — irrelevant here; locate is per-point).
fn stacked_world() -> World {
    let ring = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)];
    let markers = [1, connection_marker(0), connection_marker(0), 1, 1];
    let ground = build_layer(&ring, &markers, |_| 0.0);
    let upper = build_layer(&ring, &markers, |v| (10.0 - v.x) * 0.6);
    World::build(vec![ground, upper]).unwrap()
}

#[test]
fn locate_picks_the_floor_nearest_in_z() {
    let world = stacked_world();
    let p = Vertex::new(2.0, 5.0); // upper surface here is at z = 4.8

    let (l0, _) = world.locate(p, 0.3, f64::INFINITY).unwrap();
    assert_eq!(l0, 0, "z near the ground picks the ground layer");

    let (l1, _) = world.locate(p, 4.5, f64::INFINITY).unwrap();
    assert_eq!(l1, 1, "z near the upper floor picks the upper layer");

    // Height budget: a point 2.0 above the ground with max_dz 0.5
    // matches nothing.
    assert_eq!(world.locate(p, 2.0, 0.5), None);

    // Outside every footprint.
    assert_eq!(world.locate(Vertex::new(50.0, 5.0), 0.0, f64::INFINITY), None);
}

#[test]
fn nearest_snaps_in_3d() {
    let world = stacked_world();
    // Off-mesh in xy, closer to the upper surface in z: x = -1 snaps to
    // x = 0 where ground z = 0 and upper z = 6.
    let n = world.nearest(Vertex::new(-1.0, 5.0), 5.5).unwrap();
    assert_eq!(n.layer, 1);
    assert!((n.point.x - 0.0).abs() < 1e-9);
    assert!((n.z - 6.0).abs() < 1e-9);

    let n = world.nearest(Vertex::new(-1.0, 5.0), 0.5).unwrap();
    assert_eq!(n.layer, 0);
    assert!((n.z - 0.0).abs() < 1e-9);

    // On-mesh point: distance is purely vertical.
    let n = world.nearest(Vertex::new(2.0, 5.0), 0.25).unwrap();
    assert_eq!(n.layer, 0);
    assert!((n.distance - 0.25).abs() < 1e-9);
}
