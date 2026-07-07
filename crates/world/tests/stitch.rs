//! End-to-end stitch tests: two hand-cut layers sharing one seam chain.
//!
//! Layer 0 is flat ground on `[0,10]×[0,10]` (z = 0). Layer 1 is a ramp
//! on `[10,20]×[0,10]` rising to z = 5 at x = 20. The shared cut at
//! x = 10 is the seam: the 3D chain (10,0)–(10,5)–(10,10) inserted
//! verbatim into both PSLGs with connection marker 0.

use rsnav_common::Vertex;
use rsnav_navmesh::{build_from_cdt, connection_marker, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};
use rsnav_world::{World, WorldBuildError, WorldPathError, WorldPathOptions, WorldPoint};

/// Triangulate a simple ring of points with per-segment markers.
fn build_layer(pts: &[(f64, f64)], markers: &[i32], z: impl Fn(Vertex) -> f64) -> NavMesh {
    assert_eq!(pts.len(), markers.len());
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
            .map(|i| PslgSegment {
                a: i,
                b: (i + 1) % n,
                marker: markers[i as usize],
            })
            .collect(),
        holes: Vec::new(),
    };
    form_skeleton(&mut mesh, &pslg, None).unwrap();
    let mut nav = build_from_cdt(&mesh);
    nav.assign_vertex_z(z);
    nav
}

const SEAM: i32 = 0; // connection id used by the fixtures

fn ground_layer() -> NavMesh {
    // CCW ring; the seam chain (10,0)→(10,5)→(10,10) runs up the right
    // side.
    build_layer(
        &[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)],
        &[
            1,                              // (0,0)→(10,0)
            connection_marker(SEAM as u32), // (10,0)→(10,5)
            connection_marker(SEAM as u32), // (10,5)→(10,10)
            1,                              // (10,10)→(0,10)
            1,                              // (0,10)→(0,0)
        ],
        |_| 0.0,
    )
}

fn ramp_layer() -> NavMesh {
    // Same seam chain traversed the other way (down the left side),
    // with bit-identical vertices.
    build_layer(
        &[(10.0, 0.0), (20.0, 0.0), (20.0, 10.0), (10.0, 10.0), (10.0, 5.0)],
        &[
            1,                              // (10,0)→(20,0)
            1,                              // (20,0)→(20,10)
            1,                              // (20,10)→(10,10)
            connection_marker(SEAM as u32), // (10,10)→(10,5)
            connection_marker(SEAM as u32), // (10,5)→(10,0)
        ],
        |v| (v.x - 10.0) * 0.5,
    )
}

/// A free-floating third layer sharing no seam with the others.
fn island_layer() -> NavMesh {
    build_layer(
        &[(100.0, 0.0), (110.0, 0.0), (110.0, 10.0), (100.0, 10.0)],
        &[1, 1, 1, 1],
        |_| 0.0,
    )
}

#[test]
fn seam_sub_edges_match_bit_exactly() {
    let world = World::build(vec![ground_layer(), ramp_layer()]).unwrap();
    assert_eq!(world.layer_count(), 2);
    assert_eq!(world.connections().len(), 1);
    let c = &world.connections()[0];
    assert_eq!(c.id, SEAM as u32);
    assert_eq!(c.sub_edges.len(), 2, "the seam chain has two sub-edges");
    for s in &c.sub_edges {
        assert_ne!(s.a.layer, s.b.layer, "each sub-edge joins the two layers");
        // The link is symmetric.
        assert_eq!(world.seam_neighbor(s.a), Some(s.b));
        assert_eq!(world.seam_neighbor(s.b), Some(s.a));
    }
}

#[test]
fn reachability_spans_the_seam_but_not_the_island() {
    let world = World::build(vec![ground_layer(), ramp_layer(), island_layer()]).unwrap();
    let t0 = world
        .layer(0)
        .bsp
        .locate(&world.layer(0).navmesh, Vertex::new(2.0, 5.0))
        .unwrap();
    let t1 = world
        .layer(1)
        .bsp
        .locate(&world.layer(1).navmesh, Vertex::new(18.0, 5.0))
        .unwrap();
    let t2 = world
        .layer(2)
        .bsp
        .locate(&world.layer(2).navmesh, Vertex::new(105.0, 5.0))
        .unwrap();
    assert!(world.reachable((0, t0), (1, t1)));
    assert!(!world.reachable((0, t0), (2, t2)));
    assert!(!world.reachable((1, t1), (2, t2)));
}

#[test]
fn unmatched_seam_edge_fails_the_build() {
    match World::build(vec![ground_layer()]) {
        Err(WorldBuildError::UnmatchedSeamEdge { connection, layer, .. }) => {
            assert_eq!(connection, SEAM as u32);
            assert_eq!(layer, 0);
        }
        other => panic!("expected UnmatchedSeamEdge, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn path_crosses_the_seam_with_true_3d_cost() {
    let world = World::build(vec![ground_layer(), ramp_layer()]).unwrap();
    let start = WorldPoint { layer: 0, pos: Vertex::new(2.0, 5.0) };
    let goal = WorldPoint { layer: 1, pos: Vertex::new(18.0, 5.0) };
    let path = world
        .find_path(start, goal, &WorldPathOptions::default())
        .unwrap();

    // The corridor changes layers exactly once.
    let switches = path
        .triangles
        .windows(2)
        .filter(|w| w[0].0 != w[1].0)
        .count();
    assert_eq!(switches, 1, "corridor: {:?}", path.triangles);

    // Endpoints and heights: start flat, goal 4 up the ramp.
    assert_eq!(path.points.first().map(|p| p.pos), Some(start.pos));
    assert_eq!(path.points.last().map(|p| p.pos), Some(goal.pos));
    assert!((path.points.first().unwrap().z - 0.0).abs() < 1e-9);
    assert!((path.points.last().unwrap().z - 4.0).abs() < 1e-9);

    // Heights are continuous across the seam: consecutive points never
    // jump z without moving.
    for w in path.points.windows(2) {
        if w[0].pos == w[1].pos {
            assert!((w[0].z - w[1].z).abs() < 1e-9, "z jump at {:?}", w[0].pos);
        }
    }

    // Length: exactly the surface geodesic — 8 flat, then √(8² + 4²)
    // up the ramp. The cross-seam funnel pulls one string over the
    // whole corridor, so the seam adds no kink.
    let expect = 8.0 + (8.0f64 * 8.0 + 4.0 * 4.0).sqrt();
    let len = path.length();
    assert!(
        (len - expect).abs() < 1e-9,
        "path length {len} != surface geodesic {expect}: {:?}",
        path.points
    );
}

#[test]
fn same_layer_pathing_still_works() {
    let world = World::build(vec![ground_layer(), ramp_layer()]).unwrap();
    let path = world
        .find_path(
            WorldPoint { layer: 0, pos: Vertex::new(1.0, 1.0) },
            WorldPoint { layer: 0, pos: Vertex::new(9.0, 9.0) },
            &WorldPathOptions::default(),
        )
        .unwrap();
    assert!(path.triangles.iter().all(|&(l, _)| l == 0));
    assert!((path.length() - (8.0f64 * 8.0 + 8.0 * 8.0).sqrt()).abs() < 1e-9);
}

#[test]
fn unreachable_island_reports_unreachable() {
    let world = World::build(vec![ground_layer(), ramp_layer(), island_layer()]).unwrap();
    let err = world
        .find_path(
            WorldPoint { layer: 0, pos: Vertex::new(2.0, 5.0) },
            WorldPoint { layer: 2, pos: Vertex::new(105.0, 5.0) },
            &WorldPathOptions::default(),
        )
        .unwrap_err();
    assert_eq!(err, WorldPathError::Unreachable);
}

#[test]
fn clearance_applies_across_the_world() {
    let world = World::build(vec![ground_layer(), ramp_layer()]).unwrap();
    // A fat agent still fits through the 5-wide seam sub-edges.
    let path = world
        .find_path(
            WorldPoint { layer: 0, pos: Vertex::new(2.0, 5.0) },
            WorldPoint { layer: 1, pos: Vertex::new(18.0, 5.0) },
            &WorldPathOptions { distance_from_wall: 1.0 },
        )
        .unwrap();
    assert!(path.length() > 0.0);
}
