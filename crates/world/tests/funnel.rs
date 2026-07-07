//! Cross-seam funnel acceptance tests.
//!
//! The three fixtures cover the three regimes:
//! - flat seam           → the seam must be *invisible*: straight line,
//!                         zero kink, length == the 2D ground truth
//! - slope change        → straight in the shared frame, true 3D length
//! - folded (stacked)    → the corridor's projection self-overlaps, the
//!                         hinge unfold must kick in and produce the
//!                         geodesic of the unfolded development

use rsnav_common::Vertex;
use rsnav_navmesh::{build_from_cdt, connection_marker, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};
use rsnav_world::{World, WorldPathOptions, WorldPoint};

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

const C: i32 = 0;

/// `[0,10]×[0,10]` with the seam chain (10,0)–(10,5)–(10,10) on the
/// right.
fn left_square(z: impl Fn(Vertex) -> f64) -> NavMesh {
    build_layer(
        &[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)],
        &[1, connection_marker(C as u32), connection_marker(C as u32), 1, 1],
        z,
    )
}

/// `[10,20]×[0,10]` with the same seam chain on the left.
fn right_square(z: impl Fn(Vertex) -> f64) -> NavMesh {
    build_layer(
        &[(10.0, 0.0), (20.0, 0.0), (20.0, 10.0), (10.0, 10.0), (10.0, 5.0)],
        &[1, 1, 1, connection_marker(C as u32), connection_marker(C as u32)],
        z,
    )
}

#[test]
fn flat_seam_is_invisible() {
    // Both layers flat: the world is one 20×10 rectangle wearing a seam
    // down its middle. The path must be *identical* to the single-mesh
    // ground truth — the straight line — plus the inserted crossing
    // point, which lies exactly on that line (zero kink).
    let world = World::build(vec![left_square(|_| 0.0), right_square(|_| 0.0)]).unwrap();
    let start = WorldPoint { layer: 0, pos: Vertex::new(2.0, 2.0) };
    let goal = WorldPoint { layer: 1, pos: Vertex::new(18.0, 3.0) };
    let path = world
        .find_path(start, goal, &WorldPathOptions::default())
        .unwrap();

    let straight = start.pos.distance(goal.pos);
    assert!(
        (path.length() - straight).abs() < 1e-9,
        "seam introduced a kink: length {} vs straight {straight}: {:?}",
        path.length(),
        path.points
    );
    assert_eq!(path.points.len(), 3, "start, crossing, goal: {:?}", path.points);
    let mid = path.points[1];
    assert_eq!(mid.layer, 0, "crossing attributed to the exited layer");
    assert!((mid.pos.x - 10.0).abs() < 1e-9);
    assert!((mid.pos.y - 2.5).abs() < 1e-9, "crossing at {:?}", mid.pos);
    assert!(path.points.iter().all(|p| p.z == 0.0));
}

#[test]
fn seam_chain_vertex_pins_the_crossing_by_at_most_the_detour() {
    // Known remaining artifact, kept visible on purpose. The seam chain
    // has a vertex at (10,5); when the true optimum crosses one
    // sub-edge but A*'s greedy portal entries make both channels look
    // identical (both project onto the shared vertex), the search can
    // commit to the neighboring sub-edge and the funnel — confined to
    // that channel — pins the crossing to the chain vertex. Same class
    // of artifact as Detour's tile-boundary pinning. The path stays
    // valid and its excess is bounded by the corner detour; a
    // line-of-sight smoothing pass across seams is the systematic fix.
    let world = World::build(vec![left_square(|_| 0.0), right_square(|_| 0.0)]).unwrap();
    let start = WorldPoint { layer: 0, pos: Vertex::new(2.0, 4.0) };
    let goal = WorldPoint { layer: 1, pos: Vertex::new(18.0, 8.0) };
    let path = world
        .find_path(start, goal, &WorldPathOptions::default())
        .unwrap();
    let straight = start.pos.distance(goal.pos);
    let via_vertex = start.pos.distance(Vertex::new(10.0, 5.0))
        + Vertex::new(10.0, 5.0).distance(goal.pos);
    assert!(path.length() >= straight - 1e-9);
    assert!(
        path.length() <= via_vertex + 1e-9,
        "worse than the chain-vertex detour: {} vs {via_vertex}: {:?}",
        path.length(),
        path.points
    );
}

#[test]
fn ramp_seam_walks_the_surface_geodesic() {
    // Right layer rises to z = 5 at x = 20. Start/goal on the y = 5
    // line: the shared-frame straight line *is* the geodesic of this
    // development, so the path is [start, (10,5), goal] with 3D length
    // 8 + √(8² + 4²) = 8 + 4√5.
    let world =
        World::build(vec![left_square(|_| 0.0), right_square(|v| (v.x - 10.0) * 0.5)]).unwrap();
    let path = world
        .find_path(
            WorldPoint { layer: 0, pos: Vertex::new(2.0, 5.0) },
            WorldPoint { layer: 1, pos: Vertex::new(18.0, 5.0) },
            &WorldPathOptions::default(),
        )
        .unwrap();
    let expect = 8.0 + (8.0f64 * 8.0 + 4.0 * 4.0).sqrt();
    assert!(
        (path.length() - expect).abs() < 1e-9,
        "length {} != geodesic {expect}: {:?}",
        path.length(),
        path.points
    );
    // Heights are continuous and end 4 up the ramp.
    assert!((path.points.last().unwrap().z - 4.0).abs() < 1e-9);
    for w in path.points.windows(2) {
        if w[0].pos == w[1].pos {
            assert!((w[0].z - w[1].z).abs() < 1e-9);
        }
    }
}

#[test]
fn folded_layers_unfold_across_the_hinge() {
    // The second layer is a ramp switchback occupying the SAME xy
    // footprint as the ground layer — its chart projects directly on
    // top. A shared-frame funnel would be nonsense (the corridor's
    // projection doubles back through itself); the hinge unfold
    // reflects the entered chart across x = 10 and pulls the string
    // once over the development.
    let ground = left_square(|_| 0.0);
    let upper = build_layer(
        &[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)],
        &[1, connection_marker(C as u32), connection_marker(C as u32), 1, 1],
        |v| (10.0 - v.x) * 0.5, // rises back over the ground layer
    );
    let world = World::build(vec![ground, upper]).unwrap();

    let start = WorldPoint { layer: 0, pos: Vertex::new(2.0, 6.0) };
    let goal = WorldPoint { layer: 1, pos: Vertex::new(2.0, 9.0) };
    let path = world
        .find_path(start, goal, &WorldPathOptions::default())
        .unwrap();

    // Both layers appear in the corridor.
    assert!(path.triangles.iter().any(|&(l, _)| l == 0));
    assert!(path.triangles.iter().any(|&(l, _)| l == 1));

    // In the unfolded development the goal sits at (18, 9); the
    // straight line from (2,6) crosses the hinge at (10, 7.5). Folding
    // back, the path is start → (10,7.5) on the seam → goal, with the
    // two legs of equal projected length (the mirror symmetry of the
    // optimal fold crossing).
    assert_eq!(path.points.len(), 3, "start, crossing, goal: {:?}", path.points);
    let mid = path.points[1];
    assert_eq!(mid.layer, 0);
    assert!((mid.pos.x - 10.0).abs() < 1e-9);
    assert!((mid.pos.y - 7.5).abs() < 1e-9, "crossing at {:?}", mid.pos);
    assert!((mid.z - 0.0).abs() < 1e-9, "seam is at ground height");

    let leg1 = path.points[0].pos.distance(mid.pos);
    let leg2 = mid.pos.distance(path.points[2].pos);
    assert!(
        (leg1 - leg2).abs() < 1e-9,
        "unfolded geodesic crosses the hinge symmetrically: {leg1} vs {leg2}"
    );

    // Goal height: 4 up the switchback at x = 2.
    assert!((path.points[2].z - 4.0).abs() < 1e-9);

    // Total 3D length: leg1 flat + leg2 climbing 4.
    let expect = leg1 + (leg2 * leg2 + 16.0).sqrt();
    assert!(
        (path.length() - expect).abs() < 1e-9,
        "length {} != {expect}",
        path.length()
    );
}

#[test]
fn fat_agent_shrinks_seam_portals_only_at_real_walls() {
    // Clearance pulls the path off wall vertices but must NOT shrink
    // the interior seam vertex (10,5) — the floor continues across the
    // seam, so hugging it is legal.
    let world = World::build(vec![left_square(|_| 0.0), right_square(|_| 0.0)]).unwrap();
    let path = world
        .find_path(
            WorldPoint { layer: 0, pos: Vertex::new(2.0, 2.0) },
            WorldPoint { layer: 1, pos: Vertex::new(18.0, 3.0) },
            &WorldPathOptions { distance_from_wall: 1.5 },
        )
        .unwrap();
    // Straight line still possible: the crossing at (10,2.5) sits above
    // the shrink zone of the real wall corner (10,0) and the interior
    // seam-chain vertex (10,5) is not a wall vertex, so nothing pulls
    // the path off the straight line.
    let straight = Vertex::new(2.0, 2.0).distance(Vertex::new(18.0, 3.0));
    assert!(
        (path.length() - straight).abs() < 1e-9,
        "clearance pinched the seam: {} vs {straight}: {:?}",
        path.length(),
        path.points
    );
}
