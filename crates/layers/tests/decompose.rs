//! End-to-end layer discovery: soup in, stitched world out.
//!
//! The bridge fixture is the acceptance test for cut placement: the
//! ground must stay ONE uncut layer (continuous under the deck — no
//! seam in open ground), with seams only where the ramps rise out of
//! step reach of the surrounding floor: the rim of the overlap set.

use rsnav_common::{Vec3, Vertex};
use rsnav_layers::{build_layered_world, LayersConfig};
use rsnav_voxel::{synth, PolySoup};
use rsnav_world::WorldPathOptions;

fn merge(base: &mut PolySoup, add: &PolySoup) {
    base.append(add);
}

fn cfg(voxel: f64) -> LayersConfig {
    LayersConfig {
        voxel_size: voxel,
        ..LayersConfig::default()
    }
}

#[test]
fn flat_plane_is_one_layer_no_seams() {
    let soup = synth::plane(10.0, 10.0, 1);
    let built = build_layered_world(&soup, &cfg(0.2)).unwrap();
    assert_eq!(built.stats.per_layer.len(), 1, "{:?}", built.stats);
    assert_eq!(built.stats.connections, 0);
    assert_eq!(built.stats.seam_links, 0);

    // Path across the plane is straight and flat. Positions are local
    // to `built.origin`.
    let w = &built.world;
    let a = w.nearest(Vertex::new(2.0, 2.0), 0.0).unwrap();
    let b = w.nearest(Vertex::new(9.0, 9.0), 0.0).unwrap();
    let path = w
        .find_path(
            rsnav_world::WorldPoint { layer: a.layer, pos: a.point },
            rsnav_world::WorldPoint { layer: b.layer, pos: b.point },
            &WorldPathOptions::default(),
        )
        .unwrap();
    assert!((path.length() - a.point.distance(b.point)).abs() < 1e-9);
}

/// Floor + ramp + raised platform. Nothing overlaps vertically here,
/// so the decomposer may keep floor, ramp, and platform interior in
/// one chart (their footprints never share vertices — the cliff rim
/// splits into narrow flanking layers instead). What must hold: the
/// floor dominates, the cliff rim produced seams, no cut crosses open
/// floor, and the platform is walkable from the floor with the right
/// climb.
#[test]
fn platform_splits_at_the_ramp_crest() {
    let soup = synth::floor_with_ramp_and_platform();
    let built = build_layered_world(&soup, &cfg(0.2)).unwrap();

    assert!(
        built.stats.per_layer.len() >= 2,
        "the cliff rim must split off the floor chart: {:?}",
        built.stats
    );
    assert!(built.stats.connections >= 1, "the rim must be seamed, not walled");

    // The largest layer is the floor; it must remain one connected
    // piece with most of the walkable area.
    let mut areas: Vec<f64> = built.stats.per_layer.iter().map(|l| l.walkable_area).collect();
    areas.sort_by(|a, b| b.total_cmp(a));
    assert!(
        areas[0] > areas[1..].iter().sum::<f64>(),
        "floor should dominate: {areas:?}"
    );

    // From the floor far corner up onto the platform. World-space:
    // floor spans roughly [-6,6]×[-4,4] at z=0; platform center
    // ≈ (8, 0, 1.6). Convert to local grid frame.
    let w = &built.world;
    let local = |x: f64, y: f64| Vertex::new(x - built.origin.x, y - built.origin.y);
    let a = w.nearest(local(-5.0, -3.0), 0.0).expect("floor point");
    let b = w.nearest(local(8.0, 0.0), 1.6).expect("platform point");
    assert!((b.z - 1.6).abs() < 0.3, "goal is on the platform top: {b:?}");
    let path = w
        .find_path(
            rsnav_world::WorldPoint { layer: a.layer, pos: a.point },
            rsnav_world::WorldPoint { layer: b.layer, pos: b.point },
            &WorldPathOptions::default(),
        )
        .expect("platform reachable from the floor");
    // The path climbs ~1.5-1.6 in z.
    let dz = path.points.last().unwrap().z - path.points.first().unwrap().z;
    assert!(dz > 1.2, "path should climb the ramp: dz = {dz}");
}

/// Ground plane with a bridge deck crossing above it: THE ∂O test.
fn bridge_soup() -> PolySoup {
    // Ground 24 × 16, centered at origin.
    let mut soup = synth::plane(24.0, 16.0, 1);
    // Deck: 4 wide (y ∈ [-2, 2]), spanning x ∈ [-6, 6] at z = 2.5 —
    // enough clearance (1.8 needed) to walk underneath.
    let deck = {
        let mut d = PolySoup::new();
        d.vertices = vec![
            Vec3::new(-6.0, -2.0, 2.5),
            Vec3::new(6.0, -2.0, 2.5),
            Vec3::new(6.0, 2.0, 2.5),
            Vec3::new(-6.0, 2.0, 2.5),
        ];
        d.triangles = vec![[0, 1, 2], [0, 2, 3]];
        d
    };
    merge(&mut soup, &deck);
    // Ramps down to the ground at both ends: run 5, rise 2.5.
    // West ramp rises from x = -11 (z 0) to x = -6 (z 2.5).
    merge(&mut soup, &synth::ramp(-11.0, 5.0, 2.5, 4.0));
    // East ramp descends from x = 6 (z 2.5) to x = 11 (z 0): build the
    // rising ramp at x ∈ [6, 11] and mirror it about x = 8.5.
    let mut east = synth::ramp(6.0, 5.0, 2.5, 4.0);
    for v in &mut east.vertices {
        v.x = 17.0 - v.x;
    }
    merge(&mut soup, &east);
    soup
}

#[test]
fn bridge_ground_stays_uncut_and_seams_sit_on_the_ramps() {
    let built = build_layered_world(&bridge_soup(), &cfg(0.2)).unwrap();
    let w = &built.world;
    let local = |x: f64, y: f64| Vertex::new(x - built.origin.x, y - built.origin.y);

    // Ground must be ONE layer: the two sides of the deck's shadow are
    // connected underneath (2.5 clearance > 1.8 required), so points
    // north and south of the bridge and directly under it are all the
    // same layer, reachable without any seam crossing.
    let north = w.nearest(local(0.0, 6.0), 0.0).unwrap();
    let south = w.nearest(local(0.0, -6.0), 0.0).unwrap();
    let under = w.nearest(local(0.0, 0.0), 0.0).unwrap();
    assert_eq!(north.layer, south.layer);
    assert_eq!(north.layer, under.layer);
    let under_path = w
        .find_path(
            rsnav_world::WorldPoint { layer: north.layer, pos: north.point },
            rsnav_world::WorldPoint { layer: south.layer, pos: south.point },
            &WorldPathOptions::default(),
        )
        .expect("walk under the bridge");
    assert!(
        under_path.triangles.iter().all(|&(l, _)| l == north.layer),
        "under-bridge path must stay on the ground layer"
    );
    // It should pass under the deck, not detour around its ends: with
    // a straight-ish crossing the length stays near 12.
    assert!(
        under_path.length() < 14.0,
        "under-bridge path detoured: {} via {:?}",
        under_path.length(),
        under_path.points.len()
    );

    // Deck top is a different layer, reachable via the ramps.
    let deck = w.nearest(local(0.0, 0.0), 2.5).unwrap();
    assert_ne!(deck.layer, north.layer);
    let onto = w
        .find_path(
            rsnav_world::WorldPoint { layer: north.layer, pos: north.point },
            rsnav_world::WorldPoint { layer: deck.layer, pos: deck.point },
            &WorldPathOptions::default(),
        )
        .expect("climb onto the deck");
    assert!(onto.points.last().unwrap().z > 2.0);

    // ∂O: every seam sub-edge must sit on the ramps or deck edges —
    // i.e. inside the bridge structure's xy band (|y| ≤ 2 + one cell),
    // never out in the open ground.
    let band = 2.0 + 2.0 * built.cell_size;
    let mid_y = -built.origin.y; // world y = 0 in local frame
    for conn in w.connections() {
        for se in &conn.sub_edges {
            let layer = w.layer(se.a.layer);
            let tri = layer.navmesh.triangle(se.a.tri);
            let (va, vb) = tri.edge_vertices(se.a.edge as usize);
            for v in [layer.navmesh.vertex(va), layer.navmesh.vertex(vb)] {
                assert!(
                    (v.y - mid_y).abs() <= band,
                    "seam vertex {v:?} strayed outside the bridge band (|y-{mid_y}| ≤ {band})"
                );
            }
        }
    }
}

#[test]
fn deterministic_across_runs() {
    let soup = bridge_soup();
    let a = build_layered_world(&soup, &cfg(0.2)).unwrap();
    let b = build_layered_world(&soup, &cfg(0.2)).unwrap();
    assert_eq!(a.stats.per_layer.len(), b.stats.per_layer.len());
    for (x, y) in a.stats.per_layer.iter().zip(&b.stats.per_layer) {
        assert_eq!(x.spans, y.spans);
        assert_eq!(x.triangles, y.triangles);
    }
    for (la, lb) in (0..a.stats.per_layer.len()).map(|i| {
        (
            &a.world.layer(i as u32).navmesh,
            &b.world.layer(i as u32).navmesh,
        )
    }) {
        assert_eq!(la.to_bytes(), lb.to_bytes(), "navmesh bytes must be identical");
    }
}
