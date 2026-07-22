//! End-to-end inset pipeline scenarios: the interactions the winding
//! design exists to handle, asserted through `build_from_cdt` regions.
//!
//! 1. Two holes whose dilations merge into one, then at a larger inset
//!    cross the shrunk perimeter — every stage must build.
//! 2. A dumbbell perimeter whose corridor pinches off into two regions,
//!    and a dead-end dog leg that vanishes entirely.

use rsnav_common::Vertex;
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::{build_cdt_with_inset, InsetOptions, InsetRing, RingKind};

fn to_verts(pts: &[(f64, f64)]) -> Vec<Vertex> {
    pts.iter().map(|&(x, y)| Vertex::new(x, y)).collect()
}

fn build(perimeter: &[(f64, f64)], holes: &[&[(f64, f64)]], inset: f64) -> NavMesh {
    let perim = to_verts(perimeter);
    let hole_verts: Vec<Vec<Vertex>> = holes.iter().map(|h| to_verts(h)).collect();
    let mut rings = vec![InsetRing {
        points: &perim,
        kind: RingKind::Perimeter,
        marker: 1,
    }];
    for (i, h) in hole_verts.iter().enumerate() {
        rings.push(InsetRing {
            points: h,
            kind: RingKind::Hole,
            marker: 2 + i as i32,
        });
    }
    let built = build_cdt_with_inset(&rings, inset, &InsetOptions::default())
        .expect("inset build failed");
    build_from_cdt(&built.mesh)
}

fn covers(nav: &NavMesh, x: f64, y: f64) -> bool {
    let p = Vertex::new(x, y);
    nav.triangles.iter().any(|t| {
        let a = nav.vertex(t.vertices[0]);
        let b = nav.vertex(t.vertices[1]);
        let c = nav.vertex(t.vertices[2]);
        rsnav_common::geom::point_in_triangle(a, b, c, p)
    })
}

/// 100x50 room, two 10x10 holes with a 6-wide gap between them.
const ROOM: &[(f64, f64)] = &[(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)];
const HOLE_A: &[(f64, f64)] = &[(20.0, 20.0), (30.0, 20.0), (30.0, 30.0), (20.0, 30.0)];
const HOLE_B: &[(f64, f64)] = &[(36.0, 20.0), (46.0, 20.0), (46.0, 30.0), (36.0, 30.0)];

#[test]
fn two_holes_merge_under_dilation() {
    // Inset 0: the gap between the holes is walkable.
    let nav = build(ROOM, &[HOLE_A, HOLE_B], 0.0);
    assert_eq!(nav.region_count, 1);
    assert!(covers(&nav, 33.0, 25.0), "gap must be walkable at inset 0");
    assert!(!covers(&nav, 25.0, 25.0), "hole A interior walkable?");
    assert!(!covers(&nav, 41.0, 25.0), "hole B interior walkable?");

    // Inset 4: dilated holes overlap (gap 6 < 2*4) — one merged hole.
    // The mesh still builds; the gap is gone; the surrounding ring is
    // still one region.
    let nav = build(ROOM, &[HOLE_A, HOLE_B], 4.0);
    assert!(nav.triangle_count() > 0);
    assert_eq!(nav.region_count, 1, "ring around the merged hole");
    assert!(!covers(&nav, 33.0, 25.0), "gap must be swallowed by the merge");
    // Corridor below the merged hole: perimeter floor moved to y=4,
    // hole floor moved down to y=16 — still walkable between them.
    assert!(covers(&nav, 33.0, 10.0), "corridor under the holes lost");
    // Far side of the room is untouched.
    assert!(covers(&nav, 80.0, 25.0));
}

#[test]
fn merged_hole_crosses_shrunk_perimeter() {
    // Inset 18: perimeter shrinks to [18,82]x[18,32]; the merged hole
    // dilates to roughly [2,64]x[2,48], sticking out past the walkable
    // band on both sides — the "grown hole crosses shrunk perimeter"
    // case, at full strength. Must build; only the right remnant stays.
    let nav = build(ROOM, &[HOLE_A, HOLE_B], 18.0);
    assert!(nav.triangle_count() > 0, "right-side remnant must survive");
    assert_eq!(nav.region_count, 1);
    assert!(covers(&nav, 75.0, 25.0), "right remnant lost");
    assert!(!covers(&nav, 30.0, 25.0), "hole zone still walkable");
    assert!(!covers(&nav, 50.0, 25.0), "hole zone edge still walkable");
    // And beyond the shrunk perimeter nothing is walkable.
    assert!(!covers(&nav, 10.0, 25.0));
    assert!(!covers(&nav, 50.0, 10.0));
}

/// Dumbbell: two 20x20 rooms joined by a 20-long, 4-wide corridor.
const DUMBBELL: &[(f64, f64)] = &[
    (0.0, 0.0),
    (20.0, 0.0),
    (20.0, 8.0),
    (40.0, 8.0),
    (40.0, 0.0),
    (60.0, 0.0),
    (60.0, 20.0),
    (40.0, 20.0),
    (40.0, 12.0),
    (20.0, 12.0),
    (20.0, 20.0),
    (0.0, 20.0),
];

#[test]
fn dumbbell_corridor_narrows_then_splits() {
    // Inset 0 and 1: still one region, corridor walkable.
    let nav = build(DUMBBELL, &[], 0.0);
    assert_eq!(nav.region_count, 1);
    assert!(covers(&nav, 30.0, 10.0), "corridor center at inset 0");

    let nav = build(DUMBBELL, &[], 1.0);
    assert_eq!(nav.region_count, 1, "2-wide corridor still connects");
    assert!(covers(&nav, 30.0, 10.0), "corridor center at inset 1");

    // Inset 3: corridor width 4 < 2*3 — pinched off. Two regions.
    let nav = build(DUMBBELL, &[], 3.0);
    assert!(nav.triangle_count() > 0);
    assert_eq!(nav.region_count, 2, "corridor must pinch into two rooms");
    assert!(covers(&nav, 10.0, 10.0), "left room lost");
    assert!(covers(&nav, 50.0, 10.0), "right room lost");
    assert!(!covers(&nav, 30.0, 10.0), "corridor survived the pinch");
}

/// Dead-end dog leg: one 20x20 room with a 4-wide, 20-long appendage.
const DOG_LEG: &[(f64, f64)] = &[
    (0.0, 0.0),
    (20.0, 0.0),
    (20.0, 8.0),
    (40.0, 8.0),
    (40.0, 12.0),
    (20.0, 12.0),
    (20.0, 20.0),
    (0.0, 20.0),
];

#[test]
fn dead_end_dog_leg_vanishes() {
    let nav = build(DOG_LEG, &[], 0.0);
    assert_eq!(nav.region_count, 1);
    assert!(covers(&nav, 35.0, 10.0), "dog leg walkable at inset 0");

    // Inset 3: the 4-wide leg is gone entirely; one smaller region.
    let nav = build(DOG_LEG, &[], 3.0);
    assert!(nav.triangle_count() > 0);
    assert_eq!(nav.region_count, 1, "only the room remains");
    assert!(covers(&nav, 10.0, 10.0), "room lost");
    assert!(!covers(&nav, 30.0, 10.0), "dog leg survived");
    assert!(!covers(&nav, 38.0, 10.0), "dog leg tip survived");
}

/// Monotonicity across the interesting radii of the merge scene: area
/// never grows as the inset grows.
#[test]
fn area_monotone_in_inset() {
    let area = |nav: &NavMesh| -> f64 {
        nav.triangles
            .iter()
            .map(|t| {
                let a = nav.vertex(t.vertices[0]);
                let b = nav.vertex(t.vertices[1]);
                let c = nav.vertex(t.vertices[2]);
                rsnav_common::geom::signed_area2(a, b, c).abs() * 0.5
            })
            .sum()
    };
    let mut prev = f64::INFINITY;
    for inset in [0.0, 1.0, 2.0, 4.0, 8.0, 18.0, 24.9] {
        let nav = build(ROOM, &[HOLE_A, HOLE_B], inset);
        let a = area(&nav);
        assert!(
            a <= prev + 1e-6,
            "area grew: {prev} -> {a} at inset {inset}"
        );
        prev = a;
    }
    // Past half the room height the perimeter fully erodes.
    let nav = build(ROOM, &[HOLE_A, HOLE_B], 25.0);
    assert_eq!(nav.triangle_count(), 0);
}
