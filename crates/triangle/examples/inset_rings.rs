//! Authored ring input: the crossing-tolerant `build_cdt_with_inset`
//! front-end, side by side with the legacy `delaunay` /
//! `form_skeleton` / `carve_holes` path it replaces.
//!
//! Four scenes:
//!   1. A perimeter with a hole fully inside it, at inset 0 and at a
//!      real radius. Erosion shrinks the perimeter and dilates the hole
//!      with the same left-offset primitive.
//!   2. A hole ring that CROSSES the perimeter. The legacy path cannot
//!      build this at all — `form_skeleton` returns
//!      `SegmentInsertError::SelfIntersection` on the first crossing
//!      constraint. The inset path builds it at inset 0, where no
//!      offsetting happens and planarization is the entire point.
//!   3. Input winding is irrelevant: the same perimeter fed clockwise
//!      produces the same area. Normalization is internal.
//!   4. What `skipped_rings` does and does not report. It lists rings
//!      that were DEGENERATE AT ENTRY (fewer than 3 distinct points, or
//!      zero area). It does NOT list a perimeter dropped as provably
//!      fully eroded — that ring vanishes with no entry in the list, so
//!      diffing input ring count against `skipped_rings.len()` will
//!      mislead you.
//!
//! Run with:
//!   cargo run --release -p rsnav-triangle --example inset_rings
//!
//! Use a release build. `carve_by_winding` cross-checks every triangle
//! classification against the brute-force `winding_number` under
//! `debug_assert`, which makes debug runs of this path dramatically
//! slower.

use rsnav_common::geom::signed_area2;
use rsnav_common::Vertex;
use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
use rsnav_triangle::{
    build_cdt_with_inset, delaunay, form_skeleton, CdtMesh, DivConqOptions, InsetOptions,
    InsetRing, RingKind, VertexSlot,
};

fn verts(pts: &[(f64, f64)]) -> Vec<Vertex> {
    pts.iter().map(|&(x, y)| Vertex::new(x, y)).collect()
}

/// Live walkable area. A triangle slot counts only when it is neither
/// dead nor a ghost — the liveness test is two conditions, not one:
/// `is_dead()` OR any vertex id that is not valid (hull-fan ghosts).
/// Slot 0 is the dummy, so every scan starts at 1.
fn live_area(mesh: &CdtMesh) -> f64 {
    let mut area = 0.0;
    for i in 1..mesh.triangles.len() as u32 {
        let slot = mesh.triangle(i);
        if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        let a = mesh.vertex_pos(slot.vertices[0]);
        let b = mesh.vertex_pos(slot.vertices[1]);
        let c = mesh.vertex_pos(slot.vertices[2]);
        area += signed_area2(a, b, c).abs() * 0.5;
    }
    area
}

fn live_tris(mesh: &CdtMesh) -> usize {
    (1..mesh.triangles.len() as u32)
        .filter(|&i| {
            let slot = mesh.triangle(i);
            !slot.is_dead() && slot.vertices.iter().all(|v| v.is_valid())
        })
        .count()
}

fn main() {
    scene_1_hole_inside();
    scene_2_hole_crosses_perimeter();
    scene_3_winding_is_irrelevant();
    scene_4_skipped_rings();
}

// ---------------------------------------------------------------- 1 --

fn scene_1_hole_inside() {
    println!("== scene 1: hole fully inside the perimeter ==");
    let outer = verts(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]);
    let hole = verts(&[(15.0, 15.0), (25.0, 15.0), (25.0, 25.0), (15.0, 25.0)]);
    let rings = [
        InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 10 },
        InsetRing { points: &hole, kind: RingKind::Hole, marker: 20 },
    ];

    for &inset in &[0.0_f64, 2.0, 5.0] {
        let built = build_cdt_with_inset(&rings, inset, &InsetOptions::default())
            .expect("square with an interior hole always builds");
        // At inset r the 40x40 perimeter becomes [r, 40-r]^2 and the
        // 10x10 hole grows to (10 + 2r)^2 — the same left-offset applied
        // to rings that carry opposite natural orientations.
        let expect = (40.0 - 2.0 * inset).powi(2) - (10.0 + 2.0 * inset).powi(2);
        println!(
            "  inset {:>3}: {:>3} live triangles, area {:>8.3} (analytic {:>8.3}), skipped {}",
            inset,
            live_tris(&built.mesh),
            live_area(&built.mesh),
            expect,
            built.skipped_rings.len()
        );
    }
    println!();
}

// ---------------------------------------------------------------- 2 --

/// Perimeter and a hole ring that straddles its right wall: two of the
/// hole's edges cross x = 100. Nothing about this is exotic — a level
/// editor produces it the moment an author drags a hole half out of a
/// room.
const P2_PERIMETER: &[(f64, f64)] = &[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
const P2_HOLE: &[(f64, f64)] = &[(80.0, 40.0), (130.0, 40.0), (130.0, 60.0), (80.0, 60.0)];

fn scene_2_hole_crosses_perimeter() {
    println!("== scene 2: hole ring crosses the perimeter ==");

    // -- legacy path: delaunay -> form_skeleton --------------------
    //
    // The CDT vertex pool and the Pslg vertex list are SEPARATE
    // structures. `PslgSegment::a` / `::b` index the MESH's pool, and
    // `form_skeleton` resolves them through the mesh — so the two must
    // be pushed in the same order, in lockstep, which is why every
    // caller in this workspace does it in one loop.
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    let push = |cdt: &mut CdtMesh, pslg: &mut Pslg, ring: &[(f64, f64)], marker: i32| {
        let base = pslg.vertices.len() as u32;
        for &(x, y) in ring {
            let v = Vertex::new(x, y);
            cdt.push_vertex(VertexSlot::new(v, 0));
            pslg.vertices.push(PslgVertex::new(v));
        }
        let n = ring.len() as u32;
        for i in 0..n {
            pslg.segments.push(PslgSegment {
                a: base + i,
                b: base + (i + 1) % n,
                marker,
            });
        }
    };
    push(&mut cdt, &mut pslg, P2_PERIMETER, 1);
    push(&mut cdt, &mut pslg, P2_HOLE, 2);

    delaunay(&mut cdt, DivConqOptions::default());
    match form_skeleton(&mut cdt, &pslg, None) {
        Ok(()) => println!("  legacy: form_skeleton unexpectedly succeeded"),
        Err(e) => println!("  legacy: form_skeleton failed -> {e}"),
    }

    // -- inset path -------------------------------------------------
    //
    // Same geometry, no hole seed points, no requirement that rings
    // stay disjoint. Note inset 0.0: the offset stage is skipped
    // entirely and the pipeline is still worth reaching for, because
    // planarization is what makes crossing constraints buildable.
    let perim = verts(P2_PERIMETER);
    let hole = verts(P2_HOLE);
    let rings = [
        InsetRing { points: &perim, kind: RingKind::Perimeter, marker: 1 },
        InsetRing { points: &hole, kind: RingKind::Hole, marker: 2 },
    ];
    for &inset in &[0.0_f64, 3.0] {
        let built = build_cdt_with_inset(&rings, inset, &InsetOptions::default())
            .expect("planarized constraints cannot self-intersect");
        println!(
            "  inset  {:>3}: {:>3} live triangles, area {:>9.3}",
            inset,
            live_tris(&built.mesh),
            live_area(&built.mesh)
        );
    }
    // At inset 0 the walkable area is the perimeter minus the part of
    // the hole that lies inside it: 100*100 - 20*20 = 9600. The hole's
    // outside lobe was exterior to begin with and needs no special
    // handling — winding classification is per-triangle and local.
    println!("  (analytic at inset 0: 100*100 - 20*20 = 9600.000)");
    println!();
}

// ---------------------------------------------------------------- 3 --

fn scene_3_winding_is_irrelevant() {
    println!("== scene 3: input winding is normalized internally ==");
    let ccw = verts(&[(0.0, 0.0), (30.0, 0.0), (30.0, 30.0), (0.0, 30.0)]);
    let cw = verts(&[(0.0, 0.0), (0.0, 30.0), (30.0, 30.0), (30.0, 0.0)]);
    for (label, pts) in [("CCW", &ccw), ("CW ", &cw)] {
        let rings = [InsetRing { points: pts, kind: RingKind::Perimeter, marker: 1 }];
        let built = build_cdt_with_inset(&rings, 1.0, &InsetOptions::default()).unwrap();
        println!(
            "  perimeter given {}: area {:>8.3}",
            label,
            live_area(&built.mesh)
        );
    }
    println!("  (both must be 28*28 = 784.000)");
    println!();
}

// ---------------------------------------------------------------- 4 --

fn scene_4_skipped_rings() {
    println!("== scene 4: what skipped_rings reports ==");
    let outer = verts(&[(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]);
    // Two distinct points after consecutive-duplicate removal: degenerate
    // at entry, so this one IS reported.
    let sliver = verts(&[(5.0, 5.0), (5.0, 5.0), (6.0, 6.0)]);
    let rings = [
        InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 },
        InsetRing { points: &sliver, kind: RingKind::Hole, marker: 2 },
    ];
    let built = build_cdt_with_inset(&rings, 0.0, &InsetOptions::default()).unwrap();
    println!(
        "  degenerate hole at input index 1 -> skipped_rings = {:?}",
        built.skipped_rings
    );

    // A perimeter whose bbox min-dimension is <= 2*inset is dropped
    // before offsetting as provably fully eroded. It is NOT pushed to
    // `skipped`, so the list stays empty while the geometry is gone and
    // the build is Ok with zero live triangles.
    let rings = [InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 }];
    let built = build_cdt_with_inset(&rings, 15.0, &InsetOptions::default()).unwrap();
    println!(
        "  20x20 perimeter at inset 15: Ok, {} live triangles, skipped_rings = {:?}",
        live_tris(&built.mesh),
        built.skipped_rings
    );
    println!("  full erosion is Ok-and-empty, never an error, and never reported as skipped");
}
