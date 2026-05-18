//! Build a constrained Delaunay triangulation of a hand-coded PSLG —
//! the smallest end-to-end use of `rsnav-triangle`.
//!
//! Run with:
//!   cargo run -p rsnav-triangle --example triangulate_pslg

use rsnav_common::Vertex;
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

fn main() {
    // A 4×4 square (outer) with a 1×1 square hole in the middle.
    //
    //   (0,4) ──────────── (4,4)
    //     │   (1.5,2.5)─(2.5,2.5)  │
    //     │      │           │      │
    //     │   (1.5,1.5)─(2.5,1.5)  │
    //   (0,0) ──────────── (4,0)
    //
    let pts = [
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 4.0),
        (0.0, 4.0),
        (1.5, 1.5),
        (2.5, 1.5),
        (2.5, 2.5),
        (1.5, 2.5),
    ];

    // Push the vertices into the CDT pool and into a parallel Pslg
    // (segments + hole seeds).
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    for (x, y) in pts {
        cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
    }
    // Outer ring 0→1→2→3→0
    for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
        pslg.segments.push(PslgSegment { a, b, marker: 1 });
    }
    // Inner ring 4→5→6→7→4 (the hole)
    for &(a, b) in &[(4, 5), (5, 6), (6, 7), (7, 4)] {
        pslg.segments.push(PslgSegment { a, b, marker: 2 });
    }
    // A point that's *inside* the hole — carve_holes flood-fills from here.
    pslg.holes.push(PslgHole {
        point: Vertex::new(2.0, 2.0),
    });

    // Build: Delaunay on the points → insert segments → carve holes.
    delaunay(&mut cdt, DivConqOptions::default());
    println!("after delaunay:       {} triangles", cdt.live_triangle_count());
    form_skeleton(&mut cdt, &pslg, None);
    println!("after form_skeleton:  {} triangles", cdt.live_triangle_count());
    carve_holes(&mut cdt, &pslg, false);
    println!("after carve_holes:    {} triangles", cdt.live_triangle_count());

    // Walk every live triangle and print it.
    println!("\nlive triangles (1-based vertex ids):");
    let mut shown = 0;
    for (i, slot) in cdt.triangles.iter().enumerate().skip(1) {
        if slot.is_dead() {
            continue;
        }
        if !slot.vertices.iter().all(|v| v.is_valid()) {
            continue; // skip ghosts (shouldn't exist after carve, defensive)
        }
        let v = slot.vertices;
        println!(
            "  tri{:2}: ({}, {}, {})",
            i,
            v[0].get() + 1,
            v[1].get() + 1,
            v[2].get() + 1
        );
        shown += 1;
    }
    println!("\ntotal: {} walkable triangles", shown);
}
