//! Build a BSP over a small navmesh and run point-in-triangle and
//! nearest-point queries.
//!
//! Run with:
//!   cargo run -p rsnav-bsp --example locate_and_nearest

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_navmesh::build_from_cdt;
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

fn main() {
    let nav = build_donut_navmesh();
    println!(
        "donut navmesh: {} triangles, {} region(s)",
        nav.triangle_count(),
        nav.region_count
    );

    let bsp = Bsp::build(&nav);

    // Sample points: some inside, some in the hole, some outside.
    let samples = [
        ("near outer corner", Vertex::new(0.2, 0.2)),
        ("center of the hole", Vertex::new(2.0, 2.0)),
        ("on the upper strip", Vertex::new(2.0, 3.5)),
        ("far outside the mesh", Vertex::new(-5.0, 5.0)),
        ("just outside left wall", Vertex::new(-0.1, 2.0)),
    ];

    println!("\nlocate(p):");
    for (label, p) in samples {
        match bsp.locate(&nav, p) {
            Some(tri) => println!("  {:24}  ({:>5.1}, {:>5.1})  →  tri {}", label, p.x, p.y, tri.get()),
            None => println!("  {:24}  ({:>5.1}, {:>5.1})  →  outside any triangle", label, p.x, p.y),
        }
    }

    println!("\nnearest(p):");
    for (label, p) in samples {
        match bsp.nearest(&nav, p) {
            Some(n) => println!(
                "  {:24}  ({:>5.1}, {:>5.1})  →  tri {}, snapped to ({:.2}, {:.2}), dist {:.3}",
                label, p.x, p.y, n.triangle.get(), n.point.x, n.point.y, n.distance,
            ),
            None => println!("  {:24}  ({:>5.1}, {:>5.1})  →  mesh is empty", label, p.x, p.y),
        }
    }
}

/// 4×4 outer with a 1×1 hole in the middle, carved through the standard
/// pipeline. The same fixture every other example uses.
fn build_donut_navmesh() -> rsnav_navmesh::NavMesh {
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
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    for (x, y) in pts {
        cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
    }
    for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
        pslg.segments.push(PslgSegment { a, b, marker: 1 });
    }
    for &(a, b) in &[(4, 5), (5, 6), (6, 7), (7, 4)] {
        pslg.segments.push(PslgSegment { a, b, marker: 2 });
    }
    pslg.holes.push(PslgHole { point: Vertex::new(2.0, 2.0) });
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    carve_holes(&mut cdt, &pslg, false);
    build_from_cdt(&cdt)
}
