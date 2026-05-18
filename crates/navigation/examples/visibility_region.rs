//! Compute and print the visibility region from a point inside a 10×10
//! room with a 2×2 central pillar.
//!
//! Run with:
//!   cargo run -p rsnav-navigation --example visibility_region

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_navigation::visibility_region;
use rsnav_navmesh::build_from_cdt;
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

fn main() {
    let pts: &[(f64, f64)] = &[
        (0.0, 0.0),   // 0
        (10.0, 0.0),  // 1
        (10.0, 10.0), // 2
        (0.0, 10.0),  // 3
        (4.0, 4.0),   // 4
        (6.0, 4.0),   // 5
        (6.0, 6.0),   // 6
        (4.0, 6.0),   // 7
    ];
    let pslg = Pslg {
        vertices: pts.iter().map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y))).collect(),
        segments: vec![
            PslgSegment { a: 0, b: 1, marker: 1 },
            PslgSegment { a: 1, b: 2, marker: 1 },
            PslgSegment { a: 2, b: 3, marker: 1 },
            PslgSegment { a: 3, b: 0, marker: 1 },
            PslgSegment { a: 4, b: 5, marker: 2 },
            PslgSegment { a: 5, b: 6, marker: 2 },
            PslgSegment { a: 6, b: 7, marker: 2 },
            PslgSegment { a: 7, b: 4, marker: 2 },
        ],
        holes: vec![PslgHole { point: Vertex::new(5.0, 5.0) }],
    };
    let mut cdt = CdtMesh::new();
    for v in &pslg.vertices {
        cdt.push_vertex(VertexSlot::new(v.position, 0));
    }
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None);
    carve_holes(&mut cdt, &pslg, false);
    let nav = build_from_cdt(&cdt);
    let bsp = Bsp::build(&nav);

    println!("10×10 room with a 2×2 hole centered at (5, 5)\n");

    // Three observer positions to compare.
    for (label, source) in [
        ("center of north corridor    ", Vertex::new(5.0, 8.0)),
        ("south-west corner           ", Vertex::new(1.0, 1.0)),
        ("right next to the pillar    ", Vertex::new(3.9, 5.0)),
    ] {
        let vr = visibility_region(&nav, &bsp, source, 20.0, 64)
            .expect("source should be inside the mesh");
        // Compute the bounding box of the visible region as a quick
        // proxy for "how far can the observer see?".
        let mut min = Vertex::new(f64::INFINITY, f64::INFINITY);
        let mut max = Vertex::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in &vr.boundary {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
        // Approximate visible area by the fan-from-source sum of triangle areas.
        let n = vr.boundary.len();
        let mut area = 0.0;
        for i in 0..n {
            let a = vr.boundary[i];
            let b = vr.boundary[(i + 1) % n];
            let s = vr.source;
            area += ((a.x - s.x) * (b.y - s.y) - (a.y - s.y) * (b.x - s.x)).abs() * 0.5;
        }
        println!(
            "  {}({:>4.1}, {:>4.1})  bbox [{:.1}..{:.1}]×[{:.1}..{:.1}]  visible area ≈ {:.1}",
            label, source.x, source.y, min.x, max.x, min.y, max.y, area
        );
    }
}
