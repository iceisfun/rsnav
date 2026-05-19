//! Build a navmesh from an inline PSLG, serialize to bytes, reload, and
//! assert the round trip is exact.
//!
//! Run with:
//!   cargo run -p rsnav-navmesh --example save_and_load

use rsnav_common::Vertex;
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

fn main() {
    // 10×4 corridor with a wall in the middle dividing it into two rooms.
    let pts: &[(f64, f64)] = &[
        (0.0, 0.0),   // 0
        (10.0, 0.0),  // 1
        (10.0, 4.0),  // 2
        (0.0, 4.0),   // 3
        (5.0, 0.0),   // 4 wall bottom
        (5.0, 4.0),   // 5 wall top
    ];
    let pslg = Pslg {
        vertices: pts
            .iter()
            .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
            .collect(),
        segments: vec![
            PslgSegment { a: 0, b: 4, marker: 1 },
            PslgSegment { a: 4, b: 1, marker: 1 },
            PslgSegment { a: 1, b: 2, marker: 1 },
            PslgSegment { a: 2, b: 5, marker: 1 },
            PslgSegment { a: 5, b: 3, marker: 1 },
            PslgSegment { a: 3, b: 0, marker: 1 },
            PslgSegment { a: 4, b: 5, marker: 99 }, // wall
        ],
        holes: Vec::<PslgHole>::new(),
    };

    let mut cdt = CdtMesh::new();
    for v in &pslg.vertices {
        cdt.push_vertex(VertexSlot::new(v.position, 0));
    }
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    carve_holes(&mut cdt, &pslg, false);
    let nav = build_from_cdt(&cdt);

    println!(
        "built navmesh: {} vertices, {} triangles, {} region(s)",
        nav.vertex_count(),
        nav.triangle_count(),
        nav.region_count,
    );
    assert_eq!(nav.region_count, 2, "wall should split the corridor into 2 rooms");

    // Serialize → bytes → deserialize.
    let bytes = nav.to_bytes();
    println!("serialized to {} bytes ({:.1} KiB)", bytes.len(), bytes.len() as f64 / 1024.0);

    let reloaded = NavMesh::from_bytes(&bytes).expect("round-trip failed");
    assert_eq!(reloaded.vertex_count(), nav.vertex_count());
    assert_eq!(reloaded.triangle_count(), nav.triangle_count());
    assert_eq!(reloaded.region_count, nav.region_count);
    for (a, b) in nav.triangles.iter().zip(reloaded.triangles.iter()) {
        assert_eq!(a, b, "triangle slot diverged after round trip");
    }
    println!("round trip verified — all sections identical");

    // Walk every constrained edge and print the marker.
    let mut walls = 0usize;
    for tri in &nav.triangles {
        for edge in 0..3 {
            if tri.edge_markers[edge] != 0 {
                walls += 1;
            }
        }
    }
    println!("{} constrained edge incidences in the mesh (markers != 0)", walls);
}
