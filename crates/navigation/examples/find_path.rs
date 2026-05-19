//! Build a small navmesh with a wall in the middle, then run `find_path`
//! with and without a wall-clearance constraint to show the effect.
//!
//! Run with:
//!   cargo run -p rsnav-navigation --example find_path

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_navigation::{find_path, line_of_sight, LineOfSightResult, PathOptions};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

fn main() {
    let (nav, bsp) = build_donut_with_diag_wall();
    println!(
        "navmesh: {} tris, {} region(s)\n",
        nav.triangle_count(),
        nav.region_count
    );

    let start = Vertex::new(0.5, 0.5);
    let goal = Vertex::new(3.5, 3.5);

    // No clearance — funnel pulls the path tight against the hole corner.
    let tight = find_path(&nav, &bsp, start, goal, &PathOptions::default())
        .expect("path should exist");
    println!("distance_from_wall = 0.0  →  {} segments", tight.points.len() - 1);
    for p in &tight.points {
        println!("    ({:.3}, {:.3})", p.x, p.y);
    }
    println!("    total length: {:.3}", path_length(&tight.points));

    // With clearance — funnel pushes corners outward, so the path swings
    // wider around the central hole.
    let safe = find_path(
        &nav,
        &bsp,
        start,
        goal,
        &PathOptions { distance_from_wall: 0.3 },
    )
    .expect("path should exist");
    println!("\ndistance_from_wall = 0.3  →  {} segments", safe.points.len() - 1);
    for p in &safe.points {
        println!("    ({:.3}, {:.3})", p.x, p.y);
    }
    println!("    total length: {:.3}", path_length(&safe.points));

    // Line of sight from start to goal: blocked by the central hole.
    let start_tri = bsp.locate(&nav, start).unwrap();
    println!("\nLOS from start to goal:");
    match line_of_sight(&nav, start_tri, start, goal) {
        LineOfSightResult::Clear => println!("    Clear (visible)"),
        LineOfSightResult::Blocked { point } => {
            println!("    Blocked at ({:.3}, {:.3})", point.x, point.y)
        }
        LineOfSightResult::SourceOutsideMesh => println!("    Source outside mesh"),
    }
}

fn path_length(p: &[Vertex]) -> f64 {
    p.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// 4×4 square with a 1×1 hole in the middle. Path search has to detour.
fn build_donut_with_diag_wall() -> (NavMesh, Bsp) {
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
    let nav = build_from_cdt(&cdt);
    let bsp = Bsp::build(&nav);
    (nav, bsp)
}
