//! Place three navmesh tiles in a row, stitch their seams, and path across
//! all of them — demonstrating the multi-tile world.
//!
//! Run with:
//!   cargo run -p rsnav-navigation --example tiled_world

use rsnav_common::Vertex;
use rsnav_navigation::TiledWorld;
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
use rsnav_triangle::{carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

/// A 10×10 open tile.
fn open_tile() -> NavMesh {
    let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    for (x, y) in pts {
        cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
    }
    for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
        pslg.segments.push(PslgSegment { a, b, marker: 1 });
    }
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    build_from_cdt(&cdt)
}

/// A 10×10 tile with a 3×3 hole in the middle.
fn holed_tile() -> NavMesh {
    let pts = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
        (0.0, 10.0),
        (3.5, 3.5),
        (6.5, 3.5),
        (6.5, 6.5),
        (3.5, 6.5),
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
    pslg.holes.push(PslgHole { point: Vertex::new(5.0, 5.0) });
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    carve_holes(&mut cdt, &pslg, false);
    build_from_cdt(&cdt)
}

fn main() {
    // Three tiles in a row: open | holed | open, placed by world offset.
    let mut world = TiledWorld::new();
    world.add_tile(open_tile(), Vertex::new(0.0, 0.0));
    world.add_tile(holed_tile(), Vertex::new(10.0, 0.0));
    world.add_tile(open_tile(), Vertex::new(20.0, 0.0));
    world.stitch_all(1e-6);

    println!("{} tiles, {} links stitched\n", world.tile_count(), world.links().len());

    // Path from the first tile, through the holed middle tile, into the last.
    let start = Vertex::new(2.0, 5.0);
    let goal = Vertex::new(28.0, 5.0);
    match world.find_path(start, goal) {
        Some(path) => {
            let total: f64 = path.windows(2).map(|w| w[0].distance(w[1])).sum();
            println!("path ({:.3} long, straight line is 26.0):", total);
            for p in &path {
                println!("    ({:6.3}, {:6.3})", p.x, p.y);
            }
            println!("\nthe detour around the middle tile's hole proves the route");
            println!("crosses both seams and string-pulls across all three tiles.");
        }
        None => println!("no route — did the seams stitch?"),
    }
}
