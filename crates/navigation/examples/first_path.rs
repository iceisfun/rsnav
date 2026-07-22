//! Your first path: a grid of booleans in, a polyline out.
//!
//! Four steps, each of which runs:
//!   1. build a `Bitfield` from an ASCII map (and see what `false` means),
//!   2. build a navmesh from it,
//!   3. ask for a path and read the polyline,
//!   4. handle a click that landed off the mesh.
//!
//! Run: `cargo run -p rsnav-navigation --example first_path`
//!
//! Companion page: docs/01-quickstart.md

use rsnav_common::Vertex;
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildError, BuildOptions};
use rsnav_navigation::{find_path, nearest_point, PathError, PathOptions};
use rsnav_polygon_extract::Bitfield;

/// The map, written top row first because that is how a human reads it.
/// `.` is walkable, `#` is wall. The block in the middle reaches the top
/// border, so the only route from left to right runs along the bottom.
/// The little box on the lower left is sealed: walkable, but unreachable.
const MAP: &[&str] = &[
    "########################",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#.........######.......#",
    "#....####.######.......#",
    "#....#..#.######.......#",
    "#....####..............#",
    "########################",
];

fn main() {
    // --- Step 1: the bitfield --------------------------------------------
    //
    // A *bitfield* is a grid of booleans: `true` = walkable, `false` = wall.
    // That polarity is the first thing to get wrong, so get it wrong on
    // purpose first: `Bitfield::empty` is all-`false`, i.e. solid rock.
    let solid = Bitfield::empty(24, 14);
    match build_navmesh_from_bitfield(&solid, &BuildOptions::default()) {
        Err(BuildError::NoPerimeter) => {
            println!("Bitfield::empty(24, 14) -> BuildError::NoPerimeter (all cells are wall)");
        }
        other => panic!("expected NoPerimeter, got {other:?}"),
    }

    // The real map. Row 0 of a Bitfield is the BOTTOM row (y points up),
    // while MAP[0] is the top row as printed, so the rows are flipped on
    // load. See docs/04-units-and-conventions.md.
    let bf = ascii_to_bitfield(MAP);
    let walkable = bf.data.iter().filter(|c| **c).count();
    println!(
        "bitfield: {}x{} cells, {} walkable",
        bf.width, bf.height, walkable
    );

    // --- Step 2: the navmesh ---------------------------------------------
    //
    // A *navmesh* is that walkable area as triangles instead of cells.
    // `build_navmesh_from_bitfield` runs the whole pipeline and hands back
    // a NavBuild: the mesh, a spatial index (`bsp`) for point lookups, how
    // long the build took, and a generation counter (0 unless a background
    // worker produced it).
    let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
        .expect("the map has walkable cells");
    let nav = &build.navmesh;
    let bsp = &build.bsp;
    println!(
        "navmesh: {} triangles, {} regions, built in {:.3} ms (generation {})",
        nav.triangle_count(),
        nav.region_count,
        build.build_ms,
        build.generation
    );

    // Two regions: the open map, and the sealed box. A *region* is a set of
    // triangles reachable from each other without crossing a wall.

    // --- Step 3: a path ---------------------------------------------------
    //
    // A cell (col, row) covers the square [col, col+1] x [row, row+1], so
    // the centre of cell (3, 6) is (3.5, 6.5).
    let start = Vertex::new(3.5, 6.5);
    let goal = Vertex::new(20.5, 6.5);

    let path = find_path(nav, bsp, start, goal, &PathOptions::default())
        .expect("both endpoints are on the mesh and connected");

    println!("\npath: {} points", path.points.len());
    for (i, p) in path.points.iter().enumerate() {
        println!("  [{i}] ({:.3}, {:.3})", p.x, p.y);
    }
    // points[0] is literally `start` and points.last() is literally `goal`.
    assert_eq!(path.points[0], start);
    assert_eq!(*path.points.last().unwrap(), goal);

    render(MAP, &path.points, start, goal);

    // The three ways find_path can fail, produced deliberately.
    let outside = Vertex::new(-5.0, -5.0);
    let sealed = Vertex::new(6.5, 2.5); // inside the little box
    println!(
        "start off-mesh   -> {:?}",
        find_path(nav, bsp, outside, goal, &PathOptions::default()).unwrap_err()
    );
    println!(
        "goal off-mesh    -> {:?}",
        find_path(nav, bsp, start, outside, &PathOptions::default()).unwrap_err()
    );
    println!(
        "goal sealed off  -> {:?}",
        find_path(nav, bsp, start, sealed, &PathOptions::default()).unwrap_err()
    );
    assert_eq!(
        find_path(nav, bsp, start, sealed, &PathOptions::default()).unwrap_err(),
        PathError::Unreachable
    );

    // --- Step 4: the click that missed ------------------------------------
    //
    // A mouse click is a world position with no guarantee of being on the
    // mesh. `nearest_point` snaps it to the closest point on the surface;
    // path there instead of failing with GoalOutsideMesh.
    let click = Vertex::new(21.5, 15.0); // above the top of the map
    let snapped = nearest_point(nav, bsp, click).expect("mesh is not empty");
    println!(
        "\nclick ({:.1}, {:.1}) is off-mesh; snapped to ({:.3}, {:.3}), {:.3} away",
        click.x, click.y, snapped.point.x, snapped.point.y, snapped.distance
    );
    let path = find_path(nav, bsp, start, snapped.point, &PathOptions::default())
        .expect("the snapped point is on the mesh");
    println!("click-to-walk path: {} points", path.points.len());
}

/// ASCII rows (top row first) -> Bitfield (row 0 = bottom).
fn ascii_to_bitfield(rows: &[&str]) -> Bitfield {
    let height = rows.len() as u32;
    let width = rows[0].len() as u32;
    let mut data = vec![false; (width * height) as usize];
    for (d, row) in rows.iter().enumerate() {
        assert_eq!(row.len() as u32, width, "ragged map");
        let bitfield_row = height as usize - 1 - d;
        for (col, ch) in row.chars().enumerate() {
            data[bitfield_row * width as usize + col] = ch == '.';
        }
    }
    Bitfield::new(width, height, data).expect("data length matches width * height")
}

/// Print the map with the polyline rasterised over it.
fn render(rows: &[&str], points: &[Vertex], start: Vertex, goal: Vertex) {
    let height = rows.len();
    let width = rows[0].len();
    let mut grid: Vec<Vec<char>> = rows.iter().map(|r| r.chars().collect()).collect();

    // A path corner often sits exactly on a cell boundary (the funnel pulls
    // it onto a wall vertex), so a point is drawn in the first adjacent
    // non-wall cell rather than blindly in floor(x), floor(y).
    let mut mark = |p: Vertex, ch: char| {
        let (cx, cy) = (p.x.floor() as isize, p.y.floor() as isize);
        for (col, row) in [(cx, cy), (cx - 1, cy), (cx, cy - 1), (cx - 1, cy - 1)] {
            if col < 0 || row < 0 || col >= width as isize || row >= height as isize {
                continue;
            }
            let cell = &mut grid[height - 1 - row as usize][col as usize];
            if *cell != '#' {
                *cell = ch;
                return;
            }
        }
    };

    for leg in points.windows(2) {
        let (a, b) = (leg[0], leg[1]);
        let steps = (a.distance(b) * 4.0).ceil().max(1.0) as usize;
        for s in 0..=steps {
            mark(a.lerp(b, s as f64 / steps as f64), '*');
        }
    }
    mark(start, 'S');
    mark(goal, 'G');

    println!();
    for row in &grid {
        println!("  {}", row.iter().collect::<String>());
    }
}
