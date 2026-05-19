//! Trace a small ASCII-art bitfield into polygons + holes and print a
//! summary.
//!
//! Run with:
//!   cargo run -p rsnav-polygon-extract --example grid_to_polygons

use rsnav_polygon_extract::{extract, Bitfield, ExtractOptions};

fn main() {
    // A 12×8 walkable area with an isolated single cell on the right side
    // and an enclosed hole in the middle.
    //
    //   ############
    //   ###.....#.##
    //   ###.....#.##
    //   ############
    //   ############
    //   ####....####
    //   ############
    //   ............    ← solid wall row separates the speck below
    //
    // (Rows passed top-down for readability; grid() flips to math-up.)
    let grid = grid(12, &[
        "############",
        "###.....#.##",
        "###.....#.##",
        "############",
        "############",
        "####....####",
        "############",
        "............",
    ]);

    println!("grid is {} × {}, {} walkable cells",
        grid.width,
        grid.height,
        grid.data.iter().filter(|c| **c).count()
    );

    // Default options: 4-connectivity, remove_collinear=true, no smoothing.
    let regions = extract(&grid, &ExtractOptions::default());

    println!("\nextracted {} region(s):", regions.len());
    for (i, r) in regions.iter().enumerate() {
        println!(
            "  region {}: outer={} verts, area={:.1}, {} hole(s)",
            i,
            r.outer.vertices.len(),
            r.outer.area(),
            r.holes.len(),
        );
        for (h_i, hole) in r.holes.iter().enumerate() {
            println!(
                "    hole {}: {} verts, area={:.1}",
                h_i,
                hole.vertices.len(),
                hole.area(),
            );
        }
    }

    // Same input, but with diagonal smoothing turned on. Won't change
    // anything for axis-aligned input like this — try it on a stair-step
    // shape to see the effect.
    let mut opts = ExtractOptions::default();
    opts.diagonal_smoothing = true;
    let smoothed = extract(&grid, &opts);
    println!(
        "\nwith diagonal_smoothing: {} region(s), outer of region 0 has {} verts",
        smoothed.len(),
        smoothed.first().map_or(0, |r| r.outer.vertices.len())
    );
}

/// Parse `rows` of `#` (walkable) / `.` (wall) into a `Bitfield`. Rows are
/// top-down for readability; this flips to math-up (row 0 at the bottom).
fn grid(width: u32, rows: &[&str]) -> Bitfield {
    let height = rows.len() as u32;
    let mut data = vec![false; (width as usize) * (height as usize)];
    for (i, row) in rows.iter().enumerate() {
        let math_row = height as usize - 1 - i;
        for (col, ch) in row.chars().enumerate() {
            let walkable = ch == '#';
            data[math_row * (width as usize) + col] = walkable;
        }
    }
    Bitfield::new(width, height, data).expect("test grid: dimensions match")
}
