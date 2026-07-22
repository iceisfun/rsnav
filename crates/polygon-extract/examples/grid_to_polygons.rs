//! Trace a small ASCII-art bitfield into polygons + holes and print a
//! summary.
//!
//! Run with:
//!   cargo run -p rsnav-polygon-extract --example grid_to_polygons

use rsnav_polygon_extract::{extract, Bitfield, ExtractOptions};

fn main() {
    // A 12×8 walkable slab with three enclosed holes: a 5×2 room, a 1×2
    // slot to its right, and a 4×1 pocket lower down. The bottom row is
    // solid wall across the full width, so it is boundary, not a hole.
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
    let bits = grid(12, &[
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
        bits.width,
        bits.height,
        bits.data.iter().filter(|c| **c).count()
    );

    // Default options: 4-connectivity, remove_collinear = true, and
    // diagonal_smoothing = true (it defaults to ON; see ExtractOptions).
    let regions = extract(&bits, &ExtractOptions::default());

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

    // Same input with diagonal smoothing turned OFF, which is the only way
    // to see what the default does. On axis-aligned input like this the two
    // agree; smoothing only rewrites stair-step zigzags into diagonals.
    let mut opts = ExtractOptions::default();
    opts.diagonal_smoothing = false;
    let unsmoothed = extract(&bits, &opts);
    println!(
        "\ndiagonal_smoothing = false: {} region(s), outer of region 0 has {} verts",
        unsmoothed.len(),
        unsmoothed.first().map_or(0, |r| r.outer.vertices.len())
    );

    // A staircase, where smoothing does change the trace. Each step is one
    // cell; the default replaces the zigzag with diagonals and drops
    // vertices, at the cost of not being area-preserving (it can bulge up
    // to sqrt(2)/2 ~= 0.708 cells into the wall at a reflex corner).
    let stair = grid(8, &[
        "#.......",
        "##......",
        "###.....",
        "####....",
        "#####...",
        "######..",
        "#######.",
        "########",
    ]);
    let smooth_on = extract(&stair, &ExtractOptions::default());
    let mut off = ExtractOptions::default();
    off.diagonal_smoothing = false;
    let smooth_off = extract(&stair, &off);
    println!(
        "\nstaircase: smoothing on -> {} verts, off -> {} verts",
        smooth_on.first().map_or(0, |r| r.outer.vertices.len()),
        smooth_off.first().map_or(0, |r| r.outer.vertices.len()),
    );

    // Corner-touching cells are 4-connected only, so two cells meeting at a
    // corner are two separate regions, never one.
    let diag = grid(2, &["#.", ".#"]);
    println!(
        "corner-touching pair: {} region(s)",
        extract(&diag, &ExtractOptions::default()).len()
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
