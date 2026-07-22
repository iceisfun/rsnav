//! Grid erosion, the clearance field, and the three ways grid erosion will
//! surprise you.
//!
//! Four demonstrations, each printing numbers you can check:
//!
//!   1. QUANTIZATION. `Bitfield::eroded` at a sweep of radii. The output is a
//!      step function of the radius that jumps only at `{0, 1, sqrt2, 2,
//!      sqrt5, ...}`, so a 0.128-cell request — the typical contour inset —
//!      produces the identical grid as a request of 1.0.
//!   2. ONE TRANSFORM, MANY AGENT SIZES. `Bitfield::clearance` computes the
//!      squared-clearance field once; `ClearanceField::threshold` slices it
//!      at several radii. The transform is the expensive part and the
//!      threshold is a linear pass over the field.
//!   3. `threshold(0.0)` KEEPS EVERY CELL, WALLS INCLUDED. The field cannot
//!      tell a wall cell from a wall-adjacent walkable one; both are 0.
//!   4. THE OUTERMOST RING. Everything outside the grid reads as wall, so any
//!      radius > 0 peels the border. Padding the bitfield removes the effect.
//!
//! Run:
//!     cargo run --release -p rsnav-polygon-extract --example erode_and_clearance

use std::time::Instant;

use rsnav_polygon_extract::{Bitfield, ErodeOptions};

/// '#' is wall, '.' is walkable. Rows are given top-down and flipped on
/// load: `Bitfield` row 0 is the BOTTOM row.
const MAP: &[&str] = &[
    "########################",
    "#......................#",
    "#......................#",
    "#.......#######........#",
    "#.......#######........#",
    "#.......#######........#",
    "#......................#",
    "#......................#",
    "#....##................#",
    "#....##................#",
    "#......................#",
    "########################",
];

fn load(rows: &[&str]) -> Bitfield {
    let h = rows.len() as u32;
    let w = rows[0].len() as u32;
    let mut data = vec![false; (w as usize) * (h as usize)];
    for (i, line) in rows.iter().enumerate() {
        let row = (h as usize) - 1 - i;
        for (col, ch) in line.chars().enumerate() {
            data[row * (w as usize) + col] = ch == '.';
        }
    }
    Bitfield::new(w, h, data).expect("rows are all the same length")
}

fn walkable(bf: &Bitfield) -> usize {
    bf.data.iter().filter(|&&b| b).count()
}

/// Render a grid, bottom row last so it reads the way the map literal does.
fn show(bf: &Bitfield) -> String {
    let mut s = String::new();
    for row in (0..bf.height).rev() {
        for col in 0..bf.width {
            s.push(if bf.at(i64::from(col), i64::from(row)) { '.' } else { '#' });
        }
        s.push('\n');
    }
    s
}

/// A grid with `pad` cells of walkable space added on every side. Used only
/// to show that the outermost-ring peel is a boundary condition, not a bug.
fn pad_walkable(bf: &Bitfield, pad: u32) -> Bitfield {
    let (w, h) = (bf.width + 2 * pad, bf.height + 2 * pad);
    let mut out = Bitfield::empty(w, h);
    for r in 0..h {
        for c in 0..w {
            let inside = c >= pad && r >= pad && c < pad + bf.width && r < pad + bf.height;
            let v = if inside {
                bf.at(i64::from(c) - i64::from(pad), i64::from(r) - i64::from(pad))
            } else {
                true
            };
            out.set(c, r, v);
        }
    }
    out
}

fn main() {
    let bf = load(MAP);
    println!("map {}x{}, {} walkable cells", bf.width, bf.height, walkable(&bf));
    println!("{}", show(&bf));

    // ---- 1. quantization ------------------------------------------------
    println!("1. QUANTIZATION -- eroded() as a function of the requested radius");
    println!("   {:>22}  {:>9}", "requested radius", "walkable");
    let radii: &[(f64, &str)] = &[
        (0.0, "0.0"),
        (0.128, "0.128 (typical inset)"),
        (0.5, "0.5"),
        (1.0, "1.0"),
        (1.4, "1.4"),
        (std::f64::consts::SQRT_2, "sqrt2 = 1.41421..."),
        (1.5, "1.5"),
        (2.0, "2.0"),
        (2.3, "2.3"),
        (5.0f64.sqrt(), "sqrt5 = 2.23607..."),
    ];
    let mut prev: Option<usize> = None;
    for &(r, label) in radii {
        let out = bf
            .eroded(&ErodeOptions {
                radius: r,
                threads: 0,
            })
            .expect("finite non-negative radius");
        let n = walkable(&out);
        let step = match prev {
            Some(p) if p == n => "  (same grid as the row above)",
            _ => "",
        };
        prev = Some(n);
        println!("   {label:>22}  {n:>9}{step}");
    }
    println!();
    println!("   Every radius in (0, 1] yields the same one-cell peel: a request of 0.128");
    println!("   gets a guaranteed clearance of 1.0, 7.8x more erosion than asked for. Note");
    println!("   also that sqrt2 itself erodes MORE than 1.4 does: the test is");
    println!("   `sq >= ceil(radius^2)` in exact integers and sqrt2 evaluates in f64 to");
    println!("   2.0000000000000004, so cells at exactly sqrt2 clearance are dropped. Ties");
    println!("   round toward more erosion, deliberately.");
    println!();

    // ---- 2. one transform, three agent sizes ----------------------------
    println!("2. ONE TRANSFORM, THREE AGENT SIZES");
    let t = Instant::now();
    let field = bf.clearance(0);
    let transform_us = t.elapsed().as_secs_f64() * 1e6;
    println!("   clearance() transform: {transform_us:.1} us");
    for r in [1.0f64, 2.0, 3.0] {
        let t = Instant::now();
        let grid = field.threshold(r).expect("finite non-negative radius");
        let thr_us = t.elapsed().as_secs_f64() * 1e6;
        println!(
            "   threshold({r}) -> {:>4} walkable cells, {thr_us:.1} us",
            walkable(&grid)
        );
    }
    println!("   The transform is the expensive part; build the field once and slice it at");
    println!("   every agent size you ship, rather than calling eroded() once per size.");
    println!();
    println!("   The field is also directly readable. Squared clearance along row 6:");
    print!("   ");
    for col in 0..bf.width {
        print!("{}", field.sq_at(col, 6).min(9));
    }
    println!();
    println!("   sq_at is in cells^2 and is exact; sqrt(sq_at) is the largest radius an agent");
    println!("   may have while standing anywhere in that cell. Wall cells and walkable cells");
    println!("   8-adjacent to a wall are both 0.");
    println!();

    // ---- 3. threshold(0.0) keeps the walls ------------------------------
    println!("3. threshold(0.0) KEEPS EVERY CELL, WALLS INCLUDED");
    let all = field.threshold(0.0).expect("radius 0 is valid");
    println!(
        "   original walkable {}, threshold(0.0) walkable {} (= {} total cells)",
        walkable(&bf),
        walkable(&all),
        bf.width * bf.height
    );
    println!("   A clearance of at least zero is vacuously true everywhere, and the field");
    println!("   cannot distinguish a wall cell from a wall-adjacent walkable one -- both are");
    println!("   sq == 0. If you want the original grid, use the original grid, or");
    println!("   eroded() with radius 0, which clones:");
    let cloned = bf
        .eroded(&ErodeOptions {
            radius: 0.0,
            threads: 0,
        })
        .expect("radius 0 is valid");
    println!(
        "   eroded(0.0) walkable {} -- matches the original: {}",
        walkable(&cloned),
        cloned.data == bf.data
    );
    println!();

    // ---- 4. the outermost ring ------------------------------------------
    println!("4. THE OUTERMOST RING");
    // A grid whose walkable area runs right up to the border.
    let open = load(&[
        "........",
        "........",
        "........",
        "........",
        "........",
        "........",
    ]);
    let peeled = open
        .eroded(&ErodeOptions {
            radius: 1.0,
            threads: 0,
        })
        .expect("radius 1 is valid");
    println!(
        "   fully-open {}x{}: {} walkable -> eroded(1.0) -> {} walkable",
        open.width,
        open.height,
        walkable(&open),
        walkable(&peeled)
    );
    println!("{}", show(&peeled));
    println!("   Bitfield::at takes signed coordinates and returns false out of range, so");
    println!("   everything outside the grid is wall and any radius > 0 peels the border.");
    println!("   Correct, but a visible regression for maps whose walkable area runs to the");
    println!("   grid edge. Pad the bitfield and it disappears:");
    let padded = pad_walkable(&open, 1);
    let padded_peeled = padded
        .eroded(&ErodeOptions {
            radius: 1.0,
            threads: 0,
        })
        .expect("radius 1 is valid");
    println!(
        "   padded by 1 -> {}x{}, {} walkable -> eroded(1.0) -> {} walkable",
        padded.width,
        padded.height,
        walkable(&padded),
        walkable(&padded_peeled)
    );
    println!(
        "   the original {}x{} interior survives intact: {}",
        open.width,
        open.height,
        (0..open.height).all(|r| (0..open.width)
            .all(|c| padded_peeled.at(i64::from(c) + 1, i64::from(r) + 1)))
    );
}
