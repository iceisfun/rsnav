//! Cost comparison: grid erosion + legacy build vs contour inset build.
//!
//! Both columns produce an agent-radius-eroded navmesh for the same radius.
//! The grid column pays O(cells) up front and then builds the cheap legacy
//! path; the contour column pays nothing up front and builds the offset /
//! planarize / winding path. Radii are cell-quantized on the grid side, so
//! only integer-ish radii are an apples-to-apples comparison.

use std::time::Instant;

use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_polygon_extract::{Bitfield, ErodeOptions};

fn read_pbm(bytes: &[u8]) -> Option<(u32, u32, Vec<bool>)> {
    let mut it = bytes.splitn(2, |&b| b == b'\n');
    if it.next()? != b"P4" {
        return None;
    }
    let rest = it.next()?;
    let mut hdr = Vec::new();
    let mut idx = 0usize;
    while hdr.len() < 2 && idx < rest.len() {
        while idx < rest.len() && (rest[idx] as char).is_whitespace() {
            idx += 1;
        }
        if idx < rest.len() && rest[idx] == b'#' {
            while idx < rest.len() && rest[idx] != b'\n' {
                idx += 1;
            }
            continue;
        }
        let start = idx;
        while idx < rest.len() && !(rest[idx] as char).is_whitespace() {
            idx += 1;
        }
        hdr.push(std::str::from_utf8(&rest[start..idx]).ok()?.parse::<u32>().ok()?);
    }
    idx += 1;
    let (w, h) = (hdr[0], hdr[1]);
    let stride = ((w + 7) / 8) as usize;
    let mut cells = vec![false; (w as usize) * (h as usize)];
    for row in 0..h as usize {
        for col in 0..w as usize {
            let byte = rest[idx + row * stride + col / 8];
            cells[(h as usize - 1 - row) * w as usize + col] = (byte >> (7 - (col % 8))) & 1 == 1;
        }
    }
    Some((w, h, cells))
}

fn ms(d: std::time::Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn main() {
    let radius: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1.0);

    let mut files: Vec<_> = std::fs::read_dir("testdata")
        .expect("testdata")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "pbm"))
        .collect();
    files.sort();

    println!("radius {radius} cells\n");
    println!(
        "{:<24} {:>7} {:>8} {:>8} {:>8} | {:>9} {:>8} | {:>7}",
        "file", "cells", "erode", "build", "TOTAL", "inset", "tris", "ratio"
    );

    for f in &files {
        let bytes = std::fs::read(f).expect("read");
        let Some((w, h, cells)) = read_pbm(&bytes) else {
            continue;
        };
        let n_cells = (w as usize) * (h as usize);
        let bf = Bitfield::new(w, h, cells).expect("bf");

        // Grid path: erode the bitfield, then the cheap legacy build.
        let eo = ErodeOptions { radius, threads: 0 };
        let t = Instant::now();
        let eroded = bf.eroded(&eo).expect("erode");
        let t_erode = ms(t.elapsed());
        let mut o = BuildOptions::default();
        o.inset = None;
        let t = Instant::now();
        let g = build_navmesh_from_bitfield(&eroded, &o);
        let t_gbuild = ms(t.elapsed());
        let g_tris = g.as_ref().map(|b| b.navmesh.triangle_count()).unwrap_or(0);

        // Contour path: no pre-pass, inset fused into the CDT.
        let mut o2 = BuildOptions::default();
        o2.inset = Some(radius);
        let t = Instant::now();
        let c = build_navmesh_from_bitfield(&bf, &o2);
        let t_inset = ms(t.elapsed());
        let c_tris = c.as_ref().map(|b| b.navmesh.triangle_count()).unwrap_or(0);

        let total = t_erode + t_gbuild;
        println!(
            "{:<24} {:>6}k {:>7.1}m {:>7.1}m {:>7.1}m | {:>8.1}m {:>8} | {:>6.2}x  (grid {} tris)",
            f.file_name().unwrap().to_string_lossy(),
            n_cells / 1000,
            t_erode,
            t_gbuild,
            total,
            t_inset,
            c_tris,
            t_inset / total.max(1e-9),
            g_tris,
        );
    }
}
