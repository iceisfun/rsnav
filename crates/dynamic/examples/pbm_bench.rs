//! Load every `testdata/*.pbm` bitfield and build a navmesh from it.
//!
//! ```text
//! cargo run --release -p rsnav-dynamic --example pbm_bench -- [testdata_dir]
//! ```

use std::path::PathBuf;
use std::time::Instant;

use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_polygon_extract::Bitfield;

/// Parse a binary PBM (`P4`). Returns (width, height, row-major cells) where
/// `true` = passable. See `testdata/TEST_DATA_FORMAT.md`.
fn read_pbm(bytes: &[u8]) -> Result<(u32, u32, Vec<bool>), String> {
    let mut p = 0usize;
    // Header tokens are whitespace-separated; `#` runs to end of line.
    let mut tok = || -> Result<String, String> {
        loop {
            while p < bytes.len() && bytes[p].is_ascii_whitespace() {
                p += 1;
            }
            if p < bytes.len() && bytes[p] == b'#' {
                while p < bytes.len() && bytes[p] != b'\n' {
                    p += 1;
                }
                continue;
            }
            break;
        }
        let start = p;
        while p < bytes.len() && !bytes[p].is_ascii_whitespace() {
            p += 1;
        }
        if start == p {
            return Err("unexpected end of header".into());
        }
        Ok(String::from_utf8_lossy(&bytes[start..p]).into_owned())
    };

    if tok()? != "P4" {
        return Err("not a binary PBM (expected magic P4)".into());
    }
    let w: u32 = tok()?.parse().map_err(|_| "bad width")?;
    let h: u32 = tok()?.parse().map_err(|_| "bad height")?;
    // Exactly one whitespace byte separates the header from the raster.
    p += 1;

    let stride = ((w + 7) / 8) as usize;
    let need = stride * h as usize;
    if bytes.len() - p < need {
        return Err(format!("raster short: {} < {need}", bytes.len() - p));
    }
    let mut cells = Vec::with_capacity((w * h) as usize);
    for row in 0..h as usize {
        let base = p + row * stride;
        for col in 0..w as usize {
            cells.push(bytes[base + (col >> 3)] & (0x80 >> (col & 7)) != 0);
        }
    }
    Ok((w, h, cells))
}

fn main() {
    let dir = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("testdata"));

    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "pbm"))
        .collect();
    files.sort();

    println!(
        "{:<24} {:>11} {:>9} {:>8} {:>9} {:>10}",
        "file", "size", "tris", "regions", "build", "cells/ms"
    );
    let mut failed = 0;
    for f in &files {
        let bytes = std::fs::read(f).expect("read pbm");
        let (w, h, cells) = match read_pbm(&bytes) {
            Ok(v) => v,
            Err(e) => {
                println!("{:<24} PARSE FAILED: {e}", name(f));
                failed += 1;
                continue;
            }
        };
        let bf = Bitfield::new(w, h, cells).expect("bitfield");
        let t = Instant::now();
        match build_navmesh_from_bitfield(&bf, &BuildOptions::default()) {
            Ok(b) => {
                let ms = t.elapsed().as_secs_f64() * 1e3;
                println!(
                    "{:<24} {:>11} {:>9} {:>8} {:>7.1}ms {:>10.0}",
                    name(f),
                    format!("{w}x{h}"),
                    b.navmesh.triangle_count(),
                    b.navmesh.region_count,
                    ms,
                    (w as f64 * h as f64) / ms.max(1e-9),
                );
            }
            Err(e) => {
                println!("{:<24} {:>11} BUILD FAILED: {e}", name(f), format!("{w}x{h}"));
                failed += 1;
            }
        }
    }
    if failed > 0 {
        eprintln!("\n{failed} file(s) failed");
        std::process::exit(1);
    }
    println!("\nall {} inputs built", files.len());
}

fn name(p: &std::path::Path) -> String {
    p.file_name().unwrap_or_default().to_string_lossy().into_owned()
}
