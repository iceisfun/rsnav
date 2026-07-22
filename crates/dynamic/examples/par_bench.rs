//! Compare `build_navmesh_from_bitfield` wall time across thread counts.
//!
//! ```text
//! cargo run --release -p rsnav-dynamic --example par_bench -- [testdata_dir]
//! ```

use std::path::PathBuf;
use std::time::Instant;

use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_polygon_extract::Bitfield;

fn read_pbm(bytes: &[u8]) -> Option<(u32, u32, Vec<bool>)> {
    let mut p = 0usize;
    let mut tok = || {
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
        let s = p;
        while p < bytes.len() && !bytes[p].is_ascii_whitespace() {
            p += 1;
        }
        String::from_utf8_lossy(&bytes[s..p]).into_owned()
    };
    if tok() != "P4" {
        return None;
    }
    let w: u32 = tok().parse().ok()?;
    let h: u32 = tok().parse().ok()?;
    p += 1;
    let stride = ((w + 7) / 8) as usize;
    let mut cells = Vec::with_capacity((w * h) as usize);
    for row in 0..h as usize {
        let base = p + row * stride;
        for col in 0..w as usize {
            cells.push(bytes[base + (col >> 3)] & (0x80 >> (col & 7)) != 0);
        }
    }
    Some((w, h, cells))
}

fn main() {
    let dir = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("testdata"));
    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("read dir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "pbm"))
        .collect();
    files.sort();

    let counts = [1usize, 2, 4, 8, 16, 32, 0];
    print!("{:<24}", "file");
    for c in counts {
        print!(" {:>8}", if c == 0 { "auto".into() } else { format!("t={c}") });
    }
    println!();

    for f in &files {
        let bytes = std::fs::read(f).expect("read");
        let Some((w, h, cells)) = read_pbm(&bytes) else { continue };
        let bf = Bitfield::new(w, h, cells).expect("bitfield");
        print!("{:<24}", f.file_name().unwrap().to_string_lossy());
        let mut reference: Option<Vec<u8>> = None;
        let mut deterministic = true;
        for c in counts {
            // BuildOptions::threads governs extract too (its 0 = auto
            // inherits), so one knob covers the whole build.
            let opts = BuildOptions { threads: c, ..BuildOptions::default() };
            // Best of 3 to shake scheduler noise out.
            let mut best = f64::MAX;
            let mut bytes = Vec::new();
            for _ in 0..3 {
                let t = Instant::now();
                let b = build_navmesh_from_bitfield(&bf, &opts).expect("build");
                best = best.min(t.elapsed().as_secs_f64() * 1e3);
                bytes = b.navmesh.to_bytes();
            }
            match &reference {
                None => reference = Some(bytes),
                Some(r) => deterministic &= *r == bytes,
            }
            print!(" {:>7.1}m", best);
        }
        println!("  {}", if deterministic { "ok" } else { "DIVERGED" });
    }
}
