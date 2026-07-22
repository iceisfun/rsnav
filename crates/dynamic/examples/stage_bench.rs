//! Per-stage timing of the `extract → PSLG → CDT → NavMesh → BSP` pipeline
//! over every `testdata/*.pbm` bitfield.
//!
//! ```text
//! cargo run --release -p rsnav-dynamic --example stage_bench -- [testdata_dir]
//! cargo run --release -p rsnav-dynamic --example stage_bench -- --digest [testdata_dir]
//! cargo run --release -p rsnav-dynamic --example stage_bench -- --erode 2.0 [testdata_dir]
//! ```
//!
//! `--digest` runs the real `build_navmesh_from_bitfield` pipeline and
//! prints an FNV-1a hash of each serialized NavMesh — a change gate for
//! refactors that must stay bit-identical.
//!
//! `--erode <r>` times `Bitfield::eroded` (grid erosion, `O(cells)`)
//! against the rest of the build, which is `O(boundary)`. Putting the two
//! in one table is the point: on a large mostly-open map erosion dominates
//! a build whose output is a handful of triangles, which is exactly why it
//! is opt-in at the call site and never a default.

use std::path::PathBuf;
use std::time::Instant;

use rsnav_bsp::Bsp;
use rsnav_dynamic::BuildOptions;
use rsnav_navmesh::build_from_cdt;
use rsnav_polygon_extract::{extract, Bitfield, ErodeOptions};
use rsnav_triangle::{
    carve_holes, clip_ears, delaunay, form_skeleton, CdtMesh, DivConqOptions, Pslg, PslgHole,
    PslgSegment, PslgVertex, VertexSlot,
};

fn read_pbm(bytes: &[u8]) -> Result<(u32, u32, Vec<bool>), String> {
    let mut p = 0usize;
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
        return Err("not a binary PBM".into());
    }
    let w: u32 = tok()?.parse().map_err(|_| "bad width")?;
    let h: u32 = tok()?.parse().map_err(|_| "bad height")?;
    p += 1;

    let stride = ((w + 7) / 8) as usize;
    let mut cells = Vec::with_capacity((w * h) as usize);
    for row in 0..h as usize {
        let base = p + row * stride;
        for col in 0..w as usize {
            cells.push(bytes[base + (col >> 3)] & (0x80 >> (col & 7)) != 0);
        }
    }
    Ok((w, h, cells))
}

/// FNV-1a 64-bit, enough to pin bytes without a hash dependency.
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let digest = args.iter().any(|a| a == "--digest");
    args.retain(|a| a != "--digest");
    // `--inset <r>` routes builds through the offset/planarize/winding
    // path (BuildOptions::inset = Some(r)); without it the legacy path
    // runs, keeping the recorded default digests comparable forever.
    let mut inset: Option<f64> = None;
    if let Some(i) = args.iter().position(|a| a == "--inset") {
        let r = args
            .get(i + 1)
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|r| r.is_finite() && *r >= 0.0)
            .expect("--inset requires a finite radius >= 0");
        inset = Some(r);
        args.drain(i..=i + 1);
    }
    // `--erode <r>` times the grid erosion (rsnav_polygon_extract) that
    // bakes an agent radius into the bitfield before extraction.
    let mut erode: Option<f64> = None;
    if let Some(i) = args.iter().position(|a| a == "--erode") {
        let r = args
            .get(i + 1)
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|r| r.is_finite() && *r >= 0.0)
            .expect("--erode requires a finite radius >= 0");
        erode = Some(r);
        args.drain(i..=i + 1);
    }
    let dir = args
        .first()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("testdata"));

    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("read dir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "pbm"))
        .collect();
    files.sort();

    let base_opts = || {
        let mut o = BuildOptions::default();
        o.inset = inset;
        o
    };

    if let Some(r) = erode {
        // Erosion is O(cells) whatever the boundary looks like, so the
        // interesting column is `cells` next to `erode` — not `build`.
        println!(
            "{:<24} {:>10} {:>9} {:>9} {:>9}",
            "file", "cells", "erode", "extract", "build"
        );
        for f in &files {
            let bytes = std::fs::read(f).expect("read pbm");
            let (w, h, cells) = read_pbm(&bytes).expect("parse pbm");
            let bf = Bitfield::new(w, h, cells).expect("bitfield");
            let opts = base_opts();

            // Warm the page cache / allocator so the first file is not
            // penalized relative to the rest.
            let _ = bf
                .eroded(&ErodeOptions { radius: r, threads: 0 })
                .expect("erode");
            let t = Instant::now();
            let er = bf
                .eroded(&ErodeOptions { radius: r, threads: 0 })
                .expect("erode");
            let t_erode = t.elapsed();

            let t = Instant::now();
            let regions = extract(&er, &opts.extract);
            let t_extract = t.elapsed();
            let _ = regions;

            let t = Instant::now();
            let tris = match rsnav_dynamic::build_navmesh_from_bitfield(&er, &opts) {
                Ok(b) => b.navmesh.triangle_count(),
                // Full erosion legitimately leaves nothing to triangulate.
                Err(_) => 0,
            };
            let t_build = t.elapsed();

            let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;
            println!(
                "{:<24} {:>10} {:>8.1}m {:>8.1}m {:>8.1}m   ({} tris)",
                f.file_name().unwrap().to_string_lossy(),
                (w as usize) * (h as usize),
                ms(t_erode),
                ms(t_extract),
                ms(t_build),
                tris,
            );
        }
        return;
    }

    if digest {
        for f in &files {
            let bytes = std::fs::read(f).expect("read pbm");
            let (w, h, cells) = read_pbm(&bytes).expect("parse pbm");
            let bf = Bitfield::new(w, h, cells).expect("bitfield");
            let build = rsnav_dynamic::build_navmesh_from_bitfield(&bf, &base_opts())
                .expect("build");
            let ser = build.navmesh.to_bytes();
            println!(
                "{:<24} {:>9} tris {:>16x}",
                f.file_name().unwrap().to_string_lossy(),
                build.navmesh.triangle_count(),
                fnv1a(&ser),
            );
        }
        return;
    }

    // With --inset the CDT stages are fused inside build_cdt_with_inset,
    // so the timing table collapses to whole-pipeline phases.
    if inset.is_some() {
        println!(
            "{:<24} {:>9} {:>9}",
            "file", "extract", "build"
        );
        for f in &files {
            let bytes = std::fs::read(f).expect("read pbm");
            let (w, h, cells) = read_pbm(&bytes).expect("parse pbm");
            let bf = Bitfield::new(w, h, cells).expect("bitfield");
            let opts = base_opts();
            let t = Instant::now();
            let regions = extract(&bf, &opts.extract);
            let t_extract = t.elapsed();
            let _ = regions;
            let t = Instant::now();
            let build =
                rsnav_dynamic::build_navmesh_from_bitfield(&bf, &opts).expect("build");
            let t_build = t.elapsed();
            let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;
            println!(
                "{:<24} {:>8.1}m {:>8.1}m   ({} tris)",
                f.file_name().unwrap().to_string_lossy(),
                ms(t_extract),
                ms(t_build),
                build.navmesh.triangle_count(),
            );
        }
        return;
    }

    println!(
        "{:<24} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}",
        "file", "extract", "pslg", "delaunay", "skeleton", "carve", "clip", "navmesh", "bsp"
    );

    for f in &files {
        let bytes = std::fs::read(f).expect("read pbm");
        let (w, h, cells) = read_pbm(&bytes).expect("parse pbm");
        let bf = Bitfield::new(w, h, cells).expect("bitfield");
        let opts = BuildOptions::default();

        let t = Instant::now();
        let regions = extract(&bf, &opts.extract);
        let t_extract = t.elapsed();

        let t = Instant::now();
        let mut pslg = Pslg::new();
        let mut next_idx: u32 = 0;
        for region in &regions {
            let start_idx = next_idx;
            for v in &region.outer.vertices {
                pslg.vertices.push(PslgVertex::new(*v));
                next_idx += 1;
            }
            let n = region.outer.vertices.len() as u32;
            if n >= 3 {
                for i in 0..n {
                    pslg.segments.push(PslgSegment {
                        a: start_idx + i,
                        b: start_idx + (i + 1) % n,
                        marker: opts.perimeter_marker,
                    });
                }
            }
            for hole in &region.holes {
                let start_idx = next_idx;
                for v in &hole.vertices {
                    pslg.vertices.push(PslgVertex::new(*v));
                    next_idx += 1;
                }
                let n = hole.vertices.len() as u32;
                if n >= 3 {
                    for i in 0..n {
                        pslg.segments.push(PslgSegment {
                            a: start_idx + i,
                            b: start_idx + (i + 1) % n,
                            marker: opts.hole_marker,
                        });
                    }
                }
                if let Some(seed) = hole.interior_point() {
                    pslg.holes.push(PslgHole { point: seed });
                }
            }
        }
        let t_pslg = t.elapsed();

        let t = Instant::now();
        let mut cdt = CdtMesh::new();
        for v in &pslg.vertices {
            cdt.push_vertex(VertexSlot::new(v.position, 0));
        }
        delaunay(&mut cdt, DivConqOptions::default());
        let t_delaunay = t.elapsed();

        let t = Instant::now();
        form_skeleton(&mut cdt, &pslg, None).expect("skeleton");
        let t_skel = t.elapsed();

        let t = Instant::now();
        carve_holes(&mut cdt, &pslg, false);
        let t_carve = t.elapsed();

        let t = Instant::now();
        if opts.clip_ears_max_area > 0.0 {
            clip_ears(&mut cdt, opts.clip_ears_max_area);
        }
        let t_clip = t.elapsed();

        let t = Instant::now();
        let navmesh = build_from_cdt(&cdt);
        let t_nav = t.elapsed();

        let t = Instant::now();
        let _bsp = Bsp::build(&navmesh);
        let t_bsp = t.elapsed();

        let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;
        println!(
            "{:<24} {:>8.1}m {:>8.1}m {:>8.1}m {:>8.1}m {:>8.1}m {:>8.1}m {:>8.1}m {:>8.1}m",
            f.file_name().unwrap().to_string_lossy(),
            ms(t_extract),
            ms(t_pslg),
            ms(t_delaunay),
            ms(t_skel),
            ms(t_carve),
            ms(t_clip),
            ms(t_nav),
            ms(t_bsp),
        );
    }
}
