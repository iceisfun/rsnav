//! Per-stage timing of the `extract → PSLG → CDT → NavMesh → BSP` pipeline
//! over every `testdata/*.pbm` bitfield.
//!
//! ```text
//! cargo run --release -p rsnav-dynamic --example stage_bench -- [testdata_dir]
//! cargo run --release -p rsnav-dynamic --example stage_bench -- --digest [testdata_dir]
//! ```
//!
//! `--digest` runs the real `build_navmesh_from_bitfield` pipeline and
//! prints an FNV-1a hash of each serialized NavMesh — a change gate for
//! refactors that must stay bit-identical.

use std::path::PathBuf;
use std::time::Instant;

use rsnav_bsp::Bsp;
use rsnav_dynamic::BuildOptions;
use rsnav_navmesh::build_from_cdt;
use rsnav_polygon_extract::{extract, Bitfield};
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

    if digest {
        for f in &files {
            let bytes = std::fs::read(f).expect("read pbm");
            let (w, h, cells) = read_pbm(&bytes).expect("parse pbm");
            let bf = Bitfield::new(w, h, cells).expect("bitfield");
            let build =
                rsnav_dynamic::build_navmesh_from_bitfield(&bf, &BuildOptions::default())
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
