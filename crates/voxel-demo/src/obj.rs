//! Minimal Wavefront OBJ loader.
//!
//! Supports the subset that Recast's demo meshes use:
//! - `v x y z` — vertex position (extra coords ignored)
//! - `f a b c [d ...]` — face with 3+ vertex indices, triangulated as a fan
//! - Per-vertex tokens may include texture / normal indices
//!   (`v/vt/vn`, `v//vn`, `v/vt`); we keep only the position index.
//! - 1-based indices; negative indices count from end (`-1` = last vertex).
//! - Lines starting with `#`, `mtllib`, `usemtl`, `o`, `g`, `s`, `vn`, `vt`
//!   etc. are silently ignored.
//!
//! Output is a winding-agnostic [`PolySoup`] (the voxel pipeline doesn't care).

use rsnav_common::Vec3;
use rsnav_voxel::PolySoup;
use std::io::{self, BufRead, BufReader};
use std::path::Path;

pub fn load_obj<P: AsRef<Path>>(path: P) -> io::Result<PolySoup> {
    let file = std::fs::File::open(path)?;
    let reader = BufReader::new(file);
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();

    for (lineno, line) in reader.lines().enumerate() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut tokens = trimmed.split_ascii_whitespace();
        let head = match tokens.next() {
            Some(h) => h,
            None => continue,
        };
        match head {
            "v" => {
                let x = parse_f64(tokens.next(), lineno)?;
                let y = parse_f64(tokens.next(), lineno)?;
                let z = parse_f64(tokens.next(), lineno)?;
                vertices.push(Vec3::new(x, y, z));
            }
            "f" => {
                // Collect resolved vertex indices for this face.
                let mut face: Vec<u32> = Vec::with_capacity(4);
                for tok in tokens {
                    let v_part = tok.split('/').next().unwrap_or(tok);
                    if v_part.is_empty() {
                        continue;
                    }
                    let idx_raw: i64 = v_part.parse().map_err(|e| {
                        io::Error::new(
                            io::ErrorKind::InvalidData,
                            format!("line {}: bad index '{}': {}", lineno + 1, v_part, e),
                        )
                    })?;
                    let idx = if idx_raw > 0 {
                        (idx_raw - 1) as i64
                    } else {
                        vertices.len() as i64 + idx_raw
                    };
                    if idx < 0 || idx >= vertices.len() as i64 {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            format!("line {}: vertex index out of range", lineno + 1),
                        ));
                    }
                    face.push(idx as u32);
                }
                if face.len() < 3 {
                    continue;
                }
                // Fan triangulation: (0, i, i+1).
                for i in 1..face.len() - 1 {
                    triangles.push([face[0], face[i], face[i + 1]]);
                }
            }
            _ => {
                // Silently skip vt, vn, mtllib, usemtl, o, g, s, ...
            }
        }
    }

    Ok(PolySoup {
        vertices,
        triangles,
    })
}

fn parse_f64(token: Option<&str>, lineno: usize) -> io::Result<f64> {
    let s = token.ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("line {}: missing float", lineno + 1),
        )
    })?;
    s.parse::<f64>().map_err(|e| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("line {}: bad float '{}': {}", lineno + 1, s, e),
        )
    })
}
