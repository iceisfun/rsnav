//! Minimal binary STL loader.
//!
//! Binary STL is the most common output for CAD/3D tools. Layout:
//! - 80 bytes: header (ignored; some tools put text here)
//! - 4 bytes (u32 LE): triangle count
//! - per triangle (50 bytes):
//!   - 12 bytes (3 × f32): normal (we ignore — polygon-soup pipeline is
//!     winding-agnostic and computes normals itself)
//!   - 36 bytes (9 × f32): three vertex positions
//!   - 2 bytes (u16): attribute byte count (almost always 0)
//!
//! STL has no vertex sharing — every triangle carries its own 3 vertices,
//! so the resulting [`PolySoup`] will have `3 * triangle_count` vertices
//! with duplicates. That's fine — the voxel pipeline doesn't care.
//!
//! ASCII STL is NOT supported (rarely seen in practice; we can add
//! detection later if needed).

use rsnav_common::Vec3;
use rsnav_voxel::PolySoup;
use std::io::{self, Read};
use std::path::Path;

pub fn load_binary_stl<P: AsRef<Path>>(path: P) -> io::Result<PolySoup> {
    let mut file = std::fs::File::open(path)?;
    let mut header = [0u8; 80];
    file.read_exact(&mut header)?;
    let mut count_buf = [0u8; 4];
    file.read_exact(&mut count_buf)?;
    let count = u32::from_le_bytes(count_buf) as usize;

    let mut soup = PolySoup::with_capacity(count * 3, count);
    let mut tri_buf = [0u8; 50];
    for _ in 0..count {
        file.read_exact(&mut tri_buf)?;
        // bytes 0..12 = normal (ignored)
        // bytes 12..48 = 3 × (x, y, z) f32
        let mut verts = [Vec3::ZERO; 3];
        for (i, slot) in verts.iter_mut().enumerate() {
            let base = 12 + i * 12;
            let x = f32::from_le_bytes(tri_buf[base..base + 4].try_into().unwrap()) as f64;
            let y = f32::from_le_bytes(tri_buf[base + 4..base + 8].try_into().unwrap()) as f64;
            let z = f32::from_le_bytes(tri_buf[base + 8..base + 12].try_into().unwrap()) as f64;
            *slot = Vec3::new(x, y, z);
        }
        let base_idx = soup.vertices.len() as u32;
        soup.vertices.extend_from_slice(&verts);
        soup.triangles
            .push([base_idx, base_idx + 1, base_idx + 2]);
    }
    Ok(soup)
}
