//! Synthetic geometry generators used by tests and the debug demo.
//!
//! Everything here returns a [`PolySoup`] in **Z-up** world coordinates: XY is
//! the floor plane, +Z is up. Compose multiple primitives into a scene with
//! [`MeshBuilder`](crate::MeshBuilder).
//!
//! These primitives intentionally do NOT share vertices between adjacent
//! pieces — a ramp landing on a floor has duplicate vertices at the seam.
//! That matches real-world polygon-soup input (assets exported from a DCC
//! tool, two separate meshes placed next to each other) and exercises the
//! voxelizer's "no manifold guarantees" promise.

use crate::PolySoup;
use rsnav_common::Vec3;

/// A flat horizontal plane at `z = 0`, spanning `[-extent_x/2, +extent_x/2]`
/// in X and `[-extent_y/2, +extent_y/2]` in Y. `subdivisions` controls the
/// tessellation (1 = two triangles, 2 = eight, etc.). Useful for testing
/// the voxelizer on triangles of differing sizes.
pub fn plane(extent_x: f64, extent_y: f64, subdivisions: u32) -> PolySoup {
    assert!(extent_x > 0.0 && extent_y > 0.0);
    let n = subdivisions.max(1);
    let (cols, rows) = (n, n);
    let mut soup = PolySoup::with_capacity(
        ((cols + 1) * (rows + 1)) as usize,
        (cols * rows * 2) as usize,
    );
    for j in 0..=rows {
        for i in 0..=cols {
            let u = i as f64 / cols as f64;
            let v = j as f64 / rows as f64;
            let x = -extent_x * 0.5 + extent_x * u;
            let y = -extent_y * 0.5 + extent_y * v;
            soup.vertices.push(Vec3::new(x, y, 0.0));
        }
    }
    let stride = cols + 1;
    for j in 0..rows {
        for i in 0..cols {
            let a = j * stride + i;
            let b = a + 1;
            let c = a + stride;
            let d = c + 1;
            soup.triangles.push([a, b, d]);
            soup.triangles.push([a, d, c]);
        }
    }
    soup
}

/// A ramp surface starting at `x = x_start, z = 0` and rising linearly to
/// `z = rise` at `x = x_start + run`. Spans `y ∈ [-width/2, +width/2]`.
///
/// Six vertices: the start and end edges each get their own pair, plus an
/// underside is **not** generated (this is a single-sided slope, like a
/// thickness-less plane of geometry). The voxelizer ingests both sides
/// either way since it doesn't care about normals.
pub fn ramp(x_start: f64, run: f64, rise: f64, width: f64) -> PolySoup {
    assert!(run > 0.0 && width > 0.0);
    let half = width * 0.5;
    let x_end = x_start + run;
    let v = vec![
        Vec3::new(x_start, -half, 0.0),
        Vec3::new(x_end, -half, rise),
        Vec3::new(x_end, half, rise),
        Vec3::new(x_start, half, 0.0),
    ];
    let t = vec![[0u32, 1, 2], [0, 2, 3]];
    PolySoup {
        vertices: v,
        triangles: t,
    }
}

/// Solid axis-aligned box covering `[min, max]`. Twelve triangles, six faces.
/// Used for walls, buildings, and obstacles in test scenes.
pub fn box_aabb(min: Vec3, max: Vec3) -> PolySoup {
    assert!(max.x > min.x && max.y > min.y && max.z > min.z);
    let v = vec![
        Vec3::new(min.x, min.y, min.z), // 0
        Vec3::new(max.x, min.y, min.z), // 1
        Vec3::new(max.x, max.y, min.z), // 2
        Vec3::new(min.x, max.y, min.z), // 3
        Vec3::new(min.x, min.y, max.z), // 4
        Vec3::new(max.x, min.y, max.z), // 5
        Vec3::new(max.x, max.y, max.z), // 6
        Vec3::new(min.x, max.y, max.z), // 7
    ];
    let t = vec![
        // -Z (bottom)
        [0, 2, 1], [0, 3, 2],
        // +Z (top)
        [4, 5, 6], [4, 6, 7],
        // -Y
        [0, 1, 5], [0, 5, 4],
        // +Y
        [3, 6, 2], [3, 7, 6],
        // -X
        [0, 4, 7], [0, 7, 3],
        // +X
        [1, 2, 6], [1, 6, 5],
    ];
    PolySoup {
        vertices: v,
        triangles: t,
    }
}

/// Common composed scene: a floor with a ramp going up to a raised platform.
/// Returns one aggregated [`PolySoup`].
///
/// Layout (XY plan view), with floor at z=0, platform at z=`rise`:
/// ```text
///     y
///     ↑
///     │   ┌──────────┐   raised platform (z = rise)
///     │   │          │
///     │   │  ramp →  │
///     │ ──┴──────────┴──   floor (z = 0)
///     │
///     └────────────────→ x
/// ```
pub fn floor_with_ramp_and_platform() -> PolySoup {
    use crate::{MeshBuilder, Transform};
    let mut b = MeshBuilder::new();
    // Floor: 12 × 8, centered on origin.
    b.add_identity(&plane(12.0, 8.0, 1));
    // Ramp: starts at x = 2, runs 4 m, rises 1.5 m, 4 m wide.
    b.add_identity(&ramp(2.0, 4.0, 1.5, 4.0));
    // Platform: 4 × 4 × 0.1 box sitting at z = 1.5 (the top of the ramp).
    b.add(
        &box_aabb(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(4.0, 4.0, 0.1),
        ),
        Transform::translation(Vec3::new(6.0, -2.0, 1.5)),
    );
    b.build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_has_expected_size_and_winds_validly() {
        let p = plane(10.0, 6.0, 1);
        assert_eq!(p.vertex_count(), 4);
        assert_eq!(p.triangle_count(), 2);
        assert!(p.validate().is_ok());
        let b = p.bounds();
        assert_eq!(b.min, Vec3::new(-5.0, -3.0, 0.0));
        assert_eq!(b.max, Vec3::new(5.0, 3.0, 0.0));
    }

    #[test]
    fn plane_subdivisions() {
        let p = plane(4.0, 4.0, 4);
        assert_eq!(p.vertex_count(), 25); // (4+1)^2
        assert_eq!(p.triangle_count(), 32); // 4*4*2
        assert!(p.validate().is_ok());
    }

    #[test]
    fn ramp_spans_expected_range() {
        let r = ramp(1.0, 3.0, 2.0, 4.0);
        assert_eq!(r.vertex_count(), 4);
        assert_eq!(r.triangle_count(), 2);
        assert!(r.validate().is_ok());
        let b = r.bounds();
        assert_eq!(b.min, Vec3::new(1.0, -2.0, 0.0));
        assert_eq!(b.max, Vec3::new(4.0, 2.0, 2.0));
    }

    #[test]
    fn box_has_six_faces() {
        let b = box_aabb(Vec3::ZERO, Vec3::new(2.0, 3.0, 4.0));
        assert_eq!(b.vertex_count(), 8);
        assert_eq!(b.triangle_count(), 12);
        assert!(b.validate().is_ok());
        assert_eq!(b.bounds().min, Vec3::ZERO);
        assert_eq!(b.bounds().max, Vec3::new(2.0, 3.0, 4.0));
    }

    #[test]
    fn composed_scene_validates() {
        let s = floor_with_ramp_and_platform();
        assert!(s.validate().is_ok());
        let b = s.bounds();
        // Floor covers [-6, 6] × [-4, 4]; ramp extends to x = 6; platform to (10, 2, 1.6).
        assert!(b.min.x <= -6.0);
        assert!(b.max.x >= 10.0);
        assert!((b.max.z - 1.6).abs() < 1e-9);
    }
}
