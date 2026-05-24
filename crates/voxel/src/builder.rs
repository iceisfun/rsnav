//! [`MeshBuilder`] — aggregate multiple [`PolySoup`]s into one, with optional transforms.
//!
//! This is a pure utility. It deliberately **does not** do axis-convention
//! normalization (Y-up ↔ Z-up). The pipeline treats `Vec3.z` as up; callers
//! using Y-up source data should apply a rotation in their [`Transform`]
//! before handing off. Hiding axis flips inside the builder makes downstream
//! coordinate bugs much harder to diagnose.

use crate::PolySoup;
use rsnav_common::Vec3;

/// Affine transform applied to one input mesh during aggregation.
///
/// Stored as a 3×3 linear part (rotation + scale, allowed to be non-uniform)
/// plus a translation. For Phase 1 we don't bother with quaternions or
/// projective transforms — game assets and procedural meshes don't need them.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Transform {
    /// Row-major 3×3. `linear[r][c]` is row `r`, column `c`.
    pub linear: [[f64; 3]; 3],
    pub translation: Vec3,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        linear: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        translation: Vec3::ZERO,
    };

    #[inline]
    pub fn translation(t: Vec3) -> Self {
        let mut x = Self::IDENTITY;
        x.translation = t;
        x
    }

    #[inline]
    pub fn uniform_scale(s: f64) -> Self {
        Self {
            linear: [
                [s, 0.0, 0.0],
                [0.0, s, 0.0],
                [0.0, 0.0, s],
            ],
            translation: Vec3::ZERO,
        }
    }

    /// Rotation around the Z axis (the pipeline's up axis) by `angle` radians.
    pub fn rotation_z(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self {
            linear: [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            translation: Vec3::ZERO,
        }
    }

    /// Rotation around the Y axis by `angle` radians. Use this when reorienting
    /// source data that was authored Y-up.
    pub fn rotation_y(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self {
            linear: [
                [c, 0.0, s],
                [0.0, 1.0, 0.0],
                [-s, 0.0, c],
            ],
            translation: Vec3::ZERO,
        }
    }

    /// Rotation around the X axis by `angle` radians.
    pub fn rotation_x(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self {
            linear: [
                [1.0, 0.0, 0.0],
                [0.0, c, -s],
                [0.0, s, c],
            ],
            translation: Vec3::ZERO,
        }
    }

    /// Apply this transform to a point: `linear * p + translation`.
    #[inline]
    pub fn apply(self, p: Vec3) -> Vec3 {
        let l = self.linear;
        Vec3::new(
            l[0][0] * p.x + l[0][1] * p.y + l[0][2] * p.z + self.translation.x,
            l[1][0] * p.x + l[1][1] * p.y + l[1][2] * p.z + self.translation.y,
            l[2][0] * p.x + l[2][1] * p.y + l[2][2] * p.z + self.translation.z,
        )
    }

    /// Composition: `self.then(other)` produces a transform that applies
    /// `self` first, then `other`. (i.e. `other * self` as a matrix product.)
    pub fn then(self, other: Self) -> Self {
        let a = self.linear;
        let b = other.linear;
        let mut linear = [[0.0; 3]; 3];
        for r in 0..3 {
            for c in 0..3 {
                linear[r][c] = b[r][0] * a[0][c] + b[r][1] * a[1][c] + b[r][2] * a[2][c];
            }
        }
        Self {
            linear,
            translation: other.apply(self.translation),
        }
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Accumulates multiple [`PolySoup`] inputs into one combined soup.
///
/// Cheap construction; allocations happen as inputs are added. Use
/// [`with_capacity`](Self::with_capacity) when the final size is known
/// in advance to avoid intermediate reallocations.
#[derive(Debug, Default)]
pub struct MeshBuilder {
    soup: PolySoup,
}

impl MeshBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(vertices: usize, triangles: usize) -> Self {
        Self {
            soup: PolySoup::with_capacity(vertices, triangles),
        }
    }

    /// Append `mesh` with `xform` applied to every vertex.
    pub fn add(&mut self, mesh: &PolySoup, xform: Transform) {
        let offset = self.soup.vertices.len() as u32;
        self.soup
            .vertices
            .extend(mesh.vertices.iter().copied().map(|p| xform.apply(p)));
        self.soup
            .triangles
            .extend(mesh.triangles.iter().map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]));
    }

    /// Append `mesh` with no transform — equivalent to
    /// [`PolySoup::append`](crate::PolySoup::append) but threaded through
    /// the builder for symmetry.
    pub fn add_identity(&mut self, mesh: &PolySoup) {
        self.add(mesh, Transform::IDENTITY);
    }

    /// Consume the builder and return the aggregated mesh.
    pub fn build(self) -> PolySoup {
        self.soup
    }

    pub fn vertex_count(&self) -> usize {
        self.soup.vertex_count()
    }

    pub fn triangle_count(&self) -> usize {
        self.soup.triangle_count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad() -> PolySoup {
        PolySoup {
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        }
    }

    #[test]
    fn identity_apply_is_noop() {
        let p = Vec3::new(1.5, -2.0, 3.25);
        assert_eq!(Transform::IDENTITY.apply(p), p);
    }

    #[test]
    fn translation_applies() {
        let t = Transform::translation(Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(t.apply(Vec3::ZERO), Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(t.apply(Vec3::new(1.0, 1.0, 1.0)), Vec3::new(11.0, 21.0, 31.0));
    }

    #[test]
    fn rotation_z_quarter_turn_maps_x_to_y() {
        let r = Transform::rotation_z(std::f64::consts::FRAC_PI_2);
        let out = r.apply(Vec3::new(1.0, 0.0, 0.0));
        assert!(out.approx_eq(Vec3::new(0.0, 1.0, 0.0), 1e-12));
        let up = r.apply(Vec3::new(0.0, 0.0, 1.0));
        assert!(up.approx_eq(Vec3::new(0.0, 0.0, 1.0), 1e-12), "up axis unchanged");
    }

    #[test]
    fn rotation_y_flip_y_up_to_z_up() {
        // 90° around X maps Y-up source data into Z-up: +Y becomes +Z.
        let r = Transform::rotation_x(std::f64::consts::FRAC_PI_2);
        let mapped = r.apply(Vec3::new(0.0, 1.0, 0.0));
        assert!(mapped.approx_eq(Vec3::new(0.0, 0.0, 1.0), 1e-12));
    }

    #[test]
    fn compose_rotation_then_translation() {
        let r = Transform::rotation_z(std::f64::consts::FRAC_PI_2);
        let t = Transform::translation(Vec3::new(5.0, 0.0, 0.0));
        let rt = r.then(t);
        // (1,0,0) rotates to (0,1,0), then +(5,0,0) → (5,1,0)
        let out = rt.apply(Vec3::new(1.0, 0.0, 0.0));
        assert!(out.approx_eq(Vec3::new(5.0, 1.0, 0.0), 1e-12));
    }

    #[test]
    fn builder_aggregates_with_offset() {
        let mut b = MeshBuilder::new();
        b.add_identity(&quad());
        b.add(&quad(), Transform::translation(Vec3::new(10.0, 0.0, 0.0)));
        let soup = b.build();
        assert_eq!(soup.vertex_count(), 8);
        assert_eq!(soup.triangle_count(), 4);
        assert!(soup.validate().is_ok());
        // Second quad's first vertex is at (10, 0, 0).
        assert_eq!(soup.vertices[4], Vec3::new(10.0, 0.0, 0.0));
        // Second quad's triangles reference 4..8.
        assert_eq!(soup.triangles[2], [4, 5, 6]);
    }
}
