//! [`PolySoup`] — the canonical input to the voxel pipeline.
//!
//! Deliberately the dumbest possible mesh representation: a flat vertex pool
//! and a triangle index list. **No** manifold, watertight, winding, normal,
//! or topology guarantees. This is what the voxelizer rasterizes; downstream
//! stages do not see it.
//!
//! All loaders ([`MeshBuilder`](crate::MeshBuilder), an external OBJ loader
//! in the demo, procedural generators) reduce to producing a `PolySoup`.

use rsnav_common::{Aabb3, Vec3};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PolySoup {
    pub vertices: Vec<Vec3>,
    /// Each triangle is three indices into `vertices`. Index ordering is **not**
    /// interpreted — the voxelizer is winding-agnostic.
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PolySoupError {
    /// A triangle indexes a vertex past the end of the vertex pool.
    /// Carries the triangle index and the bad vertex index.
    OutOfRangeIndex { triangle: u32, vertex: u32 },
    /// A triangle is degenerate at the index level (two of its three corners
    /// reference the same vertex slot). Zero-area triangles in *world* space
    /// are NOT rejected here — they're harmless to the voxelizer.
    DegenerateTriangle { triangle: u32 },
}

impl PolySoup {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(vertices: usize, triangles: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertices),
            triangles: Vec::with_capacity(triangles),
        }
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// Cheap structural check: every triangle index is in range and no
    /// triangle is index-degenerate. **Does not** check for self-intersection,
    /// zero area in world space, or anything semantic — voxelization handles
    /// all of that.
    pub fn validate(&self) -> Result<(), PolySoupError> {
        let n = self.vertices.len() as u32;
        for (ti, tri) in self.triangles.iter().enumerate() {
            let ti = ti as u32;
            for &v in tri {
                if v >= n {
                    return Err(PolySoupError::OutOfRangeIndex {
                        triangle: ti,
                        vertex: v,
                    });
                }
            }
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                return Err(PolySoupError::DegenerateTriangle { triangle: ti });
            }
        }
        Ok(())
    }

    /// World-space AABB of all referenced vertices. Empty when the soup is empty.
    pub fn bounds(&self) -> Aabb3 {
        Aabb3::from_points(self.vertices.iter().copied())
    }

    /// Iterator yielding `(v0, v1, v2)` world-space vertex triples.
    /// Panics if [`validate`](Self::validate) would have failed — caller's
    /// responsibility to validate first if the soup may be malformed.
    pub fn triangle_positions(&self) -> impl Iterator<Item = [Vec3; 3]> + '_ {
        self.triangles.iter().map(move |tri| {
            [
                self.vertices[tri[0] as usize],
                self.vertices[tri[1] as usize],
                self.vertices[tri[2] as usize],
            ]
        })
    }

    /// Append `other`'s vertices and triangles into `self`, remapping indices.
    /// Cheaper than going through [`MeshBuilder`] when no transform is needed.
    pub fn append(&mut self, other: &PolySoup) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend_from_slice(&other.vertices);
        self.triangles
            .extend(other.triangles.iter().map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube() -> PolySoup {
        // Unit cube, 8 verts, 12 triangles. Winding intentionally inconsistent
        // — voxelizer doesn't care.
        let v = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(0.0, 1.0, 1.0),
        ];
        let t = vec![
            [0, 1, 2], [0, 2, 3],   // bottom
            [4, 5, 6], [4, 6, 7],   // top
            [0, 1, 5], [0, 5, 4],   // -y
            [3, 2, 6], [3, 6, 7],   // +y
            [0, 3, 7], [0, 7, 4],   // -x
            [1, 2, 6], [1, 6, 5],   // +x
        ];
        PolySoup { vertices: v, triangles: t }
    }

    #[test]
    fn empty_is_valid() {
        let s = PolySoup::new();
        assert!(s.is_empty());
        assert!(s.validate().is_ok());
        assert!(s.bounds().is_empty());
    }

    #[test]
    fn cube_validates_and_iters() {
        let s = cube();
        assert!(s.validate().is_ok());
        assert_eq!(s.vertex_count(), 8);
        assert_eq!(s.triangle_count(), 12);
        assert_eq!(s.bounds().min, Vec3::ZERO);
        assert_eq!(s.bounds().max, Vec3::new(1.0, 1.0, 1.0));
        assert_eq!(s.triangle_positions().count(), 12);
    }

    #[test]
    fn rejects_out_of_range() {
        let mut s = cube();
        s.triangles.push([0, 1, 99]);
        assert_eq!(
            s.validate(),
            Err(PolySoupError::OutOfRangeIndex { triangle: 12, vertex: 99 })
        );
    }

    #[test]
    fn rejects_index_degenerate() {
        let mut s = cube();
        s.triangles.push([3, 3, 0]);
        assert_eq!(
            s.validate(),
            Err(PolySoupError::DegenerateTriangle { triangle: 12 })
        );
    }

    #[test]
    fn append_remaps_indices() {
        let mut a = cube();
        let b = cube();
        let n0 = a.vertex_count();
        a.append(&b);
        assert_eq!(a.vertex_count(), 16);
        assert_eq!(a.triangle_count(), 24);
        // Last triangle should reference the second copy's vertex pool.
        let last = a.triangles.last().unwrap();
        assert!(last.iter().all(|&v| (v as usize) >= n0));
        assert!(a.validate().is_ok());
    }
}
