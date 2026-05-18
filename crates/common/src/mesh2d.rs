//! Indexed triangle mesh in 2D: a flat list of vertices plus a flat list of triangles.
//!
//! This is the unstructured CDT output. Adjacency, BSP, and navmesh enrichment
//! live in downstream crates.

use crate::{Aabb, Triangle, TriangleId, Vertex, VertexId};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh2d {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<Triangle>,
}

impl Mesh2d {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(vertex_capacity: usize, triangle_capacity: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertex_capacity),
            triangles: Vec::with_capacity(triangle_capacity),
        }
    }

    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn push_vertex(&mut self, v: Vertex) -> VertexId {
        let id = VertexId::new(u32::try_from(self.vertices.len()).expect("vertex overflow"));
        self.vertices.push(v);
        id
    }

    pub fn push_triangle(&mut self, t: Triangle) -> TriangleId {
        let id = TriangleId::new(u32::try_from(self.triangles.len()).expect("triangle overflow"));
        self.triangles.push(t);
        id
    }

    pub fn vertex(&self, id: VertexId) -> Vertex {
        self.vertices[id.index()]
    }

    pub fn triangle(&self, id: TriangleId) -> Triangle {
        self.triangles[id.index()]
    }

    pub fn aabb(&self) -> Aabb {
        Aabb::from_points(self.vertices.iter().copied())
    }

    /// Returns the total signed area summed over all triangles. Useful as a
    /// sanity check after construction (should be positive for a CCW navmesh).
    pub fn signed_area(&self) -> f64 {
        self.triangles
            .iter()
            .map(|t| t.signed_area(&self.vertices))
            .sum()
    }

    /// Returns the triangle that contains `p`, if any. Linear scan; intended
    /// for tests and small meshes. Production callers should use the BSP crate.
    pub fn locate(&self, p: Vertex) -> Option<TriangleId> {
        for (i, t) in self.triangles.iter().enumerate() {
            if t.contains(&self.vertices, p) {
                return Some(TriangleId::new(i as u32));
            }
        }
        None
    }

    /// Asserts that every triangle index refers to a real vertex. Returns the
    /// first offending triangle, if any.
    pub fn validate_indices(&self) -> Result<(), MeshIndexError> {
        let n = self.vertices.len() as u32;
        for (i, t) in self.triangles.iter().enumerate() {
            for v in t.v {
                if v.get() >= n {
                    return Err(MeshIndexError {
                        triangle: TriangleId::new(i as u32),
                        bad_vertex: v,
                        vertex_count: n,
                    });
                }
            }
        }
        Ok(())
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MeshIndexError {
    pub triangle: TriangleId,
    pub bad_vertex: VertexId,
    pub vertex_count: u32,
}

impl core::fmt::Display for MeshIndexError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "triangle {} references vertex {} but the mesh only has {} vertices",
            self.triangle.get(),
            self.bad_vertex.get(),
            self.vertex_count,
        )
    }
}

impl std::error::Error for MeshIndexError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    fn unit_square_mesh() -> Mesh2d {
        let mut m = Mesh2d::new();
        let a = m.push_vertex(v(0.0, 0.0));
        let b = m.push_vertex(v(1.0, 0.0));
        let c = m.push_vertex(v(1.0, 1.0));
        let d = m.push_vertex(v(0.0, 1.0));
        m.push_triangle(Triangle::new(a, b, c));
        m.push_triangle(Triangle::new(a, c, d));
        m
    }

    #[test]
    fn push_and_count() {
        let m = unit_square_mesh();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
    }

    #[test]
    fn signed_area_is_total() {
        let m = unit_square_mesh();
        assert_eq!(m.signed_area(), 1.0);
    }

    #[test]
    fn locate_finds_triangle() {
        let m = unit_square_mesh();
        assert_eq!(m.locate(v(0.25, 0.25)), Some(TriangleId::new(0)));
        assert_eq!(m.locate(v(0.25, 0.75)), Some(TriangleId::new(1)));
        assert_eq!(m.locate(v(2.0, 2.0)), None);
    }

    #[test]
    fn validate_catches_out_of_range_index() {
        let mut m = unit_square_mesh();
        m.triangles.push(Triangle::from_indices(0, 1, 999));
        let err = m.validate_indices().unwrap_err();
        assert_eq!(err.bad_vertex, VertexId::new(999));
        assert_eq!(err.triangle.index(), 2);
    }

    #[test]
    fn validate_passes_for_clean_mesh() {
        let m = unit_square_mesh();
        assert!(m.validate_indices().is_ok());
    }
}
