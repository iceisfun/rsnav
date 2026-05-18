//! Plain 2D triangle (three vertex indices) plus per-triangle geometry helpers.

use crate::{Aabb, Vertex, VertexId, geom};

/// A triangle stored as three vertex indices. Vertex order should be CCW for
/// consumers that care about orientation (navmesh, BSP).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Triangle {
    pub v: [VertexId; 3],
}

impl Triangle {
    #[inline]
    pub const fn new(a: VertexId, b: VertexId, c: VertexId) -> Self {
        Self { v: [a, b, c] }
    }

    #[inline]
    pub fn from_indices(a: u32, b: u32, c: u32) -> Self {
        Self::new(VertexId::new(a), VertexId::new(b), VertexId::new(c))
    }

    /// Returns the triangle's three vertices in the order `[v0, v1, v2]`.
    /// Panics if any index is out of bounds for `vertices`.
    pub fn positions(&self, vertices: &[Vertex]) -> [Vertex; 3] {
        [
            vertices[self.v[0].index()],
            vertices[self.v[1].index()],
            vertices[self.v[2].index()],
        ]
    }

    pub fn signed_area2(&self, vertices: &[Vertex]) -> f64 {
        let [a, b, c] = self.positions(vertices);
        geom::signed_area2(a, b, c)
    }

    #[inline]
    pub fn signed_area(&self, vertices: &[Vertex]) -> f64 {
        0.5 * self.signed_area2(vertices)
    }

    #[inline]
    pub fn area(&self, vertices: &[Vertex]) -> f64 {
        self.signed_area(vertices).abs()
    }

    pub fn centroid(&self, vertices: &[Vertex]) -> Vertex {
        let [a, b, c] = self.positions(vertices);
        Vertex::new((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0)
    }

    pub fn aabb(&self, vertices: &[Vertex]) -> Aabb {
        Aabb::from_points(self.positions(vertices))
    }

    /// True when the listed vertex order is counter-clockwise.
    #[inline]
    pub fn is_ccw(&self, vertices: &[Vertex]) -> bool {
        self.signed_area2(vertices) > 0.0
    }

    /// Returns true if `p` lies inside this triangle (boundary inclusive).
    pub fn contains(&self, vertices: &[Vertex], p: Vertex) -> bool {
        let [a, b, c] = self.positions(vertices);
        let d1 = geom::orient2d(a, b, p);
        let d2 = geom::orient2d(b, c, p);
        let d3 = geom::orient2d(c, a, p);
        let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
        let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
        !(has_neg && has_pos)
    }

    /// Barycentric coordinates of `p` with respect to the triangle's vertices.
    /// Returns `None` if the triangle is degenerate (zero area).
    pub fn barycentric(&self, vertices: &[Vertex], p: Vertex) -> Option<[f64; 3]> {
        let [a, b, c] = self.positions(vertices);
        let denom = geom::signed_area2(a, b, c);
        if denom == 0.0 {
            return None;
        }
        let inv = 1.0 / denom;
        let w_a = geom::signed_area2(b, c, p) * inv;
        let w_b = geom::signed_area2(c, a, p) * inv;
        let w_c = geom::signed_area2(a, b, p) * inv;
        Some([w_a, w_b, w_c])
    }

    /// Reverses winding by swapping the last two vertices.
    pub fn flipped(self) -> Self {
        Self::new(self.v[0], self.v[2], self.v[1])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    fn unit_tri_ccw() -> (Vec<Vertex>, Triangle) {
        let verts = vec![v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0)];
        (verts, Triangle::from_indices(0, 1, 2))
    }

    #[test]
    fn area_and_orientation() {
        let (verts, t) = unit_tri_ccw();
        assert_eq!(t.signed_area(&verts), 0.5);
        assert_eq!(t.area(&verts), 0.5);
        assert!(t.is_ccw(&verts));
        let flipped = t.flipped();
        assert_eq!(flipped.signed_area(&verts), -0.5);
        assert!(!flipped.is_ccw(&verts));
    }

    #[test]
    fn centroid_and_contains() {
        let (verts, t) = unit_tri_ccw();
        assert_eq!(t.centroid(&verts), v(1.0 / 3.0, 1.0 / 3.0));
        assert!(t.contains(&verts, v(0.25, 0.25)));
        assert!(t.contains(&verts, v(0.5, 0.5))); // on the hypotenuse
        assert!(!t.contains(&verts, v(0.8, 0.8)));
    }

    #[test]
    fn barycentric_centroid_is_one_third() {
        let (verts, t) = unit_tri_ccw();
        let b = t.barycentric(&verts, t.centroid(&verts)).unwrap();
        for w in b {
            assert!((w - 1.0 / 3.0).abs() < 1e-12);
        }
    }

    #[test]
    fn barycentric_degenerate_returns_none() {
        let verts = vec![v(0.0, 0.0), v(1.0, 1.0), v(2.0, 2.0)];
        let t = Triangle::from_indices(0, 1, 2);
        assert!(t.barycentric(&verts, v(0.5, 0.5)).is_none());
    }
}
