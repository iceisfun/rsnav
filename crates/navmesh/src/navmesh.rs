//! In-memory navmesh data structure.

use rsnav_common::{Aabb, Triangle, TriangleId, Vertex, VertexId};

/// One triangle in the navmesh, packed with its derived metadata.
///
/// `vertices` is in CCW order (positive signed area).
/// `neighbors[i]` is the triangle sharing the edge opposite `vertices[i]`,
/// or [`TriangleId::INVALID`] if that edge is on the mesh boundary.
/// `edge_markers[i]` is the constraint marker on that same edge — `0` means
/// the edge is interior / unconstrained, any non-zero value means it came
/// from a PSLG segment (and the value is the segment's marker).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NavTriangle {
    pub vertices: [VertexId; 3],
    pub neighbors: [TriangleId; 3],
    pub edge_markers: [i32; 3],
    pub area: f64,
    pub centroid: Vertex,
    /// Connected-component ID. Two triangles share a region iff there is a
    /// path between them that never crosses a constrained edge (i.e. they
    /// are both walkable and not separated by a wall).
    pub region: u32,
}

impl NavTriangle {
    /// True if edge `i` (the edge opposite `vertices[i]`) is constrained.
    #[inline]
    pub fn is_edge_constrained(&self, i: usize) -> bool {
        self.edge_markers[i] != 0
    }

    /// True if edge `i` is on the mesh boundary (no neighbor).
    #[inline]
    pub fn is_edge_boundary(&self, i: usize) -> bool {
        !self.neighbors[i].is_valid()
    }

    /// Returns the two endpoints of edge `i` in CCW order around the
    /// triangle: `(vertices[(i+1)%3], vertices[(i+2)%3])`.
    #[inline]
    pub fn edge_vertices(&self, i: usize) -> (VertexId, VertexId) {
        (self.vertices[(i + 1) % 3], self.vertices[(i + 2) % 3])
    }
}

/// A loaded or freshly-built navmesh.
///
/// `vertices` and `triangles` are flat parallel arrays indexed by
/// [`VertexId`] and [`TriangleId`]. The order is the order produced by the
/// CDT builder; serializing and reloading round-trips it exactly.
#[derive(Clone, Debug)]
pub struct NavMesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<NavTriangle>,
    pub aabb: Aabb,
    /// Number of distinct regions; equivalently `1 + max(region)`.
    pub region_count: u32,
}

impl NavMesh {
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Read a vertex position by ID. **Panics** if `id` is out of
    /// range — most commonly because the ID was issued by a different
    /// mesh. NavMesh IDs are not portable across instances; pass IDs
    /// only back to the same NavMesh that produced them.
    #[inline]
    pub fn vertex(&self, id: VertexId) -> Vertex {
        self.vertices[id.index()]
    }

    /// Read a triangle by ID. **Panics** if `id` is out of range. Same
    /// cross-mesh caveat as [`vertex`](Self::vertex).
    #[inline]
    pub fn triangle(&self, id: TriangleId) -> &NavTriangle {
        &self.triangles[id.index()]
    }

    /// `true` if the two triangles are in the same reachability region.
    /// Cheap pre-check before running A*.
    #[inline]
    pub fn reachable(&self, a: TriangleId, b: TriangleId) -> bool {
        self.triangle(a).region == self.triangle(b).region
    }

    /// Convenience: convert a [`NavTriangle`] to the geometry-only
    /// [`Triangle`] from rsnav-common, for use with shared predicates.
    pub fn as_triangle(&self, id: TriangleId) -> Triangle {
        Triangle::new(
            self.triangle(id).vertices[0],
            self.triangle(id).vertices[1],
            self.triangle(id).vertices[2],
        )
    }
}
