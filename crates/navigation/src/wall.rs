//! Wall-aware navmesh helpers shared by A* and the funnel.
//!
//! A "wall edge" is any constrained triangle edge (marker != 0) or any
//! boundary edge (no neighbor). A "wall vertex" is any vertex incident to
//! at least one wall edge.

use rsnav_navmesh::NavMesh;

/// Precomputed `vertex_id → is_wall_vertex` lookup. `O(n)` to build, `O(1)`
/// to query.
#[derive(Debug)]
pub struct WallInfo {
    pub wall_vertex: Vec<bool>,
}

impl WallInfo {
    pub fn from_navmesh(nav: &NavMesh) -> Self {
        let mut wall_vertex = vec![false; nav.vertex_count()];
        for tri in &nav.triangles {
            for i in 0..3 {
                if is_wall_edge_local(tri, i) {
                    // Edge `i` runs between vertices (i+1)%3 and (i+2)%3.
                    let va = tri.vertices[(i + 1) % 3];
                    let vb = tri.vertices[(i + 2) % 3];
                    wall_vertex[va.index()] = true;
                    wall_vertex[vb.index()] = true;
                }
            }
        }
        Self { wall_vertex }
    }

    #[inline]
    pub fn is_wall_vertex(&self, v: rsnav_common::VertexId) -> bool {
        self.wall_vertex[v.index()]
    }
}

/// Is edge `i` of `tri` a wall (constrained or boundary)?
#[inline]
pub fn is_wall_edge_local(tri: &rsnav_navmesh::NavTriangle, i: usize) -> bool {
    !tri.neighbors[i].is_valid() || tri.edge_markers[i] != 0
}
