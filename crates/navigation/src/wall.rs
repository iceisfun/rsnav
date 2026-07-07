//! Wall-aware navmesh helpers shared by A* and the funnel.
//!
//! A "wall edge" is any constrained triangle edge (marker != 0) or any
//! boundary edge (no neighbor). A "wall vertex" is any vertex incident to
//! at least one wall edge.
//!
//! In multi-level worlds, edges carrying a **connection marker**
//! ([`rsnav_navmesh::is_connection_marker`]) are seams in continuous
//! walkable floor, not walls. [`WallInfo::from_navmesh_permeable`] opts
//! into treating them as such: seam vertices stop counting as wall
//! vertices (so clearance never shrinks a seam portal) and a two-sided
//! seam edge is traversable. A *boundary* seam edge — the neighbor
//! triangle lives in another layer's mesh — still blocks single-mesh
//! traversal; crossing it is the multi-mesh router's job.

use rsnav_navmesh::{is_connection_marker, NavMesh};

/// Precomputed `vertex_id → is_wall_vertex` lookup plus the wall
/// semantics chosen at construction. `O(n)` to build, `O(1)` to query.
#[derive(Debug)]
pub struct WallInfo {
    pub wall_vertex: Vec<bool>,
    /// When `true`, connection-marked edges are seams, not walls.
    connections_permeable: bool,
}

impl WallInfo {
    /// Classic 2D semantics: every constrained or boundary edge is a
    /// wall, whatever its marker.
    pub fn from_navmesh(nav: &NavMesh) -> Self {
        Self::build(nav, false)
    }

    /// Multi-level semantics: connection-marked edges are transparent
    /// seams. With no connection markers in the mesh this is identical
    /// to [`from_navmesh`](Self::from_navmesh).
    pub fn from_navmesh_permeable(nav: &NavMesh) -> Self {
        Self::build(nav, true)
    }

    fn build(nav: &NavMesh, connections_permeable: bool) -> Self {
        let mut wall_vertex = vec![false; nav.vertex_count()];
        for tri in &nav.triangles {
            for i in 0..3 {
                // For *clearance* purposes a connection edge is never a
                // wall — even a boundary one: the walkable surface
                // continues on the neighboring layer, so the funnel must
                // not pull the path off seam vertices.
                if connections_permeable && is_connection_marker(tri.edge_markers[i]) {
                    continue;
                }
                if is_wall_edge_local(tri, i) {
                    // Edge `i` runs between vertices (i+1)%3 and (i+2)%3.
                    let va = tri.vertices[(i + 1) % 3];
                    let vb = tri.vertices[(i + 2) % 3];
                    wall_vertex[va.index()] = true;
                    wall_vertex[vb.index()] = true;
                }
            }
        }
        Self {
            wall_vertex,
            connections_permeable,
        }
    }

    #[inline]
    pub fn is_wall_vertex(&self, v: rsnav_common::VertexId) -> bool {
        self.wall_vertex[v.index()]
    }

    /// Is edge `i` of `tri` a wall for *traversal within this mesh*?
    ///
    /// A boundary edge (no neighbor) always is — even a permeable seam:
    /// its far side lives in another mesh, and only a multi-mesh router
    /// holding the connection table can cross it. A two-sided edge is a
    /// wall when constrained, unless it carries a connection marker and
    /// this `WallInfo` was built permeable.
    #[inline]
    pub fn is_wall_edge(&self, tri: &rsnav_navmesh::NavTriangle, i: usize) -> bool {
        if !tri.neighbors[i].is_valid() {
            return true;
        }
        let marker = tri.edge_markers[i];
        marker != 0 && !(self.connections_permeable && is_connection_marker(marker))
    }
}

/// Is edge `i` of `tri` a wall (constrained or boundary)?
///
/// Classic 2D predicate: any non-zero marker is a wall. Multi-level
/// callers should prefer [`WallInfo::is_wall_edge`], which respects
/// permeable connection markers.
#[inline]
pub fn is_wall_edge_local(tri: &rsnav_navmesh::NavTriangle, i: usize) -> bool {
    !tri.neighbors[i].is_valid() || tri.edge_markers[i] != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_common::Vertex;
    use rsnav_navmesh::{build_from_cdt, connection_marker};
    use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
    use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

    /// A 10×4 rectangle with a floating interior segment (4,2)–(6,2)
    /// carrying a connection marker, and the outer bottom edge (0,0)–(10,0)
    /// carrying another. Interior seam endpoints touch no real wall.
    fn seam_mesh() -> rsnav_navmesh::NavMesh {
        let pts = [
            (0.0, 0.0),  // 0
            (10.0, 0.0), // 1
            (10.0, 4.0), // 2
            (0.0, 4.0),  // 3
            (4.0, 2.0),  // 4  interior seam
            (6.0, 2.0),  // 5
        ];
        let mut mesh = CdtMesh::new();
        for (x, y) in pts {
            mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let pslg = Pslg {
            vertices: pts
                .iter()
                .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
                .collect(),
            segments: vec![
                PslgSegment { a: 0, b: 1, marker: connection_marker(7) }, // boundary seam
                PslgSegment { a: 1, b: 2, marker: 1 },
                PslgSegment { a: 2, b: 3, marker: 1 },
                PslgSegment { a: 3, b: 0, marker: 1 },
                PslgSegment { a: 4, b: 5, marker: connection_marker(3) }, // interior seam
            ],
            holes: Vec::new(),
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        build_from_cdt(&mesh)
    }

    fn vertex_id_at(nav: &rsnav_navmesh::NavMesh, p: Vertex) -> rsnav_common::VertexId {
        let i = nav
            .vertices
            .iter()
            .position(|&v| v == p)
            .expect("vertex present");
        rsnav_common::VertexId::new(i as u32)
    }

    #[test]
    fn classic_semantics_treat_connection_edges_as_walls() {
        let nav = seam_mesh();
        let walls = WallInfo::from_navmesh(&nav);
        let a = vertex_id_at(&nav, Vertex::new(4.0, 2.0));
        let b = vertex_id_at(&nav, Vertex::new(6.0, 2.0));
        assert!(walls.is_wall_vertex(a));
        assert!(walls.is_wall_vertex(b));
        // The interior seam edge blocks traversal.
        let seam_marker = connection_marker(3);
        let mut seen = false;
        for tri in &nav.triangles {
            for i in 0..3 {
                if tri.edge_markers[i] == seam_marker {
                    assert!(walls.is_wall_edge(tri, i));
                    seen = true;
                }
            }
        }
        assert!(seen, "interior seam edge missing from the mesh");
    }

    #[test]
    fn permeable_semantics_open_connection_edges() {
        let nav = seam_mesh();
        let walls = WallInfo::from_navmesh_permeable(&nav);
        // Interior seam endpoints touch only seam edges — not wall
        // vertices any more, so clearance never shrinks a seam portal.
        let a = vertex_id_at(&nav, Vertex::new(4.0, 2.0));
        let b = vertex_id_at(&nav, Vertex::new(6.0, 2.0));
        assert!(!walls.is_wall_vertex(a));
        assert!(!walls.is_wall_vertex(b));
        // Ordinary wall corners still are.
        assert!(walls.is_wall_vertex(vertex_id_at(&nav, Vertex::new(10.0, 4.0))));

        let interior = connection_marker(3);
        let boundary = connection_marker(7);
        let (mut interior_seen, mut boundary_seen) = (false, false);
        for tri in &nav.triangles {
            for i in 0..3 {
                if tri.edge_markers[i] == interior {
                    // Two-sided seam: traversable.
                    assert!(!walls.is_wall_edge(tri, i));
                    interior_seen = true;
                }
                if tri.edge_markers[i] == boundary {
                    // Boundary seam: far side lives in another mesh, so
                    // single-mesh traversal still stops here.
                    assert!(walls.is_wall_edge(tri, i));
                    boundary_seen = true;
                }
            }
        }
        assert!(interior_seen && boundary_seen);
    }
}
