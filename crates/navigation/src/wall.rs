//! Wall-aware navmesh helpers shared by A*, the funnel, and line-of-sight.
//!
//! A "wall edge" is any constrained triangle edge (marker != 0), any
//! boundary edge (no neighbor), or — at runtime — any internal portal edge
//! currently cut by a *closed* door (see [`crate::doors`]). A "wall vertex"
//! is any vertex incident to at least one wall edge.
//!
//! [`WallInfo`] is the single "is this impassable?" oracle the traversal
//! code consults. Build it with [`WallInfo::from_navmesh`] for a mesh with
//! no doors, or [`WallInfo::from_navmesh_with_doors`] to fold the current
//! door states into the same edge/vertex tests — A*, the funnel, and LOS
//! then all react to a door opening or closing without any per-feature
//! plumbing. Rebuild it whenever the mesh or any door state changes; it is
//! `O(triangles)`, the same lifecycle as [`Bsp`](rsnav_bsp::Bsp).

use std::collections::HashSet;

use rsnav_common::VertexId;
use rsnav_navmesh::{NavMesh, NavTriangle};

use crate::doors::DoorSet;

/// Canonical key for an undirected navmesh edge: its two endpoint vertex
/// indices, smaller first. Shared by both triangles that own the edge.
pub(crate) type EdgeKey = (u32, u32);

#[inline]
pub(crate) fn edge_key(a: VertexId, b: VertexId) -> EdgeKey {
    let (a, b) = (a.index() as u32, b.index() as u32);
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

/// The set of impassable edges and the wall-vertex lookup the traversal
/// stages share. `O(n)` to build, `O(1)` to query.
///
/// `wall_vertex` folds in closed-door endpoints, and `closed_edges` carries
/// the door cuts, so [`is_wall_edge`](Self::is_wall_edge) and
/// [`is_wall_vertex`](Self::is_wall_vertex) are the only wall tests the rest
/// of the crate needs — static walls and dynamic doors look identical to a
/// caller.
#[derive(Clone, Debug)]
pub struct WallInfo {
    wall_vertex: Vec<bool>,
    /// Edges cut by a currently-closed door. Empty when there are no closed
    /// doors, which makes [`is_wall_edge`](Self::is_wall_edge) collapse to
    /// the static test with no per-edge hashing.
    closed_edges: HashSet<EdgeKey>,
}

impl WallInfo {
    /// Build from the static mesh only — no doors.
    pub fn from_navmesh(nav: &NavMesh) -> Self {
        Self::build(nav, HashSet::new())
    }

    /// Build from the mesh plus the current door states: every edge cut by
    /// a *closed* door is treated as a wall, and its endpoints become wall
    /// vertices (so the funnel insets around them and A*'s min-width check
    /// treats them as walls — for a framed door these are already static
    /// wall vertices, this also covers a door cut across open floor).
    /// Open doors contribute nothing.
    pub fn from_navmesh_with_doors(nav: &NavMesh, doors: &DoorSet) -> Self {
        Self::build(nav, doors.closed_edge_keys())
    }

    fn build(nav: &NavMesh, closed_edges: HashSet<EdgeKey>) -> Self {
        let mut wall_vertex = vec![false; nav.vertex_count()];
        for tri in &nav.triangles {
            for i in 0..3 {
                if is_wall_edge_local(tri, i) {
                    let (va, vb) = tri.edge_vertices(i);
                    wall_vertex[va.index()] = true;
                    wall_vertex[vb.index()] = true;
                }
            }
        }
        // A closed door behaves like a wall, so its endpoints are wall
        // vertices for the duration it stays shut. Never *clears* a static
        // flag, so reopening the door (rebuild without it) restores exactly
        // the static set.
        for &(a, b) in &closed_edges {
            wall_vertex[a as usize] = true;
            wall_vertex[b as usize] = true;
        }
        Self {
            wall_vertex,
            closed_edges,
        }
    }

    #[inline]
    pub fn is_wall_vertex(&self, v: VertexId) -> bool {
        self.wall_vertex[v.index()]
    }

    /// Is edge `i` of `tri` impassable — a static wall (constrained or
    /// boundary) or an edge cut by a currently-closed door?
    #[inline]
    pub fn is_wall_edge(&self, tri: &NavTriangle, i: usize) -> bool {
        is_wall_edge_local(tri, i) || self.is_closed_door_edge(tri, i)
    }

    #[inline]
    fn is_closed_door_edge(&self, tri: &NavTriangle, i: usize) -> bool {
        if self.closed_edges.is_empty() {
            return false;
        }
        let (a, b) = tri.edge_vertices(i);
        self.closed_edges.contains(&edge_key(a, b))
    }
}

/// Is edge `i` of `tri` a *static* wall (constrained or boundary)? Doors are
/// not considered here — use [`WallInfo::is_wall_edge`] for the door-aware
/// test. This remains the primitive both the wall-vertex build and the
/// door resolver use to recognize the unconstrained internal portals.
#[inline]
pub fn is_wall_edge_local(tri: &NavTriangle, i: usize) -> bool {
    !tri.neighbors[i].is_valid() || tri.edge_markers[i] != 0
}
