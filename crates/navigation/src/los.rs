//! Line of sight: walk a directed segment across the navmesh's triangles,
//! stopping at the first constrained / boundary edge that the segment
//! would cross.

use rsnav_common::geom::{point_in_triangle, segment_intersection};
use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::wall::WallInfo;

/// Outcome of a line-of-sight query.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum LineOfSightResult {
    /// The whole segment lies inside the walkable navmesh; nothing blocks
    /// the view from `from` to `to`.
    Clear,
    /// A wall blocks the view at `point` — the segment crosses a
    /// constrained or boundary edge there.
    Blocked { point: Vertex },
    /// `from` is not inside any triangle of the mesh, so we can't even
    /// start the walk.
    SourceOutsideMesh,
    /// The triangle walk hit a numerical degeneracy — the segment grazed
    /// a vertex or ran collinear with a triangle edge — or its step cap
    /// before it could reach `to`. Whether the segment is actually clear
    /// is **unknown**: callers must treat this conservatively (replan
    /// the route, stop a visibility ray short, etc.). It exists as its
    /// own variant precisely so an uncertain walk can't masquerade as a
    /// verified-clear one — reporting `Clear` here would be a silent
    /// false negative in exactly the dangerous direction.
    Indeterminate,
}

/// Walk the directed segment `from → to` through the mesh, starting in
/// `start_tri` (which must contain `from`). Returns the first wall hit,
/// `Clear` if the whole segment is inside walkable space, or
/// `Indeterminate` if a numerical degeneracy stopped the walk before it
/// could decide (see [`LineOfSightResult::Indeterminate`] — treat it as
/// "not clear").
///
/// `from` must lie inside `start_tri`. Callers typically obtain
/// `start_tri` via `bsp.locate(from)`.
///
/// `walls` is the wall oracle — pass one built with
/// [`WallInfo::from_navmesh_with_doors`] and the ray will stop at a *closed*
/// door exactly as it stops at a static wall.
pub fn line_of_sight(
    nav: &NavMesh,
    walls: &WallInfo,
    start_tri: TriangleId,
    from: Vertex,
    to: Vertex,
) -> LineOfSightResult {
    let mut cur_tri = start_tri;
    // Avoid infinite loops if numerics conspire against us — bound the
    // walk at twice the triangle count.
    let max_steps = nav.triangle_count() * 2 + 4;
    for _ in 0..max_steps {
        let tri = nav.triangle(cur_tri);

        if triangle_contains(nav, cur_tri, to) {
            return LineOfSightResult::Clear;
        }

        // Find which edge of `cur_tri` the segment exits through.
        // The exit edge is the one whose endpoints straddle the line
        // `from → to` and whose intersection parameter `t_seg ∈ [0, 1]`
        // is the largest (= furthest along the segment from `from`).
        //
        // We allow `t_seg = 0` so that on-edge sources work — when `from`
        // happens to lie exactly on an edge of the current triangle
        // (very common when the user clicks near a shared edge, or when
        // visibility-region cursors land on triangulation vertices), that
        // edge has t = 0 and is a valid exit if the ray points outward.
        // For interior sources the max-t pick still selects the real
        // exit (t > 0) over any on-edge artifact at t = 0.
        let mut best_edge: Option<(usize, Vertex, f64)> = None;
        for edge in 0..3 {
            let (va, vb) = (
                tri.vertices[(edge + 1) % 3],
                tri.vertices[(edge + 2) % 3],
            );
            let pa = nav.vertex(va);
            let pb = nav.vertex(vb);
            if let Some(hit) = segment_intersection(from, to, pa, pb) {
                if hit.t < -1e-9 {
                    continue;
                }
                if best_edge.map_or(true, |(_, _, t)| hit.t > t) {
                    best_edge = Some((edge, hit.point, hit.t));
                }
            }
        }

        let (edge, hit, _t) = match best_edge {
            Some(v) => v,
            None => {
                // No exit edge found, yet `to` isn't inside this
                // triangle either: the segment grazed a vertex or ran
                // collinear with an edge. We genuinely cannot tell if
                // the rest of the segment is clear — say so, rather
                // than claiming `Clear` and hiding a possible wall.
                return LineOfSightResult::Indeterminate;
            }
        };

        if walls.is_wall_edge(tri, edge) {
            return LineOfSightResult::Blocked { point: hit };
        }

        let neighbor = tri.neighbors[edge];
        if !neighbor.is_valid() {
            // Boundary — also a wall.
            return LineOfSightResult::Blocked { point: hit };
        }
        cur_tri = neighbor;
    }
    // Step cap reached without arriving at `to` — a numerical cycle in
    // the walk. Don't claim the segment is clear; report the uncertainty.
    LineOfSightResult::Indeterminate
}

/// Point-in-triangle for the navmesh's CCW triangles (boundary inclusive).
fn triangle_contains(nav: &NavMesh, tri_id: TriangleId, p: Vertex) -> bool {
    let tri = nav.triangle(tri_id);
    let a = nav.vertex(tri.vertices[0]);
    let b = nav.vertex(tri.vertices[1]);
    let c = nav.vertex(tri.vertices[2]);
    point_in_triangle(a, b, c, p)
}
