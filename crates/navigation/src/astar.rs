//! A* search over the navmesh's triangle adjacency graph.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rsnav_common::geom::nearest_point_on_segment;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::wall::WallInfo;

#[derive(Copy, Clone, Debug)]
struct Frontier {
    triangle: TriangleId,
    f_score: f64,
}

impl PartialEq for Frontier {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}
impl Eq for Frontier {}
impl Ord for Frontier {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse so BinaryHeap acts as a min-heap on f_score.
        other.f_score.partial_cmp(&self.f_score).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Reasons A* might fail.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AstarError {
    /// Start and goal triangles are in different reachability regions.
    UnreachableRegion,
    /// Open set exhausted without reaching the goal (every viable portal
    /// rejected, typically by `min_portal_width`).
    Unreachable,
}

impl std::fmt::Display for AstarError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            AstarError::UnreachableRegion => {
                "start and goal are in different reachability regions"
            }
            AstarError::Unreachable => {
                "no traversable path connects start and goal"
            }
        };
        f.write_str(s)
    }
}

impl std::error::Error for AstarError {}

/// Find the sequence of triangles A* walks from `start` to `goal`.
///
/// The search runs on the triangle adjacency graph, but step costs are
/// **portal-crossing**, not centroid-to-centroid: each triangle records
/// the point at which the route enters it — the closest point on the
/// shared portal edge to the predecessor's entry point — and a step
/// costs the straight-line distance between consecutive entry points.
/// The start triangle is entered at the real `start_point`, and the
/// final leg into the goal triangle also pays the hop on to
/// `goal_point`.
///
/// This matters because the funnel ([`crate::funnel`]) only ever
/// produces the shortest path *within* the channel A* commits to. A
/// centroid metric can rank a channel that wraps around an obstacle
/// below the tight straight-then-turn channel — the centroid path
/// over-estimates the funnelled length by a different amount per
/// channel — and the funnel then faithfully renders the detour.
/// Portal-crossing costs track the funnelled length closely enough
/// that A* picks the right channel. The cost and heuristic stay
/// consistent (triangle inequality), so the closed set remains valid;
/// entry points are chosen greedily per triangle, so this is a close
/// approximation rather than a proof of optimality.
///
/// Edges considered for traversal:
/// - Not a wall (constrained or boundary).
/// - Wide enough for the agent body: when `min_portal_width > 0`, the
///   portal edge must be longer than the inward shift the funnel will
///   apply to it — `min_portal_width` for *each* endpoint that is a
///   wall vertex (so a portal flanked by two walls needs more than
///   `2 * min_portal_width`). This keeps A*'s route choice in lockstep
///   with [`crate::funnel`]'s clearance model: A* never commits to a
///   corridor the funnel would have to collapse to a sub-body-width gap.
///
/// The returned vector starts with `start` and ends with `goal`.
pub fn astar(
    nav: &NavMesh,
    walls: &WallInfo,
    start: TriangleId,
    goal: TriangleId,
    start_point: Vertex,
    goal_point: Vertex,
    min_portal_width: f64,
) -> Result<Vec<TriangleId>, AstarError> {
    if start == goal {
        return Ok(vec![start]);
    }
    if !nav.reachable(start, goal) {
        return Err(AstarError::UnreachableRegion);
    }

    let n = nav.triangle_count();
    let mut g_score = vec![f64::INFINITY; n];
    let mut came_from: Vec<TriangleId> = vec![TriangleId::INVALID; n];
    // Point at which the best-known route enters each triangle. The
    // start triangle is entered at the real `start_point`.
    let mut entry = vec![Vertex::ZERO; n];
    let mut closed = vec![false; n];
    let mut heap: BinaryHeap<Frontier> = BinaryHeap::new();

    g_score[start.index()] = 0.0;
    entry[start.index()] = start_point;
    heap.push(Frontier {
        triangle: start,
        f_score: start_point.distance(goal_point),
    });

    while let Some(Frontier { triangle, .. }) = heap.pop() {
        if triangle == goal {
            return Ok(reconstruct(&came_from, start, goal));
        }
        if closed[triangle.index()] {
            continue;
        }
        closed[triangle.index()] = true;

        let tri = nav.triangle(triangle);
        let cur_entry = entry[triangle.index()];

        for edge in 0..3 {
            if walls.is_wall_edge(tri, edge) {
                continue;
            }
            let va = tri.vertices[(edge + 1) % 3];
            let vb = tri.vertices[(edge + 2) % 3];
            let pa = nav.vertex(va);
            let pb = nav.vertex(vb);

            if min_portal_width > 0.0 {
                // The funnel pulls each portal endpoint that is a wall
                // vertex inward by `min_portal_width`; the width the
                // agent body can actually use is the edge length minus
                // those shifts. Reject a portal that leaves no room.
                // Mirrors `funnel::oriented_portal` exactly.
                let needed = (if walls.is_wall_vertex(va) { min_portal_width } else { 0.0 })
                    + (if walls.is_wall_vertex(vb) { min_portal_width } else { 0.0 });
                if pa.distance(pb) <= needed {
                    continue;
                }
            }

            let neighbor = tri.neighbors[edge];
            if closed[neighbor.index()] {
                continue;
            }

            // Portal-crossing cost: the route enters `neighbor` at the
            // closest point on the shared portal to where it entered
            // `triangle`. The step is the distance between those two
            // entry points; the leg into the goal triangle also pays
            // the final hop on to `goal_point`, and that triangle's
            // heuristic is then 0 (its `g` already covers the rest).
            let crossing = nearest_point_on_segment(pa, pb, cur_entry);
            let mut step_cost = cur_entry.distance(crossing);
            let h = if neighbor == goal {
                step_cost += crossing.distance(goal_point);
                0.0
            } else {
                crossing.distance(goal_point)
            };
            let tentative_g = g_score[triangle.index()] + step_cost;
            if tentative_g < g_score[neighbor.index()] {
                g_score[neighbor.index()] = tentative_g;
                came_from[neighbor.index()] = triangle;
                entry[neighbor.index()] = crossing;
                heap.push(Frontier {
                    triangle: neighbor,
                    f_score: tentative_g + h,
                });
            }
        }
    }

    Err(AstarError::Unreachable)
}

fn reconstruct(
    came_from: &[TriangleId],
    start: TriangleId,
    goal: TriangleId,
) -> Vec<TriangleId> {
    let mut path = Vec::new();
    let mut cur = goal;
    path.push(cur);
    while cur != start {
        cur = came_from[cur.index()];
        path.push(cur);
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    // Tests live in path.rs alongside the integrated find_path tests.
}
