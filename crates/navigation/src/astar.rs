//! A* search over the navmesh's triangle adjacency graph.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::wall::is_wall_edge_local;

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

/// Find the sequence of triangles A* walks from `start` to `goal`.
///
/// `goal_point` is used as the heuristic target — we estimate remaining
/// cost as the euclidean distance from each triangle's centroid to
/// `goal_point`.
///
/// Edges considered for traversal:
/// - Not a wall (constrained or boundary).
/// - Edge length > `min_portal_width` (when `min_portal_width > 0`).
///
/// The returned vector starts with `start` and ends with `goal`.
pub fn astar(
    nav: &NavMesh,
    start: TriangleId,
    goal: TriangleId,
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
    let mut closed = vec![false; n];
    let mut heap: BinaryHeap<Frontier> = BinaryHeap::new();

    g_score[start.index()] = 0.0;
    let h_start = nav.triangle(start).centroid.distance(goal_point);
    heap.push(Frontier { triangle: start, f_score: h_start });

    while let Some(Frontier { triangle, .. }) = heap.pop() {
        if triangle == goal {
            return Ok(reconstruct(&came_from, start, goal));
        }
        if closed[triangle.index()] {
            continue;
        }
        closed[triangle.index()] = true;

        let tri = nav.triangle(triangle);
        let cur_centroid = tri.centroid;

        for edge in 0..3 {
            if is_wall_edge_local(tri, edge) {
                continue;
            }
            if min_portal_width > 0.0 {
                let (va, vb) = (
                    tri.vertices[(edge + 1) % 3],
                    tri.vertices[(edge + 2) % 3],
                );
                let pa = nav.vertex(va);
                let pb = nav.vertex(vb);
                if pa.distance(pb) <= min_portal_width {
                    continue;
                }
            }
            let neighbor = tri.neighbors[edge];
            if closed[neighbor.index()] {
                continue;
            }
            let n_tri = nav.triangle(neighbor);
            let step_cost = cur_centroid.distance(n_tri.centroid);
            let tentative_g = g_score[triangle.index()] + step_cost;
            if tentative_g < g_score[neighbor.index()] {
                g_score[neighbor.index()] = tentative_g;
                came_from[neighbor.index()] = triangle;
                let h = n_tri.centroid.distance(goal_point);
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
