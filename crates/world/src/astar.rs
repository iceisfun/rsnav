//! Portal-crossing A* over `(layer, triangle)` nodes.
//!
//! Same cost model as `rsnav_navigation::astar`: each node records the
//! point (and height) at which the best-known route enters it, a step
//! costs the 3D distance between consecutive entry points, and the
//! heuristic is the 3D straight line to the goal — admissible because no
//! walkable route is shorter than it. The one addition is the seam
//! expansion: an edge that is a wall *within* its own mesh but has a
//! matched partner in the connection table expands into the partner
//! triangle exactly like an interior portal. Crossing a seam costs
//! nothing beyond the distance walked — a seam is floor, not a jump.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rsnav_common::geom::nearest_point_on_segment;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::{EdgeRef, LayerId, World};

/// One step of the corridor A* walks: a triangle plus the point/height
/// at which the route enters it.
#[derive(Copy, Clone, Debug)]
pub(crate) struct CorridorStep {
    pub layer: LayerId,
    pub tri: TriangleId,
    pub entry: Vertex,
    pub entry_z: f64,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WorldAstarError {
    /// Start and goal are in different world reachability components.
    UnreachableComponent,
    /// Open set exhausted without reaching the goal (every viable portal
    /// rejected, typically by `min_portal_width`).
    Unreachable,
}

#[derive(Copy, Clone, Debug)]
struct Frontier {
    node: usize,
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
        other.f_score.partial_cmp(&self.f_score).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub(crate) fn world_astar(
    world: &World,
    start: (LayerId, TriangleId),
    goal: (LayerId, TriangleId),
    start_point: Vertex,
    goal_point: Vertex,
    min_portal_width: f64,
) -> Result<Vec<CorridorStep>, WorldAstarError> {
    if !world.reachable(start, goal) {
        return Err(WorldAstarError::UnreachableComponent);
    }

    let start_z = world.layer(start.0).navmesh.z_at(start.1, start_point);
    let goal_z = world.layer(goal.0).navmesh.z_at(goal.1, goal_point);

    let start_node = world.flat_index(start.0, start.1);
    let goal_node = world.flat_index(goal.0, goal.1);
    if start_node == goal_node {
        return Ok(vec![CorridorStep {
            layer: start.0,
            tri: start.1,
            entry: start_point,
            entry_z: start_z,
        }]);
    }

    // Node ↔ (layer, tri) mapping: nodes are flat indices; recover the
    // layer by the offset table. Scratch arrays are sized for the whole
    // world — the same allocation profile per query as the single-mesh
    // search, just spanning all layers.
    let n = world.total_triangles();
    let mut g_score = vec![f64::INFINITY; n];
    let mut came_from: Vec<usize> = vec![usize::MAX; n];
    let mut entry = vec![Vertex::ZERO; n];
    let mut entry_z = vec![0.0f64; n];
    let mut closed = vec![false; n];
    let mut heap: BinaryHeap<Frontier> = BinaryHeap::new();

    g_score[start_node] = 0.0;
    entry[start_node] = start_point;
    entry_z[start_node] = start_z;
    heap.push(Frontier {
        node: start_node,
        f_score: dist3(start_point, start_z, goal_point, goal_z),
    });

    while let Some(Frontier { node, .. }) = heap.pop() {
        if node == goal_node {
            return Ok(reconstruct(world, &came_from, &entry, &entry_z, start_node, goal_node));
        }
        if closed[node] {
            continue;
        }
        closed[node] = true;

        let (layer_id, tri_id) = world.node_to_tri(node);
        let layer = world.layer(layer_id);
        let tri = layer.navmesh.triangle(tri_id);
        let cur_entry = entry[node];
        let cur_z = entry_z[node];

        for edge in 0..3usize {
            // Either an interior portal of this mesh, or a matched seam
            // crossing into another mesh. Everything else is a wall.
            let neighbor: Option<(LayerId, TriangleId)> =
                if !layer.walls.is_wall_edge(tri, edge) {
                    Some((layer_id, tri.neighbors[edge]))
                } else {
                    world
                        .seam_neighbor(EdgeRef {
                            layer: layer_id,
                            tri: tri_id,
                            edge: edge as u8,
                        })
                        .map(|r| (r.layer, r.tri))
                };
            let Some((n_layer, n_tri)) = neighbor else {
                continue;
            };

            let (va, vb) = tri.edge_vertices(edge);
            let pa = layer.navmesh.vertex(va);
            let pb = layer.navmesh.vertex(vb);

            if min_portal_width > 0.0 {
                // Mirror of the funnel's clearance model, same as the
                // single-mesh search. Seam endpoints are wall vertices
                // only where the seam meets a real wall (permeable
                // WallInfo), so an open seam costs no width.
                let needed = (if layer.walls.is_wall_vertex(va) { min_portal_width } else { 0.0 })
                    + (if layer.walls.is_wall_vertex(vb) { min_portal_width } else { 0.0 });
                if pa.distance(pb) <= needed {
                    continue;
                }
            }

            let n_node = world.flat_index(n_layer, n_tri);
            if closed[n_node] {
                continue;
            }

            let crossing = nearest_point_on_segment(pa, pb, cur_entry);
            let crossing_z = interp_edge_z(&layer.navmesh, va, vb, pa, pb, crossing);
            let mut step_cost = dist3(cur_entry, cur_z, crossing, crossing_z);
            let h = if n_node == goal_node {
                step_cost += dist3(crossing, crossing_z, goal_point, goal_z);
                0.0
            } else {
                dist3(crossing, crossing_z, goal_point, goal_z)
            };
            let tentative_g = g_score[node] + step_cost;
            if tentative_g < g_score[n_node] {
                g_score[n_node] = tentative_g;
                came_from[n_node] = node;
                entry[n_node] = crossing;
                entry_z[n_node] = crossing_z;
                heap.push(Frontier {
                    node: n_node,
                    f_score: tentative_g + h,
                });
            }
        }
    }

    Err(WorldAstarError::Unreachable)
}

fn reconstruct(
    world: &World,
    came_from: &[usize],
    entry: &[Vertex],
    entry_z: &[f64],
    start_node: usize,
    goal_node: usize,
) -> Vec<CorridorStep> {
    let mut steps = Vec::new();
    let mut cur = goal_node;
    loop {
        let (layer, tri) = world.node_to_tri(cur);
        steps.push(CorridorStep {
            layer,
            tri,
            entry: entry[cur],
            entry_z: entry_z[cur],
        });
        if cur == start_node {
            break;
        }
        cur = came_from[cur];
    }
    steps.reverse();
    steps
}

/// Straight-line distance between two points with heights.
#[inline]
pub(crate) fn dist3(a: Vertex, az: f64, b: Vertex, bz: f64) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let dz = bz - az;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Height of `p` on the edge `(va, vb)`, by linear interpolation between
/// the endpoint heights. `0.0` on a mesh without heights.
#[inline]
pub(crate) fn interp_edge_z(
    nav: &NavMesh,
    va: rsnav_common::VertexId,
    vb: rsnav_common::VertexId,
    pa: Vertex,
    pb: Vertex,
    p: Vertex,
) -> f64 {
    if !nav.has_z() {
        return 0.0;
    }
    let za = nav.vertex_z(va);
    let zb = nav.vertex_z(vb);
    let len2 = pa.distance_sq(pb);
    if len2 == 0.0 {
        return 0.5 * (za + zb);
    }
    let t = ((p - pa).dot(pb - pa) / len2).clamp(0.0, 1.0);
    za + t * (zb - za)
}
