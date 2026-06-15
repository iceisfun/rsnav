//! Simple Stupid Funnel string-pull, with optional wall-aware portal
//! shrinking so the output path keeps a configurable distance from
//! constrained edges.

use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::wall::WallInfo;

/// Turn a triangle sequence plus a start and goal into a string-pulled
/// polyline.
///
/// `distance_from_wall` (≥ 0) shifts each portal endpoint that's a wall
/// vertex inward along the portal by that much. When the portal would
/// collapse (both endpoints are walls and the portal length is less than
/// `2 * distance_from_wall`) we clamp to its midpoint — the path is
/// forced through the narrow gap, but never crosses outside the portal.
///
/// The output always begins with `start` and ends with `goal`. Consecutive
/// duplicates are filtered.
pub fn funnel(
    nav: &NavMesh,
    walls: &WallInfo,
    triangles: &[TriangleId],
    start: Vertex,
    goal: Vertex,
    distance_from_wall: f64,
) -> Vec<Vertex> {
    if triangles.is_empty() {
        return vec![start, goal];
    }

    // Build oriented portals (left, right) for each transition between
    // consecutive triangles, then append a final degenerate portal at the
    // goal.
    let mut portals: Vec<(Vertex, Vertex)> = Vec::with_capacity(triangles.len());
    portals.push((start, start)); // entry "portal" at the start point
    for i in 0..triangles.len().saturating_sub(1) {
        let from = triangles[i];
        let to = triangles[i + 1];
        if let Some((left, right)) = oriented_portal(nav, walls, from, to, distance_from_wall) {
            portals.push((left, right));
        }
    }
    portals.push((goal, goal));

    string_pull(&portals)
}

/// Find the shared edge between `from` and `to` and return its endpoints
/// as (left, right) relative to the direction of travel through the
/// portal. Returns `None` if the triangles don't share an edge (shouldn't
/// happen for a valid A* output).
///
/// When `distance_from_wall > 0` and either endpoint is a wall vertex,
/// shifts it inward along the portal by that much (clamped so the portal
/// can't collapse past its midpoint).
fn oriented_portal(
    nav: &NavMesh,
    walls: &WallInfo,
    from: TriangleId,
    to: TriangleId,
    distance_from_wall: f64,
) -> Option<(Vertex, Vertex)> {
    let t_from = nav.triangle(from);
    let mut edge_idx: Option<usize> = None;
    for i in 0..3 {
        if t_from.neighbors[i] == to {
            edge_idx = Some(i);
            break;
        }
    }
    let i = edge_idx?;
    let (va, vb) = (
        t_from.vertices[(i + 1) % 3],
        t_from.vertices[(i + 2) % 3],
    );
    let pa = nav.vertex(va);
    let pb = nav.vertex(vb);

    // Orient: figure out which endpoint is "left" relative to the centroid-
    // to-centroid travel direction. orient2d(from_c, to_c, pt) > 0 → pt is
    // to the left.
    let from_c = t_from.centroid;
    let to_c = nav.triangle(to).centroid;
    let pa_orient = rsnav_common::geom::orient2d(from_c, to_c, pa);
    let (left_v, right_v, left_p, right_p) = if pa_orient > 0.0 {
        (va, vb, pa, pb)
    } else {
        (vb, va, pb, pa)
    };

    if distance_from_wall <= 0.0 {
        return Some((left_p, right_p));
    }

    // Shift wall endpoints inward along the portal.
    let len = left_p.distance(right_p);
    if len == 0.0 {
        return Some((left_p, right_p));
    }
    // Per-side shift, clamped so the two shifts together never exceed the
    // portal length.
    let left_is_wall = walls.is_wall_vertex(left_v);
    let right_is_wall = walls.is_wall_vertex(right_v);
    let raw_shift_left = if left_is_wall { distance_from_wall } else { 0.0 };
    let raw_shift_right = if right_is_wall { distance_from_wall } else { 0.0 };
    let total_raw = raw_shift_left + raw_shift_right;
    let (shift_left, shift_right) = if total_raw <= len {
        (raw_shift_left, raw_shift_right)
    } else {
        let scale = len / total_raw;
        (raw_shift_left * scale, raw_shift_right * scale)
    };
    let dir = (right_p - left_p) * (1.0 / len);
    let new_left = left_p + dir * shift_left;
    let new_right = right_p + dir * -shift_right;
    Some((new_left, new_right))
}

/// Classic Simple Stupid Funnel (Mononen). `portals[i] = (left, right)`
/// in travel order. Returns the string-pulled polyline.
///
/// Pure geometry — no mesh, no coordinate space — so the tiled pathfinder
/// ([`crate::tiled`]) reuses it over world-space portals.
pub(crate) fn string_pull(portals: &[(Vertex, Vertex)]) -> Vec<Vertex> {
    if portals.is_empty() {
        return Vec::new();
    }
    let mut path = Vec::new();
    let mut apex = portals[0].0;
    let mut left = portals[0].0;
    let mut right = portals[0].1;
    let mut apex_i = 0usize;
    let mut left_i = 0usize;
    let mut right_i = 0usize;

    path.push(apex);

    let mut i = 1;
    while i < portals.len() {
        let (p_left, p_right) = portals[i];

        // Update the right side.
        //
        // Sign convention: our `tri_area2` is the standard CCW orient2d
        // (positive = CCW). Mononen's SSF pseudocode uses the negated
        // form (positive = CW), so every comparison is flipped relative
        // to his blog post.
        if tri_area2(apex, right, p_right) >= 0.0 {
            if apex == right || tri_area2(apex, left, p_right) < 0.0 {
                // Tightens the funnel.
                right = p_right;
                right_i = i;
            } else {
                // Right crossed left — apex turns left, restart funnel.
                if path.last() != Some(&left) {
                    path.push(left);
                }
                apex = left;
                apex_i = left_i;
                left = apex;
                right = apex;
                left_i = apex_i;
                right_i = apex_i;
                i = apex_i + 1;
                continue;
            }
        }

        // Update the left side.
        if tri_area2(apex, left, p_left) <= 0.0 {
            if apex == left || tri_area2(apex, right, p_left) > 0.0 {
                left = p_left;
                left_i = i;
            } else {
                if path.last() != Some(&right) {
                    path.push(right);
                }
                apex = right;
                apex_i = right_i;
                left = apex;
                right = apex;
                left_i = apex_i;
                right_i = apex_i;
                i = apex_i + 1;
                continue;
            }
        }

        i += 1;
    }
    let _ = apex_i; // only written-to; kept as a structural mirror of the C source

    // Append the goal (last portal's left/right are equal).
    let goal = portals.last().unwrap().0;
    if path.last() != Some(&goal) {
        path.push(goal);
    }
    path
}

/// Twice the signed area of triangle `(a, b, c)`. Positive when CCW
/// (standard orient2d sign convention). See `string_pull` for how this
/// relates to Mononen's SSF pseudocode.
#[inline]
fn tri_area2(a: Vertex, b: Vertex, c: Vertex) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}
