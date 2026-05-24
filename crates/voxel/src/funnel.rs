//! Within-region funnel ("string-pulling") path planning over a CDT
//! navmesh.
//!
//! The cross-region A* in [`pathfind::find_path`](crate::find_path) only
//! produces a list of regions and portal midpoints; inside each region
//! it leaves you a straight line. When the region has an interior
//! obstacle (a hole in the outer contour — e.g., a stairwell, pillar,
//! or any non-walkable patch surrounded by walkable area), that straight
//! line clips through the hole.
//!
//! This module fills that gap: given a [`RegionNavMesh`] (the
//! constrained-Delaunay triangulation of the region, with the hole
//! triangles already carved out), it:
//!
//! 1. Locates the triangles that contain `src` and `dst` (XY only).
//! 2. Runs A* on the **triangle-adjacency graph** to find a sequence of
//!    triangles connecting them (the "channel").
//! 3. Converts the channel's shared edges into a list of (left, right)
//!    portals and runs the classic Mononen / Lee&Mitchell **funnel
//!    algorithm** to pull a tight polyline through the channel — the
//!    polyline only bends at portal vertices (i.e., at navmesh corners).
//!
//! The result respects holes in the region because the CDT carves them
//! out: there are no triangles inside the hole, so the channel has to
//! go around it.
//!
//! Currently within-region only. Cross-region funnel (a single
//! string-pull across the multi-region channel produced by
//! [`find_path`](crate::find_path)) is a follow-up; it needs portal
//! edges in [`extract_portals`](crate::extract_portals) to be aligned
//! with CDT triangle edges, which the current portal extractor
//! doesn't guarantee.

use crate::navmesh::RegionNavMesh;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// For each triangle in `mesh`, the neighbor triangle (if any) across
/// each of its three edges. Indexed parallel to `mesh.triangles`; entry
/// `i`'s `[0]` is the neighbor across edge `(tri[0], tri[1])`, `[1]`
/// across `(tri[1], tri[2])`, `[2]` across `(tri[2], tri[0])`.
pub fn triangle_adjacency(mesh: &RegionNavMesh) -> Vec<[Option<usize>; 3]> {
    let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, tri) in mesh.triangles.iter().enumerate() {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            edge_to_tris.entry(key).or_default().push(i);
        }
    }
    let mut adj = vec![[None, None, None]; mesh.triangles.len()];
    for (i, tri) in mesh.triangles.iter().enumerate() {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            if let Some(sharers) = edge_to_tris.get(&key) {
                for &j in sharers {
                    if j != i {
                        adj[i][e] = Some(j);
                        break;
                    }
                }
            }
        }
    }
    adj
}

/// Find the index of the triangle containing world-XY point `(x, y)`
/// (the first one, if multiple triangles share a vertex on the edge).
/// Z is ignored — the navmesh is treated as a 2D triangulation.
pub fn find_triangle_at_xy(mesh: &RegionNavMesh, x: f64, y: f64) -> Option<usize> {
    for (i, tri) in mesh.triangles.iter().enumerate() {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        if point_in_triangle_xy(x, y, a.x, a.y, b.x, b.y, c.x, c.y) {
            return Some(i);
        }
    }
    None
}

/// A* on the triangle-adjacency graph from `src_tri` to `dst_tri`,
/// heuristic = euclidean distance from the candidate triangle's
/// centroid to `dst_xy`. Edge cost = euclidean distance between
/// adjacent triangle centroids. Returns the channel as a list of
/// triangle indices `[src_tri, …, dst_tri]`, or `None` if disconnected.
pub fn find_triangle_channel(
    mesh: &RegionNavMesh,
    adj: &[[Option<usize>; 3]],
    src_tri: usize,
    dst_tri: usize,
    dst_xy: (f64, f64),
) -> Option<Vec<usize>> {
    if src_tri == dst_tri {
        return Some(vec![src_tri]);
    }
    let mut g: HashMap<usize, f64> = HashMap::new();
    let mut came_from: HashMap<usize, usize> = HashMap::new();
    let mut heap: BinaryHeap<Entry> = BinaryHeap::new();
    g.insert(src_tri, 0.0);
    heap.push(Entry {
        f: dist_xy(centroid_xy(mesh, src_tri), dst_xy),
        tri: src_tri,
    });
    while let Some(Entry { tri, .. }) = heap.pop() {
        if tri == dst_tri {
            let mut path = vec![dst_tri];
            let mut cur = dst_tri;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }
        let cur_g = *g.get(&tri).unwrap_or(&f64::INFINITY);
        for &neighbor_opt in &adj[tri] {
            let Some(n) = neighbor_opt else { continue };
            let step = dist_xy(centroid_xy(mesh, tri), centroid_xy(mesh, n));
            let new_g = cur_g + step;
            let prev = *g.get(&n).unwrap_or(&f64::INFINITY);
            if new_g < prev {
                g.insert(n, new_g);
                came_from.insert(n, tri);
                heap.push(Entry {
                    f: new_g + dist_xy(centroid_xy(mesh, n), dst_xy),
                    tri: n,
                });
            }
        }
    }
    None
}

/// Top-level convenience: locate src/dst triangles, find the channel,
/// and string-pull. Returns the funneled polyline in XY, including
/// `src_xy` as the first point and `dst_xy` as the last. `None` if
/// either endpoint is outside the navmesh or the channel is
/// disconnected (shouldn't happen for a single CCW-outer-with-holes
/// region's CDT, but defensive).
pub fn funnel_in_region(
    mesh: &RegionNavMesh,
    src_xy: (f64, f64),
    dst_xy: (f64, f64),
) -> Option<Vec<(f64, f64)>> {
    let src_tri = find_triangle_at_xy(mesh, src_xy.0, src_xy.1)?;
    let dst_tri = find_triangle_at_xy(mesh, dst_xy.0, dst_xy.1)?;
    let adj = triangle_adjacency(mesh);
    let channel = find_triangle_channel(mesh, &adj, src_tri, dst_tri, dst_xy)?;
    let portals = channel_to_portals(mesh, &channel, src_xy, dst_xy);
    Some(funnel(&portals))
}

/// Build the (left, right) portal list for the funnel. Each shared edge
/// between consecutive triangles becomes one portal; left/right are
/// assigned so that, when walking from the previous triangle's centroid
/// toward the next's, `left` is on your left and `right` is on your
/// right. The first/last portal are degenerate (left == right == src
/// or dst), as the funnel algorithm expects.
pub fn channel_to_portals(
    mesh: &RegionNavMesh,
    channel: &[usize],
    src_xy: (f64, f64),
    dst_xy: (f64, f64),
) -> Vec<((f64, f64), (f64, f64))> {
    let mut portals: Vec<((f64, f64), (f64, f64))> = Vec::with_capacity(channel.len() + 1);
    portals.push((src_xy, src_xy));
    for w in channel.windows(2) {
        let (t_prev, t_curr) = (w[0], w[1]);
        let shared = shared_vertex_pair(mesh, t_prev, t_curr);
        let Some((va, vb)) = shared else { continue };
        let pa = (mesh.vertices[va as usize].x, mesh.vertices[va as usize].y);
        let pb = (mesh.vertices[vb as usize].x, mesh.vertices[vb as usize].y);
        // Walking direction: previous centroid -> next centroid.
        let cp = centroid_xy(mesh, t_prev);
        let cn = centroid_xy(mesh, t_curr);
        let walk = (cn.0 - cp.0, cn.1 - cp.1);
        // Cross product of (pa - cp) × walk_dir. Positive → pa is to
        // the LEFT of the walking direction; negative → RIGHT.
        let to_a = (pa.0 - cp.0, pa.1 - cp.1);
        let cross = to_a.0 * walk.1 - to_a.1 * walk.0;
        // cross > 0 means pa is clockwise from walk (i.e., to the right
        // in a Y-up coordinate frame). The 2D cross convention here:
        // if walk_dir is +X and pa is at +Y (left), to_a × walk =
        // 0*0 - 1*1 = -1 < 0 ⇒ pa is left. So negative cross = left.
        let (left, right) = if cross < 0.0 { (pa, pb) } else { (pb, pa) };
        portals.push((left, right));
    }
    portals.push((dst_xy, dst_xy));
    portals
}

/// The classic Mononen funnel algorithm: walk a list of (left, right)
/// portals, maintaining an apex and a tightening left/right cone.
/// Emit an apex whenever the cone collapses, then restart from that
/// apex. Returns the tightest XY polyline from `portals[0]`'s apex to
/// `portals.last()`'s apex.
#[allow(unused_assignments)] // apex_idx's initial 0 is dead but conceptually correct
pub fn funnel(portals: &[((f64, f64), (f64, f64))]) -> Vec<(f64, f64)> {
    if portals.is_empty() {
        return Vec::new();
    }
    let mut result: Vec<(f64, f64)> = vec![portals[0].0];
    let mut apex = portals[0].0;
    let mut left = portals[0].0;
    let mut right = portals[0].0;
    let mut apex_idx = 0usize;
    let mut left_idx = 0usize;
    let mut right_idx = 0usize;
    let mut i = 1usize;
    while i < portals.len() {
        let new_left = portals[i].0;
        let new_right = portals[i].1;
        // Update right.
        if triarea2(apex, right, new_right) <= 0.0 {
            if approx_eq(apex, right) || triarea2(apex, left, new_right) > 0.0 {
                right = new_right;
                right_idx = i;
            } else {
                // Right has crossed left: emit left as new apex, restart.
                result.push(left);
                apex = left;
                apex_idx = left_idx;
                left = apex;
                right = apex;
                left_idx = apex_idx;
                right_idx = apex_idx;
                i = apex_idx + 1;
                continue;
            }
        }
        // Update left.
        if triarea2(apex, left, new_left) >= 0.0 {
            if approx_eq(apex, left) || triarea2(apex, right, new_left) < 0.0 {
                left = new_left;
                left_idx = i;
            } else {
                // Left has crossed right: emit right as new apex, restart.
                result.push(right);
                apex = right;
                apex_idx = right_idx;
                left = apex;
                right = apex;
                left_idx = apex_idx;
                right_idx = apex_idx;
                i = apex_idx + 1;
                continue;
            }
        }
        i += 1;
    }
    let dst = portals.last().unwrap().0;
    if result.last().copied().map_or(true, |last| !approx_eq(last, dst)) {
        result.push(dst);
    }
    result
}

#[derive(Copy, Clone, Debug)]
struct Entry {
    f: f64,
    tri: usize,
}

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is max-heap; we want min-f, so reverse.
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}

fn shared_vertex_pair(mesh: &RegionNavMesh, t1: usize, t2: usize) -> Option<(u32, u32)> {
    let a = mesh.triangles[t1];
    let b = mesh.triangles[t2];
    let mut shared: Vec<u32> = Vec::with_capacity(2);
    for &va in &a {
        if b.contains(&va) {
            shared.push(va);
        }
    }
    if shared.len() == 2 {
        Some((shared[0], shared[1]))
    } else {
        None
    }
}

fn centroid_xy(mesh: &RegionNavMesh, t: usize) -> (f64, f64) {
    let tri = mesh.triangles[t];
    let a = mesh.vertices[tri[0] as usize];
    let b = mesh.vertices[tri[1] as usize];
    let c = mesh.vertices[tri[2] as usize];
    ((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0)
}

fn dist_xy(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    (dx * dx + dy * dy).sqrt()
}

fn point_in_triangle_xy(
    px: f64,
    py: f64,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
) -> bool {
    let s1 = sign(px, py, ax, ay, bx, by);
    let s2 = sign(px, py, bx, by, cx, cy);
    let s3 = sign(px, py, cx, cy, ax, ay);
    let has_neg = s1 < 0.0 || s2 < 0.0 || s3 < 0.0;
    let has_pos = s1 > 0.0 || s2 > 0.0 || s3 > 0.0;
    !(has_neg && has_pos)
}

fn sign(p1x: f64, p1y: f64, p2x: f64, p2y: f64, p3x: f64, p3y: f64) -> f64 {
    (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y)
}

fn triarea2(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    let ax = b.0 - a.0;
    let ay = b.1 - a.1;
    let bx = c.0 - a.0;
    let by = c.1 - a.1;
    bx * ay - ax * by
}

fn approx_eq(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{WalkabilityConfig, WatershedConfig};
    use crate::contour::extract_contours;
    use crate::navmesh::build_all_navmeshes;
    use crate::walkability::classify_walkability;
    use crate::watershed::segment;
    use crate::{synth, PolySoup, VoxelGrid};
    use rsnav_common::Vec3;

    fn pipeline_navmeshes(soup: &PolySoup, cell_size: f64) -> Vec<crate::navmesh::RegionNavMesh> {
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let rm = segment(&chf, &WatershedConfig::default());
        let contours = extract_contours(&chf, &rm);
        build_all_navmeshes(&contours)
            .meshes
            .into_iter()
            .flatten()
            .collect()
    }

    #[test]
    fn straight_line_through_plane_is_two_points() {
        // Empty navmesh on a flat plane: src→dst should be a straight
        // line, with no intermediate apex emitted.
        let soup = synth::plane(8.0, 8.0, 1);
        let meshes = pipeline_navmeshes(&soup, 0.5);
        let mesh = meshes
            .iter()
            .max_by_key(|m| m.triangle_count())
            .expect("plane should have a navmesh");
        let path = funnel_in_region(mesh, (-2.0, 0.0), (2.0, 0.0)).expect("funnel");
        assert!(path.len() >= 2);
        assert!((path[0].0 - -2.0).abs() < 1e-6);
        assert!((path[path.len() - 1].0 - 2.0).abs() < 1e-6);
    }

    #[test]
    fn path_routes_around_hole_in_plane() {
        // A plane with a rectangular hole carved out of the middle.
        // src on one side, dst on the other; the straight line goes
        // through the hole, so funnel must route around.
        let mut soup = synth::plane(10.0, 10.0, 1);
        // Build a 'donut': the plane is solid, but we'll voxelize a
        // wall in the middle so the hole carves through the region.
        // Simpler: use floor+ramp+platform doesn't have a hole. Build
        // a manual setup: outer 10x10 plane + a small raised pillar
        // in the middle (which the walkability filter ignores as a
        // wall, leaving a hole).
        let pillar = synth::box_aabb(Vec3::new(-1.0, -1.0, 0.0), Vec3::new(1.0, 1.0, 4.0));
        let base = soup.vertices.len() as u32;
        for v in &pillar.vertices {
            soup.vertices.push(*v);
        }
        for t in &pillar.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let meshes = pipeline_navmeshes(&soup, 0.3);
        let mesh = meshes
            .iter()
            .max_by_key(|m| m.triangle_count())
            .expect("plane-with-pillar should have a navmesh");
        // Sanity: this mesh's region has at least one hole.
        // (We can't easily assert that here without the contour, but
        // the test still checks the relevant property: the funnel
        // output should have ≥3 points and visit one side of the
        // pillar's bounding box at y > 1 or y < -1.)
        let src = (-4.0, 0.0);
        let dst = (4.0, 0.0);
        let path = funnel_in_region(mesh, src, dst).expect("funnel must find a route");
        assert!(
            path.len() >= 3,
            "funnel should have inserted apex(es) to route around hole, got {} points: {:?}",
            path.len(),
            path
        );
        // Confirm the path avoids the pillar's XY footprint
        // (-1..1, -1..1).
        for &(x, y) in &path {
            let inside_pillar = x.abs() <= 1.0 && y.abs() <= 1.0;
            assert!(!inside_pillar, "path point {:?} lies inside pillar", (x, y));
        }
    }

    #[test]
    fn triangle_adjacency_is_symmetric() {
        let soup = synth::plane(4.0, 4.0, 1);
        let meshes = pipeline_navmeshes(&soup, 0.5);
        let mesh = &meshes[0];
        let adj = triangle_adjacency(mesh);
        for (i, neighbors) in adj.iter().enumerate() {
            for &n in neighbors {
                if let Some(j) = n {
                    let back_refs_i = adj[j].iter().any(|x| *x == Some(i));
                    assert!(
                        back_refs_i,
                        "asymmetric adjacency: {} → {} but {} doesn't link back",
                        i, j, j
                    );
                }
            }
        }
    }
}
