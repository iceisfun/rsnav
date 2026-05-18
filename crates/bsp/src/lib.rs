//! BVH (AABB-tree) spatial index over a [`NavMesh`]'s triangles.
//!
//! Two queries:
//!
//! - [`Bsp::locate`]: given a point, find the triangle that contains it
//!   (or `None` if the point is outside the mesh). Average `O(log n)`.
//! - [`Bsp::nearest`]: given a point, find the triangle whose surface is
//!   closest, together with the closest point on that triangle and the
//!   euclidean distance. Average `O(log n)`.
//!
//! Build is `O(n log n)` (recursive median-split on triangle centroids
//! along the longest axis of each node's AABB).

#![forbid(unsafe_code)]

use rsnav_common::{Aabb, TriangleId, Vertex, geom};
use rsnav_navmesh::NavMesh;

/// One node in the BVH. Internal nodes have two children; leaves carry a
/// contiguous slice into [`Bsp::triangle_indices`].
#[derive(Clone, Debug)]
enum BspNode {
    Internal { aabb: Aabb, left: u32, right: u32 },
    Leaf { aabb: Aabb, start: u32, len: u32 },
}

#[derive(Clone, Debug)]
pub struct Bsp {
    nodes: Vec<BspNode>,
    triangle_indices: Vec<u32>,
    triangle_aabbs: Vec<Aabb>,
    root: u32,
}

/// Result of a nearest-triangle query.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Nearest {
    pub triangle: TriangleId,
    /// The point on the triangle (interior or boundary) closest to the query.
    pub point: Vertex,
    /// Euclidean distance from the query point to `point`.
    pub distance: f64,
}

const LEAF_THRESHOLD: usize = 8;

impl Bsp {
    /// Build a fresh BVH over `mesh`'s triangles.
    pub fn build(mesh: &NavMesh) -> Self {
        let n = mesh.triangle_count();
        let triangle_aabbs: Vec<Aabb> = mesh
            .triangles
            .iter()
            .map(|t| {
                let p0 = mesh.vertex(t.vertices[0]);
                let p1 = mesh.vertex(t.vertices[1]);
                let p2 = mesh.vertex(t.vertices[2]);
                Aabb::from_points([p0, p1, p2])
            })
            .collect();

        let mut bsp = Self {
            nodes: Vec::new(),
            triangle_indices: Vec::new(),
            triangle_aabbs,
            root: 0,
        };

        if n == 0 {
            // Single empty leaf so query paths don't need to special-case.
            bsp.nodes.push(BspNode::Leaf {
                aabb: Aabb::EMPTY,
                start: 0,
                len: 0,
            });
            return bsp;
        }

        let mut indices: Vec<u32> = (0..n as u32).collect();
        bsp.root = bsp.build_subtree(mesh, &mut indices);
        bsp
    }

    fn build_subtree(&mut self, mesh: &NavMesh, indices: &mut [u32]) -> u32 {
        let aabb = indices
            .iter()
            .fold(Aabb::EMPTY, |a, &i| a.union(&self.triangle_aabbs[i as usize]));

        if indices.len() <= LEAF_THRESHOLD {
            let start = self.triangle_indices.len() as u32;
            self.triangle_indices.extend_from_slice(indices);
            let len = indices.len() as u32;
            let node_id = self.nodes.len() as u32;
            self.nodes.push(BspNode::Leaf { aabb, start, len });
            return node_id;
        }

        // Split on the longest axis at the centroid median. `axis = 0` means
        // x, `axis = 1` means y.
        let axis: u8 = if aabb.width() >= aabb.height() { 0 } else { 1 };
        let mid = indices.len() / 2;
        indices.select_nth_unstable_by(mid, |&a, &b| {
            let ca = mesh.triangle(TriangleId::new(a)).centroid;
            let cb = mesh.triangle(TriangleId::new(b)).centroid;
            let (x, y) = if axis == 0 { (ca.x, cb.x) } else { (ca.y, cb.y) };
            x.partial_cmp(&y).unwrap_or(std::cmp::Ordering::Equal)
        });

        let (left_slice, right_slice) = indices.split_at_mut(mid);
        let left = self.build_subtree(mesh, left_slice);
        let right = self.build_subtree(mesh, right_slice);

        let node_id = self.nodes.len() as u32;
        self.nodes.push(BspNode::Internal { aabb, left, right });
        node_id
    }

    /// True when the BVH has no triangles.
    pub fn is_empty(&self) -> bool {
        self.triangle_indices.is_empty()
    }

    /// Find the triangle of `mesh` that contains `p`. Returns `None` when
    /// `p` is outside every triangle (which usually means outside the
    /// navmesh; for the "snap to nearest" use case call [`Self::nearest`]
    /// instead).
    pub fn locate(&self, mesh: &NavMesh, p: Vertex) -> Option<TriangleId> {
        self.locate_in(self.root, mesh, p)
    }

    fn locate_in(&self, node: u32, mesh: &NavMesh, p: Vertex) -> Option<TriangleId> {
        match &self.nodes[node as usize] {
            BspNode::Internal { aabb, left, right } => {
                if !aabb.contains(p) {
                    return None;
                }
                self.locate_in(*left, mesh, p)
                    .or_else(|| self.locate_in(*right, mesh, p))
            }
            BspNode::Leaf { aabb, start, len } => {
                if !aabb.contains(p) {
                    return None;
                }
                for i in *start..*start + *len {
                    let tri_idx = self.triangle_indices[i as usize];
                    if !self.triangle_aabbs[tri_idx as usize].contains(p) {
                        continue;
                    }
                    let tri = mesh.triangle(TriangleId::new(tri_idx));
                    let p0 = mesh.vertex(tri.vertices[0]);
                    let p1 = mesh.vertex(tri.vertices[1]);
                    let p2 = mesh.vertex(tri.vertices[2]);
                    if point_in_triangle_ccw(p0, p1, p2, p) {
                        return Some(TriangleId::new(tri_idx));
                    }
                }
                None
            }
        }
    }

    /// Find the triangle whose surface is closest to `p`, together with
    /// the closest point on that surface and the distance.
    ///
    /// If `p` is inside a triangle, `distance == 0.0` and `point == p`.
    pub fn nearest(&self, mesh: &NavMesh, p: Vertex) -> Option<Nearest> {
        if self.is_empty() {
            return None;
        }
        let mut best = Nearest {
            triangle: TriangleId::new(0),
            point: Vertex::ZERO,
            distance: f64::INFINITY,
        };
        let mut have_any = false;
        self.nearest_in(self.root, mesh, p, &mut best, &mut have_any);
        if have_any { Some(best) } else { None }
    }

    fn nearest_in(
        &self,
        node: u32,
        mesh: &NavMesh,
        p: Vertex,
        best: &mut Nearest,
        have_any: &mut bool,
    ) {
        let node_aabb = match &self.nodes[node as usize] {
            BspNode::Internal { aabb, .. } | BspNode::Leaf { aabb, .. } => *aabb,
        };
        // Prune: if even the closest point on this node's AABB is farther
        // than our current best, the whole subtree is hopeless.
        if *have_any {
            let lower_bound = aabb_distance(node_aabb, p);
            if lower_bound >= best.distance {
                return;
            }
        }
        match self.nodes[node as usize] {
            BspNode::Internal { left, right, .. } => {
                // Descend into the closer child first so the better bound
                // can prune the farther child sooner.
                let dl = aabb_distance(
                    match &self.nodes[left as usize] {
                        BspNode::Internal { aabb, .. } | BspNode::Leaf { aabb, .. } => *aabb,
                    },
                    p,
                );
                let dr = aabb_distance(
                    match &self.nodes[right as usize] {
                        BspNode::Internal { aabb, .. } | BspNode::Leaf { aabb, .. } => *aabb,
                    },
                    p,
                );
                let (first, second) = if dl <= dr { (left, right) } else { (right, left) };
                self.nearest_in(first, mesh, p, best, have_any);
                self.nearest_in(second, mesh, p, best, have_any);
            }
            BspNode::Leaf { start, len, .. } => {
                for i in start..start + len {
                    let tri_idx = self.triangle_indices[i as usize];
                    let tri = mesh.triangle(TriangleId::new(tri_idx));
                    let p0 = mesh.vertex(tri.vertices[0]);
                    let p1 = mesh.vertex(tri.vertices[1]);
                    let p2 = mesh.vertex(tri.vertices[2]);
                    let (closest, d) = nearest_point_on_triangle(p0, p1, p2, p);
                    if !*have_any || d < best.distance {
                        *best = Nearest {
                            triangle: TriangleId::new(tri_idx),
                            point: closest,
                            distance: d,
                        };
                        *have_any = true;
                    }
                }
            }
        }
    }
}

// --- Geometry helpers ----------------------------------------------------

/// Point-in-triangle for a CCW triangle. Boundary is inclusive.
#[inline]
fn point_in_triangle_ccw(a: Vertex, b: Vertex, c: Vertex, p: Vertex) -> bool {
    let d1 = geom::orient2d(a, b, p);
    let d2 = geom::orient2d(b, c, p);
    let d3 = geom::orient2d(c, a, p);
    d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0
}

/// Closest point on triangle `(a, b, c)` to `p`, plus the euclidean
/// distance. Works whether `p` is inside or outside the triangle.
fn nearest_point_on_triangle(a: Vertex, b: Vertex, c: Vertex, p: Vertex) -> (Vertex, f64) {
    // Inside test using the non-robust orient. (We're computing a numeric
    // distance — exact sign of zero is fine.)
    let d1 = geom::orient2d(a, b, p);
    let d2 = geom::orient2d(b, c, p);
    let d3 = geom::orient2d(c, a, p);
    let inside = d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0;
    let inside_cw = d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0;
    if inside || inside_cw {
        return (p, 0.0);
    }
    // Outside: closest point is on one of the three edges.
    let candidates = [
        geom::nearest_point_on_segment(a, b, p),
        geom::nearest_point_on_segment(b, c, p),
        geom::nearest_point_on_segment(c, a, p),
    ];
    let mut best = (candidates[0], candidates[0].distance(p));
    for c in &candidates[1..] {
        let d = c.distance(p);
        if d < best.1 {
            best = (*c, d);
        }
    }
    best
}

/// Euclidean distance from point `p` to the closest point on AABB `a`. Zero
/// when `p` is inside (or on the boundary of) `a`.
fn aabb_distance(a: Aabb, p: Vertex) -> f64 {
    let cx = p.x.max(a.min.x).min(a.max.x);
    let cy = p.y.max(a.min.y).min(a.max.y);
    let dx = p.x - cx;
    let dy = p.y - cy;
    (dx * dx + dy * dy).sqrt()
}

// --- Tests ---------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_common::Vertex;
    use rsnav_navmesh::build_from_cdt;
    use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
    use rsnav_triangle::{
        carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot,
    };

    fn build_square_with_hole() -> NavMesh {
        let pts = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 4.0),
            (0.0, 4.0),
            (1.5, 1.5),
            (2.5, 1.5),
            (2.5, 2.5),
            (1.5, 2.5),
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
                PslgSegment { a: 0, b: 1, marker: 1 },
                PslgSegment { a: 1, b: 2, marker: 1 },
                PslgSegment { a: 2, b: 3, marker: 1 },
                PslgSegment { a: 3, b: 0, marker: 1 },
                PslgSegment { a: 4, b: 5, marker: 2 },
                PslgSegment { a: 5, b: 6, marker: 2 },
                PslgSegment { a: 6, b: 7, marker: 2 },
                PslgSegment { a: 7, b: 4, marker: 2 },
            ],
            holes: vec![PslgHole { point: Vertex::new(2.0, 2.0) }],
        };
        form_skeleton(&mut mesh, &pslg, None);
        carve_holes(&mut mesh, &pslg, false);
        build_from_cdt(&mesh)
    }

    #[test]
    fn locate_inside_returns_a_triangle() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        // Center of the bottom strip.
        let hit = bsp.locate(&nav, Vertex::new(2.0, 0.75));
        assert!(hit.is_some());
        let tri = nav.triangle(hit.unwrap());
        // Sanity: the centroid of the returned triangle should be inside
        // the walkable band (y < 1.5 or y > 2.5 or x outside [1.5, 2.5]).
        let c = tri.centroid;
        assert!(
            c.x < 1.5 || c.x > 2.5 || c.y < 1.5 || c.y > 2.5,
            "centroid {:?} fell inside the hole region",
            c
        );
    }

    #[test]
    fn locate_in_hole_returns_none() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        // Dead center of the hole.
        assert_eq!(bsp.locate(&nav, Vertex::new(2.0, 2.0)), None);
    }

    #[test]
    fn locate_outside_mesh_returns_none() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        assert_eq!(bsp.locate(&nav, Vertex::new(-1.0, -1.0)), None);
        assert_eq!(bsp.locate(&nav, Vertex::new(10.0, 10.0)), None);
    }

    #[test]
    fn locate_matches_brute_force() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        let samples = [
            Vertex::new(0.1, 0.1),
            Vertex::new(3.9, 0.1),
            Vertex::new(3.9, 3.9),
            Vertex::new(0.1, 3.9),
            Vertex::new(2.0, 0.5),
            Vertex::new(0.5, 2.0),
            Vertex::new(3.5, 2.0),
            Vertex::new(2.0, 3.5),
            Vertex::new(2.0, 2.0), // in the hole
            Vertex::new(-1.0, 2.0), // outside
        ];
        for p in samples {
            let bsp_hit = bsp.locate(&nav, p);
            let brute_hit = brute_force_locate(&nav, p);
            assert_eq!(bsp_hit, brute_hit, "mismatch at point {:?}", p);
        }
    }

    fn brute_force_locate(nav: &NavMesh, p: Vertex) -> Option<TriangleId> {
        for (i, tri) in nav.triangles.iter().enumerate() {
            let p0 = nav.vertex(tri.vertices[0]);
            let p1 = nav.vertex(tri.vertices[1]);
            let p2 = nav.vertex(tri.vertices[2]);
            if point_in_triangle_ccw(p0, p1, p2, p) {
                return Some(TriangleId::new(i as u32));
            }
        }
        None
    }

    #[test]
    fn nearest_for_point_inside_is_self_zero() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        let p = Vertex::new(0.5, 0.5);
        let n = bsp.nearest(&nav, p).unwrap();
        assert_eq!(n.distance, 0.0);
        assert_eq!(n.point, p);
        assert_eq!(Some(n.triangle), bsp.locate(&nav, p));
    }

    #[test]
    fn nearest_outside_snaps_to_boundary() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        // (-1.0, 0.5) is to the left of the mesh; nearest should be on
        // the (0,0)-(0,4) edge at y=0.5.
        let p = Vertex::new(-1.0, 0.5);
        let n = bsp.nearest(&nav, p).unwrap();
        assert!((n.point.x - 0.0).abs() < 1e-9, "x snapped to {}", n.point.x);
        assert!((n.point.y - 0.5).abs() < 1e-9, "y snapped to {}", n.point.y);
        assert!((n.distance - 1.0).abs() < 1e-9);
    }

    #[test]
    fn nearest_in_hole_snaps_to_inner_boundary() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        // Dead center of the hole. The hole is bounded by inner ring
        // (1.5,1.5)–(2.5,1.5)–(2.5,2.5)–(1.5,2.5). Nearest point on the
        // navmesh's boundary should be on one of those four edges, at
        // distance 0.5.
        let p = Vertex::new(2.0, 2.0);
        let n = bsp.nearest(&nav, p).unwrap();
        assert!(
            (n.distance - 0.5).abs() < 1e-9,
            "distance was {}, expected 0.5",
            n.distance
        );
        // Closest point should be on one of the four inner-ring midpoints.
        let close = [
            Vertex::new(2.0, 1.5),
            Vertex::new(2.5, 2.0),
            Vertex::new(2.0, 2.5),
            Vertex::new(1.5, 2.0),
        ];
        let near_any = close.iter().any(|c| c.approx_eq(n.point, 1e-9));
        assert!(near_any, "nearest point {:?} not on inner ring", n.point);
    }

    #[test]
    fn nearest_matches_brute_force() {
        let nav = build_square_with_hole();
        let bsp = Bsp::build(&nav);
        let samples = [
            Vertex::new(-2.0, -2.0),
            Vertex::new(2.0, -1.0),
            Vertex::new(5.0, 2.0),
            Vertex::new(2.0, 5.0),
            Vertex::new(2.0, 2.0), // hole
            Vertex::new(2.1, 1.9), // near hole
            Vertex::new(0.5, 0.5), // inside mesh
        ];
        for p in samples {
            let bsp_n = bsp.nearest(&nav, p).unwrap();
            let brute_n = brute_force_nearest(&nav, p).unwrap();
            // Distances must match; the *triangle* might differ when two
            // triangles share a vertex that's equidistant, but the closest
            // distance should be identical.
            assert!(
                (bsp_n.distance - brute_n.distance).abs() < 1e-9,
                "distance mismatch at {:?}: bsp={} brute={}",
                p, bsp_n.distance, brute_n.distance
            );
        }
    }

    fn brute_force_nearest(nav: &NavMesh, p: Vertex) -> Option<Nearest> {
        let mut best: Option<Nearest> = None;
        for (i, tri) in nav.triangles.iter().enumerate() {
            let p0 = nav.vertex(tri.vertices[0]);
            let p1 = nav.vertex(tri.vertices[1]);
            let p2 = nav.vertex(tri.vertices[2]);
            let (close, d) = nearest_point_on_triangle(p0, p1, p2, p);
            if best.map_or(true, |b| d < b.distance) {
                best = Some(Nearest {
                    triangle: TriangleId::new(i as u32),
                    point: close,
                    distance: d,
                });
            }
        }
        best
    }

    #[test]
    fn build_with_zero_triangles_is_safe() {
        let nav = NavMesh {
            vertices: Vec::new(),
            triangles: Vec::new(),
            aabb: rsnav_common::Aabb::EMPTY,
            region_count: 0,
        };
        let bsp = Bsp::build(&nav);
        assert!(bsp.is_empty());
        assert_eq!(bsp.locate(&nav, Vertex::new(0.0, 0.0)), None);
        assert!(bsp.nearest(&nav, Vertex::new(0.0, 0.0)).is_none());
    }
}
