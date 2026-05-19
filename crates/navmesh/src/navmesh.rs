//! In-memory navmesh data structure.

use rsnav_common::{Aabb, Triangle, TriangleId, Vertex, VertexId};

/// One triangle in the navmesh, packed with its derived metadata.
///
/// `vertices` is in CCW order (positive signed area).
/// `neighbors[i]` is the triangle sharing the edge opposite `vertices[i]`,
/// or [`TriangleId::INVALID`] if that edge is on the mesh boundary.
/// `edge_markers[i]` is the constraint marker on that same edge — `0` means
/// the edge is interior / unconstrained, any non-zero value means it came
/// from a PSLG segment (and the value is the segment's marker).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NavTriangle {
    pub vertices: [VertexId; 3],
    pub neighbors: [TriangleId; 3],
    pub edge_markers: [i32; 3],
    pub area: f64,
    pub centroid: Vertex,
    /// Connected-component ID. Two triangles share a region iff there is a
    /// path between them that never crosses a constrained edge (i.e. they
    /// are both walkable and not separated by a wall).
    pub region: u32,
}

impl NavTriangle {
    /// True if edge `i` (the edge opposite `vertices[i]`) is constrained.
    #[inline]
    pub fn is_edge_constrained(&self, i: usize) -> bool {
        self.edge_markers[i] != 0
    }

    /// True if edge `i` is on the mesh boundary (no neighbor).
    #[inline]
    pub fn is_edge_boundary(&self, i: usize) -> bool {
        !self.neighbors[i].is_valid()
    }

    /// Returns the two endpoints of edge `i` in CCW order around the
    /// triangle: `(vertices[(i+1)%3], vertices[(i+2)%3])`.
    #[inline]
    pub fn edge_vertices(&self, i: usize) -> (VertexId, VertexId) {
        (self.vertices[(i + 1) % 3], self.vertices[(i + 2) % 3])
    }
}

/// A loaded or freshly-built navmesh.
///
/// `vertices` and `triangles` are flat parallel arrays indexed by
/// [`VertexId`] and [`TriangleId`]. The order is the order produced by the
/// CDT builder; serializing and reloading round-trips it exactly.
#[derive(Clone, Debug)]
pub struct NavMesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<NavTriangle>,
    pub aabb: Aabb,
    /// Number of distinct regions; equivalently `1 + max(region)`.
    pub region_count: u32,
}

impl NavMesh {
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Read a vertex position by ID. **Panics** if `id` is out of
    /// range — most commonly because the ID was issued by a different
    /// mesh. NavMesh IDs are not portable across instances; pass IDs
    /// only back to the same NavMesh that produced them.
    #[inline]
    pub fn vertex(&self, id: VertexId) -> Vertex {
        self.vertices[id.index()]
    }

    /// Read a triangle by ID. **Panics** if `id` is out of range. Same
    /// cross-mesh caveat as [`vertex`](Self::vertex).
    #[inline]
    pub fn triangle(&self, id: TriangleId) -> &NavTriangle {
        &self.triangles[id.index()]
    }

    /// `true` if the two triangles are in the same reachability region.
    /// Cheap pre-check before running A*.
    #[inline]
    pub fn reachable(&self, a: TriangleId, b: TriangleId) -> bool {
        self.triangle(a).region == self.triangle(b).region
    }

    /// Convenience: convert a [`NavTriangle`] to the geometry-only
    /// [`Triangle`] from rsnav-common, for use with shared predicates.
    pub fn as_triangle(&self, id: TriangleId) -> Triangle {
        Triangle::new(
            self.triangle(id).vertices[0],
            self.triangle(id).vertices[1],
            self.triangle(id).vertices[2],
        )
    }

    // --- Region accessors -------------------------------------------------
    //
    // Every triangle carries a `region` (connected component under the
    // "non-wall neighbor" relation). These helpers expose per-region
    // views without the caller having to scan `triangles` by hand.
    // All four treat an out-of-range / empty region id gracefully:
    // the iterator is empty, `region_area` is `0.0`, and the centroid /
    // bounds are `None`.

    /// Iterate the triangles belonging to one connected region.
    pub fn region_triangles(&self, region: u32) -> impl Iterator<Item = TriangleId> + '_ {
        self.triangles
            .iter()
            .enumerate()
            .filter(move |(_, t)| t.region == region)
            .map(|(i, _)| TriangleId::new(i as u32))
    }

    /// Total walkable area of one region — the sum of its triangle
    /// areas. `0.0` for a region id with no triangles.
    pub fn region_area(&self, region: u32) -> f64 {
        self.triangles
            .iter()
            .filter(|t| t.region == region)
            .map(|t| t.area)
            .sum()
    }

    /// Area-weighted centroid of one region — a representative "where
    /// is this region" point. `None` if the region has no triangles.
    ///
    /// Note this is the centroid of the region's *area*, which for a
    /// non-convex region need not itself be inside the region; use
    /// [`region_triangles`](Self::region_triangles) +
    /// [`NavMesh::random_point_in_region`] if you need a guaranteed
    /// interior point.
    pub fn region_centroid(&self, region: u32) -> Option<Vertex> {
        let mut acc = Vertex::ZERO;
        let mut total = 0.0;
        for t in self.triangles.iter().filter(|t| t.region == region) {
            acc = acc + t.centroid * t.area;
            total += t.area;
        }
        (total > 0.0).then(|| acc * (1.0 / total))
    }

    /// Axis-aligned bounds of one region. `None` if the region has no
    /// triangles.
    pub fn region_bounds(&self, region: u32) -> Option<Aabb> {
        let mut aabb = Aabb::EMPTY;
        let mut any = false;
        for t in self.triangles.iter().filter(|t| t.region == region) {
            for &v in &t.vertices {
                aabb.extend(self.vertex(v));
            }
            any = true;
        }
        any.then_some(aabb)
    }

    // --- Random point sampling -------------------------------------------

    /// Pick a uniformly area-distributed random point inside the
    /// walkable area.
    ///
    /// `rng` must yield uniform `f64` in `[0, 1)`. Each call consumes
    /// three values: one to choose a triangle (weighted by area, so the
    /// result is uniform over surface area, not over triangles) and two
    /// for a uniform barycentric point inside it. Returns `None` only
    /// when the mesh has no triangles.
    ///
    /// `O(n)` in the triangle count per call (a linear walk over the
    /// area CDF). Fine for spawn placement / enemy seeding; if you are
    /// sampling in a hot loop over a very large mesh, precompute your
    /// own cumulative-area table instead.
    ///
    /// ```
    /// # use rsnav_navmesh::NavMesh;
    /// # fn demo(nav: &NavMesh) {
    /// // A splitmix64-style closure works fine as the rng source.
    /// let mut state = 0x1234_5678_u64;
    /// let mut unit = || {
    ///     state = state.wrapping_add(0x9E3779B97F4A7C15);
    ///     let mut z = state;
    ///     z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    ///     z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    ///     (((z ^ (z >> 31)) >> 11) as f64) / ((1u64 << 53) as f64)
    /// };
    /// let spawn = nav.random_point(&mut unit);
    /// # let _ = spawn;
    /// # }
    /// ```
    pub fn random_point(&self, rng: impl FnMut() -> f64) -> Option<Vertex> {
        self.random_point_filtered(None, rng)
    }

    /// Like [`random_point`](Self::random_point) but restricted to one
    /// connected region — e.g. to spawn an enemy in the same room the
    /// player is in, or deliberately in a different one. `None` if the
    /// region has no triangles.
    pub fn random_point_in_region(
        &self,
        region: u32,
        rng: impl FnMut() -> f64,
    ) -> Option<Vertex> {
        self.random_point_filtered(Some(region), rng)
    }

    fn random_point_filtered(
        &self,
        region: Option<u32>,
        mut rng: impl FnMut() -> f64,
    ) -> Option<Vertex> {
        let in_scope = |t: &NavTriangle| region.map_or(true, |r| t.region == r);

        let total: f64 = self
            .triangles
            .iter()
            .filter(|t| in_scope(t))
            .map(|t| t.area)
            .sum();
        if total <= 0.0 {
            return None;
        }

        // Area-weighted triangle pick: walk the CDF. `chosen` is updated
        // every step so a floating-point overshoot still lands on the
        // last in-scope triangle rather than falling through to None.
        let mut pick = rng().clamp(0.0, 1.0) * total;
        let mut chosen: Option<&NavTriangle> = None;
        for t in self.triangles.iter().filter(|t| in_scope(t)) {
            chosen = Some(t);
            if pick < t.area {
                break;
            }
            pick -= t.area;
        }
        let t = chosen?;

        // Uniform barycentric point inside the chosen triangle.
        let mut r1 = rng().clamp(0.0, 1.0);
        let mut r2 = rng().clamp(0.0, 1.0);
        if r1 + r2 > 1.0 {
            r1 = 1.0 - r1;
            r2 = 1.0 - r2;
        }
        let a = self.vertex(t.vertices[0]);
        let b = self.vertex(t.vertices[1]);
        let c = self.vertex(t.vertices[2]);
        Some(a + (b - a) * r1 + (c - a) * r2)
    }
}

#[cfg(test)]
mod tests {
    use crate::build_from_cdt;
    use rsnav_common::Vertex;
    use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
    use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

    use super::NavMesh;

    /// A 10×4 rectangle split down the middle (x = 5) by a constrained
    /// wall — two regions of area 20 each.
    fn divided_rectangle() -> NavMesh {
        let pts = [
            (0.0, 0.0),  // 0
            (10.0, 0.0), // 1
            (10.0, 4.0), // 2
            (0.0, 4.0),  // 3
            (5.0, 0.0),  // 4 — wall bottom
            (5.0, 4.0),  // 5 — wall top
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
                PslgSegment { a: 0, b: 4, marker: 1 },
                PslgSegment { a: 4, b: 1, marker: 1 },
                PslgSegment { a: 1, b: 2, marker: 1 },
                PslgSegment { a: 2, b: 5, marker: 1 },
                PslgSegment { a: 5, b: 3, marker: 1 },
                PslgSegment { a: 3, b: 0, marker: 1 },
                PslgSegment { a: 4, b: 5, marker: 99 }, // wall
            ],
            holes: Vec::new(),
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        build_from_cdt(&mesh)
    }

    #[test]
    fn region_triangles_partition_the_mesh() {
        let nav = divided_rectangle();
        assert_eq!(nav.region_count, 2);
        let n0 = nav.region_triangles(0).count();
        let n1 = nav.region_triangles(1).count();
        assert_eq!(n0 + n1, nav.triangle_count());
        assert!(n0 > 0 && n1 > 0);
        // Every triangle id the iterator yields is actually in that region.
        for id in nav.region_triangles(0) {
            assert_eq!(nav.triangle(id).region, 0);
        }
    }

    #[test]
    fn region_area_splits_evenly() {
        let nav = divided_rectangle();
        let a0 = nav.region_area(0);
        let a1 = nav.region_area(1);
        assert!((a0 - 20.0).abs() < 1e-9, "region 0 area {a0}");
        assert!((a1 - 20.0).abs() < 1e-9, "region 1 area {a1}");
        assert!((a0 + a1 - 40.0).abs() < 1e-9);
    }

    #[test]
    fn region_centroid_and_bounds() {
        let nav = divided_rectangle();
        let c0 = nav.region_centroid(0).expect("region 0 non-empty");
        let c1 = nav.region_centroid(1).expect("region 1 non-empty");
        // One centroid sits in the left half (x < 5), the other right.
        assert!((c0.x < 5.0) ^ (c1.x < 5.0), "centroids both on one side");
        for c in [c0, c1] {
            assert!(c.y > 0.0 && c.y < 4.0);
        }
        let b0 = nav.region_bounds(0).expect("region 0 bounds");
        let b1 = nav.region_bounds(1).expect("region 1 bounds");
        let full = b0.union(&b1);
        assert_eq!(full.min, Vertex::new(0.0, 0.0));
        assert_eq!(full.max, Vertex::new(10.0, 4.0));
    }

    #[test]
    fn empty_region_id_is_handled() {
        let nav = divided_rectangle();
        assert_eq!(nav.region_triangles(99).count(), 0);
        assert_eq!(nav.region_area(99), 0.0);
        assert!(nav.region_centroid(99).is_none());
        assert!(nav.region_bounds(99).is_none());
    }

    // --- random point sampling -------------------------------------------

    /// splitmix64 → uniform f64 in [0, 1).
    struct TestRng(u64);
    impl TestRng {
        fn unit(&mut self) -> f64 {
            self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
            (((z ^ (z >> 31)) >> 11) as f64) / ((1u64 << 53) as f64)
        }
    }

    /// True if `p` lies inside any triangle of the mesh (small epsilon
    /// so points generated exactly on an edge still count).
    fn point_in_mesh(nav: &NavMesh, p: Vertex) -> Option<u32> {
        const EPS: f64 = 1e-7;
        for t in &nav.triangles {
            let a = nav.vertex(t.vertices[0]);
            let b = nav.vertex(t.vertices[1]);
            let c = nav.vertex(t.vertices[2]);
            let d1 = (b - a).cross(p - a);
            let d2 = (c - b).cross(p - b);
            let d3 = (a - c).cross(p - c);
            let has_neg = d1 < -EPS || d2 < -EPS || d3 < -EPS;
            let has_pos = d1 > EPS || d2 > EPS || d3 > EPS;
            if !(has_neg && has_pos) {
                return Some(t.region);
            }
        }
        None
    }

    #[test]
    fn random_point_lands_inside_the_mesh() {
        let nav = divided_rectangle();
        let mut rng = TestRng(0xABCD_1234);
        for _ in 0..400 {
            let p = nav.random_point(|| rng.unit()).expect("non-empty mesh");
            assert!(point_in_mesh(&nav, p).is_some(), "sample {p:?} off-mesh");
        }
    }

    #[test]
    fn random_point_in_region_stays_in_region() {
        let nav = divided_rectangle();
        let mut rng = TestRng(0x5555_AAAA);
        for region in 0..2u32 {
            for _ in 0..200 {
                let p = nav
                    .random_point_in_region(region, || rng.unit())
                    .expect("non-empty region");
                assert_eq!(
                    point_in_mesh(&nav, p),
                    Some(region),
                    "sample {p:?} not in region {region}",
                );
            }
        }
    }

    #[test]
    fn random_point_is_roughly_area_weighted() {
        // The two regions have equal area (20 each); whole-mesh sampling
        // should split close to 50/50. A broken weight (e.g. uniform
        // over triangles) would still be near 50/50 here since the two
        // regions have similar triangle counts — so this is only a
        // coarse smoke check that neither region is starved.
        let nav = divided_rectangle();
        let mut rng = TestRng(0x0F0F_0F0F);
        let mut counts = [0u32; 2];
        let n = 2_000;
        for _ in 0..n {
            let p = nav.random_point(|| rng.unit()).unwrap();
            if let Some(r) = point_in_mesh(&nav, p) {
                counts[r as usize] += 1;
            }
        }
        for (r, &c) in counts.iter().enumerate() {
            assert!(
                c as f64 > n as f64 * 0.30,
                "region {r} starved: {c}/{n}",
            );
        }
    }

    #[test]
    fn random_point_in_empty_region_is_none() {
        let nav = divided_rectangle();
        let mut rng = TestRng(1);
        assert!(nav.random_point_in_region(99, || rng.unit()).is_none());
    }
}
