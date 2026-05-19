//! Visibility region (visibility polygon) from a point inside the navmesh.
//!
//! Computed by casting `samples` rays at uniform angular intervals around
//! `source` and recording each ray's first wall hit (or `max_radius` if
//! the ray hits nothing). The result is approximate — angular resolution
//! limits how sharply wall corners are rendered — but for visualization
//! at 180–360 samples it's indistinguishable from an exact polygon at
//! typical zoom levels and runs trivially per frame.
//!
//! ## Why it doesn't need triangulation to render
//!
//! Visibility from a point `P` is always **star-shaped relative to `P`**
//! — every point inside is visible to `P` by definition. So each
//! consecutive pair of boundary points `(b[i], b[i+1])` forms a triangle
//! `(P, b[i], b[i+1])` that's entirely inside the region. Just draw the
//! `N` such triangles as a fan; their union is exactly the polygon. No
//! ear-clipping, no Delaunay, no `convex_polygon` failure on concavity.

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_navmesh::NavMesh;

use crate::los::{line_of_sight, LineOfSightResult};

#[derive(Clone, Debug)]
pub struct VisibilityRegion {
    pub source: Vertex,
    /// Boundary points, in CCW angular order around `source`. Connect
    /// consecutive points (last wraps back to the first) to form the
    /// polygon. Always contains exactly `samples` points.
    pub boundary: Vec<Vertex>,
}

/// Sample the visibility region of `source` within `max_radius`, using
/// `samples` rays evenly spaced in angle.
///
/// Returns `None` when `source` is not inside any navmesh triangle —
/// callers wanting "snap to nearest mesh point" semantics should pre-
/// snap via [`rsnav_bsp::Bsp::nearest`].
///
/// `samples` is clamped to a minimum of 8. 180 (=2° per sample) is a
/// good default for hover rendering at typical zoom; bump higher for
/// tighter corners or zoomed-in screenshots.
pub fn visibility_region(
    nav: &NavMesh,
    bsp: &Bsp,
    source: Vertex,
    max_radius: f64,
    samples: usize,
) -> Option<VisibilityRegion> {
    let src_tri = bsp.locate(nav, source)?;
    let n = samples.max(8);
    let mut boundary = Vec::with_capacity(n);
    let tau = std::f64::consts::TAU;
    for i in 0..n {
        let theta = tau * (i as f64) / (n as f64);
        let endpoint = Vertex::new(
            source.x + max_radius * theta.cos(),
            source.y + max_radius * theta.sin(),
        );
        let hit = match line_of_sight(nav, src_tri, source, endpoint) {
            LineOfSightResult::Clear => endpoint,
            LineOfSightResult::Blocked { point } => point,
            // Unreachable in practice — we just located src_tri.
            LineOfSightResult::SourceOutsideMesh => endpoint,
        };
        boundary.push(hit);
    }
    Some(VisibilityRegion { source, boundary })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_navmesh::build_from_cdt;
    use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
    use rsnav_triangle::{
        carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot,
    };

    fn build_open_square() -> (NavMesh, Bsp) {
        // 10×10 open square, no holes.
        let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let mut cdt = CdtMesh::new();
        let mut pslg = Pslg::new();
        for (x, y) in pts {
            cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
            pslg.segments.push(PslgSegment { a, b, marker: 1 });
        }
        delaunay(&mut cdt, DivConqOptions::default());
        form_skeleton(&mut cdt, &pslg, None).unwrap();
        let nav = build_from_cdt(&cdt);
        let bsp = Bsp::build(&nav);
        (nav, bsp)
    }

    fn build_square_with_hole() -> (NavMesh, Bsp) {
        let pts = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (4.0, 4.0),
            (6.0, 4.0),
            (6.0, 6.0),
            (4.0, 6.0),
        ];
        let mut cdt = CdtMesh::new();
        let mut pslg = Pslg::new();
        for (x, y) in pts {
            cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
            pslg.segments.push(PslgSegment { a, b, marker: 1 });
        }
        for &(a, b) in &[(4, 5), (5, 6), (6, 7), (7, 4)] {
            pslg.segments.push(PslgSegment { a, b, marker: 2 });
        }
        pslg.holes.push(PslgHole { point: Vertex::new(5.0, 5.0) });
        delaunay(&mut cdt, DivConqOptions::default());
        form_skeleton(&mut cdt, &pslg, None).unwrap();
        carve_holes(&mut cdt, &pslg, false);
        let nav = build_from_cdt(&cdt);
        let bsp = Bsp::build(&nav);
        (nav, bsp)
    }

    #[test]
    fn outside_mesh_returns_none() {
        let (nav, bsp) = build_open_square();
        assert!(visibility_region(&nav, &bsp, Vertex::new(-1.0, -1.0), 5.0, 64).is_none());
    }

    #[test]
    fn open_room_visibility_is_full_circle() {
        let (nav, bsp) = build_open_square();
        // Source at the center; max_radius small enough to stay inside
        // the room. Every ray should reach `max_radius` (no walls hit).
        let vr = visibility_region(&nav, &bsp, Vertex::new(5.0, 5.0), 2.0, 64).unwrap();
        for p in &vr.boundary {
            let d = vr.source.distance(*p);
            assert!(
                (d - 2.0).abs() < 1e-9,
                "expected radius 2.0, got {} at {:?}",
                d, p
            );
        }
    }

    #[test]
    fn ray_to_wall_clamps_at_wall() {
        let (nav, bsp) = build_open_square();
        // Source near the left wall; rays going west should clamp at x=0.
        let vr = visibility_region(&nav, &bsp, Vertex::new(2.0, 5.0), 10.0, 64).unwrap();
        // Find the ray closest to "due west" (angle π).
        let west_ix = vr
            .boundary
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let dx = p.x - vr.source.x;
                let dy = p.y - vr.source.y;
                let angle = dy.atan2(dx); // (-π, π]
                let west_err = (angle - std::f64::consts::PI).abs();
                (i, west_err)
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap()
            .0;
        let p = vr.boundary[west_ix];
        // Ray went west from (2, 5); first wall hit is x=0. y stays near 5.
        assert!(
            (p.x - 0.0).abs() < 0.2,
            "westward ray didn't land on x=0 wall: {:?}",
            p
        );
        assert!((p.y - 5.0).abs() < 0.2);
    }

    #[test]
    fn hole_occludes_far_side_of_room() {
        let (nav, bsp) = build_square_with_hole() ;
        // Source at (1, 5); the central hole occupies x ∈ [4, 6].
        // A ray going due east should NOT make it past the hole's near
        // wall (x = 4).
        let vr = visibility_region(&nav, &bsp, Vertex::new(1.0, 5.0), 20.0, 128).unwrap();
        // Find the ray closest to "due east" (angle 0).
        let east_ix = vr
            .boundary
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let dx = p.x - vr.source.x;
                let dy = p.y - vr.source.y;
                let angle = dy.atan2(dx);
                (i, angle.abs())
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap()
            .0;
        let p = vr.boundary[east_ix];
        assert!(
            p.x < 4.5 && (p.x - 4.0).abs() < 0.5,
            "eastward ray should stop at the hole's left wall (x=4), got {:?}",
            p
        );
    }
}
