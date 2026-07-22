//! Contour offsetting for agent-radius erosion.
//!
//! The inset pipeline shrinks walkable perimeters and grows holes by the
//! agent radius, then lets planarization + winding classification clean
//! up whatever the offset produced. The types here describe that raw
//! "offset soup":
//!
//! - [`SoupContour`] is one closed ring of the soup. It is allowed to
//!   self-intersect and to contain flipped (orientation-reversed) lobes
//!   — downstream winding math cancels those out, so the offset stage
//!   never needs to detect or repair them.
//! - [`OffsetOptions`] controls join behavior at reflex corners.
//!
//! Orientation is the winding contract: a CCW ring contributes `+1` to
//! the winding number of points it encloses, a CW ring `-1`. Perimeters
//! are normalized CCW and holes CW before offsetting, so "walkable" is
//! exactly the region with winding `>= 1`.
//!
//! The offset primitive itself ([`offset_ring_left`]) pushes every edge
//! of a ring to its left by `delta` — erosion for both ring kinds once
//! they carry their natural orientation.

use crate::polygon::{Polygon, Winding};
use crate::Vertex;

/// Options for [`offset_ring_left`].
#[derive(Copy, Clone, Debug)]
pub struct OffsetOptions {
    /// Miter limit as a multiple of the offset distance, in the same
    /// convention as SVG/Clipper: a reflex corner's miter point may sit
    /// at most `miter_limit * delta` from the original vertex; sharper
    /// corners fall back to a bevel chord. Must be `>= 1.0`.
    pub miter_limit: f64,
}

impl Default for OffsetOptions {
    fn default() -> Self {
        Self { miter_limit: 2.0 }
    }
}

/// One closed ring of the offset soup: `points` in order, implicitly
/// closed (no repeated last point), free to self-intersect. `marker` is
/// carried onto every constraint segment derived from this ring.
#[derive(Clone, Debug)]
pub struct SoupContour {
    pub points: Vec<Vertex>,
    pub marker: i32,
}

/// Offset every edge of `ring` to its **left** by `delta`, joining
/// adjacent edges, and return the (possibly self-intersecting) result.
///
/// With the natural orientations — perimeter CCW, hole CW — "left" is
/// always into the walkable region, so this one primitive erodes both
/// ring kinds. `delta = 0.0` returns the ring unchanged.
///
/// Join policy: corners that bend **away** from the offset side get a
/// miter vertex, clamped by [`OffsetOptions::miter_limit`] with a bevel
/// chord as the fallback; corners that bend into the offset side are
/// connected naively — the little backtracking lobe that produces is
/// resolved exactly by planarize + winding downstream. No arcs.
///
/// Returns `None` when the ring is degenerate: fewer than 3 distinct
/// points after consecutive-duplicate removal, or zero area
/// ([`Winding::Degenerate`]). Callers that pre-filter degenerate rings
/// never see `None`.
pub fn offset_ring_left(
    ring: &Polygon,
    delta: f64,
    marker: i32,
    opts: &OffsetOptions,
) -> Option<SoupContour> {
    // Drop consecutive duplicates (closing wrap included).
    let mut pts: Vec<Vertex> = Vec::with_capacity(ring.vertices.len());
    for &p in &ring.vertices {
        if pts.last() != Some(&p) {
            pts.push(p);
        }
    }
    while pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    if pts.len() < 3 {
        return None;
    }
    let poly = Polygon::from_vertices(pts.iter().copied());
    if poly.winding() == Winding::Degenerate {
        return None;
    }
    if delta == 0.0 {
        return Some(SoupContour { points: pts, marker });
    }

    // Per-edge left normals. Zero-length edges were removed above, but a
    // collinear spike can still yield a zero direction after normalize —
    // treat that edge as inheriting its predecessor's normal by skipping
    // the join (handled below via normalize_or_zero).
    let n = pts.len();
    let mut normals = Vec::with_capacity(n);
    for i in 0..n {
        let d = (pts[(i + 1) % n] - pts[i]).normalize_or_zero();
        normals.push(Vertex::new(-d.y, d.x));
    }

    // `dot >= miter_dot` accepts the miter: the miter length is
    // delta / cos(theta/2) with cos(theta/2) = sqrt((1 + n1.n2) / 2), so
    // length <= limit * delta  <=>  n1.n2 >= 2 / limit^2 - 1.
    let limit = opts.miter_limit.max(1.0);
    let miter_dot = 2.0 / (limit * limit) - 1.0;

    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let prev = normals[(i + n - 1) % n];
        let next = normals[i];
        let v = pts[i];
        let dot = prev.dot(next);
        // Turn direction at v: incoming direction is prev rotated right,
        // cross of incoming x outgoing == cross(prev, next) as well.
        let turn = prev.cross(next);
        if turn < 0.0 {
            // Bends away from the offset side (reflex for a left
            // offset): miter or bevel.
            if dot >= miter_dot {
                let bisector = (prev + next).normalize_or_zero();
                let cos_half = ((1.0 + dot) * 0.5).max(0.0).sqrt();
                if cos_half > 0.0 && bisector != Vertex::ZERO {
                    out.push(v + bisector * (delta / cos_half));
                    continue;
                }
            }
            // Bevel: both offset endpoints, connected by a chord.
            out.push(v + prev * delta);
            out.push(v + next * delta);
        } else if turn > 0.0 {
            // Bends into the offset side: naive connection. The two
            // offset edge endpoints around v cross each other; keep both
            // and let planarize + winding cancel the backtrack lobe.
            out.push(v + prev * delta);
            out.push(v + next * delta);
        } else {
            // Exactly collinear (or a degenerate normal): single point.
            out.push(v + next * delta);
        }
    }

    Some(SoupContour { points: out, marker })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::signed_area2;

    fn ring(pts: &[(f64, f64)]) -> Polygon {
        Polygon::from_vertices(pts.iter().map(|&(x, y)| Vertex::new(x, y)))
    }

    fn soup_area2(c: &SoupContour) -> f64 {
        let p = &c.points;
        let mut acc = 0.0;
        for i in 1..p.len() - 1 {
            acc += signed_area2(p[0], p[i], p[i + 1]);
        }
        acc
    }

    #[test]
    fn square_shrinks_to_naive_inner_soup() {
        // CCW square 0..10; left offset by 2. Convex corners emit BOTH
        // adjacent edge-offset endpoints (naive connection), so the soup
        // has 8 points; the true inner square [2,8]^2 emerges only after
        // planarize + winding culls the corner backtrack lobes.
        let sq = ring(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]);
        let out = offset_ring_left(&sq, 2.0, 1, &OffsetOptions::default()).unwrap();
        assert_eq!(out.points.len(), 8);
        for p in &out.points {
            let on_grid = |c: f64| {
                (c - 0.0).abs() < 1e-12
                    || (c - 2.0).abs() < 1e-12
                    || (c - 8.0).abs() < 1e-12
                    || (c - 10.0).abs() < 1e-12
            };
            assert!(on_grid(p.x) && on_grid(p.y), "unexpected point: {p:?}");
        }
        // The four straight offset edges must appear: consecutive point
        // pairs at x==2, x==8, y==2, y==8 respectively.
        let n = out.points.len();
        let mut edge_lines = 0;
        for i in 0..n {
            let (a, b) = (out.points[i], out.points[(i + 1) % n]);
            if (a.x == b.x && (a.x == 2.0 || a.x == 8.0) && (a.y - b.y).abs() == 10.0)
                || (a.y == b.y && (a.y == 2.0 || a.y == 8.0) && (a.x - b.x).abs() == 10.0)
            {
                edge_lines += 1;
            }
        }
        assert_eq!(edge_lines, 4, "expected the 4 straight offset edges");
    }

    #[test]
    fn cw_hole_grows_outward() {
        // CW square (a hole): left offset moves outward, area magnitude grows.
        let sq = ring(&[(0.0, 10.0), (10.0, 10.0), (10.0, 0.0), (0.0, 0.0)]);
        let out = offset_ring_left(&sq, 2.0, 1, &OffsetOptions::default()).unwrap();
        // Orientation preserved (still CW => negative area) and larger.
        let a2 = soup_area2(&out);
        assert!(a2 < 0.0, "hole must stay CW, area2 = {a2}");
        assert!(a2.abs() > 100.0 * 2.0, "hole must grow, area2 = {a2}");
    }

    #[test]
    fn reflex_corner_miter_within_limit() {
        // CCW L-shape; the reflex corner at (5,5) gets a miter on the
        // inside of the L, within miter_limit * delta of the corner.
        let l = ring(&[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 5.0),
            (5.0, 5.0),
            (5.0, 10.0),
            (0.0, 10.0),
        ]);
        let delta = 1.0;
        let out = offset_ring_left(&l, delta, 1, &OffsetOptions::default()).unwrap();
        let corner = Vertex::new(5.0, 5.0);
        let near: Vec<_> = out
            .points
            .iter()
            .filter(|p| p.distance(corner) <= 2.0 * delta + 1e-9)
            .collect();
        // 90-degree miter: single join vertex at distance delta * sqrt(2),
        // pushed INTO the material (away from the notch at x>5, y>5).
        assert_eq!(near.len(), 1, "expected one miter vertex near the corner");
        assert!(near[0].approx_eq(Vertex::new(4.0, 4.0), 1e-9), "got {:?}", near[0]);
    }

    #[test]
    fn sharp_spike_falls_back_to_bevel() {
        // A ~15-degree spike: the miter would be far past the limit, so
        // the join must bevel — no vertex farther than limit * delta
        // (+ slack) from its source corner.
        let spike = ring(&[
            (0.0, 0.0),
            (20.0, 0.0),
            (20.0, 3.0),
            (0.0, 3.0),
            (18.0, 1.5), // deep reflex spike pointing right
        ]);
        let delta = 0.5;
        let opts = OffsetOptions { miter_limit: 2.0 };
        let out = offset_ring_left(&spike, delta, 1, &opts).unwrap();
        // Every emitted point is corner-derived: plain edge-offset
        // endpoints and bevel points sit at exactly delta from their
        // corner, accepted miters at <= miter_limit * delta. So no point
        // may be farther than miter_limit * delta from every corner.
        let max_r = opts.miter_limit * delta + 1e-9;
        for p in &out.points {
            let nearest = spike
                .vertices
                .iter()
                .map(|v| v.distance(*p))
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest <= max_r,
                "vertex {p:?} exceeds the miter limit (nearest corner {nearest})"
            );
        }
    }

    #[test]
    fn delta_zero_is_identity() {
        let sq = ring(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]);
        let out = offset_ring_left(&sq, 0.0, 9, &OffsetOptions::default()).unwrap();
        assert_eq!(out.points, sq.vertices);
        assert_eq!(out.marker, 9);
    }

    #[test]
    fn narrow_rect_over_inset_flips_not_panics() {
        // 2-wide CCW rectangle, inset by 1.5: the offset edges cross;
        // result is a valid (self-intersecting) soup, no NaN/inf.
        let r = ring(&[(0.0, 0.0), (20.0, 0.0), (20.0, 2.0), (0.0, 2.0)]);
        let out = offset_ring_left(&r, 1.5, 1, &OffsetOptions::default()).unwrap();
        for p in &out.points {
            assert!(p.x.is_finite() && p.y.is_finite(), "non-finite point {p:?}");
        }
        assert!(out.points.len() >= 3);
    }

    #[test]
    fn degenerate_rings_return_none() {
        let two = ring(&[(0.0, 0.0), (1.0, 0.0)]);
        assert!(offset_ring_left(&two, 1.0, 1, &OffsetOptions::default()).is_none());
        let dup = ring(&[(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (1.0, 0.0)]);
        assert!(offset_ring_left(&dup, 1.0, 1, &OffsetOptions::default()).is_none());
        let flat = ring(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]);
        assert!(offset_ring_left(&flat, 1.0, 1, &OffsetOptions::default()).is_none());
    }
}
