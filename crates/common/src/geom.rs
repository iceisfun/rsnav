//! Pure-function geometry helpers used across crates.
//!
//! Non-adaptive predicates only — convenient for tests and high-level callers.
//! The robust adaptive `orient2d` / `incircle` (Shewchuk) lives in the `triangle`
//! crate where it is actually needed.

use crate::Vertex;

/// Twice the signed area of triangle `(a, b, c)`. Positive when `(a,b,c)` is
/// counter-clockwise, negative when clockwise, zero when collinear.
#[inline]
pub fn signed_area2(a: Vertex, b: Vertex, c: Vertex) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Non-robust 2D orientation test. Returns:
///   `> 0`  if `c` lies to the left of the directed line `a -> b`
///   `< 0`  if `c` lies to the right
///   `= 0`  if collinear
#[inline]
pub fn orient2d(a: Vertex, b: Vertex, c: Vertex) -> f64 {
    signed_area2(a, b, c)
}

/// Non-robust in-circle test. With `(a, b, c)` listed in CCW order, returns:
///   `> 0`  if `d` is strictly inside the circle through `a, b, c`
///   `< 0`  if `d` is strictly outside
///   `= 0`  if `d` lies on the circle
pub fn incircle(a: Vertex, b: Vertex, c: Vertex, d: Vertex) -> f64 {
    let adx = a.x - d.x;
    let ady = a.y - d.y;
    let bdx = b.x - d.x;
    let bdy = b.y - d.y;
    let cdx = c.x - d.x;
    let cdy = c.y - d.y;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    alift * (bdx * cdy - cdx * bdy) + blift * (cdx * ady - adx * cdy)
        + clift * (adx * bdy - bdx * ady)
}

/// Returns true if point `p` lies on the closed segment `[a, b]`. Assumes the
/// three points are exactly collinear; the caller usually checks
/// `orient2d(a, b, p) == 0` first.
#[inline]
pub fn on_segment_collinear(a: Vertex, b: Vertex, p: Vertex) -> bool {
    let min_x = a.x.min(b.x);
    let max_x = a.x.max(b.x);
    let min_y = a.y.min(b.y);
    let max_y = a.y.max(b.y);
    p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y
}

/// Result of [`segments_intersect`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SegmentIntersection {
    /// Segments are disjoint.
    None,
    /// Segments cross at a single point (interior to both, or touching at an
    /// endpoint). `point` is the intersection.
    Point { point: Vertex },
    /// Segments are collinear and overlap (including endpoint-touch).
    Collinear,
}

/// Tests whether the closed segments `[a1, a2]` and `[b1, b2]` intersect.
pub fn segments_intersect(a1: Vertex, a2: Vertex, b1: Vertex, b2: Vertex) -> SegmentIntersection {
    let d1 = orient2d(b1, b2, a1);
    let d2 = orient2d(b1, b2, a2);
    let d3 = orient2d(a1, a2, b1);
    let d4 = orient2d(a1, a2, b2);

    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        // Strict crossing: solve for the intersection parameter on segment A.
        let dx_a = a2.x - a1.x;
        let dy_a = a2.y - a1.y;
        let dx_b = b2.x - b1.x;
        let dy_b = b2.y - b1.y;
        let denom = dx_a * dy_b - dy_a * dx_b;
        // denom cannot be zero here because d1..d4 confirm strict crossing.
        let t = ((b1.x - a1.x) * dy_b - (b1.y - a1.y) * dx_b) / denom;
        return SegmentIntersection::Point {
            point: Vertex::new(a1.x + t * dx_a, a1.y + t * dy_a),
        };
    }

    // Handle collinear and endpoint-touch cases.
    if d1 == 0.0 && d2 == 0.0 && d3 == 0.0 && d4 == 0.0 {
        // All four points collinear; check whether the projections overlap.
        let overlap_x = a1.x.max(a2.x) >= b1.x.min(b2.x) && b1.x.max(b2.x) >= a1.x.min(a2.x);
        let overlap_y = a1.y.max(a2.y) >= b1.y.min(b2.y) && b1.y.max(b2.y) >= a1.y.min(a2.y);
        if overlap_x && overlap_y {
            return SegmentIntersection::Collinear;
        }
        return SegmentIntersection::None;
    }
    if d1 == 0.0 && on_segment_collinear(b1, b2, a1) {
        return SegmentIntersection::Point { point: a1 };
    }
    if d2 == 0.0 && on_segment_collinear(b1, b2, a2) {
        return SegmentIntersection::Point { point: a2 };
    }
    if d3 == 0.0 && on_segment_collinear(a1, a2, b1) {
        return SegmentIntersection::Point { point: b1 };
    }
    if d4 == 0.0 && on_segment_collinear(a1, a2, b2) {
        return SegmentIntersection::Point { point: b2 };
    }
    SegmentIntersection::None
}

/// Projects `p` onto the closed segment `[a, b]` and returns the nearest point.
pub fn nearest_point_on_segment(a: Vertex, b: Vertex, p: Vertex) -> Vertex {
    let ab = b - a;
    let len_sq = ab.length_sq();
    if len_sq == 0.0 {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Tests whether point `p` lies inside triangle `(a, b, c)`, boundary
/// **inclusive**. Winding-agnostic — works for both CW and CCW triangles.
#[inline]
pub fn point_in_triangle(a: Vertex, b: Vertex, c: Vertex, p: Vertex) -> bool {
    let d1 = orient2d(a, b, p);
    let d2 = orient2d(b, c, p);
    let d3 = orient2d(c, a, p);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

/// Closest point on triangle `(a, b, c)` to `p`, paired with the
/// euclidean distance to it.
///
/// Returns `(p, 0.0)` when `p` is inside or on the boundary; otherwise
/// the closest point lies on whichever of the three edges is nearest.
/// Winding-agnostic.
pub fn nearest_point_on_triangle(
    a: Vertex,
    b: Vertex,
    c: Vertex,
    p: Vertex,
) -> (Vertex, f64) {
    if point_in_triangle(a, b, c, p) {
        return (p, 0.0);
    }
    let candidates = [
        nearest_point_on_segment(a, b, p),
        nearest_point_on_segment(b, c, p),
        nearest_point_on_segment(c, a, p),
    ];
    let mut best = (candidates[0], candidates[0].distance(p));
    for cand in &candidates[1..] {
        let d = cand.distance(p);
        if d < best.1 {
            best = (*cand, d);
        }
    }
    best
}

/// A proper (non-parallel) crossing of two segments — see
/// [`segment_intersection`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SegmentHit {
    /// Parameter of the hit along the first segment, in `[0, 1]`.
    pub t: f64,
    /// Parameter of the hit along the second segment, in `[0, 1]`.
    pub u: f64,
    /// The intersection point.
    pub point: Vertex,
}

/// Parametric intersection of segments `(a1, a2)` and `(b1, b2)`.
///
/// Returns `None` when the segments are parallel or collinear, or when
/// they would cross outside either segment's `[0, 1]` range. Unlike
/// [`segments_intersect`] — which *classifies* the intersection and
/// reports collinear overlap — this yields the crossing parameters,
/// which a caller walking a segment across a mesh needs.
pub fn segment_intersection(
    a1: Vertex,
    a2: Vertex,
    b1: Vertex,
    b2: Vertex,
) -> Option<SegmentHit> {
    let r = a2 - a1;
    let s = b2 - b1;
    let denom = r.x * s.y - r.y * s.x;
    if denom == 0.0 {
        return None; // parallel or collinear
    }
    let q_p = b1 - a1;
    let t = (q_p.x * s.y - q_p.y * s.x) / denom;
    let u = (q_p.x * r.y - q_p.y * r.x) / denom;
    let unit = 0.0..=1.0;
    if !unit.contains(&t) || !unit.contains(&u) {
        return None;
    }
    Some(SegmentHit {
        t,
        u,
        point: Vertex::new(a1.x + t * r.x, a1.y + t * r.y),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    #[test]
    fn orient_ccw_cw_collinear() {
        assert!(orient2d(v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0)) > 0.0);
        assert!(orient2d(v(0.0, 0.0), v(1.0, 0.0), v(0.0, -1.0)) < 0.0);
        assert_eq!(orient2d(v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)), 0.0);
    }

    #[test]
    fn incircle_unit() {
        // CCW triangle on the unit circle.
        let a = v(1.0, 0.0);
        let b = v(0.0, 1.0);
        let c = v(-1.0, 0.0);
        assert!(incircle(a, b, c, v(0.0, 0.0)) > 0.0); // inside
        assert!(incircle(a, b, c, v(2.0, 0.0)) < 0.0); // outside
        assert!(incircle(a, b, c, v(0.0, -1.0)).abs() < 1e-12); // on
    }

    #[test]
    fn segment_crossing() {
        match segments_intersect(v(0.0, 0.0), v(2.0, 2.0), v(0.0, 2.0), v(2.0, 0.0)) {
            SegmentIntersection::Point { point } => {
                assert!(point.approx_eq(v(1.0, 1.0), 1e-12));
            }
            other => panic!("expected Point, got {:?}", other),
        }
    }

    #[test]
    fn segment_disjoint() {
        assert_eq!(
            segments_intersect(v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0), v(3.0, 0.0)),
            SegmentIntersection::None
        );
    }

    #[test]
    fn segment_collinear_overlap() {
        assert_eq!(
            segments_intersect(v(0.0, 0.0), v(2.0, 0.0), v(1.0, 0.0), v(3.0, 0.0)),
            SegmentIntersection::Collinear
        );
    }

    #[test]
    fn segment_touch_endpoint() {
        match segments_intersect(v(0.0, 0.0), v(1.0, 0.0), v(1.0, 0.0), v(1.0, 1.0)) {
            SegmentIntersection::Point { point } => {
                assert!(point.approx_eq(v(1.0, 0.0), 1e-12))
            }
            other => panic!("expected Point, got {:?}", other),
        }
    }

    #[test]
    fn nearest_point_clamps_to_endpoints() {
        assert_eq!(
            nearest_point_on_segment(v(0.0, 0.0), v(10.0, 0.0), v(-5.0, 3.0)),
            v(0.0, 0.0)
        );
        assert_eq!(
            nearest_point_on_segment(v(0.0, 0.0), v(10.0, 0.0), v(20.0, -2.0)),
            v(10.0, 0.0)
        );
        assert_eq!(
            nearest_point_on_segment(v(0.0, 0.0), v(10.0, 0.0), v(3.0, 7.0)),
            v(3.0, 0.0)
        );
    }

    #[test]
    fn point_in_triangle_inclusive_any_winding() {
        let a = v(0.0, 0.0);
        let b = v(4.0, 0.0);
        let c = v(0.0, 4.0);
        // Interior, on an edge, and at a vertex all count as inside.
        assert!(point_in_triangle(a, b, c, v(1.0, 1.0)));
        assert!(point_in_triangle(a, b, c, v(2.0, 0.0)));
        assert!(point_in_triangle(a, b, c, a));
        assert!(!point_in_triangle(a, b, c, v(3.0, 3.0)));
        // Same answers for the clockwise winding.
        assert!(point_in_triangle(a, c, b, v(1.0, 1.0)));
        assert!(!point_in_triangle(a, c, b, v(3.0, 3.0)));
    }

    #[test]
    fn nearest_point_on_triangle_inside_and_outside() {
        let a = v(0.0, 0.0);
        let b = v(4.0, 0.0);
        let c = v(0.0, 4.0);
        let (pt, d) = nearest_point_on_triangle(a, b, c, v(1.0, 1.0));
        assert_eq!(pt, v(1.0, 1.0));
        assert_eq!(d, 0.0);
        // A point below the a–b edge snaps onto it.
        let (pt, d) = nearest_point_on_triangle(a, b, c, v(2.0, -3.0));
        assert!(pt.approx_eq(v(2.0, 0.0), 1e-12));
        assert!((d - 3.0).abs() < 1e-12);
    }

    #[test]
    fn segment_intersection_crossing_and_parallel() {
        let hit = segment_intersection(v(0.0, 0.0), v(2.0, 2.0), v(0.0, 2.0), v(2.0, 0.0))
            .expect("segments cross");
        assert!(hit.point.approx_eq(v(1.0, 1.0), 1e-12));
        assert!((hit.t - 0.5).abs() < 1e-12 && (hit.u - 0.5).abs() < 1e-12);
        // Parallel: no crossing.
        assert!(
            segment_intersection(v(0.0, 0.0), v(2.0, 0.0), v(0.0, 1.0), v(2.0, 1.0)).is_none()
        );
        // Infinite lines cross, but outside both segments' [0,1] range.
        assert!(
            segment_intersection(v(0.0, 0.0), v(1.0, 0.0), v(5.0, -1.0), v(5.0, 1.0)).is_none()
        );
    }
}
