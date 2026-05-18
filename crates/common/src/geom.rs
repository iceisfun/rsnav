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
}
