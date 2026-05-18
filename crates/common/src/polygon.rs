//! Simple (non-self-intersecting) 2D polygons.
//!
//! A [`Polygon`] is an ordered ring of vertices; the closing edge from
//! `vertices.last()` back to `vertices.first()` is implicit. A
//! [`PolygonWithHoles`] pairs an outer ring with zero or more interior holes.

use crate::{Aabb, Vertex, geom};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Winding {
    /// Vertices ordered counter-clockwise (positive signed area).
    CounterClockwise,
    /// Vertices ordered clockwise (negative signed area).
    Clockwise,
    /// Degenerate ring (zero signed area).
    Degenerate,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<Vertex>,
}

impl Polygon {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
        }
    }

    pub fn from_vertices<I: IntoIterator<Item = Vertex>>(iter: I) -> Self {
        Self {
            vertices: iter.into_iter().collect(),
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    pub fn aabb(&self) -> Aabb {
        Aabb::from_points(self.vertices.iter().copied())
    }

    /// Twice the signed area of the polygon (shoelace formula).
    pub fn signed_area2(&self) -> f64 {
        let n = self.vertices.len();
        if n < 3 {
            return 0.0;
        }
        let mut sum = 0.0;
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            sum += a.x * b.y - b.x * a.y;
        }
        sum
    }

    #[inline]
    pub fn signed_area(&self) -> f64 {
        0.5 * self.signed_area2()
    }

    #[inline]
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    pub fn winding(&self) -> Winding {
        let s = self.signed_area2();
        if s > 0.0 {
            Winding::CounterClockwise
        } else if s < 0.0 {
            Winding::Clockwise
        } else {
            Winding::Degenerate
        }
    }

    /// Reverses the vertex order in place, flipping winding.
    pub fn reverse(&mut self) {
        self.vertices.reverse();
    }

    /// Reorients the polygon so its winding matches `target`. Degenerate
    /// polygons are left unchanged.
    pub fn ensure_winding(&mut self, target: Winding) {
        let current = self.winding();
        if current == Winding::Degenerate || current == target {
            return;
        }
        self.reverse();
    }

    /// Iterates over directed edges `(start, end)`, including the closing edge.
    pub fn edges(&self) -> impl Iterator<Item = (Vertex, Vertex)> + '_ {
        let n = self.vertices.len();
        (0..n).map(move |i| (self.vertices[i], self.vertices[(i + 1) % n]))
    }

    /// Point-in-polygon test (ray-cast). Boundary is treated as inside.
    pub fn contains(&self, p: Vertex) -> bool {
        let n = self.vertices.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let vi = self.vertices[i];
            let vj = self.vertices[j];
            // Boundary check: point lies on edge i-j?
            if geom::orient2d(vi, vj, p) == 0.0 && geom::on_segment_collinear(vi, vj, p) {
                return true;
            }
            let crosses = (vi.y > p.y) != (vj.y > p.y)
                && p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x;
            if crosses {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// A point guaranteed to lie strictly inside this simple polygon.
    ///
    /// Returns `None` for degenerate polygons (fewer than 3 vertices, all
    /// vertices collinear, or — pathologically — no ear can be found).
    /// For non-degenerate simple polygons this always succeeds.
    ///
    /// **Use this instead of the arithmetic centroid for hole seed points.**
    /// The centroid of a *convex* polygon is always interior, but the
    /// centroid of a *concave* polygon (e.g. C-shape, L-shape, U-shape)
    /// often falls outside the polygon — which silently breaks any code
    /// downstream that expects an interior point (e.g. `carve_holes` will
    /// flood-fill the wrong region).
    ///
    /// Implementation: ear-finding. Locates a polygon vertex `V_i` whose
    /// triangle `(V_{i-1}, V_i, V_{i+1})` is (a) oriented in the same
    /// direction as the polygon as a whole and (b) contains no other
    /// polygon vertex. The centroid of that ear-triangle is interior.
    /// `O(n²)` worst case; `O(n)` once an ear is found (almost always
    /// within the first few vertices).
    pub fn interior_point(&self) -> Option<Vertex> {
        let n = self.vertices.len();
        if n < 3 {
            return None;
        }
        let winding = self.winding();
        if winding == Winding::Degenerate {
            return None;
        }
        let ccw = winding == Winding::CounterClockwise;
        for i in 0..n {
            let a = self.vertices[(i + n - 1) % n];
            let b = self.vertices[i];
            let c = self.vertices[(i + 1) % n];
            let cross = geom::orient2d(a, b, c);
            let convex_here = if ccw { cross > 0.0 } else { cross < 0.0 };
            if !convex_here {
                continue;
            }
            // Reject if any OTHER vertex lies inside (or on) the ear.
            let prev_idx = (i + n - 1) % n;
            let next_idx = (i + 1) % n;
            let mut clear = true;
            for j in 0..n {
                if j == prev_idx || j == i || j == next_idx {
                    continue;
                }
                if point_in_tri_inclusive(a, b, c, self.vertices[j]) {
                    clear = false;
                    break;
                }
            }
            if clear {
                return Some(Vertex::new(
                    (a.x + b.x + c.x) / 3.0,
                    (a.y + b.y + c.y) / 3.0,
                ));
            }
        }
        None
    }

    /// Removes vertices that are exactly collinear with their neighbours.
    /// Returns the number of vertices removed.
    pub fn remove_collinear(&mut self) -> usize {
        let original = self.vertices.len();
        if original < 3 {
            return 0;
        }
        let mut cleaned: Vec<Vertex> = Vec::with_capacity(original);
        for i in 0..original {
            let prev = self.vertices[(i + original - 1) % original];
            let cur = self.vertices[i];
            let next = self.vertices[(i + 1) % original];
            if geom::orient2d(prev, cur, next) != 0.0 {
                cleaned.push(cur);
            }
        }
        // A second pass can be required if collinear runs were long; iterate
        // until stable.
        let removed_first = original - cleaned.len();
        self.vertices = cleaned;
        if removed_first > 0 {
            removed_first + self.remove_collinear()
        } else {
            0
        }
    }
}

impl From<Vec<Vertex>> for Polygon {
    fn from(vertices: Vec<Vertex>) -> Self {
        Self { vertices }
    }
}

#[inline]
fn point_in_tri_inclusive(a: Vertex, b: Vertex, c: Vertex, p: Vertex) -> bool {
    let d1 = geom::orient2d(a, b, p);
    let d2 = geom::orient2d(b, c, p);
    let d3 = geom::orient2d(c, a, p);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

/// An outer ring with zero or more interior holes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PolygonWithHoles {
    pub outer: Polygon,
    pub holes: Vec<Polygon>,
}

impl PolygonWithHoles {
    pub fn new(outer: Polygon) -> Self {
        Self {
            outer,
            holes: Vec::new(),
        }
    }

    pub fn with_holes(outer: Polygon, holes: Vec<Polygon>) -> Self {
        Self { outer, holes }
    }

    pub fn aabb(&self) -> Aabb {
        self.outer.aabb()
    }

    /// Area of the outer ring minus the area of every hole.
    pub fn area(&self) -> f64 {
        let mut a = self.outer.area();
        for h in &self.holes {
            a -= h.area();
        }
        a.max(0.0)
    }

    /// True when `p` is in the outer ring and not in any hole.
    pub fn contains(&self, p: Vertex) -> bool {
        if !self.outer.contains(p) {
            return false;
        }
        !self.holes.iter().any(|h| h.contains(p))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    fn unit_square_ccw() -> Polygon {
        Polygon::from_vertices([v(0.0, 0.0), v(1.0, 0.0), v(1.0, 1.0), v(0.0, 1.0)])
    }

    fn unit_square_cw() -> Polygon {
        Polygon::from_vertices([v(0.0, 0.0), v(0.0, 1.0), v(1.0, 1.0), v(1.0, 0.0)])
    }

    #[test]
    fn winding_and_area() {
        let ccw = unit_square_ccw();
        assert_eq!(ccw.winding(), Winding::CounterClockwise);
        assert_eq!(ccw.signed_area(), 1.0);
        assert_eq!(ccw.area(), 1.0);

        let cw = unit_square_cw();
        assert_eq!(cw.winding(), Winding::Clockwise);
        assert_eq!(cw.signed_area(), -1.0);
        assert_eq!(cw.area(), 1.0);
    }

    #[test]
    fn ensure_winding_flips_when_needed() {
        let mut p = unit_square_cw();
        p.ensure_winding(Winding::CounterClockwise);
        assert_eq!(p.winding(), Winding::CounterClockwise);
        // A second call is a no-op.
        p.ensure_winding(Winding::CounterClockwise);
        assert_eq!(p.winding(), Winding::CounterClockwise);
    }

    #[test]
    fn contains_basic() {
        let p = unit_square_ccw();
        assert!(p.contains(v(0.5, 0.5)));
        assert!(!p.contains(v(1.5, 0.5)));
        assert!(!p.contains(v(-0.1, 0.5)));
        // Boundary points are considered inside.
        assert!(p.contains(v(0.0, 0.5)));
        assert!(p.contains(v(1.0, 1.0)));
    }

    #[test]
    fn remove_collinear_strips_midpoints() {
        // Square with extra midpoints on each edge.
        let mut p = Polygon::from_vertices([
            v(0.0, 0.0),
            v(0.5, 0.0),
            v(1.0, 0.0),
            v(1.0, 0.5),
            v(1.0, 1.0),
            v(0.5, 1.0),
            v(0.0, 1.0),
            v(0.0, 0.5),
        ]);
        let removed = p.remove_collinear();
        assert_eq!(removed, 4);
        assert_eq!(p.len(), 4);
        assert_eq!(p.area(), 1.0);
    }

    #[test]
    fn polygon_with_holes_area_and_contains() {
        let outer = Polygon::from_vertices([v(0.0, 0.0), v(4.0, 0.0), v(4.0, 4.0), v(0.0, 4.0)]);
        let hole = Polygon::from_vertices([v(1.0, 1.0), v(1.0, 3.0), v(3.0, 3.0), v(3.0, 1.0)]);
        let p = PolygonWithHoles::with_holes(outer, vec![hole]);
        assert_eq!(p.area(), 16.0 - 4.0);
        assert!(p.contains(v(0.5, 0.5)));
        assert!(!p.contains(v(2.0, 2.0))); // inside the hole
    }

    #[test]
    fn interior_point_for_convex_lands_inside() {
        let p = unit_square_ccw();
        let ip = p.interior_point().unwrap();
        assert!(p.contains(ip), "interior_point {:?} not in polygon", ip);
    }

    #[test]
    fn interior_point_for_concave_c_shape_lands_inside() {
        // Classic C-shape — the kind of hole shape where the arithmetic
        // centroid falls OUTSIDE the polygon. interior_point must still
        // land inside.
        //
        //   ┌────────┐
        //   │  ┌──┐  │
        //   │  │  │  │   ← the C's interior is the U-shape around
        //   │  │  │  │     the inner cutout
        //   │  └──┘  │
        //   │        │
        //   └────────┘
        //
        // CCW around the boundary, going out around the cutout then back.
        let p = Polygon::from_vertices([
            v(0.0, 0.0),
            v(10.0, 0.0),
            v(10.0, 10.0),
            v(7.0, 10.0),
            v(7.0, 3.0),
            v(3.0, 3.0),
            v(3.0, 10.0),
            v(0.0, 10.0),
        ]);
        // Sanity check: the *centroid* is outside the polygon — this is
        // exactly the bug Polygon::interior_point fixes.
        let n = p.vertices.len() as f64;
        let cx = p.vertices.iter().map(|v| v.x).sum::<f64>() / n;
        let cy = p.vertices.iter().map(|v| v.y).sum::<f64>() / n;
        assert!(
            !p.contains(Vertex::new(cx, cy)),
            "test premise wrong: centroid IS inside this concave polygon"
        );
        // The fix.
        let ip = p.interior_point().unwrap();
        assert!(p.contains(ip), "interior_point {:?} not in C-shape", ip);
    }

    #[test]
    fn interior_point_for_collinear_polygon_returns_none() {
        let p = Polygon::from_vertices([v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)]);
        assert!(p.interior_point().is_none());
    }

    #[test]
    fn interior_point_works_for_cw_winding() {
        let p = unit_square_cw();
        let ip = p.interior_point().unwrap();
        assert!(p.contains(ip));
    }
}
