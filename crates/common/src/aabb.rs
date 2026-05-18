//! Axis-aligned bounding box (2D, `f64`).

use crate::Vertex;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vertex,
    pub max: Vertex,
}

impl Aabb {
    /// An empty AABB: `min = +inf`, `max = -inf`. Any extend brings it to a sane state.
    pub const EMPTY: Self = Self {
        min: Vertex {
            x: f64::INFINITY,
            y: f64::INFINITY,
        },
        max: Vertex {
            x: f64::NEG_INFINITY,
            y: f64::NEG_INFINITY,
        },
    };

    #[inline]
    pub fn from_point(p: Vertex) -> Self {
        Self { min: p, max: p }
    }

    pub fn from_points<I: IntoIterator<Item = Vertex>>(points: I) -> Self {
        let mut aabb = Self::EMPTY;
        for p in points {
            aabb.extend(p);
        }
        aabb
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y
    }

    #[inline]
    pub fn extend(&mut self, p: Vertex) {
        if p.x < self.min.x {
            self.min.x = p.x;
        }
        if p.y < self.min.y {
            self.min.y = p.y;
        }
        if p.x > self.max.x {
            self.max.x = p.x;
        }
        if p.y > self.max.y {
            self.max.y = p.y;
        }
    }

    #[inline]
    pub fn union(&self, other: &Self) -> Self {
        let mut out = *self;
        out.extend(other.min);
        out.extend(other.max);
        out
    }

    #[inline]
    pub fn contains(&self, p: Vertex) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    /// Returns true if `self` and `other` share at least one point (touching counts).
    #[inline]
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    #[inline]
    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    #[inline]
    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }

    #[inline]
    pub fn center(&self) -> Vertex {
        Vertex::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_then_extend() {
        let mut a = Aabb::EMPTY;
        assert!(a.is_empty());
        a.extend(Vertex::new(1.0, 2.0));
        assert!(!a.is_empty());
        assert_eq!(a.min, Vertex::new(1.0, 2.0));
        assert_eq!(a.max, Vertex::new(1.0, 2.0));
        a.extend(Vertex::new(-3.0, 5.0));
        assert_eq!(a.min, Vertex::new(-3.0, 2.0));
        assert_eq!(a.max, Vertex::new(1.0, 5.0));
    }

    #[test]
    fn from_points_and_contains() {
        let a = Aabb::from_points([
            Vertex::new(0.0, 0.0),
            Vertex::new(2.0, 1.0),
            Vertex::new(-1.0, 3.0),
        ]);
        assert!(a.contains(Vertex::new(0.0, 0.0)));
        assert!(a.contains(Vertex::new(-1.0, 3.0)));
        assert!(!a.contains(Vertex::new(3.0, 0.0)));
        assert_eq!(a.width(), 3.0);
        assert_eq!(a.height(), 3.0);
    }

    #[test]
    fn intersects_and_union() {
        let a = Aabb::from_points([Vertex::new(0.0, 0.0), Vertex::new(2.0, 2.0)]);
        let b = Aabb::from_points([Vertex::new(1.0, 1.0), Vertex::new(3.0, 3.0)]);
        let c = Aabb::from_points([Vertex::new(5.0, 5.0), Vertex::new(6.0, 6.0)]);
        assert!(a.intersects(&b));
        assert!(!a.intersects(&c));
        let u = a.union(&c);
        assert_eq!(u.min, Vertex::new(0.0, 0.0));
        assert_eq!(u.max, Vertex::new(6.0, 6.0));
    }
}
