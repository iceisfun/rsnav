//! Axis-aligned bounding box (3D, `f64`).
//!
//! Mirrors [`crate::Aabb`] for 2D; sibling type used by the voxel and
//! region-extraction pipeline.

use crate::Vec3;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Aabb3 {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb3 {
    /// An empty AABB: `min = +inf`, `max = -inf`. Any extend brings it to a sane state.
    pub const EMPTY: Self = Self {
        min: Vec3 {
            x: f64::INFINITY,
            y: f64::INFINITY,
            z: f64::INFINITY,
        },
        max: Vec3 {
            x: f64::NEG_INFINITY,
            y: f64::NEG_INFINITY,
            z: f64::NEG_INFINITY,
        },
    };

    #[inline]
    pub fn from_point(p: Vec3) -> Self {
        Self { min: p, max: p }
    }

    pub fn from_points<I: IntoIterator<Item = Vec3>>(points: I) -> Self {
        let mut aabb = Self::EMPTY;
        for p in points {
            aabb.extend(p);
        }
        aabb
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    #[inline]
    pub fn extend(&mut self, p: Vec3) {
        if p.x < self.min.x {
            self.min.x = p.x;
        }
        if p.y < self.min.y {
            self.min.y = p.y;
        }
        if p.z < self.min.z {
            self.min.z = p.z;
        }
        if p.x > self.max.x {
            self.max.x = p.x;
        }
        if p.y > self.max.y {
            self.max.y = p.y;
        }
        if p.z > self.max.z {
            self.max.z = p.z;
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
    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// Returns true if `self` and `other` share at least one point (touching counts).
    #[inline]
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    #[inline]
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    #[inline]
    pub fn center(&self) -> Vec3 {
        Vec3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_then_extend() {
        let mut a = Aabb3::EMPTY;
        assert!(a.is_empty());
        a.extend(Vec3::new(1.0, 2.0, 3.0));
        assert!(!a.is_empty());
        assert_eq!(a.min, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(a.max, Vec3::new(1.0, 2.0, 3.0));
        a.extend(Vec3::new(-3.0, 5.0, -1.0));
        assert_eq!(a.min, Vec3::new(-3.0, 2.0, -1.0));
        assert_eq!(a.max, Vec3::new(1.0, 5.0, 3.0));
    }

    #[test]
    fn from_points_and_contains() {
        let a = Aabb3::from_points([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 1.0, 4.0),
            Vec3::new(-1.0, 3.0, -2.0),
        ]);
        assert!(a.contains(Vec3::new(0.0, 0.0, 0.0)));
        assert!(a.contains(Vec3::new(-1.0, 3.0, -2.0)));
        assert!(!a.contains(Vec3::new(3.0, 0.0, 0.0)));
        assert_eq!(a.size(), Vec3::new(3.0, 3.0, 6.0));
    }

    #[test]
    fn intersects_and_union() {
        let a = Aabb3::from_points([Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0)]);
        let b = Aabb3::from_points([Vec3::new(1.0, 1.0, 1.0), Vec3::new(3.0, 3.0, 3.0)]);
        let c = Aabb3::from_points([Vec3::new(5.0, 5.0, 5.0), Vec3::new(6.0, 6.0, 6.0)]);
        assert!(a.intersects(&b));
        assert!(!a.intersects(&c));
        let u = a.union(&c);
        assert_eq!(u.min, Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(u.max, Vec3::new(6.0, 6.0, 6.0));
    }
}
