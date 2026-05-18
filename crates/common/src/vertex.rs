//! 2D vertex (point) with `f64` coordinates.

use core::ops::{Add, Mul, Neg, Sub};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
}

impl Vertex {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn from_array(a: [f64; 2]) -> Self {
        Self { x: a[0], y: a[1] }
    }

    #[inline]
    pub fn to_array(self) -> [f64; 2] {
        [self.x, self.y]
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// 2D scalar cross product `self.x * rhs.y - self.y * rhs.x`.
    #[inline]
    pub fn cross(self, rhs: Self) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }

    #[inline]
    pub fn length_sq(self) -> f64 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f64 {
        self.length_sq().sqrt()
    }

    #[inline]
    pub fn distance_sq(self, other: Self) -> f64 {
        (other - self).length_sq()
    }

    #[inline]
    pub fn distance(self, other: Self) -> f64 {
        (other - self).length()
    }

    /// Returns the unit vector in the direction of `self`. If `self` has
    /// (near-)zero length the result is the zero vector.
    #[inline]
    pub fn normalize_or_zero(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            self * (1.0 / len)
        } else {
            Self::ZERO
        }
    }

    /// Linear interpolation; `t = 0` returns `self`, `t = 1` returns `other`.
    #[inline]
    pub fn lerp(self, other: Self, t: f64) -> Self {
        self + (other - self) * t
    }

    /// Element-wise absolute comparison; useful for tests.
    pub fn approx_eq(self, other: Self, eps: f64) -> bool {
        (self.x - other.x).abs() <= eps && (self.y - other.y).abs() <= eps
    }
}

impl Add for Vertex {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vertex {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f64> for Vertex {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Neg for Vertex {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl From<[f64; 2]> for Vertex {
    #[inline]
    fn from(a: [f64; 2]) -> Self {
        Self::from_array(a)
    }
}

impl From<(f64, f64)> for Vertex {
    #[inline]
    fn from(t: (f64, f64)) -> Self {
        Self::new(t.0, t.1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic() {
        let a = Vertex::new(1.0, 2.0);
        let b = Vertex::new(4.0, 6.0);
        assert_eq!(a + b, Vertex::new(5.0, 8.0));
        assert_eq!(b - a, Vertex::new(3.0, 4.0));
        assert_eq!(a * 2.0, Vertex::new(2.0, 4.0));
        assert_eq!(-a, Vertex::new(-1.0, -2.0));
    }

    #[test]
    fn dot_cross_length() {
        let a = Vertex::new(3.0, 0.0);
        let b = Vertex::new(0.0, 4.0);
        assert_eq!(a.dot(b), 0.0);
        assert_eq!(a.cross(b), 12.0);
        assert_eq!(a.length(), 3.0);
        assert_eq!(b.length_sq(), 16.0);
        assert_eq!(a.distance(b), 5.0);
    }

    #[test]
    fn normalize_zero() {
        assert_eq!(Vertex::ZERO.normalize_or_zero(), Vertex::ZERO);
        let n = Vertex::new(3.0, 4.0).normalize_or_zero();
        assert!((n.length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn lerp_endpoints() {
        let a = Vertex::new(0.0, 0.0);
        let b = Vertex::new(10.0, -4.0);
        assert_eq!(a.lerp(b, 0.0), a);
        assert_eq!(a.lerp(b, 1.0), b);
        assert_eq!(a.lerp(b, 0.5), Vertex::new(5.0, -2.0));
    }
}
