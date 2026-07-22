//! Deterministic pseudo-random numbers for tests and benches.
//!
//! A 64-bit LCG (Knuth's MMIX constants), the same generator
//! `triangle`'s divide-and-conquer tests have used from the start.
//! Zero-dependency by design: property tests across the workspace seed
//! one of these with a fixed constant and get a reproducible sequence
//! on every platform.

/// 64-bit linear congruential generator. The tuple field is the state;
/// seed it directly: `Lcg(42)`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Lcg(pub u64);

impl Lcg {
    /// Advance the state and return the next raw 64-bit value.
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0
    }

    /// Pseudo-random `f64` in `[0, 1)`, from the top 53 bits.
    pub fn next_f64(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_sequence() {
        let mut a = Lcg(42);
        let mut b = Lcg(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn f64_in_unit_interval() {
        let mut rng = Lcg(7);
        for _ in 0..1000 {
            let x = rng.next_f64();
            assert!((0.0..1.0).contains(&x), "out of range: {x}");
        }
    }

    /// Pin the exact sequence so the generator can never silently change:
    /// property-test failures are reported by seed, and a changed stream
    /// would make those seeds unreproducible.
    #[test]
    fn sequence_pinned() {
        let mut rng = Lcg(1);
        assert_eq!(rng.next_u64(), 7806831264735756412);
        assert_eq!(rng.next_u64(), 9396908728118811419);
    }
}
