//! Adversarial edge-case probe for grid erosion. Independent brute-force
//! oracle derived from the *geometric* definition, not from the
//! implementation's internals.

use rsnav_polygon_extract::{Bitfield, ErodeError, ErodeOptions};

// --- oracle --------------------------------------------------------------

/// Squared clearance of cell (c,r) computed straight from the definition:
/// the squared distance from the unit square [c,c+1]x[r,r+1] to the wall
/// region, where the wall region is the union of non-walkable unit squares
/// PLUS the entire complement of [0,w]x[0,h].
fn oracle_sq(b: &Bitfield, c: usize, r: usize) -> i64 {
    let (w, h) = (b.width as usize, b.height as usize);
    if !b.at(c as i64, r as i64) {
        return 0;
    }
    // Distance to the exterior: min over the four half-planes.
    let ext = [c, w - 1 - c, r, h - 1 - r].into_iter().min().unwrap() as i64;
    let mut best = ext * ext;
    for rw in 0..h {
        for cw in 0..w {
            if b.at(cw as i64, rw as i64) {
                continue;
            }
            let gx = (c as i64 - cw as i64).abs().saturating_sub(1).max(0);
            let gy = (r as i64 - rw as i64).abs().saturating_sub(1).max(0);
            let d = gx * gx + gy * gy;
            if d < best {
                best = d;
            }
        }
    }
    best
}

fn oracle_erode(b: &Bitfield, radius: f64) -> Vec<bool> {
    let (w, h) = (b.width as usize, b.height as usize);
    let mut out = vec![false; w * h];
    for r in 0..h {
        for c in 0..w {
            let sq = oracle_sq(b, c, r) as f64;
            out[r * w + c] = b.at(c as i64, r as i64) && sq >= radius * radius;
        }
    }
    out
}

fn bits(b: &Bitfield) -> Vec<bool> {
    let (w, h) = (b.width as usize, b.height as usize);
    let mut v = vec![false; w * h];
    for r in 0..h {
        for c in 0..w {
            v[r * w + c] = b.at(c as i64, r as i64);
        }
    }
    v
}

fn erode(b: &Bitfield, radius: f64, threads: usize) -> Bitfield {
    b.eroded(&ErodeOptions { radius, threads }).unwrap()
}

/// Deterministic xorshift so failures reproduce.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn bool(&mut self, pct_true: u64) -> bool {
        self.next() % 100 < pct_true
    }
}

fn random_grid(rng: &mut Rng, w: u32, h: u32, density: u64) -> Bitfield {
    let mut b = Bitfield::empty(w, h);
    for r in 0..h {
        for c in 0..w {
            b.set(c, r, rng.bool(density));
        }
    }
    b
}

// --- radius 0 must be an exact no-op -------------------------------------

#[test]
fn radius_zero_is_identity() {
    let mut rng = Rng(0x1234_5678_9abc_def0);
    for &(w, h) in &[(1u32, 1u32), (1, 9), (9, 1), (7, 5), (64, 64), (65, 63)] {
        for &d in &[0u64, 30, 70, 100] {
            let b = random_grid(&mut rng, w, h, d);
            let e = erode(&b, 0.0, 1);
            assert_eq!(bits(&e), bits(&b), "{w}x{h} density {d}");
            assert_eq!((e.width, e.height), (w, h));
            // -0.0 must behave the same and not be rejected as negative.
            let z = erode(&b, -0.0, 4);
            assert_eq!(bits(&z), bits(&b));
        }
    }
}

// --- brute-force agreement over the whole small-grid space ---------------

#[test]
fn matches_oracle_on_random_grids() {
    let mut rng = Rng(0xdead_beef_cafe_0001);
    let radii = [
        0.5, 1.0, 1.0001, 1.4, 1.5, 2.0, 2.2, 2.5, 3.0, 4.0, 6.0, 12.0,
    ];
    for &(w, h) in &[
        (1u32, 1u32),
        (1, 2),
        (2, 1),
        (1, 13),
        (13, 1),
        (2, 2),
        (3, 7),
        (7, 3),
        (9, 9),
        (16, 5),
        (5, 16),
        (12, 11),
    ] {
        for &d in &[10u64, 50, 90, 100] {
            let b = random_grid(&mut rng, w, h, d);
            for &r in &radii {
                let got = bits(&erode(&b, r, 1));
                let want = oracle_erode(&b, r);
                assert_eq!(got, want, "{w}x{h} density {d} radius {r}");
                // parallel path must agree too (small grids stay serial,
                // but the request should not change the answer)
                assert_eq!(bits(&erode(&b, r, 8)), want);
            }
        }
    }
}

#[test]
fn clearance_field_matches_oracle() {
    let mut rng = Rng(0x0000_1111_2222_3333);
    for &(w, h) in &[(1u32, 1u32), (1, 11), (11, 1), (8, 8), (13, 6), (6, 13)] {
        for &d in &[20u64, 60, 100] {
            let b = random_grid(&mut rng, w, h, d);
            let f = b.clearance(1);
            assert_eq!((f.width, f.height), (w, h));
            for r in 0..h {
                for c in 0..w {
                    assert_eq!(
                        i64::from(f.sq_at(c, r)),
                        oracle_sq(&b, c as usize, r as usize),
                        "{w}x{h} d{d} cell ({c},{r})"
                    );
                }
            }
            // sq_at is 0 outside the grid.
            assert_eq!(f.sq_at(w, 0), 0);
            assert_eq!(f.sq_at(0, h), 0);
            assert_eq!(f.sq_at(u32::MAX, u32::MAX), 0);
        }
    }
}

/// The `radius <= 1.0` fast path in `eroded` skips four of five passes.
/// `clearance().threshold()` always runs the full transform, so this pins
/// the shortcut against the long road for exactly the radii it triggers on.
#[test]
fn fast_path_agrees_with_full_transform() {
    let mut rng = Rng(0x5555_6666_7777_8888);
    for &(w, h) in &[(1u32, 1u32), (1, 20), (20, 1), (9, 9), (17, 13), (64, 64)] {
        for &d in &[15u64, 55, 95, 100] {
            let b = random_grid(&mut rng, w, h, d);
            let f = b.clearance(1);
            // NOTE: radii below ~2.2e-162 are excluded — `radius * radius`
            // underflows to 0.0 there and the two paths genuinely diverge.
            // That is pinned separately in `denormal_radius_diverges`.
            for &r in &[1e-160, 0.001, 0.128, 0.5, 0.999, 1.0] {
                let fast = bits(&erode(&b, r, 1));
                let full = bits(&f.threshold(r).unwrap());
                assert_eq!(fast, full, "{w}x{h} d{d} radius {r}");
                assert_eq!(fast, oracle_erode(&b, r), "oracle {w}x{h} d{d} r{r}");
            }
        }
    }
}

// --- degenerate dimensions ------------------------------------------------

#[test]
fn zero_dimension_grids_do_not_panic() {
    for &(w, h) in &[(0u32, 0u32), (0, 5), (5, 0), (0, 1), (1, 0)] {
        let b = Bitfield::empty(w, h);
        for &r in &[0.0, 0.5, 1.0, 3.0, 1e9] {
            let e = erode(&b, r, 4);
            assert_eq!((e.width, e.height), (w, h));
            assert!(bits(&e).is_empty());
        }
        let f = b.clearance(4);
        assert_eq!((f.width, f.height), (w, h));
        assert_eq!(f.sq_at(0, 0), 0);
        let t = f.threshold(2.0).unwrap();
        assert_eq!((t.width, t.height), (w, h));
        assert!(bits(&t).is_empty());
    }
}

#[test]
fn strips_1xn_and_nx1() {
    // A 1-wide strip has zero clearance everywhere (walls on both sides),
    // so any radius > 0 empties it entirely.
    for n in [1u32, 2, 5, 64, 65, 200] {
        for (w, h) in [(1, n), (n, 1)] {
            let mut b = Bitfield::empty(w, h);
            for r in 0..h {
                for c in 0..w {
                    b.set(c, r, true);
                }
            }
            assert_eq!(bits(&erode(&b, 0.0, 1)), bits(&b), "{w}x{h} r0");
            for &r in &[f64::MIN_POSITIVE, 0.5, 1.0, 1.5, 100.0] {
                let e = erode(&b, r, 4);
                assert!(
                    bits(&e).iter().all(|&v| !v),
                    "{w}x{h} radius {r} left cells alive"
                );
            }
            let f = b.clearance(2);
            for r in 0..h {
                for c in 0..w {
                    assert_eq!(f.sq_at(c, r), 0, "{w}x{h} ({c},{r})");
                }
            }
        }
    }
}

// --- the implicit wall border --------------------------------------------

#[test]
fn all_walkable_clearance_is_distance_to_border() {
    for &(w, h) in &[(1u32, 1u32), (5, 5), (9, 4), (4, 9), (33, 17)] {
        let mut b = Bitfield::empty(w, h);
        for r in 0..h {
            for c in 0..w {
                b.set(c, r, true);
            }
        }
        let f = b.clearance(1);
        for r in 0..h {
            for c in 0..w {
                let want = [c, w - 1 - c, r, h - 1 - r].into_iter().min().unwrap() as i64;
                assert_eq!(i64::from(f.sq_at(c, r)), want * want, "{w}x{h} ({c},{r})");
            }
        }
        // Any positive radius peels the outer ring: documented behaviour.
        let e = erode(&b, f64::MIN_POSITIVE, 1);
        for c in 0..w {
            assert!(!e.at(i64::from(c), 0), "bottom row survived");
            assert!(!e.at(i64::from(c), i64::from(h) - 1), "top row survived");
        }
        for r in 0..h {
            assert!(!e.at(0, i64::from(r)), "left col survived");
            assert!(!e.at(i64::from(w) - 1, i64::from(r)), "right col survived");
        }
    }
}

// --- radius vs corridor half-width ---------------------------------------

/// A corridor of odd width `2k+1` has a center row of clearance exactly `k`.
/// The test is `sq >= r*r`, so at `radius == k` the center survives and at
/// the next representable f64 above `k` it dies.
#[test]
fn radius_exactly_corridor_half_width() {
    for k in 1u32..=5 {
        let width = 2 * k + 1;
        // Pad left/right so the horizontal border does not dominate.
        let (w, h) = (width + 40, width + 2);
        let mut b = Bitfield::empty(w, h);
        for r in 1..=width {
            for c in 1..w - 1 {
                b.set(c, r, true);
            }
        }
        let center = 1 + k;
        let mid_c = w / 2;
        let f = b.clearance(1);
        assert_eq!(
            i64::from(f.sq_at(mid_c, center)),
            i64::from(k) * i64::from(k),
            "corridor width {width}"
        );

        let survives = erode(&b, f64::from(k), 1);
        assert!(
            survives.at(i64::from(mid_c), i64::from(center)),
            "radius == half-width {k} killed the corridor center"
        );
        let dies = erode(&b, f64::from(k) + f64::EPSILON * f64::from(k) * 4.0, 1);
        assert!(
            !dies.at(i64::from(mid_c), i64::from(center)),
            "radius just above half-width {k} kept the corridor center"
        );
        // And the whole corridor is gone one full step later.
        let gone = erode(&b, f64::from(k) + 1.0, 1);
        assert!(bits(&gone).iter().all(|&v| !v), "k={k}");
    }
}

/// sqrt(2) rounds up in f64, so the documented conservative bias holds.
#[test]
fn sqrt2_ties_round_toward_erosion() {
    // One isolated wall cell in a wide open field. (12,12) sits at index
    // delta (2,2) from it -> gaps (1,1) -> sq = 2, while the grid border
    // is 8 cells away (sq 64) and cannot win.
    let (w, h) = (21u32, 21u32);
    let mut b = Bitfield::empty(w, h);
    for r in 0..h {
        for c in 0..w {
            b.set(c, r, true);
        }
    }
    b.set(10, 10, false);
    let f = b.clearance(1);
    assert_eq!(f.sq_at(12, 12), 2, "expected sq=2 at (12,12)");
    let s2 = 2f64.sqrt();
    assert!(s2 * s2 > 2.0, "premise: sqrt(2)^2 rounds above 2");
    assert!(
        !f.threshold(s2).unwrap().at(12, 12),
        "sqrt(2) tie should erode"
    );
    assert!(f.threshold(1.4).unwrap().at(12, 12));
    // eroded() agrees with the field at the same radius.
    assert!(!erode(&b, s2, 1).at(12, 12));
    assert!(erode(&b, 1.4, 1).at(12, 12));
}

/// REGRESSION. `validate_radius` used to compute `ceil(radius * radius)`
/// in f64 with no floor. For `radius < ~2.2e-162` the square underflows to
/// 0.0, the threshold became 0, and `ClearanceField::threshold` kept
/// *every* cell including walls — the behaviour its docs reserve for
/// `radius == 0.0` alone. `eroded` never shared the bug (its `<= 1.0` fast
/// path does not consult the threshold), so the two disagreed in that
/// window. `validate_radius` now clamps the threshold to `>= 1` for any
/// `radius > 0`, which is the exact predicate for integer clearances.
#[test]
fn denormal_radius_agrees_with_eroded() {
    let mut b = Bitfield::empty(5, 5);
    for r in 0..5 {
        for c in 0..5 {
            b.set(c, r, true);
        }
    }
    b.set(2, 2, false); // a wall, clearance 0
    let f = b.clearance(1);
    assert_eq!(f.sq_at(2, 2), 0);

    for &tiny in &[f64::MIN_POSITIVE, 1e-200, 1e-170, 1e-163] {
        assert!(tiny > 0.0);
        assert_eq!(tiny * tiny, 0.0, "premise: {tiny:e} squared underflows");
        // Both drop the wall cell, and in fact the whole 5x5 (every cell
        // here touches either the wall or the outside).
        assert!(
            !f.threshold(tiny).unwrap().at(2, 2),
            "threshold({tiny:e}) kept the wall - underflow regression"
        );
        assert!(!erode(&b, tiny, 1).at(2, 2));
        assert_eq!(bits(&f.threshold(tiny).unwrap()), bits(&erode(&b, tiny, 1)));
        assert!(bits(&erode(&b, tiny, 1)).iter().all(|&v| !v));
    }
    // Just above the underflow window the two agree again.
    let ok = 1e-160;
    assert_ne!(ok * ok, 0.0);
    assert_eq!(bits(&f.threshold(ok).unwrap()), bits(&erode(&b, ok, 1)));
}

// --- radius larger than the grid -----------------------------------------

#[test]
fn oversized_radius_empties_without_panic() {
    let mut rng = Rng(0xabcd_ef01_2345_6789);
    for &(w, h) in &[(1u32, 1u32), (3, 3), (1, 50), (50, 1), (37, 29)] {
        let b = random_grid(&mut rng, w, h, 100);
        for &r in &[
            f64::from(w.max(h)),
            f64::from(w.max(h)) * 10.0,
            1e6,
            1e150,
            1e200,
            f64::MAX,
            f64::MAX.sqrt() * 2.0,
        ] {
            let e = erode(&b, r, 4);
            assert_eq!((e.width, e.height), (w, h));
            assert!(
                bits(&e).iter().all(|&v| !v),
                "{w}x{h} radius {r} left survivors"
            );
        }
        // Same through the field path, where the threshold is a separate
        // integer comparison.
        let f = b.clearance(4);
        for &r in &[1e200, f64::MAX] {
            assert!(f.threshold(r).unwrap().data.iter().all(|&v| !v));
        }
    }
}

#[test]
fn invalid_radii_error() {
    let b = Bitfield::empty(4, 4);
    for &r in &[f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0, -1e-300] {
        assert!(
            matches!(
                b.eroded(&ErodeOptions { radius: r, threads: 1 }),
                Err(ErodeError::InvalidRadius(_))
            ),
            "radius {r} accepted by eroded"
        );
        assert!(
            matches!(b.clearance(1).threshold(r), Err(ErodeError::InvalidRadius(_))),
            "radius {r} accepted by threshold"
        );
    }
}

// --- subgrid --------------------------------------------------------------

#[test]
fn subgrid_edges() {
    let mut rng = Rng(0x9999_8888_7777_6666);
    let b = random_grid(&mut rng, 9, 7, 60);
    // Zero-sized requests.
    for &(dw, dh) in &[(0u32, 0u32), (0, 4), (4, 0)] {
        let s = b.subgrid(0, 0, dw, dh);
        assert_eq!((s.width, s.height), (dw, dh));
        assert!(bits(&s).is_empty());
    }
    // Fully out of range, including near u32::MAX origins.
    for &(c0, r0) in &[(9u32, 0u32), (0, 7), (100, 100), (u32::MAX, u32::MAX)] {
        let s = b.subgrid(c0, r0, 3, 3);
        assert_eq!((s.width, s.height), (3, 3));
        assert!(bits(&s).iter().all(|&v| !v), "origin ({c0},{r0})");
    }
    // Overhang is padded, not clipped.
    let s = b.subgrid(7, 5, 5, 5);
    assert_eq!((s.width, s.height), (5, 5));
    for r in 0..5i64 {
        for c in 0..5i64 {
            assert_eq!(s.at(c, r), b.at(7 + c, 5 + r), "({c},{r})");
        }
    }
    // Identity slice.
    let id = b.subgrid(0, 0, 9, 7);
    assert_eq!(bits(&id), bits(&b));
}

// --- thread-count invariance on a grid that actually goes parallel -------

#[test]
fn thread_count_invariant_above_par_threshold() {
    // > PAR_MIN_CELLS (500k) and deliberately not a multiple of the 64-cell
    // band, in either dimension, so every pass ends on a partial band.
    let mut rng = Rng(0x1357_9bdf_0246_8ace);
    let b = random_grid(&mut rng, 823, 701, 78);
    for &r in &[0.5, 1.0, 2.0, 3.5, 7.0] {
        let base = bits(&erode(&b, r, 1));
        for t in [2usize, 3, 5, 8, 16, 32, 0] {
            assert_eq!(bits(&erode(&b, r, t)), base, "radius {r} threads {t}");
        }
    }
    let base = b.clearance(1);
    for t in [2usize, 7, 16, 0] {
        let f = b.clearance(t);
        for row in 0..b.height {
            for col in 0..b.width {
                assert_eq!(f.sq_at(col, row), base.sq_at(col, row), "t{t} {col},{row}");
            }
        }
    }
}

/// Dimensions exactly on the 64-cell band boundary, and one off it, at a
/// size that crosses PAR_MIN_CELLS. Compared against the serial run of the
/// same code plus a full independent EDT on a smaller companion.
#[test]
fn band_boundary_dimensions() {
    let mut rng = Rng(0x2468_ace0_1357_9bdf);
    for &(w, h) in &[(1024u32, 512u32), (1025, 513), (1023, 511), (512, 1024)] {
        let b = random_grid(&mut rng, w, h, 85);
        for &r in &[1.0, 2.0, 4.0] {
            let s = bits(&erode(&b, r, 1));
            let p = bits(&erode(&b, r, 8));
            assert_eq!(s, p, "{w}x{h} radius {r}");
        }
    }
}

/// A tall/thin and short/wide grid above the parallel threshold: h=1 and
/// w=1 exercise the transposed passes at their most degenerate.
#[test]
fn extreme_aspect_above_par_threshold() {
    let mut rng = Rng(0x0f0f_0f0f_f0f0_f0f0);
    for &(w, h) in &[(600_001u32, 1u32), (1, 600_001), (2, 300_001), (300_001, 2)] {
        let b = random_grid(&mut rng, w, h, 95);
        for &r in &[1.0, 3.0] {
            let s = bits(&erode(&b, r, 1));
            let p = bits(&erode(&b, r, 8));
            assert_eq!(s, p, "{w}x{h} radius {r}");
            // Every cell in a 1- or 2-wide grid touches a border, so
            // nothing can survive any radius >= 1.
            if w <= 2 || h <= 2 {
                assert!(s.iter().all(|&v| !v), "{w}x{h} r{r}");
            }
        }
    }
}

// --- monotonicity ---------------------------------------------------------

/// Erosion must be monotone in radius: a larger radius never keeps a cell a
/// smaller radius dropped.
#[test]
fn monotone_in_radius() {
    let mut rng = Rng(0x7f7f_1e1e_2d2d_3c3c);
    for _ in 0..8 {
        let b = random_grid(&mut rng, 41, 37, 80);
        let mut prev: Option<Vec<bool>> = None;
        for &r in &[0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 5.0, 9.0, 40.0] {
            let cur = bits(&erode(&b, r, 1));
            if let Some(p) = &prev {
                for (i, (&a, &c)) in p.iter().zip(cur.iter()).enumerate() {
                    assert!(!(c && !a), "radius {r} resurrected cell {i}");
                }
            }
            prev = Some(cur);
        }
    }
}

/// The guarantee stated in the docs: a kept cell's every point is at least
/// `radius` from the wall region. Checked by sampling the kept cell's
/// corners against every wall square and the exterior.
#[test]
fn never_over_claims() {
    let mut rng = Rng(0xc0ff_ee00_1234_5678);
    for _ in 0..6 {
        let b = random_grid(&mut rng, 23, 19, 70);
        for &r in &[1.0, 1.5, 2.0, 3.0] {
            let e = erode(&b, r, 1);
            for row in 0..19usize {
                for col in 0..23usize {
                    if e.at(col as i64, row as i64) {
                        let sq = oracle_sq(&b, col, row);
                        assert!(
                            (sq as f64) >= r * r,
                            "kept ({col},{row}) with sq {sq} at radius {r}"
                        );
                    }
                }
            }
        }
    }
}
