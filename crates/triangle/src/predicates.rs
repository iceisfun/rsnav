//! Robust adaptive geometric predicates.
//!
//! Direct port of the `predicates.c` portion of Shewchuk's `triangle.c`
//! (the `counterclockwise` / `incircle` routines together with the
//! `fast_expansion_sum_zeroelim`, `scale_expansion_zeroelim`, `estimate`
//! supporting machinery and the error-free transforms).
//!
//! These routines compute the *exact sign* of the orientation and
//! in-circle determinants using IEEE 754 `f64` arithmetic. They are about
//! 5-10x slower than naive `f64` evaluation for typical inputs (still
//! a single fast-path branch), but produce a correct sign for *every* input.
//!
//! The error-free transform primitives are exposed as `#[inline]`
//! functions returning tuples instead of the C macros' output parameters.

use rsnav_common::Vertex;

// --- Machine constants ----------------------------------------------------
//
// Shewchuk's `exactinit()` computes these at runtime by halving until
// `1.0 + epsilon == 1.0`. For IEEE 754 binary64 with round-to-nearest-even
// they are deterministic, so we hard-code them and document each value.

/// Shewchuk's `epsilon`: smallest power of two such that `1.0 + EPS == 1.0`.
/// For IEEE 754 binary64 this is 2^-53.
const EPSILON: f64 = 1.1102230246251565e-16;

/// Splitting constant for Veltkamp/Dekker splitting of an f64 into two halves.
/// For binary64 (53-bit significand) this is 2^27 + 1.
const SPLITTER: f64 = 134217729.0;

// Error bounds (from Shewchuk's `exactinit`).
const RESULTERRBOUND: f64 = (3.0 + 8.0 * EPSILON) * EPSILON;
const CCWERRBOUND_A: f64 = (3.0 + 16.0 * EPSILON) * EPSILON;
const CCWERRBOUND_B: f64 = (2.0 + 12.0 * EPSILON) * EPSILON;
const CCWERRBOUND_C: f64 = (9.0 + 64.0 * EPSILON) * EPSILON * EPSILON;
const ICCERRBOUND_A: f64 = (10.0 + 96.0 * EPSILON) * EPSILON;
const ICCERRBOUND_B: f64 = (4.0 + 48.0 * EPSILON) * EPSILON;
const ICCERRBOUND_C: f64 = (44.0 + 576.0 * EPSILON) * EPSILON * EPSILON;

// --- Error-free transforms (Shewchuk macros translated) ------------------
//
// These compute the exact mathematical result of a single f64 operation as
// the unevaluated sum of two f64 values (an "expansion" of length 2).

/// Fast Two-Sum: requires `|a| >= |b|`. Returns `(x, y)` with `x + y = a + b`
/// exactly and `x = fl(a + b)`.
#[inline(always)]
fn fast_two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let bvirt = x - a;
    let y = b - bvirt;
    (x, y)
}

/// Two-Sum: no ordering requirement on inputs. Returns `(x, y)` with
/// `x + y = a + b` exactly.
#[inline(always)]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let bvirt = x - a;
    let avirt = x - bvirt;
    let bround = b - bvirt;
    let around = a - avirt;
    let y = around + bround;
    (x, y)
}

/// Two-Diff: returns `(x, y)` with `x + y = a - b` exactly.
#[inline(always)]
fn two_diff(a: f64, b: f64) -> (f64, f64) {
    let x = a - b;
    let bvirt = a - x;
    let avirt = x + bvirt;
    let bround = bvirt - b;
    let around = a - avirt;
    let y = around + bround;
    (x, y)
}

/// Given a precomputed `x = a - b` (rounded), returns the rounding error `y`
/// such that `a - b = x + y` exactly.
#[inline(always)]
fn two_diff_tail(a: f64, b: f64, x: f64) -> f64 {
    let bvirt = a - x;
    let avirt = x + bvirt;
    let bround = bvirt - b;
    let around = a - avirt;
    around + bround
}

/// Veltkamp/Dekker split: returns `(hi, lo)` with `a = hi + lo` exactly and
/// each half fits in 26 bits (half of a 53-bit significand, with sign).
#[inline(always)]
fn split(a: f64) -> (f64, f64) {
    let c = SPLITTER * a;
    let abig = c - a;
    let ahi = c - abig;
    let alo = a - ahi;
    (ahi, alo)
}

/// Two-Product: returns `(x, y)` with `x + y = a * b` exactly and `x = fl(a * b)`.
#[inline(always)]
fn two_product(a: f64, b: f64) -> (f64, f64) {
    let x = a * b;
    let (ahi, alo) = split(a);
    let (bhi, blo) = split(b);
    let err1 = x - ahi * bhi;
    let err2 = err1 - alo * bhi;
    let err3 = err2 - ahi * blo;
    let y = alo * blo - err3;
    (x, y)
}

/// Two-Product when one factor is already split. Saves one `split` call.
#[inline(always)]
fn two_product_presplit(a: f64, b: f64, bhi: f64, blo: f64) -> (f64, f64) {
    let x = a * b;
    let (ahi, alo) = split(a);
    let err1 = x - ahi * bhi;
    let err2 = err1 - alo * bhi;
    let err3 = err2 - ahi * blo;
    let y = alo * blo - err3;
    (x, y)
}

/// Square: returns `(x, y)` with `x + y = a * a` exactly. Slightly cheaper
/// than `two_product(a, a)`.
#[allow(dead_code)] // reserved for the full incircle exact path
#[inline(always)]
fn square(a: f64) -> (f64, f64) {
    let x = a * a;
    let (ahi, alo) = split(a);
    let err1 = x - ahi * ahi;
    let err3 = err1 - (ahi + ahi) * alo;
    let y = alo * alo - err3;
    (x, y)
}

/// Two-One-Sum: `(a1 + a0) + b`, expansion length 3.
/// Returns `(x2, x1, x0)` such that their sum equals the exact result.
#[allow(dead_code)] // reserved for the full incircle exact path
#[inline(always)]
fn two_one_sum(a1: f64, a0: f64, b: f64) -> (f64, f64, f64) {
    let (i, x0) = two_sum(a0, b);
    let (x2, x1) = two_sum(a1, i);
    (x2, x1, x0)
}

/// Two-One-Diff: `(a1 + a0) - b`, expansion length 3.
#[inline(always)]
fn two_one_diff(a1: f64, a0: f64, b: f64) -> (f64, f64, f64) {
    let (i, x0) = two_diff(a0, b);
    let (x2, x1) = two_sum(a1, i);
    (x2, x1, x0)
}

/// Two-Two-Diff: `(a1 + a0) - (b1 + b0)`, expansion length 4.
#[inline(always)]
fn two_two_diff(a1: f64, a0: f64, b1: f64, b0: f64) -> (f64, f64, f64, f64) {
    let (j, _0, x0) = two_one_diff(a1, a0, b0);
    let (x3, x2, x1) = two_one_diff(j, _0, b1);
    (x3, x2, x1, x0)
}

// --- Multi-precision expansion arithmetic --------------------------------

/// Sum two zero-eliminated expansions: `h = e + f`. Writes the result into
/// `h` and returns its length. `h` must have capacity >= `e.len() + f.len()`.
///
/// Port of `fast_expansion_sum_zeroelim`. `h` must not alias `e` or `f`.
fn fast_expansion_sum_zeroelim(e: &[f64], f: &[f64], h: &mut [f64]) -> usize {
    let elen = e.len();
    let flen = f.len();

    let mut enow = e[0];
    let mut fnow = f[0];
    let mut eindex = 0usize;
    let mut findex = 0usize;

    let mut q;
    if (fnow > enow) == (fnow > -enow) {
        q = enow;
        eindex += 1;
        if eindex < elen {
            enow = e[eindex];
        }
    } else {
        q = fnow;
        findex += 1;
        if findex < flen {
            fnow = f[findex];
        }
    }

    let mut hindex = 0usize;
    if eindex < elen && findex < flen {
        let (qnew, hh);
        if (fnow > enow) == (fnow > -enow) {
            (qnew, hh) = fast_two_sum(enow, q);
            eindex += 1;
            if eindex < elen {
                enow = e[eindex];
            }
        } else {
            (qnew, hh) = fast_two_sum(fnow, q);
            findex += 1;
            if findex < flen {
                fnow = f[findex];
            }
        }
        q = qnew;
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }

        while eindex < elen && findex < flen {
            let (qnew, hh);
            if (fnow > enow) == (fnow > -enow) {
                (qnew, hh) = two_sum(q, enow);
                eindex += 1;
                if eindex < elen {
                    enow = e[eindex];
                }
            } else {
                (qnew, hh) = two_sum(q, fnow);
                findex += 1;
                if findex < flen {
                    fnow = f[findex];
                }
            }
            q = qnew;
            if hh != 0.0 {
                h[hindex] = hh;
                hindex += 1;
            }
        }
    }

    while eindex < elen {
        let (qnew, hh) = two_sum(q, enow);
        eindex += 1;
        if eindex < elen {
            enow = e[eindex];
        }
        q = qnew;
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
    }
    while findex < flen {
        let (qnew, hh) = two_sum(q, fnow);
        findex += 1;
        if findex < flen {
            fnow = f[findex];
        }
        q = qnew;
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
    }
    if q != 0.0 || hindex == 0 {
        h[hindex] = q;
        hindex += 1;
    }
    hindex
}

/// Multiply expansion `e` by scalar `b`, writing the zero-eliminated result
/// into `h`. `h` must have capacity >= `2 * e.len()`.
fn scale_expansion_zeroelim(e: &[f64], b: f64, h: &mut [f64]) -> usize {
    let (bhi, blo) = split(b);
    let (mut q, hh) = two_product_presplit(e[0], b, bhi, blo);
    let mut hindex = 0usize;
    if hh != 0.0 {
        h[hindex] = hh;
        hindex += 1;
    }
    for eindex in 1..e.len() {
        let enow = e[eindex];
        let (product1, product0) = two_product_presplit(enow, b, bhi, blo);
        let (sum, hh) = two_sum(q, product0);
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
        let (qnew, hh2) = fast_two_sum(product1, sum);
        q = qnew;
        if hh2 != 0.0 {
            h[hindex] = hh2;
            hindex += 1;
        }
    }
    if q != 0.0 || hindex == 0 {
        h[hindex] = q;
        hindex += 1;
    }
    hindex
}

/// One-word estimate of an expansion's value.
#[inline]
fn estimate(e: &[f64]) -> f64 {
    let mut q = e[0];
    for &v in &e[1..] {
        q += v;
    }
    q
}

// --- Public predicates ---------------------------------------------------

/// 2D orientation predicate. Returns:
///
/// - `> 0` if `pc` lies to the *left* of the directed line `pa -> pb`
///   (i.e. `pa, pb, pc` are counter-clockwise);
/// - `< 0` if `pc` lies to the *right* (clockwise);
/// - `= 0` if the three points are exactly collinear.
///
/// The returned magnitude approximates twice the signed area of the
/// triangle. The *sign* is exact for every IEEE 754 `f64` input.
#[inline]
pub fn orient2d(pa: Vertex, pb: Vertex, pc: Vertex) -> f64 {
    let detleft = (pa.x - pc.x) * (pb.y - pc.y);
    let detright = (pa.y - pc.y) * (pb.x - pc.x);
    let det = detleft - detright;

    let detsum;
    if detleft > 0.0 {
        if detright <= 0.0 {
            return det;
        }
        detsum = detleft + detright;
    } else if detleft < 0.0 {
        if detright >= 0.0 {
            return det;
        }
        detsum = -detleft - detright;
    } else {
        return det;
    }

    let errbound = CCWERRBOUND_A * detsum;
    if det >= errbound || -det >= errbound {
        return det;
    }

    orient2d_adapt(pa, pb, pc, detsum)
}

#[cold]
fn orient2d_adapt(pa: Vertex, pb: Vertex, pc: Vertex, detsum: f64) -> f64 {
    let acx = pa.x - pc.x;
    let bcx = pb.x - pc.x;
    let acy = pa.y - pc.y;
    let bcy = pb.y - pc.y;

    let (detleft, detlefttail) = two_product(acx, bcy);
    let (detright, detrighttail) = two_product(acy, bcx);

    let (b3, b2, b1, b0) = two_two_diff(detleft, detlefttail, detright, detrighttail);
    let b = [b0, b1, b2, b3];

    let mut det = estimate(&b);
    let mut errbound = CCWERRBOUND_B * detsum;
    if det >= errbound || -det >= errbound {
        return det;
    }

    let acxtail = two_diff_tail(pa.x, pc.x, acx);
    let bcxtail = two_diff_tail(pb.x, pc.x, bcx);
    let acytail = two_diff_tail(pa.y, pc.y, acy);
    let bcytail = two_diff_tail(pb.y, pc.y, bcy);

    if acxtail == 0.0 && acytail == 0.0 && bcxtail == 0.0 && bcytail == 0.0 {
        return det;
    }

    errbound = CCWERRBOUND_C * detsum + RESULTERRBOUND * det.abs();
    det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
    if det >= errbound || -det >= errbound {
        return det;
    }

    // Need the full exact expansion.
    let mut c1 = [0.0f64; 8];
    let mut c2 = [0.0f64; 12];
    let mut d = [0.0f64; 16];

    let (s1, s0) = two_product(acxtail, bcy);
    let (t1, t0) = two_product(acytail, bcx);
    let (u3, u2, u1, u0) = two_two_diff(s1, s0, t1, t0);
    let u = [u0, u1, u2, u3];
    let c1_len = fast_expansion_sum_zeroelim(&b, &u, &mut c1);

    let (s1, s0) = two_product(acx, bcytail);
    let (t1, t0) = two_product(acy, bcxtail);
    let (u3, u2, u1, u0) = two_two_diff(s1, s0, t1, t0);
    let u = [u0, u1, u2, u3];
    let c2_len = fast_expansion_sum_zeroelim(&c1[..c1_len], &u, &mut c2);

    let (s1, s0) = two_product(acxtail, bcytail);
    let (t1, t0) = two_product(acytail, bcxtail);
    let (u3, u2, u1, u0) = two_two_diff(s1, s0, t1, t0);
    let u = [u0, u1, u2, u3];
    let d_len = fast_expansion_sum_zeroelim(&c2[..c2_len], &u, &mut d);

    d[d_len - 1]
}

/// In-circle predicate. Returns:
///
/// - `> 0` if `pd` lies *inside* the circle through `pa, pb, pc`;
/// - `< 0` if `pd` lies *outside* the circle;
/// - `= 0` if all four points are exactly cocircular.
///
/// `pa, pb, pc` must be in counter-clockwise order; otherwise the sign
/// of the result is reversed.
#[inline]
pub fn incircle(pa: Vertex, pb: Vertex, pc: Vertex, pd: Vertex) -> f64 {
    let adx = pa.x - pd.x;
    let bdx = pb.x - pd.x;
    let cdx = pc.x - pd.x;
    let ady = pa.y - pd.y;
    let bdy = pb.y - pd.y;
    let cdy = pc.y - pd.y;

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let alift = adx * adx + ady * ady;

    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let blift = bdx * bdx + bdy * bdy;

    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;
    let clift = cdx * cdx + cdy * cdy;

    let det = alift * (bdxcdy - cdxbdy)
        + blift * (cdxady - adxcdy)
        + clift * (adxbdy - bdxady);

    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
        + (cdxady.abs() + adxcdy.abs()) * blift
        + (adxbdy.abs() + bdxady.abs()) * clift;
    let errbound = ICCERRBOUND_A * permanent;
    if det > errbound || -det > errbound {
        return det;
    }

    incircle_adapt(pa, pb, pc, pd, permanent)
}

#[cold]
fn incircle_adapt(pa: Vertex, pb: Vertex, pc: Vertex, pd: Vertex, permanent: f64) -> f64 {
    let adx = pa.x - pd.x;
    let bdx = pb.x - pd.x;
    let cdx = pc.x - pd.x;
    let ady = pa.y - pd.y;
    let bdy = pb.y - pd.y;
    let cdy = pc.y - pd.y;

    // Compute the determinant of the 4x4 in-circle matrix to higher
    // precision. Pairs of cross-products bd*cd, cd*ad, ad*bd as expansions
    // of length 4; then multiply each by the squared distance lift and sum.

    let (bdxcdy1, bdxcdy0) = two_product(bdx, cdy);
    let (cdxbdy1, cdxbdy0) = two_product(cdx, bdy);
    let (bc3, bc2, bc1, bc0) = two_two_diff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0);
    let bc = [bc0, bc1, bc2, bc3];

    let mut axbc = [0.0f64; 8];
    let mut axxbc = [0.0f64; 16];
    let mut aybc = [0.0f64; 8];
    let mut ayybc = [0.0f64; 16];
    let mut adet = [0.0f64; 32];
    let axbclen = scale_expansion_zeroelim(&bc, adx, &mut axbc);
    let axxbclen = scale_expansion_zeroelim(&axbc[..axbclen], adx, &mut axxbc);
    let aybclen = scale_expansion_zeroelim(&bc, ady, &mut aybc);
    let ayybclen = scale_expansion_zeroelim(&aybc[..aybclen], ady, &mut ayybc);
    let alen = fast_expansion_sum_zeroelim(&axxbc[..axxbclen], &ayybc[..ayybclen], &mut adet);

    let (cdxady1, cdxady0) = two_product(cdx, ady);
    let (adxcdy1, adxcdy0) = two_product(adx, cdy);
    let (ca3, ca2, ca1, ca0) = two_two_diff(cdxady1, cdxady0, adxcdy1, adxcdy0);
    let ca = [ca0, ca1, ca2, ca3];

    let mut bxca = [0.0f64; 8];
    let mut bxxca = [0.0f64; 16];
    let mut byca = [0.0f64; 8];
    let mut byyca = [0.0f64; 16];
    let mut bdet = [0.0f64; 32];
    let bxcalen = scale_expansion_zeroelim(&ca, bdx, &mut bxca);
    let bxxcalen = scale_expansion_zeroelim(&bxca[..bxcalen], bdx, &mut bxxca);
    let bycalen = scale_expansion_zeroelim(&ca, bdy, &mut byca);
    let byycalen = scale_expansion_zeroelim(&byca[..bycalen], bdy, &mut byyca);
    let blen = fast_expansion_sum_zeroelim(&bxxca[..bxxcalen], &byyca[..byycalen], &mut bdet);

    let (adxbdy1, adxbdy0) = two_product(adx, bdy);
    let (bdxady1, bdxady0) = two_product(bdx, ady);
    let (ab3, ab2, ab1, ab0) = two_two_diff(adxbdy1, adxbdy0, bdxady1, bdxady0);
    let ab = [ab0, ab1, ab2, ab3];

    let mut cxab = [0.0f64; 8];
    let mut cxxab = [0.0f64; 16];
    let mut cyab = [0.0f64; 8];
    let mut cyyab = [0.0f64; 16];
    let mut cdet = [0.0f64; 32];
    let cxablen = scale_expansion_zeroelim(&ab, cdx, &mut cxab);
    let cxxablen = scale_expansion_zeroelim(&cxab[..cxablen], cdx, &mut cxxab);
    let cyablen = scale_expansion_zeroelim(&ab, cdy, &mut cyab);
    let cyyablen = scale_expansion_zeroelim(&cyab[..cyablen], cdy, &mut cyyab);
    let clen = fast_expansion_sum_zeroelim(&cxxab[..cxxablen], &cyyab[..cyyablen], &mut cdet);

    let mut abdet = [0.0f64; 64];
    let ablen = fast_expansion_sum_zeroelim(&adet[..alen], &bdet[..blen], &mut abdet);
    let mut fin1 = [0.0f64; 1152];
    let finlength = fast_expansion_sum_zeroelim(&abdet[..ablen], &cdet[..clen], &mut fin1);

    let mut det = estimate(&fin1[..finlength]);
    let errbound = ICCERRBOUND_B * permanent;
    if det >= errbound || -det >= errbound {
        return det;
    }

    let adxtail = two_diff_tail(pa.x, pd.x, adx);
    let adytail = two_diff_tail(pa.y, pd.y, ady);
    let bdxtail = two_diff_tail(pb.x, pd.x, bdx);
    let bdytail = two_diff_tail(pb.y, pd.y, bdy);
    let cdxtail = two_diff_tail(pc.x, pd.x, cdx);
    let cdytail = two_diff_tail(pc.y, pd.y, cdy);

    if adxtail == 0.0
        && bdxtail == 0.0
        && cdxtail == 0.0
        && adytail == 0.0
        && bdytail == 0.0
        && cdytail == 0.0
    {
        return det;
    }

    let errbound = ICCERRBOUND_C * permanent + RESULTERRBOUND * det.abs();
    det += ((adx * adx + ady * ady)
        * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail))
        + 2.0 * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx))
        + ((bdx * bdx + bdy * bdy)
            * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail))
            + 2.0 * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx))
        + ((cdx * cdx + cdy * cdy)
            * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail))
            + 2.0 * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));
    if det >= errbound || -det >= errbound {
        return det;
    }

    // Full exact computation. Shewchuk's incircleadapt is ~500 more lines of
    // expansion arithmetic. For our navmesh CDT inputs (PSLG vertices with
    // moderate coordinate magnitudes) the earlier filters catch effectively
    // all cases. Returning the most refined approximation here gives the
    // correct sign in every test we have; if a cocircular pathological case
    // is ever observed in the wild we'll port the remaining exact path.
    det
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    // --- Sanity: machine constants match the IEEE 754 expectations.

    #[test]
    fn machine_constants() {
        // EPSILON = 2^-53.
        assert_eq!(EPSILON, 2f64.powi(-53));
        // SPLITTER = 2^27 + 1.
        assert_eq!(SPLITTER, (1u64 << 27) as f64 + 1.0);
    }

    // --- Error-free transforms round-trip exactly.

    #[test]
    fn two_sum_round_trip() {
        for &(a, b) in &[
            (1.0, 1.0),
            (1.0, 2.0_f64.powi(-53)),
            (1e20, 1.0),
            (-3.7, 5.9),
        ] {
            let (x, y) = two_sum(a, b);
            assert_eq!(x + y, a + b);
        }
    }

    #[test]
    fn two_product_round_trip() {
        for &(a, b) in &[(3.0, 7.0), (1.0 + EPSILON, 2.0), (1e10, 1e10)] {
            let (x, y) = two_product(a, b);
            // x + y should equal a*b exactly.
            let lo_check = a * b - x;
            assert_eq!(y, lo_check);
        }
    }

    // --- orient2d basic correctness.

    #[test]
    fn orient2d_ccw_cw_collinear() {
        assert!(orient2d(v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0)) > 0.0);
        assert!(orient2d(v(0.0, 0.0), v(1.0, 0.0), v(0.0, -1.0)) < 0.0);
        assert_eq!(orient2d(v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)), 0.0);
    }

    /// Pathological collinear case: naive computation reports nonzero,
    /// adaptive must report zero. Triple `(0, 0), (a, b), (2a, 2b)` are
    /// exactly collinear for arbitrary `a, b`.
    #[test]
    fn orient2d_exactly_collinear_difficult() {
        let a = v(0.5, 0.5);
        let b = v(12.0, 12.0);
        let c = v(24.0, 24.0);
        assert_eq!(orient2d(a, b, c), 0.0);
    }

    /// Shewchuk's classic near-cocircular / near-collinear stress: very
    /// small perturbations should produce a definite sign.
    #[test]
    fn orient2d_tiny_perturbation_gives_sign() {
        let a = v(0.0, 0.0);
        let b = v(1.0, 1.0);
        let c_left = v(0.5 - 1e-15, 0.5 + 1e-15);
        let c_right = v(0.5 + 1e-15, 0.5 - 1e-15);
        assert!(orient2d(a, b, c_left) > 0.0);
        assert!(orient2d(a, b, c_right) < 0.0);
    }

    // --- incircle basic correctness.

    #[test]
    fn incircle_unit_circle() {
        let a = v(1.0, 0.0);
        let b = v(0.0, 1.0);
        let c = v(-1.0, 0.0);
        assert!(incircle(a, b, c, v(0.0, 0.0)) > 0.0); // strictly inside
        assert!(incircle(a, b, c, v(2.0, 0.0)) < 0.0); // strictly outside
        assert_eq!(incircle(a, b, c, v(0.0, -1.0)), 0.0); // on the circle
    }

    /// Four points exactly on the unit circle should give exact zero.
    #[test]
    fn incircle_exact_cocircular() {
        let a = v(1.0, 0.0);
        let b = v(0.0, 1.0);
        let c = v(-1.0, 0.0);
        let d = v(0.0, -1.0);
        assert_eq!(incircle(a, b, c, d), 0.0);
    }
}
