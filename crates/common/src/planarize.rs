//! Snap-rounded planarization of an offset soup.
//!
//! Input: closed [`SoupContour`] rings that may cross themselves, each
//! other, overlap collinearly, or form T-junctions. Output:
//! [`PlanarSegments`] — a set of constraint segments that intersect at
//! most in shared endpoints, with every vertex on a fixed snap grid.
//! That is exactly the precondition `form_skeleton` needs to never hit
//! `SelfIntersection`.
//!
//! Method: iterated snap rounding to a fixed point (bounded rounds).
//! Each round finds proper crossings (split both segments at the snapped
//! intersection point), endpoints lying on other segments' interiors
//! (T-junctions — split the host), collinear overlaps (split both at
//! each other's interior endpoints; the duplicated middle pieces merge),
//! and the "hot cell" rule: any vertex closer than half a grid cell to a
//! segment's interior splits that segment through the vertex, so
//! snapping can never leave a near-miss T-junction behind.
//!
//! Determinism: coordinates snap to a power-of-two grid with
//! `(x / cell).round() * cell` (exact scaling, platform-independent),
//! vertex identity is `f64::to_bits` equality, candidate pairs are
//! processed in sorted index order, splits apply sorted by parameter
//! with coordinate-bit tie-breaks, and output order is first-occurrence
//! over that deterministic scan. No hash-iteration order ever reaches
//! the output.

use std::collections::hash_map::Entry;
use std::collections::HashMap;

use crate::geom::{nearest_point_on_segment, on_segment_collinear, orient2d};
use crate::offset::SoupContour;
use crate::{Aabb, Vertex};

/// A square snap grid whose cell size is always a power of two, so
/// `x / cell` and `* cell` are exact exponent shifts and snapping is
/// bit-deterministic across platforms and input orders.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SnapGrid {
    cell: f64,
}

impl SnapGrid {
    /// Largest power of two `<= target`. Panics if `target` is not
    /// finite and positive, or so small it has no normal power of two
    /// below it.
    pub fn from_target(target: f64) -> Self {
        assert!(
            target.is_finite() && target > 0.0,
            "SnapGrid target must be finite and > 0, got {target}"
        );
        // Zero the mantissa: keeps sign (positive) and exponent only.
        let cell = f64::from_bits(target.to_bits() & 0xFFF0_0000_0000_0000);
        assert!(cell > 0.0, "SnapGrid target {target} is subnormal");
        Self { cell }
    }

    /// Pick a cell for geometry in `bbox` eroded by `inset`.
    ///
    /// Bounds: an accuracy floor of ~2^12 ulps at the coordinate scale
    /// (absorbs naive intersection-point error), and a distortion
    /// ceiling of extent * 2^-32 so snap displacement stays orders of
    /// magnitude below the geometry. When the two conflict — tiny extent
    /// far from the origin — the floor wins: a slightly coarser grid
    /// beats cells below f64 resolution. With `inset > 0` the target is
    /// inset * 2^-20 (snap distortion ~5 orders below the erosion
    /// radius), clamped to those bounds.
    pub fn auto(bbox: &Aabb, inset: f64) -> Self {
        assert!(!bbox.is_empty(), "SnapGrid::auto on an empty Aabb");
        let scale = bbox
            .min
            .x
            .abs()
            .max(bbox.min.y.abs())
            .max(bbox.max.x.abs())
            .max(bbox.max.y.abs())
            .max(1.0);
        let lo = scale * (2f64).powi(-40); // 2^-52 ulp * 2^12
        let extent = bbox.width().max(bbox.height());
        let hi = (extent * (2f64).powi(-32)).max(lo);
        let target = if inset > 0.0 {
            (inset * (2f64).powi(-20)).max(lo).min(hi)
        } else {
            lo
        };
        Self::from_target(target)
    }

    /// Snap `v` to the grid. `-0.0` is canonicalized to `+0.0` so
    /// bit-equality dedup can't split the origin.
    pub fn snap(self, v: Vertex) -> Vertex {
        Vertex::new(self.snap1(v.x), self.snap1(v.y))
    }

    fn snap1(self, x: f64) -> f64 {
        ((x / self.cell).round() * self.cell) + 0.0
    }

    pub fn cell(self) -> f64 {
        self.cell
    }
}

/// Planarized constraint set: `segments` reference `vertices` by index,
/// carry the merged input marker, and pairwise intersect at most in
/// shared endpoints. All vertices are on the snap grid, deduplicated by
/// bit-exact position.
#[derive(Clone, Debug)]
pub struct PlanarSegments {
    pub vertices: Vec<Vertex>,
    pub segments: Vec<(u32, u32, i32)>,
}

/// Failure modes of [`planarize`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PlanarizeError {
    /// Snap rounding was still discovering new splits after the round
    /// bound; the input is adversarially degenerate for this grid.
    /// Surfaced instead of ever emitting a still-crossing segment set.
    NoConvergence,
    /// No usable segments: every contour collapsed under snapping.
    Degenerate,
}

impl std::fmt::Display for PlanarizeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoConvergence => {
                write!(f, "snap rounding did not converge within the round bound")
            }
            Self::Degenerate => write!(f, "every contour collapsed under snapping"),
        }
    }
}

impl std::error::Error for PlanarizeError {}

const MAX_ROUNDS: usize = 8;
/// Below this segment count the broad phase is brute force.
const BRUTE_FORCE_LIMIT: usize = 64;

/// [`planarize_with`] using the naive [`orient2d`]. Callers that own a
/// robust orientation predicate (the `triangle` crate) should inject it
/// via [`planarize_with`] instead — near-degenerate sign errors are the
/// one thing snap rounding cannot repair by itself.
pub fn planarize(
    contours: &[SoupContour],
    grid: SnapGrid,
) -> Result<PlanarSegments, PlanarizeError> {
    planarize_with(contours, grid, orient2d)
}

/// Planarize `contours` on `grid`, using `orient` for all orientation
/// sign decisions (crossing/T-junction/collinear classification).
pub fn planarize_with(
    contours: &[SoupContour],
    grid: SnapGrid,
    orient: fn(Vertex, Vertex, Vertex) -> f64,
) -> Result<PlanarSegments, PlanarizeError> {
    let mut pool = Pool::default();
    let mut segs: Vec<(u32, u32, i32)> = Vec::new();
    for c in contours {
        let ids: Vec<u32> = c.points.iter().map(|&p| pool.intern(grid.snap(p))).collect();
        let n = ids.len();
        for i in 0..n {
            let (a, b) = (ids[i], ids[(i + 1) % n]);
            if a != b {
                segs.push(canon(a, b, c.marker));
            }
        }
    }
    merge_dups(&mut segs);
    if segs.is_empty() {
        return Err(PlanarizeError::Degenerate);
    }

    for _round in 0..MAX_ROUNDS {
        let splits = find_splits(&segs, &mut pool, grid, orient);
        if splits.iter().all(Vec::is_empty) {
            return Ok(compact(&pool, &segs));
        }
        apply_splits(&mut segs, splits, &pool);
        merge_dups(&mut segs);
    }
    Err(PlanarizeError::NoConvergence)
}

/// Test/debug helper: verify that `ps` is actually planar — no proper
/// crossings, no endpoint interior to another segment, no collinear
/// overlap beyond shared endpoints, no duplicate or zero-length
/// segments. O(n^2); intended for tests and debug assertions.
pub fn verify_planar(
    ps: &PlanarSegments,
    orient: fn(Vertex, Vertex, Vertex) -> f64,
) -> Result<(), String> {
    let mut seen: HashMap<(u32, u32), ()> = HashMap::new();
    for (idx, &(a, b, _)) in ps.segments.iter().enumerate() {
        if a == b {
            return Err(format!("segment {idx} is zero-length (vertex {a})"));
        }
        let key = (a.min(b), a.max(b));
        if seen.insert(key, ()).is_some() {
            return Err(format!("duplicate segment {idx}: {key:?}"));
        }
    }
    for i in 0..ps.segments.len() {
        for j in (i + 1)..ps.segments.len() {
            let (a1i, a2i, _) = ps.segments[i];
            let (b1i, b2i, _) = ps.segments[j];
            let shared = |v: u32| v == b1i || v == b2i;
            let a1 = ps.vertices[a1i as usize];
            let a2 = ps.vertices[a2i as usize];
            let b1 = ps.vertices[b1i as usize];
            let b2 = ps.vertices[b2i as usize];
            let d1 = orient(b1, b2, a1);
            let d2 = orient(b1, b2, a2);
            let d3 = orient(a1, a2, b1);
            let d4 = orient(a1, a2, b2);
            if d1 == 0.0 && d2 == 0.0 && d3 == 0.0 && d4 == 0.0 {
                // Collinear: any endpoint strictly interior to the other
                // segment means they overlap in more than an endpoint.
                let overlap = (on_segment_collinear(a1, a2, b1) && b1 != a1 && b1 != a2)
                    || (on_segment_collinear(a1, a2, b2) && b2 != a1 && b2 != a2)
                    || (on_segment_collinear(b1, b2, a1) && a1 != b1 && a1 != b2)
                    || (on_segment_collinear(b1, b2, a2) && a2 != b1 && a2 != b2);
                if overlap {
                    return Err(format!("segments {i} and {j} overlap collinearly"));
                }
                continue;
            }
            let strict = d1 != 0.0
                && d2 != 0.0
                && d3 != 0.0
                && d4 != 0.0
                && (d1 > 0.0) != (d2 > 0.0)
                && (d3 > 0.0) != (d4 > 0.0);
            if strict {
                return Err(format!("segments {i} and {j} properly cross"));
            }
            // T-junction: an endpoint of one on the interior of the other.
            if d1 == 0.0 && !shared(a1i) && on_segment_collinear(b1, b2, a1) {
                return Err(format!("vertex {a1i} interior to segment {j}"));
            }
            if d2 == 0.0 && !shared(a2i) && on_segment_collinear(b1, b2, a2) {
                return Err(format!("vertex {a2i} interior to segment {j}"));
            }
            if d3 == 0.0 && b1i != a1i && b1i != a2i && on_segment_collinear(a1, a2, b1) {
                return Err(format!("vertex {b1i} interior to segment {i}"));
            }
            if d4 == 0.0 && b2i != a1i && b2i != a2i && on_segment_collinear(a1, a2, b2) {
                return Err(format!("vertex {b2i} interior to segment {i}"));
            }
        }
    }
    Ok(())
}

// --- internals -----------------------------------------------------------

#[derive(Default)]
struct Pool {
    verts: Vec<Vertex>,
    map: HashMap<(u64, u64), u32>,
}

impl Pool {
    fn intern(&mut self, v: Vertex) -> u32 {
        match self.map.entry((v.x.to_bits(), v.y.to_bits())) {
            Entry::Occupied(e) => *e.get(),
            Entry::Vacant(e) => {
                let id = self.verts.len() as u32;
                self.verts.push(v);
                e.insert(id);
                id
            }
        }
    }
}

fn canon(a: u32, b: u32, marker: i32) -> (u32, u32, i32) {
    (a.min(b), a.max(b), marker)
}

/// Smallest nonzero marker wins; zero only if both are zero.
fn merge_marker(m1: i32, m2: i32) -> i32 {
    if m1 == 0 {
        m2
    } else if m2 == 0 {
        m1
    } else {
        m1.min(m2)
    }
}

fn merge_dups(segs: &mut Vec<(u32, u32, i32)>) {
    let mut map: HashMap<(u32, u32), usize> = HashMap::with_capacity(segs.len());
    let mut out: Vec<(u32, u32, i32)> = Vec::with_capacity(segs.len());
    for &(a, b, m) in segs.iter() {
        match map.entry((a, b)) {
            Entry::Occupied(e) => {
                let i = *e.get();
                out[i].2 = merge_marker(out[i].2, m);
            }
            Entry::Vacant(e) => {
                e.insert(out.len());
                out.push((a, b, m));
            }
        }
    }
    *segs = out;
}

/// Parameter of collinear point `p` along `a -> b` via the dominant axis
/// (exact for distinct grid points on a shared line).
fn param_along(a: Vertex, b: Vertex, p: Vertex) -> f64 {
    let dx = (b.x - a.x).abs();
    let dy = (b.y - a.y).abs();
    if dx >= dy {
        (p.x - a.x) / (b.x - a.x)
    } else {
        (p.y - a.y) / (b.y - a.y)
    }
}

/// Broad phase: uniform buckets over segment AABBs expanded by half a
/// snap cell (so the same structure answers "segments near this vertex").
/// `None` buckets = brute force for small inputs.
struct BroadPhase {
    bsize: f64,
    origin: Vertex,
    buckets: Option<HashMap<(i64, i64), Vec<u32>>>,
    n_segs: u32,
}

impl BroadPhase {
    fn build(segs: &[(u32, u32, i32)], pool: &Pool, snap_cell: f64) -> Self {
        let n = segs.len();
        let aabb = Aabb::from_points(pool.verts.iter().copied());
        let origin = aabb.min;
        if n < BRUTE_FORCE_LIMIT {
            return Self {
                bsize: 1.0,
                origin,
                buckets: None,
                n_segs: n as u32,
            };
        }
        let extent = aabb.width().max(aabb.height()).max(snap_cell);
        let bsize = (extent / (n as f64).sqrt().ceil()).max(4.0 * snap_cell);
        let mut buckets: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
        let half = snap_cell * 0.5;
        for (idx, &(a, b, _)) in segs.iter().enumerate() {
            let (pa, pb) = (pool.verts[a as usize], pool.verts[b as usize]);
            let (x0, x1) = range_cells(pa.x.min(pb.x) - half, pa.x.max(pb.x) + half, origin.x, bsize);
            let (y0, y1) = range_cells(pa.y.min(pb.y) - half, pa.y.max(pb.y) + half, origin.y, bsize);
            for cx in x0..=x1 {
                for cy in y0..=y1 {
                    buckets.entry((cx, cy)).or_default().push(idx as u32);
                }
            }
        }
        Self {
            bsize,
            origin,
            buckets: Some(buckets),
            n_segs: n as u32,
        }
    }

    /// All candidate segment pairs, sorted and deduplicated.
    fn pairs(&self) -> Vec<(u32, u32)> {
        match &self.buckets {
            None => {
                let mut out = Vec::new();
                for i in 0..self.n_segs {
                    for j in (i + 1)..self.n_segs {
                        out.push((i, j));
                    }
                }
                out
            }
            Some(buckets) => {
                let mut out = Vec::new();
                for list in buckets.values() {
                    for (k, &i) in list.iter().enumerate() {
                        for &j in &list[k + 1..] {
                            out.push((i.min(j), i.max(j)));
                        }
                    }
                }
                out.sort_unstable();
                out.dedup();
                out
            }
        }
    }

    /// Candidate segments whose expanded AABB may contain `p`.
    fn candidates_for_point(&self, p: Vertex, out: &mut Vec<u32>) {
        out.clear();
        match &self.buckets {
            None => out.extend(0..self.n_segs),
            Some(buckets) => {
                let cx = cell_of(p.x, self.origin.x, self.bsize);
                let cy = cell_of(p.y, self.origin.y, self.bsize);
                if let Some(list) = buckets.get(&(cx, cy)) {
                    out.extend_from_slice(list);
                }
            }
        }
    }
}

fn cell_of(x: f64, origin: f64, bsize: f64) -> i64 {
    ((x - origin) / bsize).floor() as i64
}

fn range_cells(lo: f64, hi: f64, origin: f64, bsize: f64) -> (i64, i64) {
    (cell_of(lo, origin, bsize), cell_of(hi, origin, bsize))
}

fn find_splits(
    segs: &[(u32, u32, i32)],
    pool: &mut Pool,
    grid: SnapGrid,
    orient: fn(Vertex, Vertex, Vertex) -> f64,
) -> Vec<Vec<(f64, u32)>> {
    let mut splits: Vec<Vec<(f64, u32)>> = vec![Vec::new(); segs.len()];
    let bp = BroadPhase::build(segs, pool, grid.cell());

    for (i, j) in bp.pairs() {
        process_pair(i as usize, j as usize, segs, pool, grid, orient, &mut splits);
    }

    // Hot-cell rule: a vertex within cell/2 of a segment's interior
    // splits that segment through the vertex. Two distinct grid points
    // are >= cell apart, so a near endpoint can never trigger this —
    // only genuine interior near-misses do.
    let half = grid.cell() * 0.5;
    let half_sq = half * half;
    let mut cands: Vec<u32> = Vec::new();
    for vid in 0..pool.verts.len() as u32 {
        let p = pool.verts[vid as usize];
        bp.candidates_for_point(p, &mut cands);
        for &s in &cands {
            let (a, b, _) = segs[s as usize];
            if vid == a || vid == b {
                continue;
            }
            let (pa, pb) = (pool.verts[a as usize], pool.verts[b as usize]);
            let nearest = nearest_point_on_segment(pa, pb, p);
            if nearest.distance_sq(p) < half_sq {
                let len_sq = pa.distance_sq(pb);
                let t = (p - pa).dot(pb - pa) / len_sq;
                if t > 0.0 && t < 1.0 {
                    splits[s as usize].push((t, vid));
                }
            }
        }
    }
    splits
}

#[allow(clippy::too_many_arguments)]
fn process_pair(
    i: usize,
    j: usize,
    segs: &[(u32, u32, i32)],
    pool: &mut Pool,
    grid: SnapGrid,
    orient: fn(Vertex, Vertex, Vertex) -> f64,
    splits: &mut [Vec<(f64, u32)>],
) {
    let (a1i, a2i, _) = segs[i];
    let (b1i, b2i, _) = segs[j];
    let a1 = pool.verts[a1i as usize];
    let a2 = pool.verts[a2i as usize];
    let b1 = pool.verts[b1i as usize];
    let b2 = pool.verts[b2i as usize];

    let d1 = orient(b1, b2, a1);
    let d2 = orient(b1, b2, a2);
    let d3 = orient(a1, a2, b1);
    let d4 = orient(a1, a2, b2);

    if d1 == 0.0 && d2 == 0.0 && d3 == 0.0 && d4 == 0.0 {
        // Collinear: split each segment at the other's interior endpoints.
        // Interior-ness on the shared line is exact on the dominant axis.
        for (wid, w) in [(b1i, b1), (b2i, b2)] {
            if wid != a1i && wid != a2i && on_segment_collinear(a1, a2, w) {
                splits[i].push((param_along(a1, a2, w), wid));
            }
        }
        for (wid, w) in [(a1i, a1), (a2i, a2)] {
            if wid != b1i && wid != b2i && on_segment_collinear(b1, b2, w) {
                splits[j].push((param_along(b1, b2, w), wid));
            }
        }
        return;
    }

    let strict = d1 != 0.0
        && d2 != 0.0
        && d3 != 0.0
        && d4 != 0.0
        && (d1 > 0.0) != (d2 > 0.0)
        && (d3 > 0.0) != (d4 > 0.0);
    if strict {
        // Proper crossing. Parameters from the (robust-sign) orient
        // magnitudes; the point is snapped, so parameter error only
        // affects split ordering, which the sort tie-break absorbs.
        let t = d1 / (d1 - d2);
        let u = d3 / (d3 - d4);
        let p = grid.snap(a1.lerp(a2, t));
        let vid = pool.intern(p);
        if vid != a1i && vid != a2i {
            splits[i].push((t, vid));
        }
        if vid != b1i && vid != b2i {
            splits[j].push((u, vid));
        }
        return;
    }

    // T-junctions: an endpoint exactly on the other segment's interior.
    if d1 == 0.0 && a1i != b1i && a1i != b2i && on_segment_collinear(b1, b2, a1) {
        splits[j].push((param_along(b1, b2, a1), a1i));
    }
    if d2 == 0.0 && a2i != b1i && a2i != b2i && on_segment_collinear(b1, b2, a2) {
        splits[j].push((param_along(b1, b2, a2), a2i));
    }
    if d3 == 0.0 && b1i != a1i && b1i != a2i && on_segment_collinear(a1, a2, b1) {
        splits[i].push((param_along(a1, a2, b1), b1i));
    }
    if d4 == 0.0 && b2i != a1i && b2i != a2i && on_segment_collinear(a1, a2, b2) {
        splits[i].push((param_along(a1, a2, b2), b2i));
    }
}

fn apply_splits(
    segs: &mut Vec<(u32, u32, i32)>,
    splits: Vec<Vec<(f64, u32)>>,
    pool: &Pool,
) {
    let mut out: Vec<(u32, u32, i32)> = Vec::with_capacity(segs.len() * 2);
    for (idx, &(a, b, m)) in segs.iter().enumerate() {
        let mut list = splits[idx].clone();
        if list.is_empty() {
            out.push((a, b, m));
            continue;
        }
        list.sort_by(|x, y| {
            x.0.partial_cmp(&y.0).unwrap().then_with(|| {
                let vx = pool.verts[x.1 as usize];
                let vy = pool.verts[y.1 as usize];
                (vx.x.to_bits(), vx.y.to_bits()).cmp(&(vy.x.to_bits(), vy.y.to_bits()))
            })
        });
        let mut used: Vec<u32> = Vec::with_capacity(list.len());
        let mut prev = a;
        for &(_, v) in &list {
            if v == a || v == b || used.contains(&v) {
                continue;
            }
            used.push(v);
            if v != prev {
                out.push(canon(prev, v, m));
                prev = v;
            }
        }
        if prev != b {
            out.push(canon(prev, b, m));
        }
    }
    *segs = out;
}

/// Re-index vertices in first-reference order over the final segment
/// scan, dropping any that ended up unused.
fn compact(pool: &Pool, segs: &[(u32, u32, i32)]) -> PlanarSegments {
    let mut remap: Vec<u32> = vec![u32::MAX; pool.verts.len()];
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut segments: Vec<(u32, u32, i32)> = Vec::with_capacity(segs.len());
    for &(a, b, m) in segs {
        for id in [a, b] {
            if remap[id as usize] == u32::MAX {
                remap[id as usize] = vertices.len() as u32;
                vertices.push(pool.verts[id as usize]);
            }
        }
        segments.push((remap[a as usize], remap[b as usize], m));
    }
    PlanarSegments { vertices, segments }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Lcg;

    fn contour(pts: &[(f64, f64)], marker: i32) -> SoupContour {
        SoupContour {
            points: pts.iter().map(|&(x, y)| Vertex::new(x, y)).collect(),
            marker,
        }
    }

    fn grid1() -> SnapGrid {
        SnapGrid::from_target(1.0)
    }

    /// Canonical, order-independent view of the output: segments as
    /// coordinate-bit tuples, sorted.
    fn canonical(ps: &PlanarSegments) -> Vec<((u64, u64), (u64, u64), i32)> {
        let key = |v: Vertex| (v.x.to_bits(), v.y.to_bits());
        let mut out: Vec<_> = ps
            .segments
            .iter()
            .map(|&(a, b, m)| {
                let ka = key(ps.vertices[a as usize]);
                let kb = key(ps.vertices[b as usize]);
                (ka.min(kb), ka.max(kb), m)
            })
            .collect();
        out.sort_unstable();
        out
    }

    fn vertex_id(ps: &PlanarSegments, x: f64, y: f64) -> Option<u32> {
        ps.vertices
            .iter()
            .position(|v| v.x == x && v.y == y)
            .map(|i| i as u32)
    }

    #[test]
    fn bowtie_x_crossing_splits_both_edges() {
        // Self-intersecting quad: edges (0,0)-(10,10) and (10,0)-(0,10)
        // cross at (5,5). Both split; 4 segments meet at the crossing.
        let bow = contour(&[(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)], 7);
        let ps = planarize(&[bow], grid1()).unwrap();
        verify_planar(&ps, orient2d).unwrap();
        assert_eq!(ps.segments.len(), 6);
        let c = vertex_id(&ps, 5.0, 5.0).expect("crossing vertex missing");
        let incident = ps
            .segments
            .iter()
            .filter(|&&(a, b, _)| a == c || b == c)
            .count();
        assert_eq!(incident, 4, "4 segments must share the crossing vertex");
        assert!(ps.segments.iter().all(|&(_, _, m)| m == 7));
    }

    #[test]
    fn t_junction_splits_host_only() {
        // B's vertex (5,0) sits on the interior of A's bottom edge:
        // that edge splits in two, B is unchanged. 4+1+3 = 8 segments.
        let a = contour(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], 1);
        let b = contour(&[(5.0, 0.0), (15.0, -6.0), (15.0, -2.0)], 2);
        let ps = planarize(&[a, b], grid1()).unwrap();
        verify_planar(&ps, orient2d).unwrap();
        assert_eq!(ps.segments.len(), 8);
        let t = vertex_id(&ps, 5.0, 0.0).unwrap();
        let incident = ps
            .segments
            .iter()
            .filter(|&&(x, y, _)| x == t || y == t)
            .count();
        // Two halves of A's bottom edge + two B edges from the vertex.
        assert_eq!(incident, 4);
    }

    #[test]
    fn collinear_overlap_merges_with_min_nonzero_marker() {
        // A's right edge (x=10, y 0..10) and B's left edge (x=10, y 2..8)
        // overlap; the shared piece appears once with min marker 3.
        let a = contour(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], 5);
        let b = contour(&[(10.0, 2.0), (20.0, 2.0), (20.0, 8.0), (10.0, 8.0)], 3);
        let ps = planarize(&[a, b], grid1()).unwrap();
        verify_planar(&ps, orient2d).unwrap();
        assert_eq!(ps.segments.len(), 9);
        let lo = vertex_id(&ps, 10.0, 2.0).unwrap();
        let hi = vertex_id(&ps, 10.0, 8.0).unwrap();
        let shared = ps
            .segments
            .iter()
            .find(|&&(x, y, _)| (x == lo && y == hi) || (x == hi && y == lo))
            .expect("merged overlap piece missing");
        assert_eq!(shared.2, 3, "smallest nonzero marker must win");
    }

    #[test]
    fn zero_marker_loses_merge() {
        // Same segment from two 2-point rings, markers 0 and 4 -> 4.
        let a = contour(&[(0.0, 0.0), (10.0, 0.0)], 0);
        let b = contour(&[(0.0, 0.0), (10.0, 0.0)], 4);
        let ps = planarize(&[a, b], grid1()).unwrap();
        assert_eq!(ps.segments.len(), 1);
        assert_eq!(ps.segments[0].2, 4);
    }

    #[test]
    fn near_miss_vertex_bends_segment_through_it() {
        // B's vertex (5,0) passes within cell/2 of A's edge
        // (0,0)-(10,1): the edge must bend through the vertex.
        let a = contour(&[(0.0, 0.0), (10.0, 1.0), (0.0, 5.0)], 1);
        let b = contour(&[(5.0, 0.0), (15.0, -6.0), (15.0, -2.0)], 2);
        let ps = planarize(&[a, b], grid1()).unwrap();
        verify_planar(&ps, orient2d).unwrap();
        let v = vertex_id(&ps, 5.0, 0.0).unwrap();
        let incident = ps
            .segments
            .iter()
            .filter(|&&(x, y, _)| x == v || y == v)
            .count();
        assert_eq!(incident, 4, "edge must bend through the near vertex");
    }

    #[test]
    fn idempotent_under_reprocessing() {
        let a = contour(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], 5);
        let b = contour(&[(10.0, 2.0), (20.0, 2.0), (20.0, 8.0), (10.0, 8.0)], 3);
        let ps1 = planarize(&[a, b], grid1()).unwrap();
        // Re-feed each output segment as a 2-point contour.
        let rewrapped: Vec<SoupContour> = ps1
            .segments
            .iter()
            .map(|&(x, y, m)| SoupContour {
                points: vec![ps1.vertices[x as usize], ps1.vertices[y as usize]],
                marker: m,
            })
            .collect();
        let ps2 = planarize(&rewrapped, grid1()).unwrap();
        assert_eq!(canonical(&ps1), canonical(&ps2));
    }

    #[test]
    fn contour_order_does_not_change_geometry() {
        let a = contour(&[(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)], 1);
        let b = contour(&[(2.0, -3.0), (12.0, 7.0), (2.0, 7.0)], 2);
        let ab = planarize(&[a.clone(), b.clone()], grid1()).unwrap();
        let ba = planarize(&[b, a], grid1()).unwrap();
        verify_planar(&ab, orient2d).unwrap();
        assert_eq!(canonical(&ab), canonical(&ba));
    }

    #[test]
    fn same_input_bit_identical_output() {
        let a = contour(&[(0.1, 0.2), (9.7, 10.3), (10.2, 0.1), (0.3, 9.8)], 1);
        let b = contour(&[(2.0, -3.0), (12.0, 7.0), (2.0, 7.0)], 2);
        let g = SnapGrid::from_target(0.01);
        let r1 = planarize(&[a.clone(), b.clone()], g).unwrap();
        let r2 = planarize(&[a, b], g).unwrap();
        assert_eq!(r1.vertices.len(), r2.vertices.len());
        for (v1, v2) in r1.vertices.iter().zip(&r2.vertices) {
            assert_eq!(v1.x.to_bits(), v2.x.to_bits());
            assert_eq!(v1.y.to_bits(), v2.y.to_bits());
        }
        assert_eq!(r1.segments, r2.segments);
    }

    #[test]
    fn random_soup_converges_and_verifies() {
        // Deterministic fuzz: crossing triangles all over each other.
        // Exercises the bucketed broad phase too (>= 64 segments).
        let mut rng = Lcg(0xC0FFEE);
        let mut contours = Vec::new();
        for i in 0..40 {
            let cx = rng.next_f64() * 100.0;
            let cy = rng.next_f64() * 100.0;
            let mut pts = Vec::new();
            for _ in 0..3 {
                pts.push((
                    cx + (rng.next_f64() - 0.5) * 60.0,
                    cy + (rng.next_f64() - 0.5) * 60.0,
                ));
            }
            contours.push(contour(&pts, 1 + i));
        }
        let g = SnapGrid::from_target(0.015625);
        let ps = planarize(&contours, g).expect("must converge within the round bound");
        verify_planar(&ps, orient2d).unwrap();
        // Every vertex is on the grid (snap idempotence).
        for v in &ps.vertices {
            let s = g.snap(*v);
            assert_eq!(v.x.to_bits(), s.x.to_bits());
            assert_eq!(v.y.to_bits(), s.y.to_bits());
        }
    }

    #[test]
    fn degenerate_input_is_reported() {
        let c = contour(&[(1.0, 1.0), (1.2, 1.3), (0.8, 0.9)], 1);
        // Everything snaps to (1,1) on a cell-1 grid.
        assert_eq!(
            planarize(&[c], grid1()).unwrap_err(),
            PlanarizeError::Degenerate
        );
        assert_eq!(
            planarize(&[], grid1()).unwrap_err(),
            PlanarizeError::Degenerate
        );
    }

    #[test]
    fn snap_grid_far_from_origin_no_panic() {
        // 30-unit room at x ~ 10_000: the raw clamp form would panic
        // (accuracy floor above the distortion ceiling). The order-safe
        // form must yield a finite positive power-of-two cell.
        let bbox = Aabb::from_points([
            Vertex::new(10_000.0, 10_000.0),
            Vertex::new(10_030.0, 10_030.0),
        ]);
        for inset in [0.0, 0.128, 15.0] {
            let g = SnapGrid::auto(&bbox, inset);
            let c = g.cell();
            assert!(c.is_finite() && c > 0.0, "bad cell {c} at inset {inset}");
            // Power of two: mantissa bits all zero.
            assert_eq!(c.to_bits() & 0x000F_FFFF_FFFF_FFFF, 0, "not a power of two: {c}");
        }
    }

    #[test]
    fn snap_grid_from_target_prev_power_of_two() {
        assert_eq!(SnapGrid::from_target(1.0).cell(), 1.0);
        assert_eq!(SnapGrid::from_target(1.5).cell(), 1.0);
        assert_eq!(SnapGrid::from_target(0.7).cell(), 0.5);
        assert_eq!(SnapGrid::from_target(6.0).cell(), 4.0);
    }

    #[test]
    fn snap_canonicalizes_negative_zero() {
        let g = grid1();
        let v = g.snap(Vertex::new(-0.3, -0.4));
        assert_eq!(v.x.to_bits(), 0.0f64.to_bits());
        assert_eq!(v.y.to_bits(), 0.0f64.to_bits());
    }
}
