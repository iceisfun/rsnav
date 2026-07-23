//! Winding-number triangle classification: the cull stage of the inset
//! pipeline.
//!
//! Instead of flood-filling from hole seed points ([`carve_holes`]),
//! every live triangle is classified independently: compute the signed
//! winding number of its centroid against the *original* (un-planarized)
//! soup contours and keep the triangle iff winding `>= 1`. Perimeter
//! contours (CCW) contribute `+1`, holes (CW) `-1`, and a lobe that
//! flipped during offsetting cancels itself by its reversed orientation
//! — no lobe detection anywhere. Because classification is purely local,
//! islands, merged holes, and regions that split apart all come out
//! right with zero extra logic.
//!
//! Classification is the pipeline's hot spot: naively it is one
//! [`winding_number`] call per live triangle, and that call walks every
//! edge of every soup contour, so the cost is `triangles x soup_edges`
//! (on `act3-town`: 40k x 22.6k robust orientation tests, ~1.5s).
//! `WindingIndex` collapses it to the edges that can actually
//! contribute, and does so *exactly*: the accumulator in
//! [`winding_number`] is an `i32` incremented by literal `+/-1`, never a
//! float, so dropping terms that are provably zero — and visiting the
//! rest in any order — is bit-identical by construction. The three
//! filters are exact restatements of the two branches, not
//! approximations: a horizontal edge satisfies neither branch; both
//! branches require `min(a.y,b.y) <= p.y < max(a.y,b.y)`; and if
//! `p.x > max(a.x,b.x)` then `p` is strictly right of the supporting
//! line, so the exact predicate's `> 0.0` and `< 0.0` tests both fail.
//! The uniform y-bucket index is therefore a *candidate* filter only —
//! bucket geometry is never trusted for a decision, which is why the
//! bucket count, origin and scale cannot affect the result.
//!
//! [`carve_holes`]: crate::carve_holes

use rsnav_common::{SoupContour, Vertex};

use crate::holes::sweep;
use crate::mesh::{CdtMesh, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::predicates::orient2d;
use crate::segment::insert_subseg;

/// Signed winding number of `p` with respect to `contours` (Sunday's
/// crossing count, half-open in `y` so a ray through a vertex is never
/// double-counted). Side tests use the robust [`orient2d`], making "on
/// the contour" the only ambiguous case — and the half-open rule
/// resolves that deterministically.
pub fn winding_number(p: Vertex, contours: &[SoupContour]) -> i32 {
    let mut wn = 0i32;
    for c in contours {
        let pts = &c.points;
        let n = pts.len();
        for i in 0..n {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            if a.y <= p.y {
                if b.y > p.y && orient2d(a, b, p) > 0.0 {
                    wn += 1; // upward edge, p strictly left
                }
            } else if b.y <= p.y && orient2d(a, b, p) < 0.0 {
                wn -= 1; // downward edge, p strictly right
            }
        }
    }
    wn
}

/// One edge's contribution to the crossing count — a literal
/// transcription of [`winding_number`]'s two branches (same half-open
/// comparisons, same `orient2d` argument order, same strict sign tests).
#[inline]
fn contrib(a: Vertex, b: Vertex, p: Vertex) -> i32 {
    if a.y <= p.y {
        if b.y > p.y && orient2d(a, b, p) > 0.0 {
            1
        } else {
            0
        }
    } else if b.y <= p.y && orient2d(a, b, p) < 0.0 {
        -1
    } else {
        0
    }
}

/// Bucket of `y` in a `nb`-bin uniform partition anchored at `y0` with
/// reciprocal bin height `inv_h`.
///
/// Both the build and the query path must call this verbatim. Not
/// because the result depends on it — the bins only choose *candidates*,
/// and every decision is made by the exact `ylo`/`yhi`/`xmax` tests and
/// [`orient2d`] — but because a divergence between two copies (a
/// strength reduction of one, say) could make the queried bucket stop
/// being a superset of the contributing edges. Correctness rests on
/// monotonicity: IEEE subtract, multiply, `floor` and `clamp` are each
/// monotone and Rust never contracts or reassociates them, so an edge
/// filed in `bin(ylo)..=bin(yhi)` is in `bin(p.y)` for every `p.y` in
/// `[ylo, yhi]`.
#[inline]
fn bin(y: f64, y0: f64, inv_h: f64, nb: usize) -> usize {
    let k = ((y - y0) * inv_h).floor();
    if k < 0.0 || k.is_nan() {
        0
    } else if k >= nb as f64 {
        nb - 1
    } else {
        (k as usize).min(nb - 1)
    }
}

/// Structure-of-arrays soup edges plus a coarse uniform y-bucket CSR
/// index, built once per [`carve_by_winding`] call and thrown away with
/// it.
///
/// The split is by access frequency: `ylo`/`yhi`/`xmax` are read for
/// every bucket candidate, while `a`/`b`/`up` are touched only by the
/// quarter that survives the filters and reaches the predicate.
/// Horizontal edges are dropped at build time (28.5% of `act3-town`'s
/// soup) since neither branch of [`winding_number`] can fire when
/// `a.y == b.y`.
struct WindingIndex {
    ylo: Vec<f64>,
    yhi: Vec<f64>,
    xmax: Vec<f64>,
    a: Vec<Vertex>,
    b: Vec<Vertex>,
    up: Vec<bool>,
    y0: f64,
    inv_h: f64,
    nb: usize,
    starts: Vec<u32>,
    items: Vec<u32>,
    /// Edges with a non-finite coordinate, scanned brute-force on every
    /// query. Normally empty, and load-bearing rather than defensive:
    /// `f64::min`/`max` silently drop NaN, so a NaN `a.y` would collapse
    /// to `ylo == yhi` and be filtered out, while [`winding_number`]
    /// takes its `else` branch (`a.y <= p.y` is false for NaN) and can
    /// still emit `-1`. Diverting them here means no float reasoning is
    /// ever needed about non-finite input.
    fallback: Vec<(Vertex, Vertex)>,
}

impl WindingIndex {
    fn build(contours: &[SoupContour]) -> Self {
        Self::build_inner(contours, None)
    }

    fn build_inner(contours: &[SoupContour], nb_override: Option<usize>) -> Self {
        let mut idx = WindingIndex {
            ylo: Vec::new(),
            yhi: Vec::new(),
            xmax: Vec::new(),
            a: Vec::new(),
            b: Vec::new(),
            up: Vec::new(),
            y0: 0.0,
            inv_h: 0.0,
            nb: 0,
            starts: vec![0],
            items: Vec::new(),
            fallback: Vec::new(),
        };

        // Same iteration as winding_number: every ring edge, in order.
        for c in contours {
            let pts = &c.points;
            let n = pts.len();
            for i in 0..n {
                let a = pts[i];
                let b = pts[(i + 1) % n];
                if !(a.x.is_finite() && a.y.is_finite() && b.x.is_finite() && b.y.is_finite()) {
                    idx.fallback.push((a, b));
                    continue;
                }
                if a.y == b.y {
                    continue;
                }
                idx.ylo.push(a.y.min(b.y));
                idx.yhi.push(a.y.max(b.y));
                idx.xmax.push(a.x.max(b.x));
                idx.a.push(a);
                idx.b.push(b);
                idx.up.push(a.y < b.y);
            }
        }

        let n = idx.ylo.len();
        if n == 0 {
            return idx;
        }
        // Edge ids are stored as u32 in the CSR item list.
        debug_assert!(n < u32::MAX as usize);

        let mut y0 = f64::INFINITY;
        let mut y1 = f64::NEG_INFINITY;
        for i in 0..n {
            y0 = y0.min(idx.ylo[i]);
            y1 = y1.max(idx.yhi[i]);
        }
        let mut nb = nb_override.unwrap_or((n / 16).clamp(1, 4096));
        let mut inv_h;
        if y1 <= y0 || y1.is_nan() || y0.is_nan() {
            // Zero (or degenerate) y extent: one bin, and no reciprocal
            // — `1.0/0.0` is inf and `0.0 * inf` is NaN.
            nb = 1;
            inv_h = 0.0;
        } else {
            inv_h = 1.0 / ((y1 - y0) / nb as f64);
        }
        // An edge is filed into every bucket its y span touches, so the
        // CSR item count is `sum(span)`, bounded only by `n * nb` — for a
        // soup of tall edges that is quadratic in `n` and would dwarf the
        // mesh itself. Bitfield-derived soups are short-edged and never
        // come close, so rather than coarsen `nb` unconditionally, price
        // the index first (one O(n) pass) and shrink `nb` only if it does
        // not fit the budget. Spans scale with `nb`, so one rescale
        // suffices: it lands at `<= BUDGET + n` items. Purely a memory
        // knob — like `nb` itself it cannot change any winding number.
        const BUDGET: usize = 1 << 23;
        if nb > 1 {
            let mut total = 0usize;
            for i in 0..n {
                total += bin(idx.yhi[i], y0, inv_h, nb) - bin(idx.ylo[i], y0, inv_h, nb) + 1;
            }
            if total > BUDGET {
                nb = ((nb as u128 * BUDGET as u128) / total as u128).max(1) as usize;
                inv_h = 1.0 / ((y1 - y0) / nb as f64);
            }
        }
        idx.y0 = y0;
        idx.inv_h = inv_h;
        idx.nb = nb;

        // Two-pass counting sort into CSR; no hashing, no sorting.
        let mut starts = vec![0u32; nb + 1];
        for i in 0..n {
            let lo = bin(idx.ylo[i], y0, inv_h, nb);
            let hi = bin(idx.yhi[i], y0, inv_h, nb);
            for slot in starts.iter_mut().take(hi + 1).skip(lo) {
                *slot += 1;
            }
        }
        // `items` holds one entry per (edge, spanned bucket) pair, so its
        // length is bounded only by `n * nb`; accumulate in usize so the
        // prefix sum cannot wrap the u32 the CSR stores.
        let mut acc = 0usize;
        for slot in starts.iter_mut() {
            let c = *slot as usize;
            *slot = acc as u32;
            acc += c;
            debug_assert!(acc < u32::MAX as usize);
        }
        let mut items = vec![0u32; acc];
        let mut cursor = starts.clone();
        for i in 0..n {
            let lo = bin(idx.ylo[i], y0, inv_h, nb);
            let hi = bin(idx.yhi[i], y0, inv_h, nb);
            for b in lo..=hi {
                items[cursor[b] as usize] = i as u32;
                cursor[b] += 1;
            }
        }
        idx.starts = starts;
        idx.items = items;
        idx
    }

    /// Signed winding number of `p`, identical to
    /// `winding_number(p, contours)` for the `contours` this was built
    /// from.
    fn winding(&self, p: Vertex) -> i32 {
        let mut wn = 0i32;
        // Before any early return: the fallback edges are outside every
        // filter's reasoning and must always be counted.
        for &(a, b) in &self.fallback {
            wn += contrib(a, b, p);
        }
        if self.nb == 0 {
            return wn;
        }
        // Must be `bin` verbatim (see its doc comment): clamping out-of-
        // extent `p.y` into the end buckets rather than returning early
        // is what keeps the queried bucket a superset of the
        // contributing edges. Rounding in `inv_h` can put a `p.y` that
        // is *strictly inside* the extent one bucket past the end, so an
        // early return there would drop live edges. Clamping instead is
        // free: an out-of-extent `p` is rejected by the exact `ylo`/
        // `yhi` tests below, and a NaN `p.y` bins to 0 and then fails
        // every predicate, matching `winding_number`'s fall-through.
        let bb = bin(p.y, self.y0, self.inv_h, self.nb);
        let lo = self.starts[bb] as usize;
        let hi = self.starts[bb + 1] as usize;
        for &ei in &self.items[lo..hi] {
            let i = ei as usize;
            if p.y < self.ylo[i] || p.y >= self.yhi[i] || p.x > self.xmax[i] {
                continue;
            }
            let o = orient2d(self.a[i], self.b[i], p);
            if self.up[i] {
                if o > 0.0 {
                    wn += 1;
                }
            } else if o < 0.0 {
                wn -= 1;
            }
        }
        wn
    }
}

/// Kill every live triangle whose centroid has winding `< 1` against
/// `contours`, maintaining subseg bookkeeping exactly like the plague
/// carve: subsegs between two dying triangles die, subsegs on the
/// kept/killed boundary survive with a promoted marker, and any exposed
/// boundary edge that ends up without a subseg gets a marker-1 backfill
/// so no wall is ever silently unmarked.
///
/// The keep rule is `winding >= 1` (not `== 1`): overlapping authored
/// perimeters produce winding 2 and must stay walkable.
///
/// Returns the number of triangles killed.
pub fn carve_by_winding(mesh: &mut CdtMesh, contours: &[SoupContour]) -> usize {
    // Classification: deterministic slot-order scan, no worklist.
    let mut infected: Vec<bool> = vec![false; mesh.triangles.len()];
    let idx = WindingIndex::build(contours);
    for tri_idx in 1..mesh.triangles.len() as u32 {
        let slot = mesh.triangle(tri_idx);
        if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        let a = mesh.vertex_pos(slot.vertices[0]);
        let b = mesh.vertex_pos(slot.vertices[1]);
        let c = mesh.vertex_pos(slot.vertices[2]);
        let centroid = Vertex::new((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
        // Permanent differential oracle: free in release, and it turns
        // every inset test in the workspace (the seeded 25-scene sweep
        // included) into an equivalence proof over real geometry.
        debug_assert_eq!(
            idx.winding(centroid),
            winding_number(centroid, contours),
            "winding index/brute divergence at {centroid:?}"
        );
        if idx.winding(centroid) < 1 {
            infected[tri_idx as usize] = true;
        }
    }
    // Ghost triangles (hull fans) always die.
    for tri_idx in 1..mesh.triangles.len() as u32 {
        let slot = mesh.triangle(tri_idx);
        if !slot.is_dead() && !slot.vertices.iter().all(|v| v.is_valid()) {
            infected[tri_idx as usize] = true;
        }
    }

    cleanup_boundary_subsegs(mesh, &infected);
    let killed = sweep(mesh, &infected);
    backfill_boundary_subsegs(mesh);
    killed
}

/// Kill subsegs whose marker is in `soup_markers` and that separate two
/// *live* triangles — interior constraints left over from soup contours
/// (e.g. a dilated hole's lobe boundary crossing kept area). Left in
/// place they would split regions and fabricate walls.
///
/// `soup_markers` must be a **sorted, deduplicated slice of the exact
/// marker values** carried by the soup contours; membership is a binary
/// search. It is never a numeric-range predicate: marker schemes differ
/// per caller (the demo loads perimeters as `(i+1)*10`, holes as
/// `1000 + i*10`, authors new rings at `>= 2001`; the dynamic pipeline
/// uses 1 and 2) and no threshold separates soup from non-soup. Markers
/// outside the set are untouched.
///
/// Returns the number of subsegs killed.
pub fn drop_interior_constraints(mesh: &mut CdtMesh, soup_markers: &[i32]) -> usize {
    debug_assert!(soup_markers.windows(2).all(|w| w[0] < w[1]));
    // Subseg -> triangle back-pointers are only opportunistically
    // maintained (stale ones can even alias a reused slot), so derive
    // each subseg's live sides from the reliable direction: a scan over
    // live triangles' edge slots.
    let sides = live_sides(mesh);
    let mut killed = 0usize;
    // Slot 0 is the dummy subseg — never a real constraint.
    for sub_idx in 1..mesh.subsegs.len() as u32 {
        let slot = mesh.subseg(sub_idx);
        if slot.is_dead() || soup_markers.binary_search(&slot.marker).is_err() {
            continue;
        }
        if let [Some(t0), Some(t1)] = sides[sub_idx as usize] {
            mesh.ts_dissolve(t0);
            mesh.ts_dissolve(t1);
            mesh.kill_subseg(sub_idx);
            killed += 1;
        }
    }
    killed
}

/// For every subseg, up to two live triangle edges bonded to it — found
/// by scanning live triangles in slot order (deterministic, and immune
/// to stale subseg->triangle back-pointers).
fn live_sides(mesh: &CdtMesh) -> Vec<[Option<Otri>; 2]> {
    let mut sides: Vec<[Option<Otri>; 2]> = vec![[None, None]; mesh.subsegs.len()];
    for tri_idx in 1..mesh.triangles.len() as u32 {
        let slot = mesh.triangle(tri_idx);
        if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        for orient in 0..3u8 {
            let here = Otri::new(tri_idx, orient);
            let sub = mesh.tspivot(here);
            if sub.sub != DUMMY_SUB {
                let entry = &mut sides[sub.sub as usize];
                if entry[0].is_none() {
                    entry[0] = Some(here);
                } else if entry[1].is_none() {
                    entry[1] = Some(here);
                }
            }
        }
    }
    sides
}

/// Subseg bookkeeping for a precomputed infected set (the non-BFS
/// analogue of the cleanup `plague` does while spreading): subsegs
/// between two dying triangles (or a dying triangle and the outside)
/// die with them; subsegs on the kept/killed boundary detach from the
/// dying side and get their marker (and boundary vertex markers)
/// promoted, exactly like the plague path.
pub(crate) fn cleanup_boundary_subsegs(mesh: &mut CdtMesh, infected: &[bool]) {
    for tri_idx in 1..mesh.triangles.len() as u32 {
        if !infected[tri_idx as usize] || mesh.triangle(tri_idx).is_dead() {
            continue;
        }
        for orient in 0..3u8 {
            let here = Otri::new(tri_idx, orient);
            let sub = mesh.tspivot(here);
            if sub.sub == DUMMY_SUB {
                continue;
            }
            let neighbor = mesh.sym(here);
            if neighbor.tri == DUMMY_TRI || infected[neighbor.tri as usize] {
                // Both sides dying: the subseg goes too. Detach every
                // triangle pointer to it so the scan never double-kills.
                mesh.kill_subseg(sub.sub);
                mesh.ts_dissolve(here);
                if neighbor.tri != DUMMY_TRI {
                    mesh.ts_dissolve(neighbor);
                }
            } else {
                // Neighbor survives: subseg becomes a boundary edge.
                mesh.st_dissolve(sub);
                if mesh.subseg(sub.sub).marker == 0 {
                    mesh.subseg_mut(sub.sub).marker = 1;
                }
                let norg = mesh.org(neighbor);
                let ndest = mesh.dest(neighbor);
                if norg.is_valid() && mesh.vertex(norg).marker == 0 {
                    mesh.vertex_mut(norg).marker = 1;
                }
                if ndest.is_valid() && mesh.vertex(ndest).marker == 0 {
                    mesh.vertex_mut(ndest).marker = 1;
                }
            }
        }
    }
}

/// After the sweep, any live triangle edge with no neighbor and no
/// subseg gets a marker-1 subseg (mirrors `clip_ears`' promotion rule).
/// Normally every kept/killed boundary already carries a constraint;
/// this converts the rare sliver-centroid misclassification into a
/// still-valid, still-marked mesh instead of a silently unmarked wall.
fn backfill_boundary_subsegs(mesh: &mut CdtMesh) {
    for tri_idx in 1..mesh.triangles.len() as u32 {
        let slot = mesh.triangle(tri_idx);
        if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        for orient in 0..3u8 {
            let here = Otri::new(tri_idx, orient);
            if mesh.sym(here).tri == DUMMY_TRI && mesh.tspivot(here).sub == DUMMY_SUB {
                insert_subseg(mesh, here, 1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::divconq::{delaunay, DivConqOptions};
    use crate::holes::testfix::{build_square_with_square_hole, push};
    use crate::holes::carve_holes;
    use crate::pslg::{Pslg, PslgSegment, PslgVertex};
    use crate::segment::form_skeleton;
    use rsnav_common::VertexId;

    fn contour(pts: &[(f64, f64)], marker: i32) -> SoupContour {
        SoupContour {
            points: pts.iter().map(|&(x, y)| Vertex::new(x, y)).collect(),
            marker,
        }
    }

    fn ccw_square(x0: f64, y0: f64, x1: f64, y1: f64, marker: i32) -> SoupContour {
        contour(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)], marker)
    }

    fn cw_square(x0: f64, y0: f64, x1: f64, y1: f64, marker: i32) -> SoupContour {
        contour(&[(x0, y0), (x0, y1), (x1, y1), (x1, y0)], marker)
    }

    #[test]
    fn winding_primitives() {
        let ccw = ccw_square(0.0, 0.0, 10.0, 10.0, 1);
        let cw = cw_square(0.0, 0.0, 10.0, 10.0, 1);
        let p = Vertex::new(5.0, 5.0);
        assert_eq!(winding_number(p, &[ccw.clone()]), 1);
        assert_eq!(winding_number(p, &[cw.clone()]), -1);
        assert_eq!(winding_number(Vertex::new(20.0, 5.0), &[ccw.clone()]), 0);
        // Nested: CCW outer + CW inner -> 0 inside the inner ring, +1 in
        // the ring between them.
        let inner = cw_square(4.0, 4.0, 6.0, 6.0, 2);
        assert_eq!(winding_number(p, &[ccw.clone(), inner.clone()]), 0);
        assert_eq!(winding_number(Vertex::new(2.0, 2.0), &[ccw.clone(), inner]), 1);
        // Overlapping CCW perimeters stack to 2.
        let shifted = ccw_square(3.0, 3.0, 13.0, 13.0, 3);
        assert_eq!(winding_number(p, &[ccw, shifted]), 2);
    }

    #[test]
    fn winding_bowtie_lobes_are_opposite() {
        // Self-crossing quad: left lobe traversed CCW (+1), right lobe
        // CW (-1) — the flipped-lobe cancellation the cull relies on.
        let bow = contour(&[(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)], 1);
        assert_eq!(winding_number(Vertex::new(2.0, 5.0), &[bow.clone()]), 1);
        assert_eq!(winding_number(Vertex::new(8.0, 5.0), &[bow.clone()]), -1);
        assert_eq!(winding_number(Vertex::new(5.0, 2.0), &[bow.clone()]), 0);
        assert_eq!(winding_number(Vertex::new(15.0, 5.0), &[bow]), 0);
    }

    #[test]
    fn winding_ray_through_vertices_not_double_counted() {
        // The +x ray from the center passes exactly through the right
        // corner of a diamond; the half-open rule must count it once.
        let diamond = contour(&[(5.0, 0.0), (10.0, 5.0), (5.0, 10.0), (0.0, 5.0)], 1);
        assert_eq!(winding_number(Vertex::new(5.0, 5.0), &[diamond.clone()]), 1);
        // A point whose ray passes through the top vertex, from outside.
        assert_eq!(winding_number(Vertex::new(-5.0, 10.0), &[diamond]), 0);
    }

    /// The winding cull must reproduce carve_holes exactly on a clean
    /// (non-crossing) scene: same survivors, same subseg bookkeeping.
    #[test]
    fn matches_carve_holes_on_square_with_hole() {
        let survivors = |mesh: &CdtMesh| -> Vec<(u32, [VertexId; 3])> {
            (1..mesh.triangles.len() as u32)
                .filter(|&i| !mesh.triangle(i).is_dead())
                .map(|i| (i, mesh.triangle(i).vertices))
                .collect()
        };
        let live_subsegs = |mesh: &CdtMesh| -> Vec<(u32, i32)> {
            (0..mesh.subsegs.len() as u32)
                .filter(|&i| !mesh.subseg(i).is_dead())
                .map(|i| (i, mesh.subseg(i).marker))
                .collect()
        };

        let (mut m1, pslg) = build_square_with_square_hole();
        carve_holes(&mut m1, &pslg, false);

        let (mut m2, _) = build_square_with_square_hole();
        // Perimeter CCW, hole CW (the fixture's inner ring is authored
        // CCW, so reverse it for the winding convention).
        let outer = ccw_square(0.0, 0.0, 4.0, 4.0, 10);
        let hole = cw_square(1.5, 1.5, 2.5, 2.5, 20);
        let killed = carve_by_winding(&mut m2, &[outer, hole]);

        assert!(killed > 0);
        assert_eq!(m2.live_triangle_count(), 8);
        assert_eq!(survivors(&m1), survivors(&m2), "survivor sets differ");
        assert_eq!(live_subsegs(&m1), live_subsegs(&m2), "subseg sets differ");
    }

    /// Nested island: a hole containing a smaller perimeter. The moat is
    /// culled, the island survives — impossible for seed-based carving
    /// without extra seeds, free for winding classification.
    #[test]
    fn nested_island_survives() {
        let mut mesh = CdtMesh::new();
        let rings: [&[(f64, f64)]; 3] = [
            &[(0.0, 0.0), (12.0, 0.0), (12.0, 12.0), (0.0, 12.0)],
            &[(2.0, 2.0), (10.0, 2.0), (10.0, 10.0), (2.0, 10.0)],
            &[(4.0, 4.0), (8.0, 4.0), (8.0, 8.0), (4.0, 8.0)],
        ];
        let mut pslg = Pslg::new();
        let mut next = 0u32;
        for ring in rings {
            let start = next;
            for &(x, y) in ring {
                push(&mut mesh, x, y);
                pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
                next += 1;
            }
            for i in 0..4u32 {
                pslg.segments.push(PslgSegment {
                    a: start + i,
                    b: start + (i + 1) % 4,
                    marker: 1,
                });
            }
        }
        delaunay(&mut mesh, DivConqOptions::default());
        form_skeleton(&mut mesh, &pslg, None).unwrap();

        let outer = ccw_square(0.0, 0.0, 12.0, 12.0, 1);
        let moat = cw_square(2.0, 2.0, 10.0, 10.0, 1);
        let island = ccw_square(4.0, 4.0, 8.0, 8.0, 1);
        carve_by_winding(&mut mesh, &[outer, moat, island]);

        let covers = |mesh: &CdtMesh, p: Vertex| -> bool {
            (1..mesh.triangles.len() as u32).any(|i| {
                let slot = mesh.triangle(i);
                if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
                    return false;
                }
                let positions: Vec<Vertex> =
                    mesh.vertices.iter().map(|v| v.position).collect();
                rsnav_common::Triangle::new(
                    slot.vertices[0],
                    slot.vertices[1],
                    slot.vertices[2],
                )
                .contains(&positions, p)
            })
        };
        assert!(covers(&mesh, Vertex::new(1.0, 6.0)), "outer ring area lost");
        assert!(!covers(&mesh, Vertex::new(3.0, 6.0)), "moat not culled");
        assert!(covers(&mesh, Vertex::new(6.0, 6.0)), "island lost");
    }

    /// Fully flipped soup (all-CW ring): winding is -1 everywhere inside,
    /// so everything dies — no panic, no survivors.
    #[test]
    fn flipped_ring_culls_everything() {
        let mut mesh = CdtMesh::new();
        let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (0.0, 4.0)];
        let mut pslg = Pslg::new();
        for &(x, y) in &pts {
            push(&mut mesh, x, y);
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        for i in 0..4u32 {
            pslg.segments.push(PslgSegment { a: i, b: (i + 1) % 4, marker: 1 });
        }
        delaunay(&mut mesh, DivConqOptions::default());
        form_skeleton(&mut mesh, &pslg, None).unwrap();

        let flipped = cw_square(0.0, 0.0, 10.0, 4.0, 1);
        carve_by_winding(&mut mesh, &[flipped]);
        assert_eq!(mesh.live_triangle_count(), 0);
    }

    /// drop_interior_constraints is exact set membership, not a
    /// threshold: a high-marker soup constraint between two live
    /// triangles dies, an unrelated marker survives.
    #[test]
    fn drop_interior_constraints_is_set_membership() {
        // Square with an interior diagonal-ish constraint: build a
        // square PSLG plus one interior segment between two corners,
        // marked 1000 (the demo's loaded-hole scheme) and another marked
        // 55 that must survive.
        let mut mesh = CdtMesh::new();
        let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let mut pslg = Pslg::new();
        for &(x, y) in &pts {
            push(&mut mesh, x, y);
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        for i in 0..4u32 {
            pslg.segments.push(PslgSegment { a: i, b: (i + 1) % 4, marker: 10 });
        }
        // Interior diagonal 0-2 marked 1000 (soup), and we also mark the
        // 1-3 diagonal 55 (non-soup)… but only one diagonal can exist in
        // a triangulation of 4 points. Use a 5th center vertex instead:
        // constraints center-corner0 (1000) and center-corner2 (55).
        push(&mut mesh, 5.0, 5.0);
        pslg.vertices.push(PslgVertex::new(Vertex::new(5.0, 5.0)));
        pslg.segments.push(PslgSegment { a: 4, b: 0, marker: 1000 });
        pslg.segments.push(PslgSegment { a: 4, b: 2, marker: 55 });

        delaunay(&mut mesh, DivConqOptions::default());
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        let outer = ccw_square(0.0, 0.0, 10.0, 10.0, 10);
        carve_by_winding(&mut mesh, &[outer]);

        let live_markers = |mesh: &CdtMesh| -> Vec<i32> {
            (0..mesh.subsegs.len() as u32)
                .filter(|&i| !mesh.subseg(i).is_dead())
                .map(|i| mesh.subseg(i).marker)
                .collect()
        };
        assert!(live_markers(&mesh).contains(&1000));
        assert!(live_markers(&mesh).contains(&55));

        let killed = drop_interior_constraints(&mut mesh, &[10, 1000]);
        assert_eq!(killed, 1, "exactly the marker-1000 interior constraint dies");
        let markers = live_markers(&mesh);
        assert!(!markers.contains(&1000), "soup constraint must die");
        assert!(markers.contains(&55), "non-soup marker must survive");
        // Boundary marker-10 subsegs are NOT interior (one side dead
        // after the carve), so the soup set containing 10 leaves them.
        assert_eq!(markers.iter().filter(|&&m| m == 10).count(), 4);
    }

    // --- WindingIndex pinned against the brute-force oracle -------------
    //
    // winding_number stays the reference implementation (it is also the
    // independent oracle used by inset.rs' property sweep); every case
    // below asserts exact i32 equality against it, the same way
    // holes.rs pins grid-inverted seeding against a linear scan.

    fn assert_agrees(contours: &[SoupContour], pts: &[Vertex]) {
        let idx = WindingIndex::build(contours);
        for &p in pts {
            assert_eq!(
                idx.winding(p),
                winding_number(p, contours),
                "index disagrees at {p:?}"
            );
        }
    }

    /// Every point the fixture tests above exercise, plus the cases the
    /// filters are most likely to get wrong: on a vertex, on an edge, at
    /// the exact ylo/yhi band ends, and outside the bbox on all four
    /// sides.
    #[test]
    fn index_matches_oracle_on_fixtures() {
        let ccw = ccw_square(0.0, 0.0, 10.0, 10.0, 1);
        let cw = cw_square(0.0, 0.0, 10.0, 10.0, 1);
        let inner = cw_square(4.0, 4.0, 6.0, 6.0, 2);
        let shifted = ccw_square(3.0, 3.0, 13.0, 13.0, 3);
        let bow = contour(&[(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)], 1);
        let diamond = contour(&[(5.0, 0.0), (10.0, 5.0), (5.0, 10.0), (0.0, 5.0)], 1);

        let mut pts = Vec::new();
        for &(x, y) in &[
            (5.0, 5.0),
            (20.0, 5.0),
            (2.0, 2.0),
            (2.0, 5.0),
            (8.0, 5.0),
            (5.0, 2.0),
            (15.0, 5.0),
            (-5.0, 10.0),
            // vertices, edge midpoints, band ends
            (0.0, 0.0),
            (10.0, 10.0),
            (10.0, 0.0),
            (0.0, 10.0),
            (5.0, 0.0),
            (0.0, 5.0),
            (10.0, 5.0),
            (5.0, 10.0),
            (4.0, 4.0),
            (6.0, 6.0),
            // outside on all four sides
            (-1.0, 5.0),
            (11.0, 5.0),
            (5.0, -1.0),
            (5.0, 11.0),
            (-100.0, -100.0),
            (100.0, 100.0),
        ] {
            pts.push(Vertex::new(x, y));
        }

        for set in [
            vec![ccw.clone()],
            vec![cw.clone()],
            vec![ccw.clone(), inner.clone()],
            vec![ccw.clone(), shifted.clone()],
            vec![bow.clone()],
            vec![diamond.clone()],
            vec![ccw, cw, inner, shifted, bow, diamond],
        ] {
            assert_agrees(&set, &pts);
        }
    }

    /// Randomized stars and bowties against the oracle, with query points
    /// deliberately snapped onto vertices, edge midpoints and the y
    /// extremes — the coordinates where the half-open band test and the
    /// x-max prune sit exactly on their boundaries.
    #[test]
    fn index_matches_oracle_on_random_scenes() {
        use rsnav_common::rng::Lcg;

        let mut rng = Lcg(0x171D_E0FF);
        for _scene in 0..40 {
            let mut contours = Vec::new();
            let rings = 1 + (rng.next_u64() % 4) as usize;
            for _ in 0..rings {
                let cx = rng.next_f64() * 20.0 - 10.0;
                let cy = rng.next_f64() * 20.0 - 10.0;
                let n = 3 + (rng.next_u64() % 10) as usize;
                let pts: Vec<Vertex> = (0..n)
                    .map(|i| {
                        let t = (i as f64 / n as f64) * std::f64::consts::TAU;
                        // Alternating radii make self-crossing lobes.
                        let r = if i % 2 == 0 { 6.0 } else { 2.0 } * (0.5 + rng.next_f64());
                        Vertex::new(cx + r * t.cos(), cy + r * t.sin())
                    })
                    .collect();
                contours.push(SoupContour { points: pts, marker: 1 });
            }

            let mut pts = Vec::new();
            // Snapped: every vertex, every edge midpoint.
            for c in &contours {
                let n = c.points.len();
                for i in 0..n {
                    let a = c.points[i];
                    let b = c.points[(i + 1) % n];
                    pts.push(a);
                    pts.push(Vertex::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0));
                }
            }
            // The exact y extremes, swept in x.
            let ylo = contours
                .iter()
                .flat_map(|c| c.points.iter())
                .fold(f64::INFINITY, |m, v| m.min(v.y));
            let yhi = contours
                .iter()
                .flat_map(|c| c.points.iter())
                .fold(f64::NEG_INFINITY, |m, v| m.max(v.y));
            for i in 0..20 {
                let x = -20.0 + i as f64 * 2.0;
                pts.push(Vertex::new(x, ylo));
                pts.push(Vertex::new(x, yhi));
            }
            // Bulk random probes.
            for _ in 0..200 {
                pts.push(Vertex::new(
                    rng.next_f64() * 40.0 - 20.0,
                    rng.next_f64() * 40.0 - 20.0,
                ));
            }
            assert_agrees(&contours, &pts);
        }
    }

    /// The bucket count is a performance knob and nothing else: the same
    /// soup and points must give identical i32s at every `nb`. This is
    /// the "reordering cannot change an integer sum" leg of the design,
    /// exercised directly.
    #[test]
    fn index_is_invariant_to_bucket_count() {
        use rsnav_common::rng::Lcg;

        let mut rng = Lcg(0xB0CC_1E5);
        let contours = vec![
            ccw_square(0.0, 0.0, 30.0, 30.0, 1),
            cw_square(5.0, 5.0, 12.0, 12.0, 2),
            cw_square(18.0, 4.0, 26.0, 21.0, 3),
            contour(&[(2.0, 20.0), (14.0, 29.0), (14.0, 20.0), (2.0, 29.0)], 4),
        ];
        let pts: Vec<Vertex> = (0..2000)
            .map(|_| Vertex::new(rng.next_f64() * 34.0 - 2.0, rng.next_f64() * 34.0 - 2.0))
            .collect();

        let reference: Vec<i32> = pts.iter().map(|&p| winding_number(p, &contours)).collect();
        for nb in [1usize, 2, 17, 4096] {
            let idx = WindingIndex::build_inner(&contours, Some(nb));
            let got: Vec<i32> = pts.iter().map(|&p| idx.winding(p)).collect();
            assert_eq!(got, reference, "bucket count {nb} changed the result");
        }
    }

    /// Degenerate soups: nothing to index, nothing but horizontal edges
    /// (the `nb == 0` path), a ring with zero y extent (the `!(y1 > y0)`
    /// guard), and duplicate points.
    #[test]
    fn index_matches_oracle_on_degenerate_soups() {
        let pts: Vec<Vertex> = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.0, 5.0),
            (-1.0, 0.0),
            (100.0, 0.0),
            (2.5, 1e-300),
        ]
        .iter()
        .map(|&(x, y)| Vertex::new(x, y))
        .collect();

        assert_agrees(&[], &pts);
        // All-horizontal: a degenerate "ring" along y = 0.
        assert_agrees(&[contour(&[(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)], 1)], &pts);
        // Zero y extent across several rings.
        assert_agrees(
            &[
                contour(&[(0.0, 0.0), (5.0, 0.0)], 1),
                contour(&[(1.0, 0.0), (9.0, 0.0)], 2),
            ],
            &pts,
        );
        // Duplicate points (zero-length edges) mixed with real ones.
        assert_agrees(
            &[contour(
                &[(0.0, 0.0), (0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (5.0, 5.0), (0.0, 5.0)],
                1,
            )],
            &pts,
        );
        // Single point.
        assert_agrees(&[contour(&[(3.0, 3.0)], 1)], &pts);
    }

    /// The last few ULPs below the soup's maximum y. Rounding in `inv_h`
    /// can send a `p.y` that is strictly inside the extent to bucket
    /// `nb`, so the query must *clamp* like `bin` rather than treat that
    /// as out-of-extent — an early return there silently drops every
    /// edge in the top bucket and culls walkable triangles. The random
    /// sweep misses this because it probes `yhi` exactly, where the
    /// half-open rule makes both implementations agree trivially.
    #[test]
    fn index_matches_oracle_just_below_top_of_extent() {
        use rsnav_common::rng::Lcg;

        // The witness the early-return version failed on: 2 non-
        // horizontal edges => default nb == 1, no override needed.
        let tri = contour(
            &[
                (0.0, 13.038690931021135),
                (10.0, 13.038690931021135),
                (5.0, 29.597568789406075),
            ],
            1,
        );
        assert_agrees(
            std::slice::from_ref(&tri),
            &[Vertex::new(5.0, 29.597568789406072)],
        );

        // And a sweep: random extents, probed at the ULPs just below
        // yhi, across every bucket count.
        let mut rng = Lcg(0x51DE_BEEF);
        for _ in 0..300 {
            let y0 = rng.next_f64() * 40.0 - 20.0;
            let y1 = y0 + rng.next_f64() * 30.0 + 0.5;
            let x0 = rng.next_f64() * 10.0 - 20.0;
            let rect = ccw_square(x0, y0, x0 + 12.0, y1, 1);
            let mut pts = Vec::new();
            for steps in 0..8u32 {
                let mut y = y1;
                for _ in 0..steps {
                    y = f64::from_bits(y.to_bits() - 1);
                }
                pts.push(Vertex::new(x0 + 6.0, y));
            }
            assert_agrees(std::slice::from_ref(&rect), &pts);
            for nb in [1usize, 2, 17, 4096] {
                let idx = WindingIndex::build_inner(std::slice::from_ref(&rect), Some(nb));
                for &p in &pts {
                    assert_eq!(
                        idx.winding(p),
                        winding_number(p, std::slice::from_ref(&rect)),
                        "nb {nb} disagrees at {p:?}"
                    );
                }
            }
        }
    }

    /// The CSR item list is budgeted: a soup of full-height edges must
    /// not allocate `n * nb` entries, and shrinking `nb` to fit must
    /// leave every winding number untouched.
    #[test]
    fn index_item_budget_bounds_tall_edge_soups() {
        use rsnav_common::rng::Lcg;

        // Concentric tall, thin rings: every edge spans nearly the full
        // y extent, the worst case for per-bucket filing.
        let mut rng = Lcg(0x7A11_0000);
        let contours: Vec<SoupContour> = (0..1200)
            .map(|i| {
                let x = i as f64 * 0.5;
                ccw_square(x, -100_000.0, x + 0.25, 100_000.0, i as i32)
            })
            .collect();
        let n: usize = contours.iter().map(|c| c.points.len()).sum();
        let idx = WindingIndex::build(&contours);
        assert!(
            idx.items.len() <= (1 << 23) + n,
            "item list {} exceeds budget",
            idx.items.len()
        );

        let pts: Vec<Vertex> = (0..500)
            .map(|_| {
                Vertex::new(
                    rng.next_f64() * 700.0 - 50.0,
                    rng.next_f64() * 220_000.0 - 110_000.0,
                )
            })
            .collect();
        for &p in &pts {
            assert_eq!(idx.winding(p), winding_number(p, &contours), "at {p:?}");
        }
    }

    /// Non-finite coordinates route to the always-scanned fallback list,
    /// so they keep matching the oracle instead of being silently
    /// filtered out by `f64::min`/`max`'s NaN-dropping.
    #[test]
    fn index_matches_oracle_with_non_finite_edges() {
        let base = ccw_square(0.0, 0.0, 10.0, 10.0, 1);
        let pts: Vec<Vertex> = [
            (5.0, 5.0),
            (-1.0, 5.0),
            (20.0, 5.0),
            (5.0, 0.0),
            (5.0, 10.0),
            (0.0, 0.0),
        ]
        .iter()
        .map(|&(x, y)| Vertex::new(x, y))
        .collect();

        let nan = contour(&[(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)], 9);
        let mut with_nan = nan.clone();
        with_nan.points[1] = Vertex::new(2.0, f64::NAN);
        assert_agrees(&[base.clone(), with_nan], &pts);

        let mut with_inf = nan.clone();
        with_inf.points[1] = Vertex::new(f64::INFINITY, 2.0);
        assert_agrees(&[base.clone(), with_inf], &pts);

        let mut with_neg_inf = nan;
        with_neg_inf.points[0] = Vertex::new(1.0, f64::NEG_INFINITY);
        assert_agrees(&[base, with_neg_inf], &pts);
    }

    /// Bookkeeping invariants after a winding carve: every live subseg
    /// borders at least one live triangle, and every live/dead boundary
    /// edge carries a live subseg with a nonzero marker.
    #[test]
    fn subseg_bookkeeping_invariants() {
        let (mut mesh, _) = build_square_with_square_hole();
        let outer = ccw_square(0.0, 0.0, 4.0, 4.0, 10);
        let hole = cw_square(1.5, 1.5, 2.5, 2.5, 20);
        carve_by_winding(&mut mesh, &[outer, hole]);

        // Every live subseg is bonded to at least one live triangle edge
        // (derived triangle-side, since subseg back-pointers are only
        // opportunistically maintained in this port).
        // Slot 0 is the dummy subseg; real subsegs start at 1.
        let sides = super::live_sides(&mesh);
        for sub_idx in 1..mesh.subsegs.len() as u32 {
            if mesh.subseg(sub_idx).is_dead() {
                continue;
            }
            assert!(
                sides[sub_idx as usize][0].is_some(),
                "subseg {sub_idx} floats with no live triangle"
            );
        }
        // Every exposed edge of a live triangle carries a marked subseg.
        for tri_idx in 1..mesh.triangles.len() as u32 {
            let slot = mesh.triangle(tri_idx);
            if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
                continue;
            }
            for orient in 0..3u8 {
                let here = Otri::new(tri_idx, orient);
                if mesh.sym(here).tri == DUMMY_TRI {
                    let sub = mesh.tspivot(here);
                    assert_ne!(sub.sub, DUMMY_SUB, "boundary edge without subseg");
                    assert_ne!(mesh.subseg(sub.sub).marker, 0, "unmarked wall");
                }
            }
        }
    }
}


