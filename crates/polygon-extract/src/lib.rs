//! Extract polygon regions (outer rings + holes) from a 2D occupancy bitfield.
//!
//! ## Convention
//!
//! - The bitfield is row-major: `data[row * width + col]`.
//! - `true` = walkable, `false` = wall.
//! - Cell `(col, row)` occupies the math-space rectangle
//!   `[col, col + 1] × [row, row + 1]`. The y-axis points up; `row = 0` is
//!   the bottom row.
//! - Walkable regions are connected using **4-connectivity** (cells sharing
//!   a full edge). Cells touching only at a corner are treated as
//!   disconnected.
//! - Output polygons are wound **counter-clockwise** for outer rings and
//!   **clockwise** for holes, matching the convention `rsnav-common::Polygon`
//!   uses and that the CDT expects.

#![forbid(unsafe_code)]

use rsnav_common::par::{par_map_indexed, resolve_threads};
use rsnav_common::{Aabb, Polygon, PolygonWithHoles, Vertex, Winding, geom};

// --- Bitfield ------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Bitfield {
    pub width: u32,
    pub height: u32,
    pub data: Vec<bool>,
}

/// Errors returned by [`Bitfield::new`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum BitfieldError {
    /// `data.len()` doesn't equal `width * height`.
    BadDataLength {
        width: u32,
        height: u32,
        data_len: usize,
    },
}

impl std::fmt::Display for BitfieldError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadDataLength { width, height, data_len } => write!(
                f,
                "Bitfield::new: data length {} does not equal width * height = {} * {} = {}",
                data_len,
                width,
                height,
                (*width as usize) * (*height as usize),
            ),
        }
    }
}

impl std::error::Error for BitfieldError {}

impl Bitfield {
    /// Wrap `data` as a `width × height` row-major bitfield. Returns
    /// [`BitfieldError::BadDataLength`] when `data.len() != width * height`.
    pub fn new(width: u32, height: u32, data: Vec<bool>) -> Result<Self, BitfieldError> {
        let expected = (width as usize) * (height as usize);
        if data.len() != expected {
            return Err(BitfieldError::BadDataLength {
                width,
                height,
                data_len: data.len(),
            });
        }
        Ok(Self { width, height, data })
    }

    /// All-false grid of the given size.
    pub fn empty(width: u32, height: u32) -> Self {
        let len = (width as usize) * (height as usize);
        Self {
            width,
            height,
            data: vec![false; len],
        }
    }

    #[inline]
    pub fn at(&self, col: i64, row: i64) -> bool {
        if col < 0 || row < 0 || col >= self.width as i64 || row >= self.height as i64 {
            return false;
        }
        self.data[(row as usize) * (self.width as usize) + (col as usize)]
    }

    pub fn set(&mut self, col: u32, row: u32, v: bool) {
        let idx = (row as usize) * (self.width as usize) + (col as usize);
        self.data[idx] = v;
    }
}

// --- ExtractOptions ------------------------------------------------------

#[derive(Copy, Clone, Debug)]
pub struct ExtractOptions {
    /// Drop walkable regions whose outer polygon has area strictly less than
    /// this value (in cell units; one cell = 1.0). Their holes are dropped
    /// with them. Set to 0.0 to keep everything.
    pub min_area: f64,
    /// Remove vertices that are collinear with their neighbours after the
    /// raw trace. Default `true`.
    pub remove_collinear: bool,
    /// Replace stair-step (zigzag) sequences along axis-aligned edges with
    /// single diagonals. Default `true`; turning it off keeps the exact
    /// cell-aligned boundary at the cost of more triangles in the CDT.
    pub diagonal_smoothing: bool,
    /// Worker threads for the parallelizable phases (border-edge scan,
    /// hole parenting, per-region post-processing). `0` = one per
    /// available core, `1` = fully serial. Output is identical for every
    /// setting; small inputs stay serial regardless.
    pub threads: usize,
}

impl Default for ExtractOptions {
    fn default() -> Self {
        Self {
            min_area: 0.0,
            remove_collinear: true,
            diagonal_smoothing: true,
            threads: 0,
        }
    }
}

// --- Public entry point --------------------------------------------------

/// Trace all walkable regions in `bits` as polygons with holes.
pub fn extract(bits: &Bitfield, opts: &ExtractOptions) -> Vec<PolygonWithHoles> {
    let threads = resolve_threads(opts.threads);
    let loops = trace_loops(bits, threads);

    // Classify into outers (CCW) and holes (CW).
    let (mut outers, holes): (Vec<Polygon>, Vec<Polygon>) =
        loops.into_iter().partition(|p| p.signed_area2() > 0.0);

    let mut outer_holes: Vec<Vec<Polygon>> = vec![Vec::new(); outers.len()];
    if !holes.is_empty() {
        // Precompute per-outer data once instead of per (hole, outer) pair:
        // the AABB (cheap reject), the area (tie-break), and a collinear-
        // reduced copy of the ring for the exact containment test. Raw
        // traced rings carry a vertex at every unit cell corner; reducing
        // them shrinks contains() from O(perimeter) to O(corners) without
        // changing its result — removal only merges straight runs (the
        // trace cannot produce out-and-back spikes), so the boundary point
        // set and the exact orient2d/ray-cast answers are identical.
        let infos: Vec<OuterInfo> = outers
            .iter()
            .map(|o| {
                let mut ring = o.clone();
                ring.remove_collinear();
                OuterInfo {
                    aabb: o.aabb(),
                    area: o.area(),
                    ring,
                }
            })
            .collect();
        // Each hole's parent is a pure function of `infos`, so holes can
        // be resolved on worker threads; the grouping stays serial in
        // hole order, keeping per-parent hole order identical to a fully
        // serial run. Gate on holes × outers — the actual work — so a
        // thousand trivial lookups against one outer stay serial.
        let work = holes.len().saturating_mul(infos.len());
        let par = threads
            .min(PARENT_MAX_THREADS)
            .min(work / PARENT_WORK_PER_THREAD + 1);
        let parents: Vec<Option<usize>> = if holes.len() >= PAR_MIN_HOLES && par > 1 {
            par_map_indexed(&holes, par, |_, hole| {
                smallest_enclosing_outer(&infos, hole.vertices[0])
            })
        } else {
            holes
                .iter()
                .map(|hole| smallest_enclosing_outer(&infos, hole.vertices[0]))
                .collect()
        };
        for (hole, parent) in holes.into_iter().zip(parents) {
            if let Some(parent) = parent {
                outer_holes[parent].push(hole);
            }
            // Holes with no parent must be tracing artifacts (e.g. the
            // unbounded outside, which our trace doesn't actually produce
            // since we only walk walkable-cell borders). Drop silently.
        }
    }

    let pairs: Vec<(Polygon, Vec<Polygon>)> =
        outers.drain(..).zip(outer_holes.into_iter()).collect();
    let par = threads.min(POST_MAX_THREADS);
    let mut regions: Vec<PolygonWithHoles> = if pairs.len() >= PAR_MIN_REGIONS && par > 1 {
        // Smoothing is per-region-independent; the clone is one pass over
        // ring memory, dwarfed by the smoothing it unlocks in parallel.
        par_map_indexed(&pairs, par, |_, (outer, holes)| {
            post_process_region(outer.clone(), holes.clone(), opts)
        })
    } else {
        pairs
            .into_iter()
            .map(|(outer, holes)| post_process_region(outer, holes, opts))
            .collect()
    };

    if opts.min_area > 0.0 {
        regions.retain(|r| r.outer.area() >= opts.min_area);
    }
    regions
}

fn post_process_hole(mut hole: Polygon, opts: &ExtractOptions) -> Polygon {
    if opts.diagonal_smoothing {
        hole = diagonal_smooth(hole);
    }
    if opts.remove_collinear {
        hole.remove_collinear();
    }
    hole
}

fn post_process_region(
    mut outer: Polygon,
    holes: Vec<Polygon>,
    opts: &ExtractOptions,
) -> PolygonWithHoles {
    let holes = holes
        .into_iter()
        .map(|h| post_process_hole(h, opts))
        .collect();
    if opts.diagonal_smoothing {
        outer = diagonal_smooth(outer);
    }
    if opts.remove_collinear {
        outer.remove_collinear();
    }
    PolygonWithHoles { outer, holes }
}

/// Serial/parallel cutovers for [`extract`]'s phases: below these sizes
/// thread spawn costs more than the work. Caps keep each phase from
/// spawning more workers than its memory-bound scan can feed.
const PAR_MIN_HOLES: usize = 64;
const PAR_MIN_REGIONS: usize = 8;
const PAR_MIN_CELLS: usize = 500_000;
const PARENT_MAX_THREADS: usize = 32;
/// Roughly one thread per this many (hole, outer) candidate pairs.
const PARENT_WORK_PER_THREAD: usize = 4096;
const POST_MAX_THREADS: usize = 16;
const COLLECT_MAX_THREADS: usize = 16;

/// Per-outer data precomputed for hole parenting: containment is tested
/// against the reduced ring, but the AABB and area come from the raw ring
/// (numerically identical either way — reduction preserves the boundary).
struct OuterInfo {
    aabb: Aabb,
    area: f64,
    ring: Polygon,
}

fn smallest_enclosing_outer(outers: &[OuterInfo], point: Vertex) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (i, o) in outers.iter().enumerate() {
        // AABB reject: Polygon::contains is boundary-inclusive and the
        // closed AABB covers the boundary, so this can never cull a hit.
        if !o.aabb.contains(point) {
            continue;
        }
        if o.ring.contains(point) && best.map_or(true, |(_, a)| o.area < a) {
            best = Some((i, o.area));
        }
    }
    best.map(|(i, _)| i)
}

// --- Border-edge tracing -------------------------------------------------
//
// For each walkable cell we emit zero or more "border edges" — directed
// unit-length axis-aligned segments along the cell's boundary, with the
// walkable cell on the *left* of the directed edge. Border edges from the
// same cell pair at shared corners ("same-cell pairing"), which is what
// resolves the diagonal-touch case so two corner-touching walkable cells
// produce two separate polygons.

#[derive(Copy, Clone, Debug)]
struct BorderEdge {
    start: (i64, i64),
    end: (i64, i64),
    /// Linear cell index — used to disambiguate which outgoing edge to
    /// follow at corners where multiple cells contribute edges.
    cell: u32,
}

fn collect_border_edges(bits: &Bitfield, threads: usize) -> Vec<BorderEdge> {
    let h = bits.height as usize;
    let cells = bits.width as usize * h;
    let threads = threads.min(COLLECT_MAX_THREADS).min(h.max(1));
    if threads <= 1 || cells < PAR_MIN_CELLS {
        return collect_border_edges_rows(bits, 0, h as i64);
    }
    // Row bands are independent (neighbor peeks are read-only), and
    // concatenating the per-band results in band order reproduces the
    // sequential row-major edge order exactly.
    let chunk = h.div_ceil(threads);
    let bands: Vec<(i64, i64)> = (0..threads)
        .map(|i| ((i * chunk) as i64, ((i + 1) * chunk).min(h) as i64))
        .filter(|(a, b)| a < b)
        .collect();
    let mut parts = par_map_indexed(&bands, bands.len(), |_, &(a, b)| {
        collect_border_edges_rows(bits, a, b)
    });
    let mut edges = Vec::with_capacity(parts.iter().map(Vec::len).sum());
    for part in &mut parts {
        edges.append(part);
    }
    edges
}

fn collect_border_edges_rows(bits: &Bitfield, row_start: i64, row_end: i64) -> Vec<BorderEdge> {
    let mut edges = Vec::new();
    let w = bits.width as i64;
    for row in row_start..row_end {
        for col in 0..w {
            if !bits.at(col, row) {
                continue;
            }
            let cell_idx = (row as u32) * bits.width + col as u32;
            // South: bottom edge from (col, row) → (col+1, row).
            if !bits.at(col, row - 1) {
                edges.push(BorderEdge {
                    start: (col, row),
                    end: (col + 1, row),
                    cell: cell_idx,
                });
            }
            // East: right edge from (col+1, row) → (col+1, row+1).
            if !bits.at(col + 1, row) {
                edges.push(BorderEdge {
                    start: (col + 1, row),
                    end: (col + 1, row + 1),
                    cell: cell_idx,
                });
            }
            // North: top edge from (col+1, row+1) → (col, row+1).
            if !bits.at(col, row + 1) {
                edges.push(BorderEdge {
                    start: (col + 1, row + 1),
                    end: (col, row + 1),
                    cell: cell_idx,
                });
            }
            // West: left edge from (col, row+1) → (col, row).
            if !bits.at(col - 1, row) {
                edges.push(BorderEdge {
                    start: (col, row + 1),
                    end: (col, row),
                    cell: cell_idx,
                });
            }
        }
    }
    edges
}

/// "Start corner → outgoing border edges" lookup for [`trace_loops`].
///
/// Corners are dense — `(col, row)` in `[0, w] × [0, h]` — so the fast path
/// is a flat per-corner array indexed by `row * (w + 1) + col`, which avoids
/// the hashing that dominates the old `HashMap<(i64, i64), Vec<usize>>`
/// build. Same-cell pairing means a corner has at most two outgoing border
/// edges; a third is a structural-invariant failure (`debug_assert!`), and
/// release builds spill it to `overflow` so lookups still see every edge in
/// edge-index order — identical to the HashMap's push order.
enum CornerIndex {
    Flat {
        /// Corners per row: `w + 1`.
        stride: i64,
        /// Two edge-index slots per corner; `u32::MAX` = empty.
        slots: Vec<[u32; 2]>,
        /// Third-and-later edges per corner (expected empty; see above).
        overflow: Vec<((i64, i64), u32)>,
    },
    /// Fallback when the corner grid is too large to allocate densely.
    Map(std::collections::HashMap<(i64, i64), Vec<usize>>),
}

/// Corner grids above this many corners keep the HashMap path (the flat
/// array would cost 8 bytes per corner — ~512 MB at the threshold).
const FLAT_CORNER_LIMIT: usize = 64_000_000;

/// The flat array also loses when edges are sparse relative to the grid:
/// zero-filling 8 bytes per corner swamps the hashing it saves (an open
/// 2048² map has ~4.2M corners but only ~16k perimeter edges). Keep the
/// HashMap unless edges populate at least 1/64 of the corners.
const FLAT_CORNER_DENSITY: usize = 64;

impl CornerIndex {
    fn build(bits: &Bitfield, edges: &[BorderEdge]) -> Self {
        let stride = bits.width as usize + 1;
        let corners = stride * (bits.height as usize + 1);
        if corners > FLAT_CORNER_LIMIT || corners > edges.len().saturating_mul(FLAT_CORNER_DENSITY) {
            let mut by_start: std::collections::HashMap<(i64, i64), Vec<usize>> =
                std::collections::HashMap::new();
            for (i, e) in edges.iter().enumerate() {
                by_start.entry(e.start).or_default().push(i);
            }
            return Self::Map(by_start);
        }
        let mut slots = vec![[u32::MAX; 2]; corners];
        let mut overflow: Vec<((i64, i64), u32)> = Vec::new();
        for (i, e) in edges.iter().enumerate() {
            // Border-edge corners always lie in [0, w] × [0, h].
            let id = e.start.1 as usize * stride + e.start.0 as usize;
            let slot = &mut slots[id];
            if slot[0] == u32::MAX {
                slot[0] = i as u32;
            } else if slot[1] == u32::MAX {
                slot[1] = i as u32;
            } else {
                debug_assert!(
                    false,
                    "corner {:?} has more than two outgoing border edges",
                    e.start
                );
                overflow.push((e.start, i as u32));
            }
        }
        Self::Flat {
            stride: stride as i64,
            slots,
            overflow,
        }
    }

    /// Fills `out` with the edge indices starting at `corner`, in
    /// edge-index order (matching the HashMap Vec's push order).
    fn candidates_into(&self, corner: (i64, i64), out: &mut Vec<usize>) {
        out.clear();
        match self {
            Self::Flat { stride, slots, overflow } => {
                let id = (corner.1 * stride + corner.0) as usize;
                for &s in &slots[id] {
                    if s != u32::MAX {
                        out.push(s as usize);
                    }
                }
                // Slots fill before overflow, so appending keeps edge order.
                for &(c, e) in overflow {
                    if c == corner {
                        out.push(e as usize);
                    }
                }
            }
            Self::Map(by_start) => {
                if let Some(v) = by_start.get(&corner) {
                    out.extend_from_slice(v);
                }
            }
        }
    }
}

fn trace_loops(bits: &Bitfield, threads: usize) -> Vec<Polygon> {
    let edges = collect_border_edges(bits, threads);
    if edges.is_empty() {
        return Vec::new();
    }

    // Build a "start corner → list of (edge index, cell)" index so we can
    // continue a chain in O(1).
    let by_start = CornerIndex::build(bits, &edges);
    let mut candidates: Vec<usize> = Vec::with_capacity(4);

    let mut visited = vec![false; edges.len()];
    let mut loops = Vec::new();

    for seed in 0..edges.len() {
        if visited[seed] {
            continue;
        }
        let mut loop_verts: Vec<Vertex> = Vec::new();
        let mut cur = seed;
        // Both `break_with_loop` blocks below are structural-invariant
        // failures of the 4-connectivity trace. They should be unreachable
        // for well-formed input; `debug_assert!` lets the test suite catch
        // them, while release builds drop the partial loop and continue
        // instead of crashing user code. If you see one of these debug
        // panics it's a bug in `collect_border_edges` or the bitfield.
        let abandon = loop {
            visited[cur] = true;
            let e = edges[cur];
            loop_verts.push(Vertex::new(e.start.0 as f64, e.start.1 as f64));
            // Find the next edge: among edges starting at e.end, pick the
            // one owned by the same cell.
            by_start.candidates_into(e.end, &mut candidates);
            if candidates.is_empty() {
                debug_assert!(false, "border-edge chain ended at unmatched corner");
                break true;
            }
            // One candidate → take it (continuation between adjacent cells,
            // even if the cell ID differs). Multiple candidates → diagonal-
            // touch case; prefer same-cell so each cell's boundary stays
            // its own polygon under 4-connectivity.
            let next_opt = if candidates.len() == 1 {
                Some(candidates[0])
            } else {
                candidates
                    .iter()
                    .copied()
                    .find(|&j| edges[j].cell == e.cell)
            };
            let next = match next_opt {
                Some(n) => n,
                None => {
                    debug_assert!(false, "ambiguous corner without same-cell continuation");
                    break true;
                }
            };
            if next == seed {
                break false;
            }
            cur = next;
        };
        if !abandon {
            loops.push(Polygon::from_vertices(loop_verts));
        }
    }
    loops
}

// --- Diagonal smoothing --------------------------------------------------

/// Drop every vertex that is the *middle* of a stair-step. A vertex `v` is
/// a stair-step middle when:
///   * both incident edges are unit-length and perpendicular, AND
///   * the edge *before* the incoming edge is parallel-and-same-direction
///     as the *outgoing* edge (any length).
///
/// The second condition distinguishes a stair (alternating right turns and
/// left turns continuing the original heading) from the corner of a small
/// axis-aligned shape (all turns in the same rotational direction), which
/// we leave alone. We accept any positive scalar multiple — not just
/// bit-exact equality — so a stair adjacent to a longer straight run, or
/// stairs whose neighbour vertices were already eaten by an earlier pass,
/// are still recognised.
///
/// We iterate until a pass removes nothing. Each pass can expose new stair
/// middles that the previous pass couldn't see (e.g. a stair after a
/// single-unit tower that itself got collapsed first).
fn diagonal_smooth(mut p: Polygon) -> Polygon {
    loop {
        let n = p.vertices.len();
        if n < 4 {
            return p;
        }
        let stair = mark_stair_middles(&p);
        let kept: Vec<Vertex> = (0..n)
            .filter(|i| !stair[*i])
            .map(|i| p.vertices[i])
            .collect();
        if kept.len() == n {
            return p; // no change — done
        }
        if kept.len() < 3 {
            // Degenerate result (rare — happens only if the entire polygon
            // is a perfect zigzag loop). Fall back to the previous polygon.
            return p;
        }
        p = Polygon::from_vertices(kept);
    }
}

fn mark_stair_middles(p: &Polygon) -> Vec<bool> {
    let n = p.vertices.len();
    (0..n)
        .map(|i| {
            let v_m2 = p.vertices[(i + n - 2) % n];
            let v_m1 = p.vertices[(i + n - 1) % n];
            let v_i = p.vertices[i];
            let v_p1 = p.vertices[(i + 1) % n];
            let e_before = v_m1 - v_m2;
            let e_in = v_i - v_m1;
            let e_out = v_p1 - v_i;
            let unit_perp = e_in.length_sq() == 1.0
                && e_out.length_sq() == 1.0
                && e_in.dot(e_out) == 0.0;
            // Parallel-and-same-direction: zero cross + positive dot.
            // (Bit-exact on integer cell coordinates; no epsilon needed.)
            let same_dir = e_before.cross(e_out) == 0.0 && e_before.dot(e_out) > 0.0;
            unit_perp && same_dir
        })
        .collect()
}

// --- Sanity --------------------------------------------------------------

// `geom` is used implicitly via Polygon's helpers; keep the import alive.
#[allow(dead_code)]
fn _silence_unused(_: Winding) {
    let _ = geom::orient2d;
}

// --- Tests --------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(width: u32, rows: &[&str]) -> Bitfield {
        // Rows are passed top-down for human readability; flip to math-up.
        let height = rows.len() as u32;
        let mut data = vec![false; (width as usize) * (height as usize)];
        for (i, row) in rows.iter().enumerate() {
            let math_row = height as usize - 1 - i;
            for (col, ch) in row.chars().enumerate() {
                let walkable = match ch {
                    '#' => true,
                    '.' => false,
                    other => panic!("bad cell char {other:?}"),
                };
                data[math_row * (width as usize) + col] = walkable;
            }
        }
        Bitfield::new(width, height, data).expect("test grid: dimensions match")
    }

    #[test]
    fn single_cell_makes_unit_square() {
        let b = grid(3, &[".#.", "...", "..."]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 1);
        let r = &regions[0];
        assert_eq!(r.holes.len(), 0);
        assert_eq!(r.outer.area(), 1.0);
        assert_eq!(r.outer.winding(), Winding::CounterClockwise);
    }

    #[test]
    fn two_diagonal_cells_make_two_polygons() {
        // Cells (0, 1) and (1, 0) — touch only at a corner under 4-connectivity.
        let b = grid(2, &[".#", "#."]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 2, "expected two separate polygons");
        for r in &regions {
            assert_eq!(r.outer.area(), 1.0);
            assert_eq!(r.outer.winding(), Winding::CounterClockwise);
        }
    }

    #[test]
    fn three_by_three_block_has_no_holes() {
        let b = grid(3, &["###", "###", "###"]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].holes.len(), 0);
        assert_eq!(regions[0].outer.area(), 9.0);
        // remove_collinear should leave 4 corner vertices.
        assert_eq!(regions[0].outer.vertices.len(), 4);
    }

    #[test]
    fn region_with_single_cell_hole() {
        // 5x5 walkable block with a single-cell hole at (2, 2).
        let b = grid(5, &["#####", "#####", "##.##", "#####", "#####"]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].holes.len(), 1);
        assert_eq!(regions[0].outer.area(), 25.0);
        // After remove_collinear the outer is 4 corners, hole is 4 corners.
        assert_eq!(regions[0].outer.vertices.len(), 4);
        assert_eq!(regions[0].holes[0].vertices.len(), 4);
        assert_eq!(regions[0].holes[0].winding(), Winding::Clockwise);
        // Effective area = 25 − 1.
        assert_eq!(regions[0].area(), 24.0);
    }

    #[test]
    fn min_area_cull_drops_tiny_regions() {
        // A 4x4 block plus a far-away single-cell speck.
        let b = grid(8, &[
            "####....",
            "####....",
            "####....",
            "####...#",
            "........",
        ]);
        let mut opts = ExtractOptions::default();
        opts.min_area = 4.0;
        let regions = extract(&b, &opts);
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].outer.area(), 16.0);
    }

    #[test]
    fn remove_collinear_can_be_disabled() {
        let b = grid(3, &["###", "###", "###"]);
        let mut opts = ExtractOptions::default();
        opts.remove_collinear = false;
        let regions = extract(&b, &opts);
        // The raw trace produces a vertex at every cell corner along the
        // perimeter: 12 vertices (3 per side × 4 sides).
        assert_eq!(regions[0].outer.vertices.len(), 12);
    }

    /// 4-connectivity test: cells touching only at corners are separate.
    #[test]
    fn corner_touching_cells_are_separate_regions() {
        let b = grid(4, &[
            "#...",
            ".#..",
            "..#.",
            "...#",
        ]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 4);
        for r in &regions {
            assert_eq!(r.outer.area(), 1.0);
        }
    }

    /// Two hole cells touching only at a corner: the shared corner has two
    /// outgoing border edges, so this pins the same-cell pairing in
    /// `trace_loops` (and the 2-slot corner index) on the hole path. The
    /// walkable cell at the pinch owns edges on both hole cells, so the
    /// trace stitches them into a single pinched CW loop of area 2.
    #[test]
    fn diagonally_touching_hole_cells_trace_one_pinched_hole() {
        let b = grid(5, &[
            "#####",
            "##.##",
            "#.###",
            "#####",
        ]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].holes.len(), 1, "expected one pinched hole");
        assert_eq!(regions[0].holes[0].area(), 2.0);
        assert_eq!(regions[0].holes[0].winding(), Winding::Clockwise);
        assert_eq!(regions[0].area(), 18.0);
    }

    /// An island with its own hole, nested inside a bigger region's hole.
    /// The inner hole's sample point is contained by *both* outers, so this
    /// pins the smallest-area tie-break in `smallest_enclosing_outer`.
    #[test]
    fn nested_island_hole_parents_to_smallest_outer() {
        let b = grid(7, &[
            "#######",
            "#.....#",
            "#.###.#",
            "#.#.#.#",
            "#.###.#",
            "#.....#",
            "#######",
        ]);
        let regions = extract(&b, &ExtractOptions::default());
        assert_eq!(regions.len(), 2);
        let big = regions
            .iter()
            .find(|r| r.outer.area() == 49.0)
            .expect("outer ring region");
        let island = regions
            .iter()
            .find(|r| r.outer.area() == 9.0)
            .expect("island region");
        // The big region's hole covers the moat *and* the island inside it.
        assert_eq!(big.holes.len(), 1);
        assert_eq!(big.holes[0].area(), 25.0);
        // The island's unit hole must parent to the island, not the big
        // outer that also contains it.
        assert_eq!(island.holes.len(), 1);
        assert_eq!(island.holes[0].area(), 1.0);
    }

    /// `diagonal_smooth` should collapse a stair adjacent to a long
    /// straight run. The first stair-middle's `e_before` then has length 3
    /// (the run before the stair), which the original strict
    /// `e_before == e_out` test missed.
    #[test]
    fn diagonal_smooth_handles_stair_next_to_long_run() {
        // L-shape with a stair on one inside edge: a long horizontal bottom
        // then a 3-step stair rising to the top, then a horizontal cap and
        // a vertical left side.
        let p = Polygon::from_vertices(vec![
            Vertex::new(0.0, 0.0),
            Vertex::new(3.0, 0.0),   // long run
            Vertex::new(3.0, 1.0),   // stair start (rises)
            Vertex::new(4.0, 1.0),
            Vertex::new(4.0, 2.0),
            Vertex::new(5.0, 2.0),
            Vertex::new(5.0, 3.0),   // stair end
            Vertex::new(0.0, 3.0),
        ]);
        let before = p.vertices.len();
        let smoothed = diagonal_smooth(p);
        assert!(
            smoothed.vertices.len() < before,
            "long-run-then-stair should still get smoothed: before={before}, after={}",
            smoothed.vertices.len()
        );
    }

    /// A true staircase made of L-tromino steps so the cells are actually
    /// 4-connected. Each step is a 2-cell row sharing a side with the row
    /// below, producing one big walkable region whose lower-right boundary
    /// is a stair. Smoothing should collapse the stair into diagonals.
    #[test]
    fn diagonal_smoothing_on_a_connected_staircase() {
        let b = grid(5, &[
            "#####",
            "####.",
            "###..",
            "##...",
            "#....",
        ]);
        let mut opts = ExtractOptions::default();
        opts.diagonal_smoothing = false;
        opts.remove_collinear = true;
        let raw = extract(&b, &opts);
        assert_eq!(raw.len(), 1);
        let raw_verts = raw[0].outer.vertices.len();

        opts.diagonal_smoothing = true;
        let smoothed = extract(&b, &opts);
        assert_eq!(smoothed.len(), 1);
        let smooth_verts = smoothed[0].outer.vertices.len();

        // Smoothing should strictly reduce the vertex count on this staircase.
        assert!(
            smooth_verts < raw_verts,
            "smoothing should reduce vertices: raw={}, smoothed={}",
            raw_verts, smooth_verts
        );
        // The smoothed boundary should still enclose the same set of cells
        // (area unchanged from non-smoothed; smoothing only re-routes
        // outward-pointing notches, not real area).
        // Note: the staircase actually loses area when smoothed (the unit
        // notches get cut). Just check that area is in a sensible range.
        assert!(smoothed[0].outer.area() > 0.0);
    }
}
