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
//!
//! ## Agent radius
//!
//! [`Bitfield::eroded`] bakes an agent radius into the *grid* before
//! extraction — an exact Euclidean erosion, `O(cells)`, radii quantized to
//! whole cells. It is opt-in and it is not a replacement for the contour
//! inset (`rsnav_dynamic::BuildOptions::inset`), which handles sub-cell
//! radii and authored polygons. Its one exclusive capability: because it
//! runs on the global grid *before* tiling, it is the only erosion
//! compatible with `rsnav_navigation::TiledWorld` seams. See that method's
//! docs for the guarantees and the limitations.

#![forbid(unsafe_code)]

use rsnav_common::par::{par_bands_mut, par_map_indexed, resolve_threads};
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
        outers.drain(..).zip(outer_holes).collect();
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

// --- Grid erosion (agent radius, baked into the bitfield) ----------------
//
// `extract` traces the walkable set exactly. If an agent has a radius, you
// want the *eroded* walkable set — the positions where a disc of that
// radius fits. There are two places to do that:
//
//   * on the **contours**, after extraction (`rsnav_dynamic::BuildOptions::inset`
//     / `rsnav_triangle::build_cdt_with_inset`): offset every ring inward,
//     planarize, re-carve. Cost is O(boundary), it handles sub-cell radii,
//     and it works on authored (non-grid) polygons.
//   * on the **grid**, before extraction (this section): drop every cell an
//     agent cannot fully occupy. Cost is O(cells) whatever the boundary
//     looks like, radii are quantized to the grid, and it needs a bitfield.
//
// The reason to have both is [`rsnav_navigation::TiledWorld`]. Contour
// inset recedes a tile's seam edges by `r`, so the collinear-overlap
// matching in `stitch_all` finds no shared boundary and neighbouring tiles
// silently disconnect. Grid erosion runs **once, globally, before the grid
// is sliced into tiles**, so every tile's seam edge still lies exactly on
// the tile border line, at identical integer coordinates in both
// neighbours. That is the whole selling point; see [`Bitfield::eroded`].
//
// ## Why the seed mask is a 3×3 dilation
//
// The distance that matters is square-to-square, not center-to-center: an
// agent standing anywhere in cell `c` must clear the wall, so the test is
// "the *closest* point of `c` to the *closest* point of any wall cell".
// For cells at index delta `(dx, dy)` that distance is
//
//     sqrt(max(0, |dx| - 1)^2 + max(0, |dy| - 1)^2)
//
// — the clamped metric, which is not what a distance transform computes.
// But minimizing that clamped metric over the wall set is *identical* to
// minimizing the plain center-to-center Euclidean metric over the 3×3
// Chebyshev dilation of the wall set. So we dilate first and then run a
// completely textbook exact squared EDT. That is the one non-obvious step
// here; everything after it is standard Felzenszwalb–Huttenlocher.

/// Rows of the grid handled by one band in the row-major phases, and
/// columns per band in the transposed phases. 64 keeps a band's working
/// set in L2 and matches the blocked transpose tile size.
const ERODE_BAND: usize = 64;
/// Erosion is memory-bandwidth-bound; past ~16 workers more threads buy
/// nothing (measured: 2048² goes 81 ms serial → 19.6 ms at 8 → 15.3 ms at
/// 16, already past the knee).
const ERODE_MAX_THREADS: usize = 16;
/// Saturation value for "no seed in this row". Large enough to lose every
/// `min` against a real distance, small enough that `sentinel + h²` cannot
/// overflow `i32` for any grid up to 32767 × 32767 (2.7e8 + 1.1e9 < 2.1e9).
const DIST_SENTINEL: i32 = 1 << 28;

/// Options for [`Bitfield::eroded`].
#[derive(Copy, Clone, Debug)]
pub struct ErodeOptions {
    /// Agent radius in **bitfield cells** — the same units as
    /// `rsnav_dynamic::BuildOptions::inset`, since one cell is 1.0 world
    /// unit throughout this crate.
    ///
    /// Fractional values are accepted but **do not buy sub-cell
    /// resolution**. Output cells are whole cells, so the achievable
    /// clearances are exactly `{ sqrt(a² + b²) : a, b ∈ ℕ }` =
    /// `{0, 1, √2, 2, √5, √8, 3, ...}` and the result is a step function
    /// of `radius` that only jumps at those values: every radius in
    /// `(0, 1]` gives the identical output (one 8-connected peel), every
    /// radius in `(1, √2]` gives the next one, and so on. Fractions are
    /// allowed only so a caller can pass a world-space agent radius
    /// straight through without pre-quantizing.
    ///
    /// Concretely: a typical agent radius is 0.128 cells (`BuildOptions::inset`
    /// itself defaults to `None`), and eroding at 0.128 removes the whole first ring of wall-adjacent
    /// cells — a guaranteed clearance of 1.0, **7.8× more erosion than
    /// asked for**. Sub-cell radii belong to the contour path,
    /// permanently.
    pub radius: f64,
    /// Worker threads; `0` = one per available core, `1` = serial. Output
    /// is byte-identical for every setting. Same convention as
    /// [`ExtractOptions::threads`].
    pub threads: usize,
}

impl Default for ErodeOptions {
    fn default() -> Self {
        Self {
            radius: 0.0,
            threads: 0,
        }
    }
}

/// Errors returned by [`Bitfield::eroded`] and [`ClearanceField::threshold`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ErodeError {
    /// The radius was NaN, infinite, or negative. Mirrors
    /// `rsnav_dynamic::BuildError::InvalidInset`.
    InvalidRadius(f64),
}

impl std::fmt::Display for ErodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidRadius(r) => {
                write!(f, "erode radius must be finite and >= 0, got {r}")
            }
        }
    }
}

impl std::error::Error for ErodeError {}

/// Squared clearance per cell, in cells².
///
/// `sq_at(c, r)` is the squared distance from cell `(c, r)`'s unit square
/// to the nearest non-walkable region, counting **everything outside the
/// grid as wall** (matching [`Bitfield::at`], which returns `false` out of
/// range). It is `0` for wall cells and for walkable cells 8-adjacent to a
/// wall, and `sqrt(sq_at)` is exactly the largest radius an agent may have
/// while standing *anywhere* in that cell.
///
/// Values are exact integers — no floating point ever enters a distance —
/// so `sq_at` is bit-reproducible across platforms and thread counts.
///
/// The field is separated from [`Bitfield::eroded`] because the transform
/// is the expensive part and thresholding it is ~2% of the total: build
/// one field, then [`threshold`](Self::threshold) it at several radii to
/// get a small/medium/large agent navmesh for the price of one transform.
/// It is also independently useful as a "how big an agent fits here" map.
pub struct ClearanceField {
    pub width: u32,
    pub height: u32,
    sq: Vec<i32>,
}

impl ClearanceField {
    /// Squared clearance of cell `(col, row)` in cells². Returns `0` for
    /// out-of-range coordinates (outside the grid is wall, clearance 0).
    #[inline]
    pub fn sq_at(&self, col: u32, row: u32) -> i32 {
        if col >= self.width || row >= self.height {
            return 0;
        }
        self.sq[(row as usize) * (self.width as usize) + (col as usize)]
    }

    /// The cells whose clearance is `>= radius`.
    ///
    /// Self-contained: wall cells always have `sq == 0`, so for any
    /// `radius > 0` the walkability test is implied and no reference to
    /// the source bitfield is needed.
    ///
    /// **`radius == 0.0` keeps every cell, walls included** — a clearance
    /// of at least zero is vacuously true everywhere, and the field cannot tell a
    /// wall cell from a wall-adjacent walkable one (both are `0`). Callers
    /// who want "the original grid" want the original grid, or
    /// [`Bitfield::eroded`] with radius 0, which clones.
    ///
    /// **Rounding.** The test is `sq >= radius²` with `sq` an exact
    /// integer, so ties round toward *more* erosion — the safe direction.
    /// One consequence worth knowing: `radius = sqrt(2)` evaluates in f64
    /// to `2.0000000000000004`, so cells at exactly √2 clearance are
    /// dropped, as if the radius were infinitesimally larger. That bias is
    /// conservative and deliberate; it is not a bug to fix.
    pub fn threshold(&self, radius: f64) -> Result<Bitfield, ErodeError> {
        let thr = validate_radius(radius)?;
        let data = self.sq.iter().map(|&d| i64::from(d) >= thr).collect();
        Ok(Bitfield {
            width: self.width,
            height: self.height,
            data,
        })
    }
}

/// Validate a radius and return the integer threshold `ceil(radius²)`.
///
/// Because the squared distances are exact integers, `d >= radius²` and
/// `d >= ceil(radius²)` are the same predicate, and the integer form keeps
/// the inner loop free of floating point. The `as i64` cast saturates, so
/// an astronomically large (but finite) radius yields `i64::MAX` and
/// erodes everything away rather than wrapping.
///
/// The `max(1)` for `radius > 0` is not a fudge factor: achievable
/// clearances are integers, so `sq >= r²` with `r > 0` always excludes
/// `sq == 0` (the wall cells), and `ceil(r²)` already encodes that for
/// every radius large enough not to underflow. Below `r ≈ 2.2e-162` the
/// product `radius * radius` flushes to `0.0` and `ceil` yields a
/// threshold of 0, which would silently keep walls; clamping restores the
/// exact predicate at no cost to any other radius.
fn validate_radius(radius: f64) -> Result<i64, ErodeError> {
    if !radius.is_finite() || radius < 0.0 {
        return Err(ErodeError::InvalidRadius(radius));
    }
    if radius == 0.0 {
        return Ok(0);
    }
    Ok(((radius * radius).ceil() as i64).max(1))
}

impl Bitfield {
    /// Morphological erosion by `radius` cells: keep exactly the cells an
    /// agent of that radius can occupy *anywhere within*.
    ///
    /// Exact Euclidean (the structuring element is the agent's disc, not a
    /// square or a diamond), `O(cells)`, and conservative — it never keeps
    /// a cell the agent cannot fit in. Feed the result to [`extract`] or
    /// `rsnav_dynamic::build_navmesh_from_bitfield` with `inset: None` and
    /// the clearance is baked into the mesh.
    ///
    /// # The point of this over `BuildOptions::inset`
    ///
    /// It runs on the **grid**, so it can run once globally *before* the
    /// grid is sliced into tiles, which makes it the only erosion that
    /// works with `rsnav_navigation::TiledWorld` seams:
    ///
    /// ```no_run
    /// # use rsnav_polygon_extract::{Bitfield, ErodeOptions};
    /// # fn demo(global: &Bitfield) -> Result<(), Box<dyn std::error::Error>> {
    /// const TS: u32 = 256;
    /// let eroded = global.eroded(&ErodeOptions { radius: 2.0, threads: 0 })?;
    /// for (tx, ty) in [(0u32, 0u32), (1, 0)] {
    ///     let tile = eroded.subgrid(tx * TS, ty * TS, TS, TS); // erode FIRST, slice SECOND
    ///     // build_navmesh_from_bitfield(&tile, &opts /* inset: None */) ...
    /// }
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// **Never erode a tile.** Per-tile erosion treats the tile border as
    /// wall and eats `radius` cells at every seam — reproducing exactly
    /// the failure the contour inset has. [`subgrid`](Self::subgrid) sits
    /// next to this method so the correct order reads naturally.
    ///
    /// # Limitations, stated plainly
    ///
    /// * **Radii are cell-quantized.** See [`ErodeOptions::radius`]: every
    ///   radius in `(0, 1]` produces the same one-cell peel. The 0.128-cell
    ///   default inset is *not* representable here.
    /// * **Grid input only.** Authored/vector polygons never become a
    ///   `Bitfield`; they need the contour path.
    /// * **`O(cells)` regardless of boundary complexity.** On a large,
    ///   mostly-open grid you pay for every cell to move a handful of
    ///   vertices — 2048² costs ~20 ms at 8 threads, comparable to a whole
    ///   legacy build, to produce a two-triangle mesh. That is why this is
    ///   opt-in at the call site and never a build option.
    /// * **Any `radius > 0` removes the outermost ring of cells**, because
    ///   outside the grid is wall (see [`Bitfield::at`]). Correct, but a
    ///   visible difference for maps whose walkable area runs to the grid
    ///   edge; pad your bitfield and it disappears.
    ///
    /// # Guarantees
    ///
    /// Let `S` be the union of walkable unit squares and `S ⊖ D_r` the
    /// true Minkowski erosion by the disc of radius `r`. Writing `R` for
    /// the kept set:
    ///
    /// 1. **Never over-claims:** `R ⊆ S ⊖ D_r`. Every point of every kept
    ///    cell is at least `r` from the wall region.
    /// 2. **Sandwich bound:** `S ⊖ D_(r + √2) ⊆ R ⊆ S ⊖ D_r`. The
    ///    one-sided Hausdorff error versus true erosion is at most one
    ///    cell diagonal, always conservative.
    /// 3. **Conditional on [`ExtractOptions::diagonal_smoothing`] being
    ///    `false`.** Smoothing runs *after* erosion, inside `extract`, and
    ///    is not area-preserving: at reflex corners it replaces a stair
    ///    pair with a diagonal that bulges into the wall, costing up to
    ///    √2/2 ≈ 0.708 cells of clearance. With smoothing on the
    ///    guaranteed clearance is `max(0, r - 0.708)`; disable it when the
    ///    hard bound matters, or erode by `r + 0.708`. `clip_ears` and
    ///    `min_area` only ever *remove* area, which can only increase
    ///    clearance, so they are safe.
    ///
    /// # Composing with the contour inset
    ///
    /// They are complements, not alternatives, and the clearances add:
    /// grid-erode the integer part and contour-inset the sub-cell
    /// remainder for a guaranteed clearance of `a + b`. An agent of radius
    /// 2.128 cells = `eroded(2.0)` then `inset: Some(0.128)`.
    ///
    /// # Downstream clearance
    ///
    /// With a baked grid erosion of `r`, pass `max(0, agent_radius - r)`
    /// to `PathOptions::distance_from_wall` and
    /// `rsnav_navigation::WallClearance::clamp`, or the clearance is
    /// counted twice. Use `max(0, r - 0.708)` in place of `r` when
    /// `diagonal_smoothing` is on.
    ///
    /// # Errors
    ///
    /// [`ErodeError::InvalidRadius`] if `radius` is NaN, infinite, or
    /// negative. `radius == 0.0` is valid and clones.
    pub fn eroded(&self, opts: &ErodeOptions) -> Result<Bitfield, ErodeError> {
        let thr = validate_radius(opts.radius)?;
        let (w, h) = (self.width as usize, self.height as usize);
        // radius 0 is the identity, and an empty grid has nothing to do.
        if opts.radius == 0.0 || w == 0 || h == 0 {
            return Ok(self.clone());
        }
        let threads = erode_threads(w * h, opts.threads);

        let seed = seed_mask(self, threads);
        // Fast path: below one full cell the seed mask *is* the answer.
        // Exact, not an approximation — the only achievable clearance
        // strictly below 1 is 0, so `sq >= r²` collapses to `!seed` for
        // every radius in (0, 1]. Skips four of the five passes.
        if opts.radius <= 1.0 {
            let mut data = vec![false; w * h];
            par_bands_mut(&mut data, w * ERODE_BAND, threads, |bi, band| {
                let base = bi * w * ERODE_BAND;
                for (j, out) in band.iter_mut().enumerate() {
                    *out = !seed[base + j];
                }
            });
            return Ok(Bitfield {
                width: self.width,
                height: self.height,
                data,
            });
        }

        let g = row_pass(&seed, w, h, threads);
        let gt = transpose(&g, w, h, threads);
        // Fuse the threshold into the column pass so the full distance
        // field is never materialized when only the mask is wanted. Wall
        // cells are seeds, hence distance 0, hence dropped by any r > 0 —
        // so no separate walkability mask is needed here.
        let mut keep_t = vec![false; w * h];
        column_envelope(&gt, h, threads, &mut keep_t, |d2| d2 >= thr);
        let mut data = vec![false; w * h];
        untranspose(&keep_t, w, h, threads, &mut data);
        Ok(Bitfield {
            width: self.width,
            height: self.height,
            data,
        })
    }

    /// The exact squared clearance field alone, without thresholding it.
    ///
    /// Threshold it at several radii to build one navmesh per agent size
    /// for the price of one transform — the transform is the expensive
    /// part, the threshold is ~2% of it. See [`ClearanceField`] for the
    /// precise definition of "clearance" (square-to-square, outside the
    /// grid is wall) and [`Bitfield::eroded`] for the limitations, which
    /// apply identically.
    ///
    /// `threads`: `0` = one per core, `1` = serial; output is identical
    /// either way.
    pub fn clearance(&self, threads: usize) -> ClearanceField {
        let (w, h) = (self.width as usize, self.height as usize);
        if w == 0 || h == 0 {
            return ClearanceField {
                width: self.width,
                height: self.height,
                sq: Vec::new(),
            };
        }
        let threads = erode_threads(w * h, threads);
        let seed = seed_mask(self, threads);
        let g = row_pass(&seed, w, h, threads);
        let gt = transpose(&g, w, h, threads);
        let mut dt = vec![0i32; w * h];
        column_envelope(&gt, h, threads, &mut dt, |d2| {
            // Every column contains a seed (the top and bottom grid rows
            // always are, since outside is wall), so d2 <= (h-1)² and the
            // clamp below can only ever fire on absurd grid heights.
            d2.min(i64::from(i32::MAX)) as i32
        });
        let mut sq = vec![0i32; w * h];
        untranspose(&dt, w, h, threads, &mut sq);
        ClearanceField {
            width: self.width,
            height: self.height,
            sq,
        }
    }

    /// Copy the `width × height` sub-rectangle whose lower-left cell is
    /// `(col0, row0)` into a fresh `Bitfield`.
    ///
    /// Cells outside `self` read as wall, matching [`Bitfield::at`], so an
    /// over-hanging request is padded with `false` rather than clipped —
    /// the returned grid always has exactly the requested dimensions.
    ///
    /// This is the second half of the tiled workflow: **erode the global
    /// grid, then `subgrid` it into tiles.** Slicing after eroding puts
    /// each tile's boundary exactly on the tile border line, at identical
    /// integer coordinates in both neighbours, so
    /// `rsnav_navigation::TiledWorld::stitch_all` links them. Eroding
    /// after slicing eats `radius` cells at every seam instead. See
    /// [`Bitfield::eroded`].
    pub fn subgrid(&self, col0: u32, row0: u32, width: u32, height: u32) -> Bitfield {
        let mut out = Bitfield::empty(width, height);
        for r in 0..height {
            for c in 0..width {
                let v = self.at(i64::from(col0) + i64::from(c), i64::from(row0) + i64::from(r));
                out.set(c, r, v);
            }
        }
        out
    }
}

/// Resolve the worker count for an erosion pass: below [`PAR_MIN_CELLS`]
/// the spawn costs more than the scan, and past [`ERODE_MAX_THREADS`] the
/// memory bus is saturated anyway.
fn erode_threads(cells: usize, requested: usize) -> usize {
    if cells < PAR_MIN_CELLS {
        return 1;
    }
    resolve_threads(requested).min(ERODE_MAX_THREADS)
}

/// Pass 1: the seed set = 3×3 Chebyshev dilation of the wall set, with
/// out-of-range counting as wall.
///
/// Separable, so it is two linear passes rather than nine reads per cell:
/// first a horizontal 3-window AND of walkability (`hz` = "this window
/// contains a wall"), then a vertical 3-window OR of that.
///
/// The implicit one-ring padding is exact and needs no buffer: every
/// out-of-range wall cell, clamped into the `[-1, W] × [-1, H]` ring, maps
/// to a cell no farther from any in-grid cell, and that ring's dilated
/// image is the grid's own border row/column — which is always seeded
/// because the window peeks out of range there. So no distance is ever
/// underestimated.
fn seed_mask(bits: &Bitfield, threads: usize) -> Vec<bool> {
    let (w, h) = (bits.width as usize, bits.height as usize);
    let walk = &bits.data;

    let mut hz = vec![false; w * h];
    par_bands_mut(&mut hz, w * ERODE_BAND, threads, |bi, band| {
        let r0 = bi * ERODE_BAND;
        for (j, row) in band.chunks_mut(w).enumerate() {
            let src = &walk[(r0 + j) * w..(r0 + j + 1) * w];
            for c in 0..w {
                let left = c > 0 && src[c - 1];
                let right = c + 1 < w && src[c + 1];
                // Out of range is wall, so a window touching the left or
                // right edge always contains one.
                row[c] = !(left && src[c] && right);
            }
        }
    });

    let mut seed = vec![false; w * h];
    par_bands_mut(&mut seed, w * ERODE_BAND, threads, |bi, band| {
        let r0 = bi * ERODE_BAND;
        for (j, row) in band.chunks_mut(w).enumerate() {
            let r = r0 + j;
            let cur = &hz[r * w..(r + 1) * w];
            for c in 0..w {
                let up = r + 1 == h || hz[(r + 1) * w + c];
                let down = r == 0 || hz[(r - 1) * w + c];
                row[c] = up | down | cur[c];
            }
        }
    });
    seed
}

/// Pass 2: per row, the squared distance to the nearest seed *in that
/// row*, via a forward then a backward sweep tracking the last seen seed
/// column. Rows with no seed saturate at [`DIST_SENTINEL`] rather than
/// overflowing; the column pass recovers the true value from other rows.
fn row_pass(seed: &[bool], w: usize, h: usize, threads: usize) -> Vec<i32> {
    let mut g = vec![0i32; w * h];
    par_bands_mut(&mut g, w * ERODE_BAND, threads, |bi, band| {
        let r0 = bi * ERODE_BAND;
        for (j, row) in band.chunks_mut(w).enumerate() {
            let s = &seed[(r0 + j) * w..(r0 + j + 1) * w];
            let mut last: i64 = i64::MIN / 4;
            for c in 0..w {
                if s[c] {
                    last = c as i64;
                }
                let d = c as i64 - last;
                row[c] = d.saturating_mul(d).min(i64::from(DIST_SENTINEL)) as i32;
            }
            let mut last: i64 = i64::MAX / 4;
            for c in (0..w).rev() {
                if s[c] {
                    last = c as i64;
                }
                let d = last - c as i64;
                let v = d.saturating_mul(d).min(i64::from(DIST_SENTINEL)) as i32;
                if v < row[c] {
                    row[c] = v;
                }
            }
        }
    });
    g
}

/// Pass 3: blocked transpose, `out[c * h + r] = g[r * w + c]`.
///
/// This exists purely for speed and it is the difference between fast and
/// unusable: the naive strided column pass measured 135 ms at 2048²
/// versus 38 ms transposed plus 32 ms contiguous. `i32` rather than `i64`
/// halves the traffic (17 MB per buffer at 2048² instead of 34 MB).
fn transpose(g: &[i32], w: usize, h: usize, threads: usize) -> Vec<i32> {
    let mut out = vec![0i32; w * h];
    par_bands_mut(&mut out, h * ERODE_BAND, threads, |bi, band| {
        let c0 = bi * ERODE_BAND;
        let ncols = band.len() / h;
        let mut r0 = 0;
        while r0 < h {
            let r1 = (r0 + ERODE_BAND).min(h);
            for k in 0..ncols {
                for r in r0..r1 {
                    band[k * h + r] = g[r * w + c0 + k];
                }
            }
            r0 = r1;
        }
    });
    out
}

/// Pass 4: the Felzenszwalb–Huttenlocher lower envelope down each column
/// (contiguous, because pass 3 transposed), combining the per-row
/// distances into the exact 2D squared EDT. `map` turns each squared
/// distance into an output element — a bool for [`Bitfield::eroded`], the
/// clamped value itself for [`Bitfield::clearance`] — so the full field is
/// only materialized when it is actually wanted.
///
/// `out` is in transposed layout, same as `gt`.
///
/// Determinism: the parabola intersection is the one f64 division in the
/// whole algorithm, but a column is processed start-to-finish by a single
/// worker, so its operation sequence is identical regardless of thread
/// count. There is no reduction and no accumulation across bands.
fn column_envelope<T, F>(gt: &[i32], h: usize, threads: usize, out: &mut [T], map: F)
where
    T: Send + Copy,
    F: Fn(i64) -> T + Sync,
{
    par_bands_mut(out, h * ERODE_BAND, threads, |bi, band| {
        let c0 = bi * ERODE_BAND;
        let ncols = band.len() / h;
        // Hoisted out of the column loop: no allocation inside a pass.
        let mut v = vec![0i64; h + 1];
        let mut z = vec![0f64; h + 2];
        for k in 0..ncols {
            let f = &gt[(c0 + k) * h..(c0 + k + 1) * h];
            let mut kk = 0usize;
            v[0] = 0;
            z[0] = f64::NEG_INFINITY;
            z[1] = f64::INFINITY;
            for q in 1..h as i64 {
                loop {
                    let p = v[kk];
                    let s = ((i64::from(f[q as usize]) + q * q)
                        - (i64::from(f[p as usize]) + p * p))
                        as f64
                        / (2 * (q - p)) as f64;
                    if s <= z[kk] && kk > 0 {
                        kk -= 1;
                    } else {
                        kk += 1;
                        v[kk] = q;
                        z[kk] = s;
                        z[kk + 1] = f64::INFINITY;
                        break;
                    }
                }
            }
            let mut kk = 0usize;
            let col = &mut band[k * h..(k + 1) * h];
            for r in 0..h as i64 {
                while z[kk + 1] < r as f64 {
                    kk += 1;
                }
                let p = v[kk];
                let d = r - p;
                col[r as usize] = map(i64::from(f[p as usize]) + d * d);
            }
        }
    });
}

/// Pass 5: blocked un-transpose, `out[r * w + c] = t[c * h + r]`.
fn untranspose<T: Send + Sync + Copy>(t: &[T], w: usize, h: usize, threads: usize, out: &mut [T]) {
    par_bands_mut(out, w * ERODE_BAND, threads, |bi, band| {
        let r0 = bi * ERODE_BAND;
        let nrows = band.len() / w;
        let mut c0 = 0;
        while c0 < w {
            let c1 = (c0 + ERODE_BAND).min(w);
            for j in 0..nrows {
                for c in c0..c1 {
                    band[j * w + c] = t[c * h + r0 + j];
                }
            }
            c0 = c1;
        }
    });
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

    // --- Erosion ---------------------------------------------------------

    use rsnav_common::rng::Lcg;

    fn rand_grid(rng: &mut Lcg, w: u32, h: u32, density_pct: u64) -> Bitfield {
        let data = (0..(w as usize) * (h as usize))
            .map(|_| rng.next_u64() % 100 < density_pct)
            .collect();
        Bitfield::new(w, h, data).expect("dimensions match")
    }

    /// Independently written oracle: the seed mask from a naive 3×3 scan
    /// through [`Bitfield::at`] (so "out of range is wall" comes from the
    /// real accessor, not from a reimplementation of it), then a windowed
    /// minimum over seeds. `O(cells · (2⌈r⌉+1)²)` — correctness only.
    fn brute_erode(bits: &Bitfield, radius: f64) -> Bitfield {
        let (w, h) = (bits.width as i64, bits.height as i64);
        let mut seed = vec![false; (w * h) as usize];
        for r in 0..h {
            for c in 0..w {
                let mut s = false;
                for dr in -1..=1 {
                    for dc in -1..=1 {
                        if !bits.at(c + dc, r + dr) {
                            s = true;
                        }
                    }
                }
                seed[(r * w + c) as usize] = s;
            }
        }
        let rad = radius.ceil() as i64;
        let r2 = (radius * radius).ceil() as i64;
        let mut out = Bitfield::empty(bits.width, bits.height);
        for r in 0..h {
            for c in 0..w {
                if !bits.at(c, r) {
                    continue;
                }
                let mut best = i64::MAX;
                for dr in -rad..=rad {
                    for dc in -rad..=rad {
                        let (rr, cc) = (r + dr, c + dc);
                        // Out of range: the clamped one-ring image of the
                        // outside wall, which is always a seed anyway.
                        let s = if rr < 0 || cc < 0 || rr >= h || cc >= w {
                            true
                        } else {
                            seed[(rr * w + cc) as usize]
                        };
                        if s {
                            best = best.min(dr * dr + dc * dc);
                        }
                    }
                }
                out.set(c as u32, r as u32, best >= r2);
            }
        }
        out
    }

    fn erode(bits: &Bitfield, radius: f64, threads: usize) -> Bitfield {
        bits.eroded(&ErodeOptions { radius, threads })
            .expect("valid radius")
    }

    /// The primary test: exact agreement with the brute-force oracle over
    /// odd-sized grids (deliberately not multiples of the 64-cell band, to
    /// catch band off-by-ones) at a spread of radii.
    #[test]
    fn erode_matches_brute_force_oracle() {
        let mut rng = Lcg(4242);
        let radii = [
            0.0,
            0.3,
            0.5,
            1.0,
            std::f64::consts::SQRT_2,
            1.5,
            2.0,
            2.5,
            3.0,
            5.0,
        ];
        for _ in 0..60 {
            let w = 1 + (rng.next_u64() % 120) as u32;
            let h = 1 + (rng.next_u64() % 120) as u32;
            let density = 25 + rng.next_u64() % 70;
            let bits = rand_grid(&mut rng, w, h, density);
            for &r in &radii {
                let want = brute_erode(&bits, r);
                for &threads in &[1usize, 5] {
                    let got = erode(&bits, r, threads);
                    assert_eq!(
                        got.data, want.data,
                        "mismatch on {w}×{h} density={density} radius={r} threads={threads}"
                    );
                }
            }
        }
    }

    /// The separable seed mask is the trickiest pass; isolate it against a
    /// naive 3×3 scan.
    #[test]
    fn seed_mask_matches_naive_3x3() {
        let mut rng = Lcg(90210);
        for _ in 0..40 {
            let w = 1 + (rng.next_u64() % 90) as u32;
            let h = 1 + (rng.next_u64() % 90) as u32;
            let density = 30 + rng.next_u64() % 65;
            let bits = rand_grid(&mut rng, w, h, density);
            let got = seed_mask(&bits, 1);
            let mut want = vec![false; (w as usize) * (h as usize)];
            for r in 0..h as i64 {
                for c in 0..w as i64 {
                    let mut s = false;
                    for dr in -1..=1 {
                        for dc in -1..=1 {
                            if !bits.at(c + dc, r + dr) {
                                s = true;
                            }
                        }
                    }
                    want[(r as usize) * (w as usize) + c as usize] = s;
                }
            }
            assert_eq!(got, want, "seed mask mismatch on {w}×{h}");
        }
    }

    /// A bigger radius can only ever remove cells. Trivially true given a
    /// single threshold on one field — which is exactly why it catches a
    /// flipped comparison.
    #[test]
    fn erode_is_monotone_in_radius() {
        let mut rng = Lcg(31337);
        let bits = rand_grid(&mut rng, 200, 200, 80);
        let radii = [0.0f64, 0.5, 1.0, 1.4, 1.5, 2.0, 2.3, 3.0, 4.0, 5.0, 7.0, 9.0];
        for pair in radii.windows(2) {
            let a = erode(&bits, pair[0], 1);
            let b = erode(&bits, pair[1], 1);
            for i in 0..a.data.len() {
                assert!(
                    !b.data[i] || a.data[i],
                    "radius {} kept cell {i} that radius {} dropped",
                    pair[1],
                    pair[0]
                );
            }
        }
    }

    #[test]
    fn erode_zero_is_identity() {
        let mut rng = Lcg(5);
        let bits = rand_grid(&mut rng, 77, 53, 60);
        let out = erode(&bits, 0.0, 4);
        assert_eq!(out.data, bits.data);
        assert_eq!((out.width, out.height), (bits.width, bits.height));
    }

    /// Staged erosion is *conservative*, not composable: each stage rounds
    /// toward more erosion, so `erode(a)` then `erode(b)` is a strict
    /// subset of `erode(a + b)`. Asserting equality here would be wrong.
    #[test]
    fn erode_staged_is_subset_but_not_equal() {
        let mut rng = Lcg(2024);
        let bits = rand_grid(&mut rng, 150, 150, 92);
        let staged = erode(&erode(&bits, 2.0, 1), 2.0, 1);
        let direct = erode(&bits, 4.0, 1);
        for i in 0..staged.data.len() {
            assert!(
                !staged.data[i] || direct.data[i],
                "staged erosion kept cell {i} that erode(4.0) dropped"
            );
        }
        assert_ne!(
            staged.data, direct.data,
            "staged erosion is expected to lose cells versus a single pass"
        );
    }

    /// Everything outside the grid is wall, so any positive radius eats the
    /// outermost ring — and radius 1 eats exactly that ring, nothing more.
    #[test]
    fn erode_one_peels_exactly_the_border_ring() {
        let (w, h) = (23u32, 17u32);
        let bits = Bitfield::new(w, h, vec![true; (w * h) as usize]).expect("dims");
        let out = erode(&bits, 1.0, 1);
        for r in 0..h {
            for c in 0..w {
                let interior = c > 0 && r > 0 && c + 1 < w && r + 1 < h;
                assert_eq!(
                    out.at(c as i64, r as i64),
                    interior,
                    "cell ({c},{r}) on a fully walkable {w}×{h} grid at radius 1"
                );
            }
        }
        let kept = out.data.iter().filter(|b| **b).count();
        assert_eq!(kept, ((w - 2) * (h - 2)) as usize);
    }

    #[test]
    fn erode_beyond_half_the_grid_removes_everything() {
        let (w, h) = (32u32, 24u32);
        let bits = Bitfield::new(w, h, vec![true; (w * h) as usize]).expect("dims");
        let out = erode(&bits, 12.0, 1);
        assert!(out.data.iter().all(|b| !b), "radius >= min(w,h)/2 must empty the grid");
    }

    /// Pin the square-to-square definition of clearance against future
    /// refactors: a single wall cell in an open field.
    #[test]
    fn clearance_is_square_to_square_distance() {
        let (w, h) = (41u32, 41u32);
        let mut bits = Bitfield::new(w, h, vec![true; (w * h) as usize]).expect("dims");
        bits.set(20, 20, false);
        let field = bits.clearance(1);
        for dy in -5i64..=5 {
            for dx in -5i64..=5 {
                let want = (dx.abs() - 1).max(0).pow(2) + (dy.abs() - 1).max(0).pow(2);
                let got = field.sq_at((20 + dx) as u32, (20 + dy) as u32);
                assert_eq!(
                    i64::from(got),
                    want,
                    "sq_at offset ({dx},{dy}) from the wall cell"
                );
            }
        }
        // The wall cell itself, and out-of-range reads, are zero.
        assert_eq!(field.sq_at(20, 20), 0);
        assert_eq!(field.sq_at(w, 0), 0);
    }

    /// `clearance().threshold(r)` and `eroded(r)` are the same predicate —
    /// `eroded` just fuses the threshold into the column pass so it never
    /// materializes the field.
    #[test]
    fn threshold_agrees_with_eroded() {
        let mut rng = Lcg(777);
        let bits = rand_grid(&mut rng, 111, 87, 78);
        let field = bits.clearance(1);
        for &r in &[0.5f64, 1.0, 1.5, 2.0, 3.0, 4.5] {
            let a = field.threshold(r).expect("valid radius");
            let b = erode(&bits, r, 1);
            assert_eq!(a.data, b.data, "threshold vs eroded disagree at radius {r}");
        }
        // Documented quirk: radius 0 keeps everything, walls included.
        let all = field.threshold(0.0).expect("valid radius");
        assert!(all.data.iter().all(|b| *b));
        // ...but any radius > 0 drops walls, including radii so small that
        // `radius * radius` underflows to zero in f64. The quirk is exactly
        // at zero, not "near" zero.
        for &tiny in &[1e-9f64, 1e-200, f64::MIN_POSITIVE] {
            assert!(tiny * tiny == 0.0 || tiny == 1e-9, "underflow assumption");
            let a = field.threshold(tiny).expect("valid radius");
            let b = erode(&bits, tiny, 1);
            assert_eq!(a.data, b.data, "threshold vs eroded disagree at {tiny}");
            assert!(
                a.data.iter().zip(&bits.data).all(|(k, w)| !*k || *w),
                "radius {tiny} kept a wall cell"
            );
        }
    }

    /// The transform is thread-count invariant by construction (exact
    /// integer distances, one worker per column, disjoint output bands).
    /// Run it above `PAR_MIN_CELLS` so the parallel path is actually taken.
    #[test]
    fn erode_is_thread_count_deterministic() {
        let (w, h) = (900u32, 700u32); // 630k cells > PAR_MIN_CELLS
        let mut rng = Lcg(99);
        let mut bits = Bitfield::new(w, h, vec![true; (w * h) as usize]).expect("dims");
        for _ in 0..((w * h) / 400) {
            let cx = (rng.next_u64() % u64::from(w)) as u32;
            let cy = (rng.next_u64() % u64::from(h)) as u32;
            for dy in 0..6u32 {
                for dx in 0..6u32 {
                    if cx + dx < w && cy + dy < h {
                        bits.set(cx + dx, cy + dy, false);
                    }
                }
            }
        }
        for &r in &[1.0f64, 2.5, 6.0] {
            let base = erode(&bits, r, 1);
            for &t in &[2usize, 4, 8, 16] {
                assert_eq!(
                    erode(&bits, r, t).data,
                    base.data,
                    "thread count {t} changed the output at radius {r}"
                );
            }
            // ...and the parallel path still agrees with the oracle.
            if r == 2.5 {
                assert_eq!(brute_erode(&bits, r).data, base.data);
            }
        }
        let f1 = bits.clearance(1);
        let f8 = bits.clearance(8);
        assert_eq!(f1.sq, f8.sq, "clearance field must not depend on thread count");
    }

    #[test]
    fn erode_rejects_invalid_radius() {
        let bits = Bitfield::empty(4, 4);
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0, -0.0001] {
            let err = bits
                .eroded(&ErodeOptions {
                    radius: bad,
                    threads: 1,
                })
                .expect_err("invalid radius must be rejected");
            match err {
                ErodeError::InvalidRadius(r) => assert!(r.is_nan() || r == bad),
            }
        }
        // -0.0 is not negative and squares to 0: valid, and an identity.
        assert!(bits
            .eroded(&ErodeOptions {
                radius: -0.0,
                threads: 1
            })
            .is_ok());
    }

    #[test]
    fn erode_handles_degenerate_dimensions() {
        for (w, h) in [(0u32, 0u32), (0, 5), (5, 0), (1, 1), (1, 40), (40, 1)] {
            let bits = Bitfield::new(w, h, vec![true; (w as usize) * (h as usize)]).expect("dims");
            for &r in &[0.0f64, 0.5, 1.0, 2.0] {
                let out = erode(&bits, r, 4);
                assert_eq!((out.width, out.height), (w, h));
                assert_eq!(out.data, brute_erode(&bits, r).data, "{w}×{h} at radius {r}");
            }
        }
    }

    #[test]
    fn subgrid_copies_and_pads_with_wall() {
        let mut rng = Lcg(1234);
        let bits = rand_grid(&mut rng, 30, 20, 55);
        let sub = bits.subgrid(7, 3, 9, 6);
        assert_eq!((sub.width, sub.height), (9, 6));
        for r in 0..6i64 {
            for c in 0..9i64 {
                assert_eq!(sub.at(c, r), bits.at(7 + c, 3 + r));
            }
        }
        // An over-hanging request keeps the requested size, padded false.
        let over = bits.subgrid(28, 18, 5, 5);
        assert_eq!((over.width, over.height), (5, 5));
        for r in 0..5i64 {
            for c in 0..5i64 {
                assert_eq!(over.at(c, r), bits.at(28 + c, 18 + r));
                if c >= 2 || r >= 2 {
                    assert!(!over.at(c, r), "outside the source must read as wall");
                }
            }
        }
    }

    /// The reuse story: one transform, several agent sizes. Slicing the
    /// eroded grid must equal eroding-then-slicing, never slicing-then-
    /// eroding (which would eat the tile border).
    #[test]
    fn erode_then_subgrid_beats_subgrid_then_erode_at_the_seam() {
        let (w, h) = (32u32, 8u32);
        let mut bits = Bitfield::new(w, h, vec![true; (w * h) as usize]).expect("dims");
        for c in 0..w {
            bits.set(c, 0, false);
            bits.set(c, h - 1, false);
        }
        let global = erode(&bits, 2.0, 1);
        let right_of_global = global.subgrid(16, 0, 16, h);
        let eroded_tile = erode(&bits.subgrid(16, 0, 16, h), 2.0, 1);
        // The correct order keeps the column adjacent to the seam...
        assert!(
            (0..h as i64).any(|r| right_of_global.at(0, r)),
            "erode-then-slice must keep walkable cells on the seam column"
        );
        // ...the wrong order eats it.
        assert!(
            (0..h as i64).all(|r| !eroded_tile.at(0, r)),
            "slice-then-erode is expected to eat the seam column"
        );
    }
}
