//! Layer assignment: partition walkable spans into injective layers with
//! cuts on the boundary of the vertical-overlap set.
//!
//! The constraint a layer must satisfy is stronger than "one span per
//! column". Every layer is later triangulated as a 2D chart with one
//! height per vertex, so two spans whose cells share a grid **edge or
//! corner** would share chart vertices — which only works if the surface
//! is continuous between them. Hence the conflict rule: two spans
//! conflict when they occupy the same column, or when their cells are
//! 8-adjacent and their heights differ by more than the step height
//! (a cliff — the shared vertex would need two different heights).
//!
//! Assignment is greedy BFS over the walk-links: seed an unassigned
//! span, flood outward, and admit a span iff it conflicts with nothing
//! already in the layer. The frontier stops exactly where the surface
//! starts to overlap itself (a bridge deck coming back over the ground,
//! a ramp cresting beside the floor it rose from) — which is the rim of
//! the overlap set, the canonical place to cut. Open continuous ground
//! never conflicts with itself, so no cut ever lands in it. A
//! self-overlapping sheet (helix) conflicts with its own tail and splits
//! at the collision frontier — a residually arbitrary but valid cut.
//!
//! Walk-links that end up spanning two layers are the **seams**; they
//! are step-continuous floor by construction and become conformed
//! connection edges downstream. Links onto pruned dust layers become
//! walls.

use std::collections::VecDeque;

use rsnav_voxel::{CompactHeightfield, NEIGHBOR_DELTAS};

/// One walkable span (a surface cell) of the heightfield.
#[derive(Copy, Clone, Debug)]
pub struct Span {
    pub c: u32,
    pub r: u32,
    /// Walking height of the surface (top of the walkable voxel).
    pub z: f64,
    /// Voxel layer index of the surface. Step comparisons use this —
    /// integer arithmetic, no float fragility when a step lands exactly
    /// on the threshold.
    pub layer_idx: u32,
    /// Step-connected neighbor span ids per cardinal direction
    /// (`NEIGHBOR_DELTAS` order).
    pub links: [Option<u32>; 4],
}

/// Result of layer assignment.
#[derive(Clone, Debug)]
pub struct Assignment {
    pub spans: Vec<Span>,
    /// Layer id per span; `UNASSIGNED` for pruned spans.
    pub layer_of: Vec<u32>,
    pub layer_count: u32,
    /// Spans dropped with their dust layers.
    pub pruned_spans: usize,
    pub cols: u32,
    pub rows: u32,
    /// Prefix sums: spans of cell `(c, r)` occupy
    /// `cell_offsets[r*cols+c] .. cell_offsets[r*cols+c+1]` (spans are
    /// collected row-major, stacks ascending).
    pub cell_offsets: Vec<u32>,
}

impl Assignment {
    /// Span index range of cell `(c, r)`.
    #[inline]
    pub fn cell_spans(&self, c: u32, r: u32) -> std::ops::Range<usize> {
        let i = (r * self.cols + c) as usize;
        self.cell_offsets[i] as usize..self.cell_offsets[i + 1] as usize
    }
}

pub const UNASSIGNED: u32 = u32::MAX;

/// Flatten the heightfield's per-column stacks into an indexed span
/// array with resolved link ids, plus per-cell prefix offsets.
pub fn collect_spans(chf: &CompactHeightfield) -> (Vec<Span>, Vec<u32>) {
    // Column-major offsets so (c, r, si) → span id is O(1).
    let mut offsets = vec![0u32; (chf.cols * chf.rows) as usize + 1];
    for r in 0..chf.rows {
        for c in 0..chf.cols {
            let idx = (r * chf.cols + c) as usize;
            offsets[idx + 1] = offsets[idx] + chf.surfaces_at(c, r).len() as u32;
        }
    }
    let id_of = |c: u32, r: u32, si: u32| offsets[(r * chf.cols + c) as usize] + si;

    let mut spans = Vec::with_capacity(offsets[offsets.len() - 1] as usize);
    for r in 0..chf.rows {
        for c in 0..chf.cols {
            for (si, cell) in chf.surfaces_at(c, r).iter().enumerate() {
                let mut links = [None; 4];
                for (k, &(dc, dr)) in NEIGHBOR_DELTAS.iter().enumerate() {
                    if let Some(ni) = cell.neighbors[k] {
                        let nc = c as i32 + dc;
                        let nr = r as i32 + dr;
                        if nc >= 0 && nc < chf.cols as i32 && nr >= 0 && nr < chf.rows as i32 {
                            links[k] = Some(id_of(nc as u32, nr as u32, ni));
                        }
                    }
                }
                let _ = si;
                spans.push(Span {
                    c,
                    r,
                    z: chf.origin.z + (cell.layer as f64 + 1.0) * chf.cell_size,
                    layer_idx: cell.layer,
                    links,
                });
            }
        }
    }
    (spans, offsets)
}

/// Conflict-aware layer growth. `max_step_layers` is the walkable step
/// height in voxel layers (see `WalkabilityConfig::max_step_layers`);
/// `min_layer_spans` prunes dust layers.
///
/// Three phases, in decreasing certainty:
///
/// 1. **Plain floor.** Spans that can never conflict — single span in
///    their column, no cliff anywhere in the 8-neighborhood — are
///    grouped into base layers by walk-connectivity, atomically. Open
///    continuous ground is decided before anything contested is.
/// 2. **Wave adoption.** A multi-source BFS grows every base layer
///    outward simultaneously; a contested span (under a bridge, on a
///    ramp, ringing a cliff) joins the nearest layer that accepts it
///    without conflict. Distance ordering is the point: the floor
///    always claims its own surroundings before a structure rising off
///    it can climb far enough to compete (a single-seed greedy can
///    reach a platform *before* flooding the floor around it and then
///    cut open ground — the failure mode this phase exists to prevent).
/// 3. **Leftovers.** Whatever no base layer could adopt — bridge
///    decks, upper ramps, stacked floors — seeds new layers, grown the
///    same way, then a conflict-checked merge pass consolidates
///    fragments (two ramp flanks and their deck become one layer).
pub fn assign_layers(
    chf: &CompactHeightfield,
    max_step_layers: u32,
    min_layer_spans: usize,
) -> Assignment {
    let (spans, cell_offsets) = collect_spans(chf);
    let n = spans.len();
    let mut layer_of = vec![UNASSIGNED; n];
    // Per-layer plan-view book: cell → voxel layer of the span the
    // layer holds there. One entry per cell (injectivity by
    // construction).
    let mut books: Vec<std::collections::HashMap<(u32, u32), u32>> = Vec::new();

    // --- Phase 1: plain floor. ---
    //
    // `plain[s]`: s is the only span in its column and every span in
    // every 8-adjacent cell is within step reach.
    let cell_range = |c: u32, r: u32| {
        let i = (r * chf.cols + c) as usize;
        cell_offsets[i] as usize..cell_offsets[i + 1] as usize
    };
    let mut plain = vec![false; n];
    for (s, sp) in spans.iter().enumerate() {
        if cell_range(sp.c, sp.r).len() != 1 {
            continue;
        }
        let mut ok = true;
        'scan: for dr in -1i32..=1 {
            for dc in -1i32..=1 {
                if dc == 0 && dr == 0 {
                    continue;
                }
                let (nc, nr) = (sp.c as i32 + dc, sp.r as i32 + dr);
                if nc < 0 || nr < 0 || nc as u32 >= chf.cols || nr as u32 >= chf.rows {
                    continue;
                }
                for t in cell_range(nc as u32, nr as u32) {
                    if spans[t].layer_idx.abs_diff(sp.layer_idx) > max_step_layers {
                        ok = false;
                        break 'scan;
                    }
                }
            }
        }
        plain[s] = ok;
    }

    // Walk-connected components of the plain floor become base layers.
    let mut wave: VecDeque<u32> = VecDeque::new();
    for seed in 0..n as u32 {
        if !plain[seed as usize] || layer_of[seed as usize] != UNASSIGNED {
            continue;
        }
        let layer = books.len() as u32;
        books.push(std::collections::HashMap::new());
        let mut queue: VecDeque<u32> = VecDeque::new();
        layer_of[seed as usize] = layer;
        books[layer as usize].insert(
            (spans[seed as usize].c, spans[seed as usize].r),
            spans[seed as usize].layer_idx,
        );
        queue.push_back(seed);
        while let Some(s) = queue.pop_front() {
            for k in 0..4 {
                let Some(t) = spans[s as usize].links[k] else {
                    continue;
                };
                if !plain[t as usize] || layer_of[t as usize] != UNASSIGNED {
                    continue;
                }
                let sp = &spans[t as usize];
                // Plain spans can't conflict with each other by
                // construction; no check needed.
                layer_of[t as usize] = layer;
                books[layer as usize].insert((sp.c, sp.r), sp.layer_idx);
                queue.push_back(t);
            }
        }
    }

    // --- Phase 2: wave adoption from all base layers at once. ---
    for s in 0..n as u32 {
        if layer_of[s as usize] != UNASSIGNED {
            wave.push_back(s);
        }
    }
    grow_wave(&spans, &mut layer_of, &mut books, &mut wave, max_step_layers);

    // --- Phase 3: leftovers seed new layers, then merge. ---
    for seed in 0..n as u32 {
        if layer_of[seed as usize] != UNASSIGNED {
            continue;
        }
        let layer = books.len() as u32;
        books.push(std::collections::HashMap::new());
        layer_of[seed as usize] = layer;
        books[layer as usize].insert(
            (spans[seed as usize].c, spans[seed as usize].r),
            spans[seed as usize].layer_idx,
        );
        wave.clear();
        wave.push_back(seed);
        grow_wave(&spans, &mut layer_of, &mut books, &mut wave, max_step_layers);
    }
    merge_layers(&spans, &mut layer_of, &mut books, max_step_layers);

    // Prune dust layers and renumber densely.
    let mut sizes = vec![0usize; books.len()];
    for &l in &layer_of {
        if l != UNASSIGNED {
            sizes[l as usize] += 1;
        }
    }
    let mut remap = vec![UNASSIGNED; books.len()];
    let mut next = 0u32;
    for (l, &size) in sizes.iter().enumerate() {
        if size >= min_layer_spans.max(1) {
            remap[l] = next;
            next += 1;
        }
    }
    let mut pruned = 0usize;
    for l in layer_of.iter_mut() {
        if *l != UNASSIGNED {
            *l = remap[*l as usize];
            if *l == UNASSIGNED {
                pruned += 1;
            }
        }
    }

    Assignment {
        spans,
        layer_of,
        layer_count: next,
        pruned_spans: pruned,
        cols: chf.cols,
        rows: chf.rows,
        cell_offsets,
    }
}

/// Advance a BFS adoption wave: each popped span offers its layer to
/// its unassigned walk-neighbors; a neighbor joins iff it doesn't
/// conflict. FIFO order makes the wave advance by distance, so every
/// contested span is claimed by the *nearest* accepting layer.
fn grow_wave(
    spans: &[Span],
    layer_of: &mut [u32],
    books: &mut [std::collections::HashMap<(u32, u32), u32>],
    wave: &mut VecDeque<u32>,
    max_step_layers: u32,
) {
    while let Some(s) = wave.pop_front() {
        let layer = layer_of[s as usize];
        for k in 0..4 {
            let Some(t) = spans[s as usize].links[k] else {
                continue;
            };
            if layer_of[t as usize] != UNASSIGNED {
                continue;
            }
            let sp = &spans[t as usize];
            if conflicts(&books[layer as usize], sp, max_step_layers) {
                continue; // a cut — the seam lands here
            }
            layer_of[t as usize] = layer;
            books[layer as usize].insert((sp.c, sp.r), sp.layer_idx);
            wave.push_back(t);
        }
    }
}

/// Consolidate fragments: absorb layer B into layer A when no span of B
/// conflicts with A's book. Repeats to a fixed point in ascending id
/// order (deterministic). Typical win: the two flank wedges of a ramp
/// and the deck they lead to become one layer instead of three.
fn merge_layers(
    spans: &[Span],
    layer_of: &mut [u32],
    books: &mut Vec<std::collections::HashMap<(u32, u32), u32>>,
    max_step_layers: u32,
) {
    loop {
        let mut merged_any = false;
        for a in 0..books.len() {
            if books[a].is_empty() {
                continue;
            }
            for b in (a + 1)..books.len() {
                if books[b].is_empty() {
                    continue;
                }
                // Only merge walk-adjacent layers — merging disjoint
                // fragments is harmless for correctness but produces
                // confusing multi-region layers.
                let adjacent = spans.iter().enumerate().any(|(s, span)| {
                    layer_of[s] == a as u32
                        && span.links.iter().flatten().any(|&t| layer_of[t as usize] == b as u32)
                });
                if !adjacent {
                    continue;
                }
                let compatible = spans.iter().enumerate().all(|(s, span)| {
                    layer_of[s] != b as u32 || !conflicts(&books[a], span, max_step_layers)
                });
                if !compatible {
                    continue;
                }
                for (s, l) in layer_of.iter_mut().enumerate() {
                    if *l == b as u32 {
                        *l = a as u32;
                        books[a].insert((spans[s].c, spans[s].r), spans[s].layer_idx);
                    }
                }
                books[b].clear();
                merged_any = true;
            }
        }
        if !merged_any {
            break;
        }
    }
}

/// Does adding `sp` to the layer with plan-view `book` violate the layer
/// invariants?
fn conflicts(
    book: &std::collections::HashMap<(u32, u32), u32>,
    sp: &Span,
    max_step_layers: u32,
) -> bool {
    // Column conflict: the layer already holds a span in this cell.
    if book.contains_key(&(sp.c, sp.r)) {
        return true;
    }
    // Cliff conflict: an 8-adjacent cell of the layer whose height is
    // out of step reach — the shared chart vertex would need two
    // heights.
    for dr in -1i32..=1 {
        for dc in -1i32..=1 {
            if dc == 0 && dr == 0 {
                continue;
            }
            let nc = sp.c as i32 + dc;
            let nr = sp.r as i32 + dr;
            if nc < 0 || nr < 0 {
                continue;
            }
            if let Some(&l) = book.get(&(nc as u32, nr as u32)) {
                if l.abs_diff(sp.layer_idx) > max_step_layers {
                    return true;
                }
            }
        }
    }
    false
}

impl Assignment {
    /// Iterate the cut links: walk-connected span pairs assigned to two
    /// different (unpruned) layers. Each undirected pair yields once.
    pub fn seam_links(&self) -> impl Iterator<Item = (u32, u32)> + '_ {
        self.spans.iter().enumerate().flat_map(move |(s, span)| {
            span.links.iter().filter_map(move |&t| {
                let t = t?;
                let (a, b) = (self.layer_of[s], self.layer_of[t as usize]);
                (a != UNASSIGNED && b != UNASSIGNED && a != b && (s as u32) < t)
                    .then_some((s as u32, t))
            })
        })
    }
}
