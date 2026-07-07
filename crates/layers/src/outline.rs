//! Per-layer boundary tracing with edge tags, and the shared height rule.
//!
//! Each layer's cell set is outlined by unit edges on grid lines, every
//! edge tagged **wall** (nothing walkably continuous beyond it) or
//! **seam** (the surface continues on another layer). Loops are traced
//! with the walkable interior on the left (outer rings CCW, holes CW)
//! and simplified by merging collinear runs of the *same tag* — a
//! deterministic rule over a symmetric edge set, so the two layers
//! flanking a seam always produce the identical vertex chain. That
//! chain conformance is what lets `rsnav_world::World::build` match
//! seam sub-edges bit-exactly instead of by tolerance.
//!
//! Heights follow the same "identical on both sides" discipline: a
//! vertex's height is the average of the **step-chained height
//! cluster** around it — every span in the four adjacent cells whose
//! height chains (gaps ≤ step) to the layer's own spans there. At a
//! seam both layers' spans chain together, so both compute the same
//! cluster and the same bit-exact average; a floor stacked above or
//! below is beyond step reach and never pollutes the average.

use std::collections::HashMap;

use crate::assign::{Assignment, UNASSIGNED};

/// What lies beyond a boundary unit edge.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum EdgeTag {
    Wall,
    /// The surface continues onto this layer.
    Seam { other: u32 },
}

/// One simplified boundary loop: `points[k]` connects to
/// `points[(k+1) % len]` with tag `tags[k]`. Grid-vertex coordinates.
#[derive(Clone, Debug)]
pub struct Loop {
    pub points: Vec<(i32, i32)>,
    pub tags: Vec<EdgeTag>,
    /// Twice the signed area in grid units; positive = outer (CCW).
    pub area2: i64,
}

impl Loop {
    pub fn is_outer(&self) -> bool {
        self.area2 > 0
    }
}

/// Trace and simplify the boundary loops of `layer`'s cell set.
///
/// `cells` maps a cell to its span id for this layer.
pub fn trace_layer_outline(
    asg: &Assignment,
    cells: &HashMap<(u32, u32), u32>,
    layer: u32,
) -> Vec<Loop> {
    // Directed boundary edges, keyed by their start vertex. Interior on
    // the left. Edge k of cell (c,r) — k indexing NEIGHBOR_DELTAS:
    //   +X → (c+1,r)   → (c+1,r+1)  moving +Y
    //   +Y → (c+1,r+1) → (c,r+1)    moving -X
    //   -X → (c,r+1)   → (c,r)      moving -Y
    //   -Y → (c,r)     → (c+1,r)    moving +X
    struct DirEdge {
        to: (i32, i32),
        dir: u8,
        tag: EdgeTag,
        consumed: bool,
    }
    let mut out_edges: HashMap<(i32, i32), Vec<DirEdge>> = HashMap::new();
    let mut edge_count = 0usize;

    for (&(c, r), &sid) in cells.iter() {
        let span = &asg.spans[sid as usize];
        let (ci, ri) = (c as i32, r as i32);
        for k in 0..4usize {
            let tag = match span.links[k] {
                Some(t) if asg.layer_of[t as usize] == layer => continue, // interior
                Some(t) if asg.layer_of[t as usize] != UNASSIGNED => EdgeTag::Seam {
                    other: asg.layer_of[t as usize],
                },
                _ => EdgeTag::Wall,
            };
            let (from, to, dir) = match k {
                0 => ((ci + 1, ri), (ci + 1, ri + 1), 1u8),
                1 => ((ci + 1, ri + 1), (ci, ri + 1), 2),
                2 => ((ci, ri + 1), (ci, ri), 3),
                _ => ((ci, ri), (ci + 1, ri), 0),
            };
            out_edges.entry(from).or_default().push(DirEdge {
                to,
                dir,
                tag,
                consumed: false,
            });
            edge_count += 1;
        }
    }

    // Deterministic iteration: sort start vertices; within a vertex,
    // edges were inserted in cell-map order — sort them by direction.
    for v in out_edges.values_mut() {
        v.sort_by_key(|e| e.dir);
    }
    let mut starts: Vec<(i32, i32)> = out_edges.keys().copied().collect();
    starts.sort_unstable();

    let mut loops = Vec::new();
    let mut consumed_total = 0usize;
    for &start in &starts {
        loop {
            // Next unconsumed edge from this vertex, if any.
            let Some(first_dir) = out_edges[&start]
                .iter()
                .find(|e| !e.consumed)
                .map(|e| e.dir)
            else {
                break;
            };
            // Walk the loop: at each vertex prefer the left turn, then
            // straight, then right — hugging the interior keeps
            // diagonally-touching regions on separate loops.
            let mut raw: Vec<((i32, i32), u8, EdgeTag)> = Vec::new();
            let mut at = start;
            let mut dir = first_dir;
            loop {
                let edges = out_edges.get_mut(&at).expect("boundary is closed");
                let mut chosen = None;
                for d in [(dir + 1) % 4, dir, (dir + 3) % 4] {
                    if let Some(idx) = edges.iter().position(|e| !e.consumed && e.dir == d) {
                        chosen = Some(idx);
                        break;
                    }
                }
                let pick = &mut edges[chosen.expect("boundary walk found no outgoing edge")];
                pick.consumed = true;
                consumed_total += 1;
                raw.push((at, pick.dir, pick.tag));
                let next = pick.to;
                dir = pick.dir;
                at = next;
                if at == start {
                    break;
                }
            }

            // Simplify: keep a vertex iff direction or tag changes.
            let n = raw.len();
            let mut points = Vec::new();
            let mut tags = Vec::new();
            for i in 0..n {
                let prev = raw[(i + n - 1) % n];
                let cur = raw[i];
                if prev.1 != cur.1 || prev.2 != cur.2 {
                    points.push(cur.0);
                    tags.push(cur.2);
                }
            }
            debug_assert!(points.len() >= 4, "a grid loop has at least 4 corners");

            let mut area2 = 0i64;
            for i in 0..points.len() {
                let a = points[i];
                let b = points[(i + 1) % points.len()];
                area2 += a.0 as i64 * b.1 as i64 - b.0 as i64 * a.1 as i64;
            }
            loops.push(Loop { points, tags, area2 });
        }
    }
    debug_assert_eq!(consumed_total, edge_count, "every boundary edge belongs to a loop");
    loops
}

/// Height of grid vertex `(i, j)` as seen by `layer`: the average of
/// the step-chained height cluster around the vertex that contains the
/// layer's own spans. Chaining compares integer voxel layers, so a step
/// landing exactly on the threshold can't flip on float noise, and both
/// layers flanking a seam always agree on the cluster — and therefore
/// on the bit-exact average. Returns `None` if the layer has no span in
/// any adjacent cell (vertex doesn't belong to this layer).
pub fn vertex_z(
    asg: &Assignment,
    cell_span_range: &dyn Fn(u32, u32) -> std::ops::Range<usize>,
    layer: u32,
    i: i32,
    j: i32,
    max_step_layers: u32,
) -> Option<f64> {
    // Spans of the four cells sharing this vertex, sorted by height.
    let mut zs: Vec<(u32, f64, u32)> = Vec::with_capacity(8);
    for (dc, dr) in [(-1i32, -1i32), (0, -1), (-1, 0), (0, 0)] {
        let (c, r) = (i + dc, j + dr);
        if c < 0 || r < 0 || c as u32 >= asg.cols || r as u32 >= asg.rows {
            continue;
        }
        for s in cell_span_range(c as u32, r as u32) {
            zs.push((asg.spans[s].layer_idx, asg.spans[s].z, asg.layer_of[s]));
        }
    }
    if zs.is_empty() {
        return None;
    }
    zs.sort_by_key(|&(li, _, _)| li);

    // Chain into clusters and find the one holding this layer's spans.
    let mut cluster_start = 0usize;
    let mut own_cluster: Option<(usize, usize)> = None;
    for k in 0..=zs.len() {
        let breaks =
            k == zs.len() || (k > 0 && zs[k].0 - zs[k - 1].0 > max_step_layers);
        if breaks {
            if zs[cluster_start..k].iter().any(|&(_, _, l)| l == layer) {
                own_cluster = Some((cluster_start, k));
                break;
            }
            cluster_start = k;
        }
    }
    let (a, b) = own_cluster?;
    let sum: f64 = zs[a..b].iter().map(|&(_, z, _)| z).sum();
    Some(sum / (b - a) as f64)
}
