//! Phase-1 output contract.
//!
//! This module defines the **frozen** types that downstream consumers (the
//! navmesh-building stage, the debug viewer, serializers) compose against.
//! Once a real consumer has been written, breaking changes to these types
//! become expensive — treat additions as cheap and renames/removals as
//! migrations.

use rsnav_common::{Aabb, Aabb3, PolygonWithHoles, Vec3};

/// Stable identifier for a region within one [`PipelineOutput`].
///
/// IDs are assigned by the watershed in deterministic order (small ID = early
/// in a deterministic flood). They are **not** stable across small input
/// perturbations; persisting them across rebuilds is undefined behavior at
/// the contract level.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RegionId(pub u32);

impl RegionId {
    pub const INVALID: Self = Self(u32::MAX);

    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Region-local heightfield: a 2D grid where each cell stores a normalized
/// height (`u16` quantization across `[min_z, max_z]`) for the top walkable
/// surface within that cell.
///
/// `u16` was chosen over `u8` so that the per-cell vertical resolution is
/// always `(max_z - min_z) / 65535` — for a 10 m tall region that's ~0.15 mm,
/// well below any agent-relevant step. `u8` would give ~4 cm steps, which is
/// large enough to misclassify stair treads.
///
/// The heightfield is **region-local**: `origin_xy` is the plan-view world XY
/// of cell `(0, 0)`'s lower-left corner, and the cell grid covers
/// `[origin_xy, origin_xy + (cols, rows) * cell_size]`. `min_z` / `max_z`
/// span the vertical range of the *walkable surface* (not the full obstacle
/// envelope).
#[derive(Clone, Debug)]
pub struct Heightfield {
    /// Side length of a cell in world units (matches the pipeline's voxel size).
    pub cell_size: f64,
    /// World-space XY of cell (0,0)'s lower-left corner.
    pub origin_xy: rsnav_common::Vertex,
    /// Grid extent in cells.
    pub cols: u32,
    pub rows: u32,
    /// Length is `cols * rows`. Row-major, row 0 at `origin_xy.y`. A `None`
    /// entry means the cell is outside the region (no walkable surface).
    pub data: Vec<Option<u16>>,
    /// Z value corresponding to a stored `0`.
    pub min_z: f64,
    /// Z value corresponding to a stored `u16::MAX`.
    pub max_z: f64,
}

impl Heightfield {
    /// World-space Z at cell `(col, row)`, or `None` if the cell is outside the region.
    pub fn sample_cell(&self, col: u32, row: u32) -> Option<f64> {
        if col >= self.cols || row >= self.rows {
            return None;
        }
        let idx = (row * self.cols + col) as usize;
        let q = self.data.get(idx).copied().flatten()?;
        let t = q as f64 / u16::MAX as f64;
        Some(self.min_z + (self.max_z - self.min_z) * t)
    }

    /// World-space Z at world-space `(x, y)`, sampled from the containing cell
    /// (no interpolation — Phase 1 keeps this a nearest-cell lookup).
    pub fn sample_world(&self, x: f64, y: f64) -> Option<f64> {
        let cx = ((x - self.origin_xy.x) / self.cell_size).floor();
        let cy = ((y - self.origin_xy.y) / self.cell_size).floor();
        if cx < 0.0 || cy < 0.0 {
            return None;
        }
        self.sample_cell(cx as u32, cy as u32)
    }
}

/// One contiguous traversable region produced by the watershed.
///
/// The contour is **raw** voxel-derived: outer ring is CCW with one vertex
/// per voxel-edge inflection; holes are CW. Simplification (collinear-vertex
/// merging, polygon-edge approximation) is a separate, later stage —
/// downstream consumers (the navmesh builder) decide their own tolerance.
#[derive(Clone, Debug)]
pub struct Region {
    pub id: RegionId,
    /// Plan-view bounds (XY).
    pub bounds_plan: Aabb,
    /// Full 3D bounds (includes the heightfield's vertical extent).
    pub bounds_world: Aabb3,
    /// Plan-view contour: outer ring CCW, holes CW.
    pub contour: PolygonWithHoles,
    /// Per-cell heights for the walkable surface within this region.
    pub heightfield: Heightfield,
}

impl Region {
    /// Average walkable height (area-weighted) — useful for sorting / culling.
    pub fn mean_height(&self) -> f64 {
        let mut sum = 0.0;
        let mut count = 0u64;
        for &cell in &self.heightfield.data {
            if let Some(q) = cell {
                let t = q as f64 / u16::MAX as f64;
                sum += self.heightfield.min_z
                    + (self.heightfield.max_z - self.heightfield.min_z) * t;
                count += 1;
            }
        }
        if count == 0 {
            f64::NAN
        } else {
            sum / count as f64
        }
    }
}

/// One contiguous shared boundary between two regions.
///
/// `edge` is a polyline in world space tracing the seam between region `a`
/// and region `b`. A pair of regions may have **multiple** `Portal` entries
/// if the regions touch in multiple disjoint places (e.g. two stairs between
/// the same two floors).
///
/// `max_height_step` is the largest vertical gap encountered along the seam
/// (`0` for a flat seam where two regions meet at the same Z; positive when
/// the regions are at different heights and the seam represents a step or
/// short jump). The watershed itself only emits portals where the step is
/// within the walkability classifier's `max_step_height`; values beyond that
/// would have been filtered as unwalkable.
#[derive(Clone, Debug)]
pub struct Portal {
    pub a: RegionId,
    pub b: RegionId,
    /// Polyline in world space; always ≥ 2 vertices.
    pub edge: Vec<Vec3>,
    pub max_height_step: f64,
}

impl Portal {
    /// Total length of the seam in world XY (ignores Z).
    pub fn length_xy(&self) -> f64 {
        self.edge
            .windows(2)
            .map(|w| {
                let dx = w[1].x - w[0].x;
                let dy = w[1].y - w[0].y;
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }

    /// Midpoint along the polyline in world XY (Z taken from the actual
    /// sampled vertex nearest the midpoint).
    pub fn midpoint(&self) -> Vec3 {
        if self.edge.len() < 2 {
            return self.edge.first().copied().unwrap_or(Vec3::ZERO);
        }
        let total = self.length_xy();
        if total == 0.0 {
            return self.edge[0];
        }
        let half = total * 0.5;
        let mut walked = 0.0;
        for w in self.edge.windows(2) {
            let (a, b) = (w[0], w[1]);
            let seg = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
            if walked + seg >= half {
                let t = (half - walked) / seg;
                return a.lerp(b, t);
            }
            walked += seg;
        }
        *self.edge.last().unwrap()
    }
}

/// Top-level output of one pipeline run.
///
/// Held by reference by downstream consumers; cheap to clone the metadata,
/// expensive to clone the heightfields (which are the bulk of the memory).
#[derive(Clone, Debug)]
pub struct PipelineOutput {
    /// World-space bounds of the input geometry that was voxelized.
    pub source_bounds: Aabb3,
    /// Voxel side length used by the pipeline (regions inherit this).
    pub voxel_size: f64,
    /// All extracted regions, indexed by `RegionId.index()`.
    pub regions: Vec<Region>,
    /// All inter-region portals. Unordered with respect to (a, b) — a
    /// portal between regions X and Y may appear as (X, Y) or (Y, X);
    /// the watershed picks one canonically and sticks with it.
    pub portals: Vec<Portal>,
}

impl PipelineOutput {
    pub fn region(&self, id: RegionId) -> Option<&Region> {
        self.regions.get(id.index())
    }

    /// All portals touching `id`. O(N) over portals; fine for the
    /// per-region counts we expect (Phase 1 doesn't precompute adjacency).
    pub fn portals_for(&self, id: RegionId) -> impl Iterator<Item = &Portal> {
        self.portals
            .iter()
            .filter(move |p| p.a == id || p.b == id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_common::Vertex;

    fn empty_hf() -> Heightfield {
        Heightfield {
            cell_size: 0.5,
            origin_xy: Vertex::new(0.0, 0.0),
            cols: 2,
            rows: 2,
            data: vec![Some(0), Some(u16::MAX), None, Some(u16::MAX / 2)],
            min_z: 0.0,
            max_z: 10.0,
        }
    }

    #[test]
    fn heightfield_samples_cells() {
        let hf = empty_hf();
        assert_eq!(hf.sample_cell(0, 0), Some(0.0));
        assert_eq!(hf.sample_cell(1, 0), Some(10.0));
        assert_eq!(hf.sample_cell(0, 1), None);
        let mid = hf.sample_cell(1, 1).unwrap();
        assert!((mid - 5.0).abs() < 1e-3, "expected ~5.0, got {}", mid);
        assert_eq!(hf.sample_cell(2, 0), None); // out of range
    }

    #[test]
    fn heightfield_world_lookup() {
        let hf = empty_hf();
        // Cell (0,0) covers [0, 0.5) × [0, 0.5)
        assert_eq!(hf.sample_world(0.1, 0.1), Some(0.0));
        // Cell (1,0) covers [0.5, 1.0) × [0, 0.5)
        assert_eq!(hf.sample_world(0.7, 0.2), Some(10.0));
        // Outside the grid
        assert_eq!(hf.sample_world(-0.1, 0.0), None);
        assert_eq!(hf.sample_world(5.0, 5.0), None);
    }

    #[test]
    fn portal_length_and_midpoint() {
        let p = Portal {
            a: RegionId(0),
            b: RegionId(1),
            edge: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(3.0, 0.0, 0.0),
                Vec3::new(3.0, 4.0, 0.0),
            ],
            max_height_step: 0.0,
        };
        assert_eq!(p.length_xy(), 7.0);
        let m = p.midpoint();
        assert!((m.x - 3.0).abs() < 1e-9);
        assert!((m.y - 0.5).abs() < 1e-9);
    }

    #[test]
    fn region_mean_height() {
        let r = Region {
            id: RegionId(0),
            bounds_plan: Aabb::from_points([Vertex::new(0.0, 0.0), Vertex::new(1.0, 1.0)]),
            bounds_world: Aabb3::from_points([Vec3::ZERO, Vec3::new(1.0, 1.0, 10.0)]),
            contour: PolygonWithHoles {
                outer: rsnav_common::Polygon {
                    vertices: vec![
                        Vertex::new(0.0, 0.0),
                        Vertex::new(1.0, 0.0),
                        Vertex::new(1.0, 1.0),
                        Vertex::new(0.0, 1.0),
                    ],
                },
                holes: vec![],
            },
            heightfield: empty_hf(),
        };
        // Three filled cells at heights 0.0, 10.0, ~5.0 → mean ~5.0
        let mean = r.mean_height();
        assert!((mean - 5.0).abs() < 1e-3);
    }
}
