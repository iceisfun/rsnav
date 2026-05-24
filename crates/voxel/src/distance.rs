//! Distance field over a [`CompactHeightfield`]: per walkable cell, the
//! chamfer distance (4-neighbor, unit weights) to the nearest boundary.
//!
//! "Boundary" here means a walkable cell that's missing at least one
//! cardinal neighbor link — the edge of a walkable surface, against a
//! wall, or at a step too tall to traverse. Those cells start with
//! distance 0. Every other walkable cell starts at `u16::MAX` and gets
//! relaxed to `min(neighbor_distances) + 1` via two passes (forward
//! row-major + backward).
//!
//! The result is the input to watershed segmentation: high values mark
//! "deep interior" cells (the natural region centers); contours along
//! constant-distance lines define basins.
//!
//! 4-neighbor unit weights are a simplification — Recast uses 8-neighbor
//! chamfer weights (typically 5 cardinal, 7 diagonal) for a better
//! Euclidean approximation. Our compact heightfield only carries cardinal
//! links right now; we can upgrade later without changing the API.

use crate::walkability::CompactHeightfield;
use rsnav_common::Vec3;

#[derive(Clone, Debug)]
pub struct DistanceField {
    pub origin: Vec3,
    pub cell_size: f64,
    pub cols: u32,
    pub rows: u32,
    /// Same shape as [`CompactHeightfield`]'s columns: outer Vec indexed
    /// `row * cols + col`, inner Vec indexed by surface_index within the
    /// column. Each entry is the chamfer distance for that walkable cell.
    columns: Vec<Vec<u16>>,
}

impl DistanceField {
    pub fn at(&self, c: u32, r: u32, surface_index: u32) -> Option<u16> {
        if c >= self.cols || r >= self.rows {
            return None;
        }
        let idx = (r * self.cols + c) as usize;
        self.columns[idx].get(surface_index as usize).copied()
    }

    pub fn max_distance(&self) -> u16 {
        self.columns
            .iter()
            .flatten()
            .copied()
            .filter(|d| *d != u16::MAX)
            .max()
            .unwrap_or(0)
    }

    /// Iterate every walkable cell's distance as `(col, row, surface_index, dist)`.
    pub fn iter(&self) -> impl Iterator<Item = (u32, u32, u32, u16)> + '_ {
        let cols = self.cols;
        self.columns
            .iter()
            .enumerate()
            .flat_map(move |(i, dists)| {
                let c = (i as u32) % cols;
                let r = (i as u32) / cols;
                dists
                    .iter()
                    .enumerate()
                    .map(move |(si, &d)| (c, r, si as u32, d))
            })
    }
}

pub fn build_distance_field(chf: &CompactHeightfield) -> DistanceField {
    // Initial pass: u16::MAX for interior cells, 0 for boundary cells
    // (cells with at least one missing cardinal neighbor).
    let mut columns: Vec<Vec<u16>> = Vec::with_capacity((chf.cols * chf.rows) as usize);
    for r in 0..chf.rows {
        for c in 0..chf.cols {
            let surfaces = chf.surfaces_at(c, r);
            let mut dists = Vec::with_capacity(surfaces.len());
            for cell in surfaces {
                let on_boundary = cell.neighbors.iter().any(|n| n.is_none());
                dists.push(if on_boundary { 0 } else { u16::MAX });
            }
            columns.push(dists);
        }
    }

    // Forward pass: row-major iteration, relax against -X and -Y neighbors
    // (those already finalized this pass).
    for r in 0..chf.rows {
        for c in 0..chf.cols {
            relax_pair(chf, &mut columns, c, r, &[2, 3]); // -X, -Y
        }
    }

    // Backward pass: reverse iteration, relax against +X and +Y.
    for r in (0..chf.rows).rev() {
        for c in (0..chf.cols).rev() {
            relax_pair(chf, &mut columns, c, r, &[0, 1]); // +X, +Y
        }
    }

    DistanceField {
        origin: chf.origin,
        cell_size: chf.cell_size,
        cols: chf.cols,
        rows: chf.rows,
        columns,
    }
}

/// One relaxation step: for each walkable surface in column `(c, r)`,
/// update its distance to `min(my_dist, neighbor_dist + 1)` over the
/// neighbor directions listed in `dirs`.
fn relax_pair(
    chf: &CompactHeightfield,
    columns: &mut [Vec<u16>],
    c: u32,
    r: u32,
    dirs: &[usize],
) {
    let idx = (r * chf.cols + c) as usize;
    let surfaces_len = chf.surfaces_at(c, r).len();
    for si in 0..surfaces_len {
        let mut best = columns[idx][si];
        for &k in dirs {
            if let Some((nc, nr, n_si)) = chf.neighbor_cell(c, r, si as u32, k) {
                let n_idx = (nr * chf.cols + nc) as usize;
                let n_dist = columns[n_idx][n_si as usize].saturating_add(1);
                if n_dist < best {
                    best = n_dist;
                }
            }
        }
        columns[idx][si] = best;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::WalkabilityConfig;
    use crate::walkability::classify_walkability;
    use crate::{synth, VoxelGrid};

    #[test]
    fn flat_plane_has_distance_increasing_toward_center() {
        let soup = synth::plane(10.0, 10.0, 1);
        let config = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        let df = build_distance_field(&chf);
        // Corner cells should be at distance 0 (on boundary).
        // Center cells should have the highest distance.
        let max = df.max_distance();
        assert!(max > 0, "expected positive max distance for an open plane");
        // A cell roughly at the center should have distance >= max.
        let c_center = chf.cols / 2;
        let r_center = chf.rows / 2;
        let surfaces = chf.surfaces_at(c_center, r_center);
        assert!(!surfaces.is_empty(), "center column should be walkable");
        let center_dist = df.at(c_center, r_center, 0).unwrap();
        assert!(
            center_dist > 0,
            "center distance should be > 0, got {}",
            center_dist
        );
    }

    #[test]
    fn boundary_cells_are_zero() {
        let soup = synth::plane(4.0, 4.0, 1);
        let config = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        let df = build_distance_field(&chf);
        // Any cell with a missing neighbor should be at distance 0.
        for (c, r, si, cell) in chf.iter() {
            if cell.neighbors.iter().any(|n| n.is_none()) {
                let d = df.at(c, r, si).unwrap();
                assert_eq!(d, 0, "boundary cell ({},{},{}) should have distance 0, got {}", c, r, si, d);
            }
        }
    }

    #[test]
    fn relaxation_monotone() {
        // Every cell's distance must be ≤ (any neighbor's distance + 1).
        let soup = synth::plane(8.0, 8.0, 1);
        let config = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        let df = build_distance_field(&chf);
        for (c, r, si, cell) in chf.iter() {
            let d = df.at(c, r, si).unwrap();
            for k in 0..4 {
                if let Some((nc, nr, n_si)) = chf.neighbor_cell(c, r, si, k) {
                    let nd = df.at(nc, nr, n_si).unwrap();
                    assert!(
                        d <= nd.saturating_add(1),
                        "distance not monotone at ({},{},{}): d={}, neighbor[{}] = {}",
                        c, r, si, d, k, nd
                    );
                    let _ = cell;
                }
            }
        }
    }
}
