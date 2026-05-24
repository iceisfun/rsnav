//! Region assignment over a [`CompactHeightfield`]: connected-components
//! flood fill with two constraints.
//!
//! # Why not watershed
//!
//! Classic Recast-style watershed (seed at distance-field maxima, grow
//! until ridges meet) is great for producing compact, roughly-convex
//! regions in huge open areas — but it has a counterintuitive property:
//! a single-Z walkable surface with two "wide" parts connected by a
//! narrow neck gets split at the neck. For us, "same height, adjacent
//! in plan view" should always be the same region, so we use plain
//! connected components instead. If we ever want compactness for
//! triangulation quality, that's a post-process (Recast's
//! `merge_region_cells` style).
//!
//! # The two constraints
//!
//! 1. **Single-layer-per-column.** A region can have at most one
//!    walkable cell per `(col, row)` column. This is what splits
//!    stacked walkable surfaces (under-stairs floor + platform top
//!    above) into distinct regions. Without it the region's contour
//!    wouldn't be a valid 2D PSLG — the same XY would map to multiple
//!    Zs — and the CDT step downstream would be ill-defined.
//!
//! 2. **`max_layer_step`.** Two adjacent walkable cells only count as
//!    connected for region purposes if their layer difference is within
//!    this threshold. The walkability classifier links cells up to
//!    `max_step_height` (so the agent can physically climb), but
//!    larger Z jumps within one region would force the CDT to
//!    interpolate diagonally across a step. `max_layer_step = 1`
//!    keeps gentle ramps as one region while splitting stairs per tread.
//!
//! # The output contract
//!
//! Each region is one **single-layer-in-plan-view** connected walkable
//! surface — the "one voxel set + one heightfield" unit the downstream
//! PSLG/CDT/portal extraction operates on.
//!
//! After flood fill, regions smaller than `min_region_cells` are
//! dropped (their cells become unassigned). IDs are remapped to be
//! contiguous `0..region_count`. Merge-on-too-small (Recast's
//! `merge_region_cells`) is a follow-up if we want it.

use crate::config::WatershedConfig;
use crate::distance::{build_distance_field, DistanceField};
use crate::walkability::CompactHeightfield;
use crate::RegionId;
use rsnav_common::Vec3;
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct RegionMap {
    pub origin: Vec3,
    pub cell_size: f64,
    pub cols: u32,
    pub rows: u32,
    /// Same shape as [`CompactHeightfield`]'s columns. Each entry is the
    /// region ID for that walkable cell, or [`RegionId::INVALID`] if the
    /// cell was filtered out (region too small).
    columns: Vec<Vec<RegionId>>,
    pub region_count: u32,
}

impl RegionMap {
    pub fn at(&self, c: u32, r: u32, surface_index: u32) -> RegionId {
        if c >= self.cols || r >= self.rows {
            return RegionId::INVALID;
        }
        let idx = (r * self.cols + c) as usize;
        self.columns[idx]
            .get(surface_index as usize)
            .copied()
            .unwrap_or(RegionId::INVALID)
    }

    /// Iterate every walkable cell's region as `(col, row, surface_index, region)`.
    pub fn iter(&self) -> impl Iterator<Item = (u32, u32, u32, RegionId)> + '_ {
        let cols = self.cols;
        self.columns
            .iter()
            .enumerate()
            .flat_map(move |(i, regs)| {
                let c = (i as u32) % cols;
                let r = (i as u32) / cols;
                regs.iter()
                    .enumerate()
                    .map(move |(si, &rid)| (c, r, si as u32, rid))
            })
    }

    /// How many walkable cells belong to region `rid`. O(n) over all cells.
    pub fn cell_count(&self, rid: RegionId) -> usize {
        if rid == RegionId::INVALID {
            return 0;
        }
        self.columns
            .iter()
            .flatten()
            .filter(|&&r| r == rid)
            .count()
    }
}

/// Convenience: build the distance field (kept for visualization /
/// future use) and partition into regions in one shot.
pub fn segment(chf: &CompactHeightfield, config: &WatershedConfig) -> RegionMap {
    let df = build_distance_field(chf);
    assign_regions(chf, &df, config)
}

/// Partition the walkable cells into regions via plain connected-
/// components flood fill, subject to (a) the single-layer-per-column
/// constraint and (b) the `max_layer_step` constraint.
///
/// The distance field is unused for partition; we accept it for API
/// stability (and so a future caller can swap in a watershed-style
/// implementation without changing the signature).
pub fn assign_regions(
    chf: &CompactHeightfield,
    _df: &DistanceField,
    config: &WatershedConfig,
) -> RegionMap {
    // Allocate output, same shape as chf.
    let mut regions: Vec<Vec<RegionId>> = (0..(chf.cols * chf.rows))
        .map(|i| {
            let c = (i as u32) % chf.cols;
            let r = (i as u32) / chf.cols;
            vec![RegionId::INVALID; chf.surfaces_at(c, r).len()]
        })
        .collect();

    let max_layer_step = config.max_layer_step;
    let mut next_id: u32 = 0;
    let mut queue: VecDeque<(u32, u32, u32)> = VecDeque::new();

    for r in 0..chf.rows {
        for c in 0..chf.cols {
            let idx = (r * chf.cols + c) as usize;
            for si in 0..chf.surfaces_at(c, r).len() {
                if regions[idx][si] != RegionId::INVALID {
                    continue;
                }
                // Start a new region at this unassigned cell and flood
                // outward subject to the constraints.
                let rid = RegionId(next_id);
                next_id += 1;
                regions[idx][si] = rid;
                queue.push_back((c, r, si as u32));
                while let Some((cc, rr, ssi)) = queue.pop_front() {
                    let cur_idx = (rr * chf.cols + cc) as usize;
                    let my_rid = regions[cur_idx][ssi as usize];
                    let my_layer = chf.surfaces_at(cc, rr)[ssi as usize].layer as i64;
                    for k in 0..4 {
                        let Some((nc, nr, n_si)) = chf.neighbor_cell(cc, rr, ssi, k) else {
                            continue;
                        };
                        let n_idx = (nr * chf.cols + nc) as usize;
                        if regions[n_idx][n_si as usize] != RegionId::INVALID {
                            continue;
                        }
                        // Single-layer-per-column constraint.
                        if regions[n_idx].iter().any(|&r| r == my_rid) {
                            continue;
                        }
                        // max_layer_step constraint.
                        let n_layer = chf.surfaces_at(nc, nr)[n_si as usize].layer as i64;
                        if (my_layer - n_layer).unsigned_abs() as u32 > max_layer_step {
                            continue;
                        }
                        regions[n_idx][n_si as usize] = my_rid;
                        queue.push_back((nc, nr, n_si));
                    }
                }
            }
        }
    }

    // Step 4: filter regions smaller than `min_region_cells`. They become
    // unassigned; remaining IDs are remapped to be contiguous 0..N.
    let mut counts = vec![0usize; next_id as usize];
    for col in &regions {
        for &rid in col {
            if rid != RegionId::INVALID {
                counts[rid.index()] += 1;
            }
        }
    }
    let min_cells = config.min_region_cells as usize;
    let mut remap: Vec<RegionId> = vec![RegionId::INVALID; next_id as usize];
    let mut new_id: u32 = 0;
    for (i, &c) in counts.iter().enumerate() {
        if c >= min_cells {
            remap[i] = RegionId(new_id);
            new_id += 1;
        }
    }
    for col in &mut regions {
        for rid in col {
            if *rid != RegionId::INVALID {
                *rid = remap[rid.index()];
            }
        }
    }

    RegionMap {
        origin: chf.origin,
        cell_size: chf.cell_size,
        cols: chf.cols,
        rows: chf.rows,
        columns: regions,
        region_count: new_id,
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{WalkabilityConfig, WatershedConfig};
    use crate::walkability::classify_walkability;
    use crate::{synth, PolySoup, VoxelGrid};

    fn build_chf(soup: &PolySoup, cell_size: f64) -> (CompactHeightfield, WatershedConfig) {
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let ws = WatershedConfig::default();
        (chf, ws)
    }

    #[test]
    fn flat_plane_one_region() {
        let soup = synth::plane(8.0, 8.0, 1);
        let (chf, ws) = build_chf(&soup, 0.5);
        let rm = segment(&chf, &ws);
        assert_eq!(
            rm.region_count, 1,
            "flat plane should produce exactly one region"
        );
        for (_, _, _, rid) in rm.iter() {
            assert_eq!(rid, RegionId(0));
        }
    }

    #[test]
    fn two_disconnected_planes_two_regions() {
        // Floor at z=0 and a separate "roof" plane high above, no connection.
        let mut soup = synth::plane(6.0, 6.0, 1);
        let roof = synth::plane(6.0, 6.0, 1);
        let base = soup.vertices.len() as u32;
        soup.vertices
            .extend(roof.vertices.iter().map(|v| Vec3::new(v.x, v.y, 5.0)));
        for t in &roof.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let (chf, ws) = build_chf(&soup, 0.5);
        let rm = segment(&chf, &ws);
        assert_eq!(
            rm.region_count, 2,
            "two stacked-but-unconnected planes should produce 2 regions"
        );
    }

    #[test]
    fn under_ramp_floor_and_top_are_different_regions() {
        // Same scene as the walkability test: floor + tall enough ramp
        // that the under-ramp floor and the ramp surface both come out
        // walkable.
        let mut soup = synth::plane(10.0, 4.0, 1);
        let ramp = synth::ramp(0.0, 4.0, 3.0, 4.0);
        let base = soup.vertices.len() as u32;
        soup.vertices.extend(ramp.vertices.iter().copied());
        for t in &ramp.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let (chf, ws) = build_chf(&soup, 0.2);
        let rm = segment(&chf, &ws);
        // We expect >= 2 regions: the ground floor surface and the ramp
        // surface (a continuous walkable path going up). They may also
        // produce more if watershed splits e.g. the floor on either side
        // of the ramp's column shadow — accept ≥2 as the contract.
        assert!(
            rm.region_count >= 2,
            "expected ≥2 regions (under-ramp floor + ramp surface), got {}",
            rm.region_count
        );
    }

    #[test]
    fn tiny_region_dropped_by_filter() {
        // A 1×1 cell plane and a much larger one — the tiny one should
        // be filtered out (default min_region_cells = 8).
        let mut soup = synth::plane(6.0, 6.0, 1);
        let pillar_top = synth::plane(0.2, 0.2, 1);
        let base = soup.vertices.len() as u32;
        soup.vertices
            .extend(pillar_top.vertices.iter().map(|v| Vec3::new(v.x + 10.0, v.y, 5.0)));
        for t in &pillar_top.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let (chf, ws) = build_chf(&soup, 0.5);
        let rm = segment(&chf, &ws);
        // The 6m × 6m floor easily exceeds 8 cells; the 0.2m pillar top
        // is below 1 cell at 0.5m voxels, so it's filtered out.
        assert_eq!(rm.region_count, 1);
    }

    #[test]
    fn all_assigned_cells_have_valid_region_ids() {
        let soup = synth::floor_with_ramp_and_platform();
        let (chf, ws) = build_chf(&soup, 0.2);
        let rm = segment(&chf, &ws);
        for (c, r, si, rid) in rm.iter() {
            if rid != RegionId::INVALID {
                assert!(
                    rid.0 < rm.region_count,
                    "region id {} ≥ region_count {} at ({},{},{})",
                    rid.0, rm.region_count, c, r, si
                );
            }
        }
    }
}
