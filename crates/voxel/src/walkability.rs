//! Walkability classifier: derive a per-(col, row) "compact heightfield"
//! from a tagged [`VoxelGrid`].
//!
//! Per column, walks bottom to top and records **every** walkable cell that
//! has enough empty space above it (the agent's standing clearance). Real
//! 3D scenes have stacked surfaces — the floor underneath a ramp, the
//! ramp itself, the platform on top — and a single-surface-per-column
//! model collapses them into invisibility. Each surface gets per-direction
//! neighbor links: for each cardinal direction, the closest walkable
//! surface in the neighbor column within `max_step_height`.
//!
//! # Walkable, but unreachable
//!
//! A cell can pass walkability without being reachable from anywhere — a
//! pillar top with no adjacent step, an isolated ledge. Those become
//! single-cell "regions" later in the watershed stage and typically get
//! pruned by the `min_region_cells` filter.

use crate::config::WalkabilityConfig;
use crate::grid::area_type;
use crate::VoxelGrid;
use rsnav_common::Vec3;

/// One walkable surface within a column.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WalkableCell {
    /// Z layer index in the source [`VoxelGrid`].
    pub layer: u32,
    /// For each cardinal direction (`+X, +Y, -X, -Y`): index into the
    /// neighbor column's surfaces of the connected walkable cell, or
    /// `None` if no neighbor is within `max_step_height`.
    pub neighbors: [Option<u32>; 4],
}

/// Output of the walkability classifier: a 2D plan-view grid where each
/// `(col, row)` slot has a list of walkable surfaces (sorted by layer
/// ascending). Most columns have 0 or 1; multi-story or under-stair
/// scenes have 2+ stacked.
#[derive(Clone, Debug)]
pub struct CompactHeightfield {
    pub origin: Vec3,
    pub cell_size: f64,
    pub cols: u32,
    pub rows: u32,
    /// Layout: `columns[row * cols + col]`. Inner Vec is sorted by
    /// `layer` ascending. Empty Vec ⇒ no walkable surface in that column.
    columns: Vec<Vec<WalkableCell>>,
}

/// Cardinal neighbor offsets matching [`WalkableCell::neighbors`]:
/// `[+X, +Y, -X, -Y]`.
pub const NEIGHBOR_DELTAS: [(i32, i32); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

impl CompactHeightfield {
    /// All walkable surfaces in column `(c, r)`, sorted bottom → top.
    /// Returns an empty slice if the column has no walkable surfaces or
    /// the coords are out of range.
    pub fn surfaces_at(&self, c: u32, r: u32) -> &[WalkableCell] {
        if c >= self.cols || r >= self.rows {
            return &[];
        }
        &self.columns[(r * self.cols + c) as usize]
    }

    /// Total number of walkable cells across all columns (not the number
    /// of columns that have at least one walkable cell).
    pub fn walkable_count(&self) -> usize {
        self.columns.iter().map(|col| col.len()).sum()
    }

    /// Number of columns with at least one walkable surface.
    pub fn walkable_column_count(&self) -> usize {
        self.columns.iter().filter(|col| !col.is_empty()).count()
    }

    /// Iterate every walkable cell as `(col, row, surface_index, cell)`.
    /// `surface_index` is the position within the column's stack; pass it
    /// to [`Self::neighbor_cell`] to follow links.
    pub fn iter(&self) -> impl Iterator<Item = (u32, u32, u32, WalkableCell)> + '_ {
        let cols = self.cols;
        self.columns
            .iter()
            .enumerate()
            .flat_map(move |(i, surfaces)| {
                let c = (i as u32) % cols;
                let r = (i as u32) / cols;
                surfaces
                    .iter()
                    .enumerate()
                    .map(move |(si, &cell)| (c, r, si as u32, cell))
            })
    }

    /// World-space point representing where an agent stands on the
    /// walkable surface at `(c, r)` index `surface_index` — top of the
    /// walkable voxel, centered in XY. Returns `None` if the index is
    /// out of range.
    pub fn surface_point(&self, c: u32, r: u32, surface_index: u32) -> Option<Vec3> {
        let surfaces = self.surfaces_at(c, r);
        let cell = surfaces.get(surface_index as usize)?;
        Some(Vec3::new(
            self.origin.x + (c as f64 + 0.5) * self.cell_size,
            self.origin.y + (r as f64 + 0.5) * self.cell_size,
            self.origin.z + (cell.layer as f64 + 1.0) * self.cell_size,
        ))
    }

    /// Follow neighbor link `direction` (0..4 matching [`NEIGHBOR_DELTAS`])
    /// from cell `(c, r, surface_index)`; returns `(nc, nr, n_surface)` or
    /// `None` if no link.
    pub fn neighbor_cell(
        &self,
        c: u32,
        r: u32,
        surface_index: u32,
        direction: usize,
    ) -> Option<(u32, u32, u32)> {
        let surfaces = self.surfaces_at(c, r);
        let cell = surfaces.get(surface_index as usize)?;
        let n_idx = cell.neighbors[direction]?;
        let (dc, dr) = NEIGHBOR_DELTAS[direction];
        let nc = (c as i32) + dc;
        let nr = (r as i32) + dr;
        if nc < 0 || nc >= self.cols as i32 || nr < 0 || nr >= self.rows as i32 {
            return None;
        }
        Some((nc as u32, nr as u32, n_idx))
    }
}

/// Run the walkability pass over a tagged [`VoxelGrid`].
pub fn classify_walkability(grid: &VoxelGrid, config: &WalkabilityConfig) -> CompactHeightfield {
    let clearance = config.clearance_cells(grid.cell_size);
    let max_step = config.max_step_layers(grid.cell_size);

    // Pass 1: per column, find walkable surfaces by collapsing 2-cell
    // SAT artifacts.
    //
    // A horizontal triangle that sits exactly on a voxel face marks
    // BOTH adjacent cells (the conservative SAT can't tell which side
    // the triangle "is on"). One real surface ⇒ at most 2 cells. Any
    // contiguous group LONGER than 2 cells represents MULTIPLE distinct
    // surfaces (e.g., floor at z=0 + ramp above at z=0.4 that happen to
    // produce adjacent walkable cells) — we must split them so each
    // surface gets its own independent clearance check.
    //
    // The "surface cell" we keep per pair is the LOWER one (its top
    // edge sits at the actual surface Z). For non-artifact single cells,
    // we keep that cell as the surface.
    //
    // Clearance is checked from JUST ABOVE each strip — the strip's own
    // upper cell (the SAT artifact, if any) doesn't count as an
    // obstacle against itself.
    let mut columns: Vec<Vec<WalkableCell>> =
        vec![Vec::new(); (grid.cols * grid.rows) as usize];
    for r in 0..grid.rows {
        for c in 0..grid.cols {
            let idx = (r * grid.cols + c) as usize;
            let mut l = 0u32;
            while l < grid.layers {
                if grid.area_at(c, r, l) != area_type::WALKABLE {
                    l += 1;
                    continue;
                }
                let strip_start = l;
                // Consume at most 2 consecutive walkable cells (one SAT artifact's worth).
                let mut count = 0u32;
                while l < grid.layers
                    && grid.area_at(c, r, l) == area_type::WALKABLE
                    && count < 2
                {
                    l += 1;
                    count += 1;
                }
                let strip_end = l; // exclusive
                if has_clearance_from(grid, c, r, strip_end, clearance) {
                    columns[idx].push(WalkableCell {
                        layer: strip_start,
                        neighbors: [None; 4],
                    });
                }
                // l points to the next cell to consider. If a longer group
                // exists in this column, the outer while loop will find the
                // next walkable run starting there and process it as another
                // strip (its own surface, its own clearance check).
            }
        }
    }

    // Pass 2: link each cell's 4 cardinal neighbors. For each direction,
    // pick the neighbor column's surface whose layer is closest to ours
    // and within max_step.
    let cols_i = grid.cols as i32;
    let rows_i = grid.rows as i32;
    for r in 0..grid.rows {
        for c in 0..grid.cols {
            let cell_idx = (r * grid.cols + c) as usize;
            let count = columns[cell_idx].len();
            for si in 0..count {
                let my_layer = columns[cell_idx][si].layer as i64;
                let mut neighbors = [None; 4];
                for (k, &(dc, dr)) in NEIGHBOR_DELTAS.iter().enumerate() {
                    let nc = c as i32 + dc;
                    let nr = r as i32 + dr;
                    if nc < 0 || nc >= cols_i || nr < 0 || nr >= rows_i {
                        continue;
                    }
                    let n_idx = (nr as u32 * grid.cols + nc as u32) as usize;
                    // Find the neighbor surface with the smallest |Δlayer| within max_step.
                    let mut best: Option<(u32, i64)> = None;
                    for (ni, n_cell) in columns[n_idx].iter().enumerate() {
                        let step = (my_layer - n_cell.layer as i64).unsigned_abs() as u32;
                        if step > max_step {
                            continue;
                        }
                        let diff = (my_layer - n_cell.layer as i64).abs();
                        if best.map_or(true, |(_, b_diff)| diff < b_diff) {
                            best = Some((ni as u32, diff));
                        }
                    }
                    neighbors[k] = best.map(|(idx, _)| idx);
                }
                columns[cell_idx][si].neighbors = neighbors;
            }
        }
    }

    CompactHeightfield {
        origin: grid.origin,
        cell_size: grid.cell_size,
        cols: grid.cols,
        rows: grid.rows,
        columns,
    }
}

/// Are the `clearance` cells starting at `first_above` (inclusive) all empty?
/// `first_above` is the first cell ABOVE the walkable strip; the strip's
/// own upper cells are not counted as obstacles against themselves.
fn has_clearance_from(grid: &VoxelGrid, c: u32, r: u32, first_above: u32, clearance: u32) -> bool {
    for offset in 0..clearance {
        let above = first_above + offset;
        if above >= grid.layers {
            // Off-grid above is implicit empty (open sky).
            return true;
        }
        if grid.is_occupied(c, r, above) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::WalkabilityConfig;
    use crate::synth;
    use crate::VoxelGrid;

    fn default_walkability() -> WalkabilityConfig {
        WalkabilityConfig::default()
    }

    #[test]
    fn flat_plane_one_surface_per_column() {
        let soup = synth::plane(10.0, 10.0, 1);
        let config = default_walkability();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        assert!(chf.walkable_column_count() >= 400);
        // No column should have more than one walkable surface in a flat scene.
        for (_, _, si, _) in chf.iter() {
            assert_eq!(si, 0, "flat plane shouldn't produce stacked surfaces");
        }
    }

    #[test]
    fn vertical_wall_produces_no_walkable() {
        let soup = crate::PolySoup {
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            triangles: vec![[0, 1, 2]],
        };
        let config = default_walkability();
        let grid = VoxelGrid::from_polysoup(&soup, 0.25, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        assert_eq!(chf.walkable_count(), 0);
    }

    #[test]
    fn under_ramp_floor_and_ramp_surface_both_walkable() {
        // Build a scene where the ramp's high end gives clear clearance to
        // the floor underneath: a 4 m run rising 3 m (slope ≈ 36.9°, well
        // under the 45° default) so the ramp at x=3 sits at z=2.25, above
        // the agent's 1.8 m clearance ceiling.
        let mut soup = synth::plane(10.0, 4.0, 1);
        let ramp = synth::ramp(0.0, 4.0, 3.0, 4.0);
        let base = soup.vertices.len() as u32;
        soup.vertices.extend(ramp.vertices.iter().copied());
        for t in &ramp.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let config = default_walkability(); // 1.8 m clearance, 45° slope
        let grid = VoxelGrid::from_polysoup(&soup, 0.2, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);

        // Query column at x=3, y=0 — ramp z ≈ 2.25, agent head at 2.0.
        let cs = grid.cell_size;
        let col = (((3.0 - grid.origin.x) / cs).floor() as u32).min(grid.cols - 1);
        let row = (((0.0 - grid.origin.y) / cs).floor() as u32).min(grid.rows - 1);

        // Dump what voxelization actually put in this column, for diagnosis.
        let mut occupied = Vec::new();
        for l in 0..grid.layers {
            let a = grid.area_at(col, row, l);
            if a != area_type::EMPTY {
                occupied.push((l, a));
            }
        }
        let surfaces = chf.surfaces_at(col, row);
        assert!(
            surfaces.len() >= 2,
            "expected ≥2 stacked walkable surfaces under the ramp at col={}, row={} (world {}, {}); \
             occupied layers in column: {:?}; walkable surfaces found: {:?}",
            col,
            row,
            grid.origin.x + (col as f64 + 0.5) * cs,
            grid.origin.y + (row as f64 + 0.5) * cs,
            occupied,
            surfaces,
        );
    }

    #[test]
    fn ceiling_blocks_clearance() {
        let mut soup = synth::plane(4.0, 4.0, 1);
        let ceiling = synth::plane(4.0, 4.0, 1);
        let ceiling_shifted: Vec<Vec3> = ceiling
            .vertices
            .iter()
            .map(|v| Vec3::new(v.x, v.y, 1.0))
            .collect();
        let base = soup.vertices.len() as u32;
        soup.vertices.extend(ceiling_shifted);
        for t in &ceiling.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let config = default_walkability();
        let grid = VoxelGrid::from_polysoup(&soup, 0.25, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        // Floor cells should all fail clearance (1m ceiling < 1.8m required).
        // Ceiling cells with open sky above pass.
        for (_, _, _, cell) in chf.iter() {
            let z = grid.cell_bounds(0, 0, cell.layer).max.z;
            assert!(z > 1.0 - 1e-6, "found walkable cell below the ceiling at z={}", z);
        }
    }

    #[test]
    fn neighbor_links_follow_back() {
        let soup = synth::plane(4.0, 4.0, 1);
        let config = default_walkability();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, config.cos_max_slope());
        let chf = classify_walkability(&grid, &config);
        // Pick any cell that has a +X neighbor and verify symmetry.
        for (c, r, si, cell) in chf.iter() {
            if let Some(n_idx) = cell.neighbors[0] {
                // +X direction
                let other = &chf.surfaces_at(c + 1, r)[n_idx as usize];
                // The neighbor's -X link should point back at us.
                let back = other.neighbors[2]; // -X
                assert_eq!(
                    back,
                    Some(si),
                    "neighbor link asymmetry at ({},{}) si={}",
                    c, r, si
                );
            }
        }
    }
}
