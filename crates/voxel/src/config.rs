//! Pipeline configuration: voxel resolution + walkability rules + watershed knobs.
//!
//! Defaults aim at "reasonable for human-scale indoor + outdoor scenes" — a
//! 0.2 m voxel, max 45° slope, 0.4 m step height (think stair riser), 1.8 m
//! standing clearance. Real applications will tune these; the SOW
//! deliberately allows that.

use rsnav_common::Aabb3;

#[derive(Clone, Debug)]
pub struct PipelineConfig {
    /// Side length of a voxel in world units. Smaller = more accurate but
    /// quadratically (per layer) more expensive.
    pub voxel_size: f64,
    /// Explicit bounds for the voxel grid. `None` ⇒ derived from input geometry.
    pub bounds: Option<Aabb3>,
    pub walkability: WalkabilityConfig,
    pub watershed: WatershedConfig,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            voxel_size: 0.2,
            bounds: None,
            walkability: WalkabilityConfig::default(),
            watershed: WatershedConfig::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct WalkabilityConfig {
    /// Maximum slope (radians from horizontal) a surface can have and still
    /// be walkable. ~45° default.
    pub max_slope_rad: f64,
    /// Maximum vertical step between adjacent walkable cells. Larger steps
    /// become unwalkable boundaries (will need an explicit jump link later).
    pub max_step_height: f64,
    /// Minimum vertical clearance above a walkable cell. Cells with less
    /// clearance (low ceilings, undersides of stairs) are rejected.
    pub min_clearance: f64,
}

impl Default for WalkabilityConfig {
    fn default() -> Self {
        Self {
            max_slope_rad: 45.0_f64.to_radians(),
            max_step_height: 0.4,
            min_clearance: 1.8,
        }
    }
}

impl WalkabilityConfig {
    /// Threshold the voxelizer compares against: a triangle is walkable iff
    /// the absolute Z-component of its unit normal is ≥ this value.
    pub fn cos_max_slope(&self) -> f64 {
        self.max_slope_rad.cos()
    }

    /// Minimum number of empty cells stacked above a walkable cell required
    /// for that cell to count as standing room.
    pub fn clearance_cells(&self, cell_size: f64) -> u32 {
        assert!(cell_size > 0.0);
        (self.min_clearance / cell_size).ceil() as u32
    }

    /// Maximum allowable Z-layer difference between two neighboring walkable
    /// cells for them to be considered connected (you can step from one to
    /// the other).
    pub fn max_step_layers(&self, cell_size: f64) -> u32 {
        assert!(cell_size > 0.0);
        (self.max_step_height / cell_size).floor() as u32
    }
}

#[derive(Clone, Debug)]
pub struct WatershedConfig {
    /// Minimum number of cells in a region. Smaller regions are merged
    /// into their largest neighbor (or dropped if isolated).
    pub min_region_cells: u32,
    /// Regions with fewer cells than this are eligible to be merged into
    /// an adjacent larger region during the post-watershed merge pass.
    /// Must be ≥ `min_region_cells`.
    pub merge_region_cells: u32,
    /// Maximum **layer** difference between any two walkable cells
    /// that may belong to the same region. The walkability classifier
    /// itself links cells up to `max_step_height` (so the agent can
    /// physically step that high) — this stricter rule controls when
    /// two cells count as the *same connected walkable surface* for
    /// triangulation purposes.
    ///
    /// Default `1`: a single layer of Z-quantization slop. Smooth ramps
    /// (1 layer per cell) stay one region; stair treads (2+ layer
    /// risers) split into per-tread regions so the CDT triangulates
    /// each at its real Z instead of interpolating diagonally across
    /// the steps.
    pub max_layer_step: u32,
}

impl Default for WatershedConfig {
    fn default() -> Self {
        Self {
            min_region_cells: 8,
            merge_region_cells: 20,
            max_layer_step: 1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_self_consistent() {
        let c = PipelineConfig::default();
        assert!(c.voxel_size > 0.0);
        assert!(c.walkability.max_slope_rad > 0.0);
        assert!(c.walkability.max_step_height > 0.0);
        assert!(c.walkability.min_clearance > 0.0);
        assert!(c.watershed.merge_region_cells >= c.watershed.min_region_cells);
    }
}
