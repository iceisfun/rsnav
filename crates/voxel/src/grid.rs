//! Dense 3D voxel grid with per-cell area type (1 byte per cell).
//!
//! Storage is `Vec<u8>` indexed `((layer * rows) + row) * cols + col`. Empty
//! cells are `0`; positive values encode area types (see [`area_type`]).
//! Compared to a bit-packed occupancy grid this is 8× the memory, but
//! lets the voxelizer tag cells by triangle slope so the walkability
//! classifier can find walkable surfaces without re-walking the source
//! triangles.
//!
//! Memory: `cols * rows * layers` bytes. A 64 m × 64 m × 16 m scene at
//! 0.2 m voxels is ~8 MB — still fine for any scene you'd debug
//! interactively. Sparse "span" representations (Recast-style) are
//! cheaper for huge open worlds; we can swap later without changing the
//! consumer-facing iterator surface.

use crate::PolySoup;
use rsnav_common::{Aabb3, Vec3};

/// Area-type constants stored per voxel cell.
///
/// Higher values "win" when two triangles hit the same cell — that's how a
/// floor next to a wall stays walkable at the seam. Custom area types can
/// extend this (water, mud, custom gameplay zones) by picking a value
/// outside the reserved range.
pub mod area_type {
    /// No triangle hit this cell.
    pub const EMPTY: u8 = 0;
    /// A non-walkable triangle hit (wall, ceiling, slope steeper than the
    /// walkability threshold).
    pub const SOLID: u8 = 1;
    /// A walkable triangle hit (slope ≤ walkability threshold). Wins over
    /// SOLID when merging.
    pub const WALKABLE: u8 = 2;
}

#[derive(Clone, Debug)]
pub struct VoxelGrid {
    pub origin: Vec3,
    pub cell_size: f64,
    pub cols: u32,
    pub rows: u32,
    pub layers: u32,
    area: Vec<u8>,
}

impl VoxelGrid {
    /// Allocate a grid spanning at least `bounds`, rounded up to whole cells.
    /// All cells start at [`area_type::EMPTY`].
    pub fn new(bounds: Aabb3, cell_size: f64) -> Self {
        assert!(cell_size > 0.0, "cell_size must be positive");
        assert!(
            !bounds.is_empty(),
            "cannot build a voxel grid from empty bounds"
        );
        let size = bounds.size();
        let cols = (size.x / cell_size).ceil().max(1.0) as u32;
        let rows = (size.y / cell_size).ceil().max(1.0) as u32;
        let layers = (size.z / cell_size).ceil().max(1.0) as u32;
        let total = cols as usize * rows as usize * layers as usize;
        Self {
            origin: bounds.min,
            cell_size,
            cols,
            rows,
            layers,
            area: vec![area_type::EMPTY; total],
        }
    }

    /// Convenience: size a grid to the soup's bounds (with `padding_cells`
    /// of empty space on each side) and rasterize. The `cos_max_slope`
    /// threshold is `cos(walkability_config.max_slope_rad)`; triangles
    /// flatter than this are tagged [`area_type::WALKABLE`], everything
    /// else [`area_type::SOLID`].
    pub fn from_polysoup(
        soup: &PolySoup,
        cell_size: f64,
        padding_cells: u32,
        cos_max_slope: f64,
    ) -> Self {
        let mut bounds = soup.bounds();
        if bounds.is_empty() {
            // Empty soup → degenerate-but-safe grid (won't crash downstream).
            bounds = Aabb3::from_point(Vec3::ZERO);
        }
        let pad = cell_size * padding_cells as f64;
        bounds.min = Vec3::new(bounds.min.x - pad, bounds.min.y - pad, bounds.min.z - pad);
        bounds.max = Vec3::new(bounds.max.x + pad, bounds.max.y + pad, bounds.max.z + pad);
        let mut grid = Self::new(bounds, cell_size);
        crate::voxelize::rasterize(&mut grid, soup, cos_max_slope);
        grid
    }

    pub fn cell_count(&self) -> usize {
        self.cols as usize * self.rows as usize * self.layers as usize
    }

    pub fn occupied_count(&self) -> usize {
        self.area.iter().filter(|a| **a != area_type::EMPTY).count()
    }

    pub fn walkable_count(&self) -> usize {
        self.area
            .iter()
            .filter(|a| **a == area_type::WALKABLE)
            .count()
    }

    pub fn is_empty_grid(&self) -> bool {
        self.area.iter().all(|a| *a == area_type::EMPTY)
    }

    pub fn clear(&mut self) {
        for a in &mut self.area {
            *a = area_type::EMPTY;
        }
    }

    #[inline]
    fn linear_index(&self, c: u32, r: u32, l: u32) -> usize {
        ((l as usize * self.rows as usize) + r as usize) * self.cols as usize + c as usize
    }

    #[inline]
    fn in_range(&self, c: u32, r: u32, l: u32) -> bool {
        c < self.cols && r < self.rows && l < self.layers
    }

    /// Raw area byte of cell `(c, r, l)`. Out-of-range returns
    /// [`area_type::EMPTY`].
    pub fn area_at(&self, c: u32, r: u32, l: u32) -> u8 {
        if !self.in_range(c, r, l) {
            return area_type::EMPTY;
        }
        self.area[self.linear_index(c, r, l)]
    }

    /// `true` if the cell has any non-empty area type.
    pub fn is_occupied(&self, c: u32, r: u32, l: u32) -> bool {
        self.area_at(c, r, l) != area_type::EMPTY
    }

    /// Overwrite the area of cell `(c, r, l)`. Out-of-range coordinates are
    /// silently ignored. Use [`Self::merge_area`] when accumulating multiple
    /// triangles into the same cell — direct `set_area` is for tests and
    /// hand-crafted fixtures.
    pub fn set_area(&mut self, c: u32, r: u32, l: u32, area: u8) {
        if !self.in_range(c, r, l) {
            return;
        }
        let i = self.linear_index(c, r, l);
        self.area[i] = area;
    }

    /// Merge `area` into cell `(c, r, l)` keeping the maximum of the two.
    /// This is the rule the voxelizer uses so that a walkable triangle and
    /// a non-walkable triangle hitting the same cell resolves to walkable
    /// (you can stand on a floor next to a wall).
    pub fn merge_area(&mut self, c: u32, r: u32, l: u32, area: u8) {
        if !self.in_range(c, r, l) {
            return;
        }
        let i = self.linear_index(c, r, l);
        if area > self.area[i] {
            self.area[i] = area;
        }
    }

    pub fn cell_center(&self, c: u32, r: u32, l: u32) -> Vec3 {
        Vec3::new(
            self.origin.x + (c as f64 + 0.5) * self.cell_size,
            self.origin.y + (r as f64 + 0.5) * self.cell_size,
            self.origin.z + (l as f64 + 0.5) * self.cell_size,
        )
    }

    pub fn cell_bounds(&self, c: u32, r: u32, l: u32) -> Aabb3 {
        let cs = self.cell_size;
        let min = Vec3::new(
            self.origin.x + c as f64 * cs,
            self.origin.y + r as f64 * cs,
            self.origin.z + l as f64 * cs,
        );
        let max = Vec3::new(min.x + cs, min.y + cs, min.z + cs);
        Aabb3 { min, max }
    }

    pub fn world_bounds(&self) -> Aabb3 {
        let cs = self.cell_size;
        Aabb3 {
            min: self.origin,
            max: Vec3::new(
                self.origin.x + self.cols as f64 * cs,
                self.origin.y + self.rows as f64 * cs,
                self.origin.z + self.layers as f64 * cs,
            ),
        }
    }

    /// Yields `(col, row, layer, area)` for every non-empty cell. Order is
    /// linear-index order — layer ascending, then row, then col.
    pub fn iter_with_area(&self) -> impl Iterator<Item = (u32, u32, u32, u8)> + '_ {
        let cols = self.cols as usize;
        let cells_per_layer = cols * self.rows as usize;
        self.area
            .iter()
            .enumerate()
            .filter_map(move |(linear, &a)| {
                if a == area_type::EMPTY {
                    return None;
                }
                let l = (linear / cells_per_layer) as u32;
                let r = ((linear % cells_per_layer) / cols) as u32;
                let c = (linear % cols) as u32;
                Some((c, r, l, a))
            })
    }

    /// Yields `(col, row, layer)` for every non-empty cell. Convenience
    /// wrapper over [`Self::iter_with_area`] when the caller doesn't care
    /// about the area type.
    pub fn iter_occupied(&self) -> impl Iterator<Item = (u32, u32, u32)> + '_ {
        self.iter_with_area().map(|(c, r, l, _)| (c, r, l))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_bounds() -> Aabb3 {
        Aabb3 {
            min: Vec3::new(0.0, 0.0, 0.0),
            max: Vec3::new(4.0, 4.0, 4.0),
        }
    }

    #[test]
    fn new_sizes_correctly() {
        let g = VoxelGrid::new(unit_bounds(), 1.0);
        assert_eq!(g.cols, 4);
        assert_eq!(g.rows, 4);
        assert_eq!(g.layers, 4);
        assert_eq!(g.cell_count(), 64);
        assert_eq!(g.occupied_count(), 0);
        assert!(g.is_empty_grid());
    }

    #[test]
    fn set_and_area_at_roundtrip() {
        let mut g = VoxelGrid::new(unit_bounds(), 1.0);
        g.set_area(2, 1, 3, area_type::WALKABLE);
        assert_eq!(g.area_at(2, 1, 3), area_type::WALKABLE);
        assert!(g.is_occupied(2, 1, 3));
        assert!(!g.is_occupied(0, 0, 0));
        assert_eq!(g.occupied_count(), 1);
        assert_eq!(g.walkable_count(), 1);
    }

    #[test]
    fn merge_keeps_walkable_over_solid() {
        let mut g = VoxelGrid::new(unit_bounds(), 1.0);
        g.merge_area(1, 1, 1, area_type::SOLID);
        assert_eq!(g.area_at(1, 1, 1), area_type::SOLID);
        g.merge_area(1, 1, 1, area_type::WALKABLE);
        assert_eq!(g.area_at(1, 1, 1), area_type::WALKABLE);
        // Try to "downgrade" — should stay walkable.
        g.merge_area(1, 1, 1, area_type::SOLID);
        assert_eq!(g.area_at(1, 1, 1), area_type::WALKABLE);
    }

    #[test]
    fn out_of_range_is_safe() {
        let mut g = VoxelGrid::new(unit_bounds(), 1.0);
        g.set_area(99, 0, 0, area_type::SOLID);
        g.merge_area(99, 0, 0, area_type::WALKABLE);
        assert_eq!(g.area_at(99, 0, 0), area_type::EMPTY);
        assert!(!g.is_occupied(99, 0, 0));
        assert_eq!(g.occupied_count(), 0);
    }

    #[test]
    fn iter_with_area_returns_set_cells() {
        let mut g = VoxelGrid::new(unit_bounds(), 1.0);
        let pts = [
            (0, 0, 0, area_type::WALKABLE),
            (3, 3, 3, area_type::SOLID),
            (1, 2, 1, area_type::WALKABLE),
        ];
        for &(c, r, l, a) in &pts {
            g.set_area(c, r, l, a);
        }
        let mut got: Vec<_> = g.iter_with_area().collect();
        got.sort();
        let mut want: Vec<_> = pts.to_vec();
        want.sort();
        assert_eq!(got, want);
    }

    #[test]
    fn cell_center_and_bounds_are_consistent() {
        let g = VoxelGrid::new(unit_bounds(), 1.0);
        let c = g.cell_center(2, 1, 3);
        assert_eq!(c, Vec3::new(2.5, 1.5, 3.5));
        let b = g.cell_bounds(2, 1, 3);
        assert_eq!(b.min, Vec3::new(2.0, 1.0, 3.0));
        assert_eq!(b.max, Vec3::new(3.0, 2.0, 4.0));
        assert_eq!(b.center(), c);
    }

    #[test]
    fn clear_resets_occupancy() {
        let mut g = VoxelGrid::new(unit_bounds(), 1.0);
        g.set_area(0, 0, 0, area_type::WALKABLE);
        g.set_area(3, 3, 3, area_type::SOLID);
        assert_eq!(g.occupied_count(), 2);
        g.clear();
        assert_eq!(g.occupied_count(), 0);
        assert!(g.is_empty_grid());
    }
}
