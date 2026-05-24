//! Triangle → voxel rasterization using the Akenine-Möller separating-axis
//! test for triangle/AABB overlap, with per-triangle slope classification.
//!
//! Each triangle is classified by the angle between its plane normal and the
//! world-up axis (Z). Triangles flatter than `cos_max_slope` are tagged
//! [`area_type::WALKABLE`]; everything else (walls, ceilings, steep slopes)
//! is tagged [`area_type::SOLID`]. When two triangles overlap the same cell,
//! the higher area value wins ([`VoxelGrid::merge_area`]) — so a floor seam
//! against a wall stays walkable.
//!
//! Output is still a **surface** voxelization — a shell of voxels along the
//! triangle, no interior fill. Walkability semantics are surface-only too.
//!
//! Reference: T. Akenine-Möller, "Fast 3D Triangle-Box Overlap Testing",
//! Journal of Graphics Tools 6(1):29–33, 2001.

use crate::grid::area_type;
use crate::{PolySoup, VoxelGrid};
use rsnav_common::Vec3;

/// Rasterize every triangle of `soup` into `grid`, tagging each touched cell
/// by area type. Existing cells are merged (max wins), so multiple
/// rasterization calls accumulate cleanly.
///
/// `cos_max_slope` is `cos(max_walkable_slope_angle)`. Triangles whose plane
/// normal has `|normal_unit.z| ≥ cos_max_slope` are walkable; the rest are
/// solid. Pass `1.0` for "only perfectly horizontal triangles are walkable",
/// `0.0` for "all triangles are walkable", and `cos(45°) ≈ 0.707` for the
/// typical default.
pub fn rasterize(grid: &mut VoxelGrid, soup: &PolySoup, cos_max_slope: f64) {
    let cs = grid.cell_size;
    let half = Vec3::new(cs * 0.5, cs * 0.5, cs * 0.5);
    let origin = grid.origin;

    let cols_i = grid.cols as i64;
    let rows_i = grid.rows as i64;
    let layers_i = grid.layers as i64;

    for [a, b, c] in soup.triangle_positions() {
        let area = classify_triangle([a, b, c], cos_max_slope);
        if area == area_type::EMPTY {
            continue; // degenerate triangle — nothing to mark
        }

        // Triangle world-space 3D AABB → candidate cell range.
        let mn = Vec3::new(
            a.x.min(b.x).min(c.x),
            a.y.min(b.y).min(c.y),
            a.z.min(b.z).min(c.z),
        );
        let mx = Vec3::new(
            a.x.max(b.x).max(c.x),
            a.y.max(b.y).max(c.y),
            a.z.max(b.z).max(c.z),
        );

        // Skip triangles fully outside the grid.
        if mx.x < origin.x
            || mn.x > origin.x + cs * cols_i as f64
            || mx.y < origin.y
            || mn.y > origin.y + cs * rows_i as f64
            || mx.z < origin.z
            || mn.z > origin.z + cs * layers_i as f64
        {
            continue;
        }

        let lo_c = (((mn.x - origin.x) / cs).floor() as i64).clamp(0, cols_i - 1);
        let hi_c = (((mx.x - origin.x) / cs).floor() as i64).clamp(0, cols_i - 1);
        let lo_r = (((mn.y - origin.y) / cs).floor() as i64).clamp(0, rows_i - 1);
        let hi_r = (((mx.y - origin.y) / cs).floor() as i64).clamp(0, rows_i - 1);
        let lo_l = (((mn.z - origin.z) / cs).floor() as i64).clamp(0, layers_i - 1);
        let hi_l = (((mx.z - origin.z) / cs).floor() as i64).clamp(0, layers_i - 1);

        for l in lo_l..=hi_l {
            for r in lo_r..=hi_r {
                for col in lo_c..=hi_c {
                    let center = grid.cell_center(col as u32, r as u32, l as u32);
                    if tri_box_overlap([a, b, c], center, half) {
                        grid.merge_area(col as u32, r as u32, l as u32, area);
                    }
                }
            }
        }
    }
}

/// Classify a triangle by its slope from horizontal. Returns
/// [`area_type::WALKABLE`] when the plane is within the walkable angle,
/// [`area_type::SOLID`] otherwise, and [`area_type::EMPTY`] for degenerate
/// (zero-area) triangles which contribute nothing.
pub fn classify_triangle(tri: [Vec3; 3], cos_max_slope: f64) -> u8 {
    let e0 = tri[1] - tri[0];
    let e1 = tri[2] - tri[0];
    let n = e0.cross(e1);
    let len = n.length();
    if len == 0.0 {
        return area_type::EMPTY;
    }
    // |normal_unit.z| measures how horizontal the plane is. 1.0 = perfectly
    // horizontal (floor or ceiling), 0.0 = vertical (wall). Treat ceilings
    // as walkable here too — the walkability classifier filters them out
    // later by clearance (you can't stand on a ceiling because there's no
    // empty space above it).
    let horiz_dot = (n.z / len).abs();
    if horiz_dot >= cos_max_slope {
        area_type::WALKABLE
    } else {
        area_type::SOLID
    }
}

/// Akenine-Möller separating-axis test: returns `true` if the triangle
/// intersects the AABB centered at `box_center` with half-extents `half`.
///
/// Tests 13 potential separating axes: 3 box face normals, the triangle's
/// plane normal, and 9 cross products of (triangle edge × box axis).
/// Degenerate (zero-length) cross axes are skipped — they don't separate
/// anything.
pub fn tri_box_overlap(tri: [Vec3; 3], box_center: Vec3, half: Vec3) -> bool {
    let v0 = tri[0] - box_center;
    let v1 = tri[1] - box_center;
    let v2 = tri[2] - box_center;
    let local = [v0, v1, v2];
    let e0 = v1 - v0;
    let e1 = v2 - v1;
    let e2 = v0 - v2;

    let ax_x = Vec3::new(1.0, 0.0, 0.0);
    let ax_y = Vec3::new(0.0, 1.0, 0.0);
    let ax_z = Vec3::new(0.0, 0.0, 1.0);

    if !axis_test(ax_x, local, half) {
        return false;
    }
    if !axis_test(ax_y, local, half) {
        return false;
    }
    if !axis_test(ax_z, local, half) {
        return false;
    }

    let n = e0.cross(e1);
    if !axis_test(n, local, half) {
        return false;
    }

    for &edge in &[e0, e1, e2] {
        for &box_ax in &[ax_x, ax_y, ax_z] {
            let a = edge.cross(box_ax);
            if a.length_sq() > 1e-20 && !axis_test(a, local, half) {
                return false;
            }
        }
    }

    true
}

#[inline]
fn axis_test(axis: Vec3, tri: [Vec3; 3], half: Vec3) -> bool {
    let p0 = axis.dot(tri[0]);
    let p1 = axis.dot(tri[1]);
    let p2 = axis.dot(tri[2]);
    let mn = p0.min(p1).min(p2);
    let mx = p0.max(p1).max(p2);
    let r = half.x * axis.x.abs() + half.y * axis.y.abs() + half.z * axis.z.abs();
    // Boundary tolerance: a triangle sitting *exactly* on a cell face
    // (a floor at z=0 vs a voxel face at z=0) will miss the strict test
    // by a few ULPs because the cell-center computation accumulates FP
    // error. Without this slack the floor at z=0 fails to voxelize at
    // some cells but not others, depending on which side of -r the
    // box-local projection rounds to. Tolerance scales with `r` so that
    // genuinely-separated geometry still tests as separated.
    let eps = r * 1e-9 + 1e-12;
    !(mn > r + eps || mx < -r - eps)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;

    const COS_45: f64 = 0.707_106_781_186_547_5; // cos(45°) ≈ √2/2

    fn box_at(c: Vec3, half: f64) -> (Vec3, Vec3) {
        (c, Vec3::new(half, half, half))
    }

    #[test]
    fn classify_horizontal_is_walkable() {
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        assert_eq!(classify_triangle(tri, COS_45), area_type::WALKABLE);
    }

    #[test]
    fn classify_vertical_is_solid() {
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        assert_eq!(classify_triangle(tri, COS_45), area_type::SOLID);
    }

    #[test]
    fn classify_gentle_ramp_is_walkable() {
        // 30° ramp: rises 1 unit per √3 horizontal → normal angle 30° from up.
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(3.0_f64.sqrt(), 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        assert_eq!(classify_triangle(tri, COS_45), area_type::WALKABLE);
    }

    #[test]
    fn classify_steep_ramp_is_solid() {
        // 60° ramp: should fail the 45° walkability threshold.
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 3.0_f64.sqrt()),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        assert_eq!(classify_triangle(tri, COS_45), area_type::SOLID);
    }

    #[test]
    fn classify_degenerate_triangle() {
        let tri = [Vec3::ZERO, Vec3::ZERO, Vec3::ZERO];
        assert_eq!(classify_triangle(tri, COS_45), area_type::EMPTY);
    }

    #[test]
    fn triangle_clearly_inside_box() {
        let (center, half) = box_at(Vec3::ZERO, 2.0);
        let tri = [
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        assert!(tri_box_overlap(tri, center, half));
    }

    #[test]
    fn triangle_clearly_outside_box() {
        let (center, half) = box_at(Vec3::ZERO, 1.0);
        let tri = [
            Vec3::new(10.0, 10.0, 10.0),
            Vec3::new(12.0, 10.0, 10.0),
            Vec3::new(11.0, 12.0, 10.0),
        ];
        assert!(!tri_box_overlap(tri, center, half));
    }

    #[test]
    fn vertical_triangle_through_box_center() {
        let (center, half) = box_at(Vec3::ZERO, 1.0);
        let tri = [
            Vec3::new(0.0, -2.0, -2.0),
            Vec3::new(0.0, 2.0, -2.0),
            Vec3::new(0.0, 0.0, 2.0),
        ];
        assert!(tri_box_overlap(tri, center, half));
    }

    #[test]
    fn rasterize_horizontal_plane_marks_walkable() {
        let soup = synth::plane(10.0, 10.0, 1);
        let grid = VoxelGrid::from_polysoup(&soup, 1.0, 0, COS_45);
        assert!(grid.walkable_count() >= 100);
        // The plane is horizontal — every occupied cell should be walkable,
        // not solid.
        for (c, r, l, a) in grid.iter_with_area() {
            assert_eq!(
                a,
                area_type::WALKABLE,
                "cell ({},{},{}) should be walkable",
                c,
                r,
                l
            );
        }
    }

    #[test]
    fn rasterize_ramp_walkability_depends_on_slope() {
        // 30° ramp: walkable under 45° threshold.
        let soup = synth::ramp(0.0, 4.0, 4.0 / 3.0_f64.sqrt(), 3.0);
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 0, COS_45);
        assert!(grid.walkable_count() > 0);
        // 60° ramp: solid under 45° threshold.
        let soup_steep = synth::ramp(0.0, 1.0, 3.0_f64.sqrt(), 3.0);
        let grid_steep = VoxelGrid::from_polysoup(&soup_steep, 0.5, 0, COS_45);
        assert_eq!(grid_steep.walkable_count(), 0);
        assert!(grid_steep.occupied_count() > 0);
    }

    #[test]
    fn rasterize_box_top_and_bottom_walkable_sides_solid() {
        // Tall box: top and bottom faces are horizontal (walkable);
        // sides are vertical (solid). With walkable winning over solid in
        // merge, the top/bottom layers should be all walkable, side-only
        // cells should be solid.
        let soup = synth::box_aabb(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0));
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 0, COS_45);
        // Bottom face layer (layer 0): every occupied cell touched a
        // horizontal triangle → walkable.
        for c in 0..4 {
            for r in 0..4 {
                assert_eq!(grid.area_at(c, r, 0), area_type::WALKABLE);
                assert_eq!(grid.area_at(c, r, 3), area_type::WALKABLE);
            }
        }
        // Middle layer (layer 1): corner cells touch only side walls →
        // solid; interior is empty.
        assert_eq!(grid.area_at(0, 0, 1), area_type::SOLID);
        assert_eq!(grid.area_at(3, 3, 1), area_type::SOLID);
        assert_eq!(grid.area_at(1, 1, 1), area_type::EMPTY);
    }
}
