//! Per-region contour extraction in plan view.
//!
//! For each region produced by the watershed, trace the 2D boundary of
//! its walkable cells as a polygon: one outer ring (CCW) plus zero or
//! more holes (CW). Each contour vertex sits at a voxel cell corner and
//! carries:
//! - `xy`: world-space plan-view position
//! - `z`: world-space height sampled from the cell the walker was in
//!   when it emitted the vertex (top of that cell's walkable layer)
//! - `across_region`: the region on the OTHER side of the contour edge
//!   that goes from this vertex to the next, or [`RegionId::INVALID`]
//!   if there's no region there (the outside of the navmesh, or air
//!   above a cliff). This is the bookkeeping the portal-extraction
//!   stage uses to find shared edges between neighbor regions.
//!
//! # Algorithm
//!
//! Standard square-tracing boundary walker, adapted to multi-layer
//! cells via the [`CompactHeightfield`] surface index:
//! - A contour edge is `(cell, direction)` where the cell's cardinal
//!   neighbor in `direction` is NOT in this region (different region,
//!   or off-grid).
//! - Walk: if the current `(cell, direction)` is a boundary edge, emit
//!   the edge's CCW-end corner as a vertex and rotate the direction
//!   CCW (next direction). Else, step into the neighbor cell and
//!   rotate CW (previous direction).
//! - Terminate when state returns to the start.
//!
//! Repeat for every unvisited boundary edge — each closed loop is one
//! contour. Classify outer vs holes by signed area (positive = CCW =
//! outer; negative = CW = hole).
//!
//! # No simplification yet
//!
//! Raw contour vertices appear at every voxel-corner transition, so a
//! square 10×10 region produces ~40 vertices not 4. Douglas-Peucker /
//! collinear-vertex merging is deliberately deferred — downstream
//! consumers can simplify with whatever tolerance they prefer.

use crate::walkability::CompactHeightfield;
use crate::watershed::RegionMap;
use crate::RegionId;
use rsnav_common::{Vec3, Vertex};
use std::collections::HashSet;

#[derive(Clone, Debug)]
pub struct RegionContours {
    pub origin: Vec3,
    pub cell_size: f64,
    pub region_count: u32,
    /// One entry per region. `contours[rid.index()]` may be empty if the
    /// region was filtered out or produced no contour (degenerate cases).
    pub contours: Vec<RegionContour>,
}

#[derive(Clone, Debug, Default)]
pub struct RegionContour {
    /// CCW outer ring. Empty for an empty/degenerate region.
    pub outer: Vec<ContourVertex>,
    /// CW hole rings (zero or more).
    pub holes: Vec<Vec<ContourVertex>>,
}

impl RegionContour {
    pub fn is_empty(&self) -> bool {
        self.outer.is_empty()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ContourVertex {
    /// World-space XY at a voxel cell corner.
    pub xy: Vertex,
    /// World-space Z, taken as the top of the source cell's walkable layer.
    pub z: f64,
    /// The region on the OTHER side of the contour edge `this → next`.
    /// [`RegionId::INVALID`] when no region is across (outside of the
    /// navmesh, or empty air).
    pub across_region: RegionId,
}

impl ContourVertex {
    pub fn position3d(self) -> Vec3 {
        Vec3::new(self.xy.x, self.xy.y, self.z)
    }
}

pub fn extract_contours(chf: &CompactHeightfield, region_map: &RegionMap) -> RegionContours {
    let mut contours: Vec<RegionContour> =
        vec![RegionContour::default(); region_map.region_count as usize];
    // Visited boundary edges, keyed by (col, row, surface_index, direction).
    let mut visited: HashSet<(u32, u32, u32, usize)> = HashSet::new();

    for (c, r, si, rid) in region_map.iter() {
        if rid == RegionId::INVALID {
            continue;
        }
        for k in 0..4 {
            if visited.contains(&(c, r, si, k)) {
                continue;
            }
            if !is_boundary_edge(chf, region_map, c, r, si, k) {
                continue;
            }
            let walk = walk_contour(chf, region_map, c, r, si, k, &mut visited);
            if walk.len() < 3 {
                continue; // degenerate (shouldn't happen, but defensive)
            }
            let area = signed_area_2d(&walk);
            let entry = &mut contours[rid.index()];
            if area > 0.0 {
                // CCW → outer. Replace any prior outer (region should have only
                // one outer; if we somehow get a second, keep the larger).
                if signed_area_2d(&entry.outer) < area {
                    if !entry.outer.is_empty() {
                        // Demote the previous outer to a "hole" (rare; could
                        // happen if a region has multiple plan-view loops,
                        // which the layer constraint shouldn't allow).
                        let prev = std::mem::take(&mut entry.outer);
                        entry.holes.push(reverse_to_cw(prev));
                    }
                    entry.outer = walk;
                } else {
                    entry.holes.push(reverse_to_cw(walk));
                }
            } else {
                entry.holes.push(walk); // already CW
            }
        }
    }

    RegionContours {
        origin: chf.origin,
        cell_size: chf.cell_size,
        region_count: region_map.region_count,
        contours,
    }
}

/// Reverse a vertex sequence and update `across_region` so the across
/// labels still describe the region across each edge after reversal.
fn reverse_to_cw(mut verts: Vec<ContourVertex>) -> Vec<ContourVertex> {
    // For each edge i → i+1, the across is verts[i].across_region. After
    // reversal, that same across should belong to the edge (n-1-i) →
    // (n-2-i), which becomes (after reversal) edge i' → i'+1 where i' =
    // n-1-i. So after reversing the vector, we need to shift the across
    // labels by one slot.
    verts.reverse();
    if verts.len() >= 2 {
        let last_across = verts[verts.len() - 1].across_region;
        for i in (1..verts.len()).rev() {
            verts[i].across_region = verts[i - 1].across_region;
        }
        verts[0].across_region = last_across;
    }
    verts
}

fn is_boundary_edge(
    chf: &CompactHeightfield,
    region_map: &RegionMap,
    c: u32,
    r: u32,
    si: u32,
    k: usize,
) -> bool {
    let my_rid = region_map.at(c, r, si);
    match chf.neighbor_cell(c, r, si, k) {
        None => true,
        Some((nc, nr, n_si)) => region_map.at(nc, nr, n_si) != my_rid,
    }
}

fn neighbor_region(
    chf: &CompactHeightfield,
    region_map: &RegionMap,
    c: u32,
    r: u32,
    si: u32,
    k: usize,
) -> RegionId {
    match chf.neighbor_cell(c, r, si, k) {
        None => RegionId::INVALID,
        Some((nc, nr, n_si)) => region_map.at(nc, nr, n_si),
    }
}

/// CCW-end corner (in cell-grid coords) of the edge for direction `k`
/// in cell `(c, r)`. Walking the contour CCW with the region on our
/// left, this is the vertex we emit as we finish each edge.
fn edge_end_corner(c: u32, r: u32, k: usize) -> (u32, u32) {
    match k {
        0 => (c + 1, r + 1), // +X edge, traverse +Y, end at top-right
        1 => (c, r + 1),     // +Y edge, traverse -X, end at top-left
        2 => (c, r),         // -X edge, traverse -Y, end at bottom-left
        3 => (c + 1, r),     // -Y edge, traverse +X, end at bottom-right
        _ => unreachable!(),
    }
}

fn corner_world_xy(origin: Vec3, cell_size: f64, c: u32, r: u32) -> Vertex {
    Vertex::new(
        origin.x + c as f64 * cell_size,
        origin.y + r as f64 * cell_size,
    )
}

fn walk_contour(
    chf: &CompactHeightfield,
    region_map: &RegionMap,
    start_c: u32,
    start_r: u32,
    start_si: u32,
    start_k: usize,
    visited: &mut HashSet<(u32, u32, u32, usize)>,
) -> Vec<ContourVertex> {
    let mut verts: Vec<ContourVertex> = Vec::new();
    let mut c = start_c;
    let mut r = start_r;
    let mut si = start_si;
    let mut k = start_k;
    // Safety cap so a buggy walker can't spin forever.
    let max_steps = (chf.cols as usize) * (chf.rows as usize) * 8 + 16;

    for _ in 0..max_steps {
        if is_boundary_edge(chf, region_map, c, r, si, k) {
            visited.insert((c, r, si, k));
            let (ec, er) = edge_end_corner(c, r, k);
            let xy = corner_world_xy(chf.origin, chf.cell_size, ec, er);
            let surfaces = chf.surfaces_at(c, r);
            let layer = surfaces[si as usize].layer;
            // Z = top of the source cell's walkable voxel. If this edge
            // faces ANOTHER walkable region (a portal), average with the
            // neighbor's surface Z so both regions agree at the seam —
            // visually closes the gap that voxel quantization otherwise
            // leaves between two regions at slightly-different layers.
            let my_z = chf.origin.z + (layer as f64 + 1.0) * chf.cell_size;
            let across = neighbor_region(chf, region_map, c, r, si, k);
            let z = if across != RegionId::INVALID {
                if let Some((nc, nr, n_si)) = chf.neighbor_cell(c, r, si, k) {
                    let n_layer = chf.surfaces_at(nc, nr)[n_si as usize].layer;
                    let n_z = chf.origin.z + (n_layer as f64 + 1.0) * chf.cell_size;
                    (my_z + n_z) * 0.5
                } else {
                    my_z
                }
            } else {
                my_z
            };
            verts.push(ContourVertex {
                xy,
                z,
                across_region: across,
            });
            k = (k + 1) % 4; // turn CCW (left of walker)
        } else {
            // Step into the neighbor cell, then turn CW.
            let (nc, nr, n_si) = chf
                .neighbor_cell(c, r, si, k)
                .expect("non-boundary direction must have a neighbor");
            c = nc;
            r = nr;
            si = n_si;
            k = (k + 3) % 4;
        }

        if c == start_c && r == start_r && si == start_si && k == start_k && !verts.is_empty() {
            break;
        }
    }

    verts
}

fn signed_area_2d(verts: &[ContourVertex]) -> f64 {
    let n = verts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let v = verts[i].xy;
        let w = verts[(i + 1) % n].xy;
        sum += v.x * w.y - w.x * v.y;
    }
    sum * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{WalkabilityConfig, WatershedConfig};
    use crate::walkability::classify_walkability;
    use crate::watershed::segment;
    use crate::{synth, VoxelGrid};

    fn build_pipeline(soup: &crate::PolySoup, cell_size: f64) -> (CompactHeightfield, RegionMap) {
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let ws = WatershedConfig::default();
        let rm = segment(&chf, &ws);
        (chf, rm)
    }

    #[test]
    fn flat_plane_outer_is_ccw_rectangle() {
        let soup = synth::plane(4.0, 4.0, 1);
        let (chf, rm) = build_pipeline(&soup, 1.0);
        assert_eq!(rm.region_count, 1);
        let contours = extract_contours(&chf, &rm);
        let r0 = &contours.contours[0];
        assert!(!r0.outer.is_empty(), "should have an outer contour");
        assert!(r0.holes.is_empty(), "no holes expected for a plane");
        // Area should be positive (CCW) and close to the plane's footprint
        // (4×4 = 16 m^2, less the padding-cell shrinkage at the boundary).
        let area = signed_area_2d(&r0.outer);
        assert!(area > 0.0, "outer should be CCW (positive area), got {}", area);
        // Generously: between 12 and 25 m^2 (raw voxelization has stair-step boundary)
        assert!(area >= 12.0 && area <= 25.0, "area {} outside expected range", area);
    }

    #[test]
    fn small_plane_contour_traces_rectangular_boundary() {
        // A 2 m × 2 m plane at 0.5 m cells: exact cell count depends on
        // boundary alignment + voxelizer tolerance. We just verify the
        // contour traces a roughly-rectangular boundary at the right
        // physical scale.
        let soup = synth::plane(2.0, 2.0, 1);
        let (chf, rm) = build_pipeline(&soup, 0.5);
        let contours = extract_contours(&chf, &rm);
        let outer = &contours.contours[0].outer;
        assert!(
            outer.len() >= 8,
            "expected ≥8 perimeter vertices, got {}",
            outer.len()
        );
        let min_x = outer.iter().map(|v| v.xy.x).fold(f64::INFINITY, f64::min);
        let max_x = outer.iter().map(|v| v.xy.x).fold(f64::NEG_INFINITY, f64::max);
        let min_y = outer.iter().map(|v| v.xy.y).fold(f64::INFINITY, f64::min);
        let max_y = outer.iter().map(|v| v.xy.y).fold(f64::NEG_INFINITY, f64::max);
        let span_x = max_x - min_x;
        let span_y = max_y - min_y;
        assert!(
            span_x >= 2.0 && span_x <= 3.0,
            "x span {} not roughly 2m",
            span_x
        );
        assert!(
            span_y >= 2.0 && span_y <= 3.0,
            "y span {} not roughly 2m",
            span_y
        );
    }

    #[test]
    fn under_ramp_two_regions_each_get_a_contour() {
        // Same scene that produces two regions in the watershed test.
        let mut soup = synth::plane(10.0, 4.0, 1);
        let ramp = synth::ramp(0.0, 4.0, 3.0, 4.0);
        let base = soup.vertices.len() as u32;
        soup.vertices.extend(ramp.vertices.iter().copied());
        for t in &ramp.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let (chf, rm) = build_pipeline(&soup, 0.2);
        assert!(rm.region_count >= 2);
        let contours = extract_contours(&chf, &rm);
        let with_outer = contours
            .contours
            .iter()
            .filter(|c| !c.outer.is_empty())
            .count();
        assert_eq!(
            with_outer, rm.region_count as usize,
            "every region should produce one outer contour"
        );
    }

    #[test]
    fn contour_vertices_lie_on_grid_corners() {
        let soup = synth::plane(3.0, 3.0, 1);
        let (chf, rm) = build_pipeline(&soup, 0.5);
        let contours = extract_contours(&chf, &rm);
        let outer = &contours.contours[0].outer;
        let cs = chf.cell_size;
        for v in outer {
            // Each vertex's (x, y) should be at an integer multiple of cell_size
            // from origin (i.e., at a voxel corner).
            let rel_x = (v.xy.x - chf.origin.x) / cs;
            let rel_y = (v.xy.y - chf.origin.y) / cs;
            assert!(
                (rel_x.round() - rel_x).abs() < 1e-9,
                "vertex x={} not on grid",
                v.xy.x
            );
            assert!(
                (rel_y.round() - rel_y).abs() < 1e-9,
                "vertex y={} not on grid",
                v.xy.y
            );
        }
    }

    #[test]
    fn flat_plane_across_region_is_invalid_everywhere() {
        // A region surrounded by void — every contour edge faces nothing.
        let soup = synth::plane(2.0, 2.0, 1);
        let (chf, rm) = build_pipeline(&soup, 0.5);
        let contours = extract_contours(&chf, &rm);
        for v in &contours.contours[0].outer {
            assert_eq!(
                v.across_region,
                RegionId::INVALID,
                "isolated plane should have INVALID across everywhere"
            );
        }
    }

    #[test]
    fn two_disconnected_planes_each_get_their_own_contour() {
        let mut soup = synth::plane(3.0, 3.0, 1);
        let roof = synth::plane(3.0, 3.0, 1);
        let base = soup.vertices.len() as u32;
        soup.vertices
            .extend(roof.vertices.iter().map(|v| Vec3::new(v.x, v.y, 5.0)));
        for t in &roof.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let (chf, rm) = build_pipeline(&soup, 0.5);
        assert_eq!(rm.region_count, 2);
        let contours = extract_contours(&chf, &rm);
        assert!(!contours.contours[0].outer.is_empty());
        assert!(!contours.contours[1].outer.is_empty());
    }
}
