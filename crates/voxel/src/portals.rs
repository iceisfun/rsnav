//! Portal extraction: shared boundary segments between adjacent regions.
//!
//! Each region's contour vertices carry an `across_region` field that
//! says which region (if any) is on the other side of the edge going
//! `v[i] → v[i+1]`. Walking each region's outer ring (and holes), we
//! group consecutive edges that have the same non-`INVALID`
//! `across_region` into one [`Portal`] — the polyline along the
//! shared seam.
//!
//! # Canonical ordering
//!
//! Each portal between regions A and B can be discovered TWICE (once
//! while walking A's contour, once while walking B's). We dedupe by
//! emitting only when `region.0 < across.0`, so each portal appears
//! exactly once in the output, with `a < b`.
//!
//! # Wrap-around handling
//!
//! A run of portal edges can span the ring's index-0 boundary (e.g.,
//! ring edges 4, 5, 0, 1, 2 all face the same neighbor). The walker
//! rotates its starting point to the first non-portal edge so the run
//! is captured as one continuous polyline rather than two.

use crate::contour::{ContourVertex, RegionContours};
use crate::output::Portal;
use crate::RegionId;
use rsnav_common::Vec3;

/// Extract all portals from a set of region contours. The result lists
/// each (A, B) portal exactly once with `a.0 < b.0`.
pub fn extract_portals(contours: &RegionContours) -> Vec<Portal> {
    let mut out = Vec::new();
    for (i, contour) in contours.contours.iter().enumerate() {
        let rid = RegionId(i as u32);
        extract_portals_from_ring(&contour.outer, rid, &mut out);
        for hole in &contour.holes {
            extract_portals_from_ring(hole, rid, &mut out);
        }
    }
    out
}

fn extract_portals_from_ring(ring: &[ContourVertex], region: RegionId, out: &mut Vec<Portal>) {
    let n = ring.len();
    if n == 0 {
        return;
    }

    // Find an index whose edge faces nothing (across == INVALID) so the
    // walker starts on a non-portal edge. This makes wrap-around runs
    // capture as one polyline. If the entire ring is portal (one
    // neighbor surrounds us), emit one portal that covers the whole
    // loop.
    let start_offset = (0..n).find(|&i| ring[i].across_region == RegionId::INVALID);

    let Some(start_offset) = start_offset else {
        // Whole ring is a single portal (rare).
        let across = ring[0].across_region;
        let mut polyline: Vec<Vec3> = ring.iter().map(|v| v.position3d()).collect();
        // Close the loop so the polyline is "from start back to start".
        polyline.push(polyline[0]);
        let max_step = polyline_max_step(&polyline);
        if region.0 < across.0 {
            out.push(Portal {
                a: region,
                b: across,
                edge: polyline,
                max_height_step: max_step,
            });
        }
        return;
    };

    let mut i = 0;
    while i < n {
        let idx = (start_offset + i) % n;
        let across = ring[idx].across_region;
        if across == RegionId::INVALID {
            i += 1;
            continue;
        }
        // Find the end of this run of same-across edges.
        let mut run_len = 1;
        while i + run_len < n {
            let next_idx = (start_offset + i + run_len) % n;
            if ring[next_idx].across_region != across {
                break;
            }
            run_len += 1;
        }
        // Edges from idx to idx+run_len-1, vertices [idx..=idx+run_len].
        let mut polyline = Vec::with_capacity(run_len + 1);
        for k in 0..=run_len {
            polyline.push(ring[(start_offset + i + k) % n].position3d());
        }
        let max_step = polyline_max_step(&polyline);
        if region.0 < across.0 {
            out.push(Portal {
                a: region,
                b: across,
                edge: polyline,
                max_height_step: max_step,
            });
        }
        i += run_len;
    }
}

fn polyline_max_step(polyline: &[Vec3]) -> f64 {
    polyline
        .windows(2)
        .map(|w| (w[1].z - w[0].z).abs())
        .fold(0.0f64, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{WalkabilityConfig, WatershedConfig};
    use crate::contour::extract_contours;
    use crate::walkability::classify_walkability;
    use crate::watershed::segment;
    use crate::{synth, VoxelGrid};
    use rsnav_common::Vec3;

    fn pipeline_contours(soup: &crate::PolySoup, cell_size: f64) -> RegionContours {
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let rm = segment(&chf, &WatershedConfig::default());
        extract_contours(&chf, &rm)
    }

    #[test]
    fn isolated_plane_produces_no_portals() {
        let soup = synth::plane(4.0, 4.0, 1);
        let contours = pipeline_contours(&soup, 0.5);
        let portals = extract_portals(&contours);
        assert!(
            portals.is_empty(),
            "plane has no adjacent regions, should yield no portals"
        );
    }

    #[test]
    fn floor_with_ramp_and_platform_has_some_portals() {
        let soup = synth::floor_with_ramp_and_platform();
        let contours = pipeline_contours(&soup, 0.2);
        let portals = extract_portals(&contours);
        // At least one portal between the floor and the ramp+platform.
        assert!(
            !portals.is_empty(),
            "expected at least one portal in floor+ramp+platform scene"
        );
        // Every portal a < b
        for p in &portals {
            assert!(p.a.0 < p.b.0, "portal not canonically ordered: {:?}", p);
            assert!(p.edge.len() >= 2, "portal polyline too short: {:?}", p);
        }
    }

    #[test]
    fn portal_midpoint_lies_on_polyline() {
        let soup = synth::floor_with_ramp_and_platform();
        let contours = pipeline_contours(&soup, 0.2);
        let portals = extract_portals(&contours);
        for p in &portals {
            let mid = p.midpoint();
            // Midpoint XY should lie within the XY range of the polyline.
            let xs: Vec<f64> = p.edge.iter().map(|v| v.x).collect();
            let ys: Vec<f64> = p.edge.iter().map(|v| v.y).collect();
            let min_x = xs.iter().copied().fold(f64::INFINITY, f64::min);
            let max_x = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let min_y = ys.iter().copied().fold(f64::INFINITY, f64::min);
            let max_y = ys.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            assert!(mid.x >= min_x - 1e-9 && mid.x <= max_x + 1e-9);
            assert!(mid.y >= min_y - 1e-9 && mid.y <= max_y + 1e-9);
        }
    }
}
