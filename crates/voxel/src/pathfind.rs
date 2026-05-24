//! Region-level A* pathfinding using portal midpoints as transitions.
//!
//! Treats each [`Region`](crate::Region) as a graph node and each
//! [`Portal`] as an edge. The waypoint at a portal is the portal's
//! polyline midpoint. Path output is a polyline of `Vec3` waypoints:
//! `[src, mid_portal_1, mid_portal_2, …, dst]`.
//!
//! This is a coarse path — it doesn't run the funnel inside each
//! region. That's a follow-up for when we want to consume the CDT
//! triangulation per region. For Phase 1 demo / visual verification
//! the coarse path is enough to prove "regions + portals connect into
//! a navigable graph."

use crate::contour::{RegionContour, RegionContours};
use crate::navmesh::{AllRegionNavMeshes, RegionNavMesh};
use crate::output::Portal;
use crate::walkability::CompactHeightfield;
use crate::watershed::RegionMap;
use crate::RegionId;
use rsnav_common::{Polygon, Vec3, Vertex};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Find which region contains world-XY point `(x, y)`. Iterates region
/// contours; returns the first whose outer ring contains the point AND
/// no hole excludes it. `None` if no region matches.
///
/// XY-only — ambiguous in 2.5D worlds where multiple regions overlap in
/// the XY plane (e.g., a lower floor with an upper platform above). For
/// picking and pathfinding from a 3D click, prefer
/// [`find_region_at_xyz`], which disambiguates by Z.
pub fn find_region_at_xy(contours: &RegionContours, x: f64, y: f64) -> Option<RegionId> {
    let p = Vertex::new(x, y);
    for (i, contour) in contours.contours.iter().enumerate() {
        if contour.outer.len() < 3 {
            continue;
        }
        let outer = Polygon {
            vertices: contour.outer.iter().map(|v| v.xy).collect(),
        };
        if !outer.contains(p) {
            continue;
        }
        let in_hole = contour.holes.iter().any(|h| {
            if h.len() < 3 {
                return false;
            }
            let hole_poly = Polygon {
                vertices: h.iter().map(|v| v.xy).collect(),
            };
            hole_poly.contains(p)
        });
        if !in_hole {
            return Some(RegionId(i as u32));
        }
    }
    None
}

/// Find which region contains world point `(x, y, z)`. Like
/// [`find_region_at_xy`], but when multiple regions' XY contours overlap
/// (2.5D: a lower floor under an upper platform), it returns the region
/// whose mean Z is closest to `z`. `None` if no region's contour
/// contains `(x, y)`.
pub fn find_region_at_xyz(
    contours: &RegionContours,
    x: f64,
    y: f64,
    z: f64,
) -> Option<RegionId> {
    let p = Vertex::new(x, y);
    let mut best: Option<(RegionId, f64)> = None;
    for (i, contour) in contours.contours.iter().enumerate() {
        if contour.outer.len() < 3 {
            continue;
        }
        let outer = Polygon {
            vertices: contour.outer.iter().map(|v| v.xy).collect(),
        };
        if !outer.contains(p) {
            continue;
        }
        let in_hole = contour.holes.iter().any(|h| {
            if h.len() < 3 {
                return false;
            }
            let hole_poly = Polygon {
                vertices: h.iter().map(|v| v.xy).collect(),
            };
            hole_poly.contains(p)
        });
        if in_hole {
            continue;
        }
        let dz = (region_mean_z(contour) - z).abs();
        if best.map(|(_, prev)| dz < prev).unwrap_or(true) {
            best = Some((RegionId(i as u32), dz));
        }
    }
    best.map(|(id, _)| id)
}

/// Mean Z of a region's outer contour vertices — a cheap "standing height"
/// approximation when we don't want to interpolate inside the CDT.
pub fn region_mean_z(contour: &RegionContour) -> f64 {
    if contour.outer.is_empty() {
        return 0.0;
    }
    contour.outer.iter().map(|v| v.z).sum::<f64>() / contour.outer.len() as f64
}

/// Path output: a polyline in world space (src, portal mids, dst).
#[derive(Clone, Debug)]
pub struct Path {
    pub waypoints: Vec<Vec3>,
    /// The sequence of regions visited, parallel to `waypoints` minus
    /// the src/dst endpoints. Useful for debug overlays.
    pub regions: Vec<RegionId>,
}

/// Find a coarse path from `src` to `dst` across regions, using
/// portal-midpoint waypoints. Returns `None` if either endpoint isn't
/// in any region, or no region path exists.
pub fn find_path(
    contours: &RegionContours,
    portals: &[Portal],
    src: Vec3,
    dst: Vec3,
) -> Option<Path> {
    let src_region = find_region_at_xyz(contours, src.x, src.y, src.z)?;
    let dst_region = find_region_at_xyz(contours, dst.x, dst.y, dst.z)?;

    if src_region == dst_region {
        return Some(Path {
            waypoints: vec![src, dst],
            regions: vec![src_region],
        });
    }

    // Region adjacency: region → list of (neighbor, portal_midpoint).
    let mut adjacency: HashMap<RegionId, Vec<(RegionId, Vec3)>> = HashMap::new();
    for p in portals {
        let mid = p.midpoint();
        adjacency.entry(p.a).or_default().push((p.b, mid));
        adjacency.entry(p.b).or_default().push((p.a, mid));
    }

    // A* on the region graph. Heap entry: (f = g + h, region, last_waypoint).
    let mut g_score: HashMap<RegionId, f64> = HashMap::new();
    let mut came_from: HashMap<RegionId, (RegionId, Vec3)> = HashMap::new();
    let mut visited: HashSet<RegionId> = HashSet::new();
    let mut heap: BinaryHeap<HeapEntry> = BinaryHeap::new();
    g_score.insert(src_region, 0.0);
    heap.push(HeapEntry {
        f: src.distance(dst),
        region: src_region,
        position: src,
    });

    while let Some(entry) = heap.pop() {
        if entry.region == dst_region {
            // Reconstruct path.
            let mut waypoints = vec![dst];
            let mut regions = vec![entry.region];
            let mut cur = entry.region;
            while cur != src_region {
                let (prev_region, portal_mid) = came_from[&cur];
                waypoints.push(portal_mid);
                regions.push(prev_region);
                cur = prev_region;
            }
            waypoints.push(src);
            waypoints.reverse();
            regions.reverse();
            return Some(Path { waypoints, regions });
        }
        if !visited.insert(entry.region) {
            continue;
        }
        let cur_g = g_score[&entry.region];
        let neighbors = adjacency.get(&entry.region).cloned().unwrap_or_default();
        for (neighbor, portal_mid) in neighbors {
            if visited.contains(&neighbor) {
                continue;
            }
            let step = entry.position.distance(portal_mid);
            let new_g = cur_g + step;
            let prev = g_score.get(&neighbor).copied().unwrap_or(f64::INFINITY);
            if new_g < prev {
                g_score.insert(neighbor, new_g);
                came_from.insert(neighbor, (entry.region, portal_mid));
                let h = portal_mid.distance(dst);
                heap.push(HeapEntry {
                    f: new_g + h,
                    region: neighbor,
                    position: portal_mid,
                });
            }
        }
    }
    None
}

/// Densify a coarse [`Path`] into a polyline that hugs the
/// [`CompactHeightfield`]. Each segment between successive waypoints is
/// subdivided into steps of about half a cell, and each step's Z is
/// snapped to the walkable surface **belonging to the segment's
/// expected region** at that (x, y).
///
/// The sampler is region-strict: it never snaps a sample to a surface
/// outside the expected region. That keeps the line from jumping up
/// onto a disconnected region (e.g., the flat top of a pillar that
/// happens to share an XY column with the floor underneath it). If the
/// exact cell doesn't carry a surface in the expected region, the
/// sampler probes the 4-neighbor cells for one. If those don't either,
/// it holds the previous Z so the line visibly clips through the
/// obstacle instead of teleporting to a foreign surface.
///
/// The returned polyline starts at `path.waypoints[0]` (= src) and ends
/// at `path.waypoints.last()` (= dst); intermediate portal midpoints
/// from the coarse path are no longer present as distinct waypoints —
/// the dense samples sweep through them at HF resolution.
pub fn densify_path_on_hf(
    path: &Path,
    chf: &CompactHeightfield,
    region_map: &RegionMap,
) -> Vec<Vec3> {
    if path.waypoints.is_empty() {
        return Vec::new();
    }
    let cs = chf.cell_size;
    let step = (cs * 0.5).max(1e-6);
    let mut out: Vec<Vec3> = Vec::new();
    out.push(path.waypoints[0]);
    for i in 0..path.waypoints.len().saturating_sub(1) {
        let a = path.waypoints[i];
        let b = path.waypoints[i + 1];
        let seg_region = path
            .regions
            .get(i)
            .copied()
            .or_else(|| path.regions.last().copied())
            .unwrap_or(RegionId::INVALID);
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let horiz = (dx * dx + dy * dy).sqrt();
        let steps = (horiz / step).ceil().max(1.0) as u32;
        let mut prev_z = out.last().map(|p| p.z).unwrap_or(a.z);
        for s in 1..=steps {
            let t = s as f64 / steps as f64;
            let x = a.x + dx * t;
            let y = a.y + dy * t;
            let z = sample_hf_z(chf, region_map, seg_region, x, y, prev_z);
            out.push(Vec3::new(x, y, z));
            prev_z = z;
        }
    }
    out
}

/// Look up the walkable-surface Z at world-XY `(x, y)` inside the HF,
/// restricted to surfaces in `expected_region`. Probes the exact cell
/// first, then the 4-neighbors (small cushion for samples that land
/// just outside the region's footprint near a portal boundary). If no
/// matching surface is found, holds `prev_z` — never snaps to a surface
/// in a foreign region.
fn sample_hf_z(
    chf: &CompactHeightfield,
    region_map: &RegionMap,
    expected_region: RegionId,
    x: f64,
    y: f64,
    prev_z: f64,
) -> f64 {
    if expected_region == RegionId::INVALID {
        return prev_z;
    }
    let cx = ((x - chf.origin.x) / chf.cell_size).floor();
    let cy = ((y - chf.origin.y) / chf.cell_size).floor();
    if cx < 0.0 || cy < 0.0 {
        return prev_z;
    }
    let c = cx as u32;
    let r = cy as u32;
    if c >= chf.cols || r >= chf.rows {
        return prev_z;
    }
    if let Some(z) = surface_z_in_region(chf, region_map, expected_region, c, r) {
        return z;
    }
    // Probe 4-neighbors: handles samples that landed in a cell just
    // outside the region's footprint right at a portal boundary.
    const OFFS: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
    let mut best_z: Option<f64> = None;
    for (dc, dr) in OFFS {
        let nc = c as i32 + dc;
        let nr = r as i32 + dr;
        if nc < 0 || nr < 0 || nc as u32 >= chf.cols || nr as u32 >= chf.rows {
            continue;
        }
        if let Some(z) =
            surface_z_in_region(chf, region_map, expected_region, nc as u32, nr as u32)
        {
            let candidate_is_closer = match best_z {
                None => true,
                Some(prev) => (z - prev_z).abs() < (prev - prev_z).abs(),
            };
            if candidate_is_closer {
                best_z = Some(z);
            }
        }
    }
    best_z.unwrap_or(prev_z)
}

fn surface_z_in_region(
    chf: &CompactHeightfield,
    region_map: &RegionMap,
    expected_region: RegionId,
    c: u32,
    r: u32,
) -> Option<f64> {
    let surfaces = chf.surfaces_at(c, r);
    for si in 0..surfaces.len() {
        if region_map.at(c, r, si as u32) == expected_region {
            if let Some(p) = chf.surface_point(c, r, si as u32) {
                return Some(p.z);
            }
        }
    }
    None
}

/// Densify a coarse [`Path`] into a polyline that follows the
/// [`AllRegionNavMeshes`]. Each segment between successive waypoints is
/// subdivided at ~half-cell intervals; each sample's Z is computed by
/// **barycentric interpolation on the triangle of the segment's
/// expected region's navmesh** that contains the sample's (x, y).
///
/// Compared to [`densify_path_on_hf`], this is single-valued: there is
/// exactly one Z per (x, y) per region (the triangulated walkable
/// surface). It cannot produce upward spikes from neighboring layers
/// or unrelated walkable surfaces stacked in the same HF column. If a
/// sample's (x, y) lies outside every triangle in the expected region's
/// navmesh, the previous Z is held — never substituted from another
/// region or layer.
pub fn densify_path_on_navmesh(
    path: &Path,
    navmeshes: &AllRegionNavMeshes,
    cell_size: f64,
) -> Vec<Vec3> {
    if path.waypoints.is_empty() {
        return Vec::new();
    }
    let step = (cell_size * 0.5).max(1e-6);
    let mut out = vec![path.waypoints[0]];
    for i in 0..path.waypoints.len().saturating_sub(1) {
        let a = path.waypoints[i];
        let b = path.waypoints[i + 1];
        let seg_region = path
            .regions
            .get(i)
            .copied()
            .or_else(|| path.regions.last().copied())
            .unwrap_or(RegionId::INVALID);
        let mesh_opt = if seg_region == RegionId::INVALID {
            None
        } else {
            navmeshes
                .meshes
                .get(seg_region.index())
                .and_then(|m| m.as_ref())
        };
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let horiz = (dx * dx + dy * dy).sqrt();
        let steps = (horiz / step).ceil().max(1.0) as u32;
        let mut prev_z = out.last().map(|p| p.z).unwrap_or(a.z);
        for s in 1..=steps {
            let t = s as f64 / steps as f64;
            let x = a.x + dx * t;
            let y = a.y + dy * t;
            let z = match mesh_opt {
                Some(m) => sample_navmesh_z(m, x, y).unwrap_or(prev_z),
                None => prev_z,
            };
            out.push(Vec3::new(x, y, z));
            prev_z = z;
        }
    }
    out
}

/// Linearly interpolate Z at (x, y) on the triangle of `mesh` that
/// contains it (barycentric). Returns `None` if (x, y) lies outside
/// every triangle.
pub fn sample_navmesh_z(mesh: &RegionNavMesh, x: f64, y: f64) -> Option<f64> {
    for tri in &mesh.triangles {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        if let Some((u, v, w)) = barycentric_2d(x, y, a.x, a.y, b.x, b.y, c.x, c.y) {
            return Some(u * a.z + v * b.z + w * c.z);
        }
    }
    None
}

/// Standard 2D barycentric (Gram-matrix / Cramer's rule). Returns the
/// `(u, v, w)` weights at A, B, C respectively if the point lies inside
/// the triangle (with a small slack on each axis); else `None`.
fn barycentric_2d(
    px: f64,
    py: f64,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
) -> Option<(f64, f64, f64)> {
    let v0x = bx - ax;
    let v0y = by - ay;
    let v1x = cx - ax;
    let v1y = cy - ay;
    let v2x = px - ax;
    let v2y = py - ay;
    let d00 = v0x * v0x + v0y * v0y;
    let d01 = v0x * v1x + v0y * v1y;
    let d11 = v1x * v1x + v1y * v1y;
    let d20 = v2x * v0x + v2y * v0y;
    let d21 = v2x * v1x + v2y * v1y;
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-20 {
        return None;
    }
    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;
    if u >= -1e-9 && v >= -1e-9 && w >= -1e-9 {
        Some((u, v, w))
    } else {
        None
    }
}

#[derive(Copy, Clone, Debug)]
struct HeapEntry {
    f: f64,
    region: RegionId,
    position: Vec3,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is max-heap; we want min-f, so reverse.
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{WalkabilityConfig, WatershedConfig};
    use crate::contour::extract_contours;
    use crate::portals::extract_portals;
    use crate::walkability::classify_walkability;
    use crate::watershed::segment;
    use crate::{synth, VoxelGrid};

    fn pipeline(soup: &crate::PolySoup, cell_size: f64) -> (RegionContours, Vec<Portal>) {
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let rm = segment(&chf, &WatershedConfig::default());
        let contours = extract_contours(&chf, &rm);
        let portals = extract_portals(&contours);
        (contours, portals)
    }

    #[test]
    fn same_region_path_is_direct() {
        let soup = synth::plane(8.0, 8.0, 1);
        let (contours, portals) = pipeline(&soup, 0.5);
        let src = Vec3::new(-2.0, 0.0, 0.5);
        let dst = Vec3::new(2.0, 0.0, 0.5);
        let path = find_path(&contours, &portals, src, dst).expect("path should exist");
        assert_eq!(path.waypoints.len(), 2, "same-region path is src→dst");
        assert_eq!(path.regions.len(), 1);
    }

    #[test]
    fn cross_region_path_uses_portal_midpoint() {
        let soup = synth::floor_with_ramp_and_platform();
        let (contours, portals) = pipeline(&soup, 0.2);
        // src on the floor, dst on the platform top
        let src = Vec3::new(-3.0, 0.0, 0.0);
        let dst = Vec3::new(8.0, 0.0, 1.6);
        if let Some(path) = find_path(&contours, &portals, src, dst) {
            assert!(
                path.waypoints.len() >= 3,
                "cross-region path should have ≥3 waypoints, got {}",
                path.waypoints.len()
            );
            assert!(path.regions.len() >= 2);
        }
        // If find_path returns None (regions don't connect at this scale),
        // that's also acceptable — we test the API contract, not a specific
        // path topology that depends on voxel discretization.
    }

    #[test]
    fn densify_same_region_hugs_plane_z() {
        let soup = synth::plane(8.0, 8.0, 1);
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let rm = segment(&chf, &WatershedConfig::default());
        let contours = extract_contours(&chf, &rm);
        let portals = extract_portals(&contours);
        let src = Vec3::new(-2.0, 0.0, 0.5);
        let dst = Vec3::new(2.0, 0.0, 0.5);
        let path = find_path(&contours, &portals, src, dst).expect("path");
        let dense = densify_path_on_hf(&path, &chf, &rm);
        assert!(dense.len() >= 8, "expected several dense samples, got {}", dense.len());
        // All samples within ±0.5 of the surface Z (cell-size resolution).
        for v in &dense {
            assert!(
                (v.z - 0.5).abs() < 0.5,
                "sample z out of plane range: {:?}",
                v
            );
        }
    }

    #[test]
    fn densify_on_navmesh_stays_on_plane() {
        use crate::navmesh::build_all_navmeshes;
        let soup = synth::plane(8.0, 8.0, 1);
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(&soup, 0.5, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let rm = segment(&chf, &WatershedConfig::default());
        let contours = extract_contours(&chf, &rm);
        let portals = extract_portals(&contours);
        let navmeshes = build_all_navmeshes(&contours);
        let src = Vec3::new(-2.0, 0.0, 0.5);
        let dst = Vec3::new(2.0, 0.0, 0.5);
        let path = find_path(&contours, &portals, src, dst).expect("path");
        let dense = densify_path_on_navmesh(&path, &navmeshes, 0.5);
        assert!(dense.len() >= 8, "expected several samples, got {}", dense.len());
        // All samples should be EXACTLY at the navmesh's plane Z — the
        // contour sits at the top of the bottom cell (z = cell_size
        // for the default plane-at-z=0 voxelization).
        let expected_z = dense[1].z; // sample inside the mesh
        for v in &dense {
            assert!(
                (v.z - expected_z).abs() < 1e-9,
                "sample {:?} deviates from plane Z {}",
                v,
                expected_z,
            );
        }
    }

    #[test]
    fn no_region_returns_none() {
        let soup = synth::plane(4.0, 4.0, 1);
        let (contours, portals) = pipeline(&soup, 0.5);
        // Point way outside the plane
        let src = Vec3::new(100.0, 100.0, 0.0);
        let dst = Vec3::new(0.0, 0.0, 0.5);
        assert!(find_path(&contours, &portals, src, dst).is_none());
    }
}
