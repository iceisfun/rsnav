//! Per-region CDT-based navmesh extraction with per-vertex heights.
//!
//! Take each region's plan-view [`RegionContour`] (outer ring + holes),
//! triangulate it with [`rsnav_triangle`]'s constrained Delaunay, then
//! attach a Z value to each output vertex from the source contour's
//! per-vertex heights. The result is a 3D triangle mesh per region —
//! one walkable surface, ready for rendering or downstream pathing.
//!
//! Since we don't request Steiner-point refinement, the CDT preserves
//! input vertex indices 1:1, so the Z attribution is just a lookup by
//! index. If we ever turn on quality refinement (Triangle's `-q` flag,
//! not currently in `rsnav-triangle`), we'd need to interpolate Z for
//! the new vertices via barycentric coordinates on the containing
//! input triangle.

use crate::contour::{RegionContour, RegionContours};
use crate::RegionId;
use rsnav_common::{Polygon, Vec3, Vertex, Winding};
use rsnav_triangle::{
    carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, Pslg, PslgHole, PslgSegment,
    PslgVertex, VertexSlot,
};

/// One region's CDT'd walkable surface as a 3D triangle mesh.
#[derive(Clone, Debug)]
pub struct RegionNavMesh {
    pub region: RegionId,
    /// Per-vertex 3D position. Z comes from the contour vertex's height.
    pub vertices: Vec<Vec3>,
    /// Triangle vertex indices into `vertices`.
    pub triangles: Vec<[u32; 3]>,
}

impl RegionNavMesh {
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
}

/// CDT outputs per region. Some regions may fail to triangulate
/// (degenerate contour, self-intersecting segments) — those slots are
/// `None`.
#[derive(Clone, Debug)]
pub struct AllRegionNavMeshes {
    pub region_count: u32,
    pub meshes: Vec<Option<RegionNavMesh>>,
}

impl AllRegionNavMeshes {
    pub fn total_triangle_count(&self) -> usize {
        self.meshes
            .iter()
            .filter_map(|m| m.as_ref())
            .map(|m| m.triangle_count())
            .sum()
    }

    pub fn built_count(&self) -> usize {
        self.meshes.iter().filter(|m| m.is_some()).count()
    }
}

/// Triangulate every region's contour. The output slots match
/// [`RegionContours::contours`] by index.
pub fn build_all_navmeshes(contours: &RegionContours) -> AllRegionNavMeshes {
    let meshes = contours
        .contours
        .iter()
        .enumerate()
        .map(|(i, contour)| build_region_navmesh(RegionId(i as u32), contour))
        .collect();
    AllRegionNavMeshes {
        region_count: contours.region_count,
        meshes,
    }
}

/// Triangulate one region. Returns `None` if the contour is degenerate
/// or the CDT pipeline refused the input (e.g., self-intersecting
/// segments — shouldn't happen for our voxel-derived contours).
pub fn build_region_navmesh(rid: RegionId, contour: &RegionContour) -> Option<RegionNavMesh> {
    if contour.outer.len() < 3 {
        return None;
    }

    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    let mut z_per_vertex: Vec<f64> = Vec::new();

    // Outer ring vertices (indices 0..outer_count in the PSLG)
    let outer_count = contour.outer.len() as u32;
    for v in &contour.outer {
        cdt.push_vertex(VertexSlot::new(v.xy, 0));
        pslg.vertices.push(PslgVertex::new(v.xy));
        z_per_vertex.push(v.z);
    }
    for i in 0..outer_count {
        pslg.segments.push(PslgSegment {
            a: i,
            b: (i + 1) % outer_count,
            marker: 1,
        });
    }

    // Hole rings + seed points
    for hole in &contour.holes {
        if hole.len() < 3 {
            continue;
        }
        let hole_start = pslg.vertices.len() as u32;
        for v in hole {
            cdt.push_vertex(VertexSlot::new(v.xy, 0));
            pslg.vertices.push(PslgVertex::new(v.xy));
            z_per_vertex.push(v.z);
        }
        let hole_count = hole.len() as u32;
        for i in 0..hole_count {
            pslg.segments.push(PslgSegment {
                a: hole_start + i,
                b: hole_start + (i + 1) % hole_count,
                marker: 2,
            });
        }
        // carve_holes wants a seed POINT inside the hole. For a concave
        // hole the centroid can fall outside, so use ensure_winding then
        // interior_point.
        let xy: Vec<Vertex> = hole.iter().map(|v| v.xy).collect();
        let mut poly = Polygon { vertices: xy };
        poly.ensure_winding(Winding::CounterClockwise);
        if let Some(seed) = poly.interior_point() {
            pslg.holes.push(PslgHole { point: seed });
        }
    }

    // Run the CDT pipeline.
    delaunay(&mut cdt, DivConqOptions::default());
    if form_skeleton(&mut cdt, &pslg, None).is_err() {
        return None;
    }
    carve_holes(&mut cdt, &pslg, /* convex_outer */ false);

    // Map every CDT vertex to a 3D position. Without Steiner-point
    // refinement, CDT vertex indices equal PSLG indices, so Z by index
    // is correct.
    let mut vertices = Vec::with_capacity(cdt.vertices.len());
    let fallback_z = z_per_vertex.first().copied().unwrap_or(0.0);
    for (i, slot) in cdt.vertices.iter().enumerate() {
        let z = z_per_vertex.get(i).copied().unwrap_or(fallback_z);
        vertices.push(Vec3::new(slot.position.x, slot.position.y, z));
    }

    // Extract live triangles, skipping the dummy/ghost ones.
    let mut triangles = Vec::with_capacity(cdt.live_triangle_count());
    for tri in &cdt.triangles {
        if tri.is_dead() {
            continue;
        }
        let v0 = tri.vertices[0].0;
        let v1 = tri.vertices[1].0;
        let v2 = tri.vertices[2].0;
        if v0 == u32::MAX || v1 == u32::MAX || v2 == u32::MAX {
            continue;
        }
        triangles.push([v0, v1, v2]);
    }

    if triangles.is_empty() {
        return None;
    }

    Some(RegionNavMesh {
        region: rid,
        vertices,
        triangles,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{WalkabilityConfig, WatershedConfig};
    use crate::contour::extract_contours;
    use crate::walkability::classify_walkability;
    use crate::watershed::segment;
    use crate::{synth, VoxelGrid};

    fn full_pipeline_navmeshes(soup: &crate::PolySoup, cell_size: f64) -> AllRegionNavMeshes {
        let walk = WalkabilityConfig::default();
        let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, walk.cos_max_slope());
        let chf = classify_walkability(&grid, &walk);
        let ws = WatershedConfig::default();
        let rm = segment(&chf, &ws);
        let contours = extract_contours(&chf, &rm);
        build_all_navmeshes(&contours)
    }

    #[test]
    fn flat_plane_navmesh_is_planar() {
        let soup = synth::plane(4.0, 4.0, 1);
        let result = full_pipeline_navmeshes(&soup, 1.0);
        assert!(result.built_count() >= 1);
        let mesh = result.meshes[0].as_ref().expect("region 0 should triangulate");
        assert!(mesh.triangle_count() > 0);
        // All vertices should be at z=1.0 (the top of layer 1, which holds the floor at z=0).
        for v in &mesh.vertices {
            assert!(
                (v.z - 1.0).abs() < 1e-6,
                "plane vertex Z {} not at expected 1.0",
                v.z
            );
        }
    }

    #[test]
    fn ramp_navmesh_has_varying_z() {
        // A 4 m run × 2 m rise ramp at 0.2 m voxels gives ~20 cells along
        // the rise with one per layer — enough that *some* region's
        // navmesh spans multiple Z layers.
        let soup = synth::ramp(0.0, 4.0, 2.0, 3.0);
        let result = full_pipeline_navmeshes(&soup, 0.2);
        // The ramp is one tilted walkable surface, but the layer-per-column
        // constraint can split it into multiple layer "strands". The
        // largest strand (most triangles) should still span the slope.
        let largest = result
            .meshes
            .iter()
            .flatten()
            .max_by_key(|m| m.triangle_count())
            .expect("at least one ramp region must triangulate");
        let min_z = largest.vertices.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);
        let max_z = largest
            .vertices
            .iter()
            .map(|v| v.z)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_z - min_z > 0.4,
            "largest ramp region should span >0.4 m in Z, got {}..{} across {} regions; \
             per-region triangle counts: {:?}",
            min_z,
            max_z,
            result.built_count(),
            result.meshes.iter().filter_map(|m| m.as_ref().map(|m| m.triangle_count())).collect::<Vec<_>>(),
        );
    }

    #[test]
    fn under_ramp_produces_multiple_navmeshes() {
        let mut soup = synth::plane(10.0, 4.0, 1);
        let ramp = synth::ramp(0.0, 4.0, 3.0, 4.0);
        let base = soup.vertices.len() as u32;
        soup.vertices.extend(ramp.vertices.iter().copied());
        for t in &ramp.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let result = full_pipeline_navmeshes(&soup, 0.2);
        assert!(
            result.built_count() >= 2,
            "expected ≥2 region navmeshes, got {}",
            result.built_count()
        );
    }

    #[test]
    fn navmesh_triangle_indices_are_valid() {
        let soup = synth::floor_with_ramp_and_platform();
        let result = full_pipeline_navmeshes(&soup, 0.2);
        for m in result.meshes.iter().flatten() {
            for tri in &m.triangles {
                for &idx in tri {
                    assert!(
                        (idx as usize) < m.vertices.len(),
                        "triangle index {} >= vertex count {}",
                        idx,
                        m.vertices.len()
                    );
                }
            }
        }
    }
}
