//! Convert a freshly-carved [`CdtMesh`] into a flat, query-friendly
//! [`NavMesh`].

use std::collections::VecDeque;

use rsnav_common::{Aabb, TriangleId, VertexId};
use rsnav_triangle::mesh::{CdtMesh, Otri, DUMMY_SUB, DUMMY_TRI};

use crate::navmesh::{NavMesh, NavTriangle};

/// Walk a CDT mesh post-`carve_holes` and produce a compact [`NavMesh`].
///
/// - Dead and ghost triangles are skipped, then the surviving slots are
///   renumbered into a contiguous `0..n` index space.
/// - Vertices are likewise renumbered: only vertices referenced by at
///   least one surviving triangle make it into the output.
/// - Edge markers come from the subsegs glued to each triangle edge.
/// - Region IDs label connected components under "two triangles connect if
///   their shared edge isn't a constrained subseg".
pub fn build_from_cdt(cdt: &CdtMesh) -> NavMesh {
    // --- Decide which CDT triangles survive into the navmesh. ---
    //
    // A "live" navmesh triangle must (a) not be dead, (b) not be a ghost
    // (no INVALID corners), and (c) be reachable through the bonded
    // neighbor graph from the dummy's stored start hull triangle.
    // In practice, post-carveholes the dead/ghost filter is sufficient —
    // every surviving triangle is a real walkable face.
    let mut cdt_to_nav: Vec<u32> = vec![u32::MAX; cdt.triangles.len()];
    let mut live_cdt_indices: Vec<u32> = Vec::new();
    for (i, slot) in cdt.triangles.iter().enumerate().skip(1) {
        if slot.is_dead() {
            continue;
        }
        if !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        cdt_to_nav[i] = live_cdt_indices.len() as u32;
        live_cdt_indices.push(i as u32);
    }

    // --- Renumber vertices. ---
    let mut cdt_vert_to_nav: Vec<u32> = vec![u32::MAX; cdt.vertices.len()];
    let mut vertices: Vec<rsnav_common::Vertex> = Vec::new();
    for &cdt_idx in &live_cdt_indices {
        let slot = &cdt.triangles[cdt_idx as usize];
        for &v in &slot.vertices {
            let vi = v.index();
            if cdt_vert_to_nav[vi] == u32::MAX {
                cdt_vert_to_nav[vi] = vertices.len() as u32;
                vertices.push(cdt.vertices[vi].position);
            }
        }
    }

    // --- Build NavTriangles (without adjacency/region for now). ---
    let mut triangles: Vec<NavTriangle> = Vec::with_capacity(live_cdt_indices.len());
    for &cdt_idx in &live_cdt_indices {
        let slot = &cdt.triangles[cdt_idx as usize];
        let v: [VertexId; 3] = [
            VertexId::new(cdt_vert_to_nav[slot.vertices[0].index()]),
            VertexId::new(cdt_vert_to_nav[slot.vertices[1].index()]),
            VertexId::new(cdt_vert_to_nav[slot.vertices[2].index()]),
        ];

        // Map CDT subseg slots to per-edge constraint markers. The CDT
        // stores subsegs indexed by triangle orient, where orient `i` is
        // the edge whose apex is `vertices[i]`. That maps directly to our
        // "edge i = edge opposite vertices[i]" convention.
        let mut edge_markers = [0i32; 3];
        for orient in 0..3usize {
            let enc_sub = slot.subsegs[orient];
            if enc_sub.sub() != DUMMY_SUB {
                let s = &cdt.subsegs[enc_sub.sub() as usize];
                if !s.is_dead() {
                    edge_markers[orient] = s.marker;
                }
            }
        }

        // Geometry-derived metadata: area and centroid.
        let p0 = vertices[v[0].index()];
        let p1 = vertices[v[1].index()];
        let p2 = vertices[v[2].index()];
        let area2 =
            (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
        let area = 0.5 * area2.abs();
        let centroid = rsnav_common::Vertex::new(
            (p0.x + p1.x + p2.x) / 3.0,
            (p0.y + p1.y + p2.y) / 3.0,
        );

        triangles.push(NavTriangle {
            vertices: v,
            neighbors: [TriangleId::INVALID; 3],
            edge_markers,
            area,
            centroid,
            region: u32::MAX, // filled in below
        });
    }

    // --- Adjacency: walk CDT neighbors and remap to nav-triangle IDs. ---
    for (nav_idx, &cdt_idx) in live_cdt_indices.iter().enumerate() {
        let slot = &cdt.triangles[cdt_idx as usize];
        for orient in 0..3usize {
            let enc = slot.neighbors[orient];
            let neighbor_cdt_tri = enc.tri();
            let mapped = if neighbor_cdt_tri == DUMMY_TRI {
                u32::MAX
            } else {
                cdt_to_nav[neighbor_cdt_tri as usize]
            };
            // The neighbor encoding stores which of the neighbor's edges
            // faces us. We don't store the orient explicitly — callers
            // recover it by searching the neighbor's 3 neighbors if they
            // need to. To keep things robust, we *do* check the neighbor
            // is alive in our nav layer; CDT could in principle have a
            // bond to a triangle we filtered out (ghost), in which case we
            // treat it as a boundary.
            triangles[nav_idx].neighbors[orient] = if mapped == u32::MAX {
                TriangleId::INVALID
            } else {
                TriangleId::new(mapped)
            };
        }
    }

    // --- Connected-component (region) labelling. ---
    let n = triangles.len();
    let mut region_id = vec![u32::MAX; n];
    let mut region_count: u32 = 0;
    let mut queue: VecDeque<u32> = VecDeque::new();
    for seed in 0..n as u32 {
        if region_id[seed as usize] != u32::MAX {
            continue;
        }
        let me = region_count;
        region_count += 1;
        region_id[seed as usize] = me;
        queue.clear();
        queue.push_back(seed);
        while let Some(t) = queue.pop_front() {
            let tri = &triangles[t as usize];
            for edge in 0..3 {
                if tri.edge_markers[edge] != 0 {
                    continue; // constrained — region boundary
                }
                let n_tri = tri.neighbors[edge];
                if !n_tri.is_valid() {
                    continue;
                }
                let n_idx = n_tri.index();
                if region_id[n_idx] == u32::MAX {
                    region_id[n_idx] = me;
                    queue.push_back(n_tri.get());
                }
            }
        }
    }
    for (t, &r) in triangles.iter_mut().zip(region_id.iter()) {
        t.region = r;
    }

    // --- AABB. ---
    let aabb = Aabb::from_points(vertices.iter().copied());

    NavMesh {
        vertices,
        triangles,
        aabb,
        region_count,
    }
}

// Silence the unused-import warning when `Otri` doesn't get referenced
// (it's used only conceptually here — adjacency is derived from the raw
// neighbor encoding, not via Otri navigation).
#[allow(dead_code)]
fn _ensure_otri_alive(_: Otri) {}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_common::Vertex;
    use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
    use rsnav_triangle::{carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

    fn build_cdt_with_hole() -> CdtMesh {
        // 4x4 outer with a 1x1 square hole — same as the polygon-extract /
        // hole-carving tests. Eight surviving triangles.
        let pts = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 4.0),
            (0.0, 4.0),
            (1.5, 1.5),
            (2.5, 1.5),
            (2.5, 2.5),
            (1.5, 2.5),
        ];
        let mut mesh = CdtMesh::new();
        for (x, y) in pts {
            mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let pslg = Pslg {
            vertices: pts
                .iter()
                .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
                .collect(),
            segments: vec![
                PslgSegment { a: 0, b: 1, marker: 10 },
                PslgSegment { a: 1, b: 2, marker: 10 },
                PslgSegment { a: 2, b: 3, marker: 10 },
                PslgSegment { a: 3, b: 0, marker: 10 },
                PslgSegment { a: 4, b: 5, marker: 20 },
                PslgSegment { a: 5, b: 6, marker: 20 },
                PslgSegment { a: 6, b: 7, marker: 20 },
                PslgSegment { a: 7, b: 4, marker: 20 },
            ],
            holes: vec![PslgHole {
                point: Vertex::new(2.0, 2.0),
            }],
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        carve_holes(&mut mesh, &pslg, false);
        mesh
    }

    #[test]
    fn square_with_hole_has_one_region_and_eight_triangles() {
        let cdt = build_cdt_with_hole();
        let nav = build_from_cdt(&cdt);
        assert_eq!(nav.triangle_count(), 8);
        assert_eq!(nav.region_count, 1);
        // The annulus is simply connected — all triangles share one region.
        for t in &nav.triangles {
            assert_eq!(t.region, 0);
        }
    }

    #[test]
    fn neighbors_are_symmetric() {
        let cdt = build_cdt_with_hole();
        let nav = build_from_cdt(&cdt);
        for (i, tri) in nav.triangles.iter().enumerate() {
            for edge in 0..3 {
                let neigh = tri.neighbors[edge];
                if !neigh.is_valid() {
                    continue;
                }
                let n_tri = nav.triangle(neigh);
                let back_ref = n_tri
                    .neighbors
                    .iter()
                    .any(|n| n.is_valid() && n.index() == i);
                assert!(
                    back_ref,
                    "triangle {} has neighbor {} but {} doesn't reference {} back",
                    i,
                    neigh.get(),
                    neigh.get(),
                    i
                );
            }
        }
    }

    #[test]
    fn constrained_edges_are_marked() {
        let cdt = build_cdt_with_hole();
        let nav = build_from_cdt(&cdt);
        // After carving, every boundary edge of the surviving mesh either
        // sits between two surviving triangles (must be unmarked) or faces
        // outside (could be marked from a PSLG segment).
        let mut total_constrained = 0usize;
        for tri in &nav.triangles {
            for edge in 0..3 {
                if tri.is_edge_constrained(edge) {
                    total_constrained += 1;
                }
            }
        }
        // 4 outer + 4 inner = 8 PSLG segments. Each appears on at most one
        // surviving triangle (since the other side is gone), so we expect
        // at least 8 constrained edges across the mesh.
        assert!(
            total_constrained >= 8,
            "expected >= 8 constrained edges, saw {}",
            total_constrained
        );
    }

    #[test]
    fn two_disconnected_rooms_get_distinct_regions() {
        // Build a CDT where the only path between two halves is blocked by
        // a constrained "wall" segment. Region IDs should differ across
        // the wall.
        let mut mesh = CdtMesh::new();
        let pts = [
            (0.0, 0.0),  // 0
            (10.0, 0.0), // 1
            (10.0, 4.0), // 2
            (0.0, 4.0),  // 3
            (5.0, 0.0),  // 4 — wall bottom
            (5.0, 4.0),  // 5 — wall top
        ];
        for (x, y) in pts {
            mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let pslg = Pslg {
            vertices: pts
                .iter()
                .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
                .collect(),
            segments: vec![
                PslgSegment { a: 0, b: 4, marker: 10 },
                PslgSegment { a: 4, b: 1, marker: 10 },
                PslgSegment { a: 1, b: 2, marker: 10 },
                PslgSegment { a: 2, b: 5, marker: 10 },
                PslgSegment { a: 5, b: 3, marker: 10 },
                PslgSegment { a: 3, b: 0, marker: 10 },
                PslgSegment { a: 4, b: 5, marker: 99 }, // wall
            ],
            holes: Vec::new(),
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        let nav = build_from_cdt(&mesh);
        assert_eq!(
            nav.region_count, 2,
            "the wall should split the mesh into 2 regions"
        );
        // Sanity: triangles with centroid x < 5 are in one region;
        // triangles with x > 5 are in the other.
        let mut left_region: Option<u32> = None;
        let mut right_region: Option<u32> = None;
        for t in &nav.triangles {
            if t.centroid.x < 5.0 {
                left_region = Some(t.region);
            } else if t.centroid.x > 5.0 {
                right_region = Some(t.region);
            }
        }
        assert_ne!(left_region, right_region);
        assert!(left_region.is_some() && right_region.is_some());
    }
}
