//! Hole carving: remove the triangles inside marked holes (and the
//! unbounded exterior concavities) from a constrained Delaunay
//! triangulation.
//!
//! Counterparts to triangle.c's `infecthull`, `plague`, and the hole-handling
//! portion of `carveholes`. Regional attribute spreading
//! (`regionplague`) and area constraints are intentionally not implemented in
//! v1 — we just identify hole/concavity triangles and delete them.
//!
//! ## Algorithm
//!
//! 1. **Infect hull** (concavities): walk the convex hull. Any hull
//!    triangle whose hull edge is not protected by a constrained subseg
//!    is marked "infected" — it sits outside the user's PSLG boundary.
//! 2. **Seed each hole**: for each hole point in the PSLG, find the
//!    triangle containing it and mark it infected. With a single hole
//!    this is a linear scan over live triangles; with many holes the
//!    points are bucketed into a uniform grid and all located in one
//!    ascending triangle pass (same winner per hole, without the
//!    O(holes × triangles) blowup).
//! 3. **Plague (BFS)**: starting from infected triangles, spread
//!    infection to neighbors that aren't separated by a constrained
//!    subseg.
//! 4. **Sweep**: delete every infected triangle. Boundary edges of the
//!    surviving mesh get re-bonded to the dummy.

use std::collections::VecDeque;

use rsnav_common::{Triangle, Vertex, VertexId};

use crate::mesh::{CdtMesh, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::pslg::Pslg;

/// Run the full hole-carving pipeline against `mesh`.
///
/// `pslg.holes` provides the seed points. `convex = true` skips the
/// hull-concavity step (the input is assumed to triangulate to a convex
/// region — useful when you've explicitly added segments that form the
/// outer boundary).
///
/// Returns the number of triangles deleted.
pub fn carve_holes(mesh: &mut CdtMesh, pslg: &Pslg, convex: bool) -> usize {
    let mut infected: Vec<bool> = vec![false; mesh.triangles.len()];
    // Worklist of infected-but-not-yet-spread triangles.
    let mut worklist: VecDeque<u32> = VecDeque::new();

    if !convex {
        infect_hull(mesh, &mut infected, &mut worklist);
    }
    seed_holes(mesh, pslg, &mut infected, &mut worklist);
    plague(mesh, &mut infected, &mut worklist);
    sweep(mesh, &infected)
}

// --- infect_hull ---------------------------------------------------------

/// Walk the convex hull (via the dummy triangle's `neighbors[0]` start
/// pointer set by `remove_ghosts`). Infect every hull triangle that
/// doesn't have a subseg protecting its hull edge.
fn infect_hull(mesh: &mut CdtMesh, infected: &mut [bool], worklist: &mut VecDeque<u32>) {
    let start = mesh.sym(Otri::new(DUMMY_TRI, 0));
    if start.tri == DUMMY_TRI {
        return; // empty / degenerate
    }
    let mut hulltri = start;
    loop {
        if !infected[hulltri.tri as usize] {
            let hullsub = mesh.tspivot(hulltri);
            if hullsub.sub == DUMMY_SUB {
                infected[hulltri.tri as usize] = true;
                worklist.push_back(hulltri.tri);
            }
        }
        // Step CCW around the hull: lnext, then walk inward via oprev
        // (sym then lnext) until back at the boundary.
        hulltri = hulltri.lnext();
        let mut nexttri = oprev(mesh, hulltri);
        while nexttri.tri != DUMMY_TRI {
            hulltri = nexttri;
            nexttri = oprev(mesh, hulltri);
        }
        if hulltri == start {
            break;
        }
    }
}

// --- seed_holes ----------------------------------------------------------

fn seed_holes(
    mesh: &CdtMesh,
    pslg: &Pslg,
    infected: &mut [bool],
    worklist: &mut VecDeque<u32>,
) {
    let positions = mesh.vertices_positions();
    // Per-hole seed triangle, in `pslg.holes` order.
    let seeds: Vec<Option<u32>> = if pslg.holes.len() <= 1 {
        pslg.holes
            .iter()
            .map(|hole| locate_triangle(mesh, &positions, hole.point))
            .collect()
    } else {
        locate_holes_gridded(mesh, &positions, pslg)
    };
    // Infect in original hole order so the plague worklist ends up
    // identical to a per-hole locate_triangle pass, regardless of the
    // order the gridded pass discovered the seeds in. Holes that located
    // outside the mesh produced a `None` seed and are silently skipped,
    // matching triangle.c.
    for tri in seeds.into_iter().flatten() {
        if !infected[tri as usize] {
            infected[tri as usize] = true;
            worklist.push_back(tri);
        }
    }
}

/// Linear-scan point-in-triangle search. Returns the index of the first
/// live triangle that contains `pt` (boundary inclusive). Slow but simple
/// — fine for a handful of hole seeds; build a BSP first if you need to
/// hot-loop this on large meshes.
fn locate_triangle(mesh: &CdtMesh, positions: &[Vertex], pt: Vertex) -> Option<u32> {
    for tri_idx in 1..mesh.triangles.len() as u32 {
        let slot = mesh.triangle(tri_idx);
        if slot.is_dead() {
            continue;
        }
        // Skip ghosts (any vertex INVALID).
        if !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        let t = Triangle::new(slot.vertices[0], slot.vertices[1], slot.vertices[2]);
        if t.contains(positions, pt) {
            return Some(tri_idx);
        }
    }
    None
}

/// Batched point-in-triangle search for many holes at once. Buckets the
/// hole points into a uniform grid (~sqrt(holes) cells per axis) over
/// their bounding box, then makes one ascending pass over live, non-ghost
/// triangles, testing each against only the not-yet-located holes in the
/// grid cells its bounding box overlaps. Because the pass is ascending
/// and a hole keeps its first hit, each hole gets exactly the triangle
/// `locate_triangle` would return — but in O(triangles + holes) instead
/// of O(holes × triangles).
fn locate_holes_gridded(mesh: &CdtMesh, positions: &[Vertex], pslg: &Pslg) -> Vec<Option<u32>> {
    let holes = &pslg.holes;
    let mut seeds: Vec<Option<u32>> = vec![None; holes.len()];

    // Non-finite points can't be bucketed (NaN falls out of every cell
    // range, infinities poison the box), but `Triangle::contains` can
    // still answer for them — route them through the linear scan so they
    // seed exactly what a per-hole locate_triangle pass would.
    for (i, hole) in holes.iter().enumerate() {
        if !(hole.point.x.is_finite() && hole.point.y.is_finite()) {
            seeds[i] = locate_triangle(mesh, positions, hole.point);
        }
    }

    // Bounding box of the finite hole points.
    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for hole in holes.iter() {
        if !(hole.point.x.is_finite() && hole.point.y.is_finite()) {
            continue;
        }
        min_x = min_x.min(hole.point.x);
        min_y = min_y.min(hole.point.y);
        max_x = max_x.max(hole.point.x);
        max_y = max_y.max(hole.point.y);
    }
    if min_x > max_x {
        return seeds; // no finite holes
    }

    // Grid resolution: about sqrt(holes) cells per axis. Cell sizes are
    // clamped to >= 1.0 so zero-extent boxes (coincident holes, or all
    // holes on one row/column) still index cleanly.
    let n = (holes.len() as f64).sqrt().ceil() as usize;
    let cell_w = ((max_x - min_x) / n as f64).max(1.0);
    let cell_h = ((max_y - min_y) / n as f64).max(1.0);
    let cell_x = |x: f64| ((x - min_x) / cell_w).floor().clamp(0.0, (n - 1) as f64) as usize;
    let cell_y = |y: f64| ((y - min_y) / cell_h).floor().clamp(0.0, (n - 1) as f64) as usize;

    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); n * n];
    let mut remaining = 0usize;
    for (hole_idx, hole) in holes.iter().enumerate() {
        if !(hole.point.x.is_finite() && hole.point.y.is_finite()) {
            continue; // already seeded via the linear scan above
        }
        buckets[cell_y(hole.point.y) * n + cell_x(hole.point.x)].push(hole_idx as u32);
        remaining += 1;
    }

    // One pass over the triangles, same liveness/ghost filters as
    // locate_triangle.
    for tri_idx in 1..mesh.triangles.len() as u32 {
        if remaining == 0 {
            break; // every hole located
        }
        let slot = mesh.triangle(tri_idx);
        if slot.is_dead() {
            continue;
        }
        // Skip ghosts (any vertex INVALID).
        if !slot.vertices.iter().all(|v| v.is_valid()) {
            continue;
        }
        let t = Triangle::new(slot.vertices[0], slot.vertices[1], slot.vertices[2]);
        let [a, b, c] = t.positions(positions);
        let tx0 = a.x.min(b.x).min(c.x);
        let tx1 = a.x.max(b.x).max(c.x);
        let ty0 = a.y.min(b.y).min(c.y);
        let ty1 = a.y.max(b.y).max(c.y);
        // Triangle can't contain a hole point if the boxes don't touch.
        if tx1 < min_x || tx0 > max_x || ty1 < min_y || ty0 > max_y {
            continue;
        }
        for iy in cell_y(ty0)..=cell_y(ty1) {
            for ix in cell_x(tx0)..=cell_x(tx1) {
                for &hole_idx in &buckets[iy * n + ix] {
                    let seed = &mut seeds[hole_idx as usize];
                    if seed.is_none() && t.contains(positions, holes[hole_idx as usize].point) {
                        *seed = Some(tri_idx);
                        remaining -= 1;
                    }
                }
            }
        }
    }
    seeds
}

// --- plague --------------------------------------------------------------

/// BFS through infected triangles, spreading infection to neighbors that
/// aren't separated by a constrained subseg. Mirrors `plague()` in C
/// (which uses a worklist and the in-place infect bit; we use a parallel
/// `Vec<bool>` for cleaner Rust).
fn plague(mesh: &mut CdtMesh, infected: &mut [bool], worklist: &mut VecDeque<u32>) {
    while let Some(tri_idx) = worklist.pop_front() {
        for orient in 0..3u8 {
            let here = Otri::new(tri_idx, orient);
            let neighbor = mesh.sym(here);
            let sub = mesh.tspivot(here);

            if neighbor.tri == DUMMY_TRI || infected[neighbor.tri as usize] {
                // Subseg between two dying triangles — kill the subseg.
                if sub.sub != DUMMY_SUB {
                    mesh.kill_subseg(sub.sub);
                    if neighbor.tri != DUMMY_TRI {
                        // Make sure we don't double-free the subseg when
                        // we process the neighbor: detach it.
                        mesh.ts_dissolve(neighbor);
                    }
                }
            } else {
                // Neighbor will survive.
                if sub.sub == DUMMY_SUB {
                    // No subseg protecting it — neighbor becomes infected.
                    infected[neighbor.tri as usize] = true;
                    worklist.push_back(neighbor.tri);
                } else {
                    // Subseg protects the neighbor — the subseg becomes a
                    // boundary edge of the surviving mesh.
                    mesh.st_dissolve(sub);
                    if mesh.subseg(sub.sub).marker == 0 {
                        mesh.subseg_mut(sub.sub).marker = 1;
                    }
                    // Promote any unmarked boundary vertices to marker 1.
                    let norg = mesh.org(neighbor);
                    let ndest = mesh.dest(neighbor);
                    if norg.is_valid() && mesh.vertex(norg).marker == 0 {
                        mesh.vertex_mut(norg).marker = 1;
                    }
                    if ndest.is_valid() && mesh.vertex(ndest).marker == 0 {
                        mesh.vertex_mut(ndest).marker = 1;
                    }
                }
            }
        }
    }
}

// --- sweep ---------------------------------------------------------------

/// Detach every infected triangle from its still-alive neighbors and free
/// its slot. Returns the number of triangles killed.
///
/// Shared with the winding-based cull (`carve_by_winding`), which builds
/// its infected set by classification instead of plague BFS.
pub(crate) fn sweep(mesh: &mut CdtMesh, infected: &[bool]) -> usize {
    let mut killed = 0usize;
    for tri_idx in 1..mesh.triangles.len() as u32 {
        if !infected[tri_idx as usize] || mesh.triangle(tri_idx).is_dead() {
            continue;
        }
        // Disconnect from every surviving neighbor.
        for orient in 0..3u8 {
            let here = Otri::new(tri_idx, orient);
            let neighbor = mesh.sym(here);
            if neighbor.tri != DUMMY_TRI && !infected[neighbor.tri as usize] {
                // Neighbor survives — it should now think it has no neighbor
                // on this edge.
                mesh.dissolve(neighbor);
            }
        }
        mesh.kill_triangle(tri_idx);
        killed += 1;
    }
    killed
}

// --- helpers ------------------------------------------------------------

#[inline]
fn oprev(mesh: &CdtMesh, o: Otri) -> Otri {
    mesh.sym(o).lnext()
}

// Tiny extension on CdtMesh to expose the vertex *position* slice for
// reuse with rsnav_common's Triangle helper.
trait MeshPositions {
    fn vertices_positions(&self) -> Vec<Vertex>;
}

impl MeshPositions for CdtMesh {
    fn vertices_positions(&self) -> Vec<Vertex> {
        self.vertices.iter().map(|v| v.position).collect()
    }
}

#[allow(dead_code)]
fn _silence_vertex_id_warning(_: VertexId) {}

// --- Test fixtures (shared with winding.rs tests) ------------------------

#[cfg(test)]
pub(crate) mod testfix {
    use super::*;
    use crate::divconq::{delaunay, DivConqOptions};
    use crate::mesh::VertexSlot;
    use crate::pslg::{PslgHole, PslgSegment, PslgVertex};
    use crate::segment::form_skeleton;

    pub(crate) fn push(m: &mut CdtMesh, x: f64, y: f64) -> VertexId {
        m.push_vertex(VertexSlot::new(Vertex::new(x, y), 0))
    }

    /// 4x4 square (4 outer verts) with a 1x1 hole in the middle bounded by
    /// 4 inner verts. After carve, only the 8 triangles between the rings
    /// should survive.
    pub(crate) fn build_square_with_square_hole() -> (CdtMesh, Pslg) {
        let mut mesh = CdtMesh::new();
        // outer
        push(&mut mesh, 0.0, 0.0);
        push(&mut mesh, 4.0, 0.0);
        push(&mut mesh, 4.0, 4.0);
        push(&mut mesh, 0.0, 4.0);
        // inner
        push(&mut mesh, 1.5, 1.5);
        push(&mut mesh, 2.5, 1.5);
        push(&mut mesh, 2.5, 2.5);
        push(&mut mesh, 1.5, 2.5);

        delaunay(&mut mesh, DivConqOptions::default());

        let pslg = Pslg {
            vertices: (0..8)
                .map(|i| PslgVertex::new(mesh.vertex_pos(VertexId::new(i))))
                .collect(),
            segments: vec![
                // outer ring (CCW)
                PslgSegment { a: 0, b: 1, marker: 10 },
                PslgSegment { a: 1, b: 2, marker: 10 },
                PslgSegment { a: 2, b: 3, marker: 10 },
                PslgSegment { a: 3, b: 0, marker: 10 },
                // inner ring (CW relative to its hole; this is the hole boundary)
                PslgSegment { a: 4, b: 5, marker: 20 },
                PslgSegment { a: 5, b: 6, marker: 20 },
                PslgSegment { a: 6, b: 7, marker: 20 },
                PslgSegment { a: 7, b: 4, marker: 20 },
            ],
            holes: vec![PslgHole {
                point: Vertex::new(2.0, 2.0), // inside the inner ring
            }],
        };

        // Note: form_skeleton with mark_hull=None — calling mark_hull
        // *before* carve_holes would protect every convex-hull edge and
        // stop infect_hull from carving concavities. For a square PSLG
        // the convex hull == PSLG boundary so it happens to work either
        // way, but we follow triangle.c's order (skeleton, carve, then
        // optional markhull) consistently.
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        (mesh, pslg)
    }
}

// --- Tests --------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::testfix::{build_square_with_square_hole, push};
    use super::*;
    use crate::divconq::{delaunay, DivConqOptions};
    use crate::mesh::VertexSlot;
    use crate::pslg::{PslgHole, PslgSegment, PslgVertex};
    use crate::segment::form_skeleton;

    #[test]
    fn carve_a_square_hole() {
        let (mut mesh, pslg) = build_square_with_square_hole();
        let _before = mesh.live_triangle_count();
        let killed = carve_holes(&mut mesh, &pslg, false);
        let after = mesh.live_triangle_count();

        // 8 vertices, hull of 4 (outer corners), with a 1x1 hole bounded by
        // 4 inner corners. The walkable region between outer and inner ring
        // triangulates to exactly 8 triangles.
        assert_eq!(after, 8, "expected 8 surviving triangles after carve");

        // The hole interior (2.0, 2.0) should NOT be contained in any
        // surviving triangle.
        let positions: Vec<Vertex> = mesh.vertices.iter().map(|v| v.position).collect();
        for tri_idx in 1..mesh.triangles.len() as u32 {
            let slot = mesh.triangle(tri_idx);
            if slot.is_dead() {
                continue;
            }
            if !slot.vertices.iter().all(|v| v.is_valid()) {
                continue;
            }
            let t = Triangle::new(slot.vertices[0], slot.vertices[1], slot.vertices[2]);
            assert!(
                !t.contains(&positions, Vertex::new(2.0, 2.0)),
                "triangle {} still covers the hole interior",
                tri_idx
            );
        }

        assert!(killed > 0);
    }

    /// A.poly: 29 vertices forming an outer polygon with one square hole at
    /// (0.47, -0.5). Triangle's reference output is 29 triangles. Our pipe
    /// (D&C + form_skeleton + carve_holes) should produce 29 triangles too.
    #[test]
    fn carve_a_poly() {
        use std::path::Path;
        let pslg = crate::io::read_poly(Path::new("../../../triangle/A.poly")).unwrap();
        let mut mesh = CdtMesh::new();
        for v in &pslg.vertices {
            mesh.push_vertex(VertexSlot::new(v.position, v.marker));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        carve_holes(&mut mesh, &pslg, false);
        assert_eq!(
            mesh.live_triangle_count(),
            29,
            "A.poly should produce 29 triangles to match triangle.c's reference output"
        );
    }

    /// Large square with an `n x n` grid of 1x1 square holes on a 4-unit
    /// pitch — the many-holes analogue of the single-hole fixture above,
    /// big enough to exercise the gridded seed path.
    fn build_square_with_hole_grid(n: usize) -> (CdtMesh, Pslg) {
        let side = n as f64 * 4.0;
        let mut mesh = CdtMesh::new();
        // outer
        push(&mut mesh, 0.0, 0.0);
        push(&mut mesh, side, 0.0);
        push(&mut mesh, side, side);
        push(&mut mesh, 0.0, side);
        let mut segments = vec![
            PslgSegment { a: 0, b: 1, marker: 10 },
            PslgSegment { a: 1, b: 2, marker: 10 },
            PslgSegment { a: 2, b: 3, marker: 10 },
            PslgSegment { a: 3, b: 0, marker: 10 },
        ];
        let mut holes = Vec::new();
        for gy in 0..n {
            for gx in 0..n {
                let (cx, cy) = (gx as f64 * 4.0 + 2.0, gy as f64 * 4.0 + 2.0);
                let base = (4 + (gy * n + gx) * 4) as u32;
                push(&mut mesh, cx - 0.5, cy - 0.5);
                push(&mut mesh, cx + 0.5, cy - 0.5);
                push(&mut mesh, cx + 0.5, cy + 0.5);
                push(&mut mesh, cx - 0.5, cy + 0.5);
                for k in 0..4u32 {
                    segments.push(PslgSegment {
                        a: base + k,
                        b: base + (k + 1) % 4,
                        marker: 20,
                    });
                }
                holes.push(PslgHole {
                    point: Vertex::new(cx, cy),
                });
            }
        }

        delaunay(&mut mesh, DivConqOptions::default());

        let pslg = Pslg {
            vertices: (0..mesh.vertices.len() as u32)
                .map(|i| PslgVertex::new(mesh.vertex_pos(VertexId::new(i))))
                .collect(),
            segments,
            holes,
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        (mesh, pslg)
    }

    /// Cross-check the gridded seed path against a per-hole
    /// locate_triangle oracle: identical seed triangle for every hole,
    /// and an identical surviving triangle set after the full carve.
    #[test]
    fn gridded_seeds_match_locate_triangle_oracle() {
        // Seed triangles: gridded batch vs the linear scan.
        let (mesh, pslg) = build_square_with_hole_grid(5);
        assert_eq!(pslg.holes.len(), 25);
        let positions = mesh.vertices_positions();
        let gridded = locate_holes_gridded(&mesh, &positions, &pslg);
        for (i, hole) in pslg.holes.iter().enumerate() {
            let oracle = locate_triangle(&mesh, &positions, hole.point);
            assert!(oracle.is_some(), "hole {} not located by the oracle", i);
            assert_eq!(gridded[i], oracle, "hole {} seed triangle mismatch", i);
        }

        // Full carve: the normal pipeline (gridded path, 25 holes) vs a
        // carve seeded via locate_triangle. The fixture builds
        // deterministically, so both meshes start identical and must end
        // with the exact same surviving triangles.
        let (mut mesh_grid, pslg) = build_square_with_hole_grid(5);
        let (mut mesh_oracle, _) = build_square_with_hole_grid(5);
        let killed_grid = carve_holes(&mut mesh_grid, &pslg, false);
        let killed_oracle = {
            let mut infected = vec![false; mesh_oracle.triangles.len()];
            let mut worklist = VecDeque::new();
            infect_hull(&mut mesh_oracle, &mut infected, &mut worklist);
            let positions = mesh_oracle.vertices_positions();
            for hole in &pslg.holes {
                if let Some(tri) = locate_triangle(&mesh_oracle, &positions, hole.point) {
                    if !infected[tri as usize] {
                        infected[tri as usize] = true;
                        worklist.push_back(tri);
                    }
                }
            }
            plague(&mut mesh_oracle, &mut infected, &mut worklist);
            sweep(&mut mesh_oracle, &infected)
        };
        assert_eq!(killed_grid, killed_oracle, "carve killed different counts");

        let survivors = |mesh: &CdtMesh| -> Vec<(u32, [VertexId; 3])> {
            (1..mesh.triangles.len() as u32)
                .filter(|&i| !mesh.triangle(i).is_dead())
                .map(|i| (i, mesh.triangle(i).vertices))
                .collect()
        };
        let s = survivors(&mesh_grid);
        assert!(!s.is_empty());
        assert_eq!(s, survivors(&mesh_oracle), "surviving triangle sets differ");
    }
}
