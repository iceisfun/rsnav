//! Segment insertion: force PSLG segments into a Delaunay triangulation.
//!
//! Port of triangle.c's segment-insertion machinery:
//!
//! - [`make_vertex_map`] (precondition): give every vertex a pointer to an
//!   incident triangle so segment endpoints can be located in O(1).
//! - `insert_subseg` (utility): glue a subseg onto a triangle edge if no
//!   subseg is there yet.
//! - [`find_direction`]: from a triangle handle at one endpoint, find the
//!   first triangle that the directed segment to the second endpoint enters.
//! - [`scout_segment`]: fast path — try to march along an existing chain of
//!   triangle edges from endpoint to endpoint.
//! - [`delaunay_fixup`]: locally re-flip edges to restore the Delaunay
//!   property after a segment has been carved through.
//! - [`constrained_edge`]: slow path — when scout fails, dig a fan through
//!   the polygon by flipping edges, then `delaunay_fixup` each side.
//! - [`insert_segment`]: top-level driver for one PSLG segment.
//! - [`form_skeleton`]: process every input segment from a [`Pslg`].
//!
//! `segmentintersection` (handles self-intersecting PSLGs by inserting a
//! Steiner point at the crossing) and `conformingedge` (forces conformity
//! by recursive midpoint splitting) are intentionally **not** ported in v1.
//! Each function returns [`SegmentInsertError::SelfIntersection`] when it
//! would need them — only invalid input (a user-drawn self-crossing
//! polygon) or the `-q`/`-Y` quality-refinement modes need them.

use std::collections::HashMap;

use rsnav_common::VertexId;

use crate::flip;
use crate::mesh::{CdtMesh, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::predicates::orient2d;
use crate::pslg::Pslg;

// --- Errors --------------------------------------------------------------

/// Failure modes for [`insert_segment`] and the functions it calls.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentInsertError {
    /// The PSLG segment from `endpoint1` to `endpoint2` would cross an
    /// existing constrained subsegment. v1 doesn't support self-
    /// intersecting PSLG input (the `segmentintersection` /
    /// `conformingedge` paths from triangle.c are not ported).
    /// The CDT is left in a valid state but the segment was not inserted.
    SelfIntersection {
        endpoint1: VertexId,
        endpoint2: VertexId,
    },
    /// The segment references a vertex that isn't a corner of any live
    /// triangle. [`form_skeleton`] auto-remaps duplicate-position vertex
    /// IDs to their canonical ID (the first-occurrence index), so this
    /// fires only for genuinely missing vertices — a segment endpoint
    /// that wasn't pushed into the CDT at all.
    VertexNotInTriangulation { vertex: VertexId },
}

impl std::fmt::Display for SegmentInsertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SelfIntersection { endpoint1, endpoint2 } => write!(
                f,
                "PSLG segment ({} → {}) crosses an existing constrained subsegment \
                 (self-intersecting input is not supported in v1)",
                endpoint1.get(),
                endpoint2.get()
            ),
            Self::VertexNotInTriangulation { vertex } => write!(
                f,
                "vertex {} is not a corner of any live triangle in the CDT",
                vertex.get()
            ),
        }
    }
}

impl std::error::Error for SegmentInsertError {}

// --- makevertexmap -------------------------------------------------------

/// For every triangle corner, store a back-pointer to the triangle on the
/// vertex slot. After this runs, each vertex's `triangle` field points at
/// *some* triangle that has it as a corner. Used by [`insert_segment`] to
/// locate segment endpoints in O(1).
///
/// Port of `makevertexmap()`.
pub fn make_vertex_map(mesh: &mut CdtMesh) {
    for tri_idx in 1..mesh.triangles.len() as u32 {
        if mesh.triangle(tri_idx).is_dead() {
            continue;
        }
        for orient in 0..3u8 {
            let handle = Otri::new(tri_idx, orient);
            let v = mesh.org(handle);
            if v.is_valid() {
                mesh.vertex_mut(v).triangle = handle.encode();
            }
        }
    }
}

// --- insertsubseg --------------------------------------------------------

/// Glue a fresh subseg onto the edge held by `tri`. If a subseg is already
/// there, just promote its marker if it was unmarked.
///
/// Port of `insertsubseg()`. Also used by the winding cull's boundary
/// backfill (`carve_by_winding`).
pub(crate) fn insert_subseg(mesh: &mut CdtMesh, tri: Otri, subsegmark: i32) {
    let triorg = mesh.org(tri);
    let tridest = mesh.dest(tri);
    if triorg.is_valid() && mesh.vertex(triorg).marker == 0 {
        mesh.vertex_mut(triorg).marker = subsegmark;
    }
    if tridest.is_valid() && mesh.vertex(tridest).marker == 0 {
        mesh.vertex_mut(tridest).marker = subsegmark;
    }

    let existing = mesh.tspivot(tri);
    if existing.sub == DUMMY_SUB {
        // Fresh subseg.
        let new_sub = mesh.make_subseg();
        // Subsegment org/dest are flipped relative to the holding triangle
        // because the subseg conventionally records the *opposite* side.
        mesh.set_sorg(new_sub, tridest);
        mesh.set_sdest(new_sub, triorg);
        mesh.set_segorg(new_sub, tridest);
        mesh.set_segdest(new_sub, triorg);

        mesh.tsbond(tri, new_sub);
        let oppotri = mesh.sym(tri);
        let new_sub_other = new_sub.ssym();
        mesh.tsbond(oppotri, new_sub_other);
        mesh.subseg_mut(new_sub.sub).marker = subsegmark;
    } else if mesh.subseg(existing.sub).marker == 0 {
        mesh.subseg_mut(existing.sub).marker = subsegmark;
    }
}

// --- finddirection -------------------------------------------------------

/// Result of [`find_direction`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FindDirection {
    /// `searchpoint` is strictly inside the wedge between `searchtri`'s
    /// dest and apex (i.e. the segment is interior to this triangle).
    Within,
    /// `searchpoint` lies on the ray from `searchtri.org` to `searchtri.apex`
    /// (the segment runs along the left edge of `searchtri`).
    LeftCollinear,
    /// `searchpoint` lies on the ray from `searchtri.org` to `searchtri.dest`
    /// (the segment runs along the right edge of `searchtri`).
    RightCollinear,
}

/// Rotate `searchtri` around its origin until the directed segment from
/// `org(searchtri)` to `searchpoint` either enters the triangle, runs along
/// its dest-edge (`RightCollinear`), or runs along its apex-edge
/// (`LeftCollinear`).
///
/// Port of `finddirection()`.
pub fn find_direction(
    mesh: &CdtMesh,
    searchtri: &mut Otri,
    searchpoint: VertexId,
) -> FindDirection {
    let startvertex = mesh.org(*searchtri);
    let mut rightvertex = mesh.dest(*searchtri);
    let mut leftvertex = mesh.apex(*searchtri);

    let sp = mesh.vertex_pos(searchpoint);
    let sv = mesh.vertex_pos(startvertex);

    // Is `searchpoint` to the left of edge org→apex?
    let mut leftccw = orient2d(sp, sv, mesh.vertex_pos(leftvertex));
    let mut leftflag = leftccw > 0.0;
    // Is `searchpoint` to the right of edge org→dest?
    let mut rightccw = orient2d(sv, sp, mesh.vertex_pos(rightvertex));
    let mut rightflag = rightccw > 0.0;

    if leftflag && rightflag {
        // searchtri faces directly away from searchpoint. Try to spin
        // counter-clockwise first; if onext lands on the dummy (we'd fall
        // off the boundary) spin clockwise instead.
        let checktri = onext(mesh, *searchtri);
        if checktri.tri == DUMMY_TRI {
            leftflag = false;
        } else {
            rightflag = false;
        }
    }
    while leftflag {
        // Turn left until satisfied.
        onextself(mesh, searchtri);
        assert!(
            searchtri.tri != DUMMY_TRI,
            "find_direction: walked off the boundary while turning left from ({}, {}) toward ({}, {})",
            sv.x, sv.y, sp.x, sp.y,
        );
        leftvertex = mesh.apex(*searchtri);
        rightccw = leftccw;
        leftccw = orient2d(sp, sv, mesh.vertex_pos(leftvertex));
        leftflag = leftccw > 0.0;
    }
    while rightflag {
        // Turn right until satisfied.
        oprevself(mesh, searchtri);
        assert!(
            searchtri.tri != DUMMY_TRI,
            "find_direction: walked off the boundary while turning right from ({}, {}) toward ({}, {})",
            sv.x, sv.y, sp.x, sp.y,
        );
        rightvertex = mesh.dest(*searchtri);
        leftccw = rightccw;
        rightccw = orient2d(sv, sp, mesh.vertex_pos(rightvertex));
        rightflag = rightccw > 0.0;
    }

    if leftccw == 0.0 {
        FindDirection::LeftCollinear
    } else if rightccw == 0.0 {
        FindDirection::RightCollinear
    } else {
        FindDirection::Within
    }
}

// --- onext / oprev (spin around the origin) ---------------------------

/// onext: next edge counter-clockwise with the same origin.
/// = lprev(o) then sym.
#[inline]
fn onext(mesh: &CdtMesh, o: Otri) -> Otri {
    mesh.sym(o.lprev())
}

#[inline]
fn onextself(mesh: &CdtMesh, o: &mut Otri) {
    *o = onext(mesh, *o);
}

/// oprev: previous edge (clockwise) with the same origin.
/// = sym then lnext.
#[inline]
fn oprev(mesh: &CdtMesh, o: Otri) -> Otri {
    mesh.sym(o).lnext()
}

#[inline]
fn oprevself(mesh: &CdtMesh, o: &mut Otri) {
    *o = oprev(mesh, *o);
}

// --- scoutsegment --------------------------------------------------------

/// March from `org(searchtri)` toward `endpoint2` along existing triangle
/// edges. Returns `Ok(true)` if the segment was successfully inserted
/// (i.e. an existing edge of the mesh coincides with the segment, possibly
/// after chaining through collinear interior vertices).
///
/// On `Ok(false)`, `searchtri` is positioned at the triangle from which
/// [`constrained_edge`] should start digging.
///
/// Returns [`SegmentInsertError::SelfIntersection`] if the segment's
/// path crosses an existing constrained subsegment — v1 doesn't support
/// self-intersecting PSLG input.
///
/// Port of `scoutsegment()`.
pub fn scout_segment(
    mesh: &mut CdtMesh,
    searchtri: &mut Otri,
    endpoint2: VertexId,
    newmark: i32,
) -> Result<bool, SegmentInsertError> {
    let endpoint1 = mesh.org(*searchtri);
    let collinear = find_direction(mesh, searchtri, endpoint2);
    let rightvertex = mesh.dest(*searchtri);
    let leftvertex = mesh.apex(*searchtri);

    let ep2_pos = mesh.vertex_pos(endpoint2);
    let lv_pos = mesh.vertex_pos(leftvertex);
    let rv_pos = mesh.vertex_pos(rightvertex);

    if lv_pos == ep2_pos || rv_pos == ep2_pos {
        // The segment is already an edge in the mesh.
        if lv_pos == ep2_pos {
            *searchtri = searchtri.lprev();
        }
        insert_subseg(mesh, *searchtri, newmark);
        return Ok(true);
    }

    match collinear {
        FindDirection::LeftCollinear => {
            // Collided with an intervening collinear vertex; bind a subseg
            // up to that vertex, then recursively insert the remainder.
            *searchtri = searchtri.lprev();
            insert_subseg(mesh, *searchtri, newmark);
            scout_segment(mesh, searchtri, endpoint2, newmark)
        }
        FindDirection::RightCollinear => {
            insert_subseg(mesh, *searchtri, newmark);
            *searchtri = searchtri.lnext();
            scout_segment(mesh, searchtri, endpoint2, newmark)
        }
        FindDirection::Within => {
            // We'd have to cut through an interior edge. Check whether
            // that edge is already a constrained subseg.
            let crosstri = searchtri.lnext();
            let crosssubseg = mesh.tspivot(crosstri);
            if crosssubseg.sub == DUMMY_SUB {
                Ok(false)
            } else {
                Err(SegmentInsertError::SelfIntersection {
                    endpoint1,
                    endpoint2,
                })
            }
        }
    }
}

// --- delaunayfixup -------------------------------------------------------

/// Re-flip edges to restore the local Delaunay condition after a vertex
/// has effectively been inserted at `org(fixuptri)`. Handles reflex
/// vertices via a "stack" of inverted triangles (no actual stack — just
/// the natural recursion).
///
/// Port of `delaunayfixup()`.
pub fn delaunay_fixup(mesh: &mut CdtMesh, fixuptri: &mut Otri, leftside: bool) {
    let mut neartri = fixuptri.lnext();
    let fartri = mesh.sym(neartri);
    if fartri.tri == DUMMY_TRI {
        return;
    }
    let faredge = mesh.tspivot(neartri);
    if faredge.sub != DUMMY_SUB {
        return;
    }

    let nearvertex = mesh.apex(neartri);
    let leftvertex = mesh.org(neartri);
    let rightvertex = mesh.dest(neartri);
    let farvertex = mesh.apex(fartri);

    let lv_pos = mesh.vertex_pos(leftvertex);
    let rv_pos = mesh.vertex_pos(rightvertex);
    let nv_pos = mesh.vertex_pos(nearvertex);
    let fv_pos = mesh.vertex_pos(farvertex);

    // Reflex-vertex check on the "behind" vertex on this side.
    if leftside {
        if orient2d(nv_pos, lv_pos, fv_pos) <= 0.0 {
            return;
        }
    } else {
        if orient2d(fv_pos, rv_pos, nv_pos) <= 0.0 {
            return;
        }
    }

    if orient2d(rv_pos, lv_pos, fv_pos) > 0.0 {
        // fartri is not inverted, and no reflex vertices; just check that
        // the edge between us is locally Delaunay.
        if crate::predicates::incircle(lv_pos, fv_pos, rv_pos, nv_pos) <= 0.0 {
            return;
        }
    }
    // Else fartri is inverted: pop it from the "stack" by flipping.

    flip::flip(mesh, &mut neartri);
    *fixuptri = fixuptri.lprev(); // restore origin after the flip
    delaunay_fixup(mesh, fixuptri, leftside);
    let mut fartri_copy = fartri;
    delaunay_fixup(mesh, &mut fartri_copy, leftside);
}

// --- constrainededge -----------------------------------------------------

/// Force a segment from `org(starttri)` to `endpoint2` through the
/// triangulation by repeatedly flipping the edges it crosses, then
/// re-Delaunay-fying each side.
///
/// Returns [`SegmentInsertError::SelfIntersection`] if the dig would
/// have to flip an existing constrained subsegment.
///
/// Port of `constrainededge()`.
pub fn constrained_edge(
    mesh: &mut CdtMesh,
    starttri: &mut Otri,
    endpoint2: VertexId,
    newmark: i32,
) -> Result<(), SegmentInsertError> {
    let endpoint1 = mesh.org(*starttri);
    let endpoint1_pos = mesh.vertex_pos(endpoint1);
    let endpoint2_pos = mesh.vertex_pos(endpoint2);

    let mut fixuptri = starttri.lnext();
    flip::flip(mesh, &mut fixuptri);

    let mut collision = false;
    let mut done = false;
    while !done {
        let farvertex = mesh.org(fixuptri);
        let fv_pos = mesh.vertex_pos(farvertex);

        if fv_pos == endpoint2_pos {
            let mut fixuptri2 = oprev(mesh, fixuptri);
            // Re-Delaunay around endpoint2.
            delaunay_fixup(mesh, &mut fixuptri, false);
            delaunay_fixup(mesh, &mut fixuptri2, true);
            done = true;
        } else {
            let area = orient2d(endpoint1_pos, endpoint2_pos, fv_pos);
            if area == 0.0 {
                // Collided with a vertex between endpoint1 and endpoint2.
                collision = true;
                let mut fixuptri2 = oprev(mesh, fixuptri);
                delaunay_fixup(mesh, &mut fixuptri, false);
                delaunay_fixup(mesh, &mut fixuptri2, true);
                done = true;
            } else {
                if area > 0.0 {
                    // farvertex is to the LEFT of the segment.
                    let mut fixuptri2 = oprev(mesh, fixuptri);
                    delaunay_fixup(mesh, &mut fixuptri2, true);
                    fixuptri = fixuptri.lprev();
                } else {
                    // farvertex is to the RIGHT of the segment.
                    delaunay_fixup(mesh, &mut fixuptri, false);
                    oprevself(mesh, &mut fixuptri);
                }
                // If we'd flip a constrained edge, that's a PSLG self-intersection.
                let crosssubseg = mesh.tspivot(fixuptri);
                if crosssubseg.sub == DUMMY_SUB {
                    flip::flip(mesh, &mut fixuptri);
                } else {
                    return Err(SegmentInsertError::SelfIntersection {
                        endpoint1,
                        endpoint2,
                    });
                }
            }
        }
    }

    insert_subseg(mesh, fixuptri, newmark);

    if collision {
        // Insert the remainder of the segment past the collision point.
        if !scout_segment(mesh, &mut fixuptri, endpoint2, newmark)? {
            constrained_edge(mesh, &mut fixuptri, endpoint2, newmark)?;
        }
    }
    Ok(())
}

// --- insertsegment -------------------------------------------------------

/// Insert one PSLG segment connecting two vertices that already exist in
/// the mesh. Port of `insertsegment()`.
///
/// Returns [`SegmentInsertError::SelfIntersection`] if the segment would
/// cross an existing constrained subseg, or
/// [`SegmentInsertError::VertexNotInTriangulation`] if either endpoint
/// has been deduplicated out of the triangulation (use
/// [`form_skeleton`] for the high-level driver, which auto-remaps
/// duplicate-position vertex IDs).
///
/// Pre: [`make_vertex_map`] has been called so each endpoint has a valid
/// `triangle` back-pointer.
pub fn insert_segment(
    mesh: &mut CdtMesh,
    endpoint1: VertexId,
    endpoint2: VertexId,
    newmark: i32,
) -> Result<(), SegmentInsertError> {
    let mut searchtri1 = locate_vertex(mesh, endpoint1)
        .map_err(|vertex| SegmentInsertError::VertexNotInTriangulation { vertex })?;
    if !scout_segment(mesh, &mut searchtri1, endpoint2, newmark)? {
        // Scout from the other end too, so collisions detected from either
        // side leave us with a tight start point.
        let mut searchtri2 = locate_vertex(mesh, endpoint2)
            .map_err(|vertex| SegmentInsertError::VertexNotInTriangulation { vertex })?;
        if scout_segment(mesh, &mut searchtri2, endpoint1, newmark)? {
            return Ok(());
        }
        constrained_edge(mesh, &mut searchtri1, endpoint2, newmark)?;
    }
    Ok(())
}

/// Look up a triangle that has `v` as one of its corners and return a handle
/// at that corner (i.e. with `org` = v).
///
/// Tries the cached `vertex.triangle` first (set by [`make_vertex_map`])
/// — rotating through its three orientations to handle flips that left
/// the same triangle but shuffled the vertex slots. If that fails the
/// cache is stale (a prior segment-insertion flip moved `v` to a
/// different triangle entirely) and we fall back to a linear scan over
/// live triangles. On a successful scan we refresh the cache so the next
/// lookup is O(1) again. Returns `Err(v)` if `v` isn't a corner of any
/// live triangle — the typical cause is a duplicate-position vertex
/// dropped by `delaunay()`; [`form_skeleton`] auto-remaps those.
///
/// This mirrors triangle.c's `insertsegment()` which checks the cached
/// triangle's org and falls back to `locate()` (point location) on
/// mismatch. We use a linear scan instead of point location — it's O(n)
/// per fallback, but the fallback only fires when flips have stale-d the
/// cache, which scales with mesh complexity, not query count.
fn locate_vertex(mesh: &mut CdtMesh, v: VertexId) -> Result<Otri, VertexId> {
    let encoded = mesh.vertex(v).triangle;
    if encoded.tri() != DUMMY_TRI {
        let base = encoded.to_otri();
        for orient_off in 0..3u8 {
            let candidate = Otri::new(base.tri, (base.orient + orient_off) % 3);
            if mesh.org(candidate) == v {
                return Ok(candidate);
            }
        }
    }
    for tri_idx in 1..mesh.triangles.len() as u32 {
        if mesh.triangle(tri_idx).is_dead() {
            continue;
        }
        for orient in 0..3u8 {
            let h = Otri::new(tri_idx, orient);
            if mesh.org(h) == v {
                mesh.vertex_mut(v).triangle = h.encode();
                return Ok(h);
            }
        }
    }
    Err(v)
}

// --- formskeleton --------------------------------------------------------

/// Push every segment in `pslg` into `mesh`, in input order. The optional
/// `mark_hull_with` is the boundary marker applied to convex-hull edges
/// that aren't already constrained — pass `Some(1)` to mimic triangle.c's
/// default `markhull` step.
///
/// Returns [`SegmentInsertError::SelfIntersection`] on the first segment
/// that would cross an existing constrained subseg. The CDT is left in a
/// valid state up to that point — callers can either bail or strip the
/// bad segment from the PSLG and retry.
///
/// Auto-handles duplicate-position vertices: [`delaunay()`](crate::delaunay) silently
/// drops bit-exact duplicates, so a segment that references a dropped
/// ID would otherwise crash in `locate_vertex`. This function builds a
/// position → first-occurrence-ID remap from the mesh's own vertex pool
/// and rewrites each segment's endpoints through it. Degenerate
/// (canonically-self-loop) segments after remap are silently skipped.
///
/// Port of `formskeleton()`.
pub fn form_skeleton(
    mesh: &mut CdtMesh,
    pslg: &Pslg,
    mark_hull_with: Option<i32>,
) -> Result<(), SegmentInsertError> {
    make_vertex_map(mesh);

    // Build the position → canonical-vertex-ID remap from the mesh's
    // pool. After delaunay() has dropped exact-position duplicates, the
    // canonical (in-triangulation) ID for any position is the first
    // pool-order vertex with that position.
    let remap = canonical_remap(mesh);

    for seg in &pslg.segments {
        if (seg.a as usize) >= remap.len() || (seg.b as usize) >= remap.len() {
            // Segment endpoint out of range of the mesh's vertex pool.
            // Treat as missing-from-triangulation.
            return Err(SegmentInsertError::VertexNotInTriangulation {
                vertex: VertexId::new(
                    if (seg.a as usize) >= remap.len() { seg.a } else { seg.b },
                ),
            });
        }
        let a = remap[seg.a as usize];
        let b = remap[seg.b as usize];
        if a == b {
            continue; // self-loop after dedupe — silently skip
        }
        insert_segment(mesh, a, b, seg.marker)?;
    }
    if let Some(mark) = mark_hull_with {
        mark_hull(mesh, mark);
    }
    Ok(())
}

/// For every vertex `i` in the mesh's pool, return the ID of the
/// first-occurrence vertex with the same position. Used by
/// [`form_skeleton`] to redirect segment endpoints around the
/// duplicates that `delaunay()` silently drops.
fn canonical_remap(mesh: &CdtMesh) -> Vec<VertexId> {
    let n = mesh.vertices.len();
    let mut by_pos: HashMap<(u64, u64), VertexId> = HashMap::with_capacity(n);
    let mut remap = Vec::with_capacity(n);
    for i in 0..n {
        let v = mesh.vertices[i].position;
        let key = (v.x.to_bits(), v.y.to_bits());
        let canonical = *by_pos.entry(key).or_insert(VertexId::new(i as u32));
        remap.push(canonical);
    }
    remap
}

/// Cover every convex-hull edge with a subseg if it isn't already
/// constrained. Port of `markhull()`.
pub fn mark_hull(mesh: &mut CdtMesh, marker: i32) {
    // Start at the dummy: its neighbors[0] points at a real hull edge.
    let mut hulltri = mesh.sym(Otri::new(DUMMY_TRI, 0));
    if hulltri.tri == DUMMY_TRI {
        // No hull (empty or degenerate mesh).
        return;
    }
    let starttri = hulltri;
    loop {
        insert_subseg(mesh, hulltri, marker);
        // Step CCW around the hull: lnext, then walk inward via oprev until
        // we hit the boundary again.
        hulltri = hulltri.lnext();
        let mut nexttri = oprev(mesh, hulltri);
        while nexttri.tri != DUMMY_TRI {
            hulltri = nexttri;
            nexttri = oprev(mesh, hulltri);
        }
        if hulltri == starttri {
            break;
        }
    }
}

// --- Tests ----------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::divconq::{delaunay, DivConqOptions};
    use crate::mesh::{EncodedTri, VertexSlot};
    use crate::pslg::PslgSegment;
    use rsnav_common::Vertex;

    fn push(m: &mut CdtMesh, x: f64, y: f64) -> VertexId {
        m.push_vertex(VertexSlot::new(Vertex::new(x, y), 0))
    }

    fn build_square_delaunay() -> CdtMesh {
        let mut m = CdtMesh::new();
        push(&mut m, 0.0, 0.0);
        push(&mut m, 4.0, 0.0);
        push(&mut m, 4.0, 4.0);
        push(&mut m, 0.0, 4.0);
        delaunay(&mut m, DivConqOptions::default());
        m
    }

    #[test]
    fn make_vertex_map_back_pointers_are_valid() {
        let mut m = build_square_delaunay();
        // Reset back-pointers so we can confirm the function sets them.
        for v in &mut m.vertices {
            v.triangle = EncodedTri::DUMMY;
        }
        make_vertex_map(&mut m);
        for (i, v) in m.vertices.iter().enumerate() {
            assert_ne!(
                v.triangle.tri(),
                DUMMY_TRI,
                "vertex {} got no triangle back-pointer",
                i
            );
            // The back-pointer's referenced triangle should actually have v
            // as one of its corners.
            let t = v.triangle.to_otri();
            let corners = m.triangle(t.tri).vertices;
            assert!(
                corners.iter().any(|c| c.get() as usize == i),
                "vertex {} back-pointer triangle {} doesn't contain it",
                i, t.tri
            );
        }
    }

    #[test]
    fn find_direction_within_triangle() {
        let mut m = CdtMesh::new();
        let a = push(&mut m, 0.0, 0.0);
        let _b = push(&mut m, 5.0, 0.0);
        let c = push(&mut m, 2.5, 5.0);
        delaunay(&mut m, DivConqOptions::default());
        make_vertex_map(&mut m);
        let mut handle = locate_vertex(&mut m, a).expect("vertex a should be in the triangulation");
        let result = find_direction(&m, &mut handle, c);
        assert_eq!(m.org(handle), a);
        // c is a corner of the only real triangle; the handle should end
        // up with either dest = c (RightCollinear) or apex = c (LeftCollinear).
        let lv = m.apex(handle);
        let rv = m.dest(handle);
        assert!(lv == c || rv == c, "result {:?}, lv={:?} rv={:?}", result, lv, rv);
    }

    /// Inserting a segment that already coincides with an existing mesh edge
    /// should just add the subseg in place — no flips.
    #[test]
    fn insert_segment_coincident_with_existing_edge() {
        let mut m = build_square_delaunay();
        let tri_count_before = m.live_triangle_count();
        make_vertex_map(&mut m);
        // Hull edge (0,0)-(4,0) (verts 0 and 1) is definitely a Delaunay edge.
        insert_segment(&mut m, VertexId::new(0), VertexId::new(1), 7).unwrap();
        assert_eq!(m.live_triangle_count(), tri_count_before);
        // The subseg should now exist on at least one triangle.
        assert!(
            (1..m.subsegs.len() as u32)
                .filter(|i| !m.subseg(*i).is_dead())
                .any(|i| m.subseg(i).marker == 7),
            "expected a subseg with marker 7 to exist"
        );
    }

    /// Insert the missing diagonal of a 4-point square: the algorithm picks
    /// one diagonal Delaunay-wise; forcing the other should flip it in.
    #[test]
    fn insert_segment_forces_crossing_diagonal() {
        let mut m = build_square_delaunay();
        make_vertex_map(&mut m);
        // After Delaunay on a regular square, one diagonal exists. Try to
        // insert the OTHER diagonal; if it already coincides, the test is a
        // no-op (some perturbations may pick either). For a perfectly square
        // input the choice is arbitrary; force diagonal (0, 2).
        insert_segment(&mut m, VertexId::new(0), VertexId::new(2), 3).unwrap();
        // Mesh remains 2 triangles either way.
        assert_eq!(m.live_triangle_count(), 2);
        // Confirm there's now a subseg on the (0, 2) edge.
        let v0 = m.vertex_pos(VertexId::new(0));
        let v2 = m.vertex_pos(VertexId::new(2));
        let mut found = false;
        for s in 1..m.subsegs.len() as u32 {
            if m.subseg(s).is_dead() {
                continue;
            }
            let a = m.subseg(s).sub_vertices[0];
            let b = m.subseg(s).sub_vertices[1];
            let pa = m.vertex_pos(a);
            let pb = m.vertex_pos(b);
            if (pa == v0 && pb == v2) || (pa == v2 && pb == v0) {
                found = true;
                break;
            }
        }
        assert!(found, "expected a subseg coinciding with the forced diagonal (0,2)");
    }

    /// Load Shewchuk's A.poly, build the Delaunay over its 29 input
    /// vertices, then insert all 29 segments. We don't carve holes yet
    /// (phase 6) — this is the smoke test that the segment-insertion
    /// pipeline handles a real PSLG without panicking. The convex hull
    /// closes back on itself so every segment is a hull edge or already
    /// a Delaunay edge of the point set; this stresses scout_segment +
    /// insert_subseg.
    #[test]
    fn form_skeleton_handles_a_poly() {
        use std::path::Path;
        let pslg = crate::io::read_poly(Path::new("../../../triangle/A.poly")).unwrap();
        assert_eq!(pslg.vertices.len(), 29);
        assert_eq!(pslg.segments.len(), 29);

        let mut mesh = CdtMesh::new();
        for v in &pslg.vertices {
            mesh.push_vertex(VertexSlot::new(v.position, v.marker));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let tris_after_delaunay = mesh.live_triangle_count();

        form_skeleton(&mut mesh, &pslg, None).unwrap();

        // form_skeleton may have added new vertices (for forced subseg splits)
        // and triangles (for flipped diagonals). Triangle count should still
        // satisfy the Euler relation t = 2n - 2 - h.
        let n = mesh.vertices.len();
        let hull = mesh.hull_size as usize;
        assert_eq!(
            mesh.live_triangle_count(),
            2 * n - 2 - hull,
            "Euler relation violated after segment insertion (had {} tris pre-skeleton)",
            tris_after_delaunay,
        );

        // Every PSLG segment should now correspond to a constrained subseg.
        let subseg_count = (1..mesh.subsegs.len() as u32)
            .filter(|i| !mesh.subseg(*i).is_dead())
            .count();
        assert!(
            subseg_count >= 29,
            "expected at least 29 subsegs (one per input segment), got {}",
            subseg_count
        );
    }

    /// Full PSLG: a square boundary as 4 segments, plus a forced diagonal.
    /// After form_skeleton + mark_hull, every hull edge and the forced
    /// diagonal should be a constrained subseg.
    #[test]
    fn form_skeleton_unit_square_with_diagonal() {
        let mut mesh = CdtMesh::new();
        push(&mut mesh, 0.0, 0.0);
        push(&mut mesh, 4.0, 0.0);
        push(&mut mesh, 4.0, 4.0);
        push(&mut mesh, 0.0, 4.0);
        delaunay(&mut mesh, DivConqOptions::default());

        let pslg = Pslg {
            vertices: vec![
                crate::pslg::PslgVertex::new(Vertex::new(0.0, 0.0)),
                crate::pslg::PslgVertex::new(Vertex::new(4.0, 0.0)),
                crate::pslg::PslgVertex::new(Vertex::new(4.0, 4.0)),
                crate::pslg::PslgVertex::new(Vertex::new(0.0, 4.0)),
            ],
            segments: vec![
                PslgSegment { a: 0, b: 1, marker: 10 },
                PslgSegment { a: 1, b: 2, marker: 10 },
                PslgSegment { a: 2, b: 3, marker: 10 },
                PslgSegment { a: 3, b: 0, marker: 10 },
                PslgSegment { a: 0, b: 2, marker: 99 }, // forced diagonal
            ],
            holes: Vec::new(),
        };

        form_skeleton(&mut mesh, &pslg, Some(1)).unwrap();
        assert_eq!(mesh.live_triangle_count(), 2);

        // 4 hull subsegs + 1 diagonal = 5 subsegs.
        let live_subsegs: Vec<_> = (1..mesh.subsegs.len() as u32)
            .filter(|i| !mesh.subseg(*i).is_dead())
            .collect();
        assert_eq!(
            live_subsegs.len(),
            5,
            "expected 5 subsegs (4 hull + 1 diagonal), got {}",
            live_subsegs.len()
        );

        // Hull segments should carry their original marker (10), not be
        // overwritten by mark_hull's 1.
        let marker_99_count = live_subsegs
            .iter()
            .filter(|i| mesh.subseg(**i).marker == 99)
            .count();
        let marker_10_count = live_subsegs
            .iter()
            .filter(|i| mesh.subseg(**i).marker == 10)
            .count();
        assert_eq!(marker_99_count, 1);
        assert_eq!(marker_10_count, 4);
    }
}
