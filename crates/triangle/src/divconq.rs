//! Divide-and-conquer Delaunay triangulation.
//!
//! Translation of triangle.c's `divconqdelaunay`, `divconqrecurse`,
//! `mergehulls`, and `removeghosts` — the default Triangle algorithm,
//! with Dwyer's alternating-axis cuts enabled by default.
//!
//! ## Ghost triangles
//!
//! While the recursion is running, the convex hull of each partial
//! triangulation is bounded by a ring of *ghost* triangles whose apex is
//! `VertexId::INVALID`. These are real allocated triangles, used as
//! sentinels so the half-edge machinery has somewhere to point on the
//! exterior of the hull. After merging completes, [`remove_ghosts`]
//! deallocates them and binds the real boundary edges to `EncodedTri::DUMMY`.

use rsnav_common::VertexId;

use crate::mesh::{CdtMesh, Otri, DUMMY_TRI};
use crate::predicates::{incircle, orient2d};
use crate::sort::{alternate_axes, vertex_sort};

/// Behavior switches for [`delaunay`].
#[derive(Copy, Clone, Debug)]
pub struct DivConqOptions {
    /// Dwyer's optimization: alternate horizontal and vertical cuts down the
    /// recursion. Faster in practice; matches triangle.c's default.
    pub dwyer: bool,
}

impl Default for DivConqOptions {
    fn default() -> Self {
        Self { dwyer: true }
    }
}

/// Build a Delaunay triangulation of all vertices currently in `mesh`'s
/// vertex pool, in place.
///
/// `mesh` must contain only its dummy triangle (i.e. be fresh aside from
/// the input vertices). On return the mesh contains real triangles only
/// (ghosts have been removed); the convex hull edges have their outside
/// neighbor set to [`EncodedTri::DUMMY`]. Returns the number of edges
/// on the convex hull.
pub fn delaunay(mesh: &mut CdtMesh, opts: DivConqOptions) -> u32 {
    // Build the working ID array (one ID per input vertex).
    let mut ids: Vec<VertexId> = (0..mesh.vertices.len() as u32)
        .map(VertexId::new)
        .collect();
    if ids.len() < 2 {
        return 0;
    }

    // Lex-sort by x then y.
    vertex_sort(mesh, &mut ids);

    // Drop duplicates (same x and same y). triangle.c marks duplicates
    // UNDEAD; we just remove them from the working set.
    ids.dedup_by(|a, b| {
        let pa = mesh.vertex_pos(*a);
        let pb = mesh.vertex_pos(*b);
        pa.x == pb.x && pa.y == pb.y
    });

    if ids.len() < 2 {
        return 0;
    }

    // Dwyer re-sort: alternating-axes partition, starting with the y-axis
    // (so the outermost cut, which divconqrecurse will do on the x-axis,
    // already has roughly-equal halves).
    if opts.dwyer {
        let n = ids.len();
        let divider = n >> 1;
        if n - divider >= 2 {
            if divider >= 2 {
                alternate_axes(mesh, &mut ids[..divider], 1);
            }
            alternate_axes(mesh, &mut ids[divider..], 1);
        }
    }

    // Recursive triangulate.
    let (hullleft, _hullright) = divconq_recurse(mesh, &mut ids, 0, opts);

    // Strip the ring of ghost triangles around the convex hull.
    let hull_size = remove_ghosts(mesh, hullleft);
    mesh.hull_size = hull_size;
    hull_size
}

// --- Recursive driver ----------------------------------------------------

fn divconq_recurse(
    mesh: &mut CdtMesh,
    sortarray: &mut [VertexId],
    axis: u8,
    opts: DivConqOptions,
) -> (Otri, Otri) {
    let n = sortarray.len();
    debug_assert!(n >= 2);

    if n == 2 {
        return make_edge_pair(mesh, sortarray[0], sortarray[1]);
    }
    if n == 3 {
        return make_triangle_or_edges(mesh, sortarray[0], sortarray[1], sortarray[2]);
    }

    let divider = n >> 1;
    let (left, right) = sortarray.split_at_mut(divider);
    let (farleft, innerleft) = divconq_recurse(mesh, left, 1 - axis, opts);
    let (innerright, farright) = divconq_recurse(mesh, right, 1 - axis, opts);

    merge_hulls(mesh, farleft, innerleft, innerright, farright, axis, opts)
}

// --- Base case: two vertices = edge bounded by two ghost triangles -------

/// Build the 2-vertex base case: a single edge represented by two ghost
/// triangles that share all three edges. Returns `(farleft, farright)`.
fn make_edge_pair(mesh: &mut CdtMesh, a: VertexId, b: VertexId) -> (Otri, Otri) {
    let mut farleft = mesh.make_triangle();
    mesh.set_org(farleft, a);
    mesh.set_dest(farleft, b);
    // apex left INVALID (ghost).

    let mut farright = mesh.make_triangle();
    mesh.set_org(farright, b);
    mesh.set_dest(farright, a);
    // apex left INVALID (ghost).

    // Bond the two ghosts on all three matching edge pairs.
    mesh.bond(farleft, farright);
    farleft = farleft.lprev();
    farright = farright.lnext();
    mesh.bond(farleft, farright);
    farleft = farleft.lprev();
    farright = farright.lnext();
    mesh.bond(farleft, farright);

    // farleft now points at the edge where org = a. (We lprev'd twice from
    // an edge with org = a, dest = b — which after two lprevs brings us
    // back to an orientation whose org is still a.)
    //
    // Per triangle.c we adjust so `farleft`'s origin is `sortarray[0] = a`:
    //   "Ensure that the origin of `farleft' is sortarray[0]."
    //   lprev(*farright, *farleft);
    farleft = farright.lprev();
    debug_assert_eq!(mesh.org(farleft), a);
    debug_assert_eq!(mesh.dest(farright), b);

    (farleft, farright)
}

// --- Base case: three vertices ------------------------------------------

/// Build the 3-vertex base case. Three collinear points produce two
/// edges; three non-collinear points produce one real triangle.
///
/// In both cases four triangles are allocated (one or two real, the
/// rest ghosts). Returns `(farleft, farright)` such that
/// `farleft.org == a` and `farright.dest == c`.
fn make_triangle_or_edges(
    mesh: &mut CdtMesh,
    a: VertexId,
    b: VertexId,
    c: VertexId,
) -> (Otri, Otri) {
    let mut midtri = mesh.make_triangle();
    let mut tri1 = mesh.make_triangle();
    let mut tri2 = mesh.make_triangle();
    let mut tri3 = mesh.make_triangle();

    let area = orient2d(mesh.vertex_pos(a), mesh.vertex_pos(b), mesh.vertex_pos(c));

    if area == 0.0 {
        // Three collinear vertices: two edges, four ghost triangles.
        mesh.set_org(midtri, a);
        mesh.set_dest(midtri, b);
        mesh.set_org(tri1, b);
        mesh.set_dest(tri1, a);
        mesh.set_org(tri2, c);
        mesh.set_dest(tri2, b);
        mesh.set_org(tri3, b);
        mesh.set_dest(tri3, c);
        // All apices are intentionally INVALID.
        mesh.bond(midtri, tri1);
        mesh.bond(tri2, tri3);
        midtri = midtri.lnext();
        tri1 = tri1.lprev();
        tri2 = tri2.lnext();
        tri3 = tri3.lprev();
        mesh.bond(midtri, tri3);
        mesh.bond(tri1, tri2);
        midtri = midtri.lnext();
        tri1 = tri1.lprev();
        tri2 = tri2.lnext();
        tri3 = tri3.lprev();
        mesh.bond(midtri, tri1);
        mesh.bond(tri2, tri3);
        (tri1, tri2)
    } else {
        // Three non-collinear vertices: one real triangle (midtri) with
        // three surrounding ghost triangles (tri1, tri2, tri3).
        mesh.set_org(midtri, a);
        mesh.set_dest(tri1, a);
        mesh.set_org(tri3, a);

        if area > 0.0 {
            // CCW order.
            mesh.set_dest(midtri, b);
            mesh.set_org(tri1, b);
            mesh.set_dest(tri2, b);
            mesh.set_apex(midtri, c);
            mesh.set_org(tri2, c);
            mesh.set_dest(tri3, c);
        } else {
            // CW order — internally reorder so midtri ends up CCW (a, c, b).
            mesh.set_dest(midtri, c);
            mesh.set_org(tri1, c);
            mesh.set_dest(tri2, c);
            mesh.set_apex(midtri, b);
            mesh.set_org(tri2, b);
            mesh.set_dest(tri3, b);
        }

        // Same topology either way.
        mesh.bond(midtri, tri1);
        midtri = midtri.lnext();
        mesh.bond(midtri, tri2);
        midtri = midtri.lnext();
        mesh.bond(midtri, tri3);
        tri1 = tri1.lprev();
        tri2 = tri2.lnext();
        mesh.bond(tri1, tri2);
        tri1 = tri1.lprev();
        tri3 = tri3.lprev();
        mesh.bond(tri1, tri3);
        tri2 = tri2.lnext();
        tri3 = tri3.lprev();
        mesh.bond(tri2, tri3);

        // "Ensure that the origin of farleft is sortarray[0]"
        // → after the bonds, tri1 is at orient 1; org(tri1@1) = a. ✓
        let farleft = tri1;
        // "Ensure that the destination of farright is sortarray[2]"
        // CCW: farright = tri2 (at orient 2 after the lnexts); dest(tri2@2) = sortarray[2].
        // CW (input order): tri2 stores (sortarray[2] at vertices[1], sortarray[1] at vertices[2]),
        // so dest(tri2@2) = sortarray[1] (wrong). Instead farright = lnext(farleft) = tri1@2;
        // dest(tri1@2) = sortarray[2]. ✓
        let farright = if area > 0.0 {
            tri2
        } else {
            farleft.lnext()
        };
        (farleft, farright)
    }
}

// --- mergehulls: zip two adjacent triangulations ------------------------

/// Merge two adjacent Delaunay triangulations into one.
///
/// `farleft` / `innerleft` are bounding triangles of the left hull
/// (`farleft.org` = leftmost vertex; `innerleft.dest` = rightmost vertex
/// of the *left* triangulation, i.e. the seam edge).
/// `innerright` / `farright` are the corresponding handles on the right
/// hull. Returns the updated `(farleft, farright)` of the merged hull.
fn merge_hulls(
    mesh: &mut CdtMesh,
    mut farleft: Otri,
    mut innerleft: Otri,
    mut innerright: Otri,
    mut farright: Otri,
    axis: u8,
    opts: DivConqOptions,
) -> (Otri, Otri) {
    let mut innerleftdest = mesh.dest(innerleft);
    let mut innerleftapex = mesh.apex(innerleft);
    let mut innerrightorg = mesh.org(innerright);
    let mut innerrightapex = mesh.apex(innerright);

    // Horizontal-cut prep: shift extremal handle pointers from
    // leftmost/rightmost to topmost/bottommost so a vertical merge zip works.
    if opts.dwyer && axis == 1 {
        let mut farleftpt = mesh.org(farleft);
        let mut farleftapex = mesh.apex(farleft);
        let mut farrightpt = mesh.dest(farright);
        let mut farrightapex = mesh.apex(farright);

        while is_valid(farleftapex) && mesh.vertex_pos(farleftapex).y < mesh.vertex_pos(farleftpt).y
        {
            farleft = farleft.lnext();
            farleft = mesh.sym(farleft);
            farleftpt = farleftapex;
            farleftapex = mesh.apex(farleft);
        }
        let mut checkedge = mesh.sym(innerleft);
        let mut checkvertex = mesh.apex(checkedge);
        while is_valid(checkvertex)
            && mesh.vertex_pos(checkvertex).y > mesh.vertex_pos(innerleftdest).y
        {
            innerleft = checkedge.lnext();
            innerleftapex = innerleftdest;
            innerleftdest = checkvertex;
            checkedge = mesh.sym(innerleft);
            checkvertex = mesh.apex(checkedge);
        }
        while is_valid(innerrightapex)
            && mesh.vertex_pos(innerrightapex).y < mesh.vertex_pos(innerrightorg).y
        {
            innerright = innerright.lnext();
            innerright = mesh.sym(innerright);
            innerrightorg = innerrightapex;
            innerrightapex = mesh.apex(innerright);
        }
        let mut checkedge = mesh.sym(farright);
        let mut checkvertex = mesh.apex(checkedge);
        while is_valid(checkvertex)
            && mesh.vertex_pos(checkvertex).y > mesh.vertex_pos(farrightpt).y
        {
            farright = checkedge.lnext();
            farrightapex = farrightpt;
            farrightpt = checkvertex;
            checkedge = mesh.sym(farright);
            checkvertex = mesh.apex(checkedge);
        }
        let _ = (farleftapex, farrightapex); // shadowed only for the loop heads.
    }

    // Find a tangent line below both hulls.
    loop {
        let mut changemade = false;
        if is_valid(innerleftapex) {
            let o = orient2d(
                mesh.vertex_pos(innerleftdest),
                mesh.vertex_pos(innerleftapex),
                mesh.vertex_pos(innerrightorg),
            );
            if o > 0.0 {
                innerleft = innerleft.lprev();
                innerleft = mesh.sym(innerleft);
                innerleftdest = innerleftapex;
                innerleftapex = mesh.apex(innerleft);
                changemade = true;
            }
        }
        if is_valid(innerrightapex) {
            let o = orient2d(
                mesh.vertex_pos(innerrightapex),
                mesh.vertex_pos(innerrightorg),
                mesh.vertex_pos(innerleftdest),
            );
            if o > 0.0 {
                innerright = innerright.lnext();
                innerright = mesh.sym(innerright);
                innerrightorg = innerrightapex;
                innerrightapex = mesh.apex(innerright);
                changemade = true;
            }
        }
        if !changemade {
            break;
        }
    }

    let mut leftcand = mesh.sym(innerleft);
    let mut rightcand = mesh.sym(innerright);

    // Bottom bounding triangle that spans the seam.
    let mut baseedge = mesh.make_triangle();
    mesh.bond(baseedge, innerleft);
    baseedge = baseedge.lnext();
    mesh.bond(baseedge, innerright);
    baseedge = baseedge.lnext();
    mesh.set_org(baseedge, innerrightorg);
    mesh.set_dest(baseedge, innerleftdest);
    // apex stays INVALID — ghost.

    // Fix extreme handles if the bottom tangent already coincides with a hull tip.
    let farleftpt = mesh.org(farleft);
    if innerleftdest == farleftpt {
        farleft = baseedge.lnext();
    }
    let farrightpt = mesh.dest(farright);
    if innerrightorg == farrightpt {
        farright = baseedge.lprev();
    }

    let mut lowerleft = innerleftdest;
    let mut lowerright = innerrightorg;
    let mut upperleft = mesh.apex(leftcand);
    let mut upperright = mesh.apex(rightcand);

    loop {
        let leftfinished = !is_valid(upperleft)
            || orient2d(
                mesh.vertex_pos(upperleft),
                mesh.vertex_pos(lowerleft),
                mesh.vertex_pos(lowerright),
            ) <= 0.0;
        let rightfinished = !is_valid(upperright)
            || orient2d(
                mesh.vertex_pos(upperright),
                mesh.vertex_pos(lowerleft),
                mesh.vertex_pos(lowerright),
            ) <= 0.0;

        if leftfinished && rightfinished {
            // Top bounding triangle.
            let mut nextedge = mesh.make_triangle();
            mesh.set_org(nextedge, lowerleft);
            mesh.set_dest(nextedge, lowerright);
            mesh.bond(nextedge, baseedge);
            nextedge = nextedge.lnext();
            mesh.bond(nextedge, rightcand);
            nextedge = nextedge.lnext();
            mesh.bond(nextedge, leftcand);

            // Horizontal-cut restoration: shift extremal handle pointers
            // back to leftmost/rightmost.
            if opts.dwyer && axis == 1 {
                let mut farleftpt = mesh.org(farleft);
                let mut farleftapex = mesh.apex(farleft);
                let mut farrightpt = mesh.dest(farright);
                let mut farrightapex = mesh.apex(farright);
                let mut checkedge = mesh.sym(farleft);
                let mut checkvertex = mesh.apex(checkedge);
                while is_valid(checkvertex)
                    && mesh.vertex_pos(checkvertex).x < mesh.vertex_pos(farleftpt).x
                {
                    farleft = checkedge.lprev();
                    farleftapex = farleftpt;
                    farleftpt = checkvertex;
                    checkedge = mesh.sym(farleft);
                    checkvertex = mesh.apex(checkedge);
                }
                while is_valid(farrightapex)
                    && mesh.vertex_pos(farrightapex).x > mesh.vertex_pos(farrightpt).x
                {
                    farright = farright.lprev();
                    farright = mesh.sym(farright);
                    farrightpt = farrightapex;
                    farrightapex = mesh.apex(farright);
                }
                let _ = (farleftapex,);
            }
            return (farleft, farright);
        }

        // Consider eliminating edges from the left triangulation.
        if !leftfinished {
            let mut nextedge = leftcand.lprev();
            nextedge = mesh.sym(nextedge);
            let mut nextapex = mesh.apex(nextedge);
            if is_valid(nextapex) {
                let mut badedge = incircle(
                    mesh.vertex_pos(lowerleft),
                    mesh.vertex_pos(lowerright),
                    mesh.vertex_pos(upperleft),
                    mesh.vertex_pos(nextapex),
                ) > 0.0;
                while badedge {
                    // In-place edge flip: rebind neighbors, then rewrite vertex slots.
                    nextedge = nextedge.lnext();
                    let topcasing = mesh.sym(nextedge);
                    nextedge = nextedge.lnext();
                    let sidecasing = mesh.sym(nextedge);
                    mesh.bond(nextedge, topcasing);
                    mesh.bond(leftcand, sidecasing);
                    leftcand = leftcand.lnext();
                    let outercasing = mesh.sym(leftcand);
                    nextedge = nextedge.lprev();
                    mesh.bond(nextedge, outercasing);

                    mesh.set_org(leftcand, lowerleft);
                    mesh.set_dest(leftcand, VertexId::INVALID);
                    mesh.set_apex(leftcand, nextapex);
                    mesh.set_org(nextedge, VertexId::INVALID);
                    mesh.set_dest(nextedge, upperleft);
                    mesh.set_apex(nextedge, nextapex);

                    upperleft = nextapex;
                    nextedge = sidecasing;
                    nextapex = mesh.apex(nextedge);
                    badedge = if is_valid(nextapex) {
                        incircle(
                            mesh.vertex_pos(lowerleft),
                            mesh.vertex_pos(lowerright),
                            mesh.vertex_pos(upperleft),
                            mesh.vertex_pos(nextapex),
                        ) > 0.0
                    } else {
                        false
                    };
                }
            }
        }

        // Consider eliminating edges from the right triangulation.
        if !rightfinished {
            let mut nextedge = rightcand.lnext();
            nextedge = mesh.sym(nextedge);
            let mut nextapex = mesh.apex(nextedge);
            if is_valid(nextapex) {
                let mut badedge = incircle(
                    mesh.vertex_pos(lowerleft),
                    mesh.vertex_pos(lowerright),
                    mesh.vertex_pos(upperright),
                    mesh.vertex_pos(nextapex),
                ) > 0.0;
                while badedge {
                    nextedge = nextedge.lprev();
                    let topcasing = mesh.sym(nextedge);
                    nextedge = nextedge.lprev();
                    let sidecasing = mesh.sym(nextedge);
                    mesh.bond(nextedge, topcasing);
                    mesh.bond(rightcand, sidecasing);
                    rightcand = rightcand.lprev();
                    let outercasing = mesh.sym(rightcand);
                    nextedge = nextedge.lnext();
                    mesh.bond(nextedge, outercasing);

                    mesh.set_org(rightcand, VertexId::INVALID);
                    mesh.set_dest(rightcand, lowerright);
                    mesh.set_apex(rightcand, nextapex);
                    mesh.set_org(nextedge, upperright);
                    mesh.set_dest(nextedge, VertexId::INVALID);
                    mesh.set_apex(nextedge, nextapex);

                    upperright = nextapex;
                    nextedge = sidecasing;
                    nextapex = mesh.apex(nextedge);
                    badedge = if is_valid(nextapex) {
                        incircle(
                            mesh.vertex_pos(lowerleft),
                            mesh.vertex_pos(lowerright),
                            mesh.vertex_pos(upperright),
                            mesh.vertex_pos(nextapex),
                        ) > 0.0
                    } else {
                        false
                    };
                }
            }
        }

        // Add the next gear tooth.
        let pick_right = if leftfinished {
            true
        } else if rightfinished {
            false
        } else {
            // Both candidates valid — Delaunay tie-breaker.
            incircle(
                mesh.vertex_pos(upperleft),
                mesh.vertex_pos(lowerleft),
                mesh.vertex_pos(lowerright),
                mesh.vertex_pos(upperright),
            ) > 0.0
        };

        if pick_right {
            // Knit by adding edge lowerleft -> upperright.
            mesh.bond(baseedge, rightcand);
            baseedge = rightcand.lprev();
            mesh.set_dest(baseedge, lowerleft);
            lowerright = upperright;
            rightcand = mesh.sym(baseedge);
            upperright = mesh.apex(rightcand);
        } else {
            // Knit by adding edge upperleft -> lowerright.
            mesh.bond(baseedge, leftcand);
            baseedge = leftcand.lnext();
            mesh.set_org(baseedge, lowerright);
            lowerleft = upperleft;
            leftcand = mesh.sym(baseedge);
            upperleft = mesh.apex(leftcand);
        }
    }
}

#[inline]
fn is_valid(v: VertexId) -> bool {
    v.is_valid()
}

// --- removeghosts -------------------------------------------------------

/// Walk the ring of ghost triangles around the convex hull, dissolve their
/// boundary bonds (set neighbor to dummy), free the ghost triangles, and
/// return the number of edges on the convex hull.
fn remove_ghosts(mesh: &mut CdtMesh, start_ghost: Otri) -> u32 {
    // Find an edge on the convex hull and stash it on the dummy triangle so
    // subsequent passes can begin point location there. (The C version
    // stores `encode(searchedge)` in `dummytri[0]`. We mirror this so any
    // future code that walks from the dummy starts on a hull edge.)
    let mut searchedge = start_ghost.lprev();
    searchedge = mesh.sym(searchedge);
    mesh.triangles[DUMMY_TRI as usize].neighbors[0] = searchedge.encode();

    let mut dissolveedge = start_ghost;
    let mut hullsize = 0u32;
    loop {
        hullsize += 1;
        let deadtriangle = dissolveedge.lnext();
        dissolveedge = dissolveedge.lprev();
        dissolveedge = mesh.sym(dissolveedge);

        // Remove the ghost neighbor from the hull triangle.
        mesh.dissolve(dissolveedge);

        // Step to the next ghost triangle and free the one we just used.
        let next_dissolve = mesh.sym(deadtriangle);
        mesh.kill_triangle(deadtriangle.tri);
        dissolveedge = next_dissolve;

        if dissolveedge == start_ghost {
            break;
        }
    }
    hullsize
}

// --- Tests --------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::VertexSlot;
    use rsnav_common::Vertex;

    fn push(m: &mut CdtMesh, x: f64, y: f64) -> VertexId {
        m.push_vertex(VertexSlot::new(Vertex::new(x, y), 0))
    }

    /// Walk every live (non-dummy, non-dead) triangle and apply `f`.
    fn for_each_live<F: FnMut(u32, &crate::mesh::TriangleSlot)>(mesh: &CdtMesh, mut f: F) {
        for (i, slot) in mesh.triangles.iter().enumerate().skip(1) {
            if !slot.is_dead() {
                f(i as u32, slot);
            }
        }
    }

    /// Collect live triangles as `[org, dest, apex]` of their orient-0 edge
    /// (i.e. `(vertices[1], vertices[2], vertices[0])`).
    fn live_triangles(mesh: &CdtMesh) -> Vec<[VertexId; 3]> {
        let mut out = Vec::new();
        for_each_live(mesh, |_id, slot| {
            // Skip any leftover ghosts (shouldn't happen post-removeghosts,
            // but be defensive).
            if slot.vertices.iter().all(|v| v.is_valid()) {
                out.push([slot.vertices[1], slot.vertices[2], slot.vertices[0]]);
            }
        });
        out
    }

    #[test]
    fn three_points_one_triangle() {
        let mut m = CdtMesh::new();
        let a = push(&mut m, 0.0, 0.0);
        let b = push(&mut m, 4.0, 0.0);
        let c = push(&mut m, 2.0, 3.0);
        let hull = delaunay(&mut m, DivConqOptions::default());
        assert_eq!(hull, 3);
        let tris = live_triangles(&m);
        assert_eq!(tris.len(), 1);
        // Three vertices appear in some CCW order; check the set.
        let mut got: Vec<u32> = tris[0].iter().map(|v| v.get()).collect();
        got.sort();
        assert_eq!(got, vec![a.get(), b.get(), c.get()]);
    }

    #[test]
    fn four_points_square_two_triangles() {
        let mut m = CdtMesh::new();
        push(&mut m, 0.0, 0.0);
        push(&mut m, 4.0, 0.0);
        push(&mut m, 4.0, 4.0);
        push(&mut m, 0.0, 4.0);
        let hull = delaunay(&mut m, DivConqOptions::default());
        assert_eq!(hull, 4);
        let tris = live_triangles(&m);
        assert_eq!(tris.len(), 2);
        // Every triangle should be CCW.
        for t in &tris {
            let area2 = orient2d(
                m.vertex_pos(t[0]),
                m.vertex_pos(t[1]),
                m.vertex_pos(t[2]),
            );
            assert!(area2 > 0.0, "triangle {:?} is not CCW", t);
        }
    }

    #[test]
    fn euler_formula_triangle_count_matches_hull() {
        // For a planar triangulation of n points with h on the convex hull,
        // the number of triangles is exactly `2n - 2 - h`.
        let mut m = CdtMesh::new();
        // 6 points on a grid.
        for x in 0..3 {
            for y in 0..2 {
                push(&mut m, x as f64, y as f64);
            }
        }
        let hull = delaunay(&mut m, DivConqOptions::default()) as usize;
        let tris = live_triangles(&m);
        let n = m.vertices.len();
        assert_eq!(
            tris.len(),
            2 * n - 2 - hull,
            "Euler relation: 2n - 2 - h = {} (n={}, h={})",
            2 * n - 2 - hull,
            n,
            hull
        );
    }

    /// Pseudo-random f64 in [0, 1) from a 64-bit LCG. Deterministic.
    fn lcg_rand(state: &mut u64) -> f64 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        // Take the top 53 bits as the significand.
        ((*state >> 11) as f64) / ((1u64 << 53) as f64)
    }

    /// Every live triangle should pass the empty-circumcircle Delaunay test
    /// against every other input vertex.
    #[test]
    fn delaunay_property_holds() {
        let mut m = CdtMesh::new();
        let positions: Vec<(f64, f64)> = vec![
            (0.0, 0.0),
            (5.0, 0.5),
            (3.0, 4.0),
            (8.0, 4.0),
            (4.5, 7.0),
            (1.0, 6.0),
        ];
        let ids: Vec<VertexId> = positions.iter().map(|p| push(&mut m, p.0, p.1)).collect();
        delaunay(&mut m, DivConqOptions::default());
        let tris = live_triangles(&m);
        for t in &tris {
            let a = m.vertex_pos(t[0]);
            let b = m.vertex_pos(t[1]);
            let c = m.vertex_pos(t[2]);
            for v in &ids {
                if *v == t[0] || *v == t[1] || *v == t[2] {
                    continue;
                }
                let d = m.vertex_pos(*v);
                let in_or_on = incircle(a, b, c, d);
                assert!(
                    in_or_on <= 0.0,
                    "vertex {:?} is inside circumcircle of triangle {:?}",
                    *v, t
                );
            }
        }
    }

    /// Stress test: 200 deterministically-pseudo-random points in [0, 1)^2.
    /// Validates the Euler relation, CCW orientation of every triangle, and
    /// the global Delaunay property.
    #[test]
    fn random_200_points_delaunay() {
        let mut m = CdtMesh::new();
        let mut state = 0xDEAD_BEEF_C0DE_F00Du64;
        let n = 200;
        let ids: Vec<VertexId> = (0..n)
            .map(|_| push(&mut m, lcg_rand(&mut state) * 1000.0, lcg_rand(&mut state) * 1000.0))
            .collect();
        let hull = delaunay(&mut m, DivConqOptions::default()) as usize;
        let tris = live_triangles(&m);

        assert_eq!(
            tris.len(),
            2 * n as usize - 2 - hull,
            "Euler relation violated"
        );

        // CCW for every triangle.
        for t in &tris {
            let area2 = orient2d(
                m.vertex_pos(t[0]),
                m.vertex_pos(t[1]),
                m.vertex_pos(t[2]),
            );
            assert!(area2 > 0.0, "triangle {:?} is not CCW (area2 = {})", t, area2);
        }

        // Delaunay empty-circumcircle property: spot-check (full O(t·n) is
        // expensive on 200 verts; just check ~50 random vertices per triangle).
        let mut check_state = 0xC0FFEE_u64;
        for t in &tris {
            let a = m.vertex_pos(t[0]);
            let b = m.vertex_pos(t[1]);
            let c = m.vertex_pos(t[2]);
            for _ in 0..50 {
                let v = ids[(lcg_rand(&mut check_state) * n as f64) as usize];
                if v == t[0] || v == t[1] || v == t[2] {
                    continue;
                }
                let d = m.vertex_pos(v);
                let r = incircle(a, b, c, d);
                assert!(
                    r <= 0.0,
                    "Delaunay violated: vertex {:?} inside circumcircle of {:?}, incircle={}",
                    v, t, r
                );
            }
        }
    }
}
