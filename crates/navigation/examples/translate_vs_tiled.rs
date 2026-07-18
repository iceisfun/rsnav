//! Two ways to put a locally-built navmesh at a world origin like
//! (10000, 50000) — bake the offset with `NavMesh::translate`, or keep the
//! mesh local and let `TiledWorld` apply the offset at query time.
//!
//! Run with:
//!   cargo run -p rsnav-navigation --example translate_vs_tiled
//!
//! Either way the CDT itself runs in local coordinates near the origin,
//! which is where the triangulation predicates are happiest; only the
//! finished mesh is placed. The same world-space query is answered by both
//! placements and the polylines match.
//!
//! Which to pick:
//!
//! * `translate` **bakes** the placement: one mesh, placed once, every
//!   coordinate in the mesh is a world coordinate afterward. All the
//!   single-mesh machinery (`Bsp`, `find_path`, `line_of_sight`,
//!   `visibility_region`) works in world space with no wrapper. Translate
//!   *before* `Bsp::build` — a BVH built earlier holds stale absolute
//!   AABBs.
//! * `TiledWorld` is a **view transform**: the mesh stays local and
//!   reusable (the same tile can be added at many origins), the offset can
//!   change later (`set_tile_offset`), and seams to neighboring tiles can
//!   be stitched. The cost is going through the `TiledWorld` query surface.
//!
//! Pick one per placement — adding a pre-translated mesh as a tile with a
//! non-zero offset applies the shift twice.

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_navigation::{find_path, PathOptions, TiledWorld};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
use rsnav_triangle::{carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

/// Where this map lives in the world.
const WORLD_ORIGIN: Vertex = Vertex::new(10_000.0, 50_000.0);

/// A 20×10 room with a 4×4 pillar, built in LOCAL coordinates (origin at
/// 0,0) — the same mesh a bitfield → PSLG → CDT pipeline would produce.
fn local_room() -> NavMesh {
    let pts = [
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 10.0),
        (0.0, 10.0),
        (8.0, 3.0), // pillar
        (12.0, 3.0),
        (12.0, 7.0),
        (8.0, 7.0),
    ];
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    for (x, y) in pts {
        cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
    }
    for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
        pslg.segments.push(PslgSegment { a, b, marker: 1 });
    }
    for &(a, b) in &[(4, 5), (5, 6), (6, 7), (7, 4)] {
        pslg.segments.push(PslgSegment { a, b, marker: 2 });
    }
    pslg.holes.push(PslgHole { point: Vertex::new(10.0, 5.0) });
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    carve_holes(&mut cdt, &pslg, false);
    build_from_cdt(&cdt)
}

fn print_path(label: &str, path: &[Vertex]) {
    let total: f64 = path.windows(2).map(|w| w[0].distance(w[1])).sum();
    println!("{label} ({total:.3} long):");
    for p in path {
        println!("    ({:10.3}, {:10.3})", p.x, p.y);
    }
}

fn main() {
    // The same world-space query for both placements: cross the room past
    // the pillar.
    let start = WORLD_ORIGIN + Vertex::new(2.0, 5.0);
    let goal = WORLD_ORIGIN + Vertex::new(18.0, 5.0);

    // -- Placement 1: bake the offset into the mesh ---------------------
    //
    // translate() moves vertices, centroids, and the AABB; areas, markers,
    // regions, and adjacency are untouched. Build the Bsp AFTER
    // translating so its AABBs are world-space too.
    let mut baked = local_room();
    baked.translate(WORLD_ORIGIN);
    let bsp = Bsp::build(&baked);
    let path = find_path(&baked, &bsp, start, goal, &PathOptions::default())
        .expect("start/goal are inside the placed room");
    print_path("translate + single-mesh find_path", &path.points);

    // -- Placement 2: keep the mesh local, offset at query time ---------
    //
    // The tile's coordinates stay 0..20 × 0..10; TiledWorld maps between
    // local and world on every query. stitch_all is a no-op for a single
    // tile but is the point of this model: neighbors added later at
    // (10020, 50000) etc. stitch into one routable world.
    let mut world = TiledWorld::new();
    world.add_tile(local_room(), WORLD_ORIGIN);
    world.stitch_all(1e-6);
    let tiled_path = world
        .find_path(start, goal)
        .expect("same query on the tiled placement");
    print_path("\nTiledWorld find_path", &tiled_path);

    let deviation = path
        .points
        .iter()
        .zip(&tiled_path)
        .map(|(a, b)| a.distance(*b))
        .fold(0.0_f64, f64::max);
    println!("\nsame corners from both placements (max deviation {deviation:.2e})");
    println!("bake with translate for one mesh placed once;");
    println!("use TiledWorld to reuse/move tiles and stitch seams between them.");
}
