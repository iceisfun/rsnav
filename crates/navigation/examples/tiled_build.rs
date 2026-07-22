//! The recommended tiled workflow, end to end: erode the GLOBAL grid, slice
//! it into tiles, build each tile independently, place, stitch, path across
//! the seams.
//!
//! Run with:
//!   cargo run --release -p rsnav-navigation --example tiled_build
//!
//! The load-bearing content is the ORDER of two calls:
//!
//!   1. `Bitfield::eroded` on the whole world grid — once, globally.
//!   2. `Bitfield::subgrid` per tile.
//!
//! Doing it the other way round (subgrid, then erode each tile) treats the
//! tile border as wall — `Bitfield::at` reads out-of-range as `false` — and
//! eats `radius` cells at every seam. Both tiles then have zero boundary
//! edges on the seam line, `stitch_all` matches nothing, and `find_path`
//! across the seam returns `None` with no error anywhere. This example runs
//! both orders and prints the difference.
//!
//! `BuildOptions::inset` (contour erosion) is the same failure by a
//! different route and must stay `None` for tiled builds; that direction is
//! pinned as a negative control in
//! crates/navigation/tests/tiled_erosion_seams.rs.

use rsnav_common::Vertex;
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navigation::TiledWorld;
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::{Bitfield, ErodeOptions};

/// World is TILE*2 x TILE*2 cells, cut into four TILE x TILE tiles.
const TILE: u32 = 32;
const AGENT_RADIUS: f64 = 1.0;

/// A 64x64 map: one large walkable area with two pillars, deliberately
/// arranged so a corner-to-corner route crosses both seams.
fn world_grid() -> Bitfield {
    let n = TILE * 2;
    let mut bits = Bitfield::empty(n, n);
    for r in 2..(n - 2) {
        for c in 2..(n - 2) {
            bits.set(c, r, true);
        }
    }
    // Two pillars, away from the seam lines so the seams stay clean.
    for r in 8..22 {
        for c in 8..22 {
            bits.set(c, r, false);
        }
    }
    for r in 42..56 {
        for c in 42..56 {
            bits.set(c, r, false);
        }
    }
    bits
}

/// Build one tile. `min_area = 0.0` and `clip_ears_max_area = 0.0` because
/// both act asymmetrically between neighbours and can drop a seam-adjacent
/// fragment in one tile only.
fn build_tile(bits: &Bitfield) -> Option<NavMesh> {
    let mut opts = BuildOptions::default();
    opts.inset = None; // required for tiled builds
    opts.extract.min_area = 0.0;
    opts.clip_ears_max_area = 0.0;
    build_navmesh_from_bitfield(bits, &opts)
        .ok()
        .map(|b| b.navmesh)
}

/// Boundary edges of `nav` lying entirely on the world line `x = line_x`
/// once the tile sits at `offset`. This is the diagnostic to reach for when
/// a seam fails to link: if it is 0 on either side, the geometry is gone and
/// no tolerance will recover it. (Same helper as `seam_edges` in
/// crates/navigation/tests/tiled_erosion_seams.rs.)
fn seam_edges_x(nav: &NavMesh, offset: Vertex, line_x: f64) -> usize {
    nav.boundary_edges()
        .filter(|be| {
            let (a, b) = (nav.vertex(be.from), nav.vertex(be.to));
            (a.x + offset.x - line_x).abs() < 1e-9 && (b.x + offset.x - line_x).abs() < 1e-9
        })
        .count()
}

/// Place four tiles cut from `source` and stitch them.
/// `erode_per_tile` selects the WRONG order for contrast.
fn assemble(source: &Bitfield, radius: f64, erode_per_tile: bool) -> (TiledWorld, usize) {
    let global = if erode_per_tile {
        source.clone()
    } else {
        source
            .eroded(&ErodeOptions {
                radius,
                threads: 0,
            })
            .expect("finite non-negative radius")
    };

    let mut world = TiledWorld::new();
    let mut seam = 0usize;
    for ty in 0..2u32 {
        for tx in 0..2u32 {
            let mut cells = global.subgrid(tx * TILE, ty * TILE, TILE, TILE);
            if erode_per_tile {
                cells = cells
                    .eroded(&ErodeOptions {
                        radius,
                        threads: 0,
                    })
                    .expect("finite non-negative radius");
            }
            let Some(nav) = build_tile(&cells) else {
                continue;
            };
            let offset = Vertex::new((tx * TILE) as f64, (ty * TILE) as f64);
            seam += seam_edges_x(&nav, offset, TILE as f64);
            world.add_tile(nav, offset);
        }
    }
    // 1e-9: the tiles are cut on exact integer cell boundaries, so the
    // matching is exact and the tolerance only absorbs f64 noise.
    world.stitch_all(1e-9);
    (world, seam)
}

fn report(label: &str, world: &TiledWorld, seam: usize, start: Vertex, goal: Vertex) {
    println!("{label}");
    println!("  tiles            {}", world.tile_count());
    println!("  seam edges @x=32 {seam}");
    println!("  cross-tile links {}", world.links().len());
    match world.find_path(start, goal) {
        Some(pts) => {
            let len: f64 = pts.windows(2).map(|w| w[0].distance(w[1])).sum();
            println!("  path             {} points, length {len:.2}", pts.len());
            let head: Vec<String> = pts
                .iter()
                .take(4)
                .map(|p| format!("({:.1}, {:.1})", p.x, p.y))
                .collect();
            println!("  first corners    {}", head.join(" -> "));
        }
        None => println!("  path             None"),
    }
    println!();
}

fn main() {
    let source = world_grid();
    let start = Vertex::new(5.0, 5.0);
    let goal = Vertex::new(58.0, 58.0);

    println!(
        "world {}x{} cells, {}x{} tiles of {TILE}, agent radius {AGENT_RADIUS} cells\n",
        TILE * 2,
        TILE * 2,
        2,
        2
    );

    // Control: no clearance at all. Establishes that the placement and
    // stitching themselves are sound.
    let (w, seam) = assemble(&source, 0.0, false);
    report("no erosion (control)", &w, seam, start, goal);

    // The recommended workflow.
    let (w, seam) = assemble(&source, AGENT_RADIUS, false);
    report(
        "erode globally, then subgrid  <- do this",
        &w,
        seam,
        start,
        goal,
    );

    // The failure this page exists to prevent.
    let (w, seam) = assemble(&source, AGENT_RADIUS, true);
    report(
        "subgrid, then erode each tile  <- broken, silently",
        &w,
        seam,
        start,
        goal,
    );

    println!(
        "The clearance in the middle case is baked into the grid, so it survives\n\
         the seams. TiledWorld::find_path takes no PathOptions and applies no\n\
         clearance of its own; global grid erosion is the only route to an agent\n\
         radius through this surface."
    );
}
