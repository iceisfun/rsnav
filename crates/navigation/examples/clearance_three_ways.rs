//! One map, one agent radius, all three ways of keeping the agent off the
//! walls — with the achieved clearance *measured* rather than asserted.
//!
//! The three mechanisms look interchangeable from the outside and choosing
//! wrong fails silently, so this example puts them side by side on the same
//! geometry and reports what each one actually delivers:
//!
//!   1. baked contour inset   `BuildOptions::inset = Some(r)`   (cells)
//!   2. baked grid erosion    `Bitfield::eroded(radius: r)`     (cells)
//!   3. query-time clearance  `PathOptions::distance_from_wall` (world units)
//!   4. query-time clamp      `WallClearance::clamp(p, r)`      (world units)
//!
//! The measured column is the point of the example. Every run is measured
//! against the *same* wall set — the walls of the un-eroded baseline mesh,
//! i.e. the real geometry — so the numbers are directly comparable. Distance
//! is the exact segment-to-segment distance from each leg of the returned
//! polyline to each wall segment, not a sampled approximation.
//!
//! Run:
//!     cargo run --release -p rsnav-navigation --example clearance_three_ways
//!     cargo run --release -p rsnav-navigation --example clearance_three_ways -- 1.5

use std::time::Instant;

use rsnav_common::geom::{nearest_point_on_segment, segments_intersect, SegmentIntersection};
use rsnav_common::Vertex;
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navigation::wall::is_wall_edge_local;
use rsnav_navigation::{find_path, PathOptions, WallClearance};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::{Bitfield, ErodeOptions};

/// '#' is wall, '.' is walkable. Rows are given top-down and flipped on
/// load, because `Bitfield` row 0 is the BOTTOM row (see
/// docs/04-units-and-conventions.md).
///
/// A four-cell-tall gap between two diagonal wedges. The diagonals matter:
/// they produce portals that meet the wall at a shallow angle, which is
/// exactly where `distance_from_wall` under-delivers.
const MAP: &[&str] = &[
    "########################################",
    "#......................................#",
    "#......................................#",
    "#......................................#",
    "#...............#######................#",
    "#..............########................#",
    "#.............#########................#",
    "#............##########................#",
    "#......................................#",
    "#......................................#",
    "#......................................#",
    "#......................................#",
    "#............##########................#",
    "#.............#########................#",
    "#..............########................#",
    "#...............#######................#",
    "#......................................#",
    "########################################",
];

fn load_map() -> Bitfield {
    let h = MAP.len() as u32;
    let w = MAP[0].len() as u32;
    let mut data = vec![false; (w as usize) * (h as usize)];
    for (i, line) in MAP.iter().enumerate() {
        let row = (h as usize) - 1 - i; // top-down text -> bottom-up grid
        for (col, ch) in line.chars().enumerate() {
            data[row * (w as usize) + col] = ch == '.';
        }
    }
    Bitfield::new(w, h, data).expect("map rows are all the same length")
}

fn walkable(bf: &Bitfield) -> usize {
    bf.data.iter().filter(|&&b| b).count()
}

/// Every wall segment of `nav`, deduplicated. A wall is a constrained edge
/// or a boundary edge — the same set A*, the funnel and `WallClearance` use.
fn wall_segments(nav: &NavMesh) -> Vec<(Vertex, Vertex)> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for tri in &nav.triangles {
        for i in 0..3 {
            if !is_wall_edge_local(tri, i) {
                continue;
            }
            let (a, b) = tri.edge_vertices(i);
            let key = if a.index() <= b.index() {
                (a.index(), b.index())
            } else {
                (b.index(), a.index())
            };
            if seen.insert(key) {
                out.push((nav.vertex(a), nav.vertex(b)));
            }
        }
    }
    out
}

/// Exact distance between two segments in 2D. If they cross, it is 0;
/// otherwise the minimum is attained at an endpoint of one of them.
fn seg_seg_distance(a1: Vertex, a2: Vertex, b1: Vertex, b2: Vertex) -> f64 {
    match segments_intersect(a1, a2, b1, b2) {
        SegmentIntersection::None => {}
        _ => return 0.0,
    }
    let d = |p: Vertex, q1: Vertex, q2: Vertex| p.distance(nearest_point_on_segment(q1, q2, p));
    d(a1, b1, b2)
        .min(d(a2, b1, b2))
        .min(d(b1, a1, a2))
        .min(d(b2, a1, a2))
}

/// Minimum distance from a polyline to any wall segment, plus the index of
/// the leg where it is attained (the "witness").
fn min_clearance(
    points: &[Vertex],
    walls: &[(Vertex, Vertex)],
) -> (f64, usize, (Vertex, Vertex)) {
    let mut best = f64::INFINITY;
    let mut leg_idx = 0;
    let mut wall = (Vertex::ZERO, Vertex::ZERO);
    for (i, leg) in points.windows(2).enumerate() {
        for &(w1, w2) in walls {
            let d = seg_seg_distance(leg[0], leg[1], w1, w2);
            if d < best {
                best = d;
                leg_idx = i;
                wall = (w1, w2);
            }
        }
    }
    (best, leg_idx, wall)
}

fn path_length(points: &[Vertex]) -> f64 {
    points.windows(2).map(|s| s[0].distance(s[1])).sum()
}

struct Row {
    label: &'static str,
    build_ms: f64,
    tris: usize,
    cells: usize,
    pts: usize,
    len: f64,
    clearance: f64,
    witness: usize,
    wall: (Vertex, Vertex),
    points: Vec<Vertex>,
}

fn main() {
    let r: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1.0);

    let bf = load_map();
    let start = Vertex::new(3.0, 2.5);
    let goal = Vertex::new(36.0, 15.5);

    // The reference geometry: an un-eroded build. Its walls are the truth
    // every row below is measured against.
    let base = build_navmesh_from_bitfield(&bf, &BuildOptions::default()).expect("baseline build");
    let true_walls = wall_segments(&base.navmesh);

    println!("map {}x{}  walkable cells {}", bf.width, bf.height, walkable(&bf));
    println!("agent radius r = {r} (cells for the baked options, world units for the query options)");
    println!("start {:?} -> goal {:?}", (start.x, start.y), (goal.x, goal.y));
    println!("reference wall segments: {}", true_walls.len());
    println!();

    let mut rows = Vec::new();

    // ---- 0. no clearance at all -------------------------------------
    {
        let p = find_path(
            &base.navmesh,
            &base.bsp,
            start,
            goal,
            &PathOptions::default(),
        )
        .expect("baseline path");
        rows.push(Row {
            label: "none (baseline)",
            build_ms: base.build_ms,
            tris: base.navmesh.triangles.len(),
            cells: walkable(&bf),
            pts: p.points.len(),
            len: path_length(&p.points),
            clearance: min_clearance(&p.points, &true_walls).0,
                witness: min_clearance(&p.points, &true_walls).1,
                wall: min_clearance(&p.points, &true_walls).2,
                points: p.points.clone(),
        });
    }

    // ---- 1. baked contour inset -------------------------------------
    {
        let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default().with_inset(r))
            .expect("inset build");
        let p = find_path(
            &build.navmesh,
            &build.bsp,
            start,
            goal,
            &PathOptions::default(),
        );
        match p {
            Ok(p) => rows.push(Row {
                label: "inset (contour)",
                build_ms: build.build_ms,
                tris: build.navmesh.triangles.len(),
                cells: walkable(&bf),
                pts: p.points.len(),
                len: path_length(&p.points),
                clearance: min_clearance(&p.points, &true_walls).0,
                witness: min_clearance(&p.points, &true_walls).1,
                wall: min_clearance(&p.points, &true_walls).2,
                points: p.points.clone(),
            }),
            Err(e) => println!("inset  r={r}: no path ({e:?}) — the start/goal fell outside the eroded mesh"),
        }
    }

    // ---- 2. baked grid erosion --------------------------------------
    {
        let t = Instant::now();
        let eroded = bf
            .eroded(&ErodeOptions { radius: r, threads: 0 })
            .expect("finite non-negative radius");
        let erode_ms = t.elapsed().as_secs_f64() * 1e3;
        let build = build_navmesh_from_bitfield(&eroded, &BuildOptions::default());
        match build {
            Ok(build) => {
                let p = find_path(
                    &build.navmesh,
                    &build.bsp,
                    start,
                    goal,
                    &PathOptions::default(),
                );
                match p {
                    Ok(p) => rows.push(Row {
                        label: "eroded (grid)",
                        build_ms: erode_ms + build.build_ms,
                        tris: build.navmesh.triangles.len(),
                        cells: walkable(&eroded),
                        pts: p.points.len(),
                        len: path_length(&p.points),
                        clearance: min_clearance(&p.points, &true_walls).0,
                witness: min_clearance(&p.points, &true_walls).1,
                wall: min_clearance(&p.points, &true_walls).2,
                points: p.points.clone(),
                    }),
                    Err(e) => println!(
                        "eroded r={r}: no path ({e:?}) — start/goal fell outside the eroded mesh \
                         ({} walkable cells left of {})",
                        walkable(&eroded),
                        walkable(&bf)
                    ),
                }
            }
            Err(e) => println!("eroded r={r}: build failed ({e})"),
        }
    }

    // ---- 3. query-time distance_from_wall ---------------------------
    {
        let opts = PathOptions {
            distance_from_wall: r,
        };
        match find_path(&base.navmesh, &base.bsp, start, goal, &opts) {
            Ok(p) => rows.push(Row {
                label: "distance_from_wall",
                build_ms: base.build_ms,
                tris: base.navmesh.triangles.len(),
                cells: walkable(&bf),
                pts: p.points.len(),
                len: path_length(&p.points),
                clearance: min_clearance(&p.points, &true_walls).0,
                witness: min_clearance(&p.points, &true_walls).1,
                wall: min_clearance(&p.points, &true_walls).2,
                points: p.points.clone(),
            }),
            Err(e) => println!("distance_from_wall r={r}: no path ({e:?})"),
        }
    }

    // ---- 4. the same path, run through WallClearance::clamp ---------
    // Not a substitute for the above: clamp is for FREE movement, and the
    // ordering rule (bsp.nearest, THEN clamp) belongs to
    // docs/08-moving-agents.md. It is applied here only so the guarantee it
    // does enforce — a true distance-to-segment invariant — is visible in
    // the same column as the ones that do not.
    {
        let opts = PathOptions {
            distance_from_wall: r,
        };
        if let Ok(p) = find_path(&base.navmesh, &base.bsp, start, goal, &opts) {
            let clearance = WallClearance::from_navmesh(&base.navmesh);
            let t = Instant::now();
            let clamped: Vec<Vertex> = p
                .points
                .iter()
                .map(|&q| {
                    let on_mesh = base
                        .bsp
                        .nearest(&base.navmesh, q)
                        .map(|n| n.point)
                        .unwrap_or(q);
                    clearance.clamp(on_mesh, r)
                })
                .collect();
            let clamp_us = t.elapsed().as_secs_f64() * 1e6 / (p.points.len() as f64);
            let moved = p
                .points
                .iter()
                .zip(&clamped)
                .filter(|(a, b)| a.distance(**b) > 1e-12)
                .count();
            let vtx_min = |ps: &[Vertex]| {
                ps.iter()
                    .map(|&q| {
                        true_walls
                            .iter()
                            .map(|&(w1, w2)| q.distance(nearest_point_on_segment(w1, w2, q)))
                            .fold(f64::INFINITY, f64::min)
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            println!(
                "clamp moved {}/{} polyline vertices; min VERTEX-to-wall distance \
                 {:.4} -> {:.4}",
                moved,
                p.points.len(),
                vtx_min(&p.points),
                vtx_min(&clamped)
            );
            rows.push(Row {
                label: "  + clamp corners",
                build_ms: base.build_ms,
                tris: base.navmesh.triangles.len(),
                cells: walkable(&bf),
                pts: clamped.len(),
                len: path_length(&clamped),
                clearance: min_clearance(&clamped, &true_walls).0,
                witness: min_clearance(&clamped, &true_walls).1,
                wall: min_clearance(&clamped, &true_walls).2,
                points: clamped.clone(),
            });
            println!(
                "WallClearance: {} wall segments, clamp cost {:.1} us/call \
                 (linear scan over every segment, {} relaxation passes)",
                clearance.segment_count(),
                clamp_us,
                4
            );
            println!();
        }
    }

    println!(
        "{:<20} {:>9} {:>6} {:>7} {:>5} {:>8} {:>10} {:>5}",
        "mechanism", "build ms", "tris", "cells", "pts", "length", "min clear", "leg"
    );
    for row in &rows {
        println!(
            "{:<20} {:>9.2} {:>6} {:>7} {:>5} {:>8.3} {:>10.4} {:>5}",
            row.label, row.build_ms, row.tris, row.cells, row.pts, row.len, row.clearance, row.witness
        );
    }

    println!();
    for row in &rows {
        let pts: Vec<String> = row
            .points
            .iter()
            .map(|p| format!("({:.3},{:.3})", p.x, p.y))
            .collect();
        println!(
            "{:<20} {}\n{:<20}   closest wall segment ({:.1},{:.1})-({:.1},{:.1}) on leg {}",
            row.label,
            pts.join(" -> "),
            "",
            row.wall.0.x,
            row.wall.0.y,
            row.wall.1.x,
            row.wall.1.y,
            row.witness
        );
    }

    println!();
    println!("Reading the table:");
    println!("  'min clear' is the measured minimum distance from the returned polyline to the");
    println!("  reference geometry's walls. Compare each against the requested r = {r}.");
    println!();
    println!("  none    the funnel apexes exactly on wall vertices, so the clearance is 0 by");
    println!("          construction. This is what every other row is trying to fix.");
    println!("  inset   meets r along straight walls but falls slightly short near a wall corner");
    println!("          that protrudes into walkable space: the offset stage never generates");
    println!("          arcs, so a convex obstacle corner is mitered or beveled and the join cuts");
    println!("          inside the true r-offset arc. Check the reported witness segment.");
    println!("  eroded  meets or exceeds r, and over-delivers whenever r is not already one of");
    println!("          the achievable clearances {{0, 1, sqrt2, 2, sqrt5, ...}}. Note the cells");
    println!("          column: erosion is paid on the grid, so walkable area drops permanently.");
    println!("  d_f_w   under-delivers. The funnel shifts a portal endpoint ALONG the portal by r,");
    println!("          not along the wall normal; the path then leaves at an angle, so its");
    println!("          perpendicular distance to the corner is only about r*sin(theta).");
    println!("  clamp   moved zero vertices here, because every polyline VERTEX was already r");
    println!("          clear -- the shortfall is in the INTERIOR of a leg. clamp enforces its");
    println!("          distance-to-segment invariant at the points you hand it and nowhere else,");
    println!("          which is why it is a free-movement tool and not a path post-process.");
    println!();
    println!("The path endpoints are the literal start and goal and are never adjusted for");
    println!("clearance, so a start placed against a wall caps 'min clear' at 0 for every row.");
}
