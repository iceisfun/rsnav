//! Plan a path on a bitfield-built navmesh, then actually *walk* it, and
//! then move the same agent by hand into a wall to show what holds it back.
//!
//! Two halves, because a shipped agent needs both:
//!
//! 1. **Following a plan.** `find_path` returns a polyline;
//!    `rsnav_pathing::PathFollower` turns that polyline into a steering
//!    target each tick. Every position the agent visits is checked with
//!    `Bsp::locate` — a follower step that leaves the mesh is a bug.
//! 2. **Free movement.** Nothing about WASD, steering or knockback goes
//!    through the planner, so `PathOptions::distance_from_wall` cannot help.
//!    `WallClearance::clamp` is the runtime equivalent, and the order it
//!    composes with `Bsp::nearest` is load-bearing: snap onto the mesh
//!    first, push off the wall second.
//!
//! Run with:
//!   cargo run --release -p rsnav-navigation --example walk_the_path

use rsnav_bsp::Bsp;
use rsnav_common::{geom::nearest_point_on_segment, Vertex};
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navigation::wall::is_wall_edge_local;
use rsnav_navigation::{find_path, PathOptions, WallClearance};
use rsnav_navmesh::NavMesh;
use rsnav_pathing::{FollowerOptions, PathFollower};
use rsnav_polygon_extract::Bitfield;

/// Rows are given TOP-DOWN for readability; `bitfield_from_ascii` flips them,
/// because `Bitfield` row 0 is the BOTTOM row. '.' is walkable, '#' is wall.
const MAP: &[&str] = &[
    "........................",
    "........................",
    "........................",
    ".........####...........",
    ".........####...........",
    ".........####...........",
    ".........####...........",
    ".........####...........",
    ".........####...........",
    ".........####...........",
    "........................",
    "........................",
];

/// Agent radius in world units. One bitfield cell is 1.0 world unit, so this
/// is 0.4 of a cell.
const AGENT_RADIUS: f64 = 0.4;

fn main() {
    let bf = bitfield_from_ascii(MAP);
    let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
        .expect("the map has walkable area, so the build must succeed");
    let nav = &build.navmesh;
    let bsp = &build.bsp;
    println!(
        "navmesh: {} tris, {} region(s), built in {:.2} ms\n",
        nav.triangle_count(),
        nav.region_count,
        build.build_ms
    );

    // ---------------------------------------------------------------- plan
    let start = Vertex::new(2.5, 5.5);
    let goal = Vertex::new(21.5, 5.5);
    let path = find_path(
        nav,
        bsp,
        start,
        goal,
        &PathOptions {
            distance_from_wall: AGENT_RADIUS,
        },
    )
    .expect("start and goal are both on the mesh and connected");

    println!("planned polyline ({} points):", path.points.len());
    for p in &path.points {
        println!("    ({:>6.3}, {:>6.3})", p.x, p.y);
    }
    println!(
        "    corridor: {} triangles (NOT the same length as points)",
        path.triangles.len()
    );
    println!("    polyline length: {:.3}\n", polyline_length(&path.points));

    // --------------------------------------------------------------- follow
    // Fixed-step integration: 20 Hz, 4.0 world units/second.
    let dt = 1.0 / 20.0;
    let speed = 4.0;
    let opts = FollowerOptions {
        lookahead: 1.0,
        corner_avoidance: 0.0,
        corner_angle_threshold: 0.1,
    };
    let mut follower = PathFollower::new(path.points.clone()).unwrap();
    let mut agent = start;

    println!(
        "walking at {:.1} u/s, dt = {:.3} s, lookahead = {:.1}",
        speed, dt, opts.lookahead
    );
    println!("    tick  agent_pos          steering target     arc%   on mesh");
    let mut tick = 0usize;
    let mut off_mesh_ticks = 0usize;
    loop {
        let target = follower.target(agent, &opts);
        let on_mesh = bsp.locate(nav, agent).is_some();
        if !on_mesh {
            off_mesh_ticks += 1;
        }
        if tick % 15 == 0 || follower.at_end() {
            println!(
                "    {:>4}  ({:>6.3},{:>6.3})   ({:>6.3},{:>6.3})    {:>4.0}%   {}",
                tick,
                agent.x,
                agent.y,
                target.x,
                target.y,
                follower.progress() * 100.0,
                if on_mesh { "yes" } else { "NO" }
            );
        }
        if follower.at_end() {
            break;
        }
        let dir = (target - agent).normalize_or_zero();
        agent = agent + dir * (speed * dt);
        tick += 1;
        assert!(tick < 10_000, "follower failed to terminate");
    }
    println!(
        "    finished in {} ticks; {} tick(s) spent off the mesh; final distance to goal {:.3}\n",
        tick,
        off_mesh_ticks,
        agent.distance(goal)
    );
    assert_eq!(off_mesh_ticks, 0, "a follower step left the navmesh");

    // ------------------------------------------------- free movement / walls
    let clearance = WallClearance::from_navmesh(nav);
    println!(
        "WallClearance: {} wall segments (rebuild on any mesh or door change)",
        clearance.segment_count()
    );

    // Case 1: a hand nudge that stays on the mesh but ends up hugging the
    // wall face at x = 9. clamp() pushes the centre back out to `radius`.
    let hugging = Vertex::new(8.95, 5.5);
    report(nav, bsp, &clearance, "nudge that hugs the wall", hugging);
    let pushed = clearance.clamp(hugging, AGENT_RADIUS);
    report(nav, bsp, &clearance, "  clamp(pos, 0.4)", pushed);

    // Case 2: a hand nudge that overshoots INTO the wall block. The agent is
    // off the mesh entirely, and clamp alone cannot help: the point is more
    // than `radius` from every wall segment, just on the wrong side of one.
    let overshoot = Vertex::new(10.5, 5.5);
    report(nav, bsp, &clearance, "overshoot into the wall", overshoot);
    let clamp_only = clearance.clamp(overshoot, AGENT_RADIUS);
    report(nav, bsp, &clearance, "  clamp alone (WRONG)", clamp_only);

    // The correct order: snap back onto the mesh, THEN push off the wall.
    let snapped = bsp
        .nearest(nav, overshoot)
        .map(|n| n.point)
        .unwrap_or(overshoot);
    report(nav, bsp, &clearance, "  nearest (step 1)", snapped);
    let safe = clearance.clamp(snapped, AGENT_RADIUS);
    report(nav, bsp, &clearance, "  then clamp (step 2)", safe);
}

/// Print a position with the two properties a free-moving agent must keep:
/// on the mesh, and at least `AGENT_RADIUS` from every wall.
fn report(nav: &NavMesh, bsp: &Bsp, wc: &WallClearance, label: &str, p: Vertex) {
    let on_mesh = bsp.locate(nav, p).is_some();
    let d = dist_to_nearest_wall(nav, p);
    let _ = wc;
    println!(
        "    {:<26} ({:>6.3},{:>6.3})  on_mesh={:<5} wall_dist={:>6.3} {}",
        label,
        p.x,
        p.y,
        on_mesh,
        d,
        if on_mesh && d >= AGENT_RADIUS - 1e-9 {
            "ok"
        } else {
            "VIOLATION"
        }
    );
}

/// Smallest distance from `p` to any wall segment of the mesh. A "wall" is
/// exactly what [`is_wall_edge_local`] reports: a constrained edge
/// (`edge_markers[i] != 0`) or a boundary edge (no neighbour).
fn dist_to_nearest_wall(nav: &NavMesh, p: Vertex) -> f64 {
    let mut best = f64::INFINITY;
    for tri in &nav.triangles {
        for i in 0..3 {
            if !is_wall_edge_local(tri, i) {
                continue;
            }
            let (a, b) = tri.edge_vertices(i);
            let d = p.distance(nearest_point_on_segment(nav.vertex(a), nav.vertex(b), p));
            if d < best {
                best = d;
            }
        }
    }
    best
}

fn polyline_length(p: &[Vertex]) -> f64 {
    p.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// ASCII rows, top-down, into a `Bitfield` whose row 0 is the BOTTOM row.
/// `true` is walkable; `false` is wall. See docs/04-units-and-conventions.md.
fn bitfield_from_ascii(rows: &[&str]) -> Bitfield {
    let h = rows.len() as u32;
    let w = rows[0].len() as u32;
    let mut data = vec![false; (w * h) as usize];
    for (r, row) in rows.iter().enumerate() {
        assert_eq!(row.len() as u32, w, "ragged map");
        let bottom_up = h as usize - 1 - r;
        for (c, ch) in row.chars().enumerate() {
            data[bottom_up * w as usize + c] = ch == '.';
        }
    }
    Bitfield::new(w, h, data).expect("dimensions match the data length")
}
