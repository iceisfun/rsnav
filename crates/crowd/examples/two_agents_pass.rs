//! Minimal `rsnav-crowd` walkthrough: build an open arena, drop two
//! agents on opposite ends with opposite goals, and tick the simulation
//! until both arrive. Prints the per-agent positions every 30 ticks
//! and a final summary showing how close the agents ever got.
//!
//! Demonstrates the typical headless integration pattern:
//!   1. Build a [`NavBuild`] (here via `build_navmesh_from_bitfield`,
//!      but in a game this comes from a `NavWorker::poll_swap`).
//!   2. Construct a [`Crowd`] against it.
//!   3. Add agents (`Agent::new` → `Crowd::add_agent`) and assign
//!      goals (`Crowd::set_goal`).
//!   4. Call `Crowd::tick(dt)` each frame; observe state via
//!      `Crowd::agent(id)`.
//!
//! Run with:
//!   cargo run -p rsnav-crowd --example two_agents_pass

use std::sync::Arc;

use rsnav_common::Vertex;
use rsnav_crowd::{Agent, Crowd, CrowdConfig, Goal};
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_polygon_extract::Bitfield;

const W: u32 = 24;
const H: u32 = 10;
const TICKS_MAX: u32 = 600;
const DT: f64 = 1.0 / 60.0;

fn main() {
    // 1) Open arena, no obstacles.
    let bf = Bitfield::new(W, H, vec![true; (W as usize) * (H as usize)])
        .expect("dims match data");
    let nav = Arc::new(
        build_navmesh_from_bitfield(&bf, &BuildOptions::default())
            .expect("walkable arena builds"),
    );
    println!(
        "[main] navmesh: {} tris in {} region(s) (build_ms = {:.2})",
        nav.navmesh.triangle_count(),
        nav.navmesh.region_count,
        nav.build_ms,
    );

    // 2) Crowd with defaults: 16 angular candidates per agent per tick,
    //    1.5 s time-to-collision horizon.
    let mut crowd = Crowd::new(nav, CrowdConfig::default());

    // 3) Two agents at opposite ends, slight y-offset to break perfect
    //    head-on symmetry (lets them deterministically pick sides).
    let agent_a = crowd.add_agent(Agent::new(Vertex::new(3.0, 5.05), 0.4, 2.5));
    let agent_b = crowd.add_agent(Agent::new(Vertex::new(21.0, 4.95), 0.4, 2.5));
    crowd.set_goal(
        agent_a,
        Some(Goal { target: Vertex::new(21.0, 5.0), arrive_radius: 0.5 }),
    );
    crowd.set_goal(
        agent_b,
        Some(Goal { target: Vertex::new(3.0, 5.0), arrive_radius: 0.5 }),
    );

    // 4) Tick loop.
    let mut min_distance = f64::INFINITY;
    let mut arrived_at: Option<u32> = None;
    for t in 1..=TICKS_MAX {
        crowd.tick(DT);
        let a = crowd.agent(agent_a).expect("agent_a present");
        let b = crowd.agent(agent_b).expect("agent_b present");
        let d = a.pos.distance(b.pos);
        if d < min_distance {
            min_distance = d;
        }
        if t % 30 == 0 {
            println!(
                "[t={t:>3}] A=({:>5.2},{:>4.2}) B=({:>5.2},{:>4.2})  Δ={:.2}",
                a.pos.x, a.pos.y, b.pos.x, b.pos.y, d,
            );
        }
        if a.goal.is_none() && b.goal.is_none() {
            arrived_at = Some(t);
            break;
        }
    }

    let sum_radii = 0.4 + 0.4;
    println!();
    println!("---- summary ----");
    match arrived_at {
        Some(t) => println!("both agents arrived in {t} ticks ({:.2} s)", t as f64 * DT),
        None => println!("did not arrive within {TICKS_MAX} ticks"),
    }
    println!(
        "closest approach : {:.2} (sum-of-radii = {:.2})",
        min_distance, sum_radii,
    );
    if min_distance + 0.05 >= sum_radii {
        println!("→ avoidance held: discs never overlapped during the pass.");
    } else {
        println!("→ avoidance grazed; tune CrowdConfig::avoid_weight upward.");
    }
}
