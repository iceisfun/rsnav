//! Simulate an agent walking an L-shaped path with and without corner
//! avoidance, printing the steering target at each step.
//!
//! Run with:
//!   cargo run -p rsnav-pathing --example follow_path

use rsnav_common::Vertex;
use rsnav_pathing::{FollowerOptions, PathFollower};

fn main() {
    // L-shape: (0,0) → (5,0) → (5,5). 90° left turn at (5,0).
    let path = vec![
        Vertex::new(0.0, 0.0),
        Vertex::new(5.0, 0.0),
        Vertex::new(5.0, 5.0),
    ];

    println!("path: (0,0) → (5,0) → (5,5)  (left turn at the corner)");
    println!("agent walks straight east at speed 0.7 per step.\n");

    // Pretend the agent always lives ON the path and just steps forward
    // at fixed speed. In practice you'd update agent_pos from physics.
    for label in ["no anti-shortcut", "anti-shortcut on"] {
        let opts = if label.starts_with("no") {
            FollowerOptions {
                lookahead: 1.5,
                corner_avoidance: 0.0,
                corner_angle_threshold: 0.1,
            }
        } else {
            FollowerOptions {
                lookahead: 1.5,
                corner_avoidance: 0.8,
                corner_angle_threshold: 0.1,
            }
        };
        let mut follower = PathFollower::new(path.clone());
        let mut agent = Vertex::new(0.0, 0.0);
        println!("─── {} (corner_avoidance = {:.1}) ───", label, opts.corner_avoidance);
        println!("    step  agent_pos           steering target      arc%");
        for step in 0..12 {
            let target = follower.target(agent, &opts);
            println!(
                "    {:>4}  ({:>5.2}, {:>5.2})    ({:>5.2}, {:>5.2})    {:>4.0}%",
                step,
                agent.x,
                agent.y,
                target.x,
                target.y,
                follower.progress() * 100.0,
            );
            if follower.at_end() {
                println!("    reached end");
                break;
            }
            // Move 0.7 units toward the target.
            let dir = (target - agent).normalize_or_zero();
            agent = agent + dir * 0.7;
        }
        println!();
    }
}
