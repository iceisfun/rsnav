//! End-to-end NavWorker walkthrough: spawn a worker, attach a printing
//! `NavListener`, place a "building" by editing the bitfield, wait for
//! the rebuild event, then demolish the building and rebuild again.
//!
//! Demonstrates the typical game-loop integration pattern:
//!   1. The game owns the ground-truth bitfield.
//!   2. When the world changes, push an `Arc<Bitfield>` snapshot to
//!      the worker via `submit_snapshot`.
//!   3. Read telemetry through a `NavListener` (typed events) and/or
//!      `worker.stats()` (counters); call `poll_swap` once per frame
//!      to expose the latest build to game systems.
//!
//! Run with:
//!   cargo run -p rsnav-dynamic --example live_worker

use std::sync::Arc;

use rsnav_dynamic::{BuildOptions, NavEvent, NavListener, NavWorker};
use rsnav_polygon_extract::Bitfield;

const W: u32 = 32;
const H: u32 = 16;

fn all_walkable() -> Vec<bool> {
    vec![true; (W as usize) * (H as usize)]
}

fn paint_rect(data: &mut [bool], col: u32, row: u32, w: u32, h: u32, walkable: bool) {
    for dy in 0..h {
        for dx in 0..w {
            let x = col + dx;
            let y = row + dy;
            if x >= W || y >= H {
                continue;
            }
            data[(y as usize) * (W as usize) + (x as usize)] = walkable;
        }
    }
}

/// Submit a bitfield snapshot and block until the worker either
/// publishes a build at or past `target_gen` or records a failure at
/// the same generation. Returns the resulting build (if any).
fn submit_and_wait(
    worker: &NavWorker,
    target_gen: u64,
    label: &str,
    bf: Bitfield,
) -> Option<Arc<rsnav_dynamic::NavBuild>> {
    println!("[main] submit  : {label}");
    worker.submit_snapshot(Arc::new(bf));
    loop {
        let s = worker.stats();
        if s.last_completed_generation >= target_gen || s.builds_failed >= target_gen {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    worker.latest_published()
}

fn main() {
    // Telemetry listener: a closure is enough thanks to the blanket impl.
    let listener: Arc<dyn NavListener> = Arc::new(|ev: &NavEvent<'_>| match ev {
        NavEvent::BuildStarted { generation } => {
            println!("[worker] start  gen {generation}");
        }
        NavEvent::BuildCompleted {
            generation,
            build_ms,
            triangles,
            regions,
        } => {
            println!(
                "[worker] done   gen {generation}: {build_ms:.2}ms  {triangles}t  {regions}r"
            );
        }
        NavEvent::BuildFailed { generation, error } => {
            println!("[worker] FAIL   gen {generation}: {error}");
        }
    });
    let worker = NavWorker::spawn_with_listener(BuildOptions::default(), listener);

    // ---- 1. Initial arena: 32x16 walkable, no obstacles.
    let mut data = all_walkable();
    let bf = Bitfield::new(W, H, data.clone()).expect("dims match");
    let nav = submit_and_wait(&worker, 1, "initial 32x16 arena", bf);
    if let Some(b) = &nav {
        println!(
            "[main] mesh   : {} tris in {} region(s)",
            b.navmesh.triangle_count(),
            b.navmesh.region_count,
        );
    }

    // ---- 2. Place a 4x4 building near the middle.
    paint_rect(&mut data, 14, 6, 4, 4, /*walkable=*/ false);
    let bf = Bitfield::new(W, H, data.clone()).expect("dims match");
    let nav = submit_and_wait(&worker, 2, "+ building at (14, 6) 4x4", bf);
    if let Some(b) = &nav {
        println!(
            "[main] mesh   : {} tris in {} region(s) — hole carved",
            b.navmesh.triangle_count(),
            b.navmesh.region_count,
        );
    }

    // ---- 3. Demolish the building: clear it back to walkable.
    paint_rect(&mut data, 14, 6, 4, 4, /*walkable=*/ true);
    let bf = Bitfield::new(W, H, data).expect("dims match");
    let nav = submit_and_wait(&worker, 3, "- building demolished", bf);
    if let Some(b) = &nav {
        println!(
            "[main] mesh   : {} tris in {} region(s) — hole gone",
            b.navmesh.triangle_count(),
            b.navmesh.region_count,
        );
    }

    // ---- Final stats.
    let s = worker.stats();
    let avg = if s.builds_completed > 0 {
        s.total_build_ms / s.builds_completed as f64
    } else {
        0.0
    };
    println!();
    println!("---- worker stats ----");
    println!("submitted  : {}", s.snapshots_submitted);
    println!("coalesced  : {}", s.snapshots_coalesced);
    println!("completed  : {}", s.builds_completed);
    println!("failed     : {}", s.builds_failed);
    println!(
        "build ms   : {:.2} avg / {:.2} max / {:.2} last",
        avg, s.max_build_ms, s.last_build_ms
    );
}
