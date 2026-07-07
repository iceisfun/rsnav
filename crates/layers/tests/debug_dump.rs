//! Diagnostic (ignored): dump seam-tagged boundary runs per layer.

use rsnav_layers::{assign_layers, UNASSIGNED};
use rsnav_voxel::{classify_walkability, synth, VoxelGrid, WalkabilityConfig};

#[test]
#[ignore]
fn dump_platform_seams() {
    let soup = synth::floor_with_ramp_and_platform();
    let wcfg = WalkabilityConfig::default();
    let grid = VoxelGrid::from_polysoup(&soup, 0.2, 1, wcfg.cos_max_slope());
    let chf = classify_walkability(&grid, &wcfg);
    let asg = assign_layers(&chf, wcfg.max_step_layers(grid.cell_size), 8);
    eprintln!("layers: {}", asg.layer_count);

    // Every cross-layer link with its cells and layers.
    for (s, span) in asg.spans.iter().enumerate() {
        for (k, &t) in span.links.iter().enumerate() {
            let Some(t) = t else { continue };
            let (a, b) = (asg.layer_of[s], asg.layer_of[t as usize]);
            if a != b && a != UNASSIGNED && b != UNASSIGNED {
                let o = &asg.spans[t as usize];
                eprintln!(
                    "cut link: L{a}({},{}) z={:.2} -[{k}]-> L{b}({},{}) z={:.2}",
                    span.c, span.r, span.z, o.c, o.r, o.z
                );
            }
        }
    }

    // Trace outlines of each layer and print seam segments.
    use std::collections::HashMap;
    let mut cells: Vec<HashMap<(u32, u32), u32>> = vec![HashMap::new(); asg.layer_count as usize];
    for (sid, &l) in asg.layer_of.iter().enumerate() {
        if l != UNASSIGNED {
            let sp = &asg.spans[sid];
            cells[l as usize].insert((sp.c, sp.r), sid as u32);
        }
    }
    let range = |c: u32, r: u32| asg.cell_spans(c, r);
    for layer in 0..asg.layer_count {
        let loops = rsnav_layers::outline::trace_layer_outline(&asg, &cells[layer as usize], layer);
        for lp in &loops {
            let n = lp.points.len();
            for k in 0..n {
                if let rsnav_layers::outline::EdgeTag::Seam { other } = lp.tags[k] {
                    let a = lp.points[k];
                    let b = lp.points[(k + 1) % n];
                    let za = rsnav_layers::outline::vertex_z(
                        &asg, &range, layer, a.0, a.1, wcfg.max_step_layers(grid.cell_size),
                    );
                    let zb = rsnav_layers::outline::vertex_z(
                        &asg, &range, layer, b.0, b.1, wcfg.max_step_layers(grid.cell_size),
                    );
                    eprintln!(
                        "L{layer} seam({other}): {:?} z={:?} -> {:?} z={:?}",
                        a, za, b, zb
                    );
                }
            }
        }
    }
}
