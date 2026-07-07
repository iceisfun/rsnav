//! Diagnostic (ignored): dump the bridge fixture's decomposition.

use rsnav_common::{Vec3, Vertex};
use rsnav_layers::{build_layered_world, LayersConfig};
use rsnav_voxel::{synth, PolySoup};

fn bridge_soup() -> PolySoup {
    let mut soup = synth::plane(24.0, 16.0, 1);
    let deck = {
        let mut d = PolySoup::new();
        d.vertices = vec![
            Vec3::new(-6.0, -2.0, 2.5),
            Vec3::new(6.0, -2.0, 2.5),
            Vec3::new(6.0, 2.0, 2.5),
            Vec3::new(-6.0, 2.0, 2.5),
        ];
        d.triangles = vec![[0, 1, 2], [0, 2, 3]];
        d
    };
    soup.append(&deck);
    soup.append(&synth::ramp(-11.0, 5.0, 2.5, 4.0));
    let mut east = synth::ramp(6.0, 5.0, 2.5, 4.0);
    for v in &mut east.vertices {
        v.x = 17.0 - v.x;
    }
    soup.append(&east);
    soup
}

#[test]
#[ignore]
fn dump_bridge() {
    let cfg = LayersConfig { voxel_size: 0.2, ..LayersConfig::default() };
    let built = build_layered_world(&bridge_soup(), &cfg).unwrap();
    eprintln!("origin {:?} cell {}", built.origin, built.cell_size);
    eprintln!(
        "layers {} connections {} seam_links {} pruned {}",
        built.stats.per_layer.len(),
        built.stats.connections,
        built.stats.seam_links,
        built.stats.pruned_spans
    );
    for (i, l) in built.stats.per_layer.iter().enumerate() {
        eprintln!("  L{i}: spans {} tris {} area {:.1}", l.spans, l.triangles, l.walkable_area);
    }
    let w = &built.world;
    for c in w.connections() {
        eprintln!(
            "conn {}: {} sub-edges, layers ({},{})",
            c.id,
            c.sub_edges.len(),
            c.sub_edges[0].a.layer,
            c.sub_edges[0].b.layer
        );
    }
    let local = |x: f64, y: f64| Vertex::new(x - built.origin.x, y - built.origin.y);
    for (name, x, y, z) in [
        ("north", 0.0, 6.0, 0.0),
        ("south", 0.0, -6.0, 0.0),
        ("under", 0.0, 0.0, 0.0),
        ("deck", 0.0, 0.0, 2.5),
        ("west-ramp-mid", -8.5, 0.0, 1.25),
    ] {
        let n = w.nearest(local(x, y), z).unwrap();
        eprintln!(
            "{name}: layer {} at {:?} z {:.2} dist {:.2}",
            n.layer, n.point, n.z, n.distance
        );
    }
}
