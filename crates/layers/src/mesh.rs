//! Per-layer meshing: tagged outlines → conformed PSLG → CDT → NavMesh.

use std::collections::{BTreeMap, HashMap};

use rsnav_common::{Polygon, Vertex};
use rsnav_navmesh::{build_from_cdt, connection_marker, NavMesh};
use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
use rsnav_triangle::{
    carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, SegmentInsertError, VertexSlot,
};

use crate::assign::Assignment;
use crate::outline::{trace_layer_outline, vertex_z, EdgeTag};

/// Marker used for ordinary wall segments.
pub const WALL_MARKER: i32 = 1;

#[derive(Debug)]
pub enum LayerMeshError {
    /// A boundary vertex had no height cluster — the layer's cell data
    /// is inconsistent (should not happen).
    MissingHeight { layer: u32, at: (i32, i32) },
    /// The CDT rejected a boundary segment. Grid outlines only meet at
    /// shared vertices, so this indicates a tracing bug.
    SegmentInsertion(SegmentInsertError),
    /// A hole loop was too degenerate to seed.
    DegenerateHole { layer: u32 },
    /// The carved layer contains no triangles.
    EmptyLayer { layer: u32 },
}

impl std::fmt::Display for LayerMeshError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LayerMeshError::MissingHeight { layer, at } => {
                write!(f, "layer {layer}: no height cluster at grid vertex {at:?}")
            }
            LayerMeshError::SegmentInsertion(e) => write!(f, "segment insertion failed: {e:?}"),
            LayerMeshError::DegenerateHole { layer } => {
                write!(f, "layer {layer}: hole loop with no interior point")
            }
            LayerMeshError::EmptyLayer { layer } => {
                write!(f, "layer {layer}: carve left no walkable triangles")
            }
        }
    }
}

impl std::error::Error for LayerMeshError {}

/// Build one layer's navmesh.
///
/// `pair_ids` maps an unordered layer pair to its connection id — the
/// same table is used for every layer, so the two sides of a seam tag
/// their segments with the same `connection_marker`.
pub fn build_layer_mesh(
    asg: &Assignment,
    cells: &HashMap<(u32, u32), u32>,
    layer: u32,
    cell_size: f64,
    max_step_layers: u32,
    sample_step: u32,
    pair_ids: &HashMap<(u32, u32), u32>,
) -> Result<NavMesh, LayerMeshError> {
    let loops = trace_layer_outline(asg, cells, layer);
    let to_local = |(i, j): (i32, i32)| Vertex::new(i as f64 * cell_size, j as f64 * cell_size);
    let range = |c: u32, r: u32| asg.cell_spans(c, r);

    // --- Vertex table: boundary vertices first, deterministic order. ---
    let mut index: BTreeMap<(i32, i32), u32> = BTreeMap::new();
    for lp in &loops {
        for &p in &lp.points {
            let next = index.len() as u32;
            index.entry(p).or_insert(next);
        }
    }
    let mut grid_vertices: Vec<(i32, i32)> = vec![(0, 0); index.len()];
    for (&p, &i) in &index {
        grid_vertices[i as usize] = p;
    }

    // Heights for every boundary vertex.
    let mut zs: Vec<f64> = Vec::with_capacity(grid_vertices.len());
    for &(i, j) in &grid_vertices {
        let z = vertex_z(asg, &range, layer, i, j, max_step_layers)
            .ok_or(LayerMeshError::MissingHeight { layer, at: (i, j) })?;
        zs.push(z);
    }

    // --- Segments with markers. ---
    let mut segments: Vec<PslgSegment> = Vec::new();
    for lp in &loops {
        let n = lp.points.len();
        for k in 0..n {
            let a = index[&lp.points[k]];
            let b = index[&lp.points[(k + 1) % n]];
            let marker = match lp.tags[k] {
                EdgeTag::Wall => WALL_MARKER,
                EdgeTag::Seam { other } => {
                    let key = (layer.min(other), layer.max(other));
                    connection_marker(pair_ids[&key])
                }
            };
            segments.push(PslgSegment { a, b, marker });
        }
    }

    // --- Hole seeds. ---
    let mut holes: Vec<PslgHole> = Vec::new();
    for lp in loops.iter().filter(|l| !l.is_outer()) {
        let poly = Polygon {
            vertices: lp.points.iter().map(|&p| to_local(p)).collect(),
        };
        let seed = poly
            .interior_point()
            .ok_or(LayerMeshError::DegenerateHole { layer })?;
        holes.push(PslgHole { point: seed });
    }

    // --- Interior height samples. ---
    //
    // Boundary vertices alone leave big flat triangles that can't
    // represent interior relief; sample the surface on a sparse grid.
    // A vertex qualifies when all four adjacent cells belong to this
    // layer (so it can't sit on a constrained segment) and the local
    // surface isn't flat (flat interiors interpolate exactly from the
    // boundary anyway).
    let mut samples: Vec<((i32, i32), f64)> = Vec::new();
    if sample_step > 0 {
        let step = sample_step as i32;
        // Deterministic iteration over the layer's cell bounding box.
        let (mut min_c, mut min_r, mut max_c, mut max_r) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
        for &(c, r) in cells.keys() {
            min_c = min_c.min(c as i32);
            min_r = min_r.min(r as i32);
            max_c = max_c.max(c as i32);
            max_r = max_r.max(r as i32);
        }
        for j in (min_r + 1)..=(max_r) {
            for i in (min_c + 1)..=(max_c) {
                if i % step != 0 || j % step != 0 {
                    continue;
                }
                if index.contains_key(&(i, j)) {
                    continue;
                }
                let mut own_z: Vec<f64> = Vec::with_capacity(4);
                let mut interior = true;
                for (dc, dr) in [(-1i32, -1i32), (0, -1), (-1, 0), (0, 0)] {
                    let (c, r) = (i + dc, j + dr);
                    if c < 0 || r < 0 {
                        interior = false;
                        break;
                    }
                    match cells.get(&(c as u32, r as u32)) {
                        Some(&sid) => own_z.push(asg.spans[sid as usize].z),
                        None => {
                            interior = false;
                            break;
                        }
                    }
                }
                if !interior {
                    continue;
                }
                let (min_z, max_z) = own_z
                    .iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &z| {
                        (lo.min(z), hi.max(z))
                    });
                if max_z - min_z < 1e-9 {
                    continue; // locally flat — boundary interpolation is exact
                }
                let z = vertex_z(asg, &range, layer, i, j, max_step_layers)
                    .ok_or(LayerMeshError::MissingHeight { layer, at: (i, j) })?;
                samples.push(((i, j), z));
            }
        }
    }

    // --- CDT pipeline. ---
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    let mut zmap: HashMap<(u64, u64), f64> = HashMap::new();
    for (k, &(i, j)) in grid_vertices.iter().enumerate() {
        let p = to_local((i, j));
        cdt.push_vertex(VertexSlot::new(p, 0));
        pslg.vertices.push(PslgVertex::new(p));
        zmap.insert((p.x.to_bits(), p.y.to_bits()), zs[k]);
    }
    for &((i, j), z) in &samples {
        let p = to_local((i, j));
        cdt.push_vertex(VertexSlot::new(p, 0));
        pslg.vertices.push(PslgVertex::new(p));
        zmap.insert((p.x.to_bits(), p.y.to_bits()), z);
    }
    pslg.segments = segments;
    pslg.holes = holes;

    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).map_err(LayerMeshError::SegmentInsertion)?;
    carve_holes(&mut cdt, &pslg, false);
    let mut nav = build_from_cdt(&cdt);
    if nav.triangle_count() == 0 {
        return Err(LayerMeshError::EmptyLayer { layer });
    }
    nav.assign_vertex_z(|v| {
        *zmap
            .get(&(v.x.to_bits(), v.y.to_bits()))
            .expect("CDT invents no vertices: every navmesh vertex was an input")
    });
    Ok(nav)
}
