//! Planar Straight Line Graph: the input to a CDT.
//!
//! A PSLG is a set of vertices, a set of straight-line segments connecting
//! pairs of vertices (the constraints), and a set of "hole" points used to
//! mark regions that should be carved out of the final triangulation.

use rsnav_common::Vertex;

/// One input vertex. Carries optional per-vertex attributes and a boundary
/// marker (Triangle's `-A` / `-B` conventions).
#[derive(Clone, Debug, PartialEq)]
pub struct PslgVertex {
    pub position: Vertex,
    pub attributes: Vec<f64>,
    pub marker: i32,
}

impl PslgVertex {
    pub fn new(position: Vertex) -> Self {
        Self {
            position,
            attributes: Vec::new(),
            marker: 0,
        }
    }

    pub fn with_marker(mut self, marker: i32) -> Self {
        self.marker = marker;
        self
    }

    pub fn with_attribute(mut self, attr: f64) -> Self {
        self.attributes.push(attr);
        self
    }
}

/// One input segment (a straight-line constraint between two vertices).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PslgSegment {
    /// Index of the first endpoint in [`Pslg::vertices`].
    pub a: u32,
    /// Index of the second endpoint in [`Pslg::vertices`].
    pub b: u32,
    pub marker: i32,
}

impl PslgSegment {
    pub fn new(a: u32, b: u32) -> Self {
        Self { a, b, marker: 0 }
    }
}

/// A point that marks a hole. Every triangle of the post-segment-insertion
/// CDT that contains this point (or is reachable from it without crossing
/// a constrained segment) is carved out of the final mesh.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PslgHole {
    pub point: Vertex,
}

/// Input PSLG: vertices, segments, holes.
///
/// Mirrors the data carried by a Triangle `.poly` file (regional attributes
/// and area constraints are deferred until we need them).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Pslg {
    pub vertices: Vec<PslgVertex>,
    pub segments: Vec<PslgSegment>,
    pub holes: Vec<PslgHole>,
}

impl Pslg {
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of per-vertex attributes. Inferred from the first vertex;
    /// the file readers enforce that every vertex carries the same count.
    pub fn vertex_attribute_count(&self) -> usize {
        self.vertices.first().map_or(0, |v| v.attributes.len())
    }

    /// True if any vertex carries a non-zero marker. Determines whether
    /// the .node/.poly writer emits the marker column.
    pub fn has_vertex_markers(&self) -> bool {
        self.vertices.iter().any(|v| v.marker != 0)
    }

    /// True if any segment carries a non-zero marker.
    pub fn has_segment_markers(&self) -> bool {
        self.segments.iter().any(|s| s.marker != 0)
    }

    /// Collapse exact-position duplicate vertices into single IDs and
    /// remap every segment endpoint accordingly. Useful preprocessing
    /// before [`crate::form_skeleton`], because the divide-and-conquer
    /// Delaunay drops exact-position duplicates from its triangulation
    /// — if any segment references one of the dropped IDs the segment
    /// insertion later panics looking for a vertex that's no longer in
    /// any live triangle.
    ///
    /// Two vertices are considered duplicates only when both coordinates
    /// match bit-for-bit. Attribute and marker mismatches are resolved
    /// to the first-encountered vertex (later duplicates inherit nothing
    /// from the survivor).
    ///
    /// Degenerate segments (endpoints collapsed into the same canonical
    /// vertex) are dropped silently. Holes are unchanged.
    pub fn deduplicate(&self) -> Self {
        use std::collections::HashMap;
        let mut canonical_id: HashMap<(u64, u64), u32> = HashMap::new();
        let mut new_vertices: Vec<PslgVertex> = Vec::new();
        let mut remap: Vec<u32> = Vec::with_capacity(self.vertices.len());
        for v in &self.vertices {
            let key = (v.position.x.to_bits(), v.position.y.to_bits());
            let id = *canonical_id.entry(key).or_insert_with(|| {
                let new_id = new_vertices.len() as u32;
                new_vertices.push(v.clone());
                new_id
            });
            remap.push(id);
        }
        let new_segments: Vec<PslgSegment> = self
            .segments
            .iter()
            .map(|s| PslgSegment {
                a: remap[s.a as usize],
                b: remap[s.b as usize],
                marker: s.marker,
            })
            .filter(|s| s.a != s.b)
            .collect();
        Self {
            vertices: new_vertices,
            segments: new_segments,
            holes: self.holes.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_attribute_count_is_consistent() {
        let mut p = Pslg::new();
        p.vertices.push(PslgVertex::new(Vertex::new(0.0, 0.0)).with_attribute(1.0));
        p.vertices.push(PslgVertex::new(Vertex::new(1.0, 0.0)).with_attribute(2.0));
        assert_eq!(p.vertex_attribute_count(), 1);
    }

    #[test]
    fn deduplicate_collapses_exact_position_duplicates() {
        let mut p = Pslg::new();
        // Three positions: (0,0), (1,0), (0,0) again.
        p.vertices.push(PslgVertex::new(Vertex::new(0.0, 0.0)));
        p.vertices.push(PslgVertex::new(Vertex::new(1.0, 0.0)));
        p.vertices.push(PslgVertex::new(Vertex::new(0.0, 0.0)));
        // Segment 0 → 2 references the duplicate-of-0; should collapse to 0→0 and drop.
        p.segments.push(PslgSegment::new(0, 1));
        p.segments.push(PslgSegment::new(0, 2)); // degenerate after dedup
        p.segments.push(PslgSegment::new(2, 1)); // should remap to 0→1

        let d = p.deduplicate();
        assert_eq!(d.vertices.len(), 2);
        assert_eq!(d.segments.len(), 2);
        // Both surviving segments should connect IDs 0 and 1 in some order.
        for s in &d.segments {
            assert!((s.a == 0 && s.b == 1) || (s.a == 1 && s.b == 0));
        }
    }

    #[test]
    fn deduplicate_preserves_unique_input_unchanged() {
        let mut p = Pslg::new();
        p.vertices.push(PslgVertex::new(Vertex::new(0.0, 0.0)));
        p.vertices.push(PslgVertex::new(Vertex::new(1.0, 0.0)));
        p.vertices.push(PslgVertex::new(Vertex::new(0.0, 1.0)));
        p.segments.push(PslgSegment::new(0, 1));
        p.segments.push(PslgSegment::new(1, 2));
        p.segments.push(PslgSegment::new(2, 0));
        let d = p.deduplicate();
        assert_eq!(d.vertices, p.vertices);
        assert_eq!(d.segments, p.segments);
    }

    #[test]
    fn marker_detection() {
        let mut p = Pslg::new();
        p.vertices.push(PslgVertex::new(Vertex::new(0.0, 0.0)));
        p.vertices.push(PslgVertex::new(Vertex::new(1.0, 0.0)).with_marker(5));
        assert!(p.has_vertex_markers());

        p.segments.push(PslgSegment::new(0, 1));
        assert!(!p.has_segment_markers());
        p.segments.last_mut().unwrap().marker = 7;
        assert!(p.has_segment_markers());
    }
}
