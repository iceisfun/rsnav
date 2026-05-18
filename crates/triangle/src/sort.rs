//! Vertex sorting utilities for divide-and-conquer Delaunay.
//!
//! Counterparts to triangle.c's `vertexsort`, `vertexmedian`, and
//! `alternateaxes`. The C versions hand-rolled randomized quicksort/quickselect
//! over pointer arrays; in Rust we permute a slice of [`VertexId`]s using
//! `slice::sort_by` and `slice::select_nth_unstable_by`, which match the
//! same asymptotics.

use std::cmp::Ordering;

use rsnav_common::{Vertex, VertexId};

use crate::mesh::CdtMesh;

/// Lexicographic comparison: primary key the given axis, secondary key the
/// other axis. `axis = 0` means x is primary, y is secondary; `axis = 1`
/// reverses them. Used to match triangle.c's `vertexmedian` ordering.
#[inline]
fn lex_cmp(a: Vertex, b: Vertex, axis: u8) -> Ordering {
    let (pa, sa) = if axis == 0 { (a.x, a.y) } else { (a.y, a.x) };
    let (pb, sb) = if axis == 0 { (b.x, b.y) } else { (b.y, b.x) };
    pa.partial_cmp(&pb)
        .unwrap_or(Ordering::Equal)
        .then_with(|| sa.partial_cmp(&sb).unwrap_or(Ordering::Equal))
}

/// Sort `ids` so that the corresponding vertex positions are in ascending
/// lexicographic order — x as primary key, y as secondary.
///
/// Counterpart to triangle.c `vertexsort()`.
pub fn vertex_sort(mesh: &CdtMesh, ids: &mut [VertexId]) {
    ids.sort_by(|a, b| lex_cmp(mesh.vertex_pos(*a), mesh.vertex_pos(*b), 0));
}

/// Partial sort: after the call, `ids[..median]` lex-precede `ids[median..]`
/// using the given axis ordering, but neither side is necessarily fully
/// sorted. Linear expected time.
///
/// Counterpart to triangle.c `vertexmedian()`.
pub fn vertex_median(mesh: &CdtMesh, ids: &mut [VertexId], median: usize, axis: u8) {
    if ids.is_empty() || median >= ids.len() {
        return;
    }
    ids.select_nth_unstable_by(median, |a, b| {
        lex_cmp(mesh.vertex_pos(*a), mesh.vertex_pos(*b), axis)
    });
}

/// Recursive alternating-axis partition, as used by the divide-and-conquer
/// Delaunay driver. At each level the slice is partitioned at its median on
/// the current axis; each half is then recursively partitioned on the other
/// axis. Subsets of `<=3` vertices are always sorted on the x-axis (the
/// `divconqrecurse` base cases require x-sorted input).
///
/// Counterpart to triangle.c `alternateaxes()`.
pub fn alternate_axes(mesh: &CdtMesh, ids: &mut [VertexId], axis: u8) {
    let n = ids.len();
    if n <= 1 {
        return;
    }
    let effective_axis = if n <= 3 { 0 } else { axis };
    let divider = n >> 1;
    vertex_median(mesh, ids, divider, effective_axis);
    // For tiny inputs ≤3 we also fully sort: the base case in divconqrecurse
    // assumes the leaf array is sorted on x, but select_nth only guarantees
    // the median position. Pull a final tiny sort to match the C invariant.
    if n <= 3 {
        ids.sort_by(|a, b| lex_cmp(mesh.vertex_pos(*a), mesh.vertex_pos(*b), 0));
        return;
    }
    let (left, right_with_pivot) = ids.split_at_mut(divider);
    // `right_with_pivot[0]` is the median element. The C version recurses on
    // the *whole* right slice (including the pivot at its boundary).
    if !left.is_empty() {
        alternate_axes(mesh, left, 1 - axis);
    }
    if !right_with_pivot.is_empty() {
        alternate_axes(mesh, right_with_pivot, 1 - axis);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::VertexSlot;

    fn push(mesh: &mut CdtMesh, x: f64, y: f64) -> VertexId {
        mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0))
    }

    #[test]
    fn vertex_sort_lex() {
        let mut m = CdtMesh::new();
        let a = push(&mut m, 3.0, 1.0);
        let b = push(&mut m, 1.0, 2.0);
        let c = push(&mut m, 1.0, 1.0);
        let d = push(&mut m, 2.0, 5.0);
        let mut ids = vec![a, b, c, d];
        vertex_sort(&m, &mut ids);
        // Expected lex order: (1,1)=c, (1,2)=b, (2,5)=d, (3,1)=a
        assert_eq!(ids, vec![c, b, d, a]);
    }

    #[test]
    fn vertex_median_places_middle() {
        let mut m = CdtMesh::new();
        let v: Vec<VertexId> = (0..7)
            .map(|i| push(&mut m, i as f64, 0.0))
            .collect();
        let mut shuffled = v.clone();
        shuffled.swap(0, 6);
        shuffled.swap(1, 5);
        vertex_median(&m, &mut shuffled, 3, 0);
        // Element at index 3 should be the median by x — vertex with x = 3.
        let mid_pos = m.vertex_pos(shuffled[3]);
        assert_eq!(mid_pos.x, 3.0);
        // Everything to the left has x < 3.
        for id in &shuffled[..3] {
            assert!(m.vertex_pos(*id).x < 3.0);
        }
        // Everything to the right has x > 3.
        for id in &shuffled[4..] {
            assert!(m.vertex_pos(*id).x > 3.0);
        }
    }

    #[test]
    fn alternate_axes_terminates_and_partitions() {
        let mut m = CdtMesh::new();
        // 8 vertices on a 4x2 grid.
        let mut ids = Vec::new();
        for x in 0..4 {
            for y in 0..2 {
                ids.push(push(&mut m, x as f64, y as f64));
            }
        }
        let original: Vec<VertexId> = ids.clone();
        alternate_axes(&m, &mut ids, 0);

        // Same multiset of IDs (just permuted).
        let mut a: Vec<u32> = ids.iter().map(|i| i.get()).collect();
        let mut b: Vec<u32> = original.iter().map(|i| i.get()).collect();
        a.sort();
        b.sort();
        assert_eq!(a, b);

        // First half should be entirely on the left of the median x.
        let n = ids.len();
        let mid = n / 2;
        let median_x = m.vertex_pos(ids[mid]).x;
        for id in &ids[..mid] {
            assert!(m.vertex_pos(*id).x <= median_x);
        }
    }

    #[test]
    fn alternate_axes_tiny_inputs_sort_by_x() {
        let mut m = CdtMesh::new();
        let a = push(&mut m, 2.0, 0.0);
        let b = push(&mut m, 1.0, 0.0);
        let mut ids = vec![a, b];
        alternate_axes(&m, &mut ids, 1); // ask for y-axis; should still come out x-sorted
        assert_eq!(ids, vec![b, a]);
    }
}
