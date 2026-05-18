//! Binary serialization for [`NavMesh`].
//!
//! Format spec: see `FORMAT.md` in this crate.

use std::io::{self, Read, Write};

use rsnav_common::{Aabb, TriangleId, Vertex, VertexId};

use crate::navmesh::{NavMesh, NavTriangle};

// --- Constants -----------------------------------------------------------

/// Magic bytes at the start of every navmesh file.
pub const MAGIC: &[u8; 8] = b"RSNAVMSH";

/// On-disk format version. Bumped on every breaking change.
pub const FORMAT_VERSION: u32 = 1;

// Section type IDs.
const SECTION_META: u32 = 1;
const SECTION_VERTICES: u32 = 2;
const SECTION_TRIANGLES: u32 = 3;
const SECTION_ADJACENCY: u32 = 4;
const SECTION_EDGE_MARKERS: u32 = 5;
const SECTION_TRI_INFO: u32 = 6;

// Sizes used in offset math.
const FILE_HEADER_BYTES: u64 = 16; // magic(8) + version(4) + section_count(4)
const SECTION_ENTRY_BYTES: u64 = 24; // type(4) + reserved(4) + offset(8) + length(8)

// --- Errors --------------------------------------------------------------

#[derive(Debug)]
pub enum SaveError {
    Io(io::Error),
}

impl std::fmt::Display for SaveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SaveError::Io(e) => write!(f, "io error while saving navmesh: {e}"),
        }
    }
}
impl std::error::Error for SaveError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SaveError::Io(e) => Some(e),
        }
    }
}
impl From<io::Error> for SaveError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

#[derive(Debug)]
pub enum LoadError {
    Io(io::Error),
    BadMagic,
    UnsupportedVersion(u32),
    MissingRequiredSection(&'static str),
    SectionLengthMismatch {
        section: &'static str,
        expected: u64,
        found: u64,
    },
    Truncated,
    BadIndex {
        kind: &'static str,
        value: u32,
        max: u32,
    },
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Io(e) => write!(f, "io error while loading navmesh: {e}"),
            LoadError::BadMagic => write!(f, "not a navmesh file (bad magic bytes)"),
            LoadError::UnsupportedVersion(v) => {
                write!(f, "unsupported navmesh format version: {v}")
            }
            LoadError::MissingRequiredSection(s) => {
                write!(f, "navmesh file is missing required section: {s}")
            }
            LoadError::SectionLengthMismatch { section, expected, found } => write!(
                f,
                "section {section} length mismatch: expected {expected} bytes, found {found}"
            ),
            LoadError::Truncated => write!(f, "navmesh file is truncated"),
            LoadError::BadIndex { kind, value, max } => {
                write!(f, "{kind} index {value} exceeds maximum {max}")
            }
        }
    }
}
impl std::error::Error for LoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            LoadError::Io(e) => Some(e),
            _ => None,
        }
    }
}
impl From<io::Error> for LoadError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

// --- Save ----------------------------------------------------------------

impl NavMesh {
    /// Serialize to bytes in the v1 navmesh format.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        self.write_to(&mut buf).expect("Vec<u8> never errors");
        buf
    }

    /// Serialize to an `io::Write`. Returns the number of bytes written.
    pub fn write_to<W: Write>(&self, w: &mut W) -> Result<usize, SaveError> {
        // Pre-compute each section's body so we know their sizes for the
        // section table.
        let meta = self.section_meta_bytes();
        let vertices = self.section_vertices_bytes();
        let triangles = self.section_triangles_bytes();
        let adjacency = self.section_adjacency_bytes();
        let edge_markers = self.section_edge_markers_bytes();
        let tri_info = self.section_tri_info_bytes();

        let sections: [(u32, &[u8]); 6] = [
            (SECTION_META, &meta),
            (SECTION_VERTICES, &vertices),
            (SECTION_TRIANGLES, &triangles),
            (SECTION_ADJACENCY, &adjacency),
            (SECTION_EDGE_MARKERS, &edge_markers),
            (SECTION_TRI_INFO, &tri_info),
        ];

        // Layout: header (16) + table (24 * N) + section bodies.
        let table_bytes = SECTION_ENTRY_BYTES * sections.len() as u64;
        let mut next_offset = FILE_HEADER_BYTES + table_bytes;
        let mut entries: Vec<(u32, u64, u64)> = Vec::with_capacity(sections.len());
        for (ty, body) in &sections {
            let len = body.len() as u64;
            entries.push((*ty, next_offset, len));
            next_offset += len;
        }

        let mut written = 0usize;

        // File header.
        w.write_all(MAGIC)?;
        written += MAGIC.len();
        w.write_all(&FORMAT_VERSION.to_le_bytes())?;
        written += 4;
        let section_count = sections.len() as u32;
        w.write_all(&section_count.to_le_bytes())?;
        written += 4;

        // Section table.
        for (ty, offset, length) in &entries {
            w.write_all(&ty.to_le_bytes())?;
            w.write_all(&0u32.to_le_bytes())?; // reserved
            w.write_all(&offset.to_le_bytes())?;
            w.write_all(&length.to_le_bytes())?;
            written += SECTION_ENTRY_BYTES as usize;
        }

        // Section bodies.
        for (_, body) in &sections {
            w.write_all(body)?;
            written += body.len();
        }

        Ok(written)
    }

    fn section_meta_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(8 * 8);
        b.extend_from_slice(&(self.vertices.len() as u32).to_le_bytes());
        b.extend_from_slice(&(self.triangles.len() as u32).to_le_bytes());
        b.extend_from_slice(&self.region_count.to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes()); // padding to align f64
        b.extend_from_slice(&self.aabb.min.x.to_le_bytes());
        b.extend_from_slice(&self.aabb.min.y.to_le_bytes());
        b.extend_from_slice(&self.aabb.max.x.to_le_bytes());
        b.extend_from_slice(&self.aabb.max.y.to_le_bytes());
        b
    }

    fn section_vertices_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(self.vertices.len() * 16);
        for v in &self.vertices {
            b.extend_from_slice(&v.x.to_le_bytes());
            b.extend_from_slice(&v.y.to_le_bytes());
        }
        b
    }

    fn section_triangles_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(self.triangles.len() * 12);
        for t in &self.triangles {
            for v in &t.vertices {
                b.extend_from_slice(&v.get().to_le_bytes());
            }
        }
        b
    }

    fn section_adjacency_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(self.triangles.len() * 12);
        for t in &self.triangles {
            for n in &t.neighbors {
                let raw = if n.is_valid() { n.get() } else { u32::MAX };
                b.extend_from_slice(&raw.to_le_bytes());
            }
        }
        b
    }

    fn section_edge_markers_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(self.triangles.len() * 12);
        for t in &self.triangles {
            for m in &t.edge_markers {
                b.extend_from_slice(&m.to_le_bytes());
            }
        }
        b
    }

    fn section_tri_info_bytes(&self) -> Vec<u8> {
        // 8 (area) + 16 (centroid) + 4 (region) = 28 bytes per triangle.
        let mut b = Vec::with_capacity(self.triangles.len() * 28);
        for t in &self.triangles {
            b.extend_from_slice(&t.area.to_le_bytes());
            b.extend_from_slice(&t.centroid.x.to_le_bytes());
            b.extend_from_slice(&t.centroid.y.to_le_bytes());
            b.extend_from_slice(&t.region.to_le_bytes());
        }
        b
    }
}

// --- Load ----------------------------------------------------------------

impl NavMesh {
    /// Load from a byte slice. Unknown sections are silently ignored.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, LoadError> {
        Self::read_from(&mut io::Cursor::new(bytes))
    }

    /// Load from an `io::Read`. Reads the file header + section table, then
    /// pulls each section by its absolute offset.
    pub fn read_from<R: Read>(r: &mut R) -> Result<Self, LoadError> {
        let mut all = Vec::new();
        r.read_to_end(&mut all)?;
        let bytes = &all[..];
        if bytes.len() < FILE_HEADER_BYTES as usize {
            return Err(LoadError::Truncated);
        }
        if &bytes[0..8] != MAGIC {
            return Err(LoadError::BadMagic);
        }
        let version = read_u32(bytes, 8)?;
        if version != FORMAT_VERSION {
            return Err(LoadError::UnsupportedVersion(version));
        }
        let section_count = read_u32(bytes, 12)? as usize;
        let table_bytes = SECTION_ENTRY_BYTES as usize * section_count;
        if bytes.len() < FILE_HEADER_BYTES as usize + table_bytes {
            return Err(LoadError::Truncated);
        }

        // Section table entries: (type, offset, length).
        let mut meta_range: Option<(usize, usize)> = None;
        let mut vertices_range: Option<(usize, usize)> = None;
        let mut triangles_range: Option<(usize, usize)> = None;
        let mut adjacency_range: Option<(usize, usize)> = None;
        let mut edge_markers_range: Option<(usize, usize)> = None;
        let mut tri_info_range: Option<(usize, usize)> = None;

        for i in 0..section_count {
            let entry_off = FILE_HEADER_BYTES as usize + i * SECTION_ENTRY_BYTES as usize;
            let ty = read_u32(bytes, entry_off)?;
            // entry_off + 4 is `reserved` — skipped.
            let off = read_u64(bytes, entry_off + 8)? as usize;
            let len = read_u64(bytes, entry_off + 16)? as usize;
            if off + len > bytes.len() {
                return Err(LoadError::Truncated);
            }
            let target = match ty {
                SECTION_META => &mut meta_range,
                SECTION_VERTICES => &mut vertices_range,
                SECTION_TRIANGLES => &mut triangles_range,
                SECTION_ADJACENCY => &mut adjacency_range,
                SECTION_EDGE_MARKERS => &mut edge_markers_range,
                SECTION_TRI_INFO => &mut tri_info_range,
                _ => continue, // unknown section type — skip per spec
            };
            *target = Some((off, len));
        }

        // META: required. Fixed 48 bytes per the spec
        // (12 counts + 4 padding + 32 aabb).
        let (meta_off, meta_len) =
            meta_range.ok_or(LoadError::MissingRequiredSection("META"))?;
        if meta_len < 48 {
            return Err(LoadError::SectionLengthMismatch {
                section: "META",
                expected: 48,
                found: meta_len as u64,
            });
        }
        let vcount = read_u32(bytes, meta_off)? as usize;
        let tcount = read_u32(bytes, meta_off + 4)? as usize;
        let rcount = read_u32(bytes, meta_off + 8)?;
        // skip reserved padding at +12
        let aabb = Aabb {
            min: Vertex::new(
                read_f64(bytes, meta_off + 16)?,
                read_f64(bytes, meta_off + 24)?,
            ),
            max: Vertex::new(
                read_f64(bytes, meta_off + 32)?,
                read_f64(bytes, meta_off + 40)?,
            ),
        };

        // VERTICES: required.
        let (v_off, v_len) =
            vertices_range.ok_or(LoadError::MissingRequiredSection("VERTICES"))?;
        let expected_v = vcount * 16;
        if v_len != expected_v {
            return Err(LoadError::SectionLengthMismatch {
                section: "VERTICES",
                expected: expected_v as u64,
                found: v_len as u64,
            });
        }
        let mut vertices = Vec::with_capacity(vcount);
        for i in 0..vcount {
            let base = v_off + i * 16;
            vertices.push(Vertex::new(
                read_f64(bytes, base)?,
                read_f64(bytes, base + 8)?,
            ));
        }

        // TRIANGLES: required.
        let (t_off, t_len) =
            triangles_range.ok_or(LoadError::MissingRequiredSection("TRIANGLES"))?;
        let expected_t = tcount * 12;
        if t_len != expected_t {
            return Err(LoadError::SectionLengthMismatch {
                section: "TRIANGLES",
                expected: expected_t as u64,
                found: t_len as u64,
            });
        }
        let mut triangles: Vec<NavTriangle> = Vec::with_capacity(tcount);
        for i in 0..tcount {
            let base = t_off + i * 12;
            let v = [
                bounded_vertex_id(read_u32(bytes, base)?, vcount as u32)?,
                bounded_vertex_id(read_u32(bytes, base + 4)?, vcount as u32)?,
                bounded_vertex_id(read_u32(bytes, base + 8)?, vcount as u32)?,
            ];
            triangles.push(NavTriangle {
                vertices: v,
                neighbors: [TriangleId::INVALID; 3],
                edge_markers: [0; 3],
                area: 0.0,
                centroid: Vertex::ZERO,
                region: 0,
            });
        }

        // ADJACENCY: optional; recompute from triangles if absent.
        if let Some((off, len)) = adjacency_range {
            let expected = tcount * 12;
            if len != expected {
                return Err(LoadError::SectionLengthMismatch {
                    section: "ADJACENCY",
                    expected: expected as u64,
                    found: len as u64,
                });
            }
            for i in 0..tcount {
                let base = off + i * 12;
                for k in 0..3 {
                    let raw = read_u32(bytes, base + k * 4)?;
                    triangles[i].neighbors[k] = if raw == u32::MAX {
                        TriangleId::INVALID
                    } else if (raw as usize) >= tcount {
                        return Err(LoadError::BadIndex {
                            kind: "triangle neighbor",
                            value: raw,
                            max: tcount as u32,
                        });
                    } else {
                        TriangleId::new(raw)
                    };
                }
            }
        } else {
            recompute_adjacency(&mut triangles);
        }

        // EDGE_MARKERS: optional; default to all-zero if absent.
        if let Some((off, len)) = edge_markers_range {
            let expected = tcount * 12;
            if len != expected {
                return Err(LoadError::SectionLengthMismatch {
                    section: "EDGE_MARKERS",
                    expected: expected as u64,
                    found: len as u64,
                });
            }
            for i in 0..tcount {
                let base = off + i * 12;
                for k in 0..3 {
                    triangles[i].edge_markers[k] = read_i32(bytes, base + k * 4)?;
                }
            }
        }

        // TRI_INFO: optional; recompute area/centroid/region if absent.
        let mut have_tri_info = false;
        if let Some((off, len)) = tri_info_range {
            let expected = tcount * 28;
            if len != expected {
                return Err(LoadError::SectionLengthMismatch {
                    section: "TRI_INFO",
                    expected: expected as u64,
                    found: len as u64,
                });
            }
            for i in 0..tcount {
                let base = off + i * 28;
                triangles[i].area = read_f64(bytes, base)?;
                triangles[i].centroid = Vertex::new(
                    read_f64(bytes, base + 8)?,
                    read_f64(bytes, base + 16)?,
                );
                triangles[i].region = read_u32(bytes, base + 24)?;
            }
            have_tri_info = true;
        }
        if !have_tri_info {
            recompute_tri_info(&mut triangles, &vertices);
        }

        let region_count = if have_tri_info {
            rcount
        } else {
            triangles.iter().map(|t| t.region).max().map_or(0, |m| m + 1)
        };
        Ok(NavMesh {
            vertices,
            triangles,
            aabb,
            region_count,
        })
    }
}

// --- Helpers -------------------------------------------------------------

fn read_u32(b: &[u8], at: usize) -> Result<u32, LoadError> {
    let slice = b.get(at..at + 4).ok_or(LoadError::Truncated)?;
    Ok(u32::from_le_bytes(slice.try_into().unwrap()))
}
fn read_i32(b: &[u8], at: usize) -> Result<i32, LoadError> {
    let slice = b.get(at..at + 4).ok_or(LoadError::Truncated)?;
    Ok(i32::from_le_bytes(slice.try_into().unwrap()))
}
fn read_u64(b: &[u8], at: usize) -> Result<u64, LoadError> {
    let slice = b.get(at..at + 8).ok_or(LoadError::Truncated)?;
    Ok(u64::from_le_bytes(slice.try_into().unwrap()))
}
fn read_f64(b: &[u8], at: usize) -> Result<f64, LoadError> {
    let slice = b.get(at..at + 8).ok_or(LoadError::Truncated)?;
    Ok(f64::from_le_bytes(slice.try_into().unwrap()))
}

fn bounded_vertex_id(raw: u32, vcount: u32) -> Result<VertexId, LoadError> {
    if raw >= vcount {
        Err(LoadError::BadIndex {
            kind: "triangle vertex",
            value: raw,
            max: vcount,
        })
    } else {
        Ok(VertexId::new(raw))
    }
}

/// Rebuild per-triangle neighbor pointers by hashing each undirected edge.
fn recompute_adjacency(triangles: &mut [NavTriangle]) {
    use std::collections::HashMap;
    type Edge = (u32, u32);
    // Pass 1: read-only — record where each edge first appears.
    let mut edge_to_tri: HashMap<Edge, (u32, u8)> = HashMap::new();
    // Pass 1 also records the bonds we need to write, so pass 2 can be a
    // pure write loop and doesn't fight the borrow checker.
    let mut bonds: Vec<(u32, u8, u32, u8)> = Vec::new();
    for (i, t) in triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = t.edge_vertices(k);
            let key = canonical_edge(a.get(), b.get());
            if let Some(&(other_i, other_k)) = edge_to_tri.get(&key) {
                bonds.push((i as u32, k as u8, other_i, other_k));
            } else {
                edge_to_tri.insert(key, (i as u32, k as u8));
            }
        }
    }
    for (i, k, other_i, other_k) in bonds {
        triangles[i as usize].neighbors[k as usize] = TriangleId::new(other_i);
        triangles[other_i as usize].neighbors[other_k as usize] = TriangleId::new(i);
    }
}

fn canonical_edge(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn recompute_tri_info(triangles: &mut [NavTriangle], vertices: &[Vertex]) {
    use std::collections::VecDeque;
    // area + centroid
    for t in triangles.iter_mut() {
        let p0 = vertices[t.vertices[0].index()];
        let p1 = vertices[t.vertices[1].index()];
        let p2 = vertices[t.vertices[2].index()];
        let area2 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
        t.area = 0.5 * area2.abs();
        t.centroid = Vertex::new(
            (p0.x + p1.x + p2.x) / 3.0,
            (p0.y + p1.y + p2.y) / 3.0,
        );
    }
    // region_id via BFS, identical to build::build_from_cdt's logic.
    let n = triangles.len();
    let mut region_id = vec![u32::MAX; n];
    let mut next_region: u32 = 0;
    let mut queue: VecDeque<u32> = VecDeque::new();
    for seed in 0..n as u32 {
        if region_id[seed as usize] != u32::MAX {
            continue;
        }
        let me = next_region;
        next_region += 1;
        region_id[seed as usize] = me;
        queue.clear();
        queue.push_back(seed);
        while let Some(t) = queue.pop_front() {
            let tri = &triangles[t as usize];
            for edge in 0..3 {
                if tri.edge_markers[edge] != 0 {
                    continue;
                }
                let n_tri = tri.neighbors[edge];
                if !n_tri.is_valid() {
                    continue;
                }
                let n_idx = n_tri.index();
                if region_id[n_idx] == u32::MAX {
                    region_id[n_idx] = me;
                    queue.push_back(n_tri.get());
                }
            }
        }
    }
    for (t, &r) in triangles.iter_mut().zip(region_id.iter()) {
        t.region = r;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build::build_from_cdt;
    use rsnav_common::Vertex;
    use rsnav_triangle::{
        carve_holes, delaunay, form_skeleton, pslg::Pslg, pslg::PslgHole, pslg::PslgSegment,
        pslg::PslgVertex, CdtMesh, DivConqOptions, VertexSlot,
    };

    fn build_test_navmesh() -> NavMesh {
        let pts = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 4.0),
            (0.0, 4.0),
            (1.5, 1.5),
            (2.5, 1.5),
            (2.5, 2.5),
            (1.5, 2.5),
        ];
        let mut mesh = CdtMesh::new();
        for (x, y) in pts {
            mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let pslg = Pslg {
            vertices: pts
                .iter()
                .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
                .collect(),
            segments: vec![
                PslgSegment { a: 0, b: 1, marker: 10 },
                PslgSegment { a: 1, b: 2, marker: 10 },
                PslgSegment { a: 2, b: 3, marker: 10 },
                PslgSegment { a: 3, b: 0, marker: 10 },
                PslgSegment { a: 4, b: 5, marker: 20 },
                PslgSegment { a: 5, b: 6, marker: 20 },
                PslgSegment { a: 6, b: 7, marker: 20 },
                PslgSegment { a: 7, b: 4, marker: 20 },
            ],
            holes: vec![PslgHole {
                point: Vertex::new(2.0, 2.0),
            }],
        };
        form_skeleton(&mut mesh, &pslg, None);
        carve_holes(&mut mesh, &pslg, false);
        build_from_cdt(&mesh)
    }

    #[test]
    fn round_trip_via_bytes() {
        let nav = build_test_navmesh();
        let bytes = nav.to_bytes();
        let reloaded = NavMesh::from_bytes(&bytes).unwrap();

        assert_eq!(nav.vertices.len(), reloaded.vertices.len());
        assert_eq!(nav.triangles.len(), reloaded.triangles.len());
        assert_eq!(nav.region_count, reloaded.region_count);
        for (a, b) in nav.vertices.iter().zip(reloaded.vertices.iter()) {
            assert_eq!(a, b);
        }
        for (a, b) in nav.triangles.iter().zip(reloaded.triangles.iter()) {
            assert_eq!(a, b);
        }
        assert_eq!(nav.aabb, reloaded.aabb);
    }

    #[test]
    fn header_magic_and_version() {
        let nav = build_test_navmesh();
        let bytes = nav.to_bytes();
        assert_eq!(&bytes[0..8], MAGIC);
        assert_eq!(
            u32::from_le_bytes(bytes[8..12].try_into().unwrap()),
            FORMAT_VERSION
        );
    }

    #[test]
    fn bad_magic_rejected() {
        let mut bytes = build_test_navmesh().to_bytes();
        bytes[0] = b'X';
        match NavMesh::from_bytes(&bytes) {
            Err(LoadError::BadMagic) => {}
            other => panic!("expected BadMagic, got {other:?}"),
        }
    }

    #[test]
    fn unknown_section_types_are_ignored() {
        // Manually craft a file with one valid META/VERTICES/TRIANGLES plus
        // an extra unknown section. Loading should still succeed.
        let nav = build_test_navmesh();
        let bytes = nav.to_bytes();
        // Use the existing valid bytes; replace section 5 (TRI_INFO) type
        // with an unknown ID and confirm load still works (TRI_INFO will
        // be recomputed instead).
        let mut tampered = bytes.clone();
        let entry_off =
            FILE_HEADER_BYTES as usize + 5 * SECTION_ENTRY_BYTES as usize;
        tampered[entry_off..entry_off + 4].copy_from_slice(&9999u32.to_le_bytes());
        let reloaded = NavMesh::from_bytes(&tampered).expect("should still load");
        assert_eq!(reloaded.triangle_count(), nav.triangle_count());
        // Without TRI_INFO, region IDs are recomputed and should match.
        for (a, b) in nav.triangles.iter().zip(reloaded.triangles.iter()) {
            assert_eq!(a.region, b.region);
        }
    }

    #[test]
    fn load_works_without_optional_sections() {
        // Hand-craft a minimal file: META + VERTICES + TRIANGLES + EDGE_MARKERS
        // only. The loader should recompute adjacency, area/centroid, and
        // region IDs.
        let nav = build_test_navmesh();
        let mut buf = Vec::new();
        // We'll write 4 sections instead of 6.
        let meta = nav.section_meta_bytes();
        let verts = nav.section_vertices_bytes();
        let tris = nav.section_triangles_bytes();
        let markers = nav.section_edge_markers_bytes();
        let section_specs: [(u32, &[u8]); 4] = [
            (SECTION_META, &meta),
            (SECTION_VERTICES, &verts),
            (SECTION_TRIANGLES, &tris),
            (SECTION_EDGE_MARKERS, &markers),
        ];
        let table = SECTION_ENTRY_BYTES * section_specs.len() as u64;
        let mut next_off = FILE_HEADER_BYTES + table;
        let mut entries = Vec::new();
        for (ty, body) in &section_specs {
            entries.push((*ty, next_off, body.len() as u64));
            next_off += body.len() as u64;
        }
        buf.extend_from_slice(MAGIC);
        buf.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
        buf.extend_from_slice(&(section_specs.len() as u32).to_le_bytes());
        for (ty, off, len) in &entries {
            buf.extend_from_slice(&ty.to_le_bytes());
            buf.extend_from_slice(&0u32.to_le_bytes());
            buf.extend_from_slice(&off.to_le_bytes());
            buf.extend_from_slice(&len.to_le_bytes());
        }
        for (_, body) in &section_specs {
            buf.extend_from_slice(body);
        }
        let reloaded = NavMesh::from_bytes(&buf).unwrap();
        assert_eq!(reloaded.triangle_count(), nav.triangle_count());
        assert_eq!(reloaded.region_count, nav.region_count);
        // Adjacency should be valid (symmetric).
        for (i, t) in reloaded.triangles.iter().enumerate() {
            for edge in 0..3 {
                if let Some(n_tri) = if t.neighbors[edge].is_valid() {
                    Some(t.neighbors[edge])
                } else {
                    None
                } {
                    let back = reloaded
                        .triangle(n_tri)
                        .neighbors
                        .iter()
                        .any(|n| n.is_valid() && n.index() == i);
                    assert!(back);
                }
            }
        }
    }
}
