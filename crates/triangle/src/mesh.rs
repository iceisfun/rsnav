//! In-memory CDT mesh: triangle pool, subsegment pool, vertex pool, and the
//! [`Otri`]/[`Osub`] handles for navigating them.
//!
//! Translation of triangle.c's `struct triangle`, `struct subseg`, and the
//! `otri`/`osub` manipulation primitives. Where the C version packs metadata
//! into the low bits of pointers (orient, infect, dead), we keep separate
//! flag bytes on each slot — safe and easier to debug, marginally larger.

use rsnav_common::{Vertex, VertexId};

// --- Constant tables ----------------------------------------------------

/// `PLUS1_MOD3[i]` = `(i + 1) % 3`. Used everywhere the C code uses
/// `plus1mod3`.
pub const PLUS1_MOD3: [u8; 3] = [1, 2, 0];

/// `MINUS1_MOD3[i]` = `(i + 2) % 3`. Used everywhere the C code uses
/// `minus1mod3`.
pub const MINUS1_MOD3: [u8; 3] = [2, 0, 1];

// --- Sentinel indices ---------------------------------------------------

/// Reserved index for the "dummy" triangle. Every triangle pool stores its
/// dummy at slot 0; any neighbor reference that lacks a real triangle points
/// here so we never branch on a null check.
pub const DUMMY_TRI: u32 = 0;

/// Reserved index for the "dummy" subsegment. Same role as [`DUMMY_TRI`] but
/// for the subsegment pool.
pub const DUMMY_SUB: u32 = 0;

// --- Encoded handles -----------------------------------------------------

/// Packed (triangle index, orient) reference. High 30 bits index, low 2 bits
/// orient (0..2). Mirrors triangle.c's pointer-tag `encode()`/`decode()`.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct EncodedTri(pub u32);

impl EncodedTri {
    /// Dummy reference (triangle 0, orient 0). Used as the "no neighbor" value.
    pub const DUMMY: Self = Self(DUMMY_TRI << 2);

    #[inline]
    pub const fn pack(tri: u32, orient: u8) -> Self {
        debug_assert!(orient < 3);
        debug_assert!(tri < (1u32 << 30));
        Self((tri << 2) | orient as u32)
    }

    #[inline]
    pub const fn tri(self) -> u32 {
        self.0 >> 2
    }

    #[inline]
    pub const fn orient(self) -> u8 {
        (self.0 & 0b11) as u8
    }

    #[inline]
    pub fn to_otri(self) -> Otri {
        Otri {
            tri: self.tri(),
            orient: self.orient(),
        }
    }
}

/// Packed (subseg index, orient) reference. High 31 bits index, low 1 bit
/// orient (0..1).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct EncodedSub(pub u32);

impl EncodedSub {
    pub const DUMMY: Self = Self(DUMMY_SUB << 1);

    #[inline]
    pub const fn pack(sub: u32, orient: u8) -> Self {
        debug_assert!(orient < 2);
        debug_assert!(sub < (1u32 << 31));
        Self((sub << 1) | orient as u32)
    }

    #[inline]
    pub const fn sub(self) -> u32 {
        self.0 >> 1
    }

    #[inline]
    pub const fn orient(self) -> u8 {
        (self.0 & 0b1) as u8
    }

    #[inline]
    pub fn to_osub(self) -> Osub {
        Osub {
            sub: self.sub(),
            orient: self.orient(),
        }
    }
}

// --- Slot storage --------------------------------------------------------

/// One triangle in the pool.
///
/// Layout matches triangle.c with the orient/infect/dead bits hoisted into
/// the `flags` byte:
///
/// - `neighbors[orient]` is the triangle attached across the edge held by
///   the handle with this orient.
/// - `vertices[orient]` is the *apex* of the edge held by `orient`. The
///   *org* and *dest* of that edge are at `PLUS1_MOD3[orient]` and
///   `MINUS1_MOD3[orient]` respectively.
/// - `subsegs[orient]` is the subsegment glued to that edge (or
///   [`EncodedSub::DUMMY`] for free edges).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TriangleSlot {
    pub neighbors: [EncodedTri; 3],
    pub vertices: [VertexId; 3],
    pub subsegs: [EncodedSub; 3],
    pub flags: u8,
}

impl TriangleSlot {
    const FLAG_INFECTED: u8 = 1 << 0;
    const FLAG_DEAD: u8 = 1 << 1;

    /// Default state for a freshly allocated triangle: every neighbor and
    /// subseg points at its dummy, every vertex is `INVALID`, no flags set.
    pub const fn fresh() -> Self {
        Self {
            neighbors: [EncodedTri::DUMMY; 3],
            vertices: [VertexId::INVALID; 3],
            subsegs: [EncodedSub::DUMMY; 3],
            flags: 0,
        }
    }

    #[inline]
    pub const fn is_dead(self) -> bool {
        self.flags & Self::FLAG_DEAD != 0
    }

    #[inline]
    pub const fn is_infected(self) -> bool {
        self.flags & Self::FLAG_INFECTED != 0
    }
}

/// One subsegment in the pool.
///
/// - `next[orient]` is the next subseg in the segment chain on the given side.
/// - `sub_vertices[orient]` indexes the org of this subseg viewed from `orient`.
///   (Org of orient 0 = first endpoint; org of orient 1 = second endpoint.)
/// - `seg_vertices[orient]` indexes the org of the *full* inserted segment that
///   this subseg belongs to (useful when long segments are split).
/// - `triangles[orient]` is the abutting triangle on that side.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SubsegSlot {
    pub next: [EncodedSub; 2],
    pub sub_vertices: [VertexId; 2],
    pub seg_vertices: [VertexId; 2],
    pub triangles: [EncodedTri; 2],
    pub marker: i32,
    pub flags: u8,
}

impl SubsegSlot {
    const FLAG_DEAD: u8 = 1 << 0;

    pub const fn fresh() -> Self {
        Self {
            next: [EncodedSub::DUMMY; 2],
            sub_vertices: [VertexId::INVALID; 2],
            seg_vertices: [VertexId::INVALID; 2],
            triangles: [EncodedTri::DUMMY; 2],
            marker: 0,
            flags: 0,
        }
    }

    #[inline]
    pub const fn is_dead(self) -> bool {
        self.flags & Self::FLAG_DEAD != 0
    }
}

/// Classification of a vertex in the mesh.
///
/// Matches `enum verttype` in triangle.c (the subset we care about).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum VertexType {
    Input = 0,
    Segment = 1,
    Free = 2,
    Dead = 3,
    Undead = 4,
}

/// Mesh-side metadata for one vertex: the geometric position plus marker
/// plus a back-pointer to one incident triangle (set during segment insertion).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VertexSlot {
    pub position: Vertex,
    pub marker: i32,
    pub vtype: VertexType,
    /// One triangle incident to this vertex, used during segment insertion.
    /// `EncodedTri::DUMMY` if not yet set.
    pub triangle: EncodedTri,
}

impl VertexSlot {
    pub const fn new(position: Vertex, marker: i32) -> Self {
        Self {
            position,
            marker,
            vtype: VertexType::Input,
            triangle: EncodedTri::DUMMY,
        }
    }
}

// --- Otri / Osub: live handles into the pools ---------------------------

/// Live handle holding one edge of one triangle.
///
/// By convention the edge points counter-clockwise around the triangle.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Otri {
    pub tri: u32,
    pub orient: u8,
}

impl Otri {
    pub const DUMMY: Self = Self {
        tri: DUMMY_TRI,
        orient: 0,
    };

    #[inline]
    pub fn new(tri: u32, orient: u8) -> Self {
        debug_assert!(orient < 3);
        Self { tri, orient }
    }

    #[inline]
    pub fn encode(self) -> EncodedTri {
        EncodedTri::pack(self.tri, self.orient)
    }

    /// `lnext`: next edge CCW around the same triangle.
    #[inline]
    pub fn lnext(self) -> Self {
        Self {
            tri: self.tri,
            orient: PLUS1_MOD3[self.orient as usize],
        }
    }

    /// `lprev`: previous edge (CW) around the same triangle.
    #[inline]
    pub fn lprev(self) -> Self {
        Self {
            tri: self.tri,
            orient: MINUS1_MOD3[self.orient as usize],
        }
    }
}

/// Live handle holding one side of one subsegment.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Osub {
    pub sub: u32,
    pub orient: u8,
}

impl Osub {
    pub const DUMMY: Self = Self {
        sub: DUMMY_SUB,
        orient: 0,
    };

    #[inline]
    pub fn new(sub: u32, orient: u8) -> Self {
        debug_assert!(orient < 2);
        Self { sub, orient }
    }

    #[inline]
    pub fn encode(self) -> EncodedSub {
        EncodedSub::pack(self.sub, self.orient)
    }

    /// `ssym`: toggle which side of the subseg this handle holds.
    #[inline]
    pub fn ssym(self) -> Self {
        Self {
            sub: self.sub,
            orient: 1 - self.orient,
        }
    }
}

// --- The mesh ------------------------------------------------------------

/// CDT mesh with vertex / triangle / subsegment pools.
///
/// The pools are plain `Vec`s. Slot 0 in `triangles` and `subsegs` is the
/// reserved dummy (the sentinel that "no real triangle / subsegment is here").
/// User-visible triangle and subseg indices therefore start at 1.
#[derive(Clone, Debug)]
pub struct CdtMesh {
    pub vertices: Vec<VertexSlot>,
    pub triangles: Vec<TriangleSlot>,
    pub subsegs: Vec<SubsegSlot>,

    /// Number of triangles on the convex hull (set after triangulation).
    pub hull_size: u32,

    /// Free-list of recycled triangle indices (dead slots). Allocations
    /// preferentially pop from here before extending the `Vec`.
    free_tris: Vec<u32>,
    free_subs: Vec<u32>,
}

impl Default for CdtMesh {
    fn default() -> Self {
        Self::new()
    }
}

impl CdtMesh {
    /// Construct an empty mesh with the dummy triangle/subseg already in
    /// place at index 0.
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: vec![TriangleSlot::fresh()],
            subsegs: vec![SubsegSlot::fresh()],
            hull_size: 0,
            free_tris: Vec::new(),
            free_subs: Vec::new(),
        }
    }

    /// Reserve capacity in the vertex/triangle/subseg pools.
    pub fn reserve(&mut self, vertex_cap: usize, triangle_cap: usize, subseg_cap: usize) {
        self.vertices.reserve(vertex_cap);
        // +1 for the dummy already present.
        self.triangles.reserve(triangle_cap);
        self.subsegs.reserve(subseg_cap);
    }

    // -- vertex pool ----------------------------------------------------

    pub fn push_vertex(&mut self, slot: VertexSlot) -> VertexId {
        let id = VertexId::new(u32::try_from(self.vertices.len()).expect("vertex overflow"));
        self.vertices.push(slot);
        id
    }

    #[inline]
    pub fn vertex(&self, id: VertexId) -> &VertexSlot {
        &self.vertices[id.index()]
    }

    #[inline]
    pub fn vertex_mut(&mut self, id: VertexId) -> &mut VertexSlot {
        &mut self.vertices[id.index()]
    }

    #[inline]
    pub fn vertex_pos(&self, id: VertexId) -> Vertex {
        self.vertices[id.index()].position
    }

    // -- triangle pool --------------------------------------------------

    /// Allocate a fresh triangle. Returns a handle to edge 0 of the new
    /// triangle. Translation of `maketriangle()`.
    pub fn make_triangle(&mut self) -> Otri {
        let tri = if let Some(idx) = self.free_tris.pop() {
            // Reset the recycled slot.
            self.triangles[idx as usize] = TriangleSlot::fresh();
            idx
        } else {
            let idx = u32::try_from(self.triangles.len()).expect("triangle overflow");
            assert!(idx < (1u32 << 30), "triangle index exceeds 30-bit encoding");
            self.triangles.push(TriangleSlot::fresh());
            idx
        };
        Otri { tri, orient: 0 }
    }

    /// Mark a triangle as dead and put its slot on the free list.
    /// Translation of `triangledealloc()`. The caller is responsible for
    /// having torn down any bonds first.
    pub fn kill_triangle(&mut self, tri: u32) {
        debug_assert!(tri != DUMMY_TRI, "must not kill the dummy triangle");
        let slot = &mut self.triangles[tri as usize];
        slot.flags |= TriangleSlot::FLAG_DEAD;
        self.free_tris.push(tri);
    }

    /// Read a triangle slot by raw index. **Panics** if `tri` is out of
    /// range — most commonly because the index was issued by a
    /// different mesh. Triangle indices are not portable across
    /// `CdtMesh` instances.
    #[inline]
    pub fn triangle(&self, tri: u32) -> &TriangleSlot {
        &self.triangles[tri as usize]
    }

    /// Mutable variant of [`triangle`](Self::triangle). Same cross-mesh
    /// caveat.
    #[inline]
    pub fn triangle_mut(&mut self, tri: u32) -> &mut TriangleSlot {
        &mut self.triangles[tri as usize]
    }

    /// Live triangle count, excluding the dummy and any recycled slots.
    ///
    /// Uses saturating subtraction: by construction `triangles.len() >= 1`
    /// (the dummy occupies slot 0) and `free_tris.len() <= triangles.len() - 1`,
    /// so the natural difference is always non-negative — but a single
    /// off-by-one bug elsewhere would otherwise produce an unsigned-
    /// underflow panic that's hard to root-cause. Saturating turns it
    /// into a quietly wrong `0` instead.
    #[inline]
    pub fn live_triangle_count(&self) -> usize {
        self.triangles
            .len()
            .saturating_sub(1)
            .saturating_sub(self.free_tris.len())
    }

    // -- subseg pool ----------------------------------------------------

    pub fn make_subseg(&mut self) -> Osub {
        let sub = if let Some(idx) = self.free_subs.pop() {
            self.subsegs[idx as usize] = SubsegSlot::fresh();
            idx
        } else {
            let idx = u32::try_from(self.subsegs.len()).expect("subseg overflow");
            assert!(idx < (1u32 << 31), "subseg index exceeds 31-bit encoding");
            self.subsegs.push(SubsegSlot::fresh());
            idx
        };
        Osub { sub, orient: 0 }
    }

    pub fn kill_subseg(&mut self, sub: u32) {
        debug_assert!(sub != DUMMY_SUB, "must not kill the dummy subseg");
        let slot = &mut self.subsegs[sub as usize];
        slot.flags |= SubsegSlot::FLAG_DEAD;
        self.free_subs.push(sub);
    }

    #[inline]
    pub fn subseg(&self, sub: u32) -> &SubsegSlot {
        &self.subsegs[sub as usize]
    }

    #[inline]
    pub fn subseg_mut(&mut self, sub: u32) -> &mut SubsegSlot {
        &mut self.subsegs[sub as usize]
    }

    #[inline]
    pub fn live_subseg_count(&self) -> usize {
        // Same saturating-subtract rationale as `live_triangle_count`.
        self.subsegs
            .len()
            .saturating_sub(1)
            .saturating_sub(self.free_subs.len())
    }

    // --- Triangle handle accessors (org/dest/apex/sym/bond) -----------

    /// `org(o)`: origin vertex of the edge held by `o`.
    #[inline]
    pub fn org(&self, o: Otri) -> VertexId {
        self.triangle(o.tri).vertices[PLUS1_MOD3[o.orient as usize] as usize]
    }

    /// `dest(o)`: destination vertex of the edge held by `o`.
    #[inline]
    pub fn dest(&self, o: Otri) -> VertexId {
        self.triangle(o.tri).vertices[MINUS1_MOD3[o.orient as usize] as usize]
    }

    /// `apex(o)`: apex vertex (the corner opposite the edge held by `o`).
    #[inline]
    pub fn apex(&self, o: Otri) -> VertexId {
        self.triangle(o.tri).vertices[o.orient as usize]
    }

    pub fn set_org(&mut self, o: Otri, v: VertexId) {
        self.triangle_mut(o.tri).vertices[PLUS1_MOD3[o.orient as usize] as usize] = v;
    }
    pub fn set_dest(&mut self, o: Otri, v: VertexId) {
        self.triangle_mut(o.tri).vertices[MINUS1_MOD3[o.orient as usize] as usize] = v;
    }
    pub fn set_apex(&mut self, o: Otri, v: VertexId) {
        self.triangle_mut(o.tri).vertices[o.orient as usize] = v;
    }

    /// Set all three corners of the triangle held by `o` to `(org, dest, apex)`
    /// relative to the edge held by `o`. Convenience for the constructors.
    pub fn set_corners(&mut self, o: Otri, org: VertexId, dest: VertexId, apex: VertexId) {
        self.set_org(o, org);
        self.set_dest(o, dest);
        self.set_apex(o, apex);
    }

    /// `sym(o)`: the abutting triangle through edge `o`'s shared edge.
    /// Returns [`Otri::DUMMY`] if there is no neighbor.
    #[inline]
    pub fn sym(&self, o: Otri) -> Otri {
        self.triangle(o.tri).neighbors[o.orient as usize].to_otri()
    }

    /// `bond(a, b)`: glue the two edges together. Equivalent to triangle.c's
    /// `bond(otri1, otri2)`.
    pub fn bond(&mut self, a: Otri, b: Otri) {
        let enc_a = a.encode();
        let enc_b = b.encode();
        self.triangle_mut(a.tri).neighbors[a.orient as usize] = enc_b;
        self.triangle_mut(b.tri).neighbors[b.orient as usize] = enc_a;
    }

    /// `dissolve(o)`: unset the neighbor on edge `o`'s side. The other
    /// triangle still thinks it's bonded; callers must arrange for it to be
    /// rebound or killed.
    pub fn dissolve(&mut self, o: Otri) {
        self.triangle_mut(o.tri).neighbors[o.orient as usize] = EncodedTri::DUMMY;
    }

    /// `tspivot(o)`: the subseg glued to edge `o` (DUMMY if none).
    #[inline]
    pub fn tspivot(&self, o: Otri) -> Osub {
        self.triangle(o.tri).subsegs[o.orient as usize].to_osub()
    }

    /// `tsbond(o, s)`: glue triangle edge `o` to subseg side `s`.
    pub fn tsbond(&mut self, o: Otri, s: Osub) {
        self.triangle_mut(o.tri).subsegs[o.orient as usize] = s.encode();
        self.subseg_mut(s.sub).triangles[s.orient as usize] = o.encode();
    }

    pub fn ts_dissolve(&mut self, o: Otri) {
        self.triangle_mut(o.tri).subsegs[o.orient as usize] = EncodedSub::DUMMY;
    }

    pub fn st_dissolve(&mut self, s: Osub) {
        self.subseg_mut(s.sub).triangles[s.orient as usize] = EncodedTri::DUMMY;
    }

    /// `stpivot(s)`: the triangle abutting subseg side `s` (DUMMY if none).
    #[inline]
    pub fn stpivot(&self, s: Osub) -> Otri {
        self.subseg(s.sub).triangles[s.orient as usize].to_otri()
    }

    // --- Subseg handle accessors --------------------------------------

    #[inline]
    pub fn sorg(&self, s: Osub) -> VertexId {
        self.subseg(s.sub).sub_vertices[s.orient as usize]
    }

    #[inline]
    pub fn sdest(&self, s: Osub) -> VertexId {
        self.subseg(s.sub).sub_vertices[1 - s.orient as usize]
    }

    pub fn set_sorg(&mut self, s: Osub, v: VertexId) {
        self.subseg_mut(s.sub).sub_vertices[s.orient as usize] = v;
    }

    pub fn set_sdest(&mut self, s: Osub, v: VertexId) {
        self.subseg_mut(s.sub).sub_vertices[1 - s.orient as usize] = v;
    }

    #[inline]
    pub fn segorg(&self, s: Osub) -> VertexId {
        self.subseg(s.sub).seg_vertices[s.orient as usize]
    }

    #[inline]
    pub fn segdest(&self, s: Osub) -> VertexId {
        self.subseg(s.sub).seg_vertices[1 - s.orient as usize]
    }

    pub fn set_segorg(&mut self, s: Osub, v: VertexId) {
        self.subseg_mut(s.sub).seg_vertices[s.orient as usize] = v;
    }

    pub fn set_segdest(&mut self, s: Osub, v: VertexId) {
        self.subseg_mut(s.sub).seg_vertices[1 - s.orient as usize] = v;
    }

    /// `sbond(a, b)`: link two subsegments into a chain.
    pub fn sbond(&mut self, a: Osub, b: Osub) {
        let enc_a = a.encode();
        let enc_b = b.encode();
        self.subseg_mut(a.sub).next[a.orient as usize] = enc_b;
        self.subseg_mut(b.sub).next[b.orient as usize] = enc_a;
    }

    /// `spivot(s)`: the next subseg on the chain at this side.
    #[inline]
    pub fn spivot(&self, s: Osub) -> Osub {
        self.subseg(s.sub).next[s.orient as usize].to_osub()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    // -- Encoding round-trip ---

    #[test]
    fn encoded_tri_round_trip() {
        for tri in [1u32, 5, 1000, (1u32 << 30) - 1] {
            for orient in 0..3u8 {
                let enc = EncodedTri::pack(tri, orient);
                assert_eq!(enc.tri(), tri);
                assert_eq!(enc.orient(), orient);
                assert_eq!(enc.to_otri(), Otri { tri, orient });
            }
        }
    }

    #[test]
    fn encoded_sub_round_trip() {
        for sub in [1u32, 5, 1000, (1u32 << 31) - 1] {
            for orient in 0..2u8 {
                let enc = EncodedSub::pack(sub, orient);
                assert_eq!(enc.sub(), sub);
                assert_eq!(enc.orient(), orient);
                assert_eq!(enc.to_osub(), Osub { sub, orient });
            }
        }
    }

    #[test]
    fn dummy_constants() {
        assert_eq!(EncodedTri::DUMMY.tri(), DUMMY_TRI);
        assert_eq!(EncodedTri::DUMMY.orient(), 0);
        assert_eq!(EncodedSub::DUMMY.sub(), DUMMY_SUB);
        assert_eq!(EncodedSub::DUMMY.orient(), 0);
    }

    // -- Otri local navigation ---

    #[test]
    fn lnext_lprev_are_inverses() {
        let o = Otri::new(7, 0);
        assert_eq!(o.lnext().lprev(), o);
        assert_eq!(o.lprev().lnext(), o);
        assert_eq!(o.lnext().lnext().lnext(), o);
        assert_eq!(o.lprev().lprev().lprev(), o);
    }

    #[test]
    fn osub_ssym_is_involution() {
        let s = Osub::new(3, 0);
        assert_eq!(s.ssym(), Osub::new(3, 1));
        assert_eq!(s.ssym().ssym(), s);
    }

    // -- Triangle pool ---

    #[test]
    fn make_triangle_starts_after_dummy() {
        let mut m = CdtMesh::new();
        let a = m.make_triangle();
        let b = m.make_triangle();
        assert_eq!(a.tri, 1);
        assert_eq!(b.tri, 2);
        assert_eq!(m.live_triangle_count(), 2);
    }

    #[test]
    fn kill_recycles_triangle_slot() {
        let mut m = CdtMesh::new();
        let a = m.make_triangle();
        let _b = m.make_triangle();
        m.kill_triangle(a.tri);
        assert_eq!(m.live_triangle_count(), 1);
        let c = m.make_triangle();
        assert_eq!(c.tri, a.tri); // recycled
        assert!(!m.triangle(c.tri).is_dead());
    }

    // -- Half-edge bonds ---

    /// Build two CCW triangles sharing edge (B,C): t1 = (A,B,C), t2 = (D,C,B).
    /// Bond them through their shared edge, then walk sym/lnext to confirm.
    #[test]
    fn bond_sym_round_trip() {
        let mut m = CdtMesh::new();
        let a = m.push_vertex(VertexSlot::new(v(0.0, 0.0), 0));
        let b = m.push_vertex(VertexSlot::new(v(1.0, 0.0), 0));
        let c = m.push_vertex(VertexSlot::new(v(0.0, 1.0), 0));
        let d = m.push_vertex(VertexSlot::new(v(1.0, 1.0), 0));

        let t1 = m.make_triangle();
        m.set_corners(t1, a, b, c);
        let t2 = m.make_triangle();
        m.set_corners(t2, d, c, b);

        // With set_corners(t, A, B, C) the edges at orient 0/1/2 are
        // AB / BC / CA respectively. So edge BC lives at lnext(t1) (orient 1).
        // Likewise edge CB of t2 lives at lnext(t2) (orient 1).
        let edge_bc_in_t1 = t1.lnext();
        let edge_cb_in_t2 = t2.lnext();
        assert_eq!((m.org(edge_bc_in_t1), m.dest(edge_bc_in_t1)), (b, c));
        assert_eq!((m.org(edge_cb_in_t2), m.dest(edge_cb_in_t2)), (c, b));

        m.bond(edge_bc_in_t1, edge_cb_in_t2);

        assert_eq!(m.sym(edge_bc_in_t1), edge_cb_in_t2);
        assert_eq!(m.sym(edge_cb_in_t2), edge_bc_in_t1);

        // Walk: from edge_bc_in_t1, sym then lnext lands on t2's next edge
        // (org=B, dest=D, apex=C).
        let stepped = m.sym(edge_bc_in_t1).lnext();
        assert_eq!(m.org(stepped), b);
        assert_eq!(m.dest(stepped), d);
        assert_eq!(m.apex(stepped), c);
    }

    /// Spinning lnext around a triangle revisits the three edges in CCW order.
    #[test]
    fn lnext_visits_three_edges() {
        let mut m = CdtMesh::new();
        let a = m.push_vertex(VertexSlot::new(v(0.0, 0.0), 0));
        let b = m.push_vertex(VertexSlot::new(v(1.0, 0.0), 0));
        let c = m.push_vertex(VertexSlot::new(v(0.0, 1.0), 0));
        let t = m.make_triangle();
        m.set_corners(t, a, b, c);

        let e0 = t;
        let e1 = e0.lnext();
        let e2 = e1.lnext();
        assert_eq!(e2.lnext(), e0);
        assert_eq!(
            [m.org(e0), m.org(e1), m.org(e2)],
            [a, b, c],
            "org cycles through the corners in CCW order"
        );
        assert_eq!([m.dest(e0), m.dest(e1), m.dest(e2)], [b, c, a]);
        assert_eq!([m.apex(e0), m.apex(e1), m.apex(e2)], [c, a, b]);
    }

    // -- Subseg ↔ triangle bond ---

    #[test]
    fn tsbond_round_trip() {
        let mut m = CdtMesh::new();
        let a = m.push_vertex(VertexSlot::new(v(0.0, 0.0), 0));
        let b = m.push_vertex(VertexSlot::new(v(1.0, 0.0), 0));
        let c = m.push_vertex(VertexSlot::new(v(0.0, 1.0), 0));
        let t = m.make_triangle();
        m.set_corners(t, a, b, c);

        let s = m.make_subseg();
        m.set_sorg(s, a);
        m.set_sdest(s, b);

        m.tsbond(t, s);
        assert_eq!(m.tspivot(t), s);
        assert_eq!(m.stpivot(s), t);

        m.ts_dissolve(t);
        assert_eq!(m.tspivot(t), Osub::DUMMY);
        // stpivot still reports t — caller is expected to manage one side at a time.
        assert_eq!(m.stpivot(s), t);
    }

    #[test]
    fn unbonded_edge_reports_dummy_neighbor() {
        let mut m = CdtMesh::new();
        let t = m.make_triangle();
        assert_eq!(m.sym(t), Otri::DUMMY);
        assert_eq!(m.tspivot(t), Osub::DUMMY);
    }
}
