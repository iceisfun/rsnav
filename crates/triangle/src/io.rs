//! Readers and writers for Triangle's `.node`, `.poly`, and `.ele` ASCII files.
//!
//! Format reference: <https://www.cs.cmu.edu/~quake/triangle.html>
//!
//! The format treats `#` as a comment marker (to end of line); blank lines
//! are skipped. Vertex IDs in a file are *self-consistent* in their base
//! (0-based or 1-based) — we detect the base from the first vertex's ID and
//! translate to 0-based indices internally.

use std::fmt::Write as _;
use std::fs;
use std::io;
use std::path::Path;

use rsnav_common::Vertex;

use crate::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};

/// All errors a Triangle-format reader can produce.
#[derive(Debug)]
pub enum IoError {
    Io(io::Error),
    /// File ended before the parser expected.
    UnexpectedEof { context: &'static str },
    /// A numeric token failed to parse.
    Parse {
        line: usize,
        context: &'static str,
        token: String,
    },
    /// Wrong number of tokens on a line.
    BadColumnCount {
        line: usize,
        context: &'static str,
        expected: String,
        found: usize,
    },
    /// A referenced vertex index is out of range.
    VertexOutOfRange {
        line: usize,
        context: &'static str,
        index: i64,
        vertex_count: usize,
    },
    /// `.node` / `.poly` files must have dimension 2.
    UnsupportedDimension { line: usize, dim: usize },
    /// All vertices must carry the same number of attributes.
    InconsistentAttributes {
        line: usize,
        expected: usize,
        found: usize,
    },
}

impl std::fmt::Display for IoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IoError::Io(e) => write!(f, "io error: {e}"),
            IoError::UnexpectedEof { context } => {
                write!(f, "unexpected end of file while reading {context}")
            }
            IoError::Parse { line, context, token } => write!(
                f,
                "line {line}: failed to parse {context} token '{token}'"
            ),
            IoError::BadColumnCount { line, context, expected, found } => write!(
                f,
                "line {line}: {context} expects {expected} columns, found {found}"
            ),
            IoError::VertexOutOfRange { line, context, index, vertex_count } => write!(
                f,
                "line {line}: {context} references vertex {index} but only {vertex_count} vertices exist"
            ),
            IoError::UnsupportedDimension { line, dim } => write!(
                f,
                "line {line}: only 2D is supported (file declares dimension {dim})"
            ),
            IoError::InconsistentAttributes { line, expected, found } => write!(
                f,
                "line {line}: expected {expected} per-vertex attributes, found {found}"
            ),
        }
    }
}

impl std::error::Error for IoError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            IoError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for IoError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

pub type Result<T> = std::result::Result<T, IoError>;

// --- Tokenizer -----------------------------------------------------------

/// Stream of (line_no, [&str tokens]) over a Triangle ASCII file.
///
/// Skips blank lines and strips `#...` end-of-line comments. Yields the
/// line number (1-based) of each non-empty content line, alongside the
/// whitespace-split tokens of that line.
struct TokenLines<'a> {
    inner: std::iter::Enumerate<std::str::Lines<'a>>,
}

impl<'a> TokenLines<'a> {
    fn new(src: &'a str) -> Self {
        Self {
            inner: src.lines().enumerate(),
        }
    }
}

impl<'a> Iterator for TokenLines<'a> {
    type Item = (usize, Vec<&'a str>);

    fn next(&mut self) -> Option<Self::Item> {
        for (i, raw) in self.inner.by_ref() {
            let line_no = i + 1;
            let trimmed = raw.split('#').next().unwrap().trim();
            if trimmed.is_empty() {
                continue;
            }
            let tokens: Vec<&str> = trimmed.split_whitespace().collect();
            return Some((line_no, tokens));
        }
        None
    }
}

fn next_line<'a>(it: &mut TokenLines<'a>, context: &'static str) -> Result<(usize, Vec<&'a str>)> {
    it.next().ok_or(IoError::UnexpectedEof { context })
}

fn parse_int(tok: &str, line: usize, context: &'static str) -> Result<i64> {
    tok.parse::<i64>().map_err(|_| IoError::Parse {
        line,
        context,
        token: tok.to_string(),
    })
}

fn parse_usize(tok: &str, line: usize, context: &'static str) -> Result<usize> {
    let n = parse_int(tok, line, context)?;
    if n < 0 {
        return Err(IoError::Parse {
            line,
            context,
            token: tok.to_string(),
        });
    }
    Ok(n as usize)
}

fn parse_real(tok: &str, line: usize, context: &'static str) -> Result<f64> {
    tok.parse::<f64>().map_err(|_| IoError::Parse {
        line,
        context,
        token: tok.to_string(),
    })
}

// --- .node and .poly vertex section --------------------------------------

/// Header of a .node/.poly vertex section.
struct VertexHeader {
    count: usize,
    attribute_count: usize,
    marker_count: usize,
}

fn read_vertex_header(line: usize, tokens: &[&str]) -> Result<VertexHeader> {
    if tokens.len() != 4 {
        return Err(IoError::BadColumnCount {
            line,
            context: "vertex header",
            expected: "4".into(),
            found: tokens.len(),
        });
    }
    let count = parse_usize(tokens[0], line, "vertex count")?;
    let dim = parse_usize(tokens[1], line, "dimension")?;
    if dim != 2 {
        return Err(IoError::UnsupportedDimension { line, dim });
    }
    let attribute_count = parse_usize(tokens[2], line, "attribute count")?;
    let marker_count = parse_usize(tokens[3], line, "marker count")?;
    if marker_count > 1 {
        return Err(IoError::Parse {
            line,
            context: "marker count (must be 0 or 1)",
            token: tokens[3].to_string(),
        });
    }
    Ok(VertexHeader {
        count,
        attribute_count,
        marker_count,
    })
}

/// Read the body of a vertex section.
///
/// Returns the vertices in file order plus the base ID used in the file
/// (0 or 1) so subsequent sections can normalize their indices.
fn read_vertex_body(
    lines: &mut TokenLines<'_>,
    header: &VertexHeader,
) -> Result<(Vec<PslgVertex>, i64)> {
    let expected_cols = 3 + header.attribute_count + header.marker_count;
    let mut vertices = Vec::with_capacity(header.count);
    let mut base = 0i64;

    for i in 0..header.count {
        let (line, toks) = next_line(lines, "vertex row")?;
        if toks.len() != expected_cols {
            return Err(IoError::BadColumnCount {
                line,
                context: "vertex row",
                expected: format!("{expected_cols}"),
                found: toks.len(),
            });
        }
        let id = parse_int(toks[0], line, "vertex id")?;
        if i == 0 {
            base = id;
        }
        let x = parse_real(toks[1], line, "vertex x")?;
        let y = parse_real(toks[2], line, "vertex y")?;
        let mut attrs = Vec::with_capacity(header.attribute_count);
        for k in 0..header.attribute_count {
            attrs.push(parse_real(toks[3 + k], line, "vertex attribute")?);
        }
        let marker = if header.marker_count == 1 {
            parse_int(toks[3 + header.attribute_count], line, "vertex marker")? as i32
        } else {
            0
        };
        vertices.push(PslgVertex {
            position: Vertex::new(x, y),
            attributes: attrs,
            marker,
        });
    }

    Ok((vertices, base))
}

// --- .poly reader --------------------------------------------------------

/// Read a `.poly` file into a [`Pslg`].
///
/// If the file's vertex header declares 0 vertices, the function falls back
/// to reading a companion `<stem>.node` file from the same directory (the
/// behavior of Triangle's `-p` switch with a separate node file).
pub fn read_poly(path: &Path) -> Result<Pslg> {
    let src = fs::read_to_string(path)?;
    let mut lines = TokenLines::new(&src);

    let (line, toks) = next_line(&mut lines, "vertex header")?;
    let header = read_vertex_header(line, &toks)?;

    let (vertices, base) = if header.count == 0 {
        // Companion .node file.
        let node_path = path.with_extension("node");
        let nsrc = fs::read_to_string(&node_path)?;
        let mut nlines = TokenLines::new(&nsrc);
        let (nl, ntoks) = next_line(&mut nlines, "companion .node header")?;
        let nheader = read_vertex_header(nl, &ntoks)?;
        read_vertex_body(&mut nlines, &nheader)?
    } else {
        read_vertex_body(&mut lines, &header)?
    };

    // Segment header.
    let (line, toks) = next_line(&mut lines, "segment header")?;
    if toks.len() != 2 {
        return Err(IoError::BadColumnCount {
            line,
            context: "segment header",
            expected: "2".into(),
            found: toks.len(),
        });
    }
    let seg_count = parse_usize(toks[0], line, "segment count")?;
    let seg_marker_count = parse_usize(toks[1], line, "segment marker count")?;
    if seg_marker_count > 1 {
        return Err(IoError::Parse {
            line,
            context: "segment marker count (must be 0 or 1)",
            token: toks[1].to_string(),
        });
    }
    let seg_cols = 3 + seg_marker_count;
    let mut segments = Vec::with_capacity(seg_count);
    for _ in 0..seg_count {
        let (line, toks) = next_line(&mut lines, "segment row")?;
        if toks.len() != seg_cols {
            return Err(IoError::BadColumnCount {
                line,
                context: "segment row",
                expected: format!("{seg_cols}"),
                found: toks.len(),
            });
        }
        let _id = parse_int(toks[0], line, "segment id")?;
        let a = parse_int(toks[1], line, "segment endpoint A")?;
        let b = parse_int(toks[2], line, "segment endpoint B")?;
        let a = normalize_vertex_index(a, base, line, "segment endpoint A", vertices.len())?;
        let b = normalize_vertex_index(b, base, line, "segment endpoint B", vertices.len())?;
        let marker = if seg_marker_count == 1 {
            parse_int(toks[3], line, "segment marker")? as i32
        } else {
            0
        };
        segments.push(PslgSegment { a, b, marker });
    }

    // Hole header.
    let mut holes = Vec::new();
    if let Some((line, toks)) = lines.next() {
        if toks.len() != 1 {
            return Err(IoError::BadColumnCount {
                line,
                context: "hole header",
                expected: "1".into(),
                found: toks.len(),
            });
        }
        let hole_count = parse_usize(toks[0], line, "hole count")?;
        for _ in 0..hole_count {
            let (line, toks) = next_line(&mut lines, "hole row")?;
            if toks.len() != 3 {
                return Err(IoError::BadColumnCount {
                    line,
                    context: "hole row",
                    expected: "3".into(),
                    found: toks.len(),
                });
            }
            let _id = parse_int(toks[0], line, "hole id")?;
            let x = parse_real(toks[1], line, "hole x")?;
            let y = parse_real(toks[2], line, "hole y")?;
            holes.push(PslgHole {
                point: Vertex::new(x, y),
            });
        }
    }

    // Optional region section — silently ignore for now (CDT-only path).

    Ok(Pslg {
        vertices,
        segments,
        holes,
    })
}

fn normalize_vertex_index(
    raw: i64,
    base: i64,
    line: usize,
    context: &'static str,
    vertex_count: usize,
) -> Result<u32> {
    let zero_based = raw - base;
    if zero_based < 0 || zero_based as usize >= vertex_count {
        return Err(IoError::VertexOutOfRange {
            line,
            context,
            index: raw,
            vertex_count,
        });
    }
    Ok(zero_based as u32)
}

// --- .node reader (standalone) ------------------------------------------

/// Read a `.node` file (vertices only, no segments / holes).
pub fn read_node(path: &Path) -> Result<Pslg> {
    let src = fs::read_to_string(path)?;
    let mut lines = TokenLines::new(&src);
    let (line, toks) = next_line(&mut lines, "vertex header")?;
    let header = read_vertex_header(line, &toks)?;
    let (vertices, _base) = read_vertex_body(&mut lines, &header)?;
    Ok(Pslg {
        vertices,
        segments: Vec::new(),
        holes: Vec::new(),
    })
}

// --- Writers ------------------------------------------------------------

/// Settings controlling writer output.
#[derive(Copy, Clone, Debug)]
pub struct WriteOptions {
    /// Base for emitted IDs. Triangle uses `1` by default; pass `0` to mimic
    /// the `-z` switch.
    pub index_base: u32,
    /// Whether to emit a per-vertex boundary marker column.
    pub emit_vertex_markers: bool,
    /// Whether to emit a per-segment boundary marker column.
    pub emit_segment_markers: bool,
}

impl Default for WriteOptions {
    fn default() -> Self {
        Self {
            index_base: 1,
            emit_vertex_markers: true,
            emit_segment_markers: true,
        }
    }
}

/// Format a `.node` file for `pslg`.
pub fn format_node(pslg: &Pslg, opts: WriteOptions) -> String {
    let attribute_count = pslg.vertex_attribute_count();
    let marker_col = if opts.emit_vertex_markers { 1 } else { 0 };
    let mut s = String::new();
    writeln!(
        s,
        "{}  {}  {}  {}",
        pslg.vertices.len(),
        2,
        attribute_count,
        marker_col,
    )
    .unwrap();
    for (i, v) in pslg.vertices.iter().enumerate() {
        let id = i as u32 + opts.index_base;
        write!(s, "{:4}    {}  {}", id, v.position.x, v.position.y).unwrap();
        for a in &v.attributes {
            write!(s, "  {}", a).unwrap();
        }
        if opts.emit_vertex_markers {
            write!(s, "    {}", v.marker).unwrap();
        }
        s.push('\n');
    }
    s
}

/// Triangle elements (`.ele`) writer.
///
/// Each row is `<element_id> <v1> <v2> <v3>`. We do not currently emit
/// per-element attributes (Triangle's `-A` switch); the third header column
/// is always 0.
pub fn format_ele(triangles: &[[u32; 3]], opts: WriteOptions) -> String {
    let mut s = String::new();
    writeln!(s, "{}  3  0", triangles.len()).unwrap();
    for (i, t) in triangles.iter().enumerate() {
        writeln!(
            s,
            "{:4}    {}    {}    {}",
            i as u32 + opts.index_base,
            t[0] + opts.index_base,
            t[1] + opts.index_base,
            t[2] + opts.index_base
        )
        .unwrap();
    }
    s
}

/// Format a `.poly` file from `pslg`. Vertex section is included inline.
pub fn format_poly(pslg: &Pslg, opts: WriteOptions) -> String {
    let mut s = format_node(pslg, opts);
    // Segment section.
    let marker_col = if opts.emit_segment_markers { 1 } else { 0 };
    writeln!(s, "{} {}", pslg.segments.len(), marker_col).unwrap();
    for (i, seg) in pslg.segments.iter().enumerate() {
        write!(
            s,
            "{:4} {} {}",
            i as u32 + opts.index_base,
            seg.a + opts.index_base,
            seg.b + opts.index_base
        )
        .unwrap();
        if opts.emit_segment_markers {
            write!(s, " {}", seg.marker).unwrap();
        }
        s.push('\n');
    }
    // Hole section.
    writeln!(s, "{}", pslg.holes.len()).unwrap();
    for (i, h) in pslg.holes.iter().enumerate() {
        writeln!(
            s,
            "{} {} {}",
            i as u32 + opts.index_base,
            h.point.x,
            h.point.y
        )
        .unwrap();
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn poly_string() -> &'static str {
        "\
# A unit square with one square hole.
4 2 0 0
1 0.0 0.0
2 4.0 0.0
3 4.0 4.0
4 0.0 4.0
4 0
1 1 2
2 2 3
3 3 4
4 4 1
1
1 2.0 2.0
"
    }

    #[test]
    fn read_inline_poly() {
        let dir = std::env::temp_dir();
        let path = dir.join("rsnav-poly-test.poly");
        std::fs::write(&path, poly_string()).unwrap();
        let p = read_poly(&path).unwrap();
        assert_eq!(p.vertices.len(), 4);
        assert_eq!(p.vertices[0].position, Vertex::new(0.0, 0.0));
        assert_eq!(p.vertices[3].position, Vertex::new(0.0, 4.0));
        assert_eq!(p.segments.len(), 4);
        // Segment IDs in the file are 1-based; we should have 0-based internally.
        assert_eq!(p.segments[0], PslgSegment::new(0, 1));
        assert_eq!(p.segments[3], PslgSegment::new(3, 0));
        assert_eq!(p.holes.len(), 1);
        assert_eq!(p.holes[0].point, Vertex::new(2.0, 2.0));
    }

    #[test]
    fn zero_based_indices_also_work() {
        let txt = "\
4 2 0 0
0 0 0
1 1 0
2 1 1
3 0 1
4 0
0 0 1
1 1 2
2 2 3
3 3 0
0
";
        let dir = std::env::temp_dir();
        let path = dir.join("rsnav-zb-poly-test.poly");
        std::fs::write(&path, txt).unwrap();
        let p = read_poly(&path).unwrap();
        assert_eq!(p.vertices.len(), 4);
        assert_eq!(p.segments[0], PslgSegment::new(0, 1));
        assert_eq!(p.segments[3], PslgSegment::new(3, 0));
    }

    #[test]
    fn round_trip_a_poly_from_triangle_distribution() {
        // The reference A.poly from Shewchuk's distribution.
        let p = read_poly(Path::new("../../../triangle/A.poly")).unwrap();
        assert_eq!(p.vertices.len(), 29);
        assert_eq!(p.segments.len(), 29);
        assert_eq!(p.holes.len(), 1);
        // The hole point sits inside the inner ring.
        assert_eq!(p.holes[0].point, Vertex::new(0.47, -0.5));
        // The first vertex appears verbatim.
        assert_eq!(p.vertices[0].position, Vertex::new(0.2, -0.7764));
        // It carries one attribute (the `1` column in the header).
        assert_eq!(p.vertices[0].attributes, vec![-0.57]);
    }

    #[test]
    fn format_node_round_trips() {
        let pslg = Pslg {
            vertices: vec![
                PslgVertex::new(Vertex::new(0.0, 0.0)),
                PslgVertex::new(Vertex::new(1.0, 1.0)).with_marker(2),
            ],
            segments: Vec::new(),
            holes: Vec::new(),
        };
        let s = format_node(&pslg, WriteOptions::default());
        // Re-read it.
        let dir = std::env::temp_dir();
        let path = dir.join("rsnav-node-rt.node");
        std::fs::write(&path, &s).unwrap();
        let back = read_node(&path).unwrap();
        assert_eq!(back.vertices, pslg.vertices);
    }

    #[test]
    fn format_poly_round_trips() {
        let pslg = Pslg {
            vertices: vec![
                PslgVertex::new(Vertex::new(0.0, 0.0)),
                PslgVertex::new(Vertex::new(1.0, 0.0)),
                PslgVertex::new(Vertex::new(1.0, 1.0)),
                PslgVertex::new(Vertex::new(0.0, 1.0)),
            ],
            segments: vec![
                PslgSegment::new(0, 1),
                PslgSegment::new(1, 2),
                PslgSegment::new(2, 3),
                PslgSegment::new(3, 0),
            ],
            holes: vec![PslgHole {
                point: Vertex::new(0.5, 0.5),
            }],
        };
        let s = format_poly(&pslg, WriteOptions::default());
        let dir = std::env::temp_dir();
        let path = dir.join("rsnav-poly-rt.poly");
        std::fs::write(&path, &s).unwrap();
        let back = read_poly(&path).unwrap();
        assert_eq!(back, pslg);
    }

    #[test]
    fn format_ele_uses_index_base() {
        let s = format_ele(
            &[[0, 1, 2], [0, 2, 3]],
            WriteOptions {
                index_base: 1,
                ..Default::default()
            },
        );
        let mut lines = s.lines();
        assert_eq!(lines.next().unwrap().trim(), "2  3  0");
        assert!(lines.next().unwrap().contains("1    2    3"));
        assert!(lines.next().unwrap().contains("1    3    4"));
    }
}
