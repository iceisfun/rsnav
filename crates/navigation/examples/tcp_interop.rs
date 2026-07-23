//! Send a navmesh over a plain TCP socket and path on it at the other end.
//!
//! rsnav gives you two calls at the wire boundary: `NavMesh::to_bytes` to
//! encode and `NavMesh::from_bytes` to decode. Everything else here — the
//! socket, the length prefix, the fact that it is TCP at all — is just a
//! demonstration. Your game does its own thing and rsnav does not care.
//!
//! Run: `cargo run -p rsnav-navigation --example tcp_interop`

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::thread;

use rsnav_bsp::Bsp;
use rsnav_common::Vertex;
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navigation::{find_path, PathOptions};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

// Top row first, as a human reads it. A solid block splits the room so a
// straight line from start to goal is impossible — the path has to bend.
const MAP: &[&str] = &[
    "####################",
    "#..................#",
    "#..................#",
    "#........####......#",
    "#........####......#",
    "#........####......#",
    "#..................#",
    "#..................#",
    "####################",
];

fn ascii_to_bitfield(rows: &[&str]) -> Bitfield {
    let h = rows.len() as u32;
    let w = rows[0].len() as u32;
    let mut cells = vec![false; (w * h) as usize];
    for (d, row) in rows.iter().enumerate() {
        // Row 0 of a Bitfield is the bottom row; MAP[0] is the top.
        let bf_row = h as usize - 1 - d;
        for (col, ch) in row.bytes().enumerate() {
            cells[bf_row * w as usize + col] = ch == b'.';
        }
    }
    Bitfield::new(w, h, cells).expect("valid bitfield")
}

// A navmesh blob is self-contained, so one length prefix is all a stream
// needs to know where it ends. (`read_from` would instead read to EOF —
// see docs/14 — which only works if you close after a single mesh.)
fn send_frame(stream: &mut TcpStream, payload: &[u8]) -> std::io::Result<()> {
    stream.write_all(&(payload.len() as u32).to_le_bytes())?;
    stream.write_all(payload)?;
    stream.flush()
}

fn recv_frame(stream: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let mut len = [0u8; 4];
    stream.read_exact(&mut len)?;
    let mut payload = vec![0u8; u32::from_le_bytes(len) as usize];
    stream.read_exact(&mut payload)?;
    Ok(payload)
}

fn main() {
    // ---- server side: build a navmesh, encode it ----
    let bf = ascii_to_bitfield(MAP);
    let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
        .expect("map has walkable cells");
    let sent_tris = build.navmesh.triangle_count();
    let payload = build.navmesh.to_bytes();
    println!(
        "server: navmesh {sent_tris} triangles -> {} bytes on the wire",
        payload.len()
    );

    // Loopback in-process so the example is one runnable program; a real
    // server owns the listener. The framing is the point, not the topology.
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind loopback");
    let addr = listener.local_addr().expect("local addr");
    let server = thread::spawn(move || {
        let (mut conn, _peer) = listener.accept().expect("accept");
        send_frame(&mut conn, &payload).expect("send frame");
    });

    // ---- client side: receive bytes, decode, rebuild, query ----
    let mut conn = TcpStream::connect(addr).expect("connect");
    let bytes = recv_frame(&mut conn).expect("recv frame");
    println!("client: received {} bytes", bytes.len());

    let nav = NavMesh::from_bytes(&bytes).expect("decode navmesh");
    // The BVH is not serialized (docs/14) — the receiver rebuilds it, and
    // any other derived structure, from the decoded mesh.
    let bsp = Bsp::build(&nav);
    println!(
        "client: decoded {} triangles, rebuilt the BVH locally",
        nav.triangle_count()
    );

    let path = find_path(
        &nav,
        &bsp,
        Vertex::new(2.5, 4.5),
        Vertex::new(17.5, 4.5),
        &PathOptions::default(),
    )
    .expect("path across the received mesh");
    println!(
        "client: path of {} points across the received mesh",
        path.points.len()
    );

    server.join().expect("server thread");
    assert_eq!(nav.triangle_count(), sent_tris, "round trip preserved the mesh");
}
