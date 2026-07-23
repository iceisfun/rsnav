# Sending a navmesh over the wire

For anyone moving a baked navmesh between processes or machines: a server
handing a mesh to clients, a tool feeding a running game, an editor pushing an
update. rsnav gives you exactly two things at this boundary — `to_bytes` to
encode and `from_bytes` to decode — and is deliberately incurious about
everything else. The transport, the framing, the message types, the reliability,
the compression, the encryption: that is your game's protocol, and most games
already have one. This page prepares the encode/decode stage and demonstrates it
over a plain TCP socket. It does not propose a protocol for you to adopt.

If you have not yet read [saving and loading](14-saving-and-loading.md), read it
first — it owns the four calls, the error types, the byte layout, and the rule
about what the receiver has to rebuild. This page is only the delta for a socket
instead of a file.

## The encode/decode boundary is two calls

```rust
let bytes: Vec<u8> = nav.to_bytes();          // encode: hand these to your transport
let nav = NavMesh::from_bytes(&bytes)?;       // decode: from the bytes you received
```

The bytes are byte-for-byte the same blob [14](14-saving-and-loading.md) writes
to a file — there is no separate "wire format." A payload you produce on a Rust
server is decoded by a Rust client with `from_bytes`, or by a client in any other
language that implements the reader from
[`crates/navmesh/FORMAT.md`](../crates/navmesh/FORMAT.md). The format is
little-endian everywhere, fixed-width, and skips unknown sections, precisely so a
non-Rust client is a couple of hours of work, not a research project.

## The one wire-specific rule: frame it

The single thing a socket changes versus a file is that a stream has no end until
you give it one. `NavMesh::read_from` reads to EOF
([14 §trap](14-saving-and-loading.md#trap-read_from-consumes-the-entire-reader)),
so calling it on a socket only works if the sender transmits exactly one mesh and
then closes the connection. Anything longer-lived — one connection carrying many
meshes, or a mesh followed by your own game traffic — must say how long the blob
is. Length-prefix it:

```rust
// send
stream.write_all(&(payload.len() as u32).to_le_bytes())?;
stream.write_all(&payload)?;

// receive
let mut len = [0u8; 4];
stream.read_exact(&mut len)?;
let mut payload = vec![0u8; u32::from_le_bytes(len) as usize];
stream.read_exact(&mut payload)?;
let nav = NavMesh::from_bytes(&payload)?;      // exact bytes, no read-to-EOF
```

That four-byte prefix is the whole "protocol" this page endorses, and it is only
here so the example is honest. Your real framing — a header with a message type,
a sequence number, a checksum, whatever — replaces it. rsnav neither provides nor
wants an opinion on it.

## The demonstration

[`crates/navigation/examples/tcp_interop.rs`](../crates/navigation/examples/tcp_interop.rs)
builds a navmesh, ships it across a loopback TCP connection with the length prefix
above, decodes it on the far side, rebuilds the BVH, and paths across the received
mesh. It runs as one program (the "server" is a thread) so there is nothing to
coordinate:

```
cargo run -p rsnav-navigation --example tcp_interop

server: navmesh 8 triangles -> 848 bytes on the wire
client: received 848 bytes
client: decoded 8 triangles, rebuilt the BVH locally
client: path of 4 points across the received mesh
```

The client half is the part that generalizes to any transport: read the bytes,
`from_bytes`, `Bsp::build`, query.

## The receiver rebuilds the same things a file load does

Decoding gives you a `NavMesh` and nothing else. The `Bsp`, `WallInfo`,
`WallClearance` and any `DoorSet` are derived and are **not** on the wire, exactly
as they are not in a file. The receiver rebuilds them in the same order and with
the same translate-first hazard described in
[14 §load order](14-saving-and-loading.md#load-order) — the wire changes nothing
about that sequence. `TriangleId` and `VertexId` values are per-instance, so any
game data you send alongside the mesh keyed by triangle ID is only valid against
the exact bytes it was computed from.

## Determinism buys you a cheap "do you already have this?" handshake

Navmesh output is byte-identical for identical input and `BuildOptions`
([15](15-performance-and-determinism.md)), so a hash of `to_bytes()` is a stable
content id. A server pushing updates can send that hash first and let a client
that already holds the matching mesh skip the payload entirely — the same property
that makes the hash a sound bake cache key in
[14](14-saving-and-loading.md#determinism-makes-a-content-hash-a-valid-cache-key)
makes it a sound "version" token on the wire. Building that handshake is your job;
the guarantee that makes it correct is rsnav's.

## What this page will never grow into

To be explicit about the boundary, so you stop looking here for it: rsnav does not
and will not ship a message envelope, a message-type registry, delta/patch
encoding of mesh changes, compression, encryption, or session/reliability
handling. Those are transport and game concerns with no navmesh-specific right
answer. You get deterministic bytes out of one call and a validated `NavMesh` back
from another; the wire in between is yours.
