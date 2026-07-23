# rsnav-pathing

Path-following helpers for steering an agent along a polyline. This crate turns
a static path into per-frame motion; it has **zero dependency on the navmesh**
and only knows about points.

`PathFollower` owns a polyline and tracks the agent's arc-length progress. Each
step, you pass the agent's current position; the follower projects it forward
onto the path (never backtracking), advances by a lookahead distance, and
returns a steering target.

With `corner_avoidance > 0` the target is biased outward at real corners (turn
angle beyond `corner_angle_threshold`), so the agent takes a wider turn instead
of cutting across an inside-corner wall — a fix for the classic shortcutting
failure mode.

See [`docs/08-moving-agents.md`](https://github.com/iceisfun/rsnav/blob/master/docs/08-moving-agents.md)
for turning a `find_path` result into a moving character.

`#![forbid(unsafe_code)]`. Part of the [rsnav](https://github.com/iceisfun/rsnav)
workspace.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
