# rsnav-crowd

Multi-agent crowd simulation over a shared navmesh — the Detour-Crowd analogue
for the [rsnav](https://github.com/iceisfun/rsnav) stack.

Each `Agent` gets its own funnel-pulled path corridor through a shared
`NavMesh`, and a sampled velocity-obstacle solver picks a per-tick velocity that
follows the corridor while side-stepping other agents. The corridor lets an
agent keep moving along its route without a full replan every frame, and the
local-avoidance step keeps agents from walking through one another.

See [`docs/11-crowds.md`](https://github.com/iceisfun/rsnav/blob/master/docs/11-crowds.md)
for a worked example and the tuning knobs.

`#![forbid(unsafe_code)]`. Part of the rsnav workspace.

## License

Dual-licensed under either the [MIT license](https://github.com/iceisfun/rsnav/blob/master/LICENSE-MIT)
or the [Apache License, Version 2.0](https://github.com/iceisfun/rsnav/blob/master/LICENSE-APACHE),
at your option.
