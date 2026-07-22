# Running many agents

Prerequisites: a built navmesh ([05](05-building-navmeshes.md)) and a single agent that
already moves correctly ([08](08-moving-agents.md)). This page covers `rsnav-crowd`: what
it simulates, what it refuses to simulate, and the handful of behaviours that will look
like bugs until you know they are design.

## What rsnav-crowd is

A per-agent primitive, and nothing above it. Each [`Agent`](../crates/crowd/src/lib.rs)
owns a funnel-pulled corridor through a shared navmesh, and every tick a sampled
velocity-obstacle solver picks one velocity per agent that follows that corridor while
side-stepping neighbouring discs. Radius is per-agent, so a fat unit and a thin one plan
different corridors through the same mesh.

What it is not, stated so you stop looking: no state machine, no formations, no goal-slot
reservation or claim system, no flow fields, no group or squad concept, no job system, no
threading. `Crowd` does not consult doors — a [`DoorSet`](09-doors-and-navworld.md) is
invisible to it, because `set_nav` builds a door-free
`WallInfo::from_navmesh` for its revalidation sweep
([lib.rs:361](../crates/crowd/src/lib.rs)) and replans go through the free `find_path`.
It also does not use [`PathFollower`](08-moving-agents.md): the crowd steers straight at
the next corridor corner with its own cursor and has no lookahead or corner-bias model.

Group-level behaviour lives in the demo crates. Run `rsnav-crowd-demo` to see one built on
top of this primitive; this page does not explain its internals.

## Setup

```rust
let nav: Arc<NavBuild> = /* build_navmesh_from_bitfield, or NavWorker::poll_swap */;
let mut crowd = Crowd::new(nav, CrowdConfig::default());
let id = crowd.add_agent(Agent::new(Vertex::new(3.0, 5.05), 0.4, 2.5));
crowd.set_goal(id, Some(Goal { target: Vertex::new(21.0, 5.0), arrive_radius: 0.5 }));
loop { crowd.tick(1.0 / 60.0); }
```

The full runnable version, including the tick loop and the arrival report, is
[`crates/crowd/examples/two_agents_pass.rs`](../crates/crowd/examples/two_agents_pass.rs):

```
cargo run --release -p rsnav-crowd --example two_agents_pass
```

`Crowd::new` takes `Arc<NavBuild>` — the whole `rsnav-dynamic` build product, not a bare
`NavMesh` plus `Bsp`. There is no other constructor. A hand-authored or directly
CDT-built mesh ([13](13-authored-geometry.md)) cannot drive a `Crowd` without being wrapped
in a `NavBuild`. That wrap is cheap and does not require the grid pipeline: `NavBuild`'s
fields are all `pub` (`navmesh`, `bsp`, `build_ms`, `generation`), so a struct literal plus
`Bsp::build(&navmesh)` is enough.

`CrowdConfig` is fixed at construction. `config()` is the only accessor and there is no
setter; `neighbor_radius` in particular is additionally frozen into the spatial hash's cell
size (`neighbor_radius.max(1.0)`, [lib.rs:308](../crates/crowd/src/lib.rs)), so it could
not be changed later without rebuilding the hash. Retuning means constructing a new `Crowd`.

## The tick model

`tick(dt)` is four ordered passes and nothing else:

1. **Replan and arrive.** Per agent: snap back onto the mesh if it drifted off, clear the
   goal if `pos.distance(target) <= arrive_radius`, then replan if the path is empty or
   `stuck >= stuck_ticks`. Replans call `find_path` with `distance_from_wall` set to the
   agent's `radius`.
2. **Rebuild the spatial hash.** A uniform grid over live agents, cleared and refilled from
   scratch every tick.
3. **Choose velocities.** Per agent: a preferred velocity toward the current corridor
   corner, then `vo_samples` angular candidates plus one zero-velocity brake, each scored
   and the best written to a scratch buffer.
4. **Integrate.** Commit the chosen velocities, advance positions, advance the corridor
   cursor, update the stuck counter.

Two consequences of that ordering are load-bearing.

Velocity choice reads each neighbour's **previous-tick** `agent.vel`, because `next_vels`
is only committed in pass 4 ([lib.rs:598-599](../crates/crowd/src/lib.rs)). The update is
simultaneous, not Gauss-Seidel: agent 0 and agent 200 see the same world, and adding an
agent does not change how earlier agents behave within the tick.

Arrival is checked at the *top* of the tick, before movement, so an agent that reaches its
goal during tick *n* has `goal == None` observable only after tick *n+1*.

`dt` affects only integration and the stuck threshold. `time_horizon` is in absolute
seconds and candidate scoring is dt-independent, so halving the timestep does not make
agents more cautious — it makes them take smaller steps toward the same chosen velocity.

A corridor is discarded by exactly six things: `set_goal`, `set_pos` and `set_radius` (all
call `invalidate_path`); `set_nav` finding the remaining route blocked; the defensive
off-mesh snap at the top of pass 1 ([lib.rs:519-527](../crates/crowd/src/lib.rs)); and
`stuck` reaching `stuck_ticks`. The first five leave an empty path that pass 1 replans on
the *next* tick; the `stuck` trigger is tested inside pass 1 and so replans within the same
tick. `set_max_speed` and `set_priority` are cheap and keep the corridor.

## CrowdConfig, field by field

Defaults are at [lib.rs:177-190](../crates/crowd/src/lib.rs).

| field | default | what it costs you if wrong |
|---|---|---|
| `vo_samples` | `16` | Angular candidates per agent per tick, spread over ±π around the preferred direction, *plus* one zero-velocity brake. The implementation uses `n = vo_samples.max(2)`, emits `k in 0..=n/2` on the positive side and mirrors `0 < k < n/2`, so 16 yields 16 angular candidates and 17 evaluations total. Too low and the solver cannot find the gap it needs; every sample is a full neighbour sweep, so this is the main per-agent cost dial. |
| `neighbor_radius` | `6.0` | World-space cutoff for avoidance; the actual search is `neighbor_radius + me.radius`. Too small and agents notice each other too late; too large and every agent scores against a crowd it will never touch. Frozen into the hash cell size at construction. |
| `time_horizon` | `1.5` s | Time-to-collision horizon. A candidate whose predicted TTC is under this is penalised linearly more the sooner the collision. Long horizons make agents swerve at distances that look timid. |
| `stuck_ticks` | `60` (≈1 s at 60 Hz) | Consecutive ticks of near-zero progress before the corridor is discarded and rebuilt. "Near-zero" is `moved < max_speed * dt * 0.1` ([lib.rs:625-631](../crates/crowd/src/lib.rs)). This is also the *only* recovery path from a corridor that is geometrically clear but too narrow — see `set_nav` below. |
| `align_weight` | `1.0` | Weight on `v · v_pref / max_speed²`. |
| `avoid_weight` | `2.0` | Weight on the TTC penalty; the caution dial. Heavier gives wider berths and slower throughput; lighter gives tighter packing and more brushing contact. |
| `arrive_eps` | `0.25` | How close the agent must come to the current corner before the cursor advances. Set below the agent's per-tick travel distance and the cursor can fail to advance, which reads as an agent orbiting a corner. |
| `hold_speed_frac` | `0.5` | Speed, as a fraction of `max_speed`, at which a *goal-less* agent nudges out of the way. This is why a parked agent yields instead of being an immovable wall. `0.0` restores fully immovable idle agents; `choose_hold_velocity` early-returns `Vertex::ZERO` when the nudge is not positive. |

The score being maximised per candidate is
`align_weight * (v·v_pref / max_speed²) - avoid_weight * collision_penalty(v)`
([lib.rs:658-669](../crates/crowd/src/lib.rs)).

## Reading agent state

`Crowd` hands out `&Agent`, never `&mut Agent`. All of `Agent`'s fields are `pub`, which
reads like you can mutate them, but after insertion the only routes are the five setters.
`agent(id) -> Option<&Agent>` for one, `agents() -> impl Iterator<Item = (AgentId, &Agent)>`
for every live one (holes skipped).

For debug rendering there are two more:

- `path(id) -> &[Vertex]` — the funnel-pulled corridor. `path[0]` is the agent's position
  *at plan time*, `path.last()` is the goal.
- `path_cursor(id) -> Option<usize>` — the index of the corner currently being steered
  toward. Draw `[agent.pos] ++ path[cursor..]` to show only the remaining route; drawing the
  whole `path` includes a leg the agent walked minutes ago.

The cursor can equal `path.len()` once every corner has been consumed
([lib.rs:616-623](../crates/crowd/src/lib.rs)), at which point `preferred_velocity` falls
back to steering at `goal.target` directly.

## Parked agents

An agent with no goal, or one whose last replan failed, does not go through candidate
scoring at all — it goes to `choose_hold_velocity`
([lib.rs:728-785](../crates/crowd/src/lib.rs)), which is three cases in order:

1. If it currently overlaps neighbours, move along the penetration-depth-weighted separation
   vector, so a pile of parked agents decompresses rather than staying jammed. Exactly
   coincident agents are fanned apart by golden-angle on their slot index, so they do not all
   pick the same escape direction.
2. Otherwise, if the zero-velocity candidate already carries a nonzero collision penalty,
   take the lowest-penalty nudge from a full circle of `vo_samples.max(4)` candidates at
   `hold_speed_frac * max_speed`.
3. Otherwise stay put.

This is why a stationary agent is soft rather than an obstacle, and why an agent with an
unreachable goal drifts slightly instead of locking in place.

## Radius and priority

`Agent::radius` does double duty. It is passed as `PathOptions::distance_from_wall` on
every replan, so it is the agent's planning clearance, *and* it is the disc radius used for
disc-disc avoidance. Changing it via `set_radius` therefore invalidates the path. Note the
consequence for units and for double counting: `distance_from_wall` is in world units and
is not a Euclidean guarantee, and if your navmesh was already built with a baked inset you
must account for that — see [06](06-clearance.md), which owns that arithmetic.

`Agent::priority` is right-of-way. With `diff = other - me` clamped to ±4, the per-neighbour
scaling is exactly `2^(diff/2)`, bounding the factor to `[0.25, 4.0]`
([lib.rs:862-865](../crates/crowd/src/lib.rs)). At `diff == 0` it is exactly `1.0`, so a
crowd that never touches `priority` is unaffected. The factor is applied **only** to the
predicted-collision branch; an already-overlapping contact sets the penalty to `1.0`
undiscounted ([lib.rs:823-827](../crates/crowd/src/lib.rs)), so even a top-priority agent
still separates from a neighbour it is currently inside.

## Swapping the navmesh

`Crowd::set_nav(Arc<NavBuild>)` is the handoff from `NavWorker::poll_swap` ([10](10-dynamic-rebuilds.md)
owns that protocol). It builds **one** `WallInfo` for the whole sweep, then for each agent
with a non-empty path revalidates `[agent.pos, remaining corners…]` — the agent's current
position, not the stale plan-time start — with `rsnav_navigation::path_clear`. Routes still
clear are kept; only broken ones are invalidated for replan on the next tick. That avoids a
global replan storm when a rebuild was cosmetic or strictly additive.

Validation is per-segment, not corner-only, so a building that spawns *between* two
still-on-mesh corners is caught. The test
`set_nav_invalidates_path_blocked_mid_segment` ([lib.rs:1071-1097](../crates/crowd/src/lib.rs),
test code, not a runnable example) pins exactly that.

The stated non-goal: revalidation is **zero-width**. `path_clear` is a line-of-sight test
and does not re-verify the agent's radius of clearance. A rebuild that narrows a corridor
below body width without severing its centerline keeps the route, and the agent only
recovers when `stuck_ticks` ticks of no progress force a clearance-aware replan. A swept-disc
revalidation is deliberately not implemented; it would cost more than the occasional late
replan it saves ([lib.rs:345-355](../crates/crowd/src/lib.rs)).

## Behaviours that will surprise you

**Perfectly symmetric head-on encounters deadlock.** Identical radii, speeds and priority
give both agents the same score surface, and neither picks a side. Both head-on tests in the
crate (`two_agents_head_on_pass_without_collision`, `higher_priority_agent_yields_less`) and
the shipped example break it with a 0.05 offset in y — the example places the two agents
at `y = 5.05` and `y = 4.95`
([two_agents_pass.rs:51-52](../crates/crowd/examples/two_agents_pass.rs)). Fix with a
positional offset, distinct goal targets, or distinct priorities.

**There is no arrival deceleration.** Every angular candidate has magnitude exactly
`max_speed`, and the only other candidate is the zero-velocity brake. `preferred_velocity`
always returns full speed toward the next corner. Agents run flat out or stop dead, and the
goal is cleared by proximity rather than by slowing into it. If you want easing, apply it
yourself when rendering, not by tuning config.

**The sim quietly teleports agents.** Avoidance never consults the navmesh — it trusts the
corridor for wall clearance. Two defensive snaps compensate: `integrate` clamps a proposed
position that fails `bsp.locate` to `bsp.nearest`
([lib.rs:607-614](../crates/crowd/src/lib.rs)), and `replan_and_arrive` re-snaps an
off-mesh agent at the top of the tick ([lib.rs:519-527](../crates/crowd/src/lib.rs)). The
useful effect is that an agent squeezed against a wall slides along it rather than through
it. The consequence you must plan for is that a position read between ticks may not equal
`pos + vel * dt`, so client-side prediction or dead reckoning off `Agent::vel` will drift.

**An agent with an unreachable goal runs a full A\* every single tick, forever.** On a
failed replan the arm sets `plan_failed = true`, clears the path and resets `stuck` to 0
([lib.rs:560-565](../crates/crowd/src/lib.rs)). The next tick's guard is
`slot.path.is_empty() || slot.stuck >= stuck_ticks`, and an empty path makes that true
immediately. `plan_failed` does not latch and does not suppress anything; it is read in
exactly two places, the public getter and the branch routing the agent to hold-velocity.
This contradicts `Slot::plan_failed`'s own field doc, which claims it "suppresses further
replan attempts" — errata (d) in [16](16-troubleshooting.md). Poll `plan_failed(id)` and
clear or relocate the goal yourself; nothing else will.

**`AgentId` slots are recycled and carry no generation tag.** `remove_agent` leaves a
`None` hole and the next `add_agent` fills the first hole it finds
([lib.rs:381-386](../crates/crowd/src/lib.rs)), so an id held across a removal can
silently address a different agent. Do not cache `AgentId` in long-lived game state without
your own validity check.

**All five setters silently no-op on an unknown id.** `set_goal`, `set_pos`, `set_radius`,
`set_max_speed` and `set_priority` return nothing and report nothing when the slot is empty
or out of range. So does `path()`, which returns an empty slice for a dead id and therefore
cannot distinguish "no agent" from "no path" — `path_cursor()` returns `None` for a dead id
and is the one to use if you need to tell them apart.

**`slots` never shrinks.** `agent_count()` is an O(slots) scan, `next_vels` is resized to
`slots.len()` every tick, and every per-tick pass iterates the full slab. A crowd that
peaked at 10,000 agents keeps paying 10,000-wide loops with 10 live agents. If your peak
and steady-state counts differ by an order of magnitude, construct a fresh `Crowd` rather
than relying on removals to reclaim anything.

## Specification by test

There is one example and no other runnable driver, so the in-crate tests are the closest
thing to a behavioural specification. All of the following are `#[cfg(test)]` code in
[`crates/crowd/src/lib.rs`](../crates/crowd/src/lib.rs) and are not usage samples:

- `single_agent_walks_to_goal_on_open_arena` (lib.rs:903) — the baseline arrival contract.
- `two_agents_head_on_pass_without_collision` (lib.rs:925) — the pass, with the symmetry-breaking offset.
- `parked_agents_decompress` (lib.rs:984) — `hold_speed_frac` unjamming a pile of goal-less agents.
- `idle_agent_stays_put` (lib.rs:1006) — the other half: soft, but it does settle.
- `set_nav_keeps_still_valid_paths` (lib.rs:1019), `set_nav_clears_paths_whose_goal_left_the_mesh` (lib.rs:1045), `set_nav_invalidates_path_blocked_mid_segment` (lib.rs:1072) — the three revalidation outcomes.
- `priority_factor_is_neutral_at_equal_priority` (lib.rs:1100), `higher_priority_agent_yields_less` (lib.rs:1113) — the priority contract.

## Scaling honesty

`rsnav-crowd` is single-threaded throughout and uses none of `rsnav-common`'s `par`
primitives. Per tick, cost is roughly `agents × vo_samples × neighbours_in_range`, on top of
one `find_path` per replanning agent — and `find_path` rebuilds an O(triangles) `WallInfo`
on every call ([07](07-paths-and-queries.md)), which the crowd does not hoist. With many
agents replanning in the same tick, that rebuild, not the avoidance solver, is what you will
see in a profile. Stagger replans by staggering goal assignment if it hurts.

Determinism: the workspace guarantees byte-identical *navmesh builds* across thread counts
([15](15-performance-and-determinism.md)). That guarantee does not extend to `Crowd`. Reading
the source, a crowd looks deterministic — the spatial hash is only ever read through
`bins.get(&key)` in a fixed dx/dy order, per-bin index vectors are filled in ascending slot
order, and the collision penalty takes a max, which is order-independent — but there is no
test asserting it and none was run. Treat crowd determinism as unverified and do not build
lockstep networking or replay on it without measuring first.
