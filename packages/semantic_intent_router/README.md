# semantic_intent_router (follow-only profile)

A loopback, OpenAI-compatible `/v1/chat/completions` endpoint that Robonix Pilot
uses as its `vlm.upstream`. It is deliberately **not** a language model: it maps
one bounded Chinese utterance onto exactly one Robonix capability call, or onto
nothing at all.

## Why Pilot points at this

The reference deployment routes voice commands through Pilot. Pilot expects a
VLM behind an OpenAI-compatible API, so this endpoint plays that role while
staying exact-match and offline. That keeps Pilot, Executor and the capability
contract in the loop — the router can only ask for a setpoint change, never a
velocity.

## What it can emit

Exactly one capability, from `packages/go2_robottrack`:

```
go2_robottrack.follow_distance     (contract robonix/primitive/follow/distance)
  operation: set | adjust | get
  meters:    absolute distance, or a signed delta for adjust
```

It never publishes a `Twist`, never names a `cmd_vel` topic, and never bypasses
the chassis guard. Changing the target distance changes what the distance
controller aims for; the command still travels the normal
`source mux -> velocity smoother -> velocity guard -> chassis guard -> chassis`
path.

## Accepted utterances

| Utterance | Intent |
|---|---|
| `离我远一点` / `再离我远点儿` | `adjust +1.0 m` |
| `靠近一点` / `向我靠近一点` | `adjust -1.0 m` |
| `保持5米` / `保持五米` | `set 5.0 m` |
| `跟随距离设为3米` | `set 3.0 m` |
| `设置为一点五米` | `set 1.5 m` |

Arabic and Chinese numerals (including `点` decimals) are both accepted. A
relative request is exactly one step and is never replayed after executor
feedback, so repeating an utterance cannot accumulate metres.

## Difference from the Go2 reference

The Go2 package routers two intent families: this one, and semantic-navigation
landmarks (which needs a saved-landmark store and the `semantic_navigation`
skill). The pure-follow deployment carries no semantic-navigation skill, so that
family is absent here. An unrecognized utterance returns an empty RTDL tree
rather than a guess.

## Not every user-role message is the user

Pilot replans after each plan, and feeds results back as **user-role** messages.
Four kinds are injected (`system/pilot/src/planner.rs`, `history.rs`):

| starts with | carries |
|---|---|
| `Executor feedback for the current RTDL leaf (not a new user request): ` | the `leaf_result` JSON this router reads |
| `Executor feedback scope: ` | a per-tree attribution line, pushed *before* that tree's results |
| `Executor feedback: ` | an observation-image note |
| `Pilot harness feedback: ` | a plan-control correction |

The utterance is found by scanning for the last user turn, so every one of these
must be excluded from that scan. Only the first is treated as a *result* --
`leaf_results` slices past that exact prefix. Excluding them by label rather than
by whole sentence is what keeps a Pilot rewording from silently turning the scope
line back into "the command".

## Run and test

```bash
bash scripts/start.sh                      # 127.0.0.1:18080 by default
bash tests/run_offline_tests.sh            # no ROS, no network, no robot
```

`SEMANTIC_INTENT_HOST` must be a literal loopback address; binding elsewhere is
refused at construction.
