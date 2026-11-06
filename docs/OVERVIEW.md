# skill-follow-person-rbnx

Robonix × MiniCPM-RobotTrack person-following deployment for the **AgileX Ranger
Mini v3** — a port of the Go2 reference (`robot-unitree-go2`, PR #6) with the
chassis swapped and everything else carried over.

## What was ported, and what changed

The reference's follow profile is *the existing navigation stack plus a
RobotTrack primitive*. The only intentional deviation here is the chassis:

| Layer | Go2 reference | This repo |
|---|---|---|
| Chassis | `go2_chassis` + SDK daemon (fail-closed gate, 0.25 m/s clamp, 0.25 s timeout) | `ranger_chassis` + **chassis guard** restoring the same contract |
| Camera | D435i | Orbbec Gemini 336L |
| Everything else | identical | identical |

Carried over unchanged: the source mux, the Nav2 velocity smoother, the
velocity guard, the Pilot/Executor capability path, the `follow/distance`
contract, and the Chinese follow-distance router.

## The command chain

```
RobotTrack ──────────────► /ranger/robottrack/cmd_vel_raw ─┐
                                                           ├─► source mux
Nav2 controller ─────────► /ranger/robottrack/nav_cmd_vel_raw ┘      │
                                                                     ▼
                                                               /cmd_vel_nav
                                                                     │
                                                     Nav2 velocity smoother
                                                                     ▼
                                                          /cmd_vel_guard_input
                                                                     │
                                                     Nav2 velocity guard
                                                                     ▼
                                                          /cmd_vel_guarded
                                                                     │
                                                     Ranger chassis guard
                                                                     ▼
                                                    /cmd_vel → ranger_chassis
```

RobotTrack never asks Nav2 to plan: the follow loop only reuses the smoother
and the guard, exactly as the reference states. Voice commands enter through
Pilot, whose `vlm.upstream` points at the local follow-distance router.

## Layout

```
robonix_manifest.yaml            # systems + primitives + services for follow
config/nav2_params.yaml          # Ranger nav2 params + smoother/velocity_smoother
config/navigate.xml              # Nav2 behaviour tree
config/rtabmap_params.yaml       # mapping params
config/robottrack_assets.yaml    # MiniCPM model asset pins (no download)
soma.yaml                        # minimal follow-only robot model
packages/go2_robottrack/         # follow bridge + Ranger chassis guard
packages/semantic_intent_router/ # Pilot's "VLM": NL → follow_distance calls
start.sh / stop.sh / build.sh    # thin rbnx wrappers (run inside the container)
scripts/                         # inference server, intent router, launcher, shutdown
upstream/MiniCPM-Robot/          # model weights (git-ignored, ~7.1 GB, mounted in)
```

## Run (inside the follow container)

1. **Stop navigation first** (mandatory — both stacks own `can_ranger`).
2. Then:

```bash
cp .env.example .env          # then fill in LIDAR_IP and the Tencent secrets
bash build.sh                 # rbnx build + colcon overlay
bash scripts/start_follow.sh  # inference server + intent router + rbnx boot
```

Bring the stack up in `mode: dry-run` first (the default), confirm the camera,
inference and prediction logs, then flip to `mode: live`.

`bash scripts/shutdown-follow.sh` performs a full teardown — `rbnx shutdown`
plus the two sidecars and any orphans it leaves behind — so `can_ranger` and the
host ports are actually released.

## What each script calls

`scripts/follow.sh` is the host-side entry point; everything else is either run
by it through `docker exec`, or run inside the container by `rbnx`.

### `rbnx`

| Script | Command | Purpose |
|---|---|---|
| `start.sh` | `rbnx boot --no-update-check -f <manifest>` | brings the whole stack up |
| `start.sh` (rollback) | `rbnx shutdown -f <manifest>` | only on a failed boot |
| `stop.sh`, `scripts/shutdown-follow.sh` | `rbnx shutdown -f <manifest>` | graceful teardown, keyed off `rbnx-boot/state.json` |
| `build.sh` | `rbnx build -f <manifest>` | builds every declared package |
| `packages/go2_robottrack/start.sh` | `rbnx path robonix-api` | locates the robonix-api library for `PYTHONPATH` |

`rbnx shutdown` only owns the components rbnx started. The inference server, the
intent router and `rmw_zenohd` are **not** rbnx's, so the shutdown script tracks
them by pid file and kills them itself.

### ROS 2

| Script | Interface | Purpose |
|---|---|---|
| `scripts/follow.sh status` | `ros2 topic info /cmd_vel` | distinguishes parked (0 publishers) / live (1) / **dangerous (>1)**; also reports which vision backends are live |
| `scripts/follow.sh check` | `ros2 topic hz` × 3 | camera health: `color/image_raw`, `depth/image_raw`, `color/camera_info`; plus the vision-backend banner |
| `scripts/watch_follow_chain.py` | one **rclpy** node | subscribes every hop of the chain and redraws a table per second |

The dashboard uses an rclpy subscriber rather than `ros2 topic hz` because
shelling out costs ~15 s per topic and there are five hops. Everything that
touches ROS first sources `/opt/ros/humble/setup.bash` and sets
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`.

### docker

| Script | Calls | Purpose |
|---|---|---|
| `scripts/run_container.sh` | `ps -a`, `stop`, `rm`, `run`, `inspect` | idempotent (re)creation; carries `--init`, `--runtime nvidia` + `NVIDIA_VISIBLE_DEVICES=all` (`--gpus all` no longer works on this host), the USB device-cgroup rule |
| `scripts/follow.sh` | `ps`, `exec`, `inspect`, `restart`, `start` | status, running commands inside, and the post-teardown container restart |

All of it goes through `sudo docker` — no docker group is required.

### Project files

| File | Consumer | Role |
|---|---|---|
| `robonix_manifest.yaml` | every script | single source of truth: mode, parameters, package declarations |
| `soma.yaml` | soma service | body description (URDF, components, providers) |
| `rbnx-boot/state.json` | `shutdown-follow.sh` | the only thing `rbnx shutdown` keys off; absent → skip rbnx, sweep instead |
| `rbnx-boot/cache/` | `rbnx boot` | per-package source checkouts |
| `third_party/service-navigation-rbnx` | manifest (`path:`) | nav2 with the pre-smoother opt-in, pinned |
| `packages/go2_robottrack` | manifest (`path:`) | follow bridge **and the chassis guard** |
| `packages/semantic_intent_router` | `start_follow.sh` directly | Pilot's "VLM" |
| `.env` | `start.sh`, `start_follow.sh` | secrets and machine-local overrides |

### Sidecars start_follow.sh owns

Started before `rbnx boot`, stopped by pid file (rbnx knows nothing about them):

| Process | Launched as | Endpoint |
|---|---|---|
| MiniCPM-RobotTrack inference | `scripts/start_robottrack_inference_server.sh` | 127.0.0.1:5801 |
| Follow-distance intent router | `packages/semantic_intent_router/scripts/start.sh` | 127.0.0.1:18080 |
| `rmw_zenohd` | `start.sh`, only if 7447 is free | 7447 |

The inference server's vision towers (DINOv3 + SigLIP) run on the on-device
TensorRT 10.7 FP16 engines. `ROBOTTRACK_VISION_BACKEND=torch` puts both back on
the Go2 reference's torch path; it is the supported rollback, not a broken
state. See `docs/specified-person-follow.md` Step 15.

### Where the chassis guard lives

Not in any shell script — it is inside the `go2_robottrack` package, and it is
started by the node itself:

```python
# packages/go2_robottrack/go2_robottrack/ros_node.py
if RuntimeConfig.from_mapping(config).mode == "live":
    guard = RangerChassisGuardNode(config_overrides=config, context=context)
```

It subscribes `/cmd_vel_guarded` and publishes `/cmd_vel` at 50 Hz. The policy
half (`chassis_guard.py`) imports no ROS, so it is unit-testable offline.
**In dry-run the node is never constructed**, which is what makes "nothing can
publish a velocity" true rather than merely intended.

## Why a separate repo

The hard rule is: **follow work must never break navigation.** Two physically
separate Robonix deployments — separate manifest, separate start/stop, separate
`rbnx-boot` state — removes the shared-manifest risk. The one remaining shared
resource is the `can_ranger` bus and `/cmd_vel`, owned by whichever stack is
running. That is the **single-service rule**: only one stack is up at a time.

## Safety

- The follow bridge defaults to `mode: dry-run`: it predicts and logs but
  publishes **no** velocity. In dry-run the chassis guard is not started either,
  so nothing writes `/cmd_vel`.
- Only the chassis guard writes `/cmd_vel`, and it enforces forward-only,
  the Go2 speed envelope, a command timeout and the Ranger turn-radius
  constraint — see `packages/go2_robottrack/go2_robottrack/chassis_guard.py`.
- `guard_min_turn_radius_m` is set from the vendor manual (0.81 m), not measured.
  Calibrate it on the robot before tuning follow behaviour.
- Always verify the navigation/SLAM stack still boots after any change.
