# robot-unitree-ranger

Standalone MiniCPM-RobotTrack person-following deployment for the **AgileX
Ranger Mini v3**, split off from the navigation/SLAM deployment
(`robot-agilex-ranger_mini_v3`) so the two can never interfere.

This repo follows `robot-unitree-go2` in spirit but carries only what follow
needs: a Ranger chassis primitive, an Orbbec Gemini 336L camera primitive, and
the MiniCPM-RobotTrack follow bridge.

## Why a separate repo

The hard rule is: **follow work must never break navigation.** Keeping the two
as physically separate Robonix deployments — separate `robonix_manifest.yaml`,
separate start/stop, separate `rbnx-boot` state — removes the shared-manifest
risk entirely. The one remaining shared resource is the `can_ranger` CAN bus and
`/cmd_vel`, owned by whichever stack is running. That is governed by the
**single-service rule**: only one stack is up at a time.

## Layout

```
robonix_manifest.yaml          # 3 primitives: ranger_chassis, front_camera, go2_robottrack
soma.yaml                      # minimal follow-only robot model
build.sh / start.sh / stop.sh  # thin rbnx wrappers (run inside the container)
packages/go2_robottrack/       # follow bridge (model-agnostic; copied from robot-unitree-go2)
config/robottrack_assets.yaml  # MiniCPM model asset pins (no download)
scripts/                       # inference server + follow launcher + asset verify
upstream/MiniCPM-Robot/        # model weights (git-ignored, mounted at runtime, ~7.1GB)
```

## Run (inside the robonix container)

The whole stack runs inside the `robonix_ros_dev` container. The model tree
lives OUTSIDE the container under `upstream/MiniCPM-Robot` (git-ignored) and is
mounted in at runtime; nothing is downloaded.

1. **Stop navigation first** (mandatory — both stacks own `can_ranger`).
2. Mount this repo into the container, then:

```bash
bash build.sh                              # rbnx build -f robonix_manifest.yaml
bash scripts/start_follow.sh               # inference server + rbnx boot
bash stop.sh                               # rbnx shutdown
```

## Safety

- The follow bridge defaults to `mode: dry-run`: it predicts and logs but
  publishes **no** velocity to `/cmd_vel` until a human sets `mode: live` in
  `robonix_manifest.yaml`.
- Always verify the navigation/SLAM stack still boots after any change.
- See `MiniCPM_RobotTrack部署环境安装记录.md` (sibling doc) for full progress,
  pitfalls, and acceptance records.
