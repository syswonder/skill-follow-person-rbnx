# third_party

Vendored source that must not silently move under a network fetch.

## service-navigation-rbnx

| | |
|---|---|
| Upstream | <https://github.com/syswonder/service-navigation-rbnx> |
| Pinned revision | `d10fe3dc96110473b8b778fe92527dc38fb8d6ac` |
| Commit subject | `feat(navigation): expose optional raw velocity output` |
| On upstream `main`? | **No** — it is a PR-branch commit |

### Why it is vendored

The follow command chain needs the Nav2 **pre-smoother raw output** opt-in:

```yaml
controller_velocity_output_topic: /ranger/robottrack/nav_cmd_vel_raw
```

That key makes `controller_server` (and `behavior_server`) publish somewhere
other than `/cmd_vel_nav`, so the RobotTrack source mux is the **sole**
publisher of `/cmd_vel_nav`. Without it both the controller and the mux write
that topic and the smoother receives two racing producers.

The commit that adds it is not on upstream `main`, so `branch: main` cannot
supply it. The Go2 reference solves this the same way — it gitlink-pins the
submodule to exactly this commit — and documents it as a temporary pin "until
the upstream PR is merged".

Vendoring also means the follow deployment cannot change underneath a
`rbnx boot` because upstream moved.

### What it costs

The copy is 444 KB (excluding `.git`) and is **not** a git submodule here: it is
plain source, so `git status` in this repo shows it as ordinary files. When the
upstream PR merges, replace the directory with a `url:` + `branch: main` entry
in `robonix_manifest.yaml` and delete this note.

`scripts/validate_manifest_consistency.py` asserts the opt-in is present, so a
bad refresh fails offline rather than at boot.

### Scope

This directory is used **only** by the follow deployment. The navigation
deployment (`robot-agilex-ranger_mini_v3`) keeps its own pinned copy of this
service in its own `rbnx-boot/cache/` and is unaffected.
