#!/usr/bin/env python3
"""Static consistency checks for the follow deployment manifest.

These are the failure modes that are invisible until the stack is already
booting: a command-chain topic that nothing consumes, a provider the body
description never declares, a config key the package silently drops.  All of
them are cheap to catch offline.

Exit status is non-zero if any check fails.
"""

from __future__ import annotations

from dataclasses import fields
import sys
from pathlib import Path

import yaml


DEPLOY_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(DEPLOY_DIR / "packages" / "go2_robottrack"))

from go2_robottrack.core import RuntimeConfig  # noqa: E402


class Failure(Exception):
    pass


def _load(name: str):
    with open(DEPLOY_DIR / name, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _expand(value, env: dict):
    if not isinstance(value, str):
        return value
    for key, replacement in env.items():
        value = value.replace(f"${{{key}}}", str(replacement))
    return value


def _providers(document) -> set[str]:
    return {
        entry["name"] for entry in (document.get("primitive") or [])
    } | {entry["name"] for entry in (document.get("service") or [])}


def _soma_providers(soma) -> set[str]:
    found: set[str] = set()

    def walk(components):
        for component in components or []:
            for export in component.get("exports") or []:
                found.add(export["provider_id"])
            walk(component.get("components"))

    for export in soma["robot"].get("exports") or []:
        found.add(export["provider_id"])
    walk(soma["robot"].get("components"))
    return found


def check_command_chain(manifest, config) -> list[str]:
    """Every hop of the velocity chain must join up."""

    notes: list[str] = []
    env = manifest.get("env") or {}

    nav2 = next(s for s in manifest["service"] if s["name"] == "nav2")["config"]
    nav2_guard_output = _expand(nav2["velocity_output_topic"], env)
    if nav2_guard_output != config.guard_input_topic:
        raise Failure(
            "the Nav2 velocity guard writes "
            f"{nav2_guard_output!r} but the chassis guard reads "
            f"{config.guard_input_topic!r}; nothing would reach the chassis"
        )
    notes.append(f"nav2 velocity guard -> {nav2_guard_output} -> chassis guard")

    chassis = next(
        p for p in manifest["primitive"] if p["name"] == "ranger_chassis"
    )["config"]
    if chassis["cmd_vel_topic"] != config.guard_output_topic:
        raise Failure(
            f"the chassis subscribes {chassis['cmd_vel_topic']!r} but the guard "
            f"publishes {config.guard_output_topic!r}"
        )
    notes.append(f"chassis guard -> {chassis['cmd_vel_topic']} -> ranger_chassis")

    if config.selected_output_topic == config.guard_output_topic:
        raise Failure(
            "the source mux and the chassis guard would both write "
            f"{config.guard_output_topic!r}"
        )
    notes.append(f"source mux -> {config.selected_output_topic} -> velocity smoother")
    return notes


def check_config_keys(manifest, config) -> list[str]:
    """The manifest must not carry keys the package silently discards."""

    block = next(
        p for p in manifest["primitive"] if p["name"] == "go2_robottrack"
    )["config"]
    known = {field.name for field in fields(RuntimeConfig)}
    # Consumed by the provider as deployment metadata, not by RuntimeConfig.
    metadata = {"camera_info_topic", "asset_manifest", "upstream_root", "source_mux"}
    dropped = sorted(set(block) - known - metadata)
    if dropped:
        raise Failure(f"manifest keys the package would ignore: {dropped}")
    return [f"all {len(block)} robottrack config keys are recognized"]


def check_providers(manifest, soma) -> list[str]:
    """Every manifest provider must be declared by the body description."""

    declared = _soma_providers(soma)
    manifest_providers = _providers(manifest)
    # The RobotTrack primitive registers through Atlas rather than the body
    # description, matching the Go2 reference.
    self_registering = {"go2_robottrack"}
    missing = sorted(manifest_providers - declared - self_registering)
    if missing:
        raise Failure(f"manifest providers absent from soma.yaml: {missing}")
    extra = sorted(declared - manifest_providers)
    if extra:
        raise Failure(f"soma.yaml declares providers the manifest lacks: {extra}")
    return [f"{len(declared)} providers declared by soma.yaml"]


def check_follow_profile(config) -> list[str]:
    """Check the profile values, but never forbid a supported mode.

    Two modes are first-class in the reference deployment, each with its own
    launcher: plain RGB follow (``distance_enabled: false``, the model's own vx
    is used) and fixed-distance follow (``distance_enabled: true``, the depth
    layer overrides vx).  Neither is an error, so this reports which one is
    configured instead of rejecting one -- the same mistake as forbidding
    ``mode: live``, which made the deployment impossible to arm.

    The distance tuning values only have to match the reference while the
    distance layer is actually running.
    """

    problems = []
    if abs(config.max_vx - 0.50) > 1e-9:
        problems.append(f"max_vx is {config.max_vx}, expected the 0.50 live profile")
    if config.distance_enabled:
        if abs(config.distance_deadband_m - 0.10) > 1e-9:
            problems.append(
                f"distance_deadband_m is {config.distance_deadband_m}, expected 0.10"
            )
        if abs(config.distance_kp - 0.50) > 1e-9:
            problems.append(f"distance_kp is {config.distance_kp}, expected 0.50")
    if config.guard_input_topic == config.guard_output_topic:
        problems.append(
            "the chassis guard reads and writes the same topic; it must sit "
            "between the Nav2 guard and the chassis"
        )
    if config.reid_enabled and (
        config.distance_identity_max_age_s > config.distance_measurement_max_age_s
    ):
        problems.append(
            "distance_identity_max_age_s "
            f"({config.distance_identity_max_age_s}) exceeds "
            "distance_measurement_max_age_s "
            f"({config.distance_measurement_max_age_s}): with the lock on, a "
            "measurement could outlive the ordinary staleness cap, which is "
            "exactly the coast the identity window exists to prevent"
        )
    if problems:
        raise Failure("; ".join(problems))

    notes = [
        f"command mode is {config.mode!r}"
        + (" (no velocity is published)" if config.mode == "dry-run" else " (ARMED)"),
    ]
    if config.distance_enabled:
        notes.append(
            "distance layer ON: vx comes from the depth measurement "
            f"(deadband {config.distance_deadband_m} / kp {config.distance_kp})"
        )
    if config.reid_enabled:
        notes.append(
            "specified-person lock ON: vx requires a verified match "
            f"(identity window {config.distance_identity_max_age_s}s); "
            "with nobody enrolled the robot turns but does not translate"
        )
    else:
        notes.append(
            "distance layer OFF: vx comes straight from the model; no fixed "
            "distance is held and depth/HOG are not subscribed"
        )
    return notes


def check_pre_smoother_optin(manifest) -> list[str]:
    """The Nav2 service in use must support the pre-smoother raw output.

    `controller_velocity_output_topic` is what keeps the source mux the sole
    publisher of /cmd_vel_nav.  The revision that adds it is not on upstream
    main, which is exactly how a `url:` + `branch: main` entry silently loses
    it and reintroduces a second publisher on that topic.
    """

    nav2 = next(s for s in manifest["service"] if s["name"] == "nav2")
    if "controller_velocity_output_topic" not in nav2["config"]:
        raise Failure(
            "nav2 config does not set controller_velocity_output_topic; the "
            "Nav2 controller and the source mux would both publish /cmd_vel_nav"
        )
    if "path" not in nav2:
        raise Failure(
            "nav2 is fetched by url; the revision carrying the pre-smoother "
            "opt-in is not on main. Vendor it under third_party/ and use path:"
        )
    package = Path(_expand(nav2["path"], {"ROBONIX_DEPLOY_DIR": str(DEPLOY_DIR)}))
    configuration = package / "nav2_wrapper" / "configuration.py"
    if not configuration.is_file():
        raise Failure(f"vendored nav2 service has no {configuration}")
    if "controller_velocity_output_topic" not in configuration.read_text(
        encoding="utf-8"
    ):
        raise Failure(
            "the vendored nav2 service has no "
            "resolve_controller_velocity_output_topic; refresh it from the "
            "pinned revision recorded in third_party/README.md"
        )
    return [
        "nav2 vendored and exposes controller_velocity_output_topic "
        f"({nav2['config']['controller_velocity_output_topic']})"
    ]


def check_guard_ceiling(config) -> list[str]:
    """The chassis guard must not be looser than the smoother above it."""

    nav2_params = _load("config/nav2_params.yaml")
    smoother = nav2_params["velocity_smoother"]["ros__parameters"]
    smoother_vx = max(smoother["max_velocity"][0], smoother["max_velocity"][1])
    if config.guard_max_vx_mps > smoother_vx + 1e-9:
        raise Failure(
            f"guard_max_vx_mps {config.guard_max_vx_mps} exceeds the smoother's "
            f"{smoother_vx}; the guard would be the looser limit"
        )
    lifecycle = nav2_params["lifecycle_manager"]["ros__parameters"]["node_names"]
    if "velocity_smoother" not in lifecycle:
        raise Failure("velocity_smoother is not in the Nav2 lifecycle node list")
    if "smoother_server" not in lifecycle:
        raise Failure("smoother_server is not in the Nav2 lifecycle node list")
    return [
        f"guard {config.guard_max_vx_mps} m/s <= smoother {smoother_vx} m/s",
        "Nav2 lifecycle manages smoother_server + velocity_smoother",
    ]


def main() -> int:
    manifest = _load("robonix_manifest.yaml")
    soma = _load("soma.yaml")
    block = next(
        p for p in manifest["primitive"] if p["name"] == "go2_robottrack"
    )["config"]
    config = RuntimeConfig.from_mapping(block)

    checks = (
        ("command chain", lambda: check_command_chain(manifest, config)),
        ("config keys", lambda: check_config_keys(manifest, config)),
        ("providers", lambda: check_providers(manifest, soma)),
        ("follow profile", lambda: check_follow_profile(config)),
        ("pre-smoother opt-in", lambda: check_pre_smoother_optin(manifest)),
        ("guard ceiling", lambda: check_guard_ceiling(config)),
    )

    failed = 0
    for name, run in checks:
        try:
            notes = run()
        except Failure as error:
            failed += 1
            print(f"[FAIL] {name}: {error}")
            continue
        for note in notes:
            print(f"[ OK ] {name}: {note}")

    print()
    if failed:
        print(f"manifest consistency FAILED ({failed} check(s))")
        return 1
    print("manifest consistency OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
