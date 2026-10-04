"""Micras plugin for ``tools/analyze.py``.

Knows what the Micras columns mean: the firmware's own monitoring variables
(``state``, ``pose_*``, ``reference_*``, ``control_*``, ``loop_*``) and the
board's devices (``motor_*_voltage``, ``wall_left_front`` and the other wall
sensors, ``pack_voltage``). ``analyze.py`` loads it when ``meta.json`` names the
``micras`` target.

State names come from the state events in ``meta.json``, never from ids written
here.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

#: Fraction of the supply beyond which a motor voltage counts as saturated.
SATURATION = 0.98
MAX_EVENTS = 50


def events(run, kind: str) -> list[dict]:
    """Events of one kind from ``meta.json``, in time order."""
    return [event for event in run.meta.get("events", []) if event.get("kind") == kind]


def run_start(run) -> float | None:
    """The first time the firmware enters RUN."""
    for event in events(run, "state"):
        if event.get("detail") == "RUN":
            return float(event["time"])

    return None


def running(run) -> np.ndarray:
    """Ticks between each entry into RUN and the next state change."""
    mask = np.zeros(run.time.shape, dtype=bool)
    timeline = events(run, "state")

    for index, event in enumerate(timeline):
        if event.get("detail") != "RUN":
            continue

        end = float(timeline[index + 1]["time"]) if index + 1 < len(timeline) else np.inf
        mask |= (run.time >= float(event["time"])) & (run.time < end)

    return mask


def statistics(values: np.ndarray, time: np.ndarray, mask: np.ndarray) -> dict:
    """Largest absolute value, when it happened, and the rms, over a mask."""
    valid = mask & np.isfinite(values)

    if not valid.any():
        return {"max": None, "max_time": None, "rms": None}

    magnitude = np.where(valid, np.abs(values), -np.inf)
    peak = int(np.argmax(magnitude))

    return {
        "max": float(magnitude[peak]),
        "max_time": float(time[peak]),
        "rms": float(np.sqrt(np.mean(values[valid] ** 2))),
    }


def pose_report(run, mask: np.ndarray) -> dict:
    """The firmware's pose estimate against the ground truth."""
    position = np.hypot(run.get("pose_x") - run.get("x"), run.get("pose_y") - run.get("y"))
    heading = np.angle(np.exp(1j * (run.get("pose_orientation") - run.get("yaw"))))
    valid = np.isfinite(position)

    return {
        "position_error": statistics(position, run.time, mask),
        "heading_error_deg": {
            key: (float(np.degrees(value)) if key != "max_time" and value is not None else value)
            for key, value in statistics(heading, run.time, mask).items()
        },
        "final_position_error": float(position[valid][-1]) if valid.any() else None,
    }


def tracking_report(run, mask: np.ndarray) -> dict:
    """How far the pose estimate strayed from the reference the controller followed."""
    return {
        "along": statistics(run.get("control_along_error"), run.time, mask),
        "across": statistics(run.get("control_across_error"), run.time, mask),
        "orientation": statistics(run.get("control_orientation_error"), run.time, mask),
    }


def saturation_report(run, mask: np.ndarray) -> dict:
    """Fraction of running ticks each motor asked for the whole supply."""
    supply = run.get("pack_voltage")
    report = {}

    for side in ("left", "right"):
        voltage = run.get(f"motor_{side}_voltage")
        valid = mask & np.isfinite(voltage) & np.isfinite(supply) & (supply > 0)
        saturated = np.abs(voltage) >= SATURATION * supply
        report[side] = float(np.mean(saturated[valid])) if valid.any() else None

    return report


def loop_report(run) -> dict:
    """The firmware's own account of its control loop."""

    def largest(name: str) -> float | None:
        values = run.get(name)
        return float(np.nanmax(values)) if np.isfinite(values).any() else None

    return {
        "worst_time_us": largest("loop_worst_time_us"),
        "missed_ticks": largest("loop_missed_ticks"),
        "saturated_iterations": largest("loop_saturated_iterations"),
        "link_dropped_samples": largest("link_dropped_samples"),
        "link_dropped_logs": largest("link_dropped_logs"),
    }


def report(run, generic: dict) -> dict:
    """The firmware's states, how it ran, and what the board counted."""
    mask = running(run)

    return {
        "states": [{"time": event["time"], "state": event["detail"]} for event in events(run, "state")][:MAX_EVENTS],
        "unbound_ports": run.meta.get("unbound_ports"),
        "running_time": float(mask.sum() * run.dt),
        "pose": pose_report(run, mask),
        "tracking": tracking_report(run, mask),
        "voltage_saturation_fraction": saturation_report(run, mask),
        "loop": loop_report(run),
    }


def speed_traces(run) -> list[tuple[str, np.ndarray, str]]:
    """The reference and the estimate of both speeds."""
    return [
        ("reference_linear_speed", run.get("reference_linear_speed"), "linear"),
        ("pose_linear_speed", run.get("pose_linear_speed"), "linear"),
        ("reference_angular_speed", run.get("reference_angular_speed"), "angular"),
        ("pose_angular_speed", run.get("pose_angular_speed"), "angular"),
    ]


def trajectory_overlays(run) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """The firmware's pose estimate and reference while it runs.

    The origin is the corner post, where no pose can be: a pair at exactly (0, 0) is a
    variable not yet written on the first tick of a run, and is left out.
    """
    overlays = []

    for label, prefix in (("pose estimate", "pose"), ("reference", "reference")):
        x = run.get(f"{prefix}_x")
        y = run.get(f"{prefix}_y")
        hide = np.where(running(run) & ((x != 0.0) | (y != 0.0)), 1.0, np.nan)
        overlays.append((label, x * hide, y * hide))

    return overlays


def plots(run, generic: dict, directory: Path) -> None:
    """The controller's terms and the wall sensors, in control.png."""
    start = generic["run_start_time"]
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    for name in (
        "control_forward_feed_forward",
        "control_forward_feedback",
        "control_rotation_feed_forward",
        "control_rotation_feedback",
    ):
        axes[0].plot(run.time, run.get(name), label=name, linewidth=0.8)

    axes[0].set_ylabel("command [V]")
    axes[0].legend(fontsize=7)

    for name in run.prefixed("wall_"):
        axes[1].plot(run.time, run.get(name), label=name, linewidth=0.8)

    axes[1].set_ylabel("wall sensor [counts]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=7)

    for axis in axes:
        if start is not None:
            axis.axvline(start, color="k", linestyle="--", linewidth=0.8)

    figure.tight_layout()
    figure.savefig(directory / "control.png", dpi=120)
    plt.close(figure)


def fmt(value, digits: int = 3) -> str:
    """Format a possibly missing number for the summary."""
    return "n/a" if value is None else f"{value:.{digits}f}"


def summarize(full: dict) -> None:
    """The states, the pose and tracking errors, saturation and the loop, on stdout."""
    print("states       " + "  ".join(f"{entry['time']:.3f}s {entry['state']}" for entry in full["states"]))

    pose = full["pose"]
    position = pose["position_error"]
    heading = pose["heading_error_deg"]
    print(
        f"pose error   max {fmt(position['max'], 4)} m at {fmt(position['max_time'])} s  "
        f"rms {fmt(position['rms'], 4)} m  final {fmt(pose['final_position_error'], 4)} m  "
        f"heading max {fmt(heading['max'], 2)} deg"
    )

    tracking = full["tracking"]
    print(
        f"tracking     along rms {fmt(tracking['along']['rms'], 4)} m  "
        f"across rms {fmt(tracking['across']['rms'], 4)} m  "
        f"orientation rms {fmt(tracking['orientation']['rms'], 4)} rad"
    )

    saturation = full["voltage_saturation_fraction"]
    print(f"saturation   left {fmt(saturation['left'])}  right {fmt(saturation['right'])}")

    loop = full["loop"]
    print(
        f"loop         worst {fmt(loop['worst_time_us'], 0)} us  missed {fmt(loop['missed_ticks'], 0)}  "
        f"saturated {fmt(loop['saturated_iterations'], 0)}"
    )


def baseline(full: dict) -> dict:
    """What a Micras baseline records beyond the engine's values, with tolerances."""
    pose = full["pose"]["position_error"]
    across = full["tracking"]["across"]

    return {
        "unbound_ports": (full["unbound_ports"], None),
        "running_time": (full["running_time"], 1.0),
        "max_pose_error": (pose["max"], 0.02),
        "rms_across_error": (across["rms"], 0.003),
    }
