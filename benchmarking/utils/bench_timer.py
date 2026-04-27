"""Shared timing helpers for the benchmarking scripts.

Each benchmark script measures three phases:
  1. startup_sim_app  - SimulationApp construction
  2. startup_world_and_scene - imports, World, env/ground, drone, cube, world.reset()
  3. runtime - fall (cube drops) + steady-state (stable RTF sample)

Results are written to benchmarking/results/<script_stem>.json so
run_all.py can aggregate and plot them.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

CUBE_SPAWN_Z = 10.0
CUBE_SIZE = 0.5
CUBE_LANDED_Z = 0.26
FALL_TIMEOUT_SIM_S = 10.0
STEADY_TARGET_SIM_S = 5.0
ROLLING_WINDOW_SIM_S = 1.0


class BenchTimer:
    """Wall-clock phase timer backed by time.perf_counter()."""

    def __init__(self) -> None:
        self._phases: dict[str, float] = {}
        self._starts: dict[str, float] = {}

    def start(self, name: str) -> None:
        self._starts[name] = time.perf_counter()

    def stop(self, name: str) -> float:
        elapsed = time.perf_counter() - self._starts[name]
        self._phases[name] = elapsed
        return elapsed

    def get(self, name: str) -> float:
        return self._phases[name]

    def as_dict(self) -> dict[str, float]:
        return dict(self._phases)


@dataclass
class RuntimeStats:
    fall_wallclock_s: float = 0.0
    fall_simulated_s: float = 0.0
    fall_steps: int = 0
    steady_wallclock_s: float = 0.0
    steady_simulated_s: float = 0.0
    steady_steps: int = 0
    rolling_rtf: list[dict[str, float]] = field(default_factory=list)
    landed: bool = False
    timed_out: bool = False


def run_cube_fall_and_steady(
    world: Any,
    cube: Any,
    physics_dt: float,
    render: bool = True,
    is_running: Any = None,
) -> RuntimeStats:
    """Step the world until the cube lands, then sample steady-state RTF.

    Args:
        world: omni.isaac.core World instance.
        cube: DynamicCuboid (or XFormPrim) with get_world_pose().
        physics_dt: expected simulated seconds per physics step.
        render: forwarded to world.step().
        is_running: optional zero-arg callable returning False to abort early
                    (use simulation_app.is_running).

    Returns:
        RuntimeStats with fall + steady-state timings and a rolling-RTF series.
    """
    stats = RuntimeStats()
    max_fall_steps = int(FALL_TIMEOUT_SIM_S / physics_dt)
    max_steady_steps = int(STEADY_TARGET_SIM_S / physics_dt)
    rolling_every = max(1, int(ROLLING_WINDOW_SIM_S / physics_dt))

    def _running() -> bool:
        return True if is_running is None else bool(is_running())

    # --- Fall phase ---
    fall_start = time.perf_counter()
    while _running() and stats.fall_steps < max_fall_steps:
        world.step(render=render)
        stats.fall_steps += 1
        z = float(cube.get_world_pose()[0][2])
        if z <= CUBE_LANDED_Z:
            stats.landed = True
            break
    stats.fall_wallclock_s = time.perf_counter() - fall_start
    stats.fall_simulated_s = stats.fall_steps * physics_dt

    if not stats.landed:
        stats.timed_out = True
        return stats

    # --- Steady-state phase ---
    steady_start = time.perf_counter()
    window_wallclock_start = steady_start
    window_sim_start = 0.0
    while _running() and stats.steady_steps < max_steady_steps:
        world.step(render=render)
        stats.steady_steps += 1

        if stats.steady_steps % rolling_every == 0:
            now = time.perf_counter()
            window_wallclock = now - window_wallclock_start
            window_sim = stats.steady_steps * physics_dt - window_sim_start
            rtf = window_sim / window_wallclock if window_wallclock > 0 else float("nan")
            stats.rolling_rtf.append({
                "sim_time_s": round(stats.steady_steps * physics_dt, 6),
                "rtf": round(rtf, 6),
            })
            window_wallclock_start = now
            window_sim_start = stats.steady_steps * physics_dt

    stats.steady_wallclock_s = time.perf_counter() - steady_start
    stats.steady_simulated_s = stats.steady_steps * physics_dt
    return stats


def _rtf(sim_s: float, wall_s: float) -> float:
    return sim_s / wall_s if wall_s > 0 else float("nan")


def report(
    script_stem: str,
    config: dict[str, Any],
    timer: BenchTimer,
    runtime: RuntimeStats,
    physics_dt: float,
    rendering_dt: float,
) -> Path:
    """Print a uniform summary block and write JSON to results/<stem>.json."""
    phases = timer.as_dict()
    startup_sim_app = phases.get("startup_sim_app", float("nan"))
    startup_world_and_scene = phases.get("startup_world_and_scene", float("nan"))
    startup_total = startup_sim_app + startup_world_and_scene

    fall_rtf = _rtf(runtime.fall_simulated_s, runtime.fall_wallclock_s)
    steady_rtf = _rtf(runtime.steady_simulated_s, runtime.steady_wallclock_s)

    result: dict[str, Any] = {
        "script": script_stem,
        "config": config,
        "physics_dt": physics_dt,
        "rendering_dt": rendering_dt,
        "startup_sim_app_s": startup_sim_app,
        "startup_world_and_scene_s": startup_world_and_scene,
        "startup_total_s": startup_total,
        "fall_wallclock_s": runtime.fall_wallclock_s,
        "fall_simulated_s": runtime.fall_simulated_s,
        "fall_steps": runtime.fall_steps,
        "fall_rtf": fall_rtf,
        "steady_wallclock_s": runtime.steady_wallclock_s,
        "steady_simulated_s": runtime.steady_simulated_s,
        "steady_steps": runtime.steady_steps,
        "steady_rtf": steady_rtf,
        "rolling_rtf": runtime.rolling_rtf,
        "landed": runtime.landed,
        "timed_out": runtime.timed_out,
    }

    print("=" * 64)
    print(f"[{script_stem}] benchmark result")
    print("-" * 64)
    print(f"  config: {config}")
    print(f"  startup_sim_app_s         : {startup_sim_app:.3f}")
    print(f"  startup_world_and_scene_s : {startup_world_and_scene:.3f}")
    print(f"  startup_total_s           : {startup_total:.3f}")
    print(f"  fall_wallclock_s          : {runtime.fall_wallclock_s:.3f}")
    print(f"  fall_simulated_s          : {runtime.fall_simulated_s:.3f}")
    print(f"  fall_steps                : {runtime.fall_steps}")
    print(f"  fall_rtf                  : {fall_rtf:.3f}")
    print(f"  steady_wallclock_s        : {runtime.steady_wallclock_s:.3f}")
    print(f"  steady_simulated_s        : {runtime.steady_simulated_s:.3f}")
    print(f"  steady_steps              : {runtime.steady_steps}")
    print(f"  steady_rtf                : {steady_rtf:.3f}")
    print(f"  landed / timed_out        : {runtime.landed} / {runtime.timed_out}")
    print("=" * 64)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    out = RESULTS_DIR / f"{script_stem}.json"
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"[{script_stem}] wrote {out}")
    return out


def parse_common_args(description: str):
    """Shared argparse setup for the benchmark scripts."""
    import argparse
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--headless",
        dest="headless",
        action="store_true",
        default=True,
        help="Run Isaac Sim in headless mode (default)",
    )
    parser.add_argument(
        "--no-headless",
        dest="headless",
        action="store_false",
        help="Run Isaac Sim with a visible window (slower; for sanity checks)",
    )
    parser.add_argument(
        "--physics-hz",
        dest="physics_hz",
        type=int,
        default=None,
        metavar="HZ",
        help="Physics step rate in Hz (overrides the script's built-in default).",
    )
    parser.add_argument(
        "--rendering-hz",
        dest="rendering_hz",
        type=int,
        default=None,
        metavar="HZ",
        help="Rendering step rate in Hz (overrides the script's built-in default).",
    )
    return parser.parse_args()


def script_stem(file_path: str) -> str:
    return Path(file_path).stem


def physics_hz_stem(file_path: str, physics_hz: int) -> str:
    """Return a result-file stem with the actual physics rate substituted.

    Replaces a trailing ``_<N>hz`` token in the filename stem with the
    supplied rate so that running script 7 at 100 Hz writes
    ``7_cube_no_pegasus_100hz.json`` rather than overwriting the 250 Hz result.
    If the stem contains no such token the rate is appended.

    Example::

        physics_hz_stem("7_cube_no_pegasus_250hz.py", 100)
        # → "7_cube_no_pegasus_100hz"
    """
    import re
    base = Path(file_path).stem
    new_base, n = re.subn(r"_\d+hz$", f"_{physics_hz}hz", base)
    if n == 0:
        new_base = f"{base}_{physics_hz}hz"
    return new_base


def rendering_hz_stem(file_path: str, rendering_hz: int, physics_hz: int | None = None) -> str:
    """Return a result-file stem that encodes the rendering rate as ``_r<N>hz``.

    Optionally also substitutes the physics rate (same as physics_hz_stem) first,
    then appends ``_r<rendering_hz>hz``.

    Example::

        rendering_hz_stem("2_cube_pegasus_flat_no_px4.py", 30)
        # → "2_cube_pegasus_flat_no_px4_r30hz"

        rendering_hz_stem("7_cube_no_pegasus_250hz.py", 30, physics_hz=100)
        # → "7_cube_no_pegasus_100hz_r30hz"
    """
    import re
    base = Path(file_path).stem
    if physics_hz is not None:
        base, n = re.subn(r"_\d+hz$", f"_{physics_hz}hz", base)
        if n == 0:
            base = f"{base}_{physics_hz}hz"
    # Strip any existing _r<N>hz suffix then append the new one
    base = re.sub(r"_r\d+hz$", "", base)
    return f"{base}_r{rendering_hz}hz"
