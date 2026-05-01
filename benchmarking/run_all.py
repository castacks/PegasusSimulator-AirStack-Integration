#!/usr/bin/env python3
"""Orchestrator: run all cube-fall benchmarks and plot the results.

Runs each benchmarking/<N>_*.py script as a subprocess via Isaac Sim's python.sh
(sequentially to avoid GPU/CPU contention), collects their per-script JSON
outputs, and produces:

  benchmarking/results/summary.json   - aggregated metrics
  benchmarking/results/summary.png    - 2x2 comparison figure

Script numbering:
   1–8   original runs (each script's natural default rate)
   9–12  scripts 7 & 8 (no-Pegasus explicit-rate) at 100 Hz and 50 Hz
  13–16  scripts 1 & 6 (no-Pegasus)               at 100 Hz and 50 Hz
  17–20  scripts 2 & 4 (Pegasus + Python)          at 100 Hz and 50 Hz
  21–24  scripts 3 & 5 (Pegasus + PX4)             at 100 Hz and 50 Hz
            IMU_INTEG_RATE is set automatically via PX4_IMU_INTEG_RATE env var
  25–28  scripts 2–5 (Pegasus) at 30 Hz rendering  (physics stays at default 250 Hz)
  29–32  scripts 2–5 (Pegasus) at 60 Hz rendering  (physics stays at default 250 Hz)

Invocation:
  python3 benchmarking/run_all.py                        # run all and plot
  python3 benchmarking/run_all.py --scripts 1,3,5        # run a subset
  python3 benchmarking/run_all.py --scripts 21-24        # run a range
  python3 benchmarking/run_all.py --skip-run             # re-plot existing JSONs
  python3 benchmarking/run_all.py --output path.png      # custom output

Environment:
  ISAACSIM_PYTHON is used to invoke each benchmark (set in .bashrc / .zshrc).
  Falls back to $ISAACSIM_PATH/python.sh or ~/isaacsim/python.sh.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

BENCH_DIR = Path(__file__).resolve().parent
RESULTS_DIR = BENCH_DIR / "results"

# Each entry is either:
#   int -> "filename.py"                          (no extra args, stem = filename stem)
#   int -> ("filename.py", ["--physics-hz", "N"]) (extra args passed to the script;
#                                                   physics_hz_stem() derives the JSON stem)
#
# Numbering scheme
#   1–8   : original runs at each script's natural default rate
#   9–12  : scripts 7 & 8 (explicit-rate baselines) at 100 Hz and 50 Hz
#   13–16 : scripts 1 & 6 (no-Pegasus) at 100 Hz and 50 Hz
#   17–20 : scripts 2 & 4 (Pegasus + Python) at 100 Hz and 50 Hz
#   21–24 : scripts 3 & 5 (Pegasus + PX4)   at 100 Hz and 50 Hz
#             → IMU_INTEG_RATE is set automatically via PX4_IMU_INTEG_RATE env var
#   25–28 : scripts 2–5 (Pegasus) at 30 Hz rendering  (physics = default 250 Hz)
#   29–32 : scripts 2–5 (Pegasus) at 60 Hz rendering  (physics = default 250 Hz)
SCRIPTS: dict = {
    1: "1_cube_no_pegasus.py",
    2: "2_cube_pegasus_flat_no_px4.py",
    3: "3_cube_pegasus_flat_px4.py",
    4: "4_cube_pegasus_complex_no_px4.py",
    5: "5_cube_pegasus_complex_px4.py",
    6: "6_cube_no_pegasus_complex.py",
    7: "7_cube_no_pegasus_250hz.py",
    8: "8_cube_no_pegasus_complex_250hz.py",
    # scripts 7 & 8 at lower rates
    9:  ("7_cube_no_pegasus_250hz.py",         ["--physics-hz", "100"]),
    10: ("8_cube_no_pegasus_complex_250hz.py", ["--physics-hz", "100"]),
    11: ("7_cube_no_pegasus_250hz.py",         ["--physics-hz", "50"]),
    12: ("8_cube_no_pegasus_complex_250hz.py", ["--physics-hz", "50"]),
    # no-Pegasus baselines at 100 Hz and 50 Hz
    13: ("1_cube_no_pegasus.py",               ["--physics-hz", "100"]),
    14: ("6_cube_no_pegasus_complex.py",       ["--physics-hz", "100"]),
    15: ("1_cube_no_pegasus.py",               ["--physics-hz", "50"]),
    16: ("6_cube_no_pegasus_complex.py",       ["--physics-hz", "50"]),
    # Pegasus + Python controller at 100 Hz and 50 Hz
    17: ("2_cube_pegasus_flat_no_px4.py",      ["--physics-hz", "100"]),
    18: ("4_cube_pegasus_complex_no_px4.py",   ["--physics-hz", "100"]),
    19: ("2_cube_pegasus_flat_no_px4.py",      ["--physics-hz", "50"]),
    20: ("4_cube_pegasus_complex_no_px4.py",   ["--physics-hz", "50"]),
    # Pegasus + PX4 at 100 Hz and 50 Hz (IMU_INTEG_RATE matches physics rate)
    21: ("3_cube_pegasus_flat_px4.py",         ["--physics-hz", "100"]),
    22: ("5_cube_pegasus_complex_px4.py",      ["--physics-hz", "100"]),
    23: ("3_cube_pegasus_flat_px4.py",         ["--physics-hz", "50"]),
    24: ("5_cube_pegasus_complex_px4.py",      ["--physics-hz", "50"]),
    # Rendering rate sweep: 30 Hz (physics stays at default 250 Hz)
    25: ("2_cube_pegasus_flat_no_px4.py",      ["--rendering-hz", "30"]),
    26: ("3_cube_pegasus_flat_px4.py",         ["--rendering-hz", "30"]),
    27: ("4_cube_pegasus_complex_no_px4.py",   ["--rendering-hz", "30"]),
    28: ("5_cube_pegasus_complex_px4.py",      ["--rendering-hz", "30"]),
    # Rendering rate sweep: 60 Hz (physics stays at default 250 Hz)
    29: ("2_cube_pegasus_flat_no_px4.py",      ["--rendering-hz", "60"]),
    30: ("3_cube_pegasus_flat_px4.py",         ["--rendering-hz", "60"]),
    31: ("4_cube_pegasus_complex_no_px4.py",   ["--rendering-hz", "60"]),
    32: ("5_cube_pegasus_complex_px4.py",      ["--rendering-hz", "60"]),
}


def _find_isaac_python() -> str:
    env = os.environ.get("ISAACSIM_PYTHON")
    if env and Path(env).is_file():
        return env
    base = os.environ.get("ISAACSIM_PATH") or str(Path.home() / "isaacsim")
    candidate = Path(base) / "python.sh"
    if candidate.is_file():
        return str(candidate)
    raise FileNotFoundError(
        "Could not locate Isaac Sim's python.sh. Set ISAACSIM_PYTHON or ISAACSIM_PATH."
    )


def _script_file_and_args(script_num: int) -> tuple[str, list[str]]:
    """Return (filename, extra_cli_args) for a script entry."""
    entry = SCRIPTS[script_num]
    if isinstance(entry, tuple):
        return entry[0], list(entry[1])
    return entry, []


def _result_stem(script_num: int) -> str:
    """Return the JSON result stem for a script entry.

    Mirrors the logic in bench_timer.physics_hz_stem / rendering_hz_stem so the
    stem matches what the script itself will write to results/.
    """
    import re
    filename, extra = _script_file_and_args(script_num)
    base = Path(filename).stem

    physics_hz: int | None = None
    rendering_hz: int | None = None
    for i, arg in enumerate(extra):
        if arg == "--physics-hz" and i + 1 < len(extra):
            physics_hz = int(extra[i + 1])
        elif arg == "--rendering-hz" and i + 1 < len(extra):
            rendering_hz = int(extra[i + 1])

    if physics_hz is not None:
        base, n = re.subn(r"_\d+hz$", f"_{physics_hz}hz", base)
        if n == 0:
            base = f"{base}_{physics_hz}hz"
    if rendering_hz is not None:
        base = re.sub(r"_r\d+hz$", "", base)
        base = f"{base}_r{rendering_hz}hz"

    return base


def _run_one(script_num: int, isaac_python: str, extra_args: list[str]) -> bool:
    filename, script_extra = _script_file_and_args(script_num)
    script = BENCH_DIR / filename
    cli = [isaac_python, str(script), *script_extra, *extra_args]
    label = f"{filename}" + (f" {' '.join(script_extra)}" if script_extra else "")
    print(f"\n[run_all] >>> running {label}", flush=True)
    t0 = time.perf_counter()
    proc = subprocess.run(cli)
    elapsed = time.perf_counter() - t0
    if proc.returncode != 0:
        print(f"[run_all] !!! {label} exited {proc.returncode} after {elapsed:.1f}s", flush=True)
        return False
    print(f"[run_all] <<< {label} ok ({elapsed:.1f}s wall)", flush=True)
    return True


def _load_results(script_nums: list[int]) -> dict[int, dict]:
    results: dict[int, dict] = {}
    for n in script_nums:
        stem = _result_stem(n)
        path = RESULTS_DIR / f"{stem}.json"
        if not path.is_file():
            print(f"[run_all] warn: missing {path}")
            continue
        with open(path) as f:
            results[n] = json.load(f)
    return results


def _print_summary_table(results: dict[int, dict]) -> None:
    print("\n" + "=" * 96)
    print(f"{'#':<3} {'script':<40} {'startup_total_s':>15} {'fall_rtf':>10} {'steady_rtf':>12}")
    print("-" * 96)
    for n in sorted(results.keys()):
        r = results[n]
        print(
            f"{n:<3} {r['script']:<40} "
            f"{r['startup_total_s']:>15.3f} "
            f"{r['fall_rtf']:>10.3f} "
            f"{r['steady_rtf']:>12.3f}"
        )
    print("=" * 96)


def _plot(results: dict[int, dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    nums = sorted(results.keys())

    def _color(n: int) -> str:
        cfg = results[n]["config"]
        if not cfg.get("pegasus"):
            return "#1f77b4"  # blue: no Pegasus baseline
        if cfg.get("drone_backend") == "px4_mavlink":
            return "#d62728"  # red: PX4
        return "#2ca02c"      # green: Pegasus, no PX4

    def _label(n: int) -> str:
        cfg = results[n]["config"]
        scene = cfg["scene"]
        scene_short = {"default_ground_plane": "flat/default", "Flat Plane": "flat plane"}.get(scene, scene)
        hz = round(1.0 / results[n]["physics_dt"])
        peg = "Pegasus" if cfg.get("pegasus") else "no Pegasus"
        backend = cfg.get("drone_backend") or ""
        drone = {"python_nonlinear_controller": "Python drone", "px4_mavlink": "PX4 drone"}.get(backend, "no drone")
        return f"#{n}: {peg}\n{scene_short} · {hz} Hz\n{drone}"

    def _legend_label(n: int) -> str:
        cfg = results[n]["config"]
        scene = cfg["scene"]
        scene_short = {"default_ground_plane": "flat", "Flat Plane": "flat plane"}.get(scene, scene)
        hz = round(1.0 / results[n]["physics_dt"])
        backend = cfg.get("drone_backend") or ""
        drone = {"python_nonlinear_controller": "Python", "px4_mavlink": "PX4"}.get(backend, "–")
        return f"#{n} {scene_short} {hz}Hz {drone}"

    colors = [_color(n) for n in nums]
    labels = [_label(n) for n in nums]
    x = np.arange(len(nums))
    bar_width = min(0.65, 5.0 / len(nums))  # narrow bars when many scripts

    fig, axes = plt.subplots(2, 2, figsize=(max(18, len(nums) * 2), 14))
    fig.suptitle("Isaac Sim + Pegasus cube-fall benchmark", fontsize=15, fontweight="bold")

    def _set_xticks(ax, rotate=45):
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=rotate, ha="right", fontsize=8.5,
                           multialignment="center")
        ax.tick_params(axis="x", pad=4)

    # Top-left: startup time breakdown (stacked bars)
    ax = axes[0, 0]
    sim_app_vals = [results[n]["startup_sim_app_s"] for n in nums]
    world_vals = [results[n]["startup_world_and_scene_s"] for n in nums]
    ax.bar(x, sim_app_vals, bar_width, color="#7f7f7f", label="SimulationApp init")
    ax.bar(x, world_vals, bar_width, bottom=sim_app_vals, color="#bcbd22", label="World + scene + reset")
    for i, n in enumerate(nums):
        total = sim_app_vals[i] + world_vals[i]
        ax.text(i, total + 0.3, f"{total:.1f}s", ha="center", va="bottom", fontsize=8, fontweight="bold")
    _set_xticks(ax)
    ax.set_ylabel("seconds")
    ax.set_title("Startup time", fontsize=11)
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.margins(x=0.05)

    # Top-right: steady-state RTF
    ax = axes[0, 1]
    steady = [results[n]["steady_rtf"] for n in nums]
    bars = ax.bar(x, steady, bar_width, color=colors)
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1.2, alpha=0.7, label="real-time = 1.0")
    for bar, v in zip(bars, steady):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.01, f"{v:.2f}",
                ha="center", va="bottom", fontsize=8, fontweight="bold")
    _set_xticks(ax)
    ax.set_ylabel("RTF  (simulated s / wall s)", fontsize=9)
    ax.set_title("Steady-state real-time factor  ◀ headline metric", fontsize=11)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.margins(x=0.05)

    # Bottom-left: fall RTF vs steady RTF side-by-side
    ax = axes[1, 0]
    fall = [results[n]["fall_rtf"] for n in nums]
    w = bar_width * 0.46
    ax.bar(x - w * 0.55, fall, w * 1.05, color="#ff7f0e", label="fall (transient)")
    ax.bar(x + w * 0.55, steady, w * 1.05, color=colors, label="steady-state")
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1.2, alpha=0.7)
    for i, (f, s) in enumerate(zip(fall, steady)):
        ax.text(i - w * 0.55, f + 0.01, f"{f:.2f}", ha="center", va="bottom", fontsize=7)
        ax.text(i + w * 0.55, s + 0.01, f"{s:.2f}", ha="center", va="bottom", fontsize=7)
    _set_xticks(ax)
    ax.set_ylabel("RTF", fontsize=9)
    ax.set_title("Fall RTF vs steady-state RTF", fontsize=11)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.margins(x=0.05)

    # Bottom-right: rolling RTF jitter (line plot, legend outside)
    ax = axes[1, 1]
    for n in nums:
        series = results[n].get("rolling_rtf", [])
        if not series:
            continue
        xs = [s["sim_time_s"] for s in series]
        ys = [s["rtf"] for s in series]
        ax.plot(xs, ys, marker="o", markersize=4, linewidth=1.5,
                color=_color(n), label=_legend_label(n))
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1.2, alpha=0.7, label="real-time = 1.0")
    ax.set_xlabel("simulated time since landing (s)", fontsize=9)
    ax.set_ylabel("rolling RTF (1 s windows)", fontsize=9)
    ax.set_title("Steady-state RTF jitter", fontsize=11)
    ax.legend(loc="upper left", fontsize=7.5, framealpha=0.85,
              bbox_to_anchor=(1.01, 1), borderaxespad=0)
    ax.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output, dpi=120, bbox_inches="tight")
    print(f"[run_all] wrote {output}")
    print(f"[run_all] wrote {output}")


def _parse_scripts(arg: str | None) -> list[int]:
    if not arg:
        return sorted(SCRIPTS.keys())
    nums: list[int] = []
    for tok in arg.split(","):
        tok = tok.strip()
        if not tok:
            continue
        # Support ranges like "7-12"
        if "-" in tok:
            lo, hi = tok.split("-", 1)
            for n in range(int(lo), int(hi) + 1):
                if n not in SCRIPTS:
                    raise SystemExit(f"Unknown script number: {n}. Valid: {sorted(SCRIPTS)}")
                nums.append(n)
        else:
            n = int(tok)
            if n not in SCRIPTS:
                raise SystemExit(f"Unknown script number: {n}. Valid: {sorted(SCRIPTS)}")
            nums.append(n)
    return nums


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scripts", default=None, help="Comma-separated script numbers (default: all)")
    parser.add_argument("--skip-run", action="store_true", help="Skip execution, just re-plot existing JSONs")
    parser.add_argument("--output", default=str(RESULTS_DIR / "summary.png"), help="Output PNG path")
    parser.add_argument("--no-headless", dest="headless", action="store_false", default=True,
                        help="Pass --no-headless through to each benchmark (default: headless)")
    args = parser.parse_args()

    nums = _parse_scripts(args.scripts)

    if not args.skip_run:
        isaac_python = _find_isaac_python()
        print(f"[run_all] using {isaac_python}")
        extra = [] if args.headless else ["--no-headless"]
        failures: list[int] = []
        for i, n in enumerate(nums):
            ok = _run_one(n, isaac_python, extra)
            if not ok:
                failures.append(n)
            if i < len(nums) - 1:
                time.sleep(2)
        if failures:
            print(f"[run_all] warning: {len(failures)}/{len(nums)} script(s) failed: {failures}")

    results = _load_results(nums)
    if not results:
        print("[run_all] no results to plot")
        return 1

    _print_summary_table(results)

    summary_json = RESULTS_DIR / "summary.json"
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    with open(summary_json, "w") as f:
        json.dump({str(k): v for k, v in results.items()}, f, indent=2)
    print(f"[run_all] wrote {summary_json}")

    _plot(results, Path(args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
