#!/usr/bin/env python
"""Benchmark 7: cube fall with pure Isaac Sim, default ground plane, variable physics Hz.

Default physics rate is 250 Hz to match Pegasus's px4 World settings, making this
directly comparable against scripts 2 and 3. Pass --physics-hz to benchmark at any
other rate (e.g. 100 or 50 Hz) without needing a separate script; the JSON result
is written with the actual rate in the filename so results never collide.

Key pairings:
  7 (250 Hz) vs 2/3  — isolates Pegasus+drone cost at matched physics_dt
  1         vs 7     — isolates the cost of the 250 Hz physics rate itself
  7 (100 Hz) vs 7 (250 Hz) — shows RTF gain from dropping physics rate

Caveats:
  - First SimulationApp launch after a reboot includes shader compile / asset cache
    warm-up; second-run numbers are more representative.
  - --no-headless adds rendering overhead; only compare headless-to-headless across scripts.

Run:
  ./python.sh benchmarking/7_cube_no_pegasus_250hz.py                        # 250 Hz (default)
  ./python.sh benchmarking/7_cube_no_pegasus_250hz.py --physics-hz 100       # 100 Hz
  ./python.sh benchmarking/7_cube_no_pegasus_250hz.py --physics-hz 50        # 50 Hz
  ./python.sh benchmarking/7_cube_no_pegasus_250hz.py --no-headless          # windowed
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.bench_timer import (
    BenchTimer,
    CUBE_SIZE,
    CUBE_SPAWN_Z,
    parse_common_args,
    physics_hz_stem,
    report,
    run_cube_fall_and_steady,
)

DEFAULT_PHYSICS_HZ = 250

args = parse_common_args(__doc__)
physics_hz = args.physics_hz or DEFAULT_PHYSICS_HZ

timer = BenchTimer()
timer.start("startup_sim_app")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})
timer.stop("startup_sim_app")

timer.start("startup_world_and_scene")
import numpy as np
import omni.timeline
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

timeline = omni.timeline.get_timeline_interface()
world = World(
    physics_dt=1.0 / physics_hz,
    rendering_dt=1.0 / 60.0,
    stage_units_in_meters=1.0,
    device="cpu",
)
world.scene.add_default_ground_plane()
cube = world.scene.add(DynamicCuboid(
    prim_path="/World/benchmark_cube",
    name="benchmark_cube",
    position=np.array([0.0, 0.0, CUBE_SPAWN_Z]),
    size=CUBE_SIZE,
    color=np.array([1.0, 0.0, 0.0]),
))
world.reset()
timer.stop("startup_world_and_scene")

physics_dt = world.get_physics_dt()
rendering_dt = world.get_rendering_dt()

timeline.play()
runtime = run_cube_fall_and_steady(
    world=world,
    cube=cube,
    physics_dt=physics_dt,
    render=True,
    is_running=simulation_app.is_running,
)
timeline.stop()

report(
    script_stem=physics_hz_stem(__file__, physics_hz),
    config={
        "pegasus": False,
        "scene": "default_ground_plane",
        "drone_backend": None,
        "headless": args.headless,
        "physics_dt_matches_pegasus": physics_hz == DEFAULT_PHYSICS_HZ,
    },
    timer=timer,
    runtime=runtime,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

simulation_app.close()
