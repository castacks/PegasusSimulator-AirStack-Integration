#!/usr/bin/env python
"""Benchmark 1: cube fall with pure Isaac Sim (Pegasus extension NOT imported).

Baseline that measures raw Isaac Sim physics + rendering cost at the natural Isaac Sim
default rate (60 Hz). Pass --physics-hz to benchmark at any other rate; the JSON result
is written with the actual rate appended so results never collide.

Caveats:
  - First SimulationApp launch after a reboot includes shader compile / asset cache
    warm-up; second-run numbers are more representative.
  - --no-headless adds rendering overhead; only compare headless-to-headless across scripts.

Run:
  ./python.sh benchmarking/1_cube_no_pegasus.py                       # 60 Hz (default)
  ./python.sh benchmarking/1_cube_no_pegasus.py --physics-hz 100
  ./python.sh benchmarking/1_cube_no_pegasus.py --physics-hz 50
  ./python.sh benchmarking/1_cube_no_pegasus.py --no-headless
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
    script_stem,
)

DEFAULT_PHYSICS_HZ = 60

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
    script_stem=(script_stem(__file__) if args.physics_hz is None
                 else physics_hz_stem(__file__, physics_hz)),
    config={
        "pegasus": False,
        "scene": "default_ground_plane",
        "drone_backend": None,
        "headless": args.headless,
    },
    timer=timer,
    runtime=runtime,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

simulation_app.close()
