#!/usr/bin/env python
"""Benchmark 4: cube fall with Pegasus loaded, Full Warehouse scene, drone with Python backend.

Same as benchmark 2 but uses the Full Warehouse USD instead of Flat Plane. Comparing
against benchmark 2 isolates the cost of complex scene geometry/rendering.

Pass --physics-hz to override the default Pegasus physics rate (250 Hz).

Caveats:
  - First SimulationApp launch after a reboot includes shader compile / asset cache
    warm-up; second-run numbers are more representative.
  - --no-headless adds rendering overhead; only compare headless-to-headless across scripts.

Run:
  ./python.sh benchmarking/4_cube_pegasus_complex_no_px4.py                       # 250 Hz (default)
  ./python.sh benchmarking/4_cube_pegasus_complex_no_px4.py --physics-hz 100
  ./python.sh benchmarking/4_cube_pegasus_complex_no_px4.py --physics-hz 50
  ./python.sh benchmarking/4_cube_pegasus_complex_no_px4.py --no-headless
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
    rendering_hz_stem,
    report,
    run_cube_fall_and_steady,
    script_stem,
)

DEFAULT_PHYSICS_HZ = 250

args = parse_common_args(__doc__)
physics_hz = args.physics_hz or DEFAULT_PHYSICS_HZ
rendering_hz = args.rendering_hz

timer = BenchTimer()
timer.start("startup_sim_app")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})
timer.stop("startup_sim_app")

timer.start("startup_world_and_scene")
import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "examples", "utils")))
from nonlinear_controller import NonlinearController

timeline = omni.timeline.get_timeline_interface()

pg = PegasusInterface()
pg._world_settings["physics_dt"] = 1.0 / physics_hz
if rendering_hz:
    pg._world_settings["rendering_dt"] = 1.0 / rendering_hz
pg._world = World(**pg._world_settings)
world = pg.world

pg.load_environment(SIMULATION_ENVIRONMENTS["Full Warehouse"])

multirotor_config = MultirotorConfig()
multirotor_config.backends = [NonlinearController(
    trajectory_file=None,
    results_file=None,
    Ki=[0.5, 0.5, 0.5],
    Kr=[2.0, 2.0, 2.0],
)]
Multirotor(
    "/World/quadrotor",
    ROBOTS["Iris"],
    0,
    [2.0, 2.0, 0.07],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=multirotor_config,
)

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
    script_stem=(rendering_hz_stem(__file__, rendering_hz, args.physics_hz and physics_hz)
                 if rendering_hz else
                 (script_stem(__file__) if args.physics_hz is None
                  else physics_hz_stem(__file__, physics_hz))),
    config={
        "pegasus": True,
        "scene": "Full Warehouse",
        "drone_backend": "python_nonlinear_controller",
        "headless": args.headless,
    },
    timer=timer,
    runtime=runtime,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

simulation_app.close()
