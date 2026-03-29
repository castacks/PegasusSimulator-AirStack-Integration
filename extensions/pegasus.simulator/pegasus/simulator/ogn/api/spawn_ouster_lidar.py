"""
.. deprecated::
    This module uses the legacy Ouster USD reference + JSON config patching
    approach for lidar near-range overrides.  It is superseded by
    :mod:`pegasus.simulator.ogn.api.spawn_rtx_lidar`, which uses Isaac Sim
    5.0+ OmniLidar prims with ``omni:sensor:Core:nearRangeM`` set directly
    on the sensor prim.

    Kept for backward compatibility with saved USD files that carry the
    ``pegasus:lidarMinRange`` custom attribute.
"""

import json
import os
import re

import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim
from pxr import UsdGeom, Gf, UsdPhysics, Sdf
from omni.physx.scripts import utils as physx_utils
import omni
import carb
import carb.settings

# ─── S3 base URL for Ouster lidar USD assets ──────────────────────────────
_OUSTER_S3_BASE = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Ouster"
)

# ─── Available lidar options ──────────────────────────────────────────────
# Each key is a *lidar USD asset name* (filename without ".usd").  This is
# also used as the ``lidar_name`` prim name in the Isaac Sim stage.
#
# Each value is a dict with:
#   "usd"    – full S3 URL to the USD asset
#   "config" – *relative* path to the RTX lidar JSON config under the
#              profile search directories (Carbonite setting
#              ``app.sensors.nv.lidar.profileBaseFolder``)
#
# The JSON config controls ``nearRangeM``, ``maxRangeM``, beam layout, etc.
#
# ── Ouster family ─────────────────────────────────────────────────────────
# Subfamilies:  OS0  (short-range, wide FOV)
#               OS1  (mid-range, general purpose)  ← default
#               OS2  (long-range, narrow FOV)
# Channels:     32, 128
# Scan rates:   10 Hz, 20 Hz
# Resolutions:  512, 1024, 2048
# Revisions:    REV6, REV7
#
# Only the default is listed explicitly.  Other Ouster models are derived
# automatically by ``_resolve_lidar_config()`` from the USD name using the
# pattern:  OS{X}_REV{Y}_{C}_{R}hz___{RES}_resolution
#
# Examples of other valid lidar names (pass as ``lidar_name``):
#   "OS0_REV7_128_10hz___512_resolution"   – OS0 short-range
#   "OS1_REV7_128_20hz___1024_resolution"  – OS1 REV7 high-rate
#   "OS2_REV6_128_10hz___2048_resolution"  – OS2 long-range hi-res
#   "OS1_REV6_32_10hz___512_resolution"    – OS1 32-beam economy
# ──────────────────────────────────────────────────────────────────────────
LIDAR_ASSETS: dict[str, dict[str, str]] = {
    # ── Default: Ouster OS1 REV6, 128-beam, 10 Hz, 512 azimuth resolution ──
    "OS1_REV6_128_10hz___512_resolution": {
        "usd": f"{_OUSTER_S3_BASE}/OS1/OS1_REV6_128_10hz___512_resolution.usd",
        "config": "Ouster/OS1/OS1_REV6_128ch10hz512res.json",
    },
}

# Default USD URL (backward-compatible constant used by callers)
OUSTER_LIDAR_USD_URL = LIDAR_ASSETS["OS1_REV6_128_10hz___512_resolution"]["usd"]


def _resolve_lidar_config(lidar_name: str) -> str | None:
    """Return the *relative* JSON config path for a lidar USD asset name.

    1. Exact lookup in :data:`LIDAR_ASSETS`.
    2. Regex derivation for any Ouster sensor whose USD name follows
       ``OS{X}_REV{Y}_{C}_{R}hz___{RES}_resolution``.
    3. ``None`` if the name cannot be resolved.
    """
    # Exact lookup
    if lidar_name in LIDAR_ASSETS:
        return LIDAR_ASSETS[lidar_name]["config"]

    # Derive for Ouster sensors
    m = re.match(
        r"(OS\d)_REV(\d+)_(\d+)_(\d+)hz___(\d+)_resolution",
        lidar_name,
    )
    if m:
        family, rev, ch, rate, res = m.groups()
        return f"Ouster/{family}/{family}_REV{rev}_{ch}ch{rate}hz{res}res.json"

    return None


def _override_lidar_near_range(lidar_name: str, min_range: float) -> None:
    """Override ``nearRangeM`` in the JSON config for a *specific* lidar model.

    The RTX lidar renderer reads ``nearRangeM`` from a JSON config file whose
    path is resolved via the Carbonite setting
    ``app.sensors.nv.lidar.profileBaseFolder``.  This function locates the
    specific JSON for *lidar_name* and patches only that file, leaving every
    other lidar configuration untouched.

    Args:
        lidar_name: USD asset name (e.g. ``"OS1_REV6_128_10hz___512_resolution"``).
        min_range:  Desired minimum detection range in metres.
    """
    config_rel = _resolve_lidar_config(lidar_name)
    if config_rel is None:
        print(
            f"[AirStack] WARNING: Cannot determine JSON config for lidar "
            f"'{lidar_name}'; nearRangeM not patched.  Add an entry to "
            f"LIDAR_ASSETS in spawn_ouster_lidar.py."
        )
        return

    settings = carb.settings.get_settings()
    search_dirs_raw = settings.get("app/sensors/nv/lidar/profileBaseFolder") or []

    # Carbonite tokens like "${isaacsim.sensors.rtx}" must be resolved first.
    tokens = carb.tokens.get_tokens_interface()

    patched = 0
    found = False
    for raw_dir in search_dirs_raw:
        resolved = tokens.resolve(raw_dir)
        json_path = os.path.join(resolved, config_rel)
        if not os.path.isfile(json_path):
            continue
        found = True
        try:
            with open(json_path, "r") as f:
                config_data = json.load(f)
            if "profile" in config_data and "nearRangeM" in config_data["profile"]:
                old_val = config_data["profile"]["nearRangeM"]
                if old_val == min_range:
                    print(f"[AirStack] nearRangeM already {min_range} in {json_path}")
                    continue
                config_data["profile"]["nearRangeM"] = min_range
                with open(json_path, "w") as f:
                    json.dump(config_data, f, indent=4)
                patched += 1
                print(f"[AirStack] Patched nearRangeM {old_val} → {min_range} in {json_path}")
        except Exception as exc:
            print(f"[AirStack] WARNING: could not patch {json_path}: {exc}")

    if patched:
        print(f"[AirStack] Done — patched {patched} config file(s) for '{lidar_name}'.")
    elif not found:
        resolved_dirs = [tokens.resolve(d) for d in search_dirs_raw]
        print(
            f"[AirStack] WARNING: Config '{config_rel}' not found in any search "
            f"directory.  Searched: {resolved_dirs}"
        )


def attach_lidar_to_drone(
    drone_prim_path: str,
    lidar_name: str,
    lidar_usd: str,
    lidar_offset: list[float],
    lidar_rotation_offset: list[float],
    frame_id: str,
    min_range: float = 0.0,
) -> str:
    """
    Attach a LiDAR USD to a drone prim (ZED-style), converting RPY -> quaternion and
    applying a corrective rotation to align with USD Z-forward convention.

    Args:
        drone_prim_path: Path to the drone prim (e.g. "/World/Drone_01").
        lidar_name: Name for the LiDAR prim under the drone.
        lidar_usd: URL or path to the LiDAR USD asset.
        lidar_offset: [x, y, z] translation offset relative to the drone.
        lidar_rotation_rpy: [roll, pitch, yaw] in degrees.
        frame_id: Name for the internal sensor prim (renamed from "sensor").

    Returns:
        The path to the renamed sensor prim (str) or None on failure.
    """

    stage = omni.usd.get_context().get_stage()
    lidar_prim_path = f"{drone_prim_path}/{lidar_name}"

    # Override nearRangeM in the lidar's JSON config BEFORE loading the USD so
    # the renderer picks up the new value when it first initialises the sensor.
    if min_range > 0.0:
        _override_lidar_near_range(lidar_name, min_range)

    # Remove existing LiDAR prim
    existing = stage.GetPrimAtPath(lidar_prim_path)
    if existing.IsValid():
        carb.log_info(f"Deleting existing LiDAR prim at {lidar_prim_path}")
        omni.kit.commands.execute("DeletePrim", path=lidar_prim_path)

    # Create new LiDAR prim and reference USD
    carb.log_info(f"Creating LiDAR prim '{lidar_prim_path}' referencing '{lidar_usd}'")
    lidar_prim = define_prim(lidar_prim_path, "Xform")
    lidar_prim.GetReferences().AddReference(lidar_usd)

    # Compute rotation
    roll_deg, pitch_deg, yaw_deg = lidar_rotation_offset
    roll_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll_deg)
    pitch_rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch_deg)
    yaw_rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg)
    combined_rot = yaw_rot * pitch_rot * roll_rot
    user_quat = combined_rot.GetQuat()

    # Corrective rotation to align LiDAR’s local axes (Z-forward)
    corrective_quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()

    user_rot = Gf.Quatf(user_quat.GetReal(), *user_quat.GetImaginary())
    corrective_rot = Gf.Quatf(corrective_quat.GetReal(), *corrective_quat.GetImaginary())
    final_rot = user_rot * corrective_rot

    # Apply translation and orientation
    xform = UsdGeom.Xformable(lidar_prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    orient_op = xform.AddOrientOp()
    translate_op.Set(Gf.Vec3d(*lidar_offset))
    orient_op.Set(final_rot)
    xform.SetXformOpOrder([translate_op, orient_op])

    # Rename internal sensor prim (done via copy + delete to preserve references)
    sensor_old_path = f"{lidar_prim_path}/sensor"
    sensor_new_path = f"{lidar_prim_path}/{frame_id}"
    sensor_old = stage.GetPrimAtPath(sensor_old_path)


    if sensor_old.IsValid():
        carb.log_info(f"Renaming internal LiDAR sensor '{sensor_old_path}' → '{sensor_new_path}'")
        omni.kit.commands.execute(
            "CopyPrim",
            path_from=sensor_old_path,
            path_to=sensor_new_path,
            duplicate_layers=True,
        )
        sensor_old.SetActive(False)  # Deactivate old sensor to prevent issues during copy
        stage.GetPrimAtPath(sensor_new_path).SetActive(True)  # Activate new sensor
        omni.kit.commands.execute("DeletePrim", path=sensor_old_path)
    else:
        carb.log_warn(f"No '/sensor' child found under '{lidar_prim_path}'.")

    # Apply RigidBody API and remove collider
    try:
        if not UsdPhysics.RigidBodyAPI(lidar_prim):
            UsdPhysics.RigidBodyAPI.Apply(lidar_prim)
    except Exception:
        carb.log_warn(f"RigidBodyAPI could not be applied to '{lidar_prim_path}'.")

    try:
        physx_utils.removeCollider(lidar_prim)
    except Exception:
        carb.log_warn(f"Failed to remove collider from '{lidar_prim_path}'.")

    # Fixed joint to drone body
    drone_body_path = f"{drone_prim_path}/body/body"
    drone_body_prim = stage.GetPrimAtPath(drone_body_path)
    if not drone_body_prim.IsValid():
        carb.log_error(f"Drone body prim '{drone_body_path}' not found; cannot attach LiDAR.")
        return None

    joint = physx_utils.createJoint(
        stage,
        joint_type="Fixed",
        from_prim=drone_body_prim,
        to_prim=lidar_prim,
    )

    if not joint or not joint.IsValid():
        carb.log_error(f"Failed to create fixed joint for LiDAR '{lidar_prim_path}'.")
        return None

    # Persist the desired near-range as a USD custom attribute on the lidar prim.
    # Pegasus extension will scan for this attribute on timeline PLAY.
    if min_range > 0.0:
        lidar_prim = stage.GetPrimAtPath(lidar_prim_path)
        if lidar_prim.IsValid():
            attr = lidar_prim.CreateAttribute(
                "pegasus:lidarMinRange", Sdf.ValueTypeNames.Float, custom=True
            )
            attr.Set(float(min_range))
            carb.log_info(
                f"Stored pegasus:lidarMinRange={min_range} on '{lidar_prim_path}'"
            )

    carb.log_info(f"LiDAR '{lidar_name}' successfully attached to '{drone_prim_path}' as '{sensor_new_path}'.")
    return sensor_new_path


def apply_lidar_overrides_from_stage() -> int:
    """Scan the current USD stage for lidar prims carrying an
    ``pegasus:lidarMinRange`` custom attribute and re-apply the
    corresponding JSON config override for each one.

    This is designed to be called from the extension's timeline-PLAY
    handler so that near-range overrides stored in a saved USD are
    re-applied every time the simulation starts — even when the
    standalone launch script has not been run.

    Returns:
        Number of overrides applied.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return 0

    applied = 0
    for prim in stage.Traverse():
        attr = prim.GetAttribute("pegasus:lidarMinRange")
        if not attr or not attr.IsValid():
            continue
        min_range = attr.Get()
        if min_range is None or min_range <= 0.0:
            continue
        # The prim name equals the lidar_name used in LIDAR_ASSETS /
        # _resolve_lidar_config (e.g. "OS1_REV6_128_10hz___512_resolution").
        lidar_name = prim.GetName()
        _override_lidar_near_range(lidar_name, float(min_range))
        applied += 1

    if applied:
        print(f"[AirStack] Applied {applied} lidar near-range override(s) from USD attributes.")
    return applied


def add_ouster_lidar_subgraph(
    parent_graph_handle: og._omni_graph_core.Graph,
    drone_prim: str,
    lidar_name: str = "OS1_REV6_128_10hz___512_resolution",
    lidar_topic_name: str = "point_cloud",
    lidar_usd: str = OUSTER_LIDAR_USD_URL,
    lidar_offset: list[float] = [0.0, 0.0, 0.025],
    lidar_rotation_offset: list[float] = [0.0, 0.0, 0.0],
    lidar_topic_namespace: str = "sensors/lidar",
    lidar_frame_id: str = "lidar",
    frame_height: int = 720,
    frame_width: int = 1280,
    robot_name: str = "robot_1",
    ros2_context_node: str | None = None,
    lidar_min_range: float = 0.0,
):
    """
    Adds a lidar and builds a minimal ROS2 OmniGraph subgraph that publishes the LiDAR's point cloud.

    The graph includes:
        - OnPlaybackTick trigger
        - LiDAR render product creation
        - RTX LiDAR ROS2 helper
        - Frame and namespace constant nodes

    The ROS2 context handle is received via a promoted ``inputs:context``
    attribute on the compound node, connected to the parent graph's
    ROS2Context node.  This avoids creating a separate ROS2Context
    inside the subgraph (which fails to initialise inside compound
    nodes) and avoids absolute-path references that break when the
    scene is saved and reloaded in a different location.

    Args:
        parent_graph_handle (og.Graph): The parent OmniGraph handle.
        drone_prim (str): Path to the drone prim.
        lidar_name (str): Name of the LiDAR prim.
        lidar_topic_name (str): ROS topic name (e.g. "point_cloud").
        lidar_usd (str): Path or URL to LiDAR USD asset.
        lidar_offset (list[float]): [x, y, z] offset relative to drone.
        lidar_rotation_offset (list[float]): [roll, pitch, yaw] in degrees.
        lidar_topic_namespace (str): ROS topic namespace.
        lidar_frame_id (str): Frame ID for ROS messages.
        frame_height (int): Render product height.
        frame_width (int): Render product width.
        robot_name (str): Robot name prefix for ROS topic namespace.
        ros2_context_node (str): Name of the ROS2Context node in the
            parent graph whose ``outputs:context`` will be wired into
            this subgraph's promoted ``inputs:context``.
        lidar_min_range (float): Minimum detection range in metres. Points closer
            than this are discarded by the RTX lidar renderer. Set > 0 to prevent
            detecting propellers or other close-range drone geometry. Default 0.0
            uses the value from the sensor's JSON config (typically 0.3 m).

    Returns:
        None
    """
    if ros2_context_node is None:
        ros2_context_node = f"{robot_name}_ROS2Context"

    controller = og.Controller()

    lidar_sensor_prim = attach_lidar_to_drone(
        drone_prim_path=drone_prim,
        lidar_name=lidar_name,
        lidar_usd=lidar_usd,
        lidar_offset=lidar_offset,
        lidar_rotation_offset=lidar_rotation_offset,
        frame_id=lidar_frame_id,
        min_range=lidar_min_range,
    )

    if lidar_sensor_prim is None:
        carb.log_error("LiDAR attachment failed; aborting subgraph creation.")
        return

    parent_graph_path = parent_graph_handle.get_path_to_graph()
    lidar_subgraph_name = f"{lidar_name}Graph"

    playback_tick = f"{lidar_name}OnPlaybackTick"
    create_render = f"{lidar_name}CreateRenderProduct"
    rtx_helper = f"{lidar_name}ROS2RtxLidarHelper"
    frame_const = f"{lidar_name}FrameIdConst"
    ns_const = f"{lidar_name}NamespaceConst"

    # ── Step 1: create the compound subgraph with a promoted context input ──
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (
                    lidar_subgraph_name,
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            (playback_tick, "omni.graph.action.OnPlaybackTick"),
                            (create_render, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            (rtx_helper, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                            (frame_const, "omni.graph.nodes.ConstantString"),
                            (ns_const, "omni.graph.nodes.ConstantString"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            (("inputs:value", frame_const), lidar_frame_id),
                            (("inputs:value", ns_const), f"{robot_name}/{lidar_topic_namespace}"),
                            (("inputs:cameraPrim", create_render), lidar_sensor_prim),
                            (("inputs:height", create_render), frame_height),
                            (("inputs:width", create_render), frame_width),
                            (("inputs:topicName", rtx_helper), lidar_topic_name),
                            (("inputs:type", rtx_helper), "point_cloud"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            (f"{playback_tick}.outputs:tick", f"{create_render}.inputs:execIn"),
                            (f"{create_render}.outputs:execOut", f"{rtx_helper}.inputs:execIn"),
                            (f"{create_render}.outputs:renderProductPath", f"{rtx_helper}.inputs:renderProductPath"),
                            (f"{frame_const}.inputs:value", f"{rtx_helper}.inputs:frameId"),
                            (f"{ns_const}.inputs:value", f"{rtx_helper}.inputs:nodeNamespace"),
                        ],
                        # Promote the helper's context input directly
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            (f"{rtx_helper}.inputs:context", "inputs:context"),
                        ],
                    },
                )
            ],
        },
    )

    # ── Step 2: wire the parent's ROS2Context → promoted context input ──
    # Full USD paths are required because these nodes already exist (they
    # were not created in this edit() call).
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CONNECT: [
                (f"{parent_graph_path}/{ros2_context_node}.outputs:context",
                 f"{parent_graph_path}/{lidar_subgraph_name}.inputs:context"),
            ],
        },
    )

    carb.log_info(f"LiDAR subgraph '{lidar_subgraph_name}' added under '{parent_graph_path}'.")