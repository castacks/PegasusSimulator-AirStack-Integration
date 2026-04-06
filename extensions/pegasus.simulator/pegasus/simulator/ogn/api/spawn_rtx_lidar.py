"""
RTX LiDAR spawn utilities for the Pegasus Simulator extension.

Uses Isaac Sim 5.0+ OmniLidar prims via ``IsaacSensorCreateRtxLidar``.
When ``min_range > 0``, sets ``omni:sensor:Core:nearRangeM`` on the sensor prim if
that attribute exists (some builds expose echo-spacing only in the lidar JSON
profile, not as a Core prim attribute—those are left to Isaac defaults).

For self-hits, tune ``min_range``, mount offset, ROS / VDB filtering
(``min_sensor_range``), or edit the RTX lidar profile JSON under the Isaac install.
"""

import omni.graph.core as og
from pxr import UsdGeom, Gf, UsdPhysics
from omni.physx.scripts import utils as physx_utils
import omni
import carb
from isaacsim.core.utils.prims import set_targets


def _rtx_lidar_render_product_prim_path(
    stage, lidar_container_path: str, command_return_path: str
) -> str:
    """Resolve prim path for ``IsaacCreateRenderProduct`` / RTX lidar.

    Vendor OmniLidar USD (e.g. OS1) usually nests the actual RTX camera under
    ``<container>/sensor``. ``IsaacSensorCreateRtxLidar`` may return either the
    container or the leaf; targeting the wrong prim breaks the graph (no hits).
    """
    seen: set[str] = set()
    candidates: list[str] = []
    for p in (
        f"{lidar_container_path}/sensor",
        f"{lidar_container_path}/Sensor",
        f"{command_return_path}/sensor",
        f"{command_return_path}/Sensor",
        command_return_path,
    ):
        if p not in seen:
            seen.add(p)
            candidates.append(p)
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            carb.log_info(f"[RTX LiDAR] Render product source prim: '{path}'")
            return path
    carb.log_warn(
        f"[RTX LiDAR] No valid sensor prim among {candidates!r}; "
        f"using '{command_return_path}'"
    )
    return command_return_path


def _set_rtx_create_render_camera_prim(
    stage, parent_graph_path: str, subgraph_name: str, create_render_name: str, sensor_prim_path: str
) -> bool:
    """Wire ``inputs:cameraPrim`` on ``IsaacCreateRenderProduct`` (compound subgraph layout varies by Kit)."""
    base = f"{parent_graph_path}/{subgraph_name}"
    for rel in (f"{base}/Subgraph/{create_render_name}", f"{base}/{create_render_name}"):
        prim = stage.GetPrimAtPath(rel)
        if prim.IsValid():
            set_targets(
                prim=prim,
                attribute="inputs:cameraPrim",
                target_prim_paths=[sensor_prim_path],
            )
            carb.log_info(
                f"[RTX LiDAR] inputs:cameraPrim on '{rel}' -> '{sensor_prim_path}'"
            )
            return True
    carb.log_error(
        f"[RTX LiDAR] IsaacCreateRenderProduct node not found under '{base}' "
        f"(tried Subgraph/ and flat layout)."
    )
    return False


# ─── RTX Lidar Config Name Aliases ──────────────────────────────────────
# Short aliases → (config, variant) tuples for IsaacSensorCreateRtxLidar.
# Isaac Sim 5.0+ requires separate ``config`` (model name) and ``variant``
# (model variant) arguments.  Pass the alias or explicit (config, variant)
# to the public API functions below.
LIDAR_CONFIG_ALIASES: dict[str, tuple[str, str]] = {
    "ouster_os1": ("OS1", "OS1_REV6_128ch10hz512res"),
    "ouster_os0": ("OS0", "OS0_REV7_128ch10hz512res"),
    "ouster_os2": ("OS2", "OS2_REV6_128ch10hz512res"),
    "example_rotary": ("Example_Rotary", ""),
    "velodyne_vlp16": ("Velodyne_VLP16", ""),
}

# Default (config, variant) used when none is specified
DEFAULT_LIDAR_CONFIG = "ouster_os1"


def _resolve_config(lidar_config: str, lidar_variant: str = "") -> tuple[str, str]:
    """Resolve a short alias or explicit names to a (config, variant) pair.

    If ``lidar_config`` matches a known alias the alias table takes precedence.
    Otherwise ``lidar_config`` is used as the model name and ``lidar_variant``
    as the variant string directly.
    """
    alias = lidar_config.lower()
    if alias in LIDAR_CONFIG_ALIASES:
        return LIDAR_CONFIG_ALIASES[alias]
    return lidar_config, lidar_variant


def attach_rtx_lidar_to_drone(
    drone_prim_path: str,
    lidar_name: str,
    lidar_config: str,
    lidar_offset: list[float],
    lidar_rotation_offset: list[float],
    frame_id: str,
    min_range: float = 0.0,
    lidar_variant: str = "",
) -> str | None:
    """Create an OmniLidar prim and attach it to a drone via fixed joint.

    Uses ``IsaacSensorCreateRtxLidar`` which loads the vendor USD mesh
    (e.g. Ouster OS1) automatically.  The ``nearRangeM`` is set directly
    on the OmniLidar prim attribute so it persists across Save/Reload.

    Args:
        drone_prim_path: Stage path to the drone (e.g. ``"/World/base_link"``).
        lidar_name: Name for the lidar container prim under the drone.
        lidar_config: Model name alias (e.g. ``"ouster_os1"``) or explicit
            Isaac Sim model name (e.g. ``"OS1"``).
        lidar_offset: ``[x, y, z]`` translation offset relative to the drone.
        lidar_rotation_offset: ``[roll, pitch, yaw]`` in degrees.
        frame_id: ROS ``frame_id`` for published point clouds (wired in the
            OmniGraph subgraph).
        min_range: Minimum detection range in metres.  Set > 0 to set
            ``omni:sensor:Core:nearRangeM`` when present on the sensor prim.
        lidar_variant: Model variant string (e.g. ``"OS1_REV6_128ch10hz512res"``).
            Ignored when ``lidar_config`` matches a known alias.

    Returns:
        Stage path to the OmniLidar sensor prim, or ``None`` on failure.
    """
    stage = omni.usd.get_context().get_stage()
    lidar_path = f"{drone_prim_path}/{lidar_name}"
    config_name, variant_name = _resolve_config(lidar_config, lidar_variant)

    # Clean up existing prim
    if stage.GetPrimAtPath(lidar_path).IsValid():
        carb.log_info(f"Deleting existing LiDAR prim at {lidar_path}")
        omni.kit.commands.execute("DeletePrim", path=lidar_path)

    # Only pass attributes that exist on typical OmniLidar sensor prims; omit
    # minDistBetweenEchos etc. when not on the schema (avoids noisy runtime warnings).
    sensor_attributes = {}
    if min_range > 0.0:
        sensor_attributes["omni:sensor:Core:nearRangeM"] = min_range

    create_kwargs = dict(
        path=lidar_path,
        parent=None,
        config=config_name,
        translation=Gf.Vec3d(*lidar_offset),
        orientation=Gf.Quatd(1, 0, 0, 0),
        **sensor_attributes,
    )
    if variant_name:
        create_kwargs["variant"] = variant_name

    success, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        **create_kwargs,
    )

    if not success or not sensor_prim:
        carb.log_error(f"Failed to create RTX LiDAR prim at {lidar_path}")
        return None

    sensor_path = str(sensor_prim.GetPath())
    carb.log_info(f"[RTX LiDAR] Sensor prim created at '{sensor_path}'")

    # Re-apply on prim so Save/Reload and command quirks still match JSON behavior.
    if sensor_prim.IsValid() and min_range > 0.0:
        near_attr = sensor_prim.GetAttribute("omni:sensor:Core:nearRangeM")
        if near_attr and near_attr.IsValid():
            near_attr.Set(min_range)
            carb.log_info(
                f"[RTX LiDAR] Set omni:sensor:Core:nearRangeM = {min_range} "
                f"on '{sensor_path}'"
            )
        else:
            carb.log_warn(
                f"[RTX LiDAR] omni:sensor:Core:nearRangeM not available on "
                f"'{sensor_path}'; near range NOT overridden."
            )

    # ── Apply rotation to the container prim ──
    container_prim = stage.GetPrimAtPath(lidar_path)
    roll_deg, pitch_deg, yaw_deg = lidar_rotation_offset
    roll_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll_deg)
    pitch_rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch_deg)
    yaw_rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg)
    combined_rot = yaw_rot * pitch_rot * roll_rot
    user_quat = combined_rot.GetQuat()

    # Corrective rotation to align OmniLidar's local axes (Z-forward) with
    # the ROS/drone frame.
    corrective_quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()
    user_rot = Gf.Quatf(user_quat.GetReal(), *user_quat.GetImaginary())
    corrective_rot = Gf.Quatf(corrective_quat.GetReal(), *corrective_quat.GetImaginary())
    combined_qf = user_rot * corrective_rot
    final_rot = Gf.Quatd(combined_qf.GetReal(), *combined_qf.GetImaginary())

    xform = UsdGeom.Xformable(container_prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient_op = xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(*lidar_offset))
    orient_op.Set(final_rot)
    xform.SetXformOpOrder([translate_op, orient_op])

    # ── Physics: RigidBody + remove collider + fixed joint ──
    try:
        if not UsdPhysics.RigidBodyAPI(container_prim):
            UsdPhysics.RigidBodyAPI.Apply(container_prim)
    except Exception:
        carb.log_warn(f"RigidBodyAPI could not be applied to '{lidar_path}'.")

    try:
        physx_utils.removeCollider(container_prim)
    except Exception:
        pass

    drone_body_path = f"{drone_prim_path}/body/body"
    drone_body_prim = stage.GetPrimAtPath(drone_body_path)
    if not drone_body_prim.IsValid():
        carb.log_error(
            f"Drone body prim '{drone_body_path}' not found; cannot attach LiDAR."
        )
        return None

    joint = physx_utils.createJoint(
        stage,
        joint_type="Fixed",
        from_prim=drone_body_prim,
        to_prim=container_prim,
    )

    if not joint or not joint.IsValid():
        carb.log_error(f"Failed to create fixed joint for LiDAR '{lidar_path}'.")
        return None

    carb.log_info(
        f"[RTX LiDAR] '{lidar_name}' attached to '{drone_prim_path}' "
        f"(sensor: '{sensor_path}')."
    )
    return sensor_path


def add_rtx_lidar_subgraph(
    parent_graph_handle: og._omni_graph_core.Graph,
    drone_prim: str,
    lidar_name: str = "Lidar",
    lidar_config: str = DEFAULT_LIDAR_CONFIG,
    lidar_variant: str = "",
    lidar_topic_name: str = "point_cloud",
    lidar_offset: list[float] = [0.0, 0.0, 0.025],
    lidar_rotation_offset: list[float] = [0.0, 0.0, 0.0],
    lidar_topic_namespace: str = "sensors/ouster",
    lidar_frame_id: str = "ouster",
    robot_name: str = "robot_1",
    min_range: float = 0.0,
    ros2_context_node: str | None = None,
):
    """Attach an RTX LiDAR and build a ROS 2 OmniGraph subgraph for it.

    The graph publishes ``sensor_msgs/PointCloud2`` via
    ``ROS2RtxLidarHelper``.  The ROS 2 context is received through a
    promoted ``inputs:context`` attribute wired from the parent graph's
    ``ROS2Context`` node.

    Args:
        parent_graph_handle: Parent OmniGraph (returned by
            ``spawn_px4_multirotor_node``).
        drone_prim: Stage path to the drone prim.
        lidar_name: Prim name for the lidar under the drone.
        lidar_config: Model name alias (e.g. ``"ouster_os1"``) or explicit
            Isaac Sim model name (e.g. ``"OS1"``).
        lidar_variant: Model variant string (e.g. ``"OS1_REV6_128ch10hz512res"``).
            Ignored when ``lidar_config`` matches a known alias.
        lidar_topic_name: ROS topic name for the point cloud.
        lidar_offset: ``[x, y, z]`` offset from the drone.
        lidar_rotation_offset: ``[roll, pitch, yaw]`` in degrees.
        lidar_topic_namespace: ROS topic namespace.
        lidar_frame_id: ROS frame ID for the point cloud.
        robot_name: Robot name prefix for topic namespacing.
        min_range: Minimum detection range in metres (0 = use sensor
            default, typically ~0.3 m).
        ros2_context_node: Name of the ``ROS2Context`` node in the parent
            graph.  Defaults to ``"{robot_name}_ROS2Context"``.
    """
    if ros2_context_node is None:
        ros2_context_node = f"{robot_name}_ROS2Context"

    sensor_path = attach_rtx_lidar_to_drone(
        drone_prim_path=drone_prim,
        lidar_name=lidar_name,
        lidar_config=lidar_config,
        lidar_offset=lidar_offset,
        lidar_rotation_offset=lidar_rotation_offset,
        frame_id=lidar_frame_id,
        min_range=min_range,
        lidar_variant=lidar_variant,
    )

    if sensor_path is None:
        carb.log_error("LiDAR attachment failed; aborting subgraph creation.")
        return

    stage = omni.usd.get_context().get_stage()
    lidar_container_path = f"{drone_prim.rstrip('/')}/{lidar_name}"
    render_prim_path = _rtx_lidar_render_product_prim_path(
        stage, lidar_container_path, sensor_path
    )

    controller = og.Controller()
    parent_graph_path = parent_graph_handle.get_path_to_graph()

    subgraph_name = f"{lidar_name}Graph"
    playback_tick = f"{lidar_name}OnPlaybackTick"
    create_render = f"{lidar_name}CreateRenderProduct"
    rtx_helper = f"{lidar_name}ROS2RtxLidarHelper"
    frame_const = f"{lidar_name}FrameIdConst"
    ns_const = f"{lidar_name}NamespaceConst"

    # ── Step 1: compound subgraph with promoted context input ──
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (
                    subgraph_name,
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
                            # cameraPrim is set via set_targets() below
                            (("inputs:topicName", rtx_helper), lidar_topic_name),
                            (("inputs:type", rtx_helper), "point_cloud"),
                            (("inputs:fullScan", rtx_helper), True),
                        ],
                        og.Controller.Keys.CONNECT: [
                            (f"{playback_tick}.outputs:tick", f"{create_render}.inputs:execIn"),
                            (f"{create_render}.outputs:execOut", f"{rtx_helper}.inputs:execIn"),
                            (f"{create_render}.outputs:renderProductPath", f"{rtx_helper}.inputs:renderProductPath"),
                            (f"{frame_const}.inputs:value", f"{rtx_helper}.inputs:frameId"),
                            (f"{ns_const}.inputs:value", f"{rtx_helper}.inputs:nodeNamespace"),
                        ],
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            (f"{rtx_helper}.inputs:context", "inputs:context"),
                        ],
                    },
                )
            ],
        },
    )

    # ── Step 1.5: set cameraPrim via USD relationship ──
    # inputs:cameraPrim is a USD target; compound subgraphs may nest under
    # ``Subgraph/`` or flat — try both. Target the nested ``sensor`` prim when present.
    if not _set_rtx_create_render_camera_prim(
        stage, parent_graph_path, subgraph_name, create_render, render_prim_path
    ):
        return

    # ── Step 2: wire parent ROS2Context → promoted context input ──
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CONNECT: [
                (
                    f"{parent_graph_path}/{ros2_context_node}.outputs:context",
                    f"{parent_graph_path}/{subgraph_name}.inputs:context",
                ),
            ],
        },
    )

    carb.log_info(
        f"[RTX LiDAR] Subgraph '{subgraph_name}' added under "
        f"'{parent_graph_path}'."
    )
