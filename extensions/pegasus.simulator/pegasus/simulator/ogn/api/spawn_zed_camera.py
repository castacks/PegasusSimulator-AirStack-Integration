import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom, Gf, Sdf
from omni.physx.scripts import utils as physx_utils
import omni

ZED_X_CAMERA_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Stereolabs/ZED_X/ZED_X.usdc"
)

def attach_camera_to_drone(
    drone_prim_path: str,
    camera_name: str,
    camera_usd: str,
    camera_offset: list[float],
    camera_rotation_offset: list[float],
    left_frame_id: str,
    right_frame_id: str,
):
    """
    Attach a ZED stereo camera USD to a drone with a fixed offset and orientation.

    Args:
        drone_prim_path (str): Path to the drone prim (e.g., "/World/Drone_01").
        camera_name (str): Name for the new camera prim created under the drone.
        camera_usd (str): USD reference path for the ZED camera model.
        camera_offset (list[float]): [x, y, z] translation offset relative to the drone.
        camera_rotation_offset (list[float]): [roll, pitch, yaw] rotation in degrees.
        left_frame_id (str): Desired left camera frame name (renamed in USD).
        right_frame_id (str): Desired right camera frame name (renamed in USD).
    """
    stage = omni.usd.get_context().get_stage()
    camera_prim_path = f"{drone_prim_path}/{camera_name}"

    # Create camera prim if it does not exist
    prim = get_prim_at_path(camera_prim_path)
    if not prim.IsValid():
        prim = define_prim(camera_prim_path, "Xform")
        prim.GetReferences().AddReference(camera_usd)

    # Remove any collision geometry to prevent unwanted physics behavior
    physx_utils.removeCollider(prim)

    # Compute orientation
    roll_deg, pitch_deg, yaw_deg = camera_rotation_offset
    roll_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll_deg)
    pitch_rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch_deg)
    yaw_rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg)

    combined_rot = yaw_rot * pitch_rot * roll_rot
    user_quat = combined_rot.GetQuat()
    user_rot = Gf.Quatf(user_quat.GetReal(), *user_quat.GetImaginary())

    # Apply translation and rotation
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    orient_op = xform.AddOrientOp()

    translate_op.Set(Gf.Vec3d(*camera_offset))
    orient_op.Set(user_rot)
    xform.SetXformOpOrder([translate_op, orient_op])

    # Create a fixed joint to lock camera to drone body
    physx_utils.createJoint(
        stage,
        joint_type="Fixed",
        from_prim=stage.GetPrimAtPath(f"{drone_prim_path}/body/body"),
        to_prim=stage.GetPrimAtPath(camera_prim_path),
    )

    # Rename internal left/right sensor prims to avoid name conflicts (currently done via copying prim)
    left_old = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/CameraLeft")
    right_old = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/CameraRight")
    left_new = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/{left_frame_id}")
    right_new = Sdf.Path(f"{camera_prim_path}/base_link/ZED_X/{right_frame_id}")

    app = omni.kit.app.get_app()
    app.update()

    omni.kit.commands.execute("CopyPrim", path_from=left_old, path_to=left_new)
    omni.kit.commands.execute("CopyPrim", path_from=right_old, path_to=right_new)

    # deactivate old prims, activate new prims
    stage.GetPrimAtPath(left_old).SetActive(False)
    stage.GetPrimAtPath(right_old).SetActive(False)
    stage.GetPrimAtPath(left_new).SetActive(True)
    stage.GetPrimAtPath(right_new).SetActive(True)

    app.update()

    print(f"Camera '{camera_name}' attached to drone '{drone_prim_path}'.")


def add_zed_stereo_camera_subgraph(
    parent_graph_handle: og._omni_graph_core.Graph,
    drone_prim: str,
    robot_name: str = "robot_1",
    camera_name: str = "ZEDCamera",
    camera_usd: str = ZED_X_CAMERA_USD_URL,
    camera_offset: list[float] = [0.12, 0.0, -0.02],
    camera_rotation_offset: list[float] = [0.0, 0.0, 0.0],
    stereo_topic_namespace: str = "front_stereo",
    sensors_topic_namespace: str = "sensors",
    left_frame_id: str = "camera_left",
    right_frame_id: str = "camera_right",
    frame_height: int = 300,
    frame_width: int = 480,
    ros2_context_node: str | None = None,
):
    """
    Create an Isaac Sim OmniGraph subgraph that connects a ZED stereo camera
    to the ROS2 bridge and render pipeline for RGB and depth outputs.

    The ROS2 context handle is received via a promoted ``inputs:context``
    attribute on the compound node, connected to the parent graph's
    ROS2Context node.  This avoids creating a separate ROS2Context
    inside the subgraph (which fails to initialise inside compound
    nodes) and avoids absolute-path references that break when the
    scene is saved and reloaded in a different location.

    Args:
        parent_graph_handle: OmniGraph parent where this subgraph will be added.
        drone_prim (str): Path to the drone prim (e.g., "/World/Drone_01").
        robot_name (str): Unique robot namespace prefix for ROS2 topics.
        camera_name (str): Name of the camera prim under the drone.
        camera_usd (str): Path to the ZED camera USD.
        camera_offset (list[float]): [x, y, z] local offset from drone body.
        camera_rotation_offset (list[float]): [roll, pitch, yaw] orientation offset.
        stereo_topic_namespace (str): ROS2 topic sub-namespace (e.g., "front_stereo").
        sensors_topic_namespace (str): Parent topic namespace (e.g., "sensors").
        left_frame_id (str): Frame ID for left camera.
        right_frame_id (str): Frame ID for right camera.
        ros2_context_node (str): Name of the ROS2Context node in the
            parent graph whose ``outputs:context`` will be wired into
            this subgraph's promoted ``inputs:context``.
    """

    if ros2_context_node is None:
        ros2_context_node = f"{robot_name}_ROS2Context"

    controller = og.Controller()
    parent_graph_path = parent_graph_handle.get_path_to_graph()
    stereo_graph_name = f"{robot_name}_{camera_name}StereoGraph"
    camera_prim_path = f"{drone_prim}/{camera_name}"

    # Physically attach the ZED camera to the drone
    attach_camera_to_drone(
        drone_prim,
        camera_name,
        camera_usd,
        camera_offset,
        camera_rotation_offset,
        left_frame_id,
        right_frame_id,
    )

    # Prim paths for left/right cameras
    left_camera_prim = f"{camera_prim_path}/base_link/ZED_X/{left_frame_id}"
    right_camera_prim = f"{camera_prim_path}/base_link/ZED_X/{right_frame_id}"

    # ROS2 topic namespaces
    left_ns = f"{robot_name}/{sensors_topic_namespace}/{stereo_topic_namespace}/left"
    right_ns = f"{robot_name}/{sensors_topic_namespace}/{stereo_topic_namespace}/right"
    stereo_ns = f"{robot_name}/{sensors_topic_namespace}/{stereo_topic_namespace}"

    # Node names
    nodes = {
        "playback": f"{robot_name}_{camera_name}_OnPlaybackTick",
        "info_helper": f"{robot_name}_{camera_name}_StereoInfoHelper",
        "stereo_ns_const": f"{robot_name}_{camera_name}_StereoNsConst",
    }

    # Node identifiers for each render/camera output
    left_nodes = {
        "create_rp": f"{robot_name}_{camera_name}_LeftCreateRenderProduct",
        "rgb_helper": f"{robot_name}_{camera_name}_LeftRGBCameraHelper",
        "depth_helper": f"{robot_name}_{camera_name}_LeftDepthCameraHelper",
        "frame_const": f"{robot_name}_{camera_name}_LeftFrameIdConst",
        "ns_const": f"{robot_name}_{camera_name}_LeftNsConst",
    }
    right_nodes = {
        "create_rp": f"{robot_name}_{camera_name}_RightCreateRenderProduct",
        "rgb_helper": f"{robot_name}_{camera_name}_RightRGBCameraHelper",
        "depth_helper": f"{robot_name}_{camera_name}_RightDepthCameraHelper",
        "frame_const": f"{robot_name}_{camera_name}_RightFrameIdConst",
        "ns_const": f"{robot_name}_{camera_name}_RightNsConst",
    }

    # ── Step 1: create the compound subgraph with promoted context inputs ──
    controller.edit(
        graph_id=parent_graph_path,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: [
                (
                    stereo_graph_name,
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            # Core nodes
                            (nodes["playback"], "omni.graph.action.OnPlaybackTick"),
                            (nodes["info_helper"], "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                            # Constant string inputs
                            (left_nodes["frame_const"], "omni.graph.nodes.ConstantString"),
                            (right_nodes["frame_const"], "omni.graph.nodes.ConstantString"),
                            (left_nodes["ns_const"], "omni.graph.nodes.ConstantString"),
                            (right_nodes["ns_const"], "omni.graph.nodes.ConstantString"),
                            (nodes["stereo_ns_const"], "omni.graph.nodes.ConstantString"),
                            # Left camera nodes
                            (left_nodes["create_rp"], "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            (left_nodes["rgb_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            (left_nodes["depth_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            # Right camera nodes
                            (right_nodes["create_rp"], "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            (right_nodes["rgb_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            (right_nodes["depth_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ],

                        # Wiring between nodes
                        og.Controller.Keys.CONNECT: [
                            # Trigger render products every physics step
                            (f"{nodes['playback']}.outputs:tick", f"{left_nodes['create_rp']}.inputs:execIn"),
                            (f"{nodes['playback']}.outputs:tick", f"{right_nodes['create_rp']}.inputs:execIn"),
                            (f"{right_nodes['create_rp']}.outputs:execOut", f"{nodes['info_helper']}.inputs:execIn"),

                            # Stereo info helper input connections
                            (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{nodes['info_helper']}.inputs:renderProductPath"),
                            (f"{right_nodes['create_rp']}.outputs:renderProductPath", f"{nodes['info_helper']}.inputs:renderProductPathRight"),

                            # Frame ID and namespace constants
                            (f"{left_nodes['frame_const']}.inputs:value", f"{nodes['info_helper']}.inputs:frameId"),
                            (f"{right_nodes['frame_const']}.inputs:value", f"{nodes['info_helper']}.inputs:frameIdRight"),
                            (f"{nodes['stereo_ns_const']}.inputs:value", f"{nodes['info_helper']}.inputs:nodeNamespace"),

                            # Left camera outputs
                            (f"{left_nodes['create_rp']}.outputs:execOut", f"{left_nodes['rgb_helper']}.inputs:execIn"),
                            (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{left_nodes['depth_helper']}.inputs:renderProductPath"),
                            (f"{left_nodes['frame_const']}.inputs:value", f"{left_nodes['rgb_helper']}.inputs:frameId"),
                            (f"{left_nodes['ns_const']}.inputs:value", f"{left_nodes['rgb_helper']}.inputs:nodeNamespace"),

                            (f"{left_nodes['create_rp']}.outputs:execOut", f"{left_nodes['depth_helper']}.inputs:execIn"),
                            (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{left_nodes['rgb_helper']}.inputs:renderProductPath"),
                            (f"{left_nodes['frame_const']}.inputs:value", f"{left_nodes['depth_helper']}.inputs:frameId"),
                            (f"{left_nodes['ns_const']}.inputs:value", f"{left_nodes['depth_helper']}.inputs:nodeNamespace"),

                            # Right camera outputs
                            (f"{right_nodes['create_rp']}.outputs:execOut", f"{right_nodes['rgb_helper']}.inputs:execIn"),
                            (f"{right_nodes['create_rp']}.outputs:renderProductPath", f"{right_nodes['rgb_helper']}.inputs:renderProductPath"),
                            (f"{right_nodes['frame_const']}.inputs:value", f"{right_nodes['rgb_helper']}.inputs:frameId"),
                            (f"{right_nodes['ns_const']}.inputs:value", f"{right_nodes['rgb_helper']}.inputs:nodeNamespace"),

                            (f"{right_nodes['create_rp']}.outputs:execOut", f"{right_nodes['depth_helper']}.inputs:execIn"),
                            (f"{right_nodes['create_rp']}.outputs:renderProductPath", f"{right_nodes['depth_helper']}.inputs:renderProductPath"),
                            (f"{right_nodes['frame_const']}.inputs:value", f"{right_nodes['depth_helper']}.inputs:frameId"),
                            (f"{right_nodes['ns_const']}.inputs:value", f"{right_nodes['depth_helper']}.inputs:nodeNamespace"),
                        ],

                        # Static attribute values
                        og.Controller.Keys.SET_VALUES: [
                            # Frame IDs and namespaces
                            (("inputs:value", left_nodes["frame_const"]), left_frame_id),
                            (("inputs:value", right_nodes["frame_const"]), right_frame_id),
                            (("inputs:value", left_nodes["ns_const"]), left_ns),
                            (("inputs:value", right_nodes["ns_const"]), right_ns),
                            (("inputs:value", nodes["stereo_ns_const"]), stereo_ns),

                            # Stereo info topics
                            (("inputs:topicName", nodes["info_helper"]), "left/camera_info"),
                            (("inputs:topicNameRight", nodes["info_helper"]), "right/camera_info"),

                            # Left render product + helpers
                            (("inputs:cameraPrim", left_nodes["create_rp"]), left_camera_prim),
                            (("inputs:height", left_nodes["create_rp"]), frame_height),
                            (("inputs:width", left_nodes["create_rp"]), frame_width),
                            (("inputs:type", left_nodes["rgb_helper"]), "rgb"),
                            (("inputs:type", left_nodes["depth_helper"]), "depth"),
                            (("inputs:topicName", left_nodes["rgb_helper"]), "image_rect"),
                            (("inputs:topicName", left_nodes["depth_helper"]), "depth_ground_truth"),

                            # Right render product + helpers
                            (("inputs:cameraPrim", right_nodes["create_rp"]), right_camera_prim),
                            (("inputs:height", right_nodes["create_rp"]), frame_height),
                            (("inputs:width", right_nodes["create_rp"]), frame_width),
                            (("inputs:type", right_nodes["rgb_helper"]), "rgb"),
                            (("inputs:type", right_nodes["depth_helper"]), "depth"),
                            (("inputs:topicName", right_nodes["rgb_helper"]), "image_rect"),
                            (("inputs:topicName", right_nodes["depth_helper"]), "depth_ground_truth"),
                        ],

                        # Promote each helper's context input with a unique boundary name.
                        # OmniGraph doesn't allow multiple promotions to the same name.
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            (f"{nodes['info_helper']}.inputs:context", "inputs:context_info"),
                            (f"{left_nodes['rgb_helper']}.inputs:context", "inputs:context_left_rgb"),
                            (f"{left_nodes['depth_helper']}.inputs:context", "inputs:context_left_depth"),
                            (f"{right_nodes['rgb_helper']}.inputs:context", "inputs:context_right_rgb"),
                            (f"{right_nodes['depth_helper']}.inputs:context", "inputs:context_right_depth"),
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
                 f"{parent_graph_path}/{stereo_graph_name}.inputs:context_info"),
                (f"{parent_graph_path}/{ros2_context_node}.outputs:context",
                 f"{parent_graph_path}/{stereo_graph_name}.inputs:context_left_rgb"),
                (f"{parent_graph_path}/{ros2_context_node}.outputs:context",
                 f"{parent_graph_path}/{stereo_graph_name}.inputs:context_left_depth"),
                (f"{parent_graph_path}/{ros2_context_node}.outputs:context",
                 f"{parent_graph_path}/{stereo_graph_name}.inputs:context_right_rgb"),
                (f"{parent_graph_path}/{ros2_context_node}.outputs:context",
                 f"{parent_graph_path}/{stereo_graph_name}.inputs:context_right_depth"),
            ],
        },
    )

    print(f"Created ZED stereo camera graph '{stereo_graph_name}' under drone '{drone_prim}'")
