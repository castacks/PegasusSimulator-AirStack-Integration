import omni.graph.core as og
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom, Gf, Sdf
from omni.physx.scripts import utils as physx_utils
import os
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
    pipeline_mode: str = "stereo",
    manage_render_updates: bool = False,
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
        pipeline_mode (str): ``stereo`` preserves both render products;
            ``mono_rgbd`` renders only the left eye and publishes RGB, ground-
            truth depth, depth point cloud, and camera info from that product.
        manage_render_updates (bool): Create the render product through
            Replicator and return its handle so a fleet launcher can pause the
            inactive Hydra textures.  This is opt-in because it changes render-
            product ownership; the historical OmniGraph-created path remains
            the default.

    Returns:
        The promoted ``inputs:render_step`` attribute used to rate-limit or
        phase-schedule this camera graph.  When ``manage_render_updates`` is
        true, returns ``(render_step_attribute, render_products)`` instead.
    """

    if pipeline_mode not in ("stereo", "mono_rgbd"):
        raise ValueError("pipeline_mode must be 'stereo' or 'mono_rgbd', got "
                         f"{pipeline_mode!r}")
    mono_rgbd = pipeline_mode == "mono_rgbd"

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
        "gate": f"{robot_name}_{camera_name}_RenderGate",
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

    # Build the mode-dependent pieces separately so the default stereo graph
    # remains exactly the historical graph.  mono_rgbd has no right render
    # product at all and publishes a PointCloud2 directly from the left GT
    # depth render product; this is materially cheaper than merely suppressing
    # the right depth writer while continuing to render the right eye.
    camera_nodes = [
        (left_nodes["create_rp"], "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        (left_nodes["rgb_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
        (left_nodes["depth_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    camera_connections = [
        (f"{nodes['gate']}.outputs:execOut", f"{left_nodes['create_rp']}.inputs:execIn"),
        (f"{left_nodes['create_rp']}.outputs:execOut", f"{left_nodes['rgb_helper']}.inputs:execIn"),
        (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{left_nodes['rgb_helper']}.inputs:renderProductPath"),
        (f"{left_nodes['frame_const']}.inputs:value", f"{left_nodes['rgb_helper']}.inputs:frameId"),
        (f"{left_nodes['ns_const']}.inputs:value", f"{left_nodes['rgb_helper']}.inputs:nodeNamespace"),
        (f"{left_nodes['create_rp']}.outputs:execOut", f"{left_nodes['depth_helper']}.inputs:execIn"),
        (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{left_nodes['depth_helper']}.inputs:renderProductPath"),
        (f"{left_nodes['frame_const']}.inputs:value", f"{left_nodes['depth_helper']}.inputs:frameId"),
        (f"{left_nodes['ns_const']}.inputs:value", f"{left_nodes['depth_helper']}.inputs:nodeNamespace"),
    ]
    camera_values = [
        (("inputs:cameraPrim", left_nodes["create_rp"]), left_camera_prim),
        (("inputs:height", left_nodes["create_rp"]), frame_height),
        (("inputs:width", left_nodes["create_rp"]), frame_width),
        (("inputs:type", left_nodes["rgb_helper"]), "rgb"),
        (("inputs:type", left_nodes["depth_helper"]), "depth"),
        (("inputs:topicName", left_nodes["rgb_helper"]), "image_rect"),
        (("inputs:topicName", left_nodes["depth_helper"]), "depth_ground_truth"),
    ]
    context_promotions = [
        (f"{left_nodes['rgb_helper']}.inputs:context", "inputs:context_left_rgb"),
        (f"{left_nodes['depth_helper']}.inputs:context", "inputs:context_left_depth"),
    ]
    context_connections = ["context_left_rgb", "context_left_depth"]
    if mono_rgbd:
        left_nodes["pcl_helper"] = f"{robot_name}_{camera_name}_LeftDepthPclHelper"
        camera_nodes.extend([
            (left_nodes["pcl_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (nodes["info_helper"], "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ])
        camera_connections.extend([
            (f"{left_nodes['create_rp']}.outputs:execOut", f"{left_nodes['pcl_helper']}.inputs:execIn"),
            (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{left_nodes['pcl_helper']}.inputs:renderProductPath"),
            (f"{left_nodes['frame_const']}.inputs:value", f"{left_nodes['pcl_helper']}.inputs:frameId"),
            (f"{left_nodes['ns_const']}.inputs:value", f"{left_nodes['pcl_helper']}.inputs:nodeNamespace"),
            (f"{left_nodes['create_rp']}.outputs:execOut", f"{nodes['info_helper']}.inputs:execIn"),
            (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{nodes['info_helper']}.inputs:renderProductPath"),
            (f"{left_nodes['frame_const']}.inputs:value", f"{nodes['info_helper']}.inputs:frameId"),
            (f"{left_nodes['ns_const']}.inputs:value", f"{nodes['info_helper']}.inputs:nodeNamespace"),
        ])
        camera_values.extend([
            (("inputs:type", left_nodes["pcl_helper"]), "depth_pcl"),
            (("inputs:topicName", left_nodes["pcl_helper"]), "depth_pcl"),
            (("inputs:topicName", nodes["info_helper"]), "camera_info"),
        ])
        context_promotions.extend([
            (f"{left_nodes['pcl_helper']}.inputs:context", "inputs:context_left_pcl"),
            (f"{nodes['info_helper']}.inputs:context", "inputs:context_info"),
        ])
        context_connections.extend(["context_left_pcl", "context_info"])
    else:
        camera_nodes.extend([
            (nodes["info_helper"], "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            (right_nodes["frame_const"], "omni.graph.nodes.ConstantString"),
            (right_nodes["ns_const"], "omni.graph.nodes.ConstantString"),
            (nodes["stereo_ns_const"], "omni.graph.nodes.ConstantString"),
            (right_nodes["create_rp"], "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            (right_nodes["rgb_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (right_nodes["depth_helper"], "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ])
        camera_connections.extend([
            (f"{nodes['gate']}.outputs:execOut", f"{right_nodes['create_rp']}.inputs:execIn"),
            (f"{right_nodes['create_rp']}.outputs:execOut", f"{nodes['info_helper']}.inputs:execIn"),
            (f"{left_nodes['create_rp']}.outputs:renderProductPath", f"{nodes['info_helper']}.inputs:renderProductPath"),
            (f"{right_nodes['create_rp']}.outputs:renderProductPath", f"{nodes['info_helper']}.inputs:renderProductPathRight"),
            (f"{left_nodes['frame_const']}.inputs:value", f"{nodes['info_helper']}.inputs:frameId"),
            (f"{right_nodes['frame_const']}.inputs:value", f"{nodes['info_helper']}.inputs:frameIdRight"),
            (f"{nodes['stereo_ns_const']}.inputs:value", f"{nodes['info_helper']}.inputs:nodeNamespace"),
            (f"{right_nodes['create_rp']}.outputs:execOut", f"{right_nodes['rgb_helper']}.inputs:execIn"),
            (f"{right_nodes['create_rp']}.outputs:renderProductPath", f"{right_nodes['rgb_helper']}.inputs:renderProductPath"),
            (f"{right_nodes['frame_const']}.inputs:value", f"{right_nodes['rgb_helper']}.inputs:frameId"),
            (f"{right_nodes['ns_const']}.inputs:value", f"{right_nodes['rgb_helper']}.inputs:nodeNamespace"),
            (f"{right_nodes['create_rp']}.outputs:execOut", f"{right_nodes['depth_helper']}.inputs:execIn"),
            (f"{right_nodes['create_rp']}.outputs:renderProductPath", f"{right_nodes['depth_helper']}.inputs:renderProductPath"),
            (f"{right_nodes['frame_const']}.inputs:value", f"{right_nodes['depth_helper']}.inputs:frameId"),
            (f"{right_nodes['ns_const']}.inputs:value", f"{right_nodes['depth_helper']}.inputs:nodeNamespace"),
        ])
        camera_values.extend([
            (("inputs:value", right_nodes["frame_const"]), right_frame_id),
            (("inputs:value", right_nodes["ns_const"]), right_ns),
            (("inputs:value", nodes["stereo_ns_const"]), stereo_ns),
            (("inputs:topicName", nodes["info_helper"]), "left/camera_info"),
            (("inputs:topicNameRight", nodes["info_helper"]), "right/camera_info"),
            (("inputs:cameraPrim", right_nodes["create_rp"]), right_camera_prim),
            (("inputs:height", right_nodes["create_rp"]), frame_height),
            (("inputs:width", right_nodes["create_rp"]), frame_width),
            (("inputs:type", right_nodes["rgb_helper"]), "rgb"),
            (("inputs:type", right_nodes["depth_helper"]), "depth"),
            (("inputs:topicName", right_nodes["rgb_helper"]), "image_rect"),
            (("inputs:topicName", right_nodes["depth_helper"]), "depth_ground_truth"),
        ])
        context_promotions.extend([
            (f"{nodes['info_helper']}.inputs:context", "inputs:context_info"),
            (f"{right_nodes['rgb_helper']}.inputs:context", "inputs:context_right_rgb"),
            (f"{right_nodes['depth_helper']}.inputs:context", "inputs:context_right_depth"),
        ])
        context_connections.extend(["context_info", "context_right_rgb", "context_right_depth"])

    # A SimulationGate controls graph execution/publication, but an authored
    # render product can remain active in Hydra after its helper has attached.
    # For fleet time slicing, create the products through Replicator so the
    # launcher owns their HydraTexture handles and can pause actual rendering
    # on inactive cameras.  Keep the graph construction above as the single
    # source of helper wiring, then mechanically replace each create-product
    # node with a static path to the equivalent Replicator product.
    managed_render_products = []
    if manage_render_updates:
        import omni.replicator.core as rep

        left_rp = rep.create.render_product(
            left_camera_prim, (frame_width, frame_height),
            name=f"{robot_name}_{camera_name}_left_rp")
        managed_render_products.append(left_rp)
        render_products_by_node = {left_nodes["create_rp"]: left_rp}
        if not mono_rgbd:
            right_rp = rep.create.render_product(
                right_camera_prim, (frame_width, frame_height),
                name=f"{robot_name}_{camera_name}_right_rp")
            managed_render_products.append(right_rp)
            render_products_by_node[right_nodes["create_rp"]] = right_rp

        create_nodes = set(render_products_by_node)
        camera_nodes = [
            item for item in camera_nodes if item[0] not in create_nodes]
        camera_values = [
            item for item in camera_values
            if not (isinstance(item[0], tuple) and
                    len(item[0]) == 2 and item[0][1] in create_nodes)]

        managed_connections = []
        for source, destination in camera_connections:
            source_node = source.split(".", 1)[0]
            destination_node = destination.split(".", 1)[0]
            if destination_node in create_nodes:
                # The product already exists; there is no create node to tick.
                continue
            if source_node not in create_nodes:
                managed_connections.append((source, destination))
                continue
            if source.endswith(".outputs:renderProductPath"):
                camera_values.append(
                    (destination, str(render_products_by_node[source_node].path)))
            elif source.endswith(".outputs:execOut"):
                managed_connections.append(
                    (f"{nodes['gate']}.outputs:execOut", destination))
            else:
                raise RuntimeError(
                    f"unsupported create-render-product edge: {source} -> "
                    f"{destination}")
        camera_connections = managed_connections

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
                            (nodes["gate"], "isaacsim.core.nodes.IsaacSimulationGate"),
                            # Constant string inputs
                            (left_nodes["frame_const"], "omni.graph.nodes.ConstantString"),
                            (left_nodes["ns_const"], "omni.graph.nodes.ConstantString"),
                            *camera_nodes,
                        ],

                        # Wiring between nodes
                        og.Controller.Keys.CONNECT: [
                            # Trigger render products through a SIMULATION GATE
                            # rather than on every physics step.
                            #
                            # Rendering is what saturates the GPU in the
                            # 8-drone benchmark: measured 2026-08-31, GPU 1 sat
                            # at 73-84 % with 5.8 GB while the sim managed
                            # RTF 0.048. Sixteen render products (left+right
                            # per drone) firing every tick is the load. The
                            # gate's `step` divides that: step=2 renders every
                            # other tick, halving camera cost for a
                            # proportional drop in image rate (~4.4 Hz -> ~2.2 Hz
                            # as measured on this scene).
                            #
                            # MEASURED with step=2 on this scene: RTF rose
                            # 0.048 -> 0.063 (+31 %) and GPU fell from 73-84 %
                            # / 5.8 GB to 0 % / 2.97 GB — but the image rate
                            # fell 4.37 -> 1.39 Hz, a 3x drop rather than the
                            # 2x asked for. The detector already cleared its
                            # 0.65 gate on a minority of ticks, so a third of
                            # the frames is a worse trade than a 31 % speedup
                            # is worth. DEFAULT IS THEREFORE 1 (every tick,
                            # original behaviour); set ZED_RENDER_STEP=2 to
                            # buy speed back when frames are cheap.
                            (f"{nodes['playback']}.outputs:tick", f"{nodes['gate']}.inputs:execIn"),
                            *camera_connections,
                        ],

                        # Static attribute values
                        og.Controller.Keys.SET_VALUES: [
                            (f"{nodes['gate']}.inputs:step",
                             int(os.environ.get("ZED_RENDER_STEP", "").strip() or 1)),
                            # Frame IDs and namespaces
                            (("inputs:value", left_nodes["frame_const"]), left_frame_id),
                            (("inputs:value", left_nodes["ns_const"]), left_ns),
                            *camera_values,
                        ],

                        # Promote each helper's context input with a unique boundary name.
                        # OmniGraph doesn't allow multiple promotions to the same name.
                        og.Controller.Keys.PROMOTE_ATTRIBUTES: [
                            (f"{nodes['gate']}.inputs:step", "inputs:render_step"),
                            *context_promotions,
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
                 f"{parent_graph_path}/{stereo_graph_name}.inputs:{boundary}")
                for boundary in context_connections
            ],
        },
    )

    print(f"Created ZED {pipeline_mode} camera graph '{stereo_graph_name}' under drone '{drone_prim}'")
    # The launcher may use this handle to stagger logical cameras across
    # simulation ticks. Leaving it at the configured step preserves the
    # historical all-cameras-together schedule.
    render_step_attribute = controller.attribute(
        f"{parent_graph_path}/{stereo_graph_name}.inputs:render_step")
    if manage_render_updates:
        return render_step_attribute, managed_render_products
    return render_step_attribute
