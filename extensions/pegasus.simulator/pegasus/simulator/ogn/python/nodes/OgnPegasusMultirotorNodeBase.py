"""
Base class for Pegasus Multirotor OmniGraph nodes.
Provides common functionality for drone spawning and management.
"""

import inspect
import os
import traceback
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import carb
import omni
import omni.client
import omni.graph.core as og
import omni.graph.tools.ogn as ogn
import omni.usd
import omni.replicator.core as rep
import omni.timeline
import usdrt.Sdf
from isaacsim.core.prims import SingleGeometryPrim as GeometryPrim, SingleRigidPrim as RigidPrim, SingleXFormPrim as XFormPrim
from isaacsim.core.utils import extensions, stage
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
import time
import threading
import struct
import socket
import asyncio

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, BACKENDS, WORLD_SETTINGS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends import Backend, BackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

# Global variables for drone simulation tracking
drone_sim_dict = {}
timeline = None

# Stored globally so they are not garbage-collected.
# A GC'd carb subscription is silently cancelled — the callback never fires.
_timeline_sub_play = None
_timeline_sub_pause = None
_timeline_sub_stop = None


def timeline_callback(event):
    """Timeline callback to handle simulation events"""
    global drone_sim_dict

    if event.type == int(omni.timeline.TimelineEventType.PLAY):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.STOP):
        # Stop all backends before clearing the dictionary to prevent zombie processes
        world = World.instance()
        for node_id, drone_data in drone_sim_dict.items():
            backend = drone_data.get('backend')
            if backend:
                try:
                    backend.stop()
                    carb.log_info(f"[Pegasus] Stopped backend for node {node_id}")
                except Exception as e:
                    carb.log_warn(f"[Pegasus] Error stopping backend for node {node_id}: {e}")

            # Remove the Vehicle's physics/timeline/render callbacks so they
            # don't conflict when compute_base re-creates the drone on Play.
            multirotor = drone_data.get('multirotor')
            if multirotor and world is not None:
                prefix = getattr(multirotor, '_stage_prefix', '')
                if prefix:
                    for suffix in ["/state", "/update", "/Sensors", "/mav_state"]:
                        try:
                            world.remove_physics_callback(prefix + suffix)
                        except Exception:
                            pass
                    try:
                        world.remove_timeline_callback(prefix + "/start_stop_sim")
                    except Exception:
                        pass
                    try:
                        world.remove_render_callback(prefix + "/GraphicalSensors")
                    except Exception:
                        pass

        num_cleared = len(drone_sim_dict)
        drone_sim_dict = {}
        carb.log_info(f"[Pegasus] Timeline STOP: cleared {num_cleared} drone(s)")


class OgnPegasusMultirotorNodeBaseState:
    def __init__(self):
        self.node_initialized: bool = False  # Flag used to check if the per-instance node state is initialized.


class OgnPegasusMultirotorNodeBase:
    """Base class for Pegasus Multirotor OmniGraph nodes"""
    
    @staticmethod
    def internal_state():
        return OgnPegasusMultirotorNodeBaseState()

    @staticmethod
    def _is_initialized(node: og.Node) -> bool:
        return og.Controller.get(node.get_attribute("state:omni_initialized"))

    @staticmethod
    def _set_initialized(node: og.Node, init: bool):
        return og.Controller.set(node.get_attribute("state:omni_initialized"), init)

    @staticmethod
    def initialize(context, node: og.Node, db_class):
        """Initialize the node"""
        # Initialize state if shared_internal_state exists
        try:
            state = db_class.shared_internal_state(node)
            state.node_initialized = True
        except AttributeError:
            # If shared_internal_state doesn't exist, create a simple state
            pass
        
        OgnPegasusMultirotorNodeBase._set_initialized(node, False)

    @staticmethod
    def release(node: og.Node):
        """Release node resources"""
        # Clean up drone simulation
        OgnPegasusMultirotorNodeBase.try_cleanup(node)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Overrides the release_instance method so that any per-script cleanup can happen before the per-node data
        is deleted.
        """
        # Same logic as when the reset button is pressed
        OgnPegasusMultirotorNodeBase.try_cleanup(node)

    @staticmethod
    def try_cleanup(node: og.Node):
        """Clean up drone simulation resources"""
        global drone_sim_dict
        
        # Skip if not setup in the first place or already cleaned up
        if not OgnPegasusMultirotorNodeBase._is_initialized(node):
            return

        node_id = node.node_id()
        
        # Clean up drone simulation if it exists
        if node_id in drone_sim_dict:
            try:
                drone_data = drone_sim_dict[node_id]
                if 'backend' in drone_data and drone_data['backend']:
                    # Clean up backend if it has cleanup methods
                    if hasattr(drone_data['backend'], 'cleanup'):
                        drone_data['backend'].cleanup()
                    
                if 'multirotor' in drone_data and drone_data['multirotor']:
                    # Clean up multirotor if it has cleanup methods
                    if hasattr(drone_data['multirotor'], 'cleanup'):
                        drone_data['multirotor'].cleanup()
                        
                del drone_sim_dict[node_id]
                print(f"Cleaned up drone simulation for node {node_id}")
                
            except Exception as e:
                print(f"Error during cleanup for node {node_id}: {e}")
                traceback.print_exc()
        
        OgnPegasusMultirotorNodeBase._set_initialized(node, False)

    @staticmethod
    def _print_stacktrace(db):
        """Print stacktrace for debugging"""
        stacktrace = traceback.format_exc().splitlines(keepends=True)
        stacktrace_iter = iter(stacktrace)
        stacktrace_output = ""

        for stacktrace_line in stacktrace_iter:
            if "OgnPegasusMultirotor" in stacktrace_line:
                # The stack trace shows that the exception originates from this file
                # Removing this useless information from the stack trace
                next(stacktrace_iter, None)
            else:
                stacktrace_output += stacktrace_line

        if hasattr(db, 'log_error'):
            db.log_error(stacktrace_output)
        else:
            print("Error:", stacktrace_output)

    @staticmethod
    def create_base_backend_config(db, backend_config_class):
        """Create base backend configuration from common inputs"""
        # This method will be overridden by specific backend implementations
        # since each backend has different configuration parameters
        raise NotImplementedError("This method should be overridden by specific backend nodes")

    @staticmethod
    def create_multirotor(db, backend):
        """Create multirotor vehicle with the given backend without validation for scalability"""
        # Create multirotor configuration
        multirotor_config = MultirotorConfig()
        multirotor_config.backends = [backend]

        # Get USD file from input parameter
        selected_usd_file = db.inputs.usdFile

        print(f"Creating multirotor with USD file: {selected_usd_file}")
        # Create multirotor vehicle directly without validation

        # get where the drone is on the stage
        drone_xform = XFormPrim(prim_path=db.inputs.dronePrim)
        position, orientation = drone_xform.get_world_pose()

        print(f"Multirotor initial position: {position}, orientation: {orientation}")

        # Initialize the "Robot" class
        # Note: we need to change the rotation to have qw first, because NVidia
        # does not keep a standard of quaternions inside its own libraries (not good, but okay)
        multirotor = Multirotor(
            stage_prefix=db.inputs.dronePrim,
            usd_file=selected_usd_file,  # Use USD file directly
            vehicle_id=0,  # This is used internally by multirotor, separate from backend vehicle_id
            init_pos=position,
            init_orientation=[orientation[1], orientation[2], orientation[3], orientation[0]], 
            config=multirotor_config,
            spawn_prim=False
        )
        print(f"Successfully created multirotor: {multirotor}")
        
        return multirotor, multirotor_config

    @staticmethod
    def compute_base(db, backend_class, backend_config_class, create_config_func) -> bool:
        """Base compute method that handles common drone spawning and execution"""
        global drone_sim_dict, timeline
        global _timeline_sub_play, _timeline_sub_pause, _timeline_sub_stop

        if not timeline:
            timeline = omni.timeline.get_timeline_interface()
            _timeline_sub_play = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PLAY), timeline_callback
            )
            _timeline_sub_pause = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PAUSE), timeline_callback
            )
            _timeline_sub_stop = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP), timeline_callback
            )

        # Check if the timeline is actually playing. If not, do NOT initialize anything.
        if not timeline.is_playing():
             # If the timeline is not playing, just propagate execution and exit
            if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
                db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return True

        # Guard: wait until the World's physics context is fully initialised.
        # In GUI mode with play-on-start, OnPlaybackTick can fire before the
        # physics scene is ready, causing add_physics_callback to crash.
        try:
            world = World.instance()
            if world is None or world.get_physics_context() is None:
                node_id = db.node.node_id()
                if node_id not in getattr(OgnPegasusMultirotorNodeBase, '_warned_no_world', set()):
                    if not hasattr(OgnPegasusMultirotorNodeBase, '_warned_no_world'):
                        OgnPegasusMultirotorNodeBase._warned_no_world = set()
                    OgnPegasusMultirotorNodeBase._warned_no_world.add(node_id)
                    reason = "World is None" if world is None else "physics context is None"
                    print(f"[Pegasus] compute_base skipping — {reason}. "
                          f"Waiting for extension to initialise World...")
                if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
                    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
                return True
        except Exception as e:
            node_id = db.node.node_id()
            if node_id not in getattr(OgnPegasusMultirotorNodeBase, '_warned_no_world', set()):
                if not hasattr(OgnPegasusMultirotorNodeBase, '_warned_no_world'):
                    OgnPegasusMultirotorNodeBase._warned_no_world = set()
                OgnPegasusMultirotorNodeBase._warned_no_world.add(node_id)
                print(f"[Pegasus] compute_base skipping — physics context error: {e}")
            if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
                db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return True

        try:
            # Check if we have drone prim input
            drone_prim_val = db.inputs.dronePrim
            if not drone_prim_val:
                # Log once per node so we can see that compute IS being called
                node_id = db.node.node_id()
                if node_id not in getattr(OgnPegasusMultirotorNodeBase, '_warned_empty', set()):
                    if not hasattr(OgnPegasusMultirotorNodeBase, '_warned_empty'):
                        OgnPegasusMultirotorNodeBase._warned_empty = set()
                    OgnPegasusMultirotorNodeBase._warned_empty.add(node_id)
                    print(f"[Pegasus] compute called but dronePrim is empty/falsy "
                          f"(value={drone_prim_val!r}).  Waiting for data nodes to resolve...")
            if drone_prim_val:
                node_id = db.node.node_id()
                
                # Initialize drone if not already done
                if node_id not in drone_sim_dict.keys():
                    print(f'Creating new drone with VEHICLE ID: {db.inputs.vehicleID}')
                    
                    # Create backend configuration using the provided function
                    backend_config = create_config_func(db)
                    backend = backend_class(config=backend_config)

                    # Create multirotor vehicle
                    multirotor, multirotor_config = OgnPegasusMultirotorNodeBase.create_multirotor(db, backend)

                    # Store drone simulation data
                    drone_sim_dict[node_id] = {
                        'backend_config': backend_config,
                        'backend': backend,
                        'multirotor_config': multirotor_config,
                        'multirotor': multirotor,
                    }
                    
                    print(f"Successfully created drone simulation for node {node_id}")
                    
        except Exception as e:
            print('Error in compute method:', e)
            traceback.print_exc()
            return False

        # Set outputs:execOut if not hidden
        if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
