"""
| File: extension.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements the Pegasus_SimulatorExtension which omni.ext.IExt that is created when this class is enabled. In turn, this class initializes the extension widget.
"""

__all__ = ["Pegasus_SimulatorExtension"]

# Python garbage collenction and asyncronous API
import gc
import asyncio
from functools import partial
from threading import Timer

# Omniverse general API
import pxr
import carb
import omni.ext
import omni.usd
import omni.kit.ui
import omni.kit.app
import omni.ui as ui
import omni.timeline

from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.api import World

# Pegasus Extension Files and API
from pegasus.simulator.params import MENU_PATH, WINDOW_TITLE, DEFAULT_WORLD_SETTINGS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from pxr import Usd, UsdGeom

from carb.eventdispatcher import get_eventdispatcher

# Setting up the UI for the extension's Widget
from pegasus.simulator.ui.ui_window import WidgetWindow
from pegasus.simulator.ui.ui_delegate import UIDelegate

from isaacsim.core.utils.stage import (
    clear_stage,
    create_new_stage,
    create_new_stage_async,
    get_current_stage,
    set_stage_units,
    set_stage_up_axis,
    update_stage_async,
)


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Pegasus_SimulatorExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        carb.log_info("Pegasus Simulator is starting up")

        # Create the Pegasus interface that manages the simulation
        self.pg = PegasusInterface()

        # Import and register the OmniGraph nodes

        # Save the extension id
        self._ext_id = ext_id

        # Create the UI of the app and its manager
        self.ui_delegate = None
        self.ui_window = None

        # Add the ability to show the window if the system requires it (QuickLayout feature)
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, partial(self.show_window, None))

        # Add the extension to the editor menu inside isaac sim
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(
                MENU_PATH, self.show_window, toggle=True, value=True
            )

        # Show the window (It call the self.show_window)
        ui.Workspace.show_window(WINDOW_TITLE, show=True)

        # Subscribe to timeline events so we can re-apply lidar overrides
        # and reset ROS2Context nodes every time the user presses Play.
        timeline = omni.timeline.get_timeline_interface()
        self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline_event
        )

        # Subscribe to stage events
        print("Subscribing to stage events")

        stage_event_stream = omni.usd.get_context().get_stage_event_stream()

        # Create a subscription to the event stream
        # The subscription will last as long as the 'subscription' variable is in scope
        self.stage_subscription = stage_event_stream.create_subscription_to_pop(
            self.on_stage_event, name="my_stage_listener"
        )

    def _on_timeline_event(self, event):
        """Called on every timeline event.  On PLAY we reset every
        ROS2Context node so a fresh DDS participant is created (the old
        uint64 pointer is stale after a USD save/reload or Stop→Play cycle).
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            # ── Reset ROS2Context nodes ──────────────────────────────────
            try:
                import omni.graph.core as og
                for graph in og.get_all_graphs():
                    for node in graph.get_nodes():
                        if node.get_node_type().get_node_type() == "isaacsim.ros2.bridge.ROS2Context":
                            # Dirty the node so it recomputes a fresh DDS participant on next evaluation.
                            env_attr = node.get_attribute("inputs:useDomainIDEnvVar")
                            if env_attr and env_attr.is_valid():
                                old_val = og.Controller.get(env_attr)
                                og.Controller.set(env_attr, not old_val)
                                og.Controller.set(env_attr, old_val)
                            carb.log_info(
                                f"[AirStack] Reset ROS2Context '{node.get_prim_path()}' for fresh DDS participant"
                            )
            except Exception as e:
                carb.log_warn(f"[AirStack] Failed to reset ROS2Context nodes on play: {e}")

    def on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            # Cancel any in-flight initialisation from a previous stage-open
            # (e.g. empty stage → saved USD fires two events back-to-back).
            if hasattr(self, "_init_task") and self._init_task is not None:
                if not self._init_task.done():
                    self._init_task.cancel()
                    carb.log_warn("Cancelled previous world-init task (new stage opened).")
                self._init_task = None

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return

            real_path = stage.GetRootLayer().realPath
            if not real_path:
                # This extension should stay out of the way of 
                # other launching methods that create their own World.
                print("USD Stage opened: (anonymous stage, skipping world init)")
                return

            print("USD Stage opened:", real_path)
            self._init_task = asyncio.ensure_future(
                self.set_current_world_to_pegasus_with_physics()
            )
    
    
    async def _wait_for_stage_loading(self, timeout_frames: int = 200) -> bool:
        """Poll until the USD stage has finished loading all assets."""
        app = omni.kit.app.get_app()
        ctx = omni.usd.get_context()
        for _ in range(timeout_frames):
            await app.next_update_async()
            _, _, is_loading = ctx.get_stage_loading_status()
            if not is_loading:
                return True
        carb.log_warn("Timed out waiting for stage to finish loading")
        return False

    async def _wait_for_world_ready(self, timeout_frames: int = 200) -> bool:
        """Poll until World.instance() returns a valid World with a physics context."""
        app = omni.kit.app.get_app()
        for _ in range(timeout_frames):
            await app.next_update_async()
            try:
                w = World.instance()
                if w is not None and w.get_physics_context() is not None:
                    return True
            except Exception:
                pass
        carb.log_warn("Timed out waiting for World to become ready")
        return False

    async def set_current_world_to_pegasus_with_physics(self):
        app = omni.kit.app.get_app()
        timeline = omni.timeline.get_timeline_interface()

        # Wait until the stage has finished loading all assets (textures, MDLs,
        # sub-USDs, etc.) before doing any heavy initialisation.
        await self._wait_for_stage_loading()

        if asyncio.current_task() and asyncio.current_task().cancelled():
            return

        # Remember whether the timeline was playing before we touch the World.
        # World() / initialize_world() internally stops the timeline, so we
        # need to restore playback state after initialisation completes.
        was_playing = timeline.is_playing()

        self.pg._world = World.instance()

        # if the world was None even after we set it, then initialize it
        if self.pg.world is None:
            carb.log_warn("The world is None")
            self.pg._world_settings = DEFAULT_WORLD_SETTINGS

            # initialize_world() does heavy synchronous work that briefly blocks
            # the asyncio loop.
            loop = asyncio.get_event_loop()
            orig_handler = loop.get_exception_handler()
            loop.set_exception_handler(lambda l, ctx: None)

            await app.next_update_async()
            await self.pg.initialize_world()

            loop.set_exception_handler(orig_handler)

            # Wait until the newly created World is fully ready.
            await self._wait_for_world_ready()

            carb.log_info("PEGASUS WORLD INITIALIZED")
            print("PEGASUS WORLD INITIALIZED", self.pg._world)
        # else if there was a world but it was missing the physics, then add physics
        else:
            try:
                physics_ctx = self.pg.world.get_physics_context()
            except Exception:
                physics_ctx = None
            if physics_ctx is None:

                # add physics to the world with synchronous call
                loop = asyncio.get_event_loop()
                orig_handler = loop.get_exception_handler()
                loop.set_exception_handler(lambda l, ctx: None)

                await app.next_update_async()
                await self.pg.world.initialize_simulation_context_async()

                loop.set_exception_handler(orig_handler)

                # wait until the world is ready
                await self._wait_for_world_ready()

        # Restore timeline playback if it was playing before World init stopped it
        if was_playing and not timeline.is_playing():
            carb.log_info("Restoring timeline playback after World initialisation")
            timeline.play()

        if self.pg.world is None:
            carb.log_warn("World still None after initialisation — skipping settings check")
            return

        # Check if the world settings match the required ones
        try:
            _phys_ctx = self.pg.world.get_physics_context()
        except Exception:
            _phys_ctx = None
        physics_dt_mismatch = (
            _phys_ctx is not None
            and self.pg.world.get_physics_dt() != self.pg._world_settings["physics_dt"]
        )
        rendering_dt_mismatch = (
            self.pg.world.get_rendering_dt() != self.pg._world_settings["rendering_dt"]
        )
        units_mismatch = (
            UsdGeom.GetStageMetersPerUnit(self.pg.world.stage)
            != self.pg._world_settings["stage_units_in_meters"]
        )

        if physics_dt_mismatch or rendering_dt_mismatch or units_mismatch:
            await self._show_settings_warning_popup(physics_dt_mismatch, rendering_dt_mismatch, units_mismatch)

    async def _show_settings_warning_popup(self, physics_dt_mismatch, rendering_dt_mismatch, units_mismatch):
        """Show a popup warning about mismatched world settings."""
        
        def on_update_settings():
            """Callback to update the world settings to match Pegasus requirements."""
            if physics_dt_mismatch or rendering_dt_mismatch:
                carb.log_info("Updating the physics and rendering dt of the world")
                self.pg.world.set_simulation_dt(
                    physics_dt=self.pg._world_settings["physics_dt"], 
                    rendering_dt=self.pg._world_settings["rendering_dt"]
                )
            
            if units_mismatch:
                carb.log_info("Updating the stage units")
                set_stage_units(self.pg.world.stage, self.pg._world_settings["stage_units_in_meters"])
            
            popup_window.visible = False
            # Show confirmation popup
            self._show_settings_confirmation_popup("updated")
        
        def on_keep_current():
            """Callback to keep current settings."""
            carb.log_info("Keeping current world settings")
            popup_window.visible = False
            # Show confirmation popup
            self._show_settings_confirmation_popup("kept")
        
        def on_close():
            """Callback when popup is closed."""
            popup_window.visible = False

        # Create the popup window
        popup_window = ui.Window(
            "World Settings Mismatch", 
            width=500, 
            height=400,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_MODAL
        )
        
        with popup_window.frame:
            with ui.VStack(spacing=10):
                ui.Label("Warning: World settings don't match Pegasus recommended settings!", 
                        style={"color": ui.color.yellow, "font_size": 16})
                
                ui.Spacer(height=5)
                
                # Show specific mismatches
                if physics_dt_mismatch:
                    ui.Label(f"Physics dt: Current = {self.pg.world.get_physics_dt():.6f}, "
                            f"Recommended = {self.pg._world_settings['physics_dt']:.6f}")
                
                if rendering_dt_mismatch:
                    ui.Label(f"Rendering dt: Current = {self.pg.world.get_rendering_dt():.6f}, "
                            f"Recommended = {self.pg._world_settings['rendering_dt']:.6f}")
                
                if units_mismatch:
                    current_units = UsdGeom.GetStageMetersPerUnit(self.pg.world.stage)
                    required_units = self.pg._world_settings["stage_units_in_meters"]
                    ui.Label(f"Stage units: Current = {current_units}, Recommended = {required_units}")
                
                ui.Spacer(height=5)
                
                ui.Label("How would you like to proceed?", style={"font_size": 14})
                
                ui.Spacer(height=5)
                
                # Buttons
                with ui.HStack(spacing=5):
                    ui.Spacer()
                    
                    update_btn = ui.Button("Update to Pegasus Settings", 
                                         clicked_fn=on_update_settings,
                                         style={"background_color": 0xFF4CAF50})
                    update_btn.width = ui.Pixel(180)
                    
                    keep_btn = ui.Button("Keep Current Settings", 
                                       clicked_fn=on_keep_current,
                                       style={"background_color": 0xFF2196F3})
                    keep_btn.width = ui.Pixel(150)
                    
                    ui.Spacer()
        
        popup_window.set_visibility_changed_fn(lambda visible: on_close() if not visible else None)
        popup_window.visible = True

    def _show_settings_confirmation_popup(self, action):
        """Show a confirmation popup with the user's choice and manual override instructions."""
        
        def on_ok():
            """Callback to close the confirmation popup."""
            confirmation_window.visible = False
        
        # Create the confirmation window
        confirmation_window = ui.Window(
            "Settings Updated" if action == "updated" else "Settings Preserved", 
            width=450, 
            height=320,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_MODAL
        )
        
        with confirmation_window.frame:
            with ui.VStack(spacing=10):
                # Success message based on action
                if action == "updated":
                    ui.Label("World settings have been updated to Pegasus recommendations!", 
                            style={"color": 0xFF4CAF50, "font_size": 16})
                else:
                    ui.Label("Current world settings have been preserved.", 
                            style={"color": 0xFF2196F3, "font_size": 16})
                
                ui.Spacer(height=5)
                
                # Information about manual override
                ui.Label("You can manually change these settings at any time by:", 
                        style={"font_size": 14})
                
                ui.Spacer(height=5)
                
                with ui.VStack(spacing=3):
                    ui.Label("1. Opening the Stage panel", style={"font_size": 14, "margin_left": 15})
                    ui.Label("2. Navigating to /World/PhysicsScene", style={"font_size": 14, "margin_left": 15})
                    ui.Label("3. Viewing the Property panel", style={"font_size": 14, "margin_left": 15})
                    ui.Label("4. Adjusting physics and rendering settings", style={"font_size": 14, "margin_left": 15})
                ui.Spacer(height=5)

                # OK button
                with ui.HStack():
                    ui.Spacer()
                    
                    ok_btn = ui.Button("OK", 
                                     clicked_fn=on_ok,
                                     style={"background_color": 0xFF607D8B})
                    ok_btn.width = ui.Pixel(80)
                    
                    ui.Spacer()
        
        confirmation_window.visible = True
        
        # Auto-close after 10 seconds
        def auto_close():
            if confirmation_window.visible:
                confirmation_window.visible = False
        
        Timer(10.0, auto_close).start()

    def show_window(self, menu, show):
        """
        Method that controls whether a widget window is created or not
        """

        if show == True:

            # Create a window and its delegate
            self.ui_delegate = UIDelegate()
            self.ui_window = WidgetWindow(self.ui_delegate)
            self.ui_window.set_visibility_changed_fn(self._visibility_changed_fn)

        # If we have a window and we are not supposed to show it, then change its visibility
        elif self.ui_window:
            self.ui_window.visible = False

    def _visibility_changed_fn(self, visible):
        """
        This method is invoked when the user pressed the "X" to close the extension window
        """

        # Update the Isaac sim menu visibility
        self._set_menu(visible)

        if not visible:
            # Destroy the window, because we create a new one in the show window method
            asyncio.ensure_future(self._destroy_window_async())

    def _set_menu(self, visible):
        """
        Method that updates the isaac sim ui menu to create the Widget window on and off
        """
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.set_value(MENU_PATH, visible)

    async def _destroy_window_async(self):

        # Wait one frame before it gets destructed (from NVidia example)
        await omni.kit.app.get_app().next_update_async()

        # Destroy the window UI if it exists
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

    def on_shutdown(self):
        """
        Callback called when the extension is shutdown
        """
        carb.log_info("Pegasus Isaac extension shutdown")

        # Destroy the isaac sim menu object
        self._menu = None

        # Destroy the window
        if self.ui_window:
            self.ui_window.destroy()
            self.ui_window = None

        # Destroy the UI delegate
        if self.ui_delegate:
            self.ui_delegate = None

        # De-register the function taht shows the window from the isaac sim ui
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, None)

        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.remove_item(MENU_PATH)

        # Call the garbage collector
        gc.collect()
