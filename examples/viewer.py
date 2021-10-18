# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import math
import sys
import time
from typing import Any, Dict, List

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

from magnum import Vector2i, Vector3, gl
from magnum.platform.glfw import Application

import habitat_sim
from examples.settings import default_sim_settings, make_cfg


class HabitatSimInteractiveViewer(Application):
    def __init__(self, sim_settings: Dict[str, Any]) -> None:
        configuration = self.Configuration()
        configuration.title = "Habitat Sim Interactive Viewer"
        Application.__init__(self, configuration)
        self.sim_settings: Dict[str:Any] = sim_settings

        # set proper viewport size
        self.viewport_size: Vector2i = gl.default_framebuffer.viewport.size()
        self.sim_settings["width"] = self.viewport_size[0]
        self.sim_settings["height"] = self.viewport_size[1]

        # set up our movement map
        key = Application.KeyEvent.Key
        self.pressed = {
            key.UP: False,
            key.DOWN: False,
            key.LEFT: False,
            key.RIGHT: False,
            key.A: False,
            key.D: False,
            key.S: False,
            key.W: False,
            key.X: False,
            key.Z: False,
        }

        # set up our movement key bindings map
        key = Application.KeyEvent.Key
        self.key_to_action = {
            key.UP: "look_up",
            key.DOWN: "look_down",
            key.LEFT: "turn_left",
            key.RIGHT: "turn_right",
            key.A: "move_left",
            key.D: "move_right",
            key.S: "move_backward",
            key.W: "move_forward",
            key.X: "move_down",
            key.Z: "move_up",
        }

        # toggle mouse utility on/off
        self.mouse_interaction = True

        # toggle physics simulation on/off
        self.simulating = True

        # toggle a single simulation step at the next opportunity if not
        # simulating continuously.
        self.simulate_single_step = False

        # configure our simulator
        self.cfg: habitat_sim.simulator.Configuration = None
        self.sim: habitat_sim.simulator.Simulator = None
        self.reconfigure_sim()

        self.time_since_last_simulation = 0.0
        self.print_help_text()

    def draw_event(self) -> None:
        """
        Calls continuously to re-render frames and swap the two frame buffers
        at a fixed rate.
        """
        agent_acts_per_sec = 60.0

        gl.default_framebuffer.clear(
            gl.FramebufferClear.COLOR | gl.FramebufferClear.DEPTH
        )

        # Agent actions should occur at a fixed rate per second
        self.time_since_last_simulation += Timer.prev_frame_duration
        num_agent_actions: int = self.time_since_last_simulation * agent_acts_per_sec
        self.move_and_look(int(num_agent_actions))

        # Occasionally a frame will pass quicker than 1/60 seconds
        if self.time_since_last_simulation >= 1.0 / 60.0:
            if self.simulating or self.simulate_single_step:
                # step physics at a fixed rate
                # In the interest of frame rate, only a single step is taken,
                # even if time_since_last_simulation is quite large
                self.sim.step_world(1.0 / 60.0)
                self.simulate_single_step = False

            # reset time_since_last_simulation, accounting for potential overflow
            self.time_since_last_simulation = math.fmod(
                self.time_since_last_simulation, 1.0 / 60.0
            )

        self.sim._sensors["color_sensor"].draw_observation()
        self.render_camera.render_target.blit_rgba_to_default()
        gl.default_framebuffer.bind()

        self.swap_buffers()
        Timer.next_frame()
        self.redraw()

    def default_agent_config(self) -> habitat_sim.agent.AgentConfiguration:
        """
        Set up our own agent and agent controls
        """
        make_action_spec = habitat_sim.agent.ActionSpec
        make_actuation_spec = habitat_sim.agent.ActuationSpec
        MOVE, LOOK = 0.07, 0.9

        # all of our possible actions' names
        action_list = [
            "move_left",
            "turn_left",
            "move_right",
            "turn_right",
            "move_backward",
            "look_up",
            "move_forward",
            "look_down",
            "move_down",
            "move_up",
        ]

        action_space: Dict[str, habitat_sim.agent.ActionSpec] = {}

        # build our action space map
        for action in action_list:
            actuation_spec_amt = MOVE if "move" in action else LOOK
            action_spec = make_action_spec(
                action, make_actuation_spec(actuation_spec_amt)
            )
            action_space[action] = action_spec

        sensor_spec: List[habitat_sim.sensor.SensorSpec] = self.cfg.agents[
            self.agent_id
        ].sensor_specifications

        agent_config = habitat_sim.agent.AgentConfiguration(
            height=1.5,
            radius=0.1,
            sensor_specifications=sensor_spec,
            action_space=action_space,
            body_type="cylinder",
        )
        return agent_config

    def reconfigure_sim(self) -> None:
        """
        Utilizes the current `self.sim_settings` to configure and set up a new
        `habitat_sim.Simulator`, and then either starts a simulation instance, or replaces
        the current simulator instance, reloading the most recently loaded scene
        """
        # configure our sim_settings but then set the agent to our default
        self.cfg = make_cfg(self.sim_settings)
        self.agent_id: int = self.sim_settings["default_agent"]
        self.cfg.agents[self.agent_id] = self.default_agent_config()

        if self.sim is None:
            self.sim = habitat_sim.Simulator(self.cfg)

        else:  # edge case
            if self.sim.config.sim_cfg.scene_id == self.cfg.sim_cfg.scene_id:
                # we need to force a reset, so change the internal config scene name
                self.sim.config.sim_cfg.scene_id = "NONE"
            self.sim.reconfigure(self.cfg)

        self.active_scene_graph = self.sim.get_active_scene_graph()
        self.default_agent = self.sim.get_agent(self.agent_id)
        self.agent_body_node = self.default_agent.scene_node
        self.render_camera = self.agent_body_node.node_sensor_suite.get("color_sensor")

        Timer.start()
        self.step = -1

    def move_and_look(self, repetitions: int) -> None:
        """
        This method is called continuously with `self.draw_event` to monitor
        any changes in the movement keys map `Dict[KeyEvent.key, Bool]`.
        When a key in the map is set to `True` the corresponding action is taken.
        """
        key = Application.KeyEvent.Key
        agent = self.sim.agents[self.agent_id]
        press: Dict[key.key, bool] = self.pressed
        act: Dict[key.key, str] = self.key_to_action

        action_queue: List[str] = [act[k] for k, v in press.items() if v]

        for _ in range(int(repetitions)):
            [agent.act(x) for x in action_queue]

    def invert_gravity(self) -> None:
        """
        Sets the gravity vector to the negative of it's previous value. This is
        a good method for testing simulation functionality.
        """
        gravity: Vector3 = self.sim.get_gravity() * -1
        self.sim.set_gravity(gravity)

    def key_press_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key press by performing the corresponding functions.
        If the key pressed is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the
        key will be set to False for the next `self.move_and_look()` to update the current actions.
        """
        key = event.key
        pressed = Application.KeyEvent.Key

        if key == pressed.ESC:
            event.accepted = True
            self.exit_event(Application.ExitEvent)
            return

        elif key == pressed.H:
            self.print_help_text()

        elif key == pressed.SPACE:
            if not self.sim.config.sim_cfg.enable_physics:
                print("Warning: physics was not enabled during setup")
            else:
                self.simulating = not self.simulating
                print("Command: physics simulating set to ", self.simulating)

        elif key == pressed.PERIOD:
            if self.simulating:
                print("Warning: physic simulation already running")
            else:
                self.simulate_single_step = True
                print("Command: physics step taken")

        # TODO: In a future PR, a mouse GRAB interaction mode will be added
        #       and this key press will be used to toggle between modes
        elif key == pressed.M:
            self.mouse_interaction = not self.mouse_interaction
            print("Command: mouse LOOK set to ", self.mouse_interaction)

        elif key == pressed.R:
            self.reconfigure_sim()
            print("Command: simulator re-loaded")

        elif key == pressed.V:
            self.invert_gravity()
            print("Command: gravity inverted")

        # update map of moving/looking keys which are currently pressed
        if key in self.pressed:
            self.pressed[key] = True
        event.accepted = True
        self.redraw()

    def key_release_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key release. When a key is released, if it
        is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the key will
        be set to False for the next `self.move_and_look()` to update the current actions.
        """
        key = event.key

        # update map of moving/looking keys which are currently pressed
        if key in self.pressed:
            self.pressed[key] = False
        event.accepted = True
        self.redraw()

    def mouse_move_event(self, event: Application.MouseEvent) -> None:
        """
        Handles `Application.MouseEvent`. When in mouse interaction mode,
        enables the left mouse button to steer the agent's facing direction.
        """
        button = Application.MouseMoveEvent.Buttons

        # if interactive mode is True
        if event.buttons == button.LEFT and self.mouse_interaction:
            agent = self.sim.agents[self.agent_id]
            delta = event.relative_position
            action = habitat_sim.agent.ObjectControls()
            act_spec = habitat_sim.agent.ActuationSpec

            # left/right on agent scene node
            action(agent.scene_node, "turn_right", act_spec(delta.x))

            # up/down on cameras' scene nodes
            action = habitat_sim.agent.ObjectControls()
            sensors = list(self.agent_body_node.subtree_sensors.values())
            [action(s.object, "look_down", act_spec(delta.y), False) for s in sensors]

        self.redraw()
        event.accepted = True

    def exit_event(self, event: Application.ExitEvent):
        """
        Overrides exit_event to properly close the Simulator before exiting the
        application.
        """
        self.sim.close(destroy=True)
        event.accepted = True
        exit(0)

    def print_help_text(self) -> None:
        """
        Print the Key Command help text.
        """
        print(
            """
=====================================================
Welcome to the Habitat-sim Python Viewer application!
=====================================================
Key Commands:
-------------
    esc:        Exit the application.
    'h':        Display this help message.
    'm':        Toggle mouse interaction mode.

    Agent Controls:
    'wasd':     Move the agent's body forward/backward and left/right.
    'zx':       Move the agent's body up/down.
    arrow keys: Turn the agent's body left/right and camera look up/down.

    Utilities:
    'r':        Reset the simulator with the most recently loaded scene.

    Object Interactions:
    SPACE:      Toggle physics simulation on/off.
    '.':        Take a single simulation step if not simulating continuously.
    'v':        (physics) Invert gravity.
=====================================================
"""
        )


class Timer:
    """
    Timer class used to keep track of time between buffer swaps
    and guide the display frame rate.
    """

    start_time = 0.0
    prev_frame_time = 0.0
    prev_frame_duration = 0.0
    running = False

    @staticmethod
    def start() -> None:
        """
        Starts timer and resets previous frame time to the start time
        """
        Timer.running = True
        Timer.start_time = time.time()
        Timer.prev_frame_time = Timer.start_time
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def stop() -> None:
        """
        Stops timer and erases any previous time data, reseting the timer
        """
        Timer.running = False
        Timer.start_time = 0.0
        Timer.prev_frame_time = 0.0
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def next_frame() -> None:
        """
        Records previous frame duration and updates the previous frame timestamp
        to the current time. If the timer is not currently running, perform nothing.
        """
        if not Timer.running:
            return
        Timer.prev_frame_duration = time.time() - Timer.prev_frame_time
        Timer.prev_frame_time = time.time()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    # optional arguments
    parser.add_argument(
        "--scene",
        default="NONE",
        type=str,
        help='scene/stage file to load (default: "NONE")',
    )
    parser.add_argument(
        "--dataset",
        default="default",
        type=str,
        metavar="DATASET",
        help="dataset configuration file to use (default: default)",
    )
    parser.add_argument(
        "--disable_physics",
        action="store_true",
        help="disable physics simulation (default: False)",
    )

    args = parser.parse_args()

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["enable_physics"] = not args.disable_physics

    HabitatSimInteractiveViewer(sim_settings).exec()
