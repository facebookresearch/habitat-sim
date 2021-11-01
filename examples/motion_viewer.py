# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import sys
from enum import Enum
from typing import Any, Callable, Dict, Optional

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

from magnum.platform.glfw import Application
from viewer import HabitatSimInteractiveViewer

from examples.fairmotion_interface import FairmotionInterface
from examples.settings import default_sim_settings
from habitat_sim.logging import logger


class FairmotionSimInteractiveViewer(HabitatSimInteractiveViewer):
    def __init__(
        self, sim_settings: Dict[str, Any], fm_settings: Dict[str, Any]
    ) -> None:
        super().__init__(sim_settings)

        # fairmotion init
        self.fm_demo = FairmotionInterface(
            self.sim,
            metadata_name="fm_demo",
            amass_path=fm_settings["amass_path"],
            metadata_dir=fm_settings["metadata_dir"],
        )

    def draw_event(self, simulation_call: Optional[Callable] = None) -> None:
        """
        Calls continuously to re-render frames and swap the two frame buffers
        at a fixed rate. Use `simulation_call` to perform method calls during
        a simulation step.
        """

        def play_motion() -> None:
            if self.fm_demo.motion is not None:
                self.fm_demo.next_pose()
                self.fm_demo.next_pose()

        super().draw_event(simulation_call=play_motion)

    def key_press_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key press by performing the corresponding functions.
        If the key pressed is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the
        key will be set to False for the next `self.move_and_look()` to update the current actions.
        """

        key = event.key
        pressed = Application.KeyEvent.Key
        mod = Application.InputEvent.Modifier

        if key == pressed.F:
            if event.modifiers == mod.SHIFT:
                self.fm_demo.hide_model()
                logger.info("Command: hide model")
            else:
                logger.info("Command: load model")
                self.fm_demo.load_model()

        elif key == pressed.K:
            # Toggle Key Frames
            self.fm_demo.cycle_model_previews()

        elif key == pressed.SLASH:
            # Toggle reverse direction of motion
            self.fm_demo.is_reversed = not self.fm_demo.is_reversed

        elif key == pressed.M:
            # cycle through mouse modes
            self.cycle_mouse_mode()
            logger.info(f"Command: mouse mode set to {self.mouse_interaction}")
            return

        elif key == pressed.R:
            super().reconfigure_sim()
            self.fm_demo = FairmotionInterface(self, metadata_name="fm_demo")
            logger.info("Command: simulator re-loaded")

        super().key_press_event(event)

    def cycle_mouse_mode(self):
        """
        Cycles through mouse modes that belong to the MouseMode emun.
        """
        self.mouse_interaction = MouseMode(
            (self.mouse_interaction.value + 1) % len(MouseMode)
        )

    def print_help_text(self) -> None:
        """
        Print the Key Command help text.
        """
        logger.info(
            """
=========================================================
Welcome to the Habitat-sim Fairmotion Viewer application!
=========================================================
Key Commands:
-------------
    esc:        Exit the application.
    'h':        Display this help message.
    'm':        Cycle through mouse mode.

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

    Fairmotion Interface:
    'f':        Load model with current motion data.
                [shft] Hide model.
    'k':        Toggle key frame preview of loaded motion.
    '/':        Set motion to play in reverse.
=========================================================
"""
        )


class MouseMode(Enum):
    LOOK = 0
    GRAB = 1
    MOTION = 3


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
    parser.add_argument(
        "--amass_path",
        type=str,
        help="amass motion file path to load motion from (default: None)",
    )
    parser.add_argument(
        "--metadata_dir",
        type=str,
        help="directory where metadata files should be saved to (default: None)",
    )
    parser

    args = parser.parse_args()

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["enable_physics"] = not args.disable_physics

    fm_settings: Dict[str, Any] = {}
    fm_settings["amass_path"] = args.amass_path
    fm_settings["metadata_dir"] = args.metadata_dir

    FairmotionSimInteractiveViewer(sim_settings, fm_settings).exec()
