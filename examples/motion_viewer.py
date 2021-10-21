# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
from typing import Any, Dict

from magnum import Deg, Quaternion, Vector3, gl
from magnum.platform.glfw import Application
from viewer import HabitatSimInteractiveViewer, Timer

from examples.fairmotion_interface import FairmotionInterface
from examples.settings import default_sim_settings


class FairmotionSimInteractiveViewer(HabitatSimInteractiveViewer):
    def __init__(self, sim_settings: Dict[str, Any]) -> None:
        super().__init__(sim_settings)

        # fairmotion init
        self.fm_demo = FairmotionInterface(self)

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
                if self.fm_demo.motion is not None:
                    self.fm_demo.next_pose()
                    self.fm_demo.next_pose()

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

    def key_press_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key press by performing the corresponding functions.
        If the key pressed is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the
        key will be set to False for the next `self.move_and_look()` to update the current actions.
        """

        key = event.key
        pressed = Application.KeyEvent.Key

        # Everything below is used for testing
        if key == pressed.F:
            print("Command: fairmotion test")
            self.fm_demo.load_motion()
            self.fm_demo.load_model()

        elif key == pressed.I:
            r = self.fm_demo.rotation_offset
            print(f"R is {r}")
            x = int(input("Rx (deg) <- "))
            if x:
                self.fm_demo.rotation_offset = (
                    Quaternion.rotation(Deg(x), Vector3.x_axis()) * r
                )
            self.fm_demo.next_pose(repeat=True)

        elif key == pressed.J:
            t = self.fm_demo.translation_offset
            print(f"Y is {t[1]}")
            y = float(input("Y <- "))
            if y:
                t[1] = y
            self.fm_demo.next_pose(repeat=True)

        elif key == pressed.K:
            t = self.fm_demo.translation_offset
            print(f"Z is {t[2]}")
            z = float(input("Z <- "))
            if z:
                t[2] = z
            self.fm_demo.next_pose(repeat=True)
        # End of testing section

        super().key_press_event(event)


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

    FairmotionSimInteractiveViewer(sim_settings).exec()
