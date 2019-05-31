# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

r"""
This is a demonstration of how to create an agent with
two RGB cameras in a stereo pair
"""

import random

import numpy as np

import habitat_sim

cv2 = None


def _render(sim, display, depth=False):
    for _ in range(100):
        obs = sim.step("turn_right")

        if display:
            stereo_pair = np.concatenate(
                [obs["left_sensor"], obs["right_sensor"]], axis=1
            )

            if depth:
                stereo_pair = np.clip(stereo_pair, 0, 10)
                stereo_pair /= 10.0

            cv2.imshow("stereo_pair", stereo_pair)
            k = cv2.waitKey()
            if k == ord("q"):
                break


def main(display=True):
    global cv2
    if display:
        import cv2 as _cv2

        cv2 = _cv2

        cv2.namedWindow("stereo_pair")

    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = (
        "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )

    left_rgb_sensor = habitat_sim.SensorSpec()
    left_rgb_sensor.uuid = "left_sensor"
    left_rgb_sensor.resolution = [256, 256]
    left_rgb_sensor.position = 1.5 * habitat_sim.geo.UP + 0.5 * habitat_sim.geo.LEFT

    right_rgb_sensor = habitat_sim.SensorSpec()
    right_rgb_sensor.uuid = "right_sensor"
    right_rgb_sensor.resolution = [256, 256]
    right_rgb_sensor.position = 1.5 * habitat_sim.geo.UP + 0.5 * habitat_sim.geo.RIGHT

    agent_config = habitat_sim.AgentConfiguration()
    agent_config.sensor_specifications = [left_rgb_sensor, right_rgb_sensor]

    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))

    _render(sim, display)
    sim.close()
    del sim

    left_depth_sensor = habitat_sim.SensorSpec()
    left_depth_sensor.uuid = "left_sensor"
    left_depth_sensor.resolution = [256, 256]
    left_depth_sensor.position = 1.5 * habitat_sim.geo.UP + 0.5 * habitat_sim.geo.LEFT
    left_depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH

    right_depth_sensor = habitat_sim.SensorSpec()
    right_depth_sensor.uuid = "right_sensor"
    right_depth_sensor.resolution = [256, 256]
    right_depth_sensor.position = 1.5 * habitat_sim.geo.UP + 0.5 * habitat_sim.geo.RIGHT
    right_depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH

    agent_config = habitat_sim.AgentConfiguration()
    agent_config.sensor_specifications = [left_depth_sensor, right_depth_sensor]

    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))

    _render(sim, display, depth=True)
    sim.close()
    del sim


if __name__ == "__main__":
    main(display=True)
