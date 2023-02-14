# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import numpy as np

import habitat_sim

cv2 = None


# Helper function to render observations from the stereo agent
def _render(sim, display, depth=False):
    for _ in range(100):
        # Just spin in a circle
        obs = sim.step("turn_right")
        # Put the two stereo observations next to eachother
        stereo_pair = np.concatenate([obs["left_sensor"], obs["right_sensor"]], axis=1)

        # If it is a depth pair, manually normalize into [0, 1]
        # so that images are always consistent
        if depth:
            stereo_pair = np.clip(stereo_pair, 0, 10)
            stereo_pair /= 10.0

        # If in RGB/RGBA format, change first to RGB and change to BGR
        if len(stereo_pair.shape) > 2:
            stereo_pair = stereo_pair[..., 0:3][..., ::-1]

        # display=False is used for the smoke test
        if display:
            cv2.imshow("stereo_pair", stereo_pair)
            k = cv2.waitKey()
            if k == ord("q"):
                break


def main(display=True):
    global cv2
    # Only import cv2 if we are doing to display
    if display:
        import cv2 as _cv2

        cv2 = _cv2

        cv2.namedWindow("stereo_pair")

    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = (
        "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )

    # First, let's create a stereo RGB agent
    left_rgb_sensor = habitat_sim.bindings.CameraSensorSpec()
    # Give it the uuid of left_sensor, this will also be how we
    # index the observations to retrieve the rendering from this sensor
    left_rgb_sensor.uuid = "left_sensor"
    left_rgb_sensor.resolution = [512, 512]
    # The left RGB sensor will be 1.5 meters off the ground
    # and 0.25 meters to the left of the center of the agent
    left_rgb_sensor.position = 1.5 * habitat_sim.geo.UP + 0.25 * habitat_sim.geo.LEFT

    # Same deal with the right sensor
    right_rgb_sensor = habitat_sim.CameraSensorSpec()
    right_rgb_sensor.uuid = "right_sensor"
    right_rgb_sensor.resolution = [512, 512]
    # The right RGB sensor will be 1.5 meters off the ground
    # and 0.25 meters to the right of the center of the agent
    right_rgb_sensor.position = 1.5 * habitat_sim.geo.UP + 0.25 * habitat_sim.geo.RIGHT

    agent_config = habitat_sim.AgentConfiguration()
    # Now we simly set the agent's list of sensor specs to be the two specs for our two sensors
    agent_config.sensor_specifications = [left_rgb_sensor, right_rgb_sensor]

    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))

    _render(sim, display)
    sim.close()

    # Now let's do the exact same thing but for a depth camera stereo pair!
    left_depth_sensor = habitat_sim.CameraSensorSpec()
    left_depth_sensor.uuid = "left_sensor"
    left_depth_sensor.resolution = [512, 512]
    left_depth_sensor.position = 1.5 * habitat_sim.geo.UP + 0.25 * habitat_sim.geo.LEFT
    # The only difference is that we set the sensor type to DEPTH
    left_depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH

    right_depth_sensor = habitat_sim.CameraSensorSpec()
    right_depth_sensor.uuid = "right_sensor"
    right_depth_sensor.resolution = [512, 512]
    right_depth_sensor.position = (
        1.5 * habitat_sim.geo.UP + 0.25 * habitat_sim.geo.RIGHT
    )
    # The only difference is that we set the sensor type to DEPTH
    right_depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH

    agent_config = habitat_sim.AgentConfiguration()
    agent_config.sensor_specifications = [left_depth_sensor, right_depth_sensor]

    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))

    _render(sim, display, depth=True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.set_defaults(display=True)
    args = parser.parse_args()
    main(display=args.display)
