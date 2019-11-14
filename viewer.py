
import cv2
import numpy as np

import habitat_sim

np.random.seed(10)
FORWARD_KEY = "w"
LEFT_KEY = "a"
RIGHT_KEY = "d"
DONE_KEY = "q"

def transform_rgb_bgr(image):
    return image[:, :, [2, 1, 0]]


def example():
    agent_cfg = habitat_sim.AgentConfiguration()
    agent_cfg.sensor_specifications[0].resolution = [256, 256]
    agent_cfg.sensor_specifications[0].position = [0.0, 1.0, 0.0]
    agent_cfg.sensor_specifications[0].uuid = "depth_camera"
    agent_cfg.sensor_specifications[0].sensor_type = habitat_sim.SensorType.COLOR
    agent_cfg.sensor_specifications[0].noise_model_kwargs = dict()
    rgb_spec = habitat_sim.SensorSpec()
    rgb_spec.resolution = [1024, 1024]
    rgb_spec.position = [0, 1, 0]
    rgb_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_spec.noise_model = "SpeckleNoiseModel"
    rgb_spec.uuid = "rgba_camera"
    agent_cfg.sensor_specifications.append(rgb_spec)

    agent_cfg.action_space["look_left"] = habitat_sim.ActionSpec(
        "look_left", habitat_sim.ActuationSpec(10.0)
    )
    agent_cfg.action_space["look_right"] = habitat_sim.ActionSpec(
        "look_right", habitat_sim.ActuationSpec(10.0)
    )

    agent_cfg.action_space["move_left"] = habitat_sim.ActionSpec(
        "move_left", habitat_sim.ActuationSpec(0.25)
    )
    agent_cfg.action_space["move_right"] = habitat_sim.ActionSpec(
        "move_right", habitat_sim.ActuationSpec(0.25)
    )

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene.id = (
        "./scene-data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )
    sim = habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))

    state = habitat_sim.AgentState()
    state.position = sim.pathfinder.get_random_navigable_point()
    state.rotation = np.quaternion(-0.31170254945755, 0, -0.950179815292358, 0)
    sim.get_agent(0).state = state

    obs = sim.get_sensor_observations()
    obs["depth_camera"] = np.clip(obs["depth_camera"], 0, 10)

    print("Agent stepping around inside environment.")

    while True:
        cv2.imshow("RGB", transform_rgb_bgr(obs["rgba_camera"]))
        k = cv2.waitKey()

        if k == ord(FORWARD_KEY):
            act = "move_forward"
        elif k == ord(LEFT_KEY):
            act = "turn_left"
        elif k == ord(RIGHT_KEY):
            act = "turn_right"
        elif k == ord("l"):
            act = "move_left"
        elif k == ord("r"):
            act = "move_right"
        elif k == ord(DONE_KEY):
            break
        else:
            continue

        print(sim.get_agent(0).state.position)

        obs = sim.step(act)


if __name__ == "__main__":
    example()
