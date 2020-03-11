import random

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim

# from habitat_sim.utils.data.data_extractor import ImageExtractor
# import matplotlib.pyplot as plt

# def display_sample(sample):
#     img = sample['rgba']
#     depth = sample['depth']

#     arr = [img, depth]
#     titles = ['rgba', 'depth']
#     plt.figure(figsize=(12 ,8))
#     for i, data in enumerate(arr):
#         ax = plt.subplot(1, 3, i+1)
#         ax.axis('off')
#         ax.set_title(titles[i])
#         plt.imshow(data)

#     plt.show()


# scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
# extractor = ImageExtractor(scene, output=["rgba", "depth"], shuffle=False, extraction_method="panorama")

# for sample in extractor[21:28]:
#     display_sample(sample)

#test_scene = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
test_scene = "/datasets01/mp3d/073118/v1/habitat/17DRP5sb8fy/17DRP5sb8fy.glb"

sim_settings = {
    "width": 256,  # Spatial resolution of the observations
    "height": 256,
    "scene": test_scene,  # Scene path
    "default_agent": 0,
    "sensor_height": 1.5,  # Height of sensors in meters
    "color_sensor": True,  # RGB sensor
    "semantic_sensor": True,  # Semantic sensor
    "depth_sensor": True,  # Depth sensor
    "triangle_sensor": True,
    "seed": 1,
}


def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene.id = settings["scene"]

    # Note: all sensors must have the same resolution
    sensors = {
        "color_sensor": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "depth_sensor": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "semantic_sensor": {
            "sensor_type": habitat_sim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "triangle_sensor": {
            "sensor_type": habitat_sim.SensorType.TRIANGLE,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            sensor_spec = habitat_sim.SensorSpec()
            sensor_spec.uuid = sensor_uuid
            sensor_spec.sensor_type = sensor_params["sensor_type"]
            sensor_spec.resolution = sensor_params["resolution"]
            sensor_spec.position = sensor_params["position"]

            sensor_specs.append(sensor_spec)

    # Here you can specify the amount of displacement in a forward action and the turn angle
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left", habitat_sim.agent.ActuationSpec(amount=30.0)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=30.0)
        ),
    }

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


cfg = make_cfg(sim_settings)
sim = habitat_sim.Simulator(cfg)


# Set agent state
agent = sim.initialize_agent(sim_settings["default_agent"])
agent_state = habitat_sim.AgentState()
agent_state.position = np.array([2.4, 0.072447, 6.0])
agent.set_state(agent_state)


total_frames = 0
max_frames = 2
action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())

while total_frames < max_frames:
    action = random.choice(action_names)
    print("action", action)
    observations = sim.step(action)
    rgb = observations["color_sensor"]
    semantic = observations["semantic_sensor"]
    depth = observations["depth_sensor"]
    triangle = observations["triangle_sensor"]

    print(f"triangle: ")
    print(f"shape: {triangle.shape}, unique: {np.unique(triangle)}\n")
    print(f"semantic: ")
    print(f"shape: {semantic.shape}, unique: {np.unique(semantic)}\n")

    total_frames += 1

    # plt.imshow(rgb)
    # plt.show()

print("done!")
