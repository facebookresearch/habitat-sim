# [import]
import math

import magnum as mn
import numpy as np
from matplotlib import pyplot as plt

import habitat_sim
from habitat_sim.gfx import LightInfo, LightPositionModel
from habitat_sim.utils.common import quat_from_angle_axis, quat_to_magnum

# [/import]

# [helpers]
def show_img(data):
    plt.figure(figsize=(12, 12))
    plt.imshow(data, interpolation="nearest")
    plt.show(block=False)
    plt.pause(1)


def get_obs(sim, show):
    obs = sim.get_sensor_observations()["rgba_camera"]
    if show:
        show_img(obs)
    return obs


def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


x_axis = np.array([1, 0, 0])
z_axis = np.array([0, 0, 1])
y_axis = np.array([0, 1, 0])

# [/helpers]

# This is wrapped in a such that it can be added to a unit test
def main(show_imgs=True):
    # [initialization]
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    backend_cfg.enable_physics = True

    # agent configuration
    sensor_cfg = habitat_sim.SensorSpec()
    sensor_cfg.resolution = [1080, 960]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_cfg]

    # create the simulator
    sim_cfg = habitat_sim.Configuration(backend_cfg, [agent_cfg])
    sim = habitat_sim.Simulator(sim_cfg)

    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [5.0, 0.0, 1.0]
    agent_state.rotation = quat_from_angle_axis(
        math.radians(70), y_axis
    ) * quat_from_angle_axis(math.radians(-20), x_axis)
    agent = sim.initialize_agent(0, agent_state)
    agent_transform = agent.scene_node.transformation_matrix()

    # [/initialization]

    # [example 1]
    get_obs(sim, show_imgs)

    # [/example 1]

    # [example 2]
    id_1 = sim.add_object(1)
    sim.set_translation(agent_transform.transform_point([0.3, 0.9, -1.8]), id_1)

    get_obs(sim, show_imgs)

    # [/example 2]

    # [example 3]
    my_default_lighting = [
        LightInfo(position=[2.0, 2.0, 1.0], model=LightPositionModel.CAMERA)
    ]

    sim.set_light_setup(my_default_lighting)

    get_obs(sim, show_imgs)

    # [/example 3]

    # [example 4]
    id_2 = sim.add_object(2)
    sim.set_rotation(mn.Quaternion.rotation(mn.Deg(80), mn.Vector3.y_axis()), id_2)
    sim.set_translation(agent_transform.transform_point([-0.6, 0.9, -1.5]), id_2)

    get_obs(sim, show_imgs)

    # [/example 4]

    # [example 5]
    light_setup_2 = [
        LightInfo(position=[8.0, 1.5, 0.0], model=LightPositionModel.GLOBAL)
    ]
    sim.set_light_setup(light_setup_2, "my_custom_lighting")

    # [/example 5]

    # [example 6]
    remove_all_objects(sim)

    id_1 = sim.add_object(2, light_setup_key="my_custom_lighting")
    sim.set_rotation(
        mn.Quaternion.rotation(mn.Deg(90), mn.Vector3.x_axis())
        * mn.Quaternion.rotation(mn.Deg(-115), mn.Vector3.z_axis()),
        id_1,
    )
    sim.set_translation(agent_transform.transform_point([-0.8, 1.05, -1.5]), id_1)

    id_2 = sim.add_object(2, light_setup_key="my_custom_lighting")
    sim.set_rotation(
        mn.Quaternion.rotation(mn.Deg(90), mn.Vector3.x_axis())
        * mn.Quaternion.rotation(mn.Deg(-50), mn.Vector3.z_axis()),
        id_2,
    )
    sim.set_translation(agent_transform.transform_point([1.0, 1.05, -1.75]), id_2)

    get_obs(sim, show_imgs)

    # [/example 6]

    # [example 7]
    existing_light_setup = sim.get_light_setup("my_custom_lighting")

    # [/example 7]

    # [example 8]
    new_light_setup = existing_light_setup + [
        LightInfo(
            position=[0.0, 0.0, 0.0],
            color=[0.8, 0.8, 0.7],
            model=LightPositionModel.CAMERA,
        )
    ]
    sim.set_light_setup(new_light_setup, "my_custom_lighting")

    get_obs(sim, show_imgs)

    # [/example 8]

    # [example 9]
    sim.set_object_light_setup(id_1, habitat_sim.gfx.DEFAULT_LIGHTING_KEY)

    get_obs(sim, show_imgs)

    # [/example 9]


if __name__ == "__main__":
    main(show_imgs=True)
