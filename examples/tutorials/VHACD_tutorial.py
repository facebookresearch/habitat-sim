# [setup]
import math
import os

import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils import viz_utils as vut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "VHACD_tutorial_output/")

if not os.path.exists(output_path):
    os.mkdir(output_path)

# Define Helper Functions
def remove_all_objects(sim):
    for id_ in sim.get_existing_object_ids():
        sim.remove_object(id_)


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "NONE"

    backend_cfg.enable_physics = True

    # sensor configurations
    sensor_specs = []
    sensor_spec = habitat_sim.SensorSpec()
    sensor_spec.uuid = "rgba_camera_1stperson"
    sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    sensor_spec.resolution = [544, 720]
    sensor_specs.append(sensor_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def simulate(sim, dt=1.0, get_frames=True, data=None):
    """Simulate dt seconds at 60Hz to the nearest fixed timestep"""
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())
    if "observations" in data:
        data["observations"] += observations


def_offset = np.array([0, 1, -1.5])
def_orientation = mn.Quaternion(((0, 0, 0), 1))


def set_object_state_from_agent(
    sim,
    ob_id,
    offset=def_offset,
    orientation=def_orientation,
):
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    ob_translation = agent_transform.transform_point(offset)
    sim.set_translation(ob_translation, ob_id)
    sim.set_rotation(orientation, ob_id)


# This tutorial walks through how to use VHACD and the time optimizations it can provide.
# VHACD is a library for running convex hull decomposition on a given mesh, which can produce significant physics optimizations


def runVHACDSimulation(obj_path):
    # data will store data from simulation
    data = {
        "observations": [],
    }

    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        sim.initialize_agent(
            0,
            habitat_sim.AgentState(
                [5.45, 2.4, 1.2],
                np.quaternion(1, 0, 0, 0),
            ),
        )

        # get the physics object attributes manager
        obj_templates_mgr = sim.get_object_template_manager()

        # create a list that will store the object ids
        obj_ids = []

        # get object template & render asset handle
        obj_id_1 = obj_templates_mgr.load_configs(
            str(os.path.join(data_path, obj_path))
        )[0]
        obj_template = obj_templates_mgr.get_template_by_ID(obj_id_1)
        obj_handle = obj_template.render_asset_handle
        obj_templates_mgr.register_template(obj_template, force_registration=True)
        obj_ids += [obj_id_1]

        # == VHACD ==
        # Now, create a new object template based on the recently created obj_template
        # specify parameters for running CHD

        # high resolution
        params = habitat_sim._ext.habitat_sim_bindings.VHACDParameters()
        params.resolution = 200000
        params.max_convex_hulls = 32

        # run VHACD, with params passed in and render
        new_handle_1 = sim.apply_convex_hull_decomposition(
            obj_handle, params, True, True
        )
        new_obj_template_1 = obj_templates_mgr.get_template_by_handle(new_handle_1)

        obj_templates_mgr.register_template(new_obj_template_1, force_registration=True)
        obj_ids += [new_obj_template_1.ID]

        # Medium resolution
        params = habitat_sim.VHACDParameters()
        params.resolution = 100000
        params.max_convex_hulls = 4

        # run VHACD, with params passed in and render
        new_handle_2 = sim.apply_convex_hull_decomposition(
            obj_handle, params, True, True
        )
        new_obj_template_2 = obj_templates_mgr.get_template_by_handle(new_handle_2)

        obj_templates_mgr.register_template(new_obj_template_2, force_registration=True)
        obj_ids += [new_obj_template_2.ID]

        # Low resolution
        params = habitat_sim.VHACDParameters()
        params.resolution = 10000
        params.max_convex_hulls = 1

        # run VHACD, with params passed in and render
        new_handle_3 = sim.apply_convex_hull_decomposition(
            obj_handle, params, True, True
        )
        new_obj_template_3 = obj_templates_mgr.get_template_by_handle(new_handle_3)

        obj_templates_mgr.register_template(new_obj_template_3, force_registration=True)
        obj_ids += [new_obj_template_3.ID]

        # now display objects
        cur_ids = []
        for i in range(len(obj_ids)):
            cur_id = sim.add_object(obj_ids[i])
            cur_ids.append(cur_id)
            # get length
            obj_node = sim.get_object_scene_node(cur_id)
            obj_bb = obj_node.cumulative_bb
            length = obj_bb.size().length() * 0.8

            total_length = length * len(obj_ids)
            dist = math.sqrt(3) * total_length / 2

            set_object_state_from_agent(
                sim,
                cur_id,
                offset=np.array(
                    [
                        -total_length / 2 + total_length * i / (len(obj_ids) - 1),
                        1.4,
                        -1 * dist,
                    ]
                ),
            )
            sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cur_id)
            vel_control = sim.get_object_velocity_control(cur_id)
            vel_control.controlling_ang_vel = True
            vel_control.angular_velocity = np.array([0, -1.56, 0])

        # simulate for 4 seconds
        simulate(sim, dt=4, get_frames=True, data=data)

        for cur_id in cur_ids:
            vel_control = sim.get_object_velocity_control(cur_id)
            vel_control.controlling_ang_vel = True
            vel_control.angular_velocity = np.array([-1.56, 0, 0])

        # simulate for 4 seconds
        simulate(sim, dt=4, get_frames=True, data=data)
    return data


if __name__ == "__main__":
    # List of objects you want to execute the VHACD tests on.
    obj_paths = [
        "test_assets/objects/chair.glb",
        "test_assets/objects/donut.glb",
    ]
    # create and show the video
    observations = []
    for obj_path in obj_paths:
        observations += runVHACDSimulation(obj_path)["observations"]
    vut.make_video(
        observations,
        "rgba_camera_1stperson",
        "color",
        output_path + "VHACD_vid_1",
        open_vid=True,
    )
