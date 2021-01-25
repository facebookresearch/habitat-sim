# [setup]
import math
import os
import random
import sys
import time

import git
import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils import viz_utils as vut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "physics_unit_tests_output/")

save_index = 0


# Define Helper Functions (Taken from rigid_object_tutorial.py)


def remove_all_objects(sim):
    for id_ in sim.get_existing_object_ids():
        sim.remove_object(id_)


def place_agent(sim, position, orientation):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = position
    agent_state.rotation = orientation
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = os.path.join(
        data_path, "test_assets/scenes/stage_floor1.glb"
    )
    assert os.path.exists(backend_cfg.scene_id)
    backend_cfg.enable_physics = True

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    camera_resolution = [544, 720]
    sensors = {
        "rgba_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 0.6, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
        "depth_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": camera_resolution,
            "position": [0.0, 0.6, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
        "rgba_camera_3rdperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 1.0, 0.3],
            "orientation": [-45, 0.0, 0.0],
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = habitat_sim.SensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]
        sensor_spec.orientation = sensor_params["orientation"]
        sensor_specs.append(sensor_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    # print("Simulating " + str(dt) + " world seconds.")
    observations = []
    physics_step_times = []
    graphics_render_times = []
    collisions = []

    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        st = time.time()
        sim.step_physics(1.0 / 60.0)
        mt = time.time()
        if get_frames:
            observations.append(sim.get_sensor_observations())
        et = time.time()
        physics_step_times.append(mt-st)
        graphics_render_times.append(et-mt)
        collisions.append(sim.get_num_active_contact_points())


    return observations, physics_step_times, graphics_render_times, collisions


# Define each test function & store in list

def bowl_drop_test(objects = ["test_assets/objects/sphere", "test_assets/objects/chair"], num_objects = 100, object_speed = 2, post_throw_settling_time = 5, object_scale = 0.5): # take in parameters here
    # start with bowl being bounding boxes

    cfg = make_configuration()
    try:  # Got to make initialization idiot proof
        sim.close()
    except NameError:
        pass
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim, [5.45, 1.8, 1.2], np.quaternion(-0.83147, 0.2, -0.55557, -0.2))

    # get the primitive assets attributes manager
    prim_templates_mgr = sim.get_asset_template_manager()

    # get the physics object attributes manager
    obj_templates_mgr = sim.get_object_template_manager()
    # [/initialize]

    # build bowl
    cube_handle = obj_templates_mgr.get_template_handles("cube")[0]

    bowlParts = []
    bowlIDs = []
    for i in range(5):
        bowlParts += [obj_templates_mgr.get_template_by_handle(cube_handle)]
    bowlParts[0].scale = np.array([1.0, 0.1, 1.0])
    bowlParts[1].scale = np.array([1.0, 0.6, 0.1])
    bowlParts[2].scale = np.array([1.0, 0.6, 0.1])
    bowlParts[3].scale = np.array([0.1, 0.6, 1.0])
    bowlParts[4].scale = np.array([0.1, 0.6, 1.0])

    for i in range(5):
        part_name = "bowl_part_" + str(i)
        obj_templates_mgr.register_template(bowlParts[i], part_name)
        bowlIDs += [sim.add_object_by_handle(part_name)]
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, bowlIDs[i])
    sim.set_translation(np.array([2.50, .05, 0.4]), bowlIDs[0])
    sim.set_translation(np.array([2.50, .35, 1.35]), bowlIDs[1])
    sim.set_translation(np.array([2.50, .35, -0.55]), bowlIDs[2])
    sim.set_translation(np.array([3.45, .35, 0.4]), bowlIDs[3])
    sim.set_translation(np.array([1.55, .35, 0.4]), bowlIDs[4])


    # load some object templates from configuration files
    object_ids = []
    for obj_path in objects:
        object_ids += [obj_templates_mgr.load_configs(
            str(os.path.join(data_path, obj_path))
        )[0]]
        obj_template = obj_templates_mgr.get_template_by_ID(object_ids[-1])
        obj_template.scale *= object_scale
        obj_templates_mgr.register_template(obj_template)

    observations = []
    physics_step_times = []
    graphics_render_times = []
    collisions = []

    # throw objects into bowl
    for i in range(num_objects):
        cur_id = sim.add_object(object_ids[i % len(object_ids)])

        obj_node = sim.get_object_scene_node(cur_id)
        obj_bb = obj_node.cumulative_bb
        diaganal_length = math.sqrt(obj_bb.size_x()**2 + obj_bb.size_y()**2 + obj_bb.size_z()**2)
        time_til_next_obj = diaganal_length / 2 / object_speed
        print(i, time_til_next_obj)
        # obj_node.scale(mn.Vector3(3,3,3))

        # set object position and velocity
        sim.set_translation(np.array([1.50, 2, 1.2]), cur_id)
        initial_linear_velocity = mn.Vector3(1, 0, -1)
        initial_linear_velocity = initial_linear_velocity.normalized() * object_speed
        sim.set_linear_velocity(initial_linear_velocity, cur_id)

        # simulate half a second, then add next object
        cur_observations, cur_physics_step_times, cur_graphics_render_times, cur_collisions = simulate(sim, dt=time_til_next_obj, get_frames=make_video)
        observations += cur_observations
        physics_step_times += cur_physics_step_times
        graphics_render_times += cur_graphics_render_times
        collisions += cur_collisions

    cur_observations, cur_physics_step_times, cur_graphics_render_times, cur_collisions = simulate(sim, dt=post_throw_settling_time, get_frames=make_video)
    observations += cur_observations
    physics_step_times += cur_physics_step_times
    graphics_render_times += cur_graphics_render_times
    collisions += cur_collisions

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "sim_basics",
            open_vid=show_video,
        )

    # [/basics]
    # return total time to run, time to load, time to simulate physics, time for rendering
    return physics_step_times, graphics_render_times, collisions 
    remove_all_objects(sim)

unit_tests = { "bowl_drop_test": bowl_drop_test } # specify parameters for each scenario

def runTest(testId):
    print("----- Running %s -----" % testId)
    start_time = time.time()
    physics_step_times, graphics_render_times, collisions = unit_tests[testId]()
    end_time = time.time()
    print("Average time per physics step: " + str(sum(physics_step_times)/len(physics_step_times)))
    print("Collisions per physics step: " + str(sum(collisions)/len(collisions)))
    print("Average time per frame render: " + str(sum(graphics_render_times)/len(graphics_render_times)))
    print("Total time for test: " + str(end_time - start_time))
    print("----- END of %s -----" % testId)
    return end_time - start_time



# Define Main driver function

def main():
    for testId in unit_tests:
        runTest(testId)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-video", dest="show_video", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.show_video
    make_video = args.make_video
    if make_video and not os.path.exists(output_path):
        os.mkdir(output_path)

    main()


# NOTES
# Command line arguments should be high-level i.e. --bowl-test-resolution-1, --bowl-test-resolution-2, --bowl-test-suite

