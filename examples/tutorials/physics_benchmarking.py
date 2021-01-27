# [setup]
import math
import os
import time

import magnum as mn
import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
from habitat_sim.utils import viz_utils as vut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "physics_benchmarking_output/")

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
    backend_cfg.scene_id = "NONE"

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
        physics_step_times.append(mt - st)
        graphics_render_times.append(et - mt)
        collisions.append(sim.get_num_active_contact_points())

    return observations, physics_step_times, graphics_render_times, collisions


# Define each test function & store in list


def bowl_drop_test(
    objects,
    num_objects=30,
    object_scale=0.5,
    object_speed=2,
    post_throw_settling_time=5,
):  # take in parameters here
    """Drops a specified number of objects into a bowl/box and returns metrics including the time to simulate each frame, render each frame, and the number of collisions each frame. """
    # start with bowl being bounding boxes

    cfg = make_configuration()
    try:  # Got to make initialization idiot proof
        sim.close()
    except NameError:
        pass

    sim = habitat_sim.Simulator(cfg)
    place_agent(sim, [5.45, 1.8, 1.2], np.quaternion(-0.83147, 0.2, -0.55557, -0.2))

    # get the physics object attributes manager
    obj_templates_mgr = sim.get_object_template_manager()
    # [/initialize]

    # build bowl
    cube_handle = obj_templates_mgr.get_template_handles("cube")[0]

    bowlParts = []
    bowlIDs = []
    for _i in range(5):
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
    sim.set_translation(np.array([2.50, 0.05, 0.4]), bowlIDs[0])
    sim.set_translation(np.array([2.50, 0.35, 1.35]), bowlIDs[1])
    sim.set_translation(np.array([2.50, 0.35, -0.55]), bowlIDs[2])
    sim.set_translation(np.array([3.45, 0.35, 0.4]), bowlIDs[3])
    sim.set_translation(np.array([1.55, 0.35, 0.4]), bowlIDs[4])

    # load some object templates from configuration files
    object_ids = []
    scales = {
        "test_assets/objects/sphere": 1,
        "test_assets/objects/chair": 0.9,
        "test_assets/objects/donut": 2,
        "test_assets/objects/nested_box": 0.5,
    }
    for obj_path in objects:
        object_ids += [
            obj_templates_mgr.load_configs(str(os.path.join(data_path, obj_path)))[0]
        ]
        obj_template = obj_templates_mgr.get_template_by_ID(object_ids[-1])
        obj_template.scale *= object_scale
        if obj_path in scales.keys():
            obj_template.scale *= scales[obj_path]
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
        diaganal_length = math.sqrt(
            obj_bb.size_x() ** 2 + obj_bb.size_y() ** 2 + obj_bb.size_z() ** 2
        )
        time_til_next_obj = diaganal_length / object_speed / 2
        # obj_node.scale(mn.Vector3(3,3,3))

        # set object position and velocity
        sim.set_translation(np.array([1.50, 2, 1.2]), cur_id)
        initial_linear_velocity = mn.Vector3(1, 0, -1)
        initial_linear_velocity = initial_linear_velocity.normalized() * object_speed
        sim.set_linear_velocity(initial_linear_velocity, cur_id)

        # simulate half a second, then add next object
        (
            cur_observations,
            cur_physics_step_times,
            cur_graphics_render_times,
            cur_collisions,
        ) = simulate(sim, dt=time_til_next_obj, get_frames=make_video)
        observations += cur_observations
        physics_step_times += cur_physics_step_times
        graphics_render_times += cur_graphics_render_times
        collisions += cur_collisions

    (
        cur_observations,
        cur_physics_step_times,
        cur_graphics_render_times,
        cur_collisions,
    ) = simulate(sim, dt=post_throw_settling_time, get_frames=make_video)
    observations += cur_observations
    physics_step_times += cur_physics_step_times
    graphics_render_times += cur_graphics_render_times
    collisions += cur_collisions

    # [/basics]
    # return total time to run, time to load, time to simulate physics, time for rendering
    remove_all_objects(sim)
    sim.close()
    return physics_step_times, graphics_render_times, collisions, observations


benchmarks = {
    #"bowl_drop_test_1": lambda: bowl_drop_test(
    #    ["test_assets/objects/sphere"], 25, 0.5, 2, 5
    #),
    "bowl_drop_test_2": lambda: bowl_drop_test(
        ["test_assets/objects/sphere", "test_assets/objects/chair"], 50, 0.5, 2, 5
    ),
    #"bowl_drop_test_3": lambda: bowl_drop_test(
    #    [
    #        "test_assets/objects/sphere",
    #        "test_assets/objects/chair",
    #        "test_assets/objects/donut",
    #        "test_assets/objects/nested_box",
    #    ],
    #    200,
    #    0.5,
    #    2,
    #    5,
    #),
}  # specify parameters for each scenario

test_sets = {
    "bowl_drop_tests": ["bowl_drop_test_1", "bowl_drop_test_2", "bowl_drop_test_3"]
}


def runTest(testId):
    print("----- Running %s -----" % testId)
    start_time = time.time()
    physics_step_times, graphics_render_times, collisions, observations = benchmarks[
        testId
    ]()
    end_time = time.time()

    print(" ========================= Performance ======================== ")
    print("| Number of frames: " + str(len(physics_step_times)))
    print(
        "| Average time per physics step: "
        + str(sum(physics_step_times) / len(physics_step_times))
    )
    print(
        "| Average time per frame render: "
        + str(sum(graphics_render_times) / len(graphics_render_times))
    )
    print("| Maximum Collisions: " + str(max(collisions)))
    print("| Total time for test: " + str(end_time - start_time))
    print("|----- END of %s -----" % testId)
    print(" ============================================================== ")

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + testId,
            open_vid=show_video,
        )

    if display_graph:
        plt.plot(physics_step_times)
        plt.title("Time to simulate physics for a frame (%s)" % testId)
        plt.xlabel("Frame #")
        plt.ylabel("Time Taken (s)")
        plt.show()

    return end_time - start_time


# Define Main driver function


def main():
    # runs all unit tests
    for testId in benchmarks:
        runTest(testId)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-video", dest="show_video", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.add_argument("--display-graph", dest="display_graph", action="store_true")

    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.show_video
    make_video = args.make_video
    display_graph = args.display_graph

    if make_video and not os.path.exists(output_path):
        os.mkdir(output_path)

    main()
