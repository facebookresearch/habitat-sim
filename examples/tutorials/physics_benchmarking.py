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

    if "observations" in data:
        data["observations"] += observations
    if "physics_step_times" in data:
        data["physics_step_times"] += physics_step_times
    if "graphics_render_times" in data:
        data["graphics_render_times"] += graphics_render_times
    if "collisions" in data:
        data["collisions"] += collisions


# Define each test function & store in list


def box_drop_test(
    objects,
    num_objects=30,
    box_size=2,
    object_speed=2,
    post_throw_settling_time=5,
):  # take in parameters here
    """Drops a specified number of objects into a box and returns metrics including the time to simulate each frame, render each frame, and the number of collisions each frame. """
    # start with box being bounding boxes

    data = {
        "observations": [],
        "physics_step_times": [],
        "graphics_render_times": [],
        "collisions": [],
    }

    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        sim.initialize_agent(
            0,
            habitat_sim.AgentState(
                [5.45 * box_size, 2.4 * box_size, 1.2 * box_size],
                np.quaternion(-0.83147, 0.2, -0.55557, -0.2),
            ),
        )

        # get the physics object attributes manager
        obj_templates_mgr = sim.get_object_template_manager()
        # [/initialize]

        # build box
        cube_handle = obj_templates_mgr.get_template_handles("cube")[0]

        boxParts = []
        boxIDs = []
        for _i in range(5):
            boxParts += [obj_templates_mgr.get_template_by_handle(cube_handle)]
        boxParts[0].scale = np.array([1.0, 0.1, 1.0]) * box_size
        boxParts[1].scale = np.array([1.0, 0.6, 0.1]) * box_size
        boxParts[2].scale = np.array([1.0, 0.6, 0.1]) * box_size
        boxParts[3].scale = np.array([0.1, 0.6, 1.0]) * box_size
        boxParts[4].scale = np.array([0.1, 0.6, 1.0]) * box_size

        for i in range(5):
            part_name = "box_part_" + str(i)
            obj_templates_mgr.register_template(boxParts[i], part_name)
            boxIDs += [sim.add_object_by_handle(part_name)]
            sim.set_object_motion_type(
                habitat_sim.physics.MotionType.KINEMATIC, boxIDs[i]
            )
        sim.set_translation(np.array([2.50, 0.05, 0.4]) * box_size, boxIDs[0])
        sim.set_translation(np.array([2.50, 0.35, 1.30]) * box_size, boxIDs[1])
        sim.set_translation(np.array([2.50, 0.35, -0.50]) * box_size, boxIDs[2])
        sim.set_translation(np.array([3.40, 0.35, 0.4]) * box_size, boxIDs[3])
        sim.set_translation(np.array([1.60, 0.35, 0.4]) * box_size, boxIDs[4])
        for i in range(5):
            sim.set_object_motion_type(habitat_sim.physics.MotionType.STATIC, boxIDs[i])

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
                obj_templates_mgr.load_configs(str(os.path.join(data_path, obj_path)))[
                    0
                ]
            ]
            obj_template = obj_templates_mgr.get_template_by_ID(object_ids[-1])
            if obj_path in scales.keys():
                obj_template.scale *= scales[obj_path]
            obj_templates_mgr.register_template(obj_template)

        # throw objects into box
        for i in range(num_objects):
            cur_id = sim.add_object(object_ids[i % len(object_ids)])

            obj_node = sim.get_object_scene_node(cur_id)
            obj_bb = obj_node.cumulative_bb
            diaganal_length = math.sqrt(
                obj_bb.size_x() ** 2 + obj_bb.size_y() ** 2 + obj_bb.size_z() ** 2
            )
            time_til_next_obj = diaganal_length / (object_speed * box_size) / 2
            # obj_node.scale(mn.Vector3(3,3,3))

            # set object position and velocity
            sim.set_translation(np.multiply(np.array([1.50, 2, 1.2]), box_size), cur_id)
            initial_linear_velocity = mn.Vector3(1, 0, -1)
            initial_linear_velocity = (
                initial_linear_velocity.normalized() * object_speed * box_size
            )
            sim.set_linear_velocity(initial_linear_velocity, cur_id)

            # simulate half a second, then add next object
            simulate(sim, dt=time_til_next_obj, get_frames=make_video, data=data)

        simulate(sim, dt=post_throw_settling_time, get_frames=make_video, data=data)

        # [/basics]
        # return total time to run, time to load, time to simulate physics, time for rendering
        remove_all_objects(sim)
    return data


benchmarks = {
    # "box_drop_test_1": lambda: box_drop_test(
    #    ["test_assets/objects/sphere"], 25, 2, 1.8, 10
    # ),
    "box_drop_test_2": lambda: box_drop_test(
        ["test_assets/objects/sphere", "test_assets/objects/chair"], 50, 2, 1.8, 10
    ),
    # "box_drop_test_3": lambda: box_drop_test(
    #    [
    #        "test_assets/objects/sphere",
    #        "test_assets/objects/chair",
    #        "test_assets/objects/donut",
    #        "test_assets/objects/nested_box",
    #    ],
    #    200,
    #    2,
    #    1.8,
    #    10,
    # ),
}  # specify parameters for each scenario

test_sets = {
    "box_drop_tests": ["box_drop_test_1", "box_drop_test_2", "box_drop_test_3"]
}


def createFrameLinePlot(testId, y_axis, title, metric):
    plt.plot(y_axis)
    plt.title(title + " (%s)" % testId)
    plt.xlabel("Frame #")
    plt.ylabel(metric)
    filename = testId + "_%s_graph" % metric + ".png"
    filename = filename.replace(" ", "_")
    plt.savefig(output_path + filename)


def runTest(testId):
    print("----- Running %s -----" % testId)
    start_time = time.time()
    data = benchmarks[testId]()
    physics_step_times = data["physics_step_times"]
    graphics_render_times = data["graphics_render_times"]
    collisions = data["collisions"]
    observations = data["observations"]
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

    if graph_step_times:
        createFrameLinePlot(
            testId,
            physics_step_times,
            "Time to simulate a frame (1/60 s)",
            "Simulation Time (s)",
        )
    if graph_render_times:
        createFrameLinePlot(
            testId,
            graphics_render_times,
            "Time to render a frame (1/60 s)",
            "Render Time (s)",
        )
    if graph_collisions:
        createFrameLinePlot(
            testId,
            collisions,
            "Active Collisions per Frame frame (1/60 s)",
            "Active Collisions",
        )

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
    parser.add_argument(
        "--graph-step-times", dest="graph_step_times", action="store_true"
    )
    parser.add_argument(
        "--graph-render-times", dest="graph_render_times", action="store_true"
    )
    parser.add_argument(
        "--graph-collisions", dest="graph_collisions", action="store_true"
    )
    # input: --graph collisions render-times step-times
    # ^ also do for test suites
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.show_video
    make_video = args.make_video
    graph_step_times = args.graph_step_times
    graph_render_times = args.graph_render_times
    graph_collisions = args.graph_collisions
    print(graph_step_times, graph_render_times, graph_collisions)

    if make_video and not os.path.exists(output_path):
        os.mkdir(output_path)

    main()
