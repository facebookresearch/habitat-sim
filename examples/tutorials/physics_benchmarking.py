# [setup]
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
    sensor_spec = habitat_sim.CameraSensorSpec()
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
        collisions.append(sim.get_physics_num_active_contact_points())

    if "observations" in data:
        data["observations"] += observations
    if "physics_step_times" in data:
        data["physics_step_times"] += physics_step_times
    if "graphics_render_times" in data:
        data["graphics_render_times"] += graphics_render_times
    if "collisions" in data:
        data["collisions"] += collisions


def_params = habitat_sim.VHACDParameters()


def box_drop_test(
    args,
    objects,
    num_objects=30,
    box_size=2,
    object_speed=2,
    post_throw_settling_time=5,
    useVHACD=False,
    VHACDParams=def_params,
):  # take in parameters here
    """Drops a specified number of objects into a box and returns metrics including the time to simulate each frame, render each frame, and the number of collisions each frame. """

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
                np.quaternion(-0.8000822, 0.1924500, -0.5345973, -0.1924500),
            ),
        )

        # get the physics object attributes manager
        obj_templates_mgr = sim.get_object_template_manager()

        # build box
        cube_handle = obj_templates_mgr.get_template_handles("cube")[0]
        box_parts = []
        boxIDs = []
        for _i in range(5):
            box_parts += [obj_templates_mgr.get_template_by_handle(cube_handle)]
        box_parts[0].scale = np.array([1.0, 0.1, 1.0]) * box_size
        box_parts[1].scale = np.array([1.0, 0.6, 0.1]) * box_size
        box_parts[2].scale = np.array([1.0, 0.6, 0.1]) * box_size
        box_parts[3].scale = np.array([0.1, 0.6, 1.0]) * box_size
        box_parts[4].scale = np.array([0.1, 0.6, 1.0]) * box_size

        for i in range(5):
            part_name = "box_part_" + str(i)
            obj_templates_mgr.register_template(box_parts[i], part_name)
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
        for obj in objects:
            obj_path = obj["path"]
            object_ids += [
                obj_templates_mgr.load_configs(str(os.path.join(data_path, obj_path)))[
                    0
                ]
            ]
            obj_template = obj_templates_mgr.get_template_by_ID(object_ids[-1])

            if "scale" in obj:
                obj_template.scale *= obj["scale"]
            obj_handle = obj_template.render_asset_handle

            if useVHACD:
                new_handle = sim.apply_convex_hull_decomposition(
                    obj_handle, VHACDParams, True, False
                )

                new_obj_template = obj_templates_mgr.get_template_by_handle(new_handle)

                if "scale" in obj:
                    new_obj_template.scale *= obj["scale"]

                obj_templates_mgr.register_template(
                    new_obj_template, force_registration=True
                )
                object_ids[-1] = new_obj_template.ID
                print("Template Registered")
            else:
                obj_templates_mgr.register_template(
                    obj_template, force_registration=True
                )

        # throw objects into box
        for i in range(num_objects):
            cur_id = sim.add_object(object_ids[i % len(object_ids)])

            obj_node = sim.get_object_scene_node(cur_id)
            obj_bb = obj_node.cumulative_bb
            diagonal_length = obj_bb.size().length()
            time_til_next_obj = diagonal_length / (object_speed * box_size) / 2

            # set object position and velocity
            sim.set_translation(np.multiply(np.array([1.50, 2, 1.2]), box_size), cur_id)
            sim.set_linear_velocity(
                mn.Vector3(1, 0, -1).normalized() * object_speed * box_size, cur_id
            )

            # simulate a short amount of time, then add next object
            simulate(sim, dt=time_til_next_obj, get_frames=args.make_video, data=data)

        simulate(
            sim, dt=post_throw_settling_time, get_frames=args.make_video, data=data
        )

        # [/basics]
        # return total time to run, time to load, time to simulate physics, time for rendering
        remove_all_objects(sim)
    return data


benchmarks = {
    "box_drop_test_1": {
        "description": "Drop 25 spheres into a box.",
        "test": lambda: box_drop_test(
            args, [{"path": "test_assets/objects/sphere", "scale": 1}], 25, 2, 1.8, 10
        ),
    },
    "box_drop_test_2": {
        "description": "Drop 25 spheres and 25 chairs into a box.",
        "test": lambda: box_drop_test(
            args,
            [
                {"path": "test_assets/objects/sphere", "scale": 1},
                {"path": "test_assets/objects/chair", "scale": 0.9},
            ],
            50,
            2,
            1.8,
            10,
        ),
    },
    "box_drop_test_3": {
        "description": "Drop 50 spheres, 5 chairs, 50 donuts, and 50 boxes into a box.",
        "test": lambda: box_drop_test(
            args,
            [
                {"path": "test_assets/objects/sphere", "scale": 1},
                {"path": "test_assets/objects/chair", "scale": 0.9},
                {"path": "test_assets/objects/donut", "scale": 2},
                {"path": "test_assets/objects/nested_box", "scale": 0.5},
            ],
            20,
            2,
            1.8,
            10,
        ),
    },
}  # specify parameters for each scenario

# Define a grouping of tests you want to run
benchmark_sets = {
    "box_drop_tests": ["box_drop_test_1", "box_drop_test_2", "box_drop_test_3"]
}


def create_frame_line_plot(test_id, y_axis, title):
    plt.plot(y_axis)
    plt.title(title + " (%s)" % test_id)
    plt.xlabel("Frame #")
    plt.ylabel(title)
    filename = test_id + "_%s_graph" % title + ".png"
    filename = filename.replace(" ", "_")
    plt.savefig(output_path + filename)
    plt.cla()


def run_test(test_id, args):
    print("----- Running %s -----" % test_id)
    start_time = time.time()
    data = benchmarks[test_id]["test"]()
    physics_step_times = data["physics_step_times"]
    graphics_render_times = data["graphics_render_times"]
    collisions = data["collisions"]
    observations = data["observations"]
    end_time = time.time()

    test_log = ""
    test_log += "Test: %s completed" % test_id + "\n"
    test_log += "\t Description: " + benchmarks[test_id]["description"] + "\n"
    test_log += (
        "\t ========================= Performance ======================== " + "\n"
    )
    test_log += "\t| Number of frames: " + str(len(physics_step_times)) + "\n"
    test_log += (
        "\t| Average time per physics step: "
        + str(sum(physics_step_times) / len(physics_step_times))
        + "\n"
    )
    test_log += (
        "\t| Average time per frame render: "
        + str(sum(graphics_render_times) / len(graphics_render_times))
        + "\n"
    )
    test_log += "\t| Maximum Collisions: " + str(max(collisions)) + "\n"
    test_log += "\t| Total time for test: " + str(end_time - start_time) + "\n"
    test_log += "\t|----- END of %s -----" % test_id + "\n"
    test_log += (
        "\t ============================================================== " + "\n"
    )
    test_log += "\n"

    print(test_log)
    if args.make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + test_id,
            open_vid=args.show_video,
        )

    for graph_name in args.graph:
        if graph_name in data:
            create_frame_line_plot(test_id, data[graph_name], graph_name)

    return test_log


def main(args):
    log_contents = ""
    if len(args.run) == 0:
        # runs all benchmarks when none are specified
        for test_id in benchmarks:
            log_contents += run_test(test_id, args)
    else:
        # only runs the benchmark/benchmark_sets specified in commandline arguments
        for test in args.run:
            if test in benchmark_sets:
                for sub_test in benchmark_sets[test]:
                    log_contents += run_test(sub_test, args)
            elif test in benchmarks:
                log_contents += run_test(test, args)
            else:
                print(test + " does not exist.")
                log_contents += test + " does not exist." + "\n" + "\n"

    if args.save_log:
        with open(output_path + "benchmark_log_file.txt", "w") as log:
            log.write(log_contents)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-video", dest="show_video", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.add_argument(
        "--save-to-log",
        dest="save_log",
        action="store_true",
        help="Save benchmark results to a log file in the script output directory.",
    )
    parser.add_argument(
        "--graph",
        default=[],
        nargs="*",
        type=str,
        metavar="N",
        help="Save graphs of specified metrics (collisions, physics_step_times, graphics_render_times) to the script output directory.",
    )
    parser.add_argument(
        "--run",
        default=[],
        nargs="*",
        type=str,
        metavar="N",
        help="Specify which benchmarks or benchmark sets should be ran. All benchmarks are run by default.",
    )

    parser.set_defaults(show_video=True, make_video=True, save_log=False)
    args = parser.parse_args()

    if not os.path.exists(output_path):
        os.mkdir(output_path)

    main(args)
