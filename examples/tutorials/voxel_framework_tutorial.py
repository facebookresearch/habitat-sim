# [setup]
import os
import time

import numpy as np

import habitat_sim
import habitat_sim.geo as geo
from habitat_sim.utils import viz_utils as vut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "voxel_framework_output/")


# Define Helper Functions (Taken from rigid_object_tutorial.py)


def remove_all_objects(sim):
    for id_ in sim.get_existing_object_ids():
        sim.remove_object(id_)


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = (
        data_path + "/scene_datasets/habitat-test-scenes/apartment_1.glb"
    )

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
        collisions.append(sim.get_num_active_contact_points())

    if "observations" in data:
        data["observations"] += observations
    if "physics_step_times" in data:
        data["physics_step_times"] += physics_step_times
    if "graphics_render_times" in data:
        data["graphics_render_times"] += graphics_render_times
    if "collisions" in data:
        data["collisions"] += collisions


if __name__ == "__main__":
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        sim.initialize_agent(
            0,
            habitat_sim.AgentState(
                [5.45, -0.5, 1.2],
                np.quaternion(-0.8000822, 0.1924500, -0.5345973, -0.1924500),
            ),
        )

        data = {"observations": []}

        # Show the stage for 2 seconds with no voxelization
        simulate(sim, dt=2, get_frames=True, data=data)

        # Voxelize the stage, and set the stage voxelizaton to be drawn
        resolution = 2000000
        sim.voxelize_stage(resolution)
        sim.set_stage_voxelization_draw(True, "Boundary")

        # Show for 2 seconds
        simulate(sim, dt=2, get_frames=True, data=data)

        voxelization = sim.get_stage_voxelization()
        # Coordinate conversion
        print(voxelization.get_voxel_index_from_global_coords((0.0, 0.0, 0.0)))
        print(voxelization.get_global_coords_from_voxel_index((0, 0, 0)))

        # running SDF calculations
        # compute a euclidean signed distance field and register the result under "ESDF"
        geo.generate_manhattan_distance_sdf(voxelization, "ESDF")

        # generate a distance flow field and register the result under "FlowField"
        geo.generate_distance_gradient_field(voxelization, "GradientField")

        # Now generate heatmap slices of the ESDF grid
        dimensions = voxelization.get_voxel_grid_dimensions()
        for i in range(dimensions[0]):
            voxelization.generate_int_slice_mesh("ESDF", i, -15, 0)
            sim.set_stage_voxelization_draw(True, "ESDF")
            simulate(sim, dt=4.0 / dimensions[0], get_frames=True, data=data)


        # generate a mesh for the flow field (with is_vector boolean = true) and display is for a second.
        voxelization.generate_mesh("GradientField")
        sim.set_stage_voxelization_draw(True, "GradientField")

        gradientFieldGrid = voxelization.get_float_grid("ESDF")
        print(gradientFieldGrid[0][0][0])


        simulate(sim, dt=1, get_frames=True, data=data)

        # Turn stage voxelization visualization off
        sim.set_stage_voxelization_draw(False)

        simulate(sim, dt=1, get_frames=True, data=data)


        if not os.path.exists(output_path):
            os.mkdir(output_path)

        # Now generate..

        vut.make_video(
            data["observations"],
            "rgba_camera_1stperson",
            "color",
            output_path + "test",
            open_vid=True,
        )
