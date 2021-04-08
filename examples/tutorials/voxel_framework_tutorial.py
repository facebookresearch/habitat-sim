# [setup]
import os
import time

import numpy as np

import habitat_sim
import habitat_sim.geo as geo
from habitat_sim.utils import viz_utils as vut
import magnum as mn
import math

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "voxel_framework_output/")


# Define Helper Functions (Taken from rigid_object_tutorial.py)


def remove_all_objects(sim):
    for id_ in sim.get_existing_object_ids():
        sim.remove_object(id_)

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
                [6.45, -1.3, -0.2],
                np.quaternion(0.7071068,0,0.7071068,0),
            ),
        )

        data = {"observations": []}

        # First, we'll create a few objects and voxelize them to varying resolutions.

        # path to the asset
        obj_path = "test_assets/objects/donut.glb"

        # get the physics object attributes manager
        obj_templates_mgr = sim.get_object_template_manager()

        # load object template
        obj_id = obj_templates_mgr.load_configs(
            str(os.path.join(data_path, obj_path))
        )[0]
        obj_template = obj_templates_mgr.get_template_by_ID(obj_id)
        obj_handle = obj_template.render_asset_handle
        obj_templates_mgr.register_template(obj_template, force_registration=True)

        # add objects
        cur_ids = []
        for i in range(3):
            cur_id = sim.add_object(obj_id)
            cur_ids.append(cur_id)
            # get length
            obj_node = sim.get_object_scene_node(cur_id)
            obj_bb = obj_node.cumulative_bb
            length = obj_bb.size().length() * 0.8

            total_length = length * 3
            dist = math.sqrt(3) * total_length / 2

            # Set the objects in front of the camera/agent
            set_object_state_from_agent(
                sim,
                cur_id,
                offset=np.array(
                    [
                        -total_length / 2 + total_length * i / 2,
                        1.4,
                        -1.4 * dist,
                    ]
                ),
            )
            sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cur_id)
            vel_control = sim.get_object_velocity_control(cur_id)
            vel_control.controlling_ang_vel = True
            vel_control.angular_velocity = np.array([ 0, 0, -1.56/2])

        # Show the objects for 1 seconds with no voxelization
        simulate(sim, dt=1, get_frames=True, data=data)

        # Now voxelize each object with different resolutions and draw its voxelization
        resolutions = [100000, 1000, 100]
        for i in range(3):
            # Voxelize the object with the specified resolution.
            sim.voxelize_object(cur_ids[i], resolutions[i])
            # Draw the 'Boundary' voxelization, which is the main initial boundary cell grid which is created
            sim.set_object_voxelization_draw(True, cur_ids[i], "Boundary")

        # Show the objects for 1 seconds with their voxelization's drawn
        simulate(sim, dt=1, get_frames=True, data=data)

        # rotate objects in the other direction for 4 seconds
        for i in range(3):
            vel_control = sim.get_object_velocity_control(cur_ids[i])
            vel_control.controlling_ang_vel = True
            vel_control.angular_velocity = np.array([0,-1.56, 0])

        simulate(sim, dt=4, get_frames=True, data=data)

        # Remove the objects
        remove_all_objects(sim)
        simulate(sim, dt=1, get_frames=True, data=data)

        # Voxelize the stage, and set the stage voxelizaton to be drawn
        resolution = 2000000
        sim.voxelize_stage(resolution)
        sim.set_stage_voxelization_draw(True, "Boundary")

        # Show for 2 seconds
        simulate(sim, dt=2, get_frames=True, data=data)

        # get the stage's voxelization for direct grid access, manipulation, and for using voxel utility functions
        voxelization = sim.get_stage_voxelization()

        # Coordinate conversion allows you to convert from global coordinates to voxel indices and vice versa.
        print(voxelization.get_voxel_index_from_global_coords((0.0, 0.0, 0.0)))
        print(voxelization.get_global_coords_from_voxel_index((0, 0, 0)))

        # Now, using the utility functions found in geo, we can compute a signed distance field
        # compute a euclidean signed distance field and register the result under 'ESDF'
        geo.generate_euclidean_distance_sdf(voxelization, "ESDF")

        # Now generate heatmap slices of the ESDF grid and visualize them.
        dimensions = voxelization.get_voxel_grid_dimensions()
        for i in range(dimensions[0]):
            # first we generate the mesh of the i_th slice of the ESDF grid (slices can only be made along the first dimension, or x-axis). We set [-15,0] as our range, so values closer to -15 will be red whereas values closer to 0 will be green.
            voxelization.generate_float_slice_mesh("ESDF", i, -15, 0)
            # Then we draw the newly created slice
            sim.set_stage_voxelization_draw(True, "ESDF")
            simulate(sim, dt=4.0 / dimensions[0], get_frames=True, data=data)

        # Now let's visualize the distance gradient vector field.

        # generate a distance flow field and register the result under 'GradientField'
        geo.generate_distance_gradient_field(voxelization, "GradientField")

        # generate the mesh for the gradient field
        voxelization.generate_mesh("GradientField")

        # visualize the vector field for 3 seconds
        sim.set_stage_voxelization_draw(True, "GradientField")
        simulate(sim, dt=3, get_frames=True, data=data)

        # Now, we'll illustrate the use of creating a custom grid and visualizing it.
        # create a new, empty bool grid
        voxelization.add_bool_grid("MiddleOfStageVoxels")
        middle_voxel_grid = voxelization.get_bool_grid("MiddleOfStageVoxels")

        # generate a mesh for the voxels which are greater than 11 voxels away from the nearest boundary.
        esdf_grid = voxelization.get_float_grid("ESDF")

        # iterate through every voxel. If the euclidean signed distance field value is below -12 (indicating it is an interior (-) voxel and that it is more than 12 voxels away from the nearest boundary cell), we set its value to true in our new grid.
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                for k in range(dimensions[2]):
                    val = esdf_grid[i][j][k]
                    if val < -12:
                        middle_voxel_grid[i][j][k] = True

        # Now generate a mesh for the grid and display it for 3 seconds.
        voxelization.generate_mesh("MiddleOfStageVoxels")
        sim.set_stage_voxelization_draw(True, "MiddleOfStageVoxels")
        simulate(sim, dt=3, get_frames=True, data=data)

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
