# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     name: Managed Rigid Object Tutorial
#     provenance: []
#   jupytext:
#     cell_metadata_filter: -all
#     formats: nb_python//py:percent,colabs//ipynb
#     notebook_metadata_filter: all
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.13.7
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %%
# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/main/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s

# %%
# %cd /content/habitat-sim
## [setup]
import os
import random
import sys

import git
import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils import viz_utils as vut

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
output_path = os.path.join(
    dir_path, "examples/tutorials/managed_rigid_object_tutorial_output/"
)


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [-0.15, -0.7, 1.0]
    agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = os.path.join(
        data_path, "scene_datasets/habitat-test-scenes/apartment_1.glb"
    )
    assert os.path.exists(backend_cfg.scene_id)

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    camera_resolution = [544, 720]
    sensor_specs = []

    rgba_camera_1stperson_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_1stperson_spec.uuid = "rgba_camera_1stperson"
    rgba_camera_1stperson_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_1stperson_spec.resolution = camera_resolution
    rgba_camera_1stperson_spec.position = [0.0, 0.6, 0.0]
    rgba_camera_1stperson_spec.orientation = [0.0, 0.0, 0.0]
    rgba_camera_1stperson_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_1stperson_spec)

    depth_camera_1stperson_spec = habitat_sim.CameraSensorSpec()
    depth_camera_1stperson_spec.uuid = "depth_camera_1stperson"
    depth_camera_1stperson_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_camera_1stperson_spec.resolution = camera_resolution
    depth_camera_1stperson_spec.position = [0.0, 0.6, 0.0]
    depth_camera_1stperson_spec.orientation = [0.0, 0.0, 0.0]
    depth_camera_1stperson_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(depth_camera_1stperson_spec)

    rgba_camera_3rdperson_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_3rdperson_spec.uuid = "rgba_camera_3rdperson"
    rgba_camera_3rdperson_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_3rdperson_spec.resolution = camera_resolution
    rgba_camera_3rdperson_spec.position = [0.0, 1.0, 0.3]
    rgba_camera_3rdperson_spec.orientation = [-45, 0.0, 0.0]
    rgba_camera_3rdperson_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_3rdperson_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations


# [/setup]
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

    # %%
    # [initialize]
    # create the simulators AND resets the simulator

    cfg = make_configuration()
    try:  # Got to make initialization idiot proof
        sim.close()
    except NameError:
        pass
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)

    # get the primitive assets attributes manager
    prim_templates_mgr = sim.get_asset_template_manager()

    # get the physics object attributes manager
    obj_templates_mgr = sim.get_object_template_manager()

    # get the rigid object manager
    rigid_obj_mgr = sim.get_rigid_object_manager()
    # [/initialize]

    # %%
    # [basics]

    # load some object templates from configuration files
    sphere_template_id = obj_templates_mgr.load_configs(
        str(os.path.join(data_path, "test_assets/objects/sphere"))
    )[0]

    # add a sphere to the scene, returns the object
    sphere_obj = rigid_obj_mgr.add_object_by_template_id(sphere_template_id)
    # move sphere
    sphere_obj.translation = [2.50, 0.0, 0.2]

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "sim_basics",
            open_vid=show_video,
        )

    # [/basics]
    rigid_obj_mgr.remove_all_objects()
    # %%
    # [object_user_configurations]
    # modify an object's user-defined configurations

    # load some object templates from configuration files
    sphere_template_id = obj_templates_mgr.load_configs(
        str(os.path.join(data_path, "test_assets/objects/sphere"))
    )[0]

    # add a sphere to the scene, returns the object
    sphere_obj = rigid_obj_mgr.add_object_by_template_id(sphere_template_id)

    # set user-defined configuration values
    user_attributes_dict = {
        "obj_val_0": "This is a sphere object named " + sphere_obj.handle,
        "obj_val_1": 17,
        "obj_val_2": False,
        "obj_val_3": 19.7,
        "obj_val_4": [2.50, 0.0, 0.2],
        "obj_val_5": mn.Quaternion.rotation(mn.Deg(90.0), [-1.0, 0.0, 0.0]),
    }
    for k, v in user_attributes_dict.items():
        sphere_obj.user_attributes.set(k, v)

    for k, _ in user_attributes_dict.items():
        print(
            "Sphere Object user attribute : {} : {}".format(
                k, sphere_obj.user_attributes.get_as_string(k)
            )
        )

    # [/object_user_configurations]
    rigid_obj_mgr.remove_all_objects()

    # %%
    # [dynamic_control]

    observations = []
    obj_templates_mgr.load_configs(
        str(os.path.join(data_path, "objects/example_objects/"))
    )
    # search for an object template by key sub-string
    cheezit_template_handle = obj_templates_mgr.get_template_handles(
        "data/objects/example_objects/cheezit"
    )[0]
    # build multiple object initial positions
    box_positions = [
        [2.39, -0.37, 0.0],
        [2.39, -0.64, 0.0],
        [2.39, -0.91, 0.0],
        [2.39, -0.64, -0.22],
        [2.39, -0.64, 0.22],
    ]
    box_orientation = mn.Quaternion.rotation(mn.Deg(90.0), [-1.0, 0.0, 0.0])
    # instance and place the boxes
    boxes = []
    for b in range(len(box_positions)):
        boxes.append(
            rigid_obj_mgr.add_object_by_template_handle(cheezit_template_handle)
        )
        boxes[b].translation = box_positions[b]
        boxes[b].rotation = box_orientation

    # anti-gravity force f=m(-g) using first object's mass (all objects have the same mass)
    anti_grav_force = -1.0 * sim.get_gravity() * boxes[0].mass

    # throw a sphere at the boxes from the agent position
    sphere_template = obj_templates_mgr.get_template_by_id(sphere_template_id)
    sphere_template.scale = [0.5, 0.5, 0.5]

    obj_templates_mgr.register_template(sphere_template)

    # create sphere
    sphere_obj = rigid_obj_mgr.add_object_by_template_id(sphere_template_id)

    sphere_obj.translation = sim.agents[0].get_state().position + [0.0, 1.0, 0.0]
    # get the vector from the sphere to a box
    target_direction = boxes[0].translation - sphere_obj.translation
    # apply an initial velocity for one step
    sphere_obj.linear_velocity = target_direction * 5
    sphere_obj.angular_velocity = [0.0, -1.0, 0.0]

    start_time = sim.get_world_time()
    dt = 3.0
    while sim.get_world_time() < start_time + dt:
        # set forces/torques before stepping the world
        for box in boxes:
            box.apply_force(anti_grav_force, [0.0, 0.0, 0.0])
            box.apply_torque([0.0, 0.01, 0.0])
        sim.step_physics(1.0 / 60.0)
        observations.append(sim.get_sensor_observations())

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "dynamic_control",
            open_vid=show_video,
        )

    # [/dynamic_control]
    rigid_obj_mgr.remove_all_objects()
    # %%
    # [kinematic_interactions]

    chefcan_template_handle = obj_templates_mgr.get_template_handles(
        "data/objects/example_objects/chefcan"
    )[0]
    chefcan_obj = rigid_obj_mgr.add_object_by_template_handle(chefcan_template_handle)
    chefcan_obj.translation = [2.4, -0.64, 0.0]
    # set object to kinematic
    chefcan_obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC

    # drop some dynamic objects
    chefcan_obj_2 = rigid_obj_mgr.add_object_by_template_handle(chefcan_template_handle)
    chefcan_obj_2.translation = [2.4, -0.64, 0.28]
    chefcan_obj_3 = rigid_obj_mgr.add_object_by_template_handle(chefcan_template_handle)
    chefcan_obj_3.translation = [2.4, -0.64, -0.28]
    chefcan_obj_4 = rigid_obj_mgr.add_object_by_template_handle(chefcan_template_handle)
    chefcan_obj_4.translation = [2.4, -0.3, 0.0]

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=True)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "kinematic_interactions",
            open_vid=show_video,
        )

    # [/kinematic_interactions]

    rigid_obj_mgr.remove_all_objects()
    # %%
    # [kinematic_update]
    observations = []

    clamp_template_handle = obj_templates_mgr.get_template_handles(
        "data/objects/example_objects/largeclamp"
    )[0]
    clamp_obj = rigid_obj_mgr.add_object_by_template_handle(clamp_template_handle)
    clamp_obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
    clamp_obj.translation = [0.8, 0.2, 0.5]

    start_time = sim.get_world_time()
    dt = 1.0
    while sim.get_world_time() < start_time + dt:
        # manually control the object's kinematic state
        clamp_obj.translation += [0.0, 0.0, 0.01]
        clamp_obj.rotation = (
            mn.Quaternion.rotation(mn.Rad(0.05), [-1.0, 0.0, 0.0]) * clamp_obj.rotation
        )
        sim.step_physics(1.0 / 60.0)
        observations.append(sim.get_sensor_observations())

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "kinematic_update",
            open_vid=show_video,
        )

    # [/kinematic_update]
    # %%
    # [velocity_control]

    # get object VelocityControl structure and setup control
    vel_control = clamp_obj.velocity_control
    vel_control.linear_velocity = [0.0, 0.0, -1.0]
    vel_control.angular_velocity = [4.0, 0.0, 0.0]
    vel_control.controlling_lin_vel = True
    vel_control.controlling_ang_vel = True

    observations = simulate(sim, dt=1.0, get_frames=True)

    # reverse linear direction
    vel_control.linear_velocity = [0.0, 0.0, 1.0]

    observations += simulate(sim, dt=1.0, get_frames=True)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "velocity_control",
            open_vid=show_video,
        )

    # [/velocity_control]
    # %%
    # [local_velocity_control]

    vel_control.linear_velocity = [0.0, 0.0, 2.3]
    vel_control.angular_velocity = [-4.3, 0.0, 0.0]
    vel_control.lin_vel_is_local = True
    vel_control.ang_vel_is_local = True

    observations = simulate(sim, dt=1.5, get_frames=True)

    # video rendering
    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "local_velocity_control",
            open_vid=show_video,
        )

    # [/local_velocity_control]
    rigid_obj_mgr.remove_all_objects()
    # %%
    # [embodied_agent]

    # load the lobot_merged asset
    locobot_template_id = obj_templates_mgr.load_configs(
        str(os.path.join(data_path, "objects/locobot_merged"))
    )[0]

    # add robot object to the scene with the agent/camera SceneNode attached
    locobot = rigid_obj_mgr.add_object_by_template_id(
        locobot_template_id, sim.agents[0].scene_node
    )
    locobot.translation = [1.75, -1.02, 0.4]

    vel_control = locobot.velocity_control
    vel_control.linear_velocity = [0.0, 0.0, -1.0]
    vel_control.angular_velocity = [0.0, 2.0, 0.0]

    # simulate robot dropping into place
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    vel_control.controlling_lin_vel = True
    vel_control.controlling_ang_vel = True
    vel_control.lin_vel_is_local = True
    vel_control.ang_vel_is_local = True

    # simulate forward and turn
    observations += simulate(sim, dt=1.0, get_frames=make_video)

    vel_control.controlling_lin_vel = False
    vel_control.angular_velocity = [0.0, 1.0, 0.0]

    # simulate turn only
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    vel_control.angular_velocity = [0.0, 0.0, 0.0]
    vel_control.controlling_lin_vel = True
    vel_control.controlling_ang_vel = True

    # simulate forward only with damped angular velocity (reset angular velocity to 0 after each step)
    observations += simulate(sim, dt=1.0, get_frames=make_video)

    vel_control.angular_velocity = [0.0, -1.25, 0.0]

    # simulate forward and turn
    observations += simulate(sim, dt=2.0, get_frames=make_video)

    vel_control.controlling_ang_vel = False
    vel_control.controlling_lin_vel = False

    # simulate settling
    observations += simulate(sim, dt=3.0, get_frames=make_video)

    # remove the agent's body while preserving the SceneNode
    rigid_obj_mgr.remove_object_by_id(locobot.object_id, delete_object_node=False)

    # demonstrate that the locobot object does not now exist'
    print("Locobot is still alive : {}".format(locobot.is_alive))

    # video rendering with embedded 1st person view
    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "robot_control",
            open_vid=show_video,
        )

    # [/embodied_agent]

    # %%
    # [embodied_agent_navmesh]

    # load the lobot_merged asset
    locobot_template_id = obj_templates_mgr.load_configs(
        str(os.path.join(data_path, "objects/locobot_merged"))
    )[0]
    # add robot object to the scene with the agent/camera SceneNode attached
    locobot = rigid_obj_mgr.add_object_by_template_id(
        locobot_template_id, sim.agents[0].scene_node
    )
    initial_rotation = locobot.rotation

    # set the agent's body to kinematic since we will be updating position manually
    locobot.motion_type = habitat_sim.physics.MotionType.KINEMATIC

    # create and configure a new VelocityControl structure
    # Note: this is NOT the object's VelocityControl, so it will not be consumed automatically in sim.step_physics
    vel_control = habitat_sim.physics.VelocityControl()
    vel_control.controlling_lin_vel = True
    vel_control.lin_vel_is_local = True
    vel_control.controlling_ang_vel = True
    vel_control.ang_vel_is_local = True
    vel_control.linear_velocity = [0.0, 0.0, -1.0]

    # try 2 variations of the control experiment
    for iteration in range(2):
        # reset observations and robot state
        observations = []
        locobot.translation = [1.75, -1.02, 0.4]
        locobot.rotation = initial_rotation
        vel_control.angular_velocity = [0.0, 0.0, 0.0]

        video_prefix = "robot_control_sliding"
        # turn sliding off for the 2nd pass
        if iteration == 1:
            sim.config.sim_cfg.allow_sliding = False
            video_prefix = "robot_control_no_sliding"

        # manually control the object's kinematic state via velocity integration
        start_time = sim.get_world_time()
        last_velocity_set = 0
        dt = 6.0
        time_step = 1.0 / 60.0
        while sim.get_world_time() < start_time + dt:
            previous_rigid_state = locobot.rigid_state

            # manually integrate the rigid state
            target_rigid_state = vel_control.integrate_transform(
                time_step, previous_rigid_state
            )

            # snap rigid state to navmesh and set state to object/agent
            end_pos = sim.step_filter(
                previous_rigid_state.translation, target_rigid_state.translation
            )
            locobot.translation = end_pos
            locobot.rotation = target_rigid_state.rotation

            # Check if a collision occured
            dist_moved_before_filter = (
                target_rigid_state.translation - previous_rigid_state.translation
            ).dot()
            dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

            # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
            # collision _didn't_ happen. One such case is going up stairs.  Instead,
            # we check to see if the the amount moved after the application of the filter
            # is _less_ than the amount moved before the application of the filter
            EPS = 1e-5
            collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

            # run any dynamics simulation
            sim.step_physics(time_step)

            # render observation
            observations.append(sim.get_sensor_observations())

            # randomize angular velocity
            last_velocity_set += time_step
            if last_velocity_set >= 1.0:
                vel_control.angular_velocity = [0.0, (random.random() - 0.5) * 2.0, 0.0]
                last_velocity_set = 0

        # video rendering with embedded 1st person views
        if make_video:
            sensor_dims = (
                sim.get_agent(0).agent_config.sensor_specifications[0].resolution
            )
            overlay_dims = (int(sensor_dims[1] / 4), int(sensor_dims[0] / 4))
            overlay_settings = [
                {
                    "obs": "rgba_camera_1stperson",
                    "type": "color",
                    "dims": overlay_dims,
                    "pos": (10, 10),
                    "border": 2,
                },
                {
                    "obs": "depth_camera_1stperson",
                    "type": "depth",
                    "dims": overlay_dims,
                    "pos": (10, 30 + overlay_dims[1]),
                    "border": 2,
                },
            ]

            vut.make_video(
                observations=observations,
                primary_obs="rgba_camera_3rdperson",
                primary_obs_type="color",
                video_file=output_path + video_prefix,
                fps=60,
                open_vid=show_video,
                overlay_settings=overlay_settings,
                depth_clip=10.0,
            )

    # [/embodied_agent_navmesh]
