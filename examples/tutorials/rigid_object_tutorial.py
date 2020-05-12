# [setup]
import math
import os

import cv2
import magnum as mn
import numpy as np

import habitat_sim
import habitat_sim.utils.common as ut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "rigid_object_tutorial_output/")


def make_video_cv2(observations, prefix="", open_vid=True, multi_obs=False):
    videodims = (720, 540)
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    video = cv2.VideoWriter(output_path + prefix + ".mp4", fourcc, 60, videodims)
    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150
    for ob in observations:

        # If in RGB/RGBA format, change first to RGB and change to BGR
        bgr_im_1st_person = ob["rgba_camera_1stperson"][..., 0:3][..., ::-1]

        if multi_obs:
            # embed the 1st person frame into the 3rd person frame and write to video
            bgr_im_3rd_person = ob["rgba_camera_3rdperson"][..., 0:3][..., ::-1]
            resized_1st_person = cv2.resize(
                bgr_im_1st_person, thumb_size, interpolation=cv2.INTER_AREA
            )
            x_offset = y_offset = 50
            bgr_im_3rd_person[
                y_offset - 1 : y_offset + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            bgr_im_3rd_person[
                y_offset : y_offset + resized_1st_person.shape[0],
                x_offset : x_offset + resized_1st_person.shape[1],
            ] = resized_1st_person
            video.write(bgr_im_3rd_person)
        else:
            # write the 1st person observation to video
            video.write(bgr_im_1st_person)
    video.release()
    if open_vid:
        os.system("open " + output_path + prefix + ".mp4")


def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [-0.15, -0.7, 1.0]
    # agent_state.position = [-0.15, -1.6, 1.0]
    agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    backend_cfg.enable_physics = True

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    camera_resolution = [540, 720]
    sensors = {
        "rgba_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 0.6, 0.0],  #::: fix y to be 0 later
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
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations


# [/setup]

# This is wrapped such that it can be added to a unit test
def main(make_video=True, show_video=True):
    if make_video:
        if not os.path.exists(output_path):
            os.mkdir(output_path)

    # [initialize]
    # create the simulator
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)

    # [/initialize]

    # [basics]

    # load some object templates from configuration files
    sphere_template_id = sim.load_object_configs(
        str(os.path.join(data_path, "test_assets/objects/sphere"))
    )[0]

    # add an object to the scene
    id_1 = sim.add_object(sphere_template_id)
    sim.set_translation(np.array([2.50, 0, 0.2]), id_1)

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        make_video_cv2(observations, prefix="sim_basics", open_vid=show_video)

    # [/basics]

    remove_all_objects(sim)

    # [dynamic_control]

    observations = []
    # search for an object template by key sub-string
    cheezit_template_handle = sim.get_template_handles("data/objects/cheezit")[0]
    box_positions = [
        np.array([2.39, -0.37, 0]),
        np.array([2.39, -0.64, 0]),
        np.array([2.39, -0.91, 0]),
        np.array([2.39, -0.64, -0.22]),
        np.array([2.39, -0.64, 0.22]),
    ]
    box_orientation = mn.Quaternion.rotation(
        mn.Rad(math.pi / 2.0), np.array([-1.0, 0, 0])
    )
    # instance and place the boxes
    box_ids = []
    for b in range(5):
        box_ids.append(sim.add_object_by_handle(cheezit_template_handle))
        sim.set_translation(box_positions[b], box_ids[b])
        sim.set_rotation(box_orientation, box_ids[b])

    # get the object's initialization attributes (all boxes initialized with same mass)
    object_init_template = sim.get_object_initialization_template(box_ids[0])
    # anti-gravity force f=m(-g)
    anti_grav_force = -1.0 * sim.get_gravity() * object_init_template.get_mass()

    # throw a sphere at the boxes from the agent position
    sphere_template = sim.get_object_template(sphere_template_id)
    sphere_template.set_scale(np.array([0.5, 0.5, 0.5]))
    sphere_id = sim.add_object(sphere_template_id)
    sim.set_translation(
        sim.agents[0].get_state().position + np.array([0, 1.0, 0]), sphere_id
    )
    # get the vector from the sphere to a box
    target_direction = sim.get_translation(box_ids[0]) - sim.get_translation(sphere_id)
    # apply an initial velocity for one step
    sim.set_linear_velocity(target_direction * 5, sphere_id)
    sim.set_angular_velocity(np.array([0, -1.0, 0]), sphere_id)

    start_time = sim.get_world_time()
    dt = 3.0
    while sim.get_world_time() < start_time + dt:
        # set forces/torques before stepping the world
        for box_id in box_ids:
            sim.apply_force(anti_grav_force, np.array([0, 0.0, 0]), box_id)
            sim.apply_torque(np.array([0, 0.01, 0]), box_id)
        sim.step_physics(1.0 / 60.0)
        observations.append(sim.get_sensor_observations())

    if make_video:
        make_video_cv2(observations, prefix="dynamic_control", open_vid=show_video)

    # [/dynamic_control]

    remove_all_objects(sim)

    # [kinematic_interactions]

    chefcan_template_handle = sim.get_template_handles("data/objects/chefcan")[0]
    id_1 = sim.add_object_by_handle(chefcan_template_handle)
    sim.set_translation(np.array([2.4, -0.64, 0]), id_1)
    # set one object to kinematic
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, id_1)

    # drop some dynamic objects
    id_2 = sim.add_object_by_handle(chefcan_template_handle)
    sim.set_translation(np.array([2.4, -0.64, 0.28]), id_2)
    id_3 = sim.add_object_by_handle(chefcan_template_handle)
    sim.set_translation(np.array([2.4, -0.64, -0.28]), id_3)
    id_4 = sim.add_object_by_handle(chefcan_template_handle)
    sim.set_translation(np.array([2.4, -0.3, 0]), id_4)

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=True)

    if make_video:
        make_video_cv2(
            observations, prefix="kinematic_interactions", open_vid=show_video
        )

    # [/kinematic_interactions]

    remove_all_objects(sim)

    # [kinematic_update]
    observations = []

    clamp_template_handle = sim.get_template_handles("data/objects/largeclamp")[0]
    id_1 = sim.add_object_by_handle(clamp_template_handle)
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, id_1)
    sim.set_translation(np.array([0.8, 0, 0.5]), id_1)

    start_time = sim.get_world_time()
    dt = 1.0
    while sim.get_world_time() < start_time + dt:
        # manually control the object's kinematic state
        sim.set_translation(sim.get_translation(id_1) + np.array([0, 0, 0.01]), id_1)
        sim.set_rotation(
            mn.Quaternion.rotation(mn.Rad(0.05), np.array([-1.0, 0, 0]))
            * sim.get_rotation(id_1),
            id_1,
        )
        sim.step_physics(1.0 / 60.0)
        observations.append(sim.get_sensor_observations())

    if make_video:
        make_video_cv2(observations, prefix="kinematic_update", open_vid=show_video)

    # [/kinematic_update]

    # [velocity_control]

    # get object VelocityControl structure and setup control
    vel_control = sim.get_object_velocity_control(id_1)
    vel_control.linear_velocity = np.array([0, 0, -1.0])
    vel_control.angular_velocity = np.array([4.0, 0, 0])
    vel_control.controlling_lin_vel = True
    vel_control.controlling_ang_vel = True

    observations = simulate(sim, dt=1.0, get_frames=True)

    # reverse linear direction
    vel_control.linear_velocity = np.array([0, 0, 1.0])

    observations += simulate(sim, dt=1.0, get_frames=True)

    make_video_cv2(observations, prefix="velocity_control", open_vid=True)

    # [/velocity_control]

    # [local_velocity_control]

    vel_control.linear_velocity = np.array([0, 0, 2.3])
    vel_control.angular_velocity = np.array([-4.3, 0.0, 0])
    vel_control.lin_vel_is_local = True
    vel_control.ang_vel_is_local = True

    observations = simulate(sim, dt=1.5, get_frames=True)

    # video rendering
    make_video_cv2(observations, prefix="local_velocity_control", open_vid=True)

    # [/local_velocity_control]

    # [embodied_agent]

    # load the lobot_merged asset
    locobot_template_id = sim.load_object_configs(
        str(os.path.join(data_path, "objects/locobot_merged"))
    )[0]

    # add robot object to the scene with the agent/camera SceneNode attached
    id_1 = sim.add_object(locobot_template_id, sim.agents[0].scene_node)
    sim.set_translation(np.array([1.75, -1.02, 0.4]), id_1)

    vel_control = sim.get_object_velocity_control(id_1)
    vel_control.linear_velocity = np.array([0, 0, -1.0])
    vel_control.angular_velocity = np.array([0.0, 2.0, 0])

    # simulate robot dropping into place
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    vel_control.controlling_lin_vel = True
    vel_control.controlling_ang_vel = True
    vel_control.lin_vel_is_local = True
    vel_control.ang_vel_is_local = True

    # simulate forward and turn
    observations += simulate(sim, dt=1.0, get_frames=make_video)

    vel_control.controlling_lin_vel = False
    vel_control.angular_velocity = np.array([0.0, 1.0, 0])

    # simulate turn only
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    vel_control.angular_velocity = np.array([0.0, 0.0, 0])
    vel_control.controlling_lin_vel = True
    vel_control.controlling_ang_vel = True

    # simulate forward only with damped angular velocity (reset angular velocity to 0 after each step)
    observations += simulate(sim, dt=1.0, get_frames=make_video)

    vel_control.angular_velocity = np.array([0.0, -1.25, 0])

    # simulate forward and turn
    observations += simulate(sim, dt=2.0, get_frames=make_video)

    vel_control.controlling_ang_vel = False
    vel_control.controlling_lin_vel = False

    # simulate settling
    observations += simulate(sim, dt=3.0, get_frames=make_video)

    # video rendering with embedded 1st person view
    make_video_cv2(observations, prefix="robot_control", open_vid=True, multi_obs=True)

    # [/embodied_agent]


if __name__ == "__main__":
    main(make_video=True, show_video=True)
