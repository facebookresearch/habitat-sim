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
output_path = os.path.join(dir_path, "URDF_robotics_tutorial_output/")


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
            # embed the 1st person RBG frame into the 3rd person frame
            bgr_im_3rd_person = ob["rgba_camera_3rdperson"][..., 0:3][..., ::-1]
            resized_1st_person_rgb = cv2.resize(
                bgr_im_1st_person, thumb_size, interpolation=cv2.INTER_AREA
            )
            x_offset = 50
            y_offset_rgb = 50
            bgr_im_3rd_person[
                y_offset_rgb - 1 : y_offset_rgb + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            bgr_im_3rd_person[
                y_offset_rgb : y_offset_rgb + resized_1st_person_rgb.shape[0],
                x_offset : x_offset + resized_1st_person_rgb.shape[1],
            ] = resized_1st_person_rgb

            # embed the 1st person DEPTH frame into the 3rd person frame
            # manually normalize depth into [0, 1] so that images are always consistent
            d_im = np.clip(ob["depth_camera_1stperson"], 0, 10)
            d_im /= 10.0
            bgr_d_im = cv2.cvtColor((d_im * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            resized_1st_person_depth = cv2.resize(
                bgr_d_im, thumb_size, interpolation=cv2.INTER_AREA
            )
            y_offset_d = y_offset_rgb + 10 + thumb_size[1]
            bgr_im_3rd_person[
                y_offset_d - 1 : y_offset_d + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            bgr_im_3rd_person[
                y_offset_d : y_offset_d + resized_1st_person_depth.shape[0],
                x_offset : x_offset + resized_1st_person_depth.shape[1],
            ] = resized_1st_person_depth

            # write the video frame
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

    urdf_files = {
        "aliengo": os.path.join(
            data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"
        ),
        "iiwa": os.path.join(
            data_path, "test_assets/urdf/kuka_iiwa/model_free_base.urdf"
        ),
        "locobot": os.path.join(
            data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"
        ),
        "locobot_light": os.path.join(
            data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"
        ),
    }

    # load a URDF file
    robot_file = urdf_files["aliengo"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file)

    # place the robot root state relative to the agent
    local_base_pos = np.array([0.0, 0.5, -2.0])
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    base_transform = mn.Matrix4.rotation(mn.Rad(-1.56), mn.Vector3(1.0, 0, 0))
    base_transform.translation = agent_transform.transform_point(local_base_pos)
    sim.set_articulated_object_root_state(robot_id, base_transform)

    # set a better initial joint state for the aliengo
    if robot_file == urdf_files["aliengo"]:
        pose = sim.get_articulated_object_positions(robot_id)
        calfDofs = [2, 5, 8, 11]
        for dof in calfDofs:
            pose[dof] = -1.0
            pose[dof - 1] = 0.45
            # also set a thigh
        sim.set_articulated_object_positions(robot_id, pose)

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    # get/set forces and velocities
    tau = sim.get_articulated_object_forces(robot_id)
    sim.set_articulated_object_forces(robot_id, tau)

    vel = sim.get_articulated_object_velocities(robot_id)
    sim.set_articulated_object_velocities(robot_id, vel)

    # reset the object state (sets dof positions/velocities/forces to 0, recomputes forward kinematics, udpate collision state)
    sim.reset_articulated_object(robot_id)
    # note: reset does not change the robot base state, do this manually
    sim.set_articulated_object_root_state(robot_id, base_transform)

    # set sleeping ON
    sim.set_articulated_object_sleep(robot_id, True)
    assert sim.get_articulated_object_sleep(robot_id) is True

    observations += simulate(sim, dt=1.0, get_frames=make_video)

    # set sleeping OFF
    sim.set_articulated_object_sleep(robot_id, False)
    assert sim.get_articulated_object_sleep(robot_id) is False

    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # get/set motiontype (KINEMATIC vs. DYNAMIC)
    sim.set_articulated_object_motion_type(
        robot_id, habitat_sim.physics.MotionType.KINEMATIC
    )
    assert (
        sim.get_articulated_object_motion_type(robot_id)
        == habitat_sim.physics.MotionType.KINEMATIC
    )

    # reset the object state (sets dof positions/velocities/forces to 0, recomputes forward kinematics, udpate collision state)
    sim.reset_articulated_object(robot_id)
    # note: reset does not change the robot base state, do this manually
    sim.set_articulated_object_root_state(robot_id, base_transform)

    # get rigid state of robot links and show proxy object at each link COM
    cube_id = sim.add_object_by_handle(sim.get_template_handles("cube")[0])
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cube_id)
    num_links = sim.get_num_articulated_links(robot_id)
    for link_id in range(num_links):
        link_rigid_state = sim.get_articulated_link_rigid_state(robot_id, link_id)
        sim.set_translation(link_rigid_state.translation, cube_id)
        sim.set_rotation(link_rigid_state.rotation, cube_id)
        observations += simulate(sim, dt=0.5, get_frames=make_video)
    sim.remove_object(cube_id)

    sim.set_articulated_object_motion_type(
        robot_id, habitat_sim.physics.MotionType.DYNAMIC
    )
    assert (
        sim.get_articulated_object_motion_type(robot_id)
        == habitat_sim.physics.MotionType.DYNAMIC
    )

    if make_video:
        make_video_cv2(observations, prefix="URDF_basics", open_vid=show_video)

    # [/basics]

    remove_all_objects(sim)  # TODO: remove all robots also


if __name__ == "__main__":
    main(make_video=True, show_video=True)
