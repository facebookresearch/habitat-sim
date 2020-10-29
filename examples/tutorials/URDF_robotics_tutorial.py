# [setup]

import os

import magnum as mn
import numpy as np

import habitat_sim

# import habitat_sim.utils.common as ut
import habitat_sim.utils.viz_utils as vut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "URDF_robotics_tutorial_output/")


def remove_all_objects(sim):
    for ob_id in sim.get_existing_object_ids():
        sim.remove_object(ob_id)
    for ob_id in sim.get_existing_articulated_object_ids():
        sim.remove_articulated_object(ob_id)


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
    if make_video and not os.path.exists(output_path):
        os.mkdir(output_path)

    # [initialize]
    # create the simulator
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)
    observations = []

    # [/initialize]

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

    # [basics]

    # load a URDF file
    robot_file = urdf_files["iiwa"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file)

    # place the robot root state relative to the agent
    local_base_pos = np.array([0.0, 0.5, -2.0])
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    base_transform = mn.Matrix4.rotation(mn.Rad(-1.56), mn.Vector3(1.0, 0, 0))
    base_transform.translation = agent_transform.transform_point(local_base_pos)
    sim.set_articulated_object_root_state(robot_id, base_transform)

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # remove the object
    sim.remove_articulated_object(robot_id)

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
    observations += simulate(sim, dt=1.5, get_frames=make_video)

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
    obj_mgr = sim.get_object_template_manager()
    cube_id = sim.add_object_by_handle(obj_mgr.get_template_handles("cube")[0])
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cube_id)
    num_links = sim.get_num_articulated_links(robot_id)
    for link_id in range(num_links):
        link_rigid_state = sim.get_articulated_link_rigid_state(robot_id, link_id)
        sim.set_translation(link_rigid_state.translation, cube_id)
        sim.set_rotation(link_rigid_state.rotation, cube_id)
        # get the link friction
        print(
            "Link "
            + str(link_id)
            + " friction coefficient = "
            + str(sim.get_articulated_link_friction(robot_id, link_id))
        )
        # Note: set this with 'sim.get_articulated_link_friction(robot_id, link_id, friction)'
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
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "URDF_basics",
            open_vid=show_video,
        )

    # clear all robots
    for robot_id in sim.get_existing_articulated_object_ids():
        sim.remove_articulated_object(robot_id)
    # [/basics]

    # [joint motors]
    observations = []

    # load a URDF file with a fixed base
    robot_file = urdf_files["iiwa"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file, True)

    # place the robot root state relative to the agent
    local_base_pos = np.array([0.0, 0.5, -2.0])
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    base_transform = mn.Matrix4.rotation(mn.Rad(-3.14), mn.Vector3(1.0, 0, 0))
    base_transform.translation = agent_transform.transform_point(local_base_pos)
    sim.set_articulated_object_root_state(robot_id, base_transform)

    # query any damping motors created by default
    existing_joint_motors = sim.get_existing_joint_motors(robot_id)
    print("default damping motors (motor_id -> dof): " + str(existing_joint_motors))

    # get the max_impulse of the damping motors
    for motor_id in existing_joint_motors:
        motor_settings = sim.get_joint_motor_settings(robot_id, motor_id)
        print(
            "   motor("
            + str(motor_id)
            + "): max_impulse = "
            + str(motor_settings.max_impulse)
        )

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # create a new velocity motor
    joint_motor_settings = habitat_sim.physics.JointMotorSettings(
        0,  # position_target
        0,  # position_gain
        1.0,  # velocity_target
        1.0,  # velocity_gain
        10.0,  # max_impulse
    )
    new_motor_id = sim.create_joint_motor(
        robot_id, 1, joint_motor_settings  # robot object id  # dof  # settings
    )
    existing_joint_motors = sim.get_existing_joint_motors(robot_id)
    print("new_motor_id: " + str(new_motor_id))
    print(
        "existing motors after create (motor_id -> dof): " + str(existing_joint_motors)
    )

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # reverse the motor velocity
    joint_motor_settings.velocity_target = -1.0
    sim.update_joint_motor(robot_id, new_motor_id, joint_motor_settings)

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # remove the new joint motor
    sim.remove_joint_motor(robot_id, new_motor_id)

    # create joint motors for all valid dofs to control a pose (1.1 for all dofs)
    joint_motor_settings = habitat_sim.physics.JointMotorSettings(0.5, 1.0, 0, 0, 1.0)
    dofs_to_motor_ids = sim.create_motors_for_all_dofs(
        robot_id,
        joint_motor_settings,  # (optional) motor settings, if not provided will be default (no gains)
    )
    print("New motors (motor_ids -> dofs): " + str(dofs_to_motor_ids))

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # remove all motors
    existing_joint_motors = sim.get_existing_joint_motors(robot_id)
    print("All motors (motor_id -> dof) before removal: " + str(existing_joint_motors))
    for motor_id in existing_joint_motors:
        sim.remove_joint_motor(robot_id, motor_id)
    print(
        "All motors (motor_id -> dof) before removal: "
        + str(sim.get_existing_joint_motors(robot_id))
    )

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "URDF_joint_motors",
            open_vid=show_video,
        )
    # [/joint motors]

    remove_all_objects(sim)


if __name__ == "__main__":
    main(make_video=True, show_video=True)
