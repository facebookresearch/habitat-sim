# [setup]

import os

import magnum as mn
import matplotlib.pyplot as plt
import numpy as np

import habitat_sim

# import habitat_sim.utils.common as ut
import habitat_sim.utils.viz_utils as vut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "URDF_robotics_tutorial_output/")


def get_process_memory_usage():
    import os

    import psutil

    return psutil.Process(os.getpid()).memory_info().rss / 1024 ** 2


def remove_all_objects(sim):
    for ob_id in sim.get_existing_object_ids():
        sim.remove_object(ob_id)
    for ob_id in sim.get_existing_articulated_object_ids():
        sim.remove_articulated_object(ob_id)


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [-0.15, -0.1, 1.0]
    # agent_state.position = [-0.15, -1.6, 1.0]
    agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    backend_cfg.enable_physics = True

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    camera_resolution = [540, 720]
    sensors = {
        "rgba_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
        "depth_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": camera_resolution,
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = habitat_sim.CameraSensorSpec()
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


def place_robot_from_agent(
    sim,
    robot_id,
    angle_correction=-1.56,
    local_base_pos=None,
):
    if local_base_pos is None:
        local_base_pos = np.array([0.0, -0.1, -2.0])
    # place the robot root state relative to the agent
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    base_transform = mn.Matrix4.rotation(
        mn.Rad(angle_correction), mn.Vector3(1.0, 0, 0)
    )
    base_transform.translation = agent_transform.transform_point(local_base_pos)
    sim.set_articulated_object_root_state(robot_id, base_transform)


urdf_files = {
    "aliengo": os.path.join(data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"),
    "iiwa": os.path.join(data_path, "test_assets/urdf/kuka_iiwa/model_free_base.urdf"),
    "locobot": os.path.join(data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"),
    "locobot_light": os.path.join(
        data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"
    ),
}


def test_urdf_memory():
    # test for memory leaks related to adding/removing AO's from URDF
    # process_memory_tracking = [get_process_memory_usage()]
    process_memory_tracking = []

    # create the simulator
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)

    # process_memory_tracking.append(get_process_memory_usage())

    # load a URDF file
    robot_key = "fetch"
    robot_file = urdf_files[robot_key]
    for _sample in range(1000):

        robot_id = sim.add_articulated_object_from_urdf(robot_file)
        process_memory_tracking.append(get_process_memory_usage())
        sim.remove_articulated_object(robot_id)
        process_memory_tracking.append(get_process_memory_usage())

    # graph the results
    plt.plot(process_memory_tracking)
    plt.title("Memory (MB) at add/remove URDF. (" + str(robot_key) + ")")
    plt.xlabel("Query #")
    plt.ylabel("Process Memory (MB)")
    plt.show()


def demo_contact_profile():
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        place_agent(sim)
        observations = []

        # add a robot to the scene
        robot_file = urdf_files["aliengo"]
        robot_id = sim.add_articulated_object_from_urdf(robot_file)

        # gets nothing because physics has not stepped yet
        print(sim.get_physics_step_collision_summary())

        # gets nothing because no active collisions yet
        sim.step_physics(0.1)
        print(sim.get_physics_step_collision_summary())

        # give time for the robot to hit the ground then check contacts
        sim.step_physics(1.0)
        print("Step Collision Summary:")
        print(sim.get_physics_step_collision_summary())

        # now add two colliding robots and run discrete collision detection
        sim.remove_articulated_object(robot_id)
        robot_id1 = sim.add_articulated_object_from_urdf(robot_file)
        robot_id2 = sim.add_articulated_object_from_urdf(robot_file)
        place_robot_from_agent(sim, robot_id1)
        place_robot_from_agent(
            sim, robot_id2, local_base_pos=np.array([0.15, -0.1, -2.0])
        )
        sim.perform_discrete_collision_detection()
        print("Step Collision Summary:")
        print(sim.get_physics_step_collision_summary())
        print(
            "Num overlapping pairs: "
            + str(sim.get_physics_num_active_overlapping_pairs())
        )
        print(
            "Num active contact points: "
            + str(sim.get_physics_num_active_contact_points())
        )
        contact_points = sim.get_physics_contact_points()
        print("Active contact points: ")
        for cp_ix, cp in enumerate(contact_points):
            print(" Contact Point " + str(cp_ix) + ":")
            print("     object_id_a = " + str(cp.object_id_a))
            print("     object_id_b = " + str(cp.object_id_b))
            print("     link_id_a = " + str(cp.link_id_a))
            print("     link_id_b = " + str(cp.link_id_b))
            print("     position_on_a_in_ws = " + str(cp.position_on_a_in_ws))
            print("     position_on_b_in_ws = " + str(cp.position_on_b_in_ws))
            print(
                "     contact_normal_on_b_in_ws = " + str(cp.contact_normal_on_b_in_ws)
            )
            print("     contact_distance = " + str(cp.contact_distance))
            print("     normal_force = " + str(cp.normal_force))
            print("     linear_friction_force1 = " + str(cp.linear_friction_force1))
            print("     linear_friction_force2 = " + str(cp.linear_friction_force2))
            print(
                "     linear_friction_direction1 = "
                + str(cp.linear_friction_direction1)
            )
            print(
                "     linear_friction_direction2 = "
                + str(cp.linear_friction_direction2)
            )
            print("     is_active = " + str(cp.is_active))

        observations.append(sim.get_sensor_observations())
        # TODO: visualize the contact points
        im = vut.observation_to_image(
            observations[-1]["rgba_camera_1stperson"], "color"
        )
        im.show()


def test_constraints(make_video=True, show_video=True):
    # [initialize]
    # create the simulator
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        place_agent(sim)
        observations = []

        # load a URDF file
        robot_file = urdf_files["aliengo"]
        robot_id = sim.add_articulated_object_from_urdf(robot_file)
        ef_link_id = 16  # foot = 16, TODO: base = -1
        ef_link2_id = 12
        iiwa_ef_link = 6

        # add a constraint vis object
        obj_mgr = sim.get_object_template_manager()
        sphere_id = sim.add_object_by_handle(obj_mgr.get_template_handles("sphere")[0])
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, sphere_id)
        sim.set_object_is_collidable(False, sphere_id)

        for test_case in range(6):
            sim.reset_articulated_object(robot_id)
            place_robot_from_agent(sim, robot_id)

            # Test constraint types:
            if test_case == 0:
                # - AO -> world
                # should constrain to the center of the sphere
                link_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id, ef_link_id
                )
                sim.set_translation(link_rigid_state.translation, sphere_id)
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id=robot_id,
                    link_id=ef_link_id,
                    global_constraint_point=link_rigid_state.translation,
                )
                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
            elif test_case == 1:
                # - AO -> world w/ offset
                # should constrain to the boundary of the sphere
                link_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id, ef_link_id
                )
                link_offset = mn.Vector3(0, 0, -0.1)
                global_constraint_position = link_rigid_state.translation
                sim.set_translation(global_constraint_position, sphere_id)
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id=robot_id,
                    link_id=ef_link_id,
                    link_offset=link_offset,
                    global_constraint_point=global_constraint_position,
                )
                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
            elif test_case == 2:
                # - AO -> AO (w/ offsets)
                robot_id2 = sim.add_articulated_object_from_urdf(robot_file)
                place_robot_from_agent(
                    sim=sim,
                    robot_id=robot_id2,
                    local_base_pos=np.array([0.35, -0.1, -2.0]),
                )
                # attach the agents' feet together
                link_b_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id2, ef_link_id
                )
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id_a=robot_id,
                    link_id_a=ef_link_id,
                    offset_a=mn.Vector3(),
                    object_id_b=robot_id2,
                    link_id_b=ef_link_id,
                    offset_b=mn.Vector3(),
                )

                # constrain 1st robot in the air by other foot
                link_a2_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id, ef_link2_id
                )
                global_constraint_position = (
                    link_a2_rigid_state.translation + mn.Vector3(0, 1.5, 0)
                )
                sim.set_translation(global_constraint_position, sphere_id)
                # note: increase max impulse: the combined weight of the robots is greater than the default impulse correction (2)
                constraint_id2 = sim.create_articulated_p2p_constraint(
                    object_id=robot_id,
                    link_id=ef_link2_id,
                    link_offset=mn.Vector3(),
                    global_constraint_point=global_constraint_position,
                    max_impulse=6,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_constraint(constraint_id2)
                sim.remove_articulated_object(robot_id2)
            elif test_case == 3:
                # - AO -> AO (global)
                robot_id2 = sim.add_articulated_object_from_urdf(
                    urdf_files["iiwa"], True
                )
                place_robot_from_agent(
                    sim=sim,
                    robot_id=robot_id2,
                    local_base_pos=np.array([0.35, -0.4, -2.0]),
                )
                jm_settings = habitat_sim.physics.JointMotorSettings()
                jm_settings.position_gain = 2.0
                sim.create_motors_for_all_dofs(robot_id2, jm_settings)
                # TODO: not a great test, could be a better setup
                # attach two agent feet to the iiwa end effector
                link_b_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id2, iiwa_ef_link
                )
                global_constraint_position = link_b_rigid_state.translation
                sim.set_translation(global_constraint_position, sphere_id)
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id_a=robot_id,
                    link_id_a=ef_link_id,
                    object_id_b=robot_id2,
                    link_id_b=iiwa_ef_link,
                    global_constraint_point=global_constraint_position,
                    max_impulse=4,
                )
                constraint_id2 = sim.create_articulated_p2p_constraint(
                    object_id_a=robot_id,
                    link_id_a=ef_link2_id,
                    object_id_b=robot_id2,
                    link_id_b=iiwa_ef_link,
                    global_constraint_point=global_constraint_position,
                    max_impulse=4,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_constraint(constraint_id2)
                sim.remove_articulated_object(robot_id2)
            elif test_case == 4:
                # - AO -> rigid

                # tilt the camera down
                prev_state = sim.get_agent(0).scene_node.rotation
                sim.get_agent(0).scene_node.rotation = (
                    mn.Quaternion.rotation(
                        mn.Rad(-0.4), prev_state.transform_vector(mn.Vector3(1.0, 0, 0))
                    )
                    * prev_state
                )

                # attach an active sphere to one robot foot w/ pivot at the object center
                active_sphere_id = sim.add_object_by_handle(
                    obj_mgr.get_template_handles("sphere")[0]
                )
                link_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id, ef_link_id
                )
                link2_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id, ef_link2_id
                )
                sim.set_translation(
                    link2_rigid_state.translation + mn.Vector3(0, -0.1, 0),
                    active_sphere_id,
                )
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id_a=robot_id,
                    link_id=ef_link2_id,
                    object_id_b=active_sphere_id,
                    max_impulse=4,
                )
                # attach the visual sphere to another robot foot w/ pivots
                sim.set_object_motion_type(
                    habitat_sim.physics.MotionType.DYNAMIC, sphere_id
                )
                constraint_id2 = sim.create_articulated_p2p_constraint(
                    object_id_a=robot_id,
                    link_id=ef_link_id,
                    object_id_b=sphere_id,
                    pivot_a=mn.Vector3(0.1, 0, 0),
                    pivot_b=mn.Vector3(-0.1, 0, 0),
                    max_impulse=4,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_constraint(constraint_id2)
                sim.set_object_motion_type(
                    habitat_sim.physics.MotionType.KINEMATIC, sphere_id
                )
                sim.remove_object(active_sphere_id)

                sim.get_agent(0).scene_node.rotation = prev_state
            elif test_case == 5:
                # - AO -> rigid (fixed) TODO: not working as expected

                # tilt the camera down
                prev_state = sim.get_agent(0).scene_node.rotation
                sim.get_agent(0).scene_node.rotation = (
                    mn.Quaternion.rotation(
                        mn.Rad(-0.4), prev_state.transform_vector(mn.Vector3(1.0, 0, 0))
                    )
                    * prev_state
                )

                # attach an active sphere to one robot foot w/ pivot at the object center
                active_sphere_id = sim.add_object_by_handle(
                    obj_mgr.get_template_handles("sphere")[0]
                )
                link2_rigid_state = sim.get_articulated_link_rigid_state(
                    robot_id, ef_link2_id
                )
                sim.set_translation(
                    link2_rigid_state.translation + mn.Vector3(0, -0.15, 0),
                    active_sphere_id,
                )
                constraint_id = sim.create_articulated_fixed_constraint(
                    object_id_a=robot_id,
                    link_id=ef_link2_id,
                    object_id_b=active_sphere_id,
                    max_impulse=4,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_object(active_sphere_id)

                sim.get_agent(0).scene_node.rotation = prev_state

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "test_constraints",
                open_vid=show_video,
            )


# This is wrapped such that it can be added to a unit test
def main(make_video=True, show_video=True):

    # [initialize]
    # create the simulator
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        place_agent(sim)
        observations = []

        # load a URDF file
        robot_file = urdf_files["iiwa"]
        robot_id = sim.add_articulated_object_from_urdf(robot_file)

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot_id)

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        for iteration in range(1, 4):
            # remove the object
            sim.remove_articulated_object(robot_id)

            # load a URDF file
            robot_file = urdf_files["aliengo"]
            urdf_global_scale = iteration / 2.0
            robot_id = sim.add_articulated_object_from_urdf(
                robot_file, False, urdf_global_scale
            )
            print("Scaled URDF by " + str(urdf_global_scale))

            # place the robot root state relative to the agent
            place_robot_from_agent(sim, robot_id)

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
        place_robot_from_agent(sim, robot_id)

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
        place_robot_from_agent(sim, robot_id)

        # get rigid state of robot links and show proxy object at each link COM
        obj_mgr = sim.get_object_template_manager()
        cube_id = sim.add_object_by_handle(obj_mgr.get_template_handles("cube")[0])
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cube_id)
        sim.set_object_is_collidable(False, cube_id)
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
        place_robot_from_agent(sim, robot_id, -3.14)

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
            "existing motors after create (motor_id -> dof): "
            + str(existing_joint_motors)
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
        joint_motor_settings = habitat_sim.physics.JointMotorSettings(
            0.5, 1.0, 0, 0, 1.0
        )
        dofs_to_motor_ids = sim.create_motors_for_all_dofs(
            robot_id,
            joint_motor_settings,  # (optional) motor settings, if not provided will be default (no gains)
        )
        print("New motors (motor_ids -> dofs): " + str(dofs_to_motor_ids))

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # remove all motors
        existing_joint_motors = sim.get_existing_joint_motors(robot_id)
        print(
            "All motors (motor_id -> dof) before removal: " + str(existing_joint_motors)
        )
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
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.display
    display = args.display
    make_video = args.make_video

    if make_video and not os.path.exists(output_path):
        os.mkdir(output_path)

    main(make_video, show_video)
    test_constraints(make_video, show_video)
    # test_urdf_memory()
    # demo_contact_profile()
