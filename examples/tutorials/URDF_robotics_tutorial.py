# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
# [setup]

import math
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
    robot,
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
    robot.transformation = base_transform


urdf_files = {
    "aliengo": os.path.join(data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"),
    "iiwa": os.path.join(data_path, "test_assets/urdf/kuka_iiwa/model_free_base.urdf"),
    "locobot": os.path.join(data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"),
    "locobot_light": os.path.join(
        data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"
    ),
    "fridge": os.path.join(data_path, "test_assets/urdf/fridge/fridge.urdf"),
    "primitive_chain": os.path.join(data_path, "test_assets/urdf/prim_chain.urdf"),
    "amass_male": os.path.join(data_path, "test_assets/urdf/amass_male.urdf"),
}


def test_urdf_memory():
    # test for memory leaks related to adding/removing AO's from URDF
    # process_memory_tracking = [get_process_memory_usage()]
    process_memory_tracking = []

    # create the simulator
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    art_obj_mgr = sim.get_articulated_object_manager()

    # process_memory_tracking.append(get_process_memory_usage())

    # load a URDF file
    robot_key = "amass_male"
    robot_file = urdf_files[robot_key]
    for _sample in range(1000):

        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)
        process_memory_tracking.append(get_process_memory_usage())
        art_obj_mgr.remove_object_by_id(robot.object_id)
        process_memory_tracking.append(get_process_memory_usage())

    # graph the results
    plt.plot(process_memory_tracking)
    plt.title("Memory (MB) at add/remove URDF. (" + str(robot_key) + ")")
    plt.xlabel("Query #")
    plt.ylabel("Process Memory (MB)")
    plt.show(block=False)
    input("Press ENTER to continue...")
    sim.close()


def demo_contact_profile():
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        place_agent(sim)
        observations = []

        # add a robot to the scene
        robot_file = urdf_files["aliengo"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)

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
        art_obj_mgr.remove_object_by_id(robot.object_id)
        robot1 = art_obj_mgr.add_articulated_object_from_urdf(robot_file)
        robot2 = art_obj_mgr.add_articulated_object_from_urdf(robot_file)
        place_robot_from_agent(sim, robot1)
        place_robot_from_agent(sim, robot2, local_base_pos=np.array([0.15, -0.1, -2.0]))
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
        art_obj_mgr = sim.get_articulated_object_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()
        place_agent(sim)
        observations = []

        # load a URDF file
        robot_file = urdf_files["aliengo"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)
        ef_link_id = 16  # foot = 16, TODO: base = -1
        ef_link2_id = 12
        iiwa_ef_link = 6

        # add a constraint vis object
        obj_attr_mgr = sim.get_object_template_manager()
        sphere = rigid_obj_mgr.add_object_by_template_handle(
            obj_attr_mgr.get_template_handles("sphere")[0]
        )
        sphere.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        sphere.collidable = False

        for test_case in range(6):
            robot.clear_joint_states()
            place_robot_from_agent(sim, robot)

            # Test constraint types:
            if test_case == 0:
                # - AO -> world
                # should constrain to the center of the sphere
                sphere.translation = robot.get_link_scene_node(ef_link_id).translation
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id=robot.object_id,
                    link_id=ef_link_id,
                    global_constraint_point=robot.get_link_scene_node(
                        ef_link_id
                    ).translation,
                )
                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
            elif test_case == 1:
                # - AO -> world w/ offset
                # should constrain to the boundary of the sphere
                global_constraint_position = robot.get_link_scene_node(
                    ef_link_id
                ).translation
                sphere.translation = global_constraint_position
                link_offset = mn.Vector3(0, 0, -0.1)
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id=robot.object_id,
                    link_id=ef_link_id,
                    link_offset=link_offset,
                    global_constraint_point=global_constraint_position,
                )
                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
            elif test_case == 2:
                # - AO -> AO (w/ offsets)
                robot2 = art_obj_mgr.add_articulated_object_from_urdf(robot_file)
                place_robot_from_agent(
                    sim=sim,
                    robot=robot2,
                    local_base_pos=np.array([0.35, -0.1, -2.0]),
                )
                # attach the agents' feet together
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id_a=robot.object_id,
                    link_id_a=ef_link_id,
                    offset_a=mn.Vector3(),
                    object_id_b=robot2.object_id,
                    link_id_b=ef_link_id,
                    offset_b=mn.Vector3(),
                )

                # constrain 1st robot in the air by other foot
                global_constraint_position = robot.get_link_scene_node(
                    ef_link2_id
                ).translation + mn.Vector3(0, 1.5, 0)
                sphere.translation = global_constraint_position
                # note: increase max impulse: the combined weight of the robots is greater than the default impulse correction (2)
                constraint_id2 = sim.create_articulated_p2p_constraint(
                    object_id=robot.object_id,
                    link_id=ef_link2_id,
                    link_offset=mn.Vector3(),
                    global_constraint_point=global_constraint_position,
                    max_impulse=6,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_constraint(constraint_id2)
                art_obj_mgr.remove_object_by_id(robot2.object_id)
            elif test_case == 3:
                # - AO -> AO (global)
                robot2 = art_obj_mgr.add_articulated_object_from_urdf(
                    urdf_files["iiwa"], True
                )
                place_robot_from_agent(
                    sim=sim,
                    robot=robot2,
                    local_base_pos=np.array([0.35, -0.4, -2.0]),
                )
                jm_settings = habitat_sim.physics.JointMotorSettings()
                jm_settings.position_gain = 2.0
                robot2.create_all_motors(jm_settings)
                # TODO: not a great test, could be a better setup
                # attach two agent feet to the iiwa end effector

                global_constraint_position = robot2.get_link_scene_node(
                    iiwa_ef_link
                ).translation
                sphere.translation = global_constraint_position
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id_a=robot.object_id,
                    link_id_a=ef_link_id,
                    object_id_b=robot2.object_id,
                    link_id_b=iiwa_ef_link,
                    global_constraint_point=global_constraint_position,
                    max_impulse=4,
                )
                constraint_id2 = sim.create_articulated_p2p_constraint(
                    object_id_a=robot.object_id,
                    link_id_a=ef_link2_id,
                    object_id_b=robot2.object_id,
                    link_id_b=iiwa_ef_link,
                    global_constraint_point=global_constraint_position,
                    max_impulse=4,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_constraint(constraint_id2)
                art_obj_mgr.remove_object_by_id(robot2.object_id)
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
                sphere2 = rigid_obj_mgr.add_object_by_template_handle(
                    obj_attr_mgr.get_template_handles("sphere")[0]
                )
                sphere2.translation = robot.get_link_scene_node(
                    ef_link2_id
                ).translation + mn.Vector3(0, -0.1, 0)
                constraint_id = sim.create_articulated_p2p_constraint(
                    object_id_a=robot.object_id,
                    link_id=ef_link2_id,
                    object_id_b=sphere2.object_id,
                    max_impulse=4,
                )

                # attach the visual sphere to another robot foot w/ pivots
                sphere.motion_type = habitat_sim.physics.MotionType.DYNAMIC
                constraint_id2 = sim.create_articulated_p2p_constraint(
                    object_id_a=robot.object_id,
                    link_id=ef_link_id,
                    object_id_b=sphere.object_id,
                    pivot_a=mn.Vector3(0.1, 0, 0),
                    pivot_b=mn.Vector3(-0.1, 0, 0),
                    max_impulse=4,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                sim.remove_constraint(constraint_id2)
                sphere.motion_type = habitat_sim.physics.MotionType.KINEMATIC
                rigid_obj_mgr.remove_object_by_id(sphere2.object_id)

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
                sphere2 = rigid_obj_mgr.add_object_by_template_handle(
                    obj_attr_mgr.get_template_handles("sphere")[0]
                )
                sphere2.translation = robot.get_link_scene_node(
                    ef_link2_id
                ).translation + mn.Vector3(0, -0.15, 0)
                constraint_id = sim.create_articulated_fixed_constraint(
                    object_id_a=robot.object_id,
                    link_id=ef_link2_id,
                    object_id_b=sphere2.object_id,
                    max_impulse=4,
                )

                observations += simulate(sim, dt=3.0, get_frames=make_video)
                sim.remove_constraint(constraint_id)
                rigid_obj_mgr.remove_object_by_id(sphere2.object_id)

                sim.get_agent(0).scene_node.rotation = prev_state

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "test_constraints",
                open_vid=show_video,
            )


def test_prim_chain(make_video=True, show_video=True):
    # load a URDF with visual and collision geometry constructed from all supported primitive types

    cfg = make_configuration()
    observations = []
    with habitat_sim.Simulator(cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        place_agent(sim)

        # load a URDF file
        robot_file = urdf_files["primitive_chain"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot, angle_correction=0)

        # simulate object settling
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "primtive_URDF",
                open_vid=show_video,
            )


# add a fridge to the world and recompute the navmesh in several configurations
def test_ao_recompute_navmesh(make_video=True, show_video=True):
    # create the simulator
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        place_agent(sim)

        # tilt the camera down
        prev_state = sim.get_agent(0).scene_node.rotation
        sim.get_agent(0).scene_node.rotation = (
            mn.Quaternion.rotation(
                mn.Rad(-0.4), prev_state.transform_vector(mn.Vector3(1.0, 0, 0))
            )
            * prev_state
        )

        # turn on navmesh vis
        sim.navmesh_visualization = True

        observations = []

        # load a URDF file
        robot_file = urdf_files["fridge"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot, angle_correction=0)

        root_transform = robot.transformation
        R = mn.Matrix4.rotation(mn.Rad(math.pi), mn.Vector3(0, 1.0, 0))
        root_transform = mn.Matrix4.from_(
            R.rotation().__matmul__(root_transform.rotation()),
            root_transform.translation + mn.Vector3(0, 0.45, 0),
        )
        robot.transformation = root_transform

        # simulate object settling
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # make object STATIC and recompute navmesh
        robot.motion_type = habitat_sim.physics.MotionType.STATIC

        observations += simulate(sim, dt=1, get_frames=make_video)
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
        observations += simulate(sim, dt=1, get_frames=make_video)

        # open a door and recompute navmesh
        pose = robot.joint_positions
        pose[0] = 2.0
        robot.joint_positions = pose
        sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
        observations += simulate(sim, dt=1, get_frames=make_video)

        # close a door slightly and recompute navmesh
        pose = robot.joint_positions
        pose[0] = 1.0
        robot.joint_positions = pose
        sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
        observations += simulate(sim, dt=1, get_frames=make_video)

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "URDF_navmesh",
                open_vid=show_video,
            )


def test_spherical_joints(make_video=True, show_video=True):
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        place_agent(sim)
        observations = []

        # load a URDF file
        robot_file = urdf_files["amass_male"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot, angle_correction=0)

        # tilt the camera down
        prev_state = sim.get_agent(0).scene_node.rotation
        sim.get_agent(0).scene_node.rotation = (
            mn.Quaternion.rotation(
                mn.Rad(-0.4), prev_state.transform_vector(mn.Vector3(1.0, 0, 0))
            )
            * prev_state
        )

        # query any damping motors created by default
        existing_joint_motors = robot.get_existing_joint_motor_ids()
        print("default damping motors (motor_id -> dof): " + str(existing_joint_motors))

        # update the of the damping motors to hold pose
        for motor_id in existing_joint_motors:
            motor_settings = robot.get_joint_motor_settings(motor_id)
            motor_settings.max_impulse = 100
            robot.update_joint_motor(motor_id, motor_settings)

        # simulate
        observations += simulate(sim, dt=5, get_frames=make_video)

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "URDF_spherical_joint_motors",
                open_vid=show_video,
            )


# This is wrapped such that it can be added to a unit test
def main(make_video=True, show_video=True):

    # [initialize]
    # create the simulator
    cfg = make_configuration()
    with habitat_sim.Simulator(cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()
        place_agent(sim)
        observations = []

        # load a URDF file
        robot_file = urdf_files["iiwa"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file)

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot)

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        for iteration in range(1, 4):
            # remove the object
            art_obj_mgr.remove_object_by_id(robot.object_id)

            # load a URDF file
            robot_file = urdf_files["aliengo"]
            urdf_global_scale = iteration / 2.0
            robot = art_obj_mgr.add_articulated_object_from_urdf(
                robot_file, False, urdf_global_scale
            )
            print("Scaled URDF by " + str(urdf_global_scale))

            # place the robot root state relative to the agent
            place_robot_from_agent(sim, robot)

            # set a better initial joint state for the aliengo
            if robot_file == urdf_files["aliengo"]:
                pose = robot.joint_positions
                calfDofs = [2, 5, 8, 11]
                for dof in calfDofs:
                    pose[dof] = -1.0
                    pose[dof - 1] = 0.45
                    # also set a thigh
                robot.joint_positions = pose

            # simulate
            observations += simulate(sim, dt=1.5, get_frames=make_video)

        # example get/set DoF forces and velocities
        tau = robot.joint_forces
        robot.joint_forces = tau

        vel = robot.joint_velocities
        robot.joint_velocities = vel

        # reset the object state (sets dof positions/velocities/forces to 0, recomputes forward kinematics, udpate collision state)
        robot.clear_joint_states()
        # note: reset does not change the robot base state, do this manually
        place_robot_from_agent(sim, robot)

        # set sleeping ON
        robot.awake = False
        assert robot.awake is False

        observations += simulate(sim, dt=1.0, get_frames=make_video)

        # set sleeping OFF
        robot.awake = True
        assert robot.awake is True

        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # get/set motiontype (KINEMATIC vs. DYNAMIC)
        robot.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        assert robot.motion_type == habitat_sim.physics.MotionType.KINEMATIC

        # reset the object state (sets dof positions/velocities/forces to 0, recomputes forward kinematics, udpate collision state)
        robot.clear_joint_states()
        # note: reset does not change the robot base state, do this manually
        place_robot_from_agent(sim, robot)

        # get rigid state of robot links and show proxy object at each link COM
        obj_attr_mgr = sim.get_object_template_manager()
        cube = rigid_obj_mgr.add_object_by_template_handle(
            obj_attr_mgr.get_template_handles("cube")[0]
        )
        cube.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        cube.collidable = False
        for link_id in range(robot.num_links):
            link_scene_node = robot.get_link_scene_node(link_id)
            cube.translation = link_scene_node.translation
            cube.rotation = link_scene_node.rotation
            # alternatively:
            cube.transformation = link_scene_node.transformation_matrix()
            # get the link friction
            print(
                f"Link {link_id} friction coefficient = {robot.get_link_friction(link_id)}"
            )
            # Note: set this with 'sim.set_articulated_link_friction(robot_id, link_id, friction)'
            observations += simulate(sim, dt=0.5, get_frames=make_video)
        rigid_obj_mgr.remove_object_by_id(cube.object_id)

        robot.motion_type = habitat_sim.physics.MotionType.DYNAMIC
        assert robot.motion_type == habitat_sim.physics.MotionType.DYNAMIC

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "URDF_basics",
                open_vid=show_video,
            )

        # clear all robots
        art_obj_mgr.remove_all_objects()
        # [/basics]

        # [joint motors]
        observations = []

        # load a URDF file with a fixed base
        robot_file = urdf_files["iiwa"]
        robot = art_obj_mgr.add_articulated_object_from_urdf(robot_file, True)

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot, -math.pi)

        # query any damping motors created by default
        existing_joint_motors = robot.get_existing_joint_motor_ids()
        print("default damping motors (motor_id -> dof): " + str(existing_joint_motors))

        # get the max_impulse of the damping motors
        for motor_id in existing_joint_motors:
            motor_settings = robot.get_joint_motor_settings(motor_id)
            print(f"   motor({motor_id}): max_impulse = {motor_settings.max_impulse}")

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
        new_motor_id = robot.create_joint_motor(
            1, joint_motor_settings  # dof  # settings
        )
        existing_joint_motors = robot.get_existing_joint_motor_ids()
        print("new_motor_id: " + str(new_motor_id))
        print(
            "existing motors after create (motor_id -> dof): "
            + str(existing_joint_motors)
        )

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # reverse the motor velocity
        joint_motor_settings.velocity_target = -1.0
        robot.update_joint_motor(new_motor_id, joint_motor_settings)

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # remove the new joint motor
        robot.remove_joint_motor(new_motor_id)

        # create joint motors for all valid dofs to control a pose (1.1 for all dofs)
        joint_motor_settings = habitat_sim.physics.JointMotorSettings(
            0.5, 1.0, 0, 0, 1.0
        )
        # (optional) motor settings, if not provided will be default (no gains)
        dofs_to_motor_ids = robot.create_all_motors(joint_motor_settings)
        print("New motors (motor_ids -> dofs): " + str(dofs_to_motor_ids))

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # remove all motors
        existing_joint_motors = robot.get_existing_joint_motor_ids()
        print(
            "All motors (motor_id -> dof) before removal: " + str(existing_joint_motors)
        )
        for motor_id in existing_joint_motors:
            robot.remove_joint_motor(motor_id)
        print(
            f"All motors (motor_id -> dof) after removal: {robot.get_existing_joint_motor_ids()}"
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

        # separately remove all articulated and rigid objects with their respective managers
        art_obj_mgr.remove_all_objects()
        rigid_obj_mgr.remove_all_objects()


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
    test_ao_recompute_navmesh(make_video, show_video)
    test_prim_chain(make_video, show_video)
    test_spherical_joints(make_video, show_video)
    test_urdf_memory()
    demo_contact_profile()
