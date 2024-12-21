# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict

import magnum as mn
import numpy as np
from PIL import Image

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg


def draw_axes(
    sim: habitat_sim.Simulator, transformation: mn.Matrix4 = None, axis_len: float = 1.0
):
    lr = sim.get_debug_line_render()
    if transformation is not None:
        lr.push_transform(transformation)
    # draw axes with x+ = red, y+ = green, z+ = blue
    lr.draw_transformed_line(mn.Vector3(), mn.Vector3(axis_len, 0, 0), mn.Color4.red())
    lr.draw_transformed_line(
        mn.Vector3(), mn.Vector3(0, axis_len, 0), mn.Color4.green()
    )
    lr.draw_transformed_line(mn.Vector3(), mn.Vector3(0, 0, axis_len), mn.Color4.blue())
    lr.draw_cone(
        translation=mn.Vector3(axis_len, 0, 0),
        apex=mn.Vector3(axis_len + axis_len * 0.2, 0, 0),
        radius=0.2,
        color=mn.Color4.red(),
        normal=mn.Vector3(1, 0, 0),
    )
    lr.draw_cone(
        translation=mn.Vector3(0, axis_len, 0),
        apex=mn.Vector3(0, axis_len + axis_len * 0.2, 0),
        radius=0.2,
        color=mn.Color4.green(),
        normal=mn.Vector3(0, 1, 0),
    )
    lr.draw_cone(
        translation=mn.Vector3(0, 0, axis_len),
        apex=mn.Vector3(0, 0, axis_len + axis_len * 0.2),
        radius=0.2,
        color=mn.Color4.blue(),
        normal=mn.Vector3(0, 0, 1),
    )
    if transformation is not None:
        lr.pop_transform()


def display_sensor_image(camera_sensor):
    camera_sensor.draw_observation()
    rgb_image = Image.fromarray(np.uint8(camera_sensor.get_observation()))
    rgb_image.show()


def main_coordinate_system_visual_examples():
    """
    Initializes a simulator and runs a set of incremental visual examples to demonstrate the default behavior of the system and cameras.
    """

    # setup the settings configuration
    sim_settings: Dict[str, Any] = default_sim_settings
    # make camera coincident with agent scenenode
    sim_settings["sensor_height"] = 0
    # sim_settings["scene"] = "data/test_assets/scenes/simple_room.stage_config.json"

    cfg = make_cfg(sim_settings)

    with habitat_sim.Simulator(cfg) as sim:
        # agent local transform is identity
        camera_agent = sim.get_agent(0)
        print(f"Agent transform: {camera_agent.scene_node.transformation}")
        # camera local transform is identity
        camera_sensor = sim._Simulator__sensors[0]["color_sensor"]
        print(f"Camera transform: {camera_sensor._sensor_object.node.transformation}")
        # camera absolute transform is identity
        print(
            f"Camera absolute transform: {camera_sensor._sensor_object.node.absolute_transformation()}"
        )

        # Example image 1: Graphics Coordinate System
        # An identity transformation render with:
        # 1. standard RGB coordinate axis along positive XYZ
        # 2. a yellow circle at (0,0,-1) with normal (0,0,1)
        # 3. a magenta line from center of the circle to +Y showing "up"
        #
        # NOTE:
        # You can see the yellow circle centered in the view but not the coordinate axis.
        # This demonstrates that the camera is facing -Z with +Y up.
        draw_axes(sim)
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)

        # Example image 2:
        # Moving the camera origin to (-2,0,0) and rotating to face the origin (aligining -Z with X) by rotating around Y 90 degrees
        #
        # NOTE:
        # You can see the relative positioning of the coordinate axes and the yellow circle.
        # The red cone is smaller because it is farther away, indicating that we are looking in the +X direction.
        camera_sensor._sensor_object.node.translation = mn.Vector3(-2, 0, 0)
        camera_sensor._sensor_object.node.rotation = mn.Quaternion.rotation(
            mn.Deg(-90), mn.Vector3(0, 1, 0)
        )
        draw_axes(sim)
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)

        # Example image 3: Robotics Coordinate System
        # Rotating the camera an additional 90 degrees around the X axis into "robotics coordinate system"
        #
        # NOTE:
        # We keep the translation of (-2,0,0) so we can see the coordinate axis
        # We can see along the red (+X) axis
        # Green (Y) points to the left
        # Blue (Z) points up
        # This is the robotics coordinate system!
        camera_sensor._sensor_object.node.translation = mn.Vector3(-2, 0, 0)
        graphics_to_robotics_quat = mn.Quaternion.rotation(
            mn.Deg(90), mn.Vector3(1, 0, 0)
        ) * mn.Quaternion.rotation(mn.Deg(-90), mn.Vector3(0, 1, 0))
        graphics_to_robotics_mat = mn.Matrix4.from_(
            graphics_to_robotics_quat.to_matrix(), mn.Vector3()
        )
        camera_sensor._sensor_object.node.rotation = graphics_to_robotics_quat
        draw_axes(sim)
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)

        print(f"Graphics to robotics transformation: {graphics_to_robotics_mat}")

        # Example image 4: robotics to graphics axis
        # We rotate the axis by the inverse of the robotics to graphics transormation to align the frames
        #
        # NOTE:
        # We can see the red X-axis aligned with the yellow circle on the -Z axis
        camera_sensor._sensor_object.node.transformation = mn.Matrix4()
        draw_axes(sim, transformation=graphics_to_robotics_mat.inverted())
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)

        # Example image 5: robotics to graphics axis (zoomed out)
        # We pull the camera back along the Z axis to show the remainder of the aligned axis
        #
        # NOTE:
        # The blue (Z) of the robotics axis is now aligned with +Y (up)
        # The green (Y) points to left as expected.
        camera_sensor._sensor_object.node.translation = mn.Vector3(0, 0, 1)
        draw_axes(sim, transformation=graphics_to_robotics_mat.inverted())
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)

        # load an axis urdf for testing
        aom = sim.get_articulated_object_manager()
        axis_urdf = aom.add_articulated_object_from_urdf(
            "data/test_assets/urdf/axis.urdf",
            fixed_base=True,
        )
        print(
            f"Axis URDF transformation {axis_urdf.transformation}, {axis_urdf.root_scene_node.absolute_transformation()}"
        )

        # Example image 7: robotic axis default in Habitat
        # We load a URDF "axis" with 3 fixed links along each unit axis with identity transformation
        #
        # NOTE:
        # The internal coordinate system of the axis URDF is aligned with the default graphics coordinate system
        #
        camera_sensor._sensor_object.node.transformation = mn.Matrix4.look_at(
            eye=mn.Vector3(2, 2, 2),  # from along the diagonal
            target=mn.Vector3(),  # to the origin
            up=mn.Vector3(0, 1, 0),  # up is graphics +Y
        )
        draw_axes(sim)
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)
        # move the URDF out of the way for future tests
        axis_urdf.translation = mn.Vector3(100, 0, 0)

        # load a rigid object axis from Blender conventions
        rotm = sim.get_object_template_manager()
        axis_template = rotm.create_new_template("axis")
        axis_template.render_asset_handle = "data/test_assets/objects/axis.glb"
        # don't let the CoM be computed from centroid of the shape
        axis_template.compute_COM_from_shape = False
        rotm.register_template(axis_template, specified_handle="axis")
        rom = sim.get_rigid_object_manager()
        axis_ro = rom.add_object_by_template_handle("axis")
        print(
            f"Axis Rigid transformation {axis_ro.transformation}, {axis_ro.root_scene_node.absolute_transformation()}"
        )

        # Example image 8: blender axis rigid in habitat
        # We load an "axis" .glb created in blender with 3 fixed meshes along each unit axis with identity transformation
        #
        # NOTE:
        # The internal coordinate system of the axis rigid is aligned with the default graphics coordinate system
        # When exporting the object from Blender, we uncheck the "+Y" exporter option to prevent rotation on export into "GLB coordinate system"
        camera_sensor._sensor_object.node.transformation = mn.Matrix4.look_at(
            eye=mn.Vector3(2, 2, 2),  # from along the diagonal
            target=mn.Vector3(),  # to the origin
            up=mn.Vector3(0, 1, 0),  # up is graphics +Y
        )
        draw_axes(sim)
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)

        # add a spot robot
        aom.add_articulated_object_from_urdf(
            "data/robots/hab_spot_arm/urdf/hab_spot_arm.urdf",
            fixed_base=True,
        )
        # Example image 9: Spot robot URDF in habitat
        # We load an spot Robot urdf
        #
        # NOTE:
        # we set <link name="base"> <inertial> <origin rpy="0 0 0" xyz="0.0 0.0 0.0" /> in hab_spot_arm.urdf to remove the added rpy rotation for Habitat
        # Note that the robot orientation aligns with the expected robotic axes: (red/x forward), (green/y left), (blue/z) up.
        camera_sensor._sensor_object.node.transformation = mn.Matrix4.look_at(
            eye=mn.Vector3(2, 2, 2),  # from along the diagonal
            target=mn.Vector3(),  # to the origin
            up=mn.Vector3(0, 1, 0),  # up is graphics +Y
        )
        draw_axes(sim)  # , transformation=graphics_to_robotics_mat.inverted())
        sim.get_debug_line_render().draw_circle(
            mn.Vector3(0, 0, -1),
            radius=0.5,
            color=mn.Color4.yellow(),
            normal=mn.Vector3(0, 0, 1),
        )
        sim.get_debug_line_render().draw_transformed_line(
            mn.Vector3(0, 0, -1), mn.Vector3(0, 1, -1), mn.Color4.magenta()
        )
        display_sensor_image(camera_sensor)
    # exit initial sim instancing


def main_fremont_robotics():
    """
    Load a Fremont scene configured for robotics coordinate system testing.
    """
    # setup the settings configuration
    sim_settings: Dict[str, Any] = default_sim_settings
    # make camera coincident with agent scenenode
    sim_settings["sensor_height"] = 0
    sim_settings[
        "scene_dataset_config_file"
    ] = "data/Fremont-Knuckles/fremont_knuckles.scene_dataset_config.json"
    # NOTE: This loads the scan.stage_config.json
    # NOTE: This file is modified to re-orient into robotics coordinate system.
    sim_settings["scene"] = "scan"

    cfg = make_cfg(sim_settings)

    with habitat_sim.Simulator(cfg) as sim:
        # set gravity to -Z direction
        sim.set_gravity(mn.Vector3(0, 0, -9.8))
        sim.navmesh_visualization = True

        # agent local transform is identity
        camera_agent = sim.get_agent(0)
        # set the default agent transform to identity (it gets randomized when added upon initialization if there is a navmesh)
        camera_agent.scene_node.transformation = mn.Matrix4()
        print(f"Agent transform: {camera_agent.scene_node.transformation}")
        # camera local transform is identity
        camera_sensor = sim._Simulator__sensors[0]["color_sensor"]
        print(f"Camera transform: {camera_sensor._sensor_object.node.transformation}")
        # camera absolute transform is identity
        print(
            f"Camera absolute transform: {camera_sensor._sensor_object.node.absolute_transformation()}"
        )

        # add a spot robot
        aom = sim.get_articulated_object_manager()
        spot = aom.add_articulated_object_from_urdf(
            "data/robots/hab_spot_arm/urdf/hab_spot_arm_no_rot.urdf",
            fixed_base=True,
        )
        spot.translation = sim.pathfinder.snap_point(spot.translation) + mn.Vector3(
            0, 0, 0.8
        )

        render_dist = 4
        for diagonal in [
            mn.Vector3(2, 2, 2),
            mn.Vector3(-2, 2, 2),
            mn.Vector3(-2, -2, 2),
            mn.Vector3(2, -2, 2),
        ]:
            diagonal = diagonal.normalized() * render_dist
            camera_sensor._sensor_object.node.transformation = mn.Matrix4.look_at(
                eye=diagonal,  # from along the diagonal
                target=mn.Vector3(),  # to the origin
                up=mn.Vector3(0, 0, 1),  # up is robotics in +Z
            )

            draw_axes(sim)
            display_sensor_image(camera_sensor)

        stage_attributes = sim.get_stage_initialization_template()
        print(stage_attributes)
        # breakpoint()
        pass


if __name__ == "__main__":
    # test the conventions and produce images
    # main_coordinate_system_visual_examples()

    main_fremont_robotics()
