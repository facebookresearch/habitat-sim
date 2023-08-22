#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import random
import typing
from os import path as osp

import magnum as mn
import numpy as np
import pytest
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.bindings
import habitat_sim.utils.settings


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics used for validation.",
)
@pytest.mark.parametrize("zfar", [500, 1000, 1500])
def test_unproject(zfar):
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()

    # configure some settings in case defaults change
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True
    cfg_settings["zfar"] = zfar
    cfg_settings["width"] = 501
    cfg_settings["height"] = 501
    cfg_settings["sensor_height"] = 0
    cfg_settings["color_sensor"] = True

    # loading the scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # position agent
        sim.agents[0].scene_node.rotation = mn.Quaternion()
        sim.agents[0].scene_node.translation = mn.Vector3(0.5, 0, 0)

        # setup camera
        far_plane = sim._sensors["depth_sensor"]._sensor_object.far_plane_dist
        assert zfar == far_plane
        render_camera = sim._sensors["color_sensor"]._sensor_object.render_camera
        depth_camera = sim._sensors["depth_sensor"]._sensor_object.render_camera

        # test unproject with known values
        center_ray = render_camera.unproject(
            mn.Vector2i(250, 250), normalized=False
        )  # middle of the viewport
        center_ray_normalized = render_camera.unproject(mn.Vector2i(250, 250))
        assert np.allclose(
            center_ray_normalized.direction,
            center_ray.direction.normalized(),
            atol=0.07,
        )
        assert np.allclose(center_ray.origin, np.array([0.5, 0, 0]), atol=0.07)
        assert np.allclose(
            center_ray_normalized.direction, np.array([0, 0, -1.0]), atol=0.02
        )

        # NOTE: viewport y==0 is at the top
        test_ray_2 = render_camera.unproject(
            mn.Vector2i(500, 500), normalized=False
        )  # bottom right of the viewport
        test_ray_2_normalized = render_camera.unproject(mn.Vector2i(500, 500))
        assert np.allclose(
            test_ray_2_normalized.direction,
            test_ray_2.direction.normalized(),
            atol=0.07,
        )
        assert np.allclose(
            test_ray_2_normalized.direction,
            np.array([0.569653, -0.581161, -0.581161]),
            atol=0.07,
        )

        # add a primitive sphere object to the world
        obj_template_mgr = sim.get_object_template_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()
        sphere_prim_handle = obj_template_mgr.get_template_handles("uvSphereSolid")[0]
        sphere_template = obj_template_mgr.get_template_by_handle(sphere_prim_handle)
        sphere_template.scale = [0.03, 0.03, 0.03]
        obj_template_mgr.register_template(sphere_template, "scaled_sphere")
        sphere_prim_handle = obj_template_mgr.get_template_handles("scaled_sphere")[0]
        sphere_obj = rigid_obj_mgr.add_object_by_template_handle(sphere_prim_handle)

        # validate that random unprojected points scaled by depth camera distance are actually on the render mesh
        # do this by creating a small collision object at the unprojected point and test it against scene geometry
        num_samples = 10
        # move the camera, test a random pixel
        cur_sample = 0
        while cur_sample < num_samples:
            # move agent
            sim.agents[0].scene_node.translation = np.random.random(3)
            # rotate agent
            sim.agents[0].scene_node.rotation = mn.Quaternion.rotation(
                mn.Rad(random.random() * mn.math.tau), mn.Vector3(0, 1, 0)
            )
            # tilt the camera
            render_camera.node.rotation = mn.Quaternion.rotation(
                mn.Rad(random.random()), mn.Vector3(1, 0, 0)
            )
            depth_camera.node.rotation = render_camera.node.rotation

            # do the unprojection from depth image
            view_point = mn.Vector2i(
                random.randint(0, render_camera.viewport[0] - 1),
                random.randint(0, render_camera.viewport[1] - 1),
            )
            # NOTE: use un-normalized rays scaled to unit z distance for this application
            ray = render_camera.unproject(view_point, normalized=False)
            depth_obs = typing.cast(
                np.ndarray, sim.get_sensor_observations()["depth_sensor"]
            )
            # NOTE: (height, width) for buffer access
            depth = depth_obs[view_point[1]][view_point[0]]

            if depth == 0.0:
                # skip depths of 0 which represent empty/background pixels
                continue

            # update the collision test object
            depth_point = ray.origin + ray.direction * depth
            sphere_obj.translation = depth_point

            # optionally render the frames for debugging
            # import habitat_sim.utils.viz_utils as vut
            # c_image = vut.observation_to_image(sim.get_sensor_observations()["color_sensor"], "color")
            # c_image.show()

            assert (
                sphere_obj.contact_test()
            ), "Must be intersecting with scene collision mesh."
            cur_sample += 1


@pytest.mark.parametrize(
    "sensor_type",
    [
        habitat_sim.SensorType.COLOR,
        habitat_sim.SensorType.DEPTH,
        habitat_sim.SensorType.SEMANTIC,
    ],
)
def test_empty_scene(sensor_type):
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "NONE"

    agent_cfg = habitat_sim.AgentConfiguration()
    agent_cfg.sensor_specifications = [habitat_sim.CameraSensorSpec()]
    agent_cfg.sensor_specifications[-1].sensor_type = sensor_type

    with habitat_sim.Simulator(
        habitat_sim.Configuration(backend_cfg, [agent_cfg])
    ) as sim:
        _ = sim.get_sensor_observations()
