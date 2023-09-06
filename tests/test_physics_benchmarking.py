#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

import habitat_sim
import habitat_sim.utils.settings


# benchmark adding/removing articulated objects from URDF files
@pytest.mark.sim_benchmarks
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
@pytest.mark.benchmark(group="URDF load->remove iterations|force_reload")
@pytest.mark.parametrize("iterations", [1, 10, 100, 200])
@pytest.mark.parametrize("force_reload", [False, True])
def test_benchmark_urdf_add_remove(benchmark, iterations, force_reload):
    # test loading and removing a URDF ArticultedObject multiple times consecutively
    def instance_remove_urdf(iterations):
        # first configure the simulator
        cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
        cfg_settings["scene"] = "NONE"
        cfg_settings["enable_physics"] = True
        hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
        with habitat_sim.Simulator(hab_cfg) as sim:
            art_obj_mgr = sim.get_articulated_object_manager()

            robot_file = "data/test_assets/urdf/kuka_iiwa/model_free_base.urdf"

            for _iteration in range(iterations):
                robot = art_obj_mgr.add_articulated_object_from_urdf(
                    robot_file, force_reload=force_reload
                )
                art_obj_mgr.remove_object_by_id(robot.object_id)

    benchmark(instance_remove_urdf, iterations)
