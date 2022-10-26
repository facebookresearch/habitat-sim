# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
from habitat_sim.gfx import (
    DEFAULT_LIGHTING_KEY,
    NO_LIGHT_KEY,
    LightInfo,
    LightPositionModel,
)
from habitat_sim.utils.settings import make_cfg


def test_get_no_light_setup(make_cfg_settings):
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        assert len(sim.get_light_setup(NO_LIGHT_KEY)) == 0


def test_set_default_light_setup(make_cfg_settings):
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        # define a directional light (w == 0)
        light_setup = [LightInfo(vector=[1.0, 1.0, 1.0, 0.0])]

        sim.set_light_setup(light_setup)
        assert sim.get_light_setup() == light_setup

        # ensure modifications to local light setup variable are not reflected in sim
        light_setup[0].model = LightPositionModel.Camera
        assert sim.get_light_setup() != light_setup

        sim.set_light_setup(light_setup, DEFAULT_LIGHTING_KEY)
        assert sim.get_light_setup() == light_setup


def test_set_custom_light_setup(make_cfg_settings):
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        custom_setup_key = "custom_setup_key"

        light_setup = sim.get_light_setup(custom_setup_key)
        assert len(light_setup) == 0

        # define a point light (w == 1)
        light_setup.append(LightInfo(vector=[1.0, 1.0, 1.0, 1.0]))
        assert sim.get_light_setup() != light_setup

        sim.set_light_setup(light_setup, custom_setup_key)
        assert sim.get_light_setup(custom_setup_key) == light_setup
