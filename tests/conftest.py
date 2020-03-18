import os.path as osp

import pytest

import habitat_sim
from examples.settings import make_cfg

_test_scene = osp.abspath(
    osp.join(
        osp.dirname(__file__),
        "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    )
)


@pytest.fixture(scope="session")
def make_cfg_settings():
    return dict(
        height=480,
        width=640,
        sensor_height=1.5,
        color_sensor=True,
        semantic_sensor=True,
        depth_sensor=True,
        silent=True,
        scene=_test_scene,
    )


# Any test globally can take `sim` as an arguement and will get
# the single instance of the Simulator
@pytest.fixture(scope="session")
def sim(make_cfg_settings):
    return habitat_sim.Simulator(make_cfg(make_cfg_settings))
