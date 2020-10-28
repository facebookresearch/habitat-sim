from os import path as osp

import pytest

_test_scene = osp.abspath(
    osp.join(
        osp.dirname(__file__),
        "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    )
)


@pytest.fixture(scope="function")
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
        frustum_culling=True,
    )
