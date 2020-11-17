from os import path as osp
import examples.settings
import pytest

_test_scene = osp.abspath(
    osp.join(
        osp.dirname(__file__),
        "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    )
)


@pytest.fixture(scope="function")
def make_cfg_settings():
  cfg = examples.settings.default_sim_settings.copy()
  cfg["height"] = 480
  cfg["width"] = 640
  cfg["sensor_height"] = 1.5
  cfg["color_sensor"] = True
  cfg["semantic_sensor"] = True
  cfg["depth_sensor"] = True
  cfg["silent"] = True
  cfg["scene"] = _test_scene
  cfg["frustum_culling"] = True
  return cfg
