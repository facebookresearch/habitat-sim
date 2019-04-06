import habitat_sim.bindings as hsim
import habitat_sim
import habitat_sim.utils
import pytest
import os.path as osp
from examples.settings import make_cfg


_test_scene = osp.abspath(
    osp.join(osp.dirname(__file__), "17DRP5sb8fy", "17DRP5sb8fy.glb")
)


# Any test globally can take `sim` as an arguement and will get
# the single instance of the Simulator
@pytest.fixture(scope="session")
def sim():
    settings = dict(
        height=480,
        width=640,
        sensor_height=1.5,
        color_sensor=True,
        semantic_sensor=True,
        depth_sensor=True,
        silent=True,
        scene=_test_scene,
    )

    return habitat_sim.Simulator(make_cfg(settings))
