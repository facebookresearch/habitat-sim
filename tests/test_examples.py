import multiprocessing
import os.path as osp

import pytest

import examples.stereo_agent


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
)
def test_stereo_agent_example():
    # This test needs to be done in its own process as there is a potentially for
    # an OpenGL context clash otherwise
    mp_ctx = multiprocessing.get_context("spawn")
    proc = mp_ctx.Process(target=examples.stereo_agent.main, args=(False,))
    proc.start()
    proc.join()

    assert proc.exitcode == 0
