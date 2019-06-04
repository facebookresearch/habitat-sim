import multiprocessing
import os.path as osp

import examples.new_actions
import pytest


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
    reason="New actions script requires the habitat-test-scenes",
)
def test_new_actions_example():
    # This test needs to be done in its own process as there is a potentially for
    # an OpenGL context clash otherwise
    mp_ctx = multiprocessing.get_context("spawn")
    proc = mp_ctx.Process(target=examples.new_actions.main)
    proc.start()
    proc.join()

    assert proc.exitcode == 0
