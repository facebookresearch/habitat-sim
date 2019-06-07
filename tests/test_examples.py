import multiprocessing
import os.path as osp

import pytest

import examples.new_actions
import examples.stereo_agent


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.parametrize(
    "example_module,args",
    [(examples.stereo_agent, (False,)), (examples.new_actions, ())],
)
def test_example_modules(example_module, args):
    # This test needs to be done in its own process as there is a potentially for
    # an OpenGL context clash otherwise
    mp_ctx = multiprocessing.get_context("spawn")
    proc = mp_ctx.Process(target=example_module.main, args=args)
    proc.start()
    proc.join()

    assert proc.exitcode == 0
