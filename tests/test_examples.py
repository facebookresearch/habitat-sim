import multiprocessing
import os.path as osp
import shlex
import subprocess

import pytest

import examples.tutorials.lighting_tutorial
import examples.tutorials.new_actions
import examples.tutorials.rigid_object_tutorial
import examples.tutorials.semantic_id_tutorial
import examples.tutorials.stereo_agent


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.parametrize(
    "args",
    [
        ('python', 'examples/tutorials/stereo_agent.py', '--no-display'),
        ('python', 'examples/tutorials/lighting_tutorial.py', '--no-show-images'),
        ('python', 'examples/tutorials/new_actions.py'),
        ('python', 'examples/tutorials/rigid_object_tutorial.py'),
        ('python', 'examples/tutorials/semantic_id_tutorial.py', '--no-show-images'),
        #(examples.tutorials.lighting_tutorial, (False,)),
        #(examples.tutorials.rigid_object_tutorial, (False,)),
        #(examples.tutorials.semantic_id_tutorial, (False,)),
    ],
)
def test_example_modules(args):
    # This test needs to be done in its own process as there is a potentially for
    # an OpenGL context clash otherwise
    mp_ctx = multiprocessing.get_context("spawn")
    #proc = mp_ctx.Process(target=example_module.main, args=args)
    #proc.start()
    #proc.join()
    print(args)
    exitcode = subprocess.call(args)

    assert exitcode == 0


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.parametrize(
    "args",
    [
        "--compute_shortest_path",
        "--compute_shortest_path --compute_action_shortest_path",
        "--enable_physics",
    ],
)
def test_example_script(args):
    subprocess.check_call(shlex.split(f"python examples/example.py {args}"))
