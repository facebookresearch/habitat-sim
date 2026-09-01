Loading URDF Articulated Objects
################################

:ref-prefix:
    habitat_sim
    habitat_sim.physics
    habitat_sim.scene
    habitat_sim.sim
    habitat_sim.simulator
    habitat_sim.utils.settings

:summary: This page shows the current URDF loading entry points in Habitat-sim: direct Python import, the interactive viewer, and SceneDataset-based registration.

.. contents::
    :class: m-block m-default

Habitat-Sim supports URDF import through the :ref:`habitat_sim.physics.ArticulatedObjectManager`.
The current repository examples for this workflow live in :gh:`tests/test_physics.py <facebookresearch/habitat-sim/blob/main/tests/test_physics.py>` and :gh:`examples/viewer.py <facebookresearch/habitat-sim/blob/main/examples/viewer.py>`.

`Load A URDF Directly In Python`_
=================================

Direct URDF import requires Bullet physics to be enabled.
The following example mirrors the tested path used by :gh:`test_articulated_object_add_remove <facebookresearch/habitat-sim/blob/main/tests/test_physics.py#L964-L1025>`.

.. code:: py

    import habitat_sim
    import habitat_sim.utils.settings

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        robot = art_obj_mgr.add_articulated_object_from_urdf(
            filepath="data/test_assets/urdf/kuka_iiwa/model_free_base.urdf"
        )
        robot.translation = [0.0, 0.0, -1.5]

Useful import options demonstrated in the current tests include:

- :gh:`fixed_base=True <facebookresearch/habitat-sim/blob/main/tests/test_physics.py#L1398-L1428>` for robots that should stay anchored to the world.
- :gh:`global_scale=... <facebookresearch/habitat-sim/blob/main/tests/test_physics.py#L998-L1004>` for quick scale adjustments at load time.
- :gh:`maintain_link_order=True <facebookresearch/habitat-sim/blob/main/tests/test_physics.py#L1032-L1083>` when downstream code depends on the original URDF link ordering.

`Load A URDF In The Interactive Viewer`_
========================================

The interactive viewer exposes the same import path without writing additional Python.

.. code:: shell-session

    $ python examples/viewer.py --scene /path/to/scene.glb

Once the viewer is running:

- Press ``t`` to enter a URDF filepath and load it.
- Press ``Shift+t`` to reload the previously entered URDF path.
- Press ``Alt+t`` to load the URDF with ``fixed_base=True``.

The viewer implementation is available in :gh:`examples/viewer.py <facebookresearch/habitat-sim/blob/main/examples/viewer.py#L557-L600>`.
For a repo-local example asset, try ``data/test_assets/urdf/kuka_iiwa/model_free_base.urdf``.

`Register URDF Assets In A SceneDataset`_
=========================================

If you want articulated objects to load as part of a dataset or scene configuration instead of being added manually at runtime, register them through :ref:`ArticulatedObjectAttributes` in the SceneDataset metadata.
The JSON schema and field descriptions for this path are documented on the :ref:`Attributes Templates JSON Tags <std:doc:attributesJSON>` page.

`Additional References`_
========================

- :gh:`test_physics.py <facebookresearch/habitat-sim/blob/main/tests/test_physics.py>` collects the current validated URDF import behaviors, including loading, fixed-base behavior, joint ordering, and motor setup.
- :gh:`test_physics_benchmarking.py <facebookresearch/habitat-sim/blob/main/tests/test_physics_benchmarking.py>` benchmarks repeated URDF load and removal cycles.
- :gh:`data/test_assets/urdf/ <facebookresearch/habitat-sim/tree/main/data/test_assets/urdf>` contains small URDF assets that are convenient for local experiments.
