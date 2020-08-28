Habitat Simulator
#################

A flexible, high-performance 3D simulator with configurable agents, multiple
sensors, and generic 3D dataset handling (with built-in support for
MatterPort3D, Gibson, Replica, and other datasets).

When rendering a scene from the Matterport3D dataset, *Habitat-Sim* achieves
several thousand frames per second (FPS) running single-threaded, and reaches
over 10,000 FPS multi-process on a single GPU!

The Habitat platform includes *Habitat-Sim* and `Habitat-Lab <http://aihabitat.org/docs/habitat-lab/>`_. To learn how these fit together, see our `ECCV 2020 tutorial series <https://aihabitat.org/tutorial/2020/>`_.

Tutorials
=========

.. class:: m-table m-fullwidth

=================================================== ======================================================================================== ======================
Basics for Navigation                               `Video <https://youtu.be/kunFMRJAu2U?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                     `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/master/examples/tutorials/colabs/ECCV_2020_Navigation.ipynb>`__

Interaction                                         `Video <https://youtu.be/6eh0PBesIgw?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                     `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/master/examples/tutorials/colabs/ECCV_2020_Interactivity.ipynb>`__

Advanced Topics                                     `Video <https://youtu.be/w_kDq6UOKos?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                     `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/master/examples/tutorials/colabs/ECCV_2020_Advanced_Features.ipynb>`__

Profiling and Optimization                          `Video <https://youtu.be/I4MjX598ZYs?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                     `Interactive Colab <https://colab.research.google.com/gist/eundersander/b62bb497519b44cf4ceb10e2079525dc/faster-rl-training-profiling-and-optimization.ipynb>`__

New Actions                                         :ref:`Web page <std:doc:new-actions>`

Notebooks                                           :ref:`Web page <std:doc:notebooks>`

Creating a stereo agent                             :ref:`Web page <std:doc:stereo-agent>`

Working with light setups                           :ref:`Web page <std:doc:lighting-setups>`

Extracting Images                                   :ref:`Web page <std:doc:image-extractor>`

Interactive Rigid Objects                           :ref:`Web page <std:doc:rigid-object-tutorial>`
=================================================== ======================================================================================== ======================

Python Classes
==============

See the `Classes <./classes.html>`_ tab.

Python Unit Tests
=================

Browse selected unit tests that demonstrate essential *Habitat-Sim* interfaces.

- `test_agent.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_agent.py>`_
- `test_attributes_manager.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_attributes_manager.py>`_
- `test_configs.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_configs.py>`_
- `test_controls.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_controls.py>`_
- `test_gfx.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_gfx.py>`_
- `test_greedy_follower.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_greedy_follower.py>`_
- `test_light_setup.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_light_setup.py>`_
- `test_material.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_material.py>`_
- `test_navmesh.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_navmesh.py>`_
- `test_physics.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_physics.py>`_
- `test_pyrobot_noisy_controls.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_pyrobot_noisy_controls.py>`_
- `test_semantic_scene.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_semantic_scene.py>`_
- `test_sensors.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_sensors.py>`_
- `test_simulator.py <https://github.com/facebookresearch/habitat-sim/blob/master/tests/test_simulator.py>`_

.. We exclude unit tests that aren't particularly self-explanatory or interesting.
.. test_snap_points
.. test_utils
.. test_compare_profiles
.. test_data_extraction
.. test_examples
.. test_profiling_utils
.. test_random_seed

C++ API Documentation
=====================

Habitat-sim is designed to be used primarily through its Python API. As such, the
end-user tutorials and docs linked above focus on Python.

If you're looking for API reference of Habitat-sim's C++ internals, please see the
`C++ API <cpp.html>`_ tab.
