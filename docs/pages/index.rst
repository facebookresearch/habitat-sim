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

=================================================== ========================================================================================================================================================== ======================
Basics for Navigation                               `Video <https://youtu.be/kunFMRJAu2U?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                                                               `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Navigation.ipynb>`__

Interaction                                         `Video <https://youtu.be/6eh0PBesIgw?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                                                               `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Interactivity.ipynb>`__

Advanced Topics                                     `Video <https://youtu.be/w_kDq6UOKos?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                                                               `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Advanced_Features.ipynb>`__

Profiling and Optimization                          `Video <https://youtu.be/I4MjX598ZYs?list=PLGywud_-HlCORC0c4uj97oppQrGiB6JNy>`__                                                                               `Interactive Colab <https://colab.research.google.com/gist/eundersander/b62bb497519b44cf4ceb10e2079525dc/faster-rl-training-profiling-and-optimization.ipynb>`__

New Actions                                         :ref:`Page <std:doc:new-actions>`

Attributes Templates JSON Tags                      :ref:`Page <std:doc:attributesJSON>`

Creating a stereo agent                             :ref:`Page <std:doc:stereo-agent>`

Working with light setups                           :ref:`Page <std:doc:lighting-setups>`

Extracting Images                                   :ref:`Page <std:doc:image-extractor>`

View Assets in Habitat-Sim                          :ref:`Page <std:doc:asset-viewer-tutorial>`                                                                                                                    `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/asset_viewer.ipynb>`__

Interactive Rigid Objects 2.0                       :ref:`Page <std:doc:managed-rigid-object-tutorial>`                                                                                                            `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/managed_rigid_object_tutorial.ipynb>`__

Gfx Replay                                                                                                                                                                                                         `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/replay_tutorial.ipynb>`__

Editing Scene Assets in Blender                     `Page <https://aihabitat.org/tutorial/editing_in_blender/>`_

Coordinate Frame Tutorial                           :ref:`Page <std:doc:coordinate-frame-tutorial>`                                                                                                                `Interactive Colab <https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/coordinate_frame_tutorial.ipynb>`__
=================================================== ========================================================================================================================================================== ======================

Python Classes
==============

See the `Classes <./classes.html>`_ tab.

Logging Configuration
=====================

See :ref:`Logging Configuration <std:doc:logging>` for how to configure *Habitat-Sim* logging.

Python Unit Tests
=================

Browse selected unit tests that demonstrate essential *Habitat-Sim* interfaces.

- :gh:`test_agent.py <facebookresearch/habitat-sim/blob/main/tests/test_agent.py>`
- :gh:`test_attributes_managers.py <facebookresearch/habitat-sim/blob/main/tests/test_attributes_managers.py>`
- :gh:`test_configs.py <facebookresearch/habitat-sim/blob/main/tests/test_configs.py>`
- :gh:`test_controls.py <facebookresearch/habitat-sim/blob/main/tests/test_controls.py>`
- :gh:`test_gfx.py <facebookresearch/habitat-sim/blob/main/tests/test_gfx.py>`
- :gh:`test_greedy_follower.py <facebookresearch/habitat-sim/blob/main/tests/test_greedy_follower.py>`
- :gh:`test_light_setup.py <facebookresearch/habitat-sim/blob/main/tests/test_light_setup.py>`
- :gh:`test_navmesh.py <facebookresearch/habitat-sim/blob/main/tests/test_navmesh.py>`
- :gh:`test_physics.py <facebookresearch/habitat-sim/blob/main/tests/test_physics.py>`
- :gh:`test_pyrobot_noisy_controls.py <facebookresearch/habitat-sim/blob/main/tests/test_pyrobot_noisy_controls.py>`
- :gh:`test_semantic_scene.py <facebookresearch/habitat-sim/blob/main/tests/test_semantic_scene.py>`
- :gh:`test_sensors.py <facebookresearch/habitat-sim/blob/main/tests/test_sensors.py>`
- :gh:`test_simulator.py <facebookresearch/habitat-sim/blob/main/tests/test_simulator.py>`

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
