Working with Lights
###################

:summary: This tutorial demonstrates the creation and manipulation of LightSetups in Habitat-sim.

.. contents::
    :class: m-block m-default

A LightSetup consists of a set of LightInfo structures defining the common configuration
of a set of point lights used to render objects in a scene. Once defined and registered,
a LightSetup can be assigned to any subset of objects in the scene, including the scene asset itself.

Each LightInfo structure in a LightSetup defines the `color, position, and LightPositionModel` of a single point light source.
The LightPositionModel defines the coordinate frame of the light and can be set to any of:

.. code:: python

    habitat_sim.gfx.LightPositionModel.CAMERA
    habitat_sim.gfx.LightPositionModel.GLOBAL
    habitat_sim.gfx.LightPositionModel.OBJECT

Each LightSetup is registered in the simulator via a unique key. Two default lighting setups are pre-defined:

.. code:: python

    habitat_sim.gfx.DEFAULT_LIGHTING_KEY
    habitat_sim.gfx.NO_LIGHT_KEY

Additional custom setups can be created and registered via the `simulator.set_light_setup` function:

.. code:: python

    sim.set_light_setup(new_light_setup, "my_custom_lighting_key")

Any existing LightSetup can be queried with `simulator.get_light_setup`:

.. code:: python

    custom_light_setup = sim.set_light_setup(n"my_custom_lighting_key")


`Working with Lights Example:`_
===============================

The example code below demonstrates the usage of default and custom LightSetups and is runnable via:

.. code:: shell-session

    $ python examples/tutorials/lighting_tutorial.py

First, we import modules we will need, define some convenience functions, and initialize the Simulator, Agent, and objects.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [setup]
    :end-before: # [/setup]

`Scene Lighting`_
=================

By default, the scene will be shaded with no lights using the `habitat_sim.gfx.NO_LIGHTING_KEY` setup.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 1]
    :end-before: # [/example 1]

.. image:: scene_default_lighting.png

To use a custom light setup for the scene, simply use the `scene_light_setup` field of
`habitat_sim.SimulatorConfiguration` when creating/reconfiguring your Simulator. Note that
while you can modify the scene's LightSetup dynamically during runtime, you will need to
reconfigure the simulator to switch the scene's LightSetup key.

.. code:: python

    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_light_setup = habitat_sim.gfx.DEFAULT_LIGHTING_KEY


`Default Object Lighting`_
==========================

By default, objects added to the scene will be rendered with Phong shading. This default LightSetup is initalized with one light at each corner of the scene mesh's bounding box.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 2]
    :end-before: # [/example 2]

.. image:: default_object_lighting.png

We can update the default lighting.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 3]
    :end-before: # [/example 3]

.. image:: change_default_lighting.png

Newly added objects will use the current default lighting.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 4]
    :end-before: # [/example 4]

.. image:: new_objects_default_lighting.png


`Multiple Light Setups`_
========================

To use multiple custom lighting setups at the same time, simply give them a name on creation.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 5]
    :end-before: # [/example 5]

To use this a light setup, pass in the name as a parameter to `Simulator.add_object`.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 6]
    :end-before: # [/example 6]

.. image:: custom_lighting.png

You can get a copy of an existing configuration with `Simulator.get_light_setup`.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 7]
    :end-before: # [/example 7]

Updates to existing light setups will update all objects using that setup

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 8]
    :end-before: # [/example 8]

.. image:: change_custom_lighting.png

You can change the light setup an individual object uses.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 9]
    :end-before: # [/example 9]

.. image:: change_object_light_setup.png
