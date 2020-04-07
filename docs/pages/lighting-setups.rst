Working with Lights
###################

.. contents::
    :class: m-block m-default

This tutorial shows how to create and manipulate light setups.

`Imports`_
==========

First, we import modules we will need.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [import]
    :end-before: # [/import]

`Helper Functions and Constants`_
=================================

Next, we define some simple helper functions and constants.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [helpers]
    :end-before: # [/helpers]

`Simulator and Agent Configuration`_
====================================

Next, we create a simulator and place our agent in the scene.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [initialization]
    :end-before: # [/initialization]

`Scene Lighting`_
=================

By default, the scene will be shaded with no lights.

.. include:: ../../examples/tutorials/lighting_tutorial.py
    :code: py
    :start-after: # [example 1]
    :end-before: # [/example 1]

.. image:: scene_default_lighting.png

To use a non-default light setup for the scene, simply use the `sceneLightSetup` field of
`habitat_sim.SimulatorConfiguration` when creating/reconfiguring your Simulator.

`Default Object Lighting`_
==========================

By default, added objects will be phong shaded with lights at the corners of the scene.

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
