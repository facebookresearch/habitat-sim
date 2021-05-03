Using JSON Files to configure Attributes
########################################

:ref-prefix:
    habitat_sim.sim
    habitat_sim.attributes
    habitat_sim.attributes_managers

:summary: This document describes the appropriate tags to be used when authoring JSON files to properly customize attributes templates.

.. contents::
    :class: m-block m-default

Attributes templates provide a mechanism by which the various constructions in Habitat can be customized and built with user-specified characteristics, either at program start or on the fly.

`Physics Manager Attributes`_
=============================
Physics Manager Attributes templates describe quantities pertinent to building the simulation world.  Any source configuration JSON files used to build these attributes should be formatted as follows:

 	<worldname>.physics_config.json

`An example of an appropriately configured Physics Manager Attributes file can be found below <../../../data/test_assets/testing.physics_config.json>`_:

.. include:: ../../data/test_assets/testing.physics_config.json
    :code: json


Below are the supported JSON tags for Physics Manager Attributes templates, and their meanings.

"physics_simulator"
	- string
	- What physics engine should be used for dynamics simulation.  Currently supports "bullet" for Bullet physics simulation, and "none", meaning kinematic motion is to be used.
"gravity"
	- 3-vector
	- The default gravity to use for physical modeling. This can be overridden by Stage attributes.
"timestep"
	- double
	- The timestep to use for forward simulation.
"friction_coefficient"
	- double
	- The default coefficient of friction. This can be overridden in Stage and Object Attributes.
"restitution_coefficient"
	- double
	- The default coefficient of restitution. This can be overridden in Stage and Object Attributes.
"rigid object paths"
	- list of strings
	- A list of locations to query for supported object files that should be available to be loaded into the world.

`Stage Attributes`_
===================
A stage in Habitat-Sim is a static object consisting of static background scenery wherein an agent acts.  Stage Attributes templates hold relevant information describing a stage's render and collision assets and physical properties.  Any source configuration files used to build these attributes should be named using the following format:

 	<stagename>.stage_config.json

`An example of an appropriately configured Stage Attributes file can be found below <../../../data/test_assets/scenes/stage_floor1.stage_config.json>`_:

.. include:: ../../data/test_assets/scenes/stage_floor1.stage_config.json
    :code: json


Stage Mesh Handles And Types
----------------------------

Below are the handles and descriptors for various mesh assets used by a stage.

"render_asset"
	- string
	- The name of the file describing the render mesh to be used by the stage.
"collision_asset"
	- string
	- The name of the file describing the collision mesh to be used by the stage.
"semantic_asset"
	- string
	- The name of the file describing the stage's semantic mesh.
"house_filename"
	- string
	- The name of the file containing semantic type maps and hierarchy.
"nav_asset"
	- string
	- The name of the file describing the NavMesh for this stage.

Stage Frame and Origin
----------------------

The tags below are used to build a coordinate frame for the stage, and will override any default values set based on render mesh file name/extension.  If either **"up"** or **"front"** are specified, both must be provided and they must be orthogonal.

"up"
	- 3-vector
	- Describes the **up** direction for the stage in the asset's local space.
"front"
	- 3-vector
	- Describes the **forward** direction for the stage in the asset's local space.
"origin"
	- 3-vector
	- Describes the **origin** of the stage in the world frame, for alignment purposes.

Stage Physics and Object-related Parameters
-------------------------------------------

Below are stage-specific physical and object-related quantities.  These values will override similarly-named values specified in the Physics Manager Attributes.

"scale"
	- 3-vector
	- The default scale to be used for the stage.
"gravity"
	- 3-vector
	- Gravity to use for physical modeling.
"is_collidable"
	- boolean
	- Whether the stage should be added to the collision and physics simulation world upon instancing.
"margin"
	- double
	- Distance margin for collision calculations.
"friction_coefficient"
	- double
	- The coefficient of friction.
"restitution_coefficient"
	- double
	- The coefficient of restitution.
"units_to_meters"
	- double
	- The conversion of given units to meters.
"requires_lighting"
	- boolean
	- Whether the stage should be rendered with lighting or flat shading.

`Object Attributes`_
====================
Object Attributes templates hold descriptive information for instancing rigid objects into Habitat-Sim.  These file names should be formatted as follows:

 	<objectname>.object_config.json

`An example of an appropriately configured Object Attributes file can be found below <../../../data/test_assets/objects/donut.object_config.json>`_:

.. include:: ../../data/test_assets/objects/donut.object_config.json
    :code: json

Object Mesh Handles And Types
-----------------------------

Below are the handles and descriptors for various mesh assets used by an object.

"render_asset"
	- string
	- The name of the file describing the render mesh to be used by the object.
"collision_asset"
	- string
	- The name of the file describing the collision mesh to be used by the object.

Object Frame and Origin
-----------------------

The tags below are used to build a coordinate frame for the object, and will override any default values set based on render mesh file name/extension.  If either **"up"** or **"front"** are specified, both must be provided and they must be orthogonal.  The object's COM is used as its origin.

"up"
	- 3-vector
	- Describes the **up** direction for the object in the asset's local space.
"front"
	- 3-vector
	- Describes the **forward** direction for the object in the asset's local space.


Below are object-specific physical quantities.  These values will override similarly-named values specified in a Physics Manager Attributes.

"scale"
	- 3-vector
	- The default scale to be used for the object.
"is_collidable"
	- boolean
	- Whether the object should be added to the simulation world with a collision shape upon instancing.
"margin"
	- double
	- Distance margin for collision calculations.
"friction_coefficient"
	- double
	- The coefficient of friction.
"restitution_coefficient"
	- double
	- The coefficient of restitution.
"units_to_meters"
	- double
	- The conversion of given units to meters.
"requires_lighting"
	- boolean
	- Whether the object should be rendered with lighting or flat shading.
"mass"
	- double
	- The mass of the object, for physics calculations.
"inertia"
	- 3-vector
	- The values of the diagonal of the inertia matrix for the object.  If not provided, will be computed automatically from the object's mass and bounding box.
"COM"
	- 3-vector
	- The center of mass for the object.  If this is not specified in JSON, it will be derived from the object's bounding box in Habitat-Sim.
"use_bounding_box_for_collision"
	- boolean
	- Whether or not to use the object's bounding box as collision geometry. Note: dynamic simulation will be significantly faster and more stable if this is true.
"join_collision_meshes"
	- boolean
	- Whether or not sub-components of the object's collision asset should be joined into a single unified collision object.
"semantic_id"
    - integer
	- The semantic id assigned to objects made with this configuration.

`Light Setup Attributes`_
=========================
Light Setup Attributes templates hold descriptive information for light setups into Habitat-Sim.  These file names should be formatted as follows:

 	<lightingname>.lighting_config.json

`An example of an appropriately configured Light Setup Attributes file can be found below <../../../data/test_assets/lights/test_lights.lighting_config.json>`_:

.. include:: ../../data/test_assets/lights/test_lights.lighting_config.json
    :code: json

The Light Setup attributes JSON should contain a single cell named "lights" that references a JSON object consisting of key-value pairs, where each key is a string ID that is unique to the lighting layout and the value is a JSON object containing appropriate combinations of the following data for the light type being described.

"position"
	- 3-vector
	- The position of the light, if the light is a point light.
"direction"
	- 3-vector
	- The direction of the light, if the light is a directional light.
"color"
	- 3-vector [R,G,B; each value 0->1]
	- RGB value for the light's color in linear space.
"intensity"
	- float
	- The intensity of the light. This color is multiplied by this value to account for rolloff.  Negative values are allowed and can be used to simulate shadows.
"type"
	- string
	- The type of the light.  "point" and "directional" are currently supported.
"position_model"
  - string
  - They frame to use to place the light. "global", meaning stage's origin, and "camera", meaning place relative to a (potentially moving) camera, are currently supported.
