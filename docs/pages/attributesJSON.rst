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
Physics Manager Attributes templates describe quantities pertinent to building the simulation world.  Any source configuration JSON files used to build these attributes should be named in the following format :

 	<worldname>.phys_scene_config.json

An example of an appropriate configured file can be found below :

.. include:: ../../data/test_assets/testing.phys_scene_config.json
    :code: json


Below are the supported JSON tags for Physics Manager Attributes templates, and their meanings.

"physics simulator"
	- string
	- What physics engine should be used for dynamics simulation.  Currently supports "bullet" for Bullet physics simulation, and "none", meaning kinematic motion is to be used.
"gravity"
	- 3-vector
	- Gravity to use for physical modeling. This can be overridden by Stage attributes.
"timestep"
	- double
	- The timestep to use for forward simulation.
"friction coefficient"
	- double
	- The coefficient of friction. This can be overridden in Stage and Object Attributes.
"restitution coefficient"
	- double
	- The coefficient of restitution. This can be overridden in Stage and Object Attributes.
"rigid object paths"
	- list of strings
	- A list of locations to query for supported object files that should be available to be loaded into the world.

`Stage Attributes`_
===================
A stage in Habitat-Sim is a static object consisting of background scenery wherein an agent acts.  Stage Attributes templates hold relevant information describing a stage's mesh, geometry and physical properties.  Any source configuration files used to build these attributes should be named in the format :

 	<stagename>.stage_config.json

An example of an appropriate configured file can be found below :

.. include:: ../../data/test_assets/scenes/stage_floor1.stage_config.json
    :code: json


Stage Mesh Handles And Types
----------------------------

Below are the handles and descriptors for various mesh assets used by a stage.

"render mesh"
	- string
	- The name of the file describing the render mesh to be used by the stage.
"collision mesh"
	- string
	- The name of the file describing the collision mesh to be used by the stage.
"semantic mesh"
	- string
	- The name of the file describing the stage's semantic mesh.
"house filename"
	- string
	- The name of the file containing semantic type maps and hierarchy.
"nav mesh"
	- string
	- The name of the file describing the NavMesh for this stage.

Stage Frame and Origin
----------------------

The tags below are used to build a coordinate frame for the stage, and will override any default values set based on render mesh file name/extension.  If either **"up"** or **"front"** are specified, both must be provided and they must be orthogonal.

"up"
	- 3-vector
	- Describes the **up** direction for the stage.
"front"
	- 3-vector
	- Describes the **forward** direction for the stage.
"origin"
	- 3-vector
	- Describes the **origin** of the stage, for alignment purposes.

Stage Physics and Object-related Parameters
-------------------------------------------

Below are stage-specific physical and object-related quantities.  These values will override similarly-named values specified in the Physics Manager Attributes.

"scale"
	- 3-vector
	- The default scale to be used for the stage.
"gravity"
	- 3-vector
	- Gravity to use for physical modeling.
"margin"
	- double
	- Distance margin for collision calculations.
"friction coefficient"
	- double
	- The coefficient of friction.
"restitution coefficient"
	- double
	- The coefficient of restitution.
"units to meters"
	- double
	- The conversion of given units to meters.
"requires lighting"
	- boolean
	- Whether or not the stage should use lighting.

`Object Attributes`_
====================
Object Attributes templates hold descriptive information for type of object that can be loaded into Habitat.  These files should be named in the format :

 	<objectname>.phys_properties.json

An example of an appropriate configured file can be found below :

.. include:: ../../data/test_assets/objects/donut.phys_properties.json
    :code: json

Object Mesh Handles And Types
-----------------------------

Below are the handles and descriptors for various mesh assets used by an object.

"render mesh"
	- string
	- The name of the file describing the render mesh to be used by the object.
"collision mesh"
	- string
	- The name of the file describing the collision mesh to be used by the object.

Object Frame and Origin
-----------------------

The tags below are used to build a coordinate frame for the object, and will override any default values set based on render mesh file name/extension.  If either **"up"** or **"front"** are specified, both must be provided and they must be orthogonal.  The object's COM is used as its origin.

"up"
	- 3-vector
	- Describes the **up** direction for the object.
"front"
	- 3-vector
	- Describes the **forward** direction for the object.


Below are object-specific physical quantities.  These values will override similarly-named values specified in a Physics Manager Attributes.

"scale"
	- 3-vector
	- The default scale to be used for the object.
"margin"
	- double
	- Distance margin for collision calculations.
"friction coefficient"
	- double
	- The coefficient of friction.
"restitution coefficient"
	- double
	- The coefficient of restitution.
"units to meters"
	- double
	- The conversion of given units to meters.
"requires lighting"
	- boolean
	- Whether or not the object should use lighting.
"mass"
	- double
	- The mass of the object, for physics calculations.
"inertia"
	- 3-vector
	- The values of the diagonal of the inertia matrix for the object.
"COM"
	- 3-vector
	- The center of mass for the object.  If this is not specified in JSON, it will be derived from the object's bounding box in Habitat-Sim.
"use bounding box for collision"
	- boolean
	- Whether collision calculations should be based on object's bounding box geometry.
"join collision meshes"
	- boolean
	- Whether collision mesh assets should be joined into a single unified collision object.
