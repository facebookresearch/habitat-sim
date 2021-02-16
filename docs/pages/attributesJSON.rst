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
Physics Manager Attributes templates describe quantities, such as timestep and what physics engine to use, pertinent to building the simulation world. Any source configuration JSON files used to build these attributes should be formatted as follows:

 	<worldname>.physics_config.json

`An example of an appropriately configured Physics Manager Attributes file can be found below <../../../data/test_assets/testing.physics_config.json>`_:

.. include:: ../../data/test_assets/testing.physics_config.json
    :code: json


Below are the supported JSON tags for Physics Manager Attributes templates, and their meanings.

"physics_simulator"
	- string
	- What physics engine should be used for dynamics simulation. Currently supports "bullet" for Bullet physics simulation, and "none", meaning kinematic motion is to be used, where collisions will still occur but forces are not applied.
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


`Scene Dataset Attributes`_
===========================
Scene Dataset Attributes describe the names and file locations of all the assets and configuration files that make up a Scene Dataset. They are designed as a kind of index for a scene dataset, organizing the links to all the constituent components of the dataset into appropriate thematic categories, as well as providing an easy mechanism for customizing any of the configurations, all in a single file. All file paths referenced in this configuration should be relative to the directory where this Scene Dataset configuration resides.

These file names should be formatted as follows:

 	<scenedatasetname>.scene_dataset_config.json

`An example of an appropriately configured Scene Dataset Configuration file can be found below <../../../data/test_assets/dataset_tests/test_dataset.scene_dataset_config.json>`_:

.. include:: ../../data/test_assets/dataset_tests/test_dataset.scene_dataset_config.json
    :code: json

'Scene Dataset: Stages, Objects, Light Setups and Scene Instances'
------------------------------------------------------------------

The Scene Dataset Attributes configuration consists of a hierarchy of JSON components used to specify all the component assets and attributes used within a scene dataset. Tags "stages", "objects", "light_setups" and "scene_instances" reference JSON objects that list the included assets and configurations for the specified type of construction. The JSON objects referenced by each of these 4 tags support the following tags:

"default_attributes":
  - JSON object
  - This tag references the default configuration values to be used for any attributes' values not explicitly specified for whatever type of construction (i.e. "stages", "objects", etc.) is being referenced. See the individual attributes types' documentation for supported tags and format.

"paths":
  - JSON object
  - This tag references an object consisting of key-value pairs where each key is the exension of a particular config (".json") or (for "objects" and "stages" only) a valid asset (such as ".glb"), and the value is a list to the paths to query for the desired configuration files or assets.  For asset-related k-v pairs, an attributes will be generated for each asset found, using any specified defaults provided, with the render and collision asset handles set to the asset's name.

"configs":
  - array of JSON objects
  - This array provides the user with an easy interface to override or extend any existing configurations.  Each array element is a JSON object that supports the creation of customized attributes.  Either "original_file" or "template_handle" (or both) must be specified.  The following tags are supported in each individual JSON object:

    "original_file":
    - string
    - If provided, this tag is the qualified filename of an existing attributes to use as a baseline attributes. Either this tag or "template_handle" must be specified for each JSON object (although both can also be given)

    "template_handle"
    - string
    - If provided, this is the handle to use to store the new attributes in the Scene Dataset library; if not provided, any customizations will be stored under the "original_file"-specified name. Either this tag or "original_file" must be specified for each JSON object (although both can also be given)

    "attributes"
    - JSON object
    - This tag references a configuration appropriate for the construction type. If "original_file" is specified, all the values from "original_file" are used, and overridden by any specified in this tag.  If "template_handle" is specified, the new attributes are saved with the specified "template_handle", otherwise they are saved with the "original_file" handle. See the individual attributes types' documentation for supported tags and format.

'Scene Dataset: Navmeshes and Semantic Scene Descriptors'
---------------------------------------------------------

The Scene Dataset Attributes configuration also contains references to the navmeshes, using the JSON tag "navmesh_instances", and semantic scene descriptor files, using the tag "semantic_scene_descriptor_instances", that are available to the Scene Dataset.  These tags each reference a JSON object consisting of key-value pairs where the keys are the aliases to use in scene instances for a particular navmesh or semantic scene descriptor, and the values are the paths to the navmesh or SSD file, respectively.

`Scene Instance Attributes`_
============================
Scene Instance Attributes templates describe all the required assets and configurations used to construct a scene instance.  This includes references to a stage, any objects to be placed in the scene, the lighting configuration and navmesh to be used and the appropriate semantic scene descriptor for the scene.

These file names should be formatted as follows:

 	<scenename>.scene_instance.json

`An example of an appropriately configured Scene Instance Attributes file can be found below <../../../data/test_assets/dataset_tests/scenes/dataset_test_scene.scene_instance.json>`_:

.. include:: ../../data/test_assets/dataset_tests/scenes/dataset_test_scene.scene_instance.json
    :code: json

'Scene Instance Parameters'
---------------------------

"translation_origin"
  - string
  - Specifies scene instance default for translation origin. Values are case-insensitive. Supports "asset_local", where the origin is defined in the individual asset's local frame, "com", where the asset's center of mass is the origin for transformations, or "unknown" if the value is unknown. This value can be overridden for the stage and any object instances in the scene.

"default_lighting"
  - string
  - The handle of the lighting setup configuration, defined in the Scene Dataset Configuration, that should be usedd when instantiating lights in the scene.

"navmesh_instance"
  - string
  - The alias of the scene's navmesh asset, which is mapped to a file handle in the Scene Dataset Configuration.

"semantic_scene_instance"
  - string
  - The alias of the scene's semantic scene descriptor file, which is mapped to a file handle in the Scene Dataset Configuration.

'Scene Instance Stage and Object parameters'
--------------------------------------------

The Scene Instance Attributes specifies all the required parameters to instantiate the scene's stage and objects, such as source attributes handle and intial transformation. These creation parameters are called Scene Object Instance Attributes, and they are tagged in the Scene Instance Attributes configuration file as follows:

"stage_instance"
  - json object
  - the required Scene Object Instance parameters to create the stage in the scene.

"object_instances"
  - array of json ojbects
  - each element of the array is a Scene Object Instance parameters required to create a single object in the scene.

The JSON tags the Scene Object Instance Attributes support are as follows:

"template_name"
  - string
  - Dataset-unique name of the template for the stage or object attributes desired to instantiate. Since attributes templates can encode full file paths, their handles can be very long; therefore, the "template_name" field supports substring lookup, and an incomplete handle can be used for this field so long as it uniquely maps to the full handle specified in the Scene Dataset Attributes.

"translation_origin"
  - string
  - Specifies the translation origin to use for the instantiation of the specified object. Values are case-insensitive. Supports "asset_local", where the origin is defined in the individual asset's local frame, "com", where the asset's center of mass is the origin for transformations, or "unknown" if the value is unknown. If this value is specified, and not set to "unknown", it will override the top-level default, if any exists. Currently stages do not support instantiation transformations.

"motion_type"
  - string
  - The desired motion type of the object being instanced. Values are case-insensitive. Values supported are "static", "kinematic", and "dynamic". This field is ignored for stages, which are always created with STATIC motion type.

"translation"
  - 3-vector
  - The translation from the origin of the scene where the object should be placed. Currently stages do not support instantiation transformations.

"rotation"
  - Quaternion (read as [w,x,y,z], with idx 0 referencing the scalar part.)
  - The rotation to be applied to the object upon instantiation. Currently stages do not support instantiation transformations.


`Stage Attributes`_
===================
A stage in Habitat-Sim is a static object consisting of static background scenery wherein an agent acts. Stage Attributes templates hold relevant information describing a stage's render and collision assets and physical properties. Any source configuration files used to build these attributes should be named using the following format:

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

The tags below are used to build a coordinate frame for the stage, and will override any default values set based on render mesh file name/extension. If either **"up"** or **"front"** are specified, both must be provided and they must be orthogonal.

"up"
	- 3-vector
	- Describes the **up** direction for the stage in the asset's local space.
"front"
	- 3-vector
	- Describes the **forward** direction for the stage in the asset's local space.
"origin"
	- 3-vector
	- Describes the **origin** of the stage in the world frame, for alignment purposes.

Stage Physical Parameters
-------------------------

Below are stage-specific physical quantities. These values will override similarly-named values specified in the Physics Manager Attributes.

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
Object Attributes templates hold descriptive information for instancing rigid objects into Habitat-Sim. These file names should be formatted as follows:

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

Object Frame
------------

The tags below are used to build a coordinate frame for the object, and will override any default values set based on render mesh file name/extension. If either **"up"** or **"front"** are specified, both must be provided and they must be orthogonal. The object's center of mass is used as its origin.

"up"
	- 3-vector
	- Describes the **up** direction for the object in the asset's local space.
"front"
	- 3-vector
	- Describes the **forward** direction for the object in the asset's local space.

Object Physical Parameters
--------------------------

Below are object-specific physical quantities. These values will override similarly-named values specified in a Physics Manager Attributes.

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
	- The values of the diagonal of the inertia matrix for the object. If not provided, will be computed automatically from the object's mass and bounding box.
"COM"
	- 3-vector
	- The center of mass for the object. If this is not specified in JSON, it will be derived from the object's bounding box in Habitat-Sim.
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
Light Setup Attributes templates hold descriptive information for light setups into Habitat-Sim. These file names should be formatted as follows:

 	<lightingname>.lighting_config.json

`An example of an appropriately configured Light Setup Attributes file can be found below <../../../data/test_assets/dataset_tests/lights/dataset_test_lights.lighting_config.json>`_:

.. include:: ../../data/test_assets/dataset_tests/lights/dataset_test_lights.lighting_config.json
    :code: json

'Light Setup Parameters'
------------------------

The Light Setup attributes JSON configuration should contain a single cell named "lights" that references a JSON object consisting of key-value pairs, where each key is a string ID that is unique to the lighting layout and the value is a JSON object containing appropriate combinations of the following data for the light type being described.

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
	- The intensity of the light. This color is multiplied by this value to account for rolloff. Negative values are allowed and can be used to simulate shadows.
"type"
	- string
	- The type of the light. "point" and "directional" are currently supported.
