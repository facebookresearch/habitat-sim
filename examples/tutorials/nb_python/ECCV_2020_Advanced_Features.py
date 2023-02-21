# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     collapsed_sections:
#     - dw0hKCHX9dii
#     - LfhL_ZcW-HsF
#     - -dFprz9y_HSQ
#     - twCl8wAR_R4d
#     name: 'ECCV 2020: Habitat-sim Advanced Features'
#     private_outputs: true
#     provenance: []
#   jupytext:
#     cell_metadata_filter: -all
#     formats: nb_python//py:percent,colabs//ipynb
#     notebook_metadata_filter: all
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.13.7
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %% [markdown]
# <a href="https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Advanced_Features.ipynb" target="_parent"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a>

# %% [markdown]
# #Habitat-sim Advanced Features
#
# This tutorial presents a number of advanced feature examples for using Habitat-sim, including:
#
# - Tracking object motion with a camera
# - Projecting 3D points into 2D
# - Configuring semantic ids for objects
# - Object/Asset configuration via template libraries

# %%
# @title Installation { display-mode: "form" }
# @markdown (double click to show code).

# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/main/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s

# %%
# @title Path Setup and Imports { display-mode: "form" }
# @markdown (double click to show code).

# %cd /content/habitat-sim
## [setup]
import math
import os
import random
import sys

import git
import magnum as mn
import numpy as np

# %matplotlib inline
from matplotlib import pyplot as plt
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as ut
from habitat_sim.utils import viz_utils as vut

try:
    import ipywidgets as widgets
    from IPython.display import display as ipydisplay

    # For using jupyter/ipywidget IO components

    HAS_WIDGETS = True
except ImportError:
    HAS_WIDGETS = False


if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
# fmt: off
output_directory = "examples/tutorials/advanced_features_output/"  # @param {type:"string"}
# fmt: on
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

# define some globals the first time we run.
if "sim" not in globals():
    global sim
    sim = None
    global obj_attr_mgr
    obj_attr_mgr = None
    global prim_attr_mgr
    obj_attr_mgr = None
    global stage_attr_mgr
    stage_attr_mgr = None
    global rigid_obj_mgr
    rigid_obj_mgr = None


# %%
# @title Define Template Dictionary Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown This cell defines utility functions that expose Attribute template object properties.


# This method builds a dictionary of k-v pairs of attribute property names and
# values shared by all attribute template types.  The values are tuples with the
# first entry being the value and the second being whether the property is
# editable and the third being the type.
def build_dict_of_Default_attrs(template):
    res_dict = {
        "handle": (template.handle, True, "string"),
        # Read-only values
        "template_id": (template.template_id, False, "int"),
        "template_class": (template.template_class, False, "string"),
        "file_directory": (template.file_directory, False, "string"),
    }
    return res_dict


# This method builds a dictionary of k-v pairs of attribute property names and
# values shared by templates of physically modeled constructs (scenes and
# objects). The values are tuples with the first entry being the value and the
# second being whether the property is editable and the third being the type.
def build_dict_of_PhyObj_attrs(phys_obj_template):
    res_dict = build_dict_of_Default_attrs(phys_obj_template)
    res_dict["scale"] = (phys_obj_template.scale, True, "vector")
    res_dict["margin"] = (phys_obj_template.margin, True, "double")
    res_dict["friction_coefficient"] = (
        phys_obj_template.friction_coefficient,
        True,
        "double",
    )
    res_dict["restitution_coefficient"] = (
        phys_obj_template.restitution_coefficient,
        True,
        "double",
    )
    res_dict["render_asset_handle"] = (
        phys_obj_template.render_asset_handle,
        True,
        "string",
    )
    res_dict["collision_asset_handle"] = (
        phys_obj_template.collision_asset_handle,
        True,
        "string",
    )
    res_dict["force_flat_shading"] = (
        phys_obj_template.force_flat_shading,
        True,
        "boolean",
    )
    # New fields, uncomment upon updating conda 8/4/20
    res_dict["orient_up"] = (phys_obj_template.orient_up, True, "vector")
    res_dict["orient_front"] = (phys_obj_template.orient_front, True, "vector")
    res_dict["units_to_meters"] = (phys_obj_template.units_to_meters, True, "double")
    res_dict["render_asset_type"] = (phys_obj_template.render_asset_type, True, "int")
    res_dict["collision_asset_type"] = (
        phys_obj_template.collision_asset_type,
        True,
        "int",
    )
    # Read-only values
    res_dict["render_asset_is_primitive"] = (
        phys_obj_template.render_asset_is_primitive,
        False,
        "boolean",
    )
    res_dict["collision_asset_is_primitive"] = (
        phys_obj_template.collision_asset_is_primitive,
        False,
        "boolean",
    )
    res_dict["use_mesh_for_collision"] = (
        phys_obj_template.use_mesh_for_collision,
        False,
        "boolean",
    )
    res_dict["is_dirty"] = (phys_obj_template.is_dirty, False, "boolean")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed object template. The values are tuples with the first
# entry being the value,the second being whether the property is editable and
# the third being the type.
def build_dict_of_Object_attrs(obj_template):
    res_dict = build_dict_of_PhyObj_attrs(obj_template)
    res_dict["com"] = (obj_template.com, True, "vector")
    res_dict["compute_COM_from_shape"] = (
        obj_template.compute_COM_from_shape,
        True,
        "boolean",
    )
    res_dict["mass"] = (obj_template.mass, True, "double")
    res_dict["inertia"] = (obj_template.inertia, True, "vector")
    res_dict["linear_damping"] = (obj_template.linear_damping, True, "double")
    res_dict["angular_damping"] = (obj_template.angular_damping, True, "double")
    res_dict["bounding_box_collisions"] = (
        obj_template.bounding_box_collisions,
        True,
        "boolean",
    )
    res_dict["join_collision_meshes"] = (
        obj_template.join_collision_meshes,
        True,
        "boolean",
    )
    # res_dict["is_visible"] = (obj_template.is_visible, True, "boolean")
    # res_dict["is_collidable"] = (obj_template.is_collidable, True, "boolean")
    res_dict["semantic_id"] = (obj_template.semantic_id, True, "int")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed scene template. The values are tuples with the first
# entry being the value,the second being whether the property is editable and
# the third being the type.
def build_dict_of_Stage_attrs(scene_template):
    res_dict = build_dict_of_PhyObj_attrs(scene_template)
    res_dict["gravity"] = (scene_template.gravity, True, "vector")
    res_dict["origin"] = (scene_template.origin, True, "vector")
    res_dict["semantic_asset_handle"] = (
        scene_template.semantic_asset_handle,
        True,
        "string",
    )
    res_dict["semantic_asset_type"] = (scene_template.semantic_asset_type, True, "int")
    res_dict["navmesh_asset_handle"] = (
        scene_template.navmesh_asset_handle,
        True,
        "string",
    )
    res_dict["house_filename"] = (scene_template.house_filename, True, "string")
    # res_dict["light_setup"] = (scene_template.light_setup, True, "string")
    # res_dict["frustrum_culling"] = (scene_template.frustrum_culling, True, "boolean")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed physics manager template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_PhysicsSim_attrs(physics_template):
    res_dict = build_dict_of_Default_attrs(physics_template)
    res_dict["gravity"] = (physics_template.gravity, True, "vector")
    res_dict["timestep"] = (physics_template.timestep, True, "double")
    res_dict["max_substeps"] = (physics_template.max_substeps, True, "int")
    res_dict["friction_coefficient"] = (
        physics_template.friction_coefficient,
        True,
        "double",
    )
    res_dict["restitution_coefficient"] = (
        physics_template.restitution_coefficient,
        True,
        "double",
    )
    # Read-only values
    res_dict["simulator"] = (physics_template.simulator, False, "string")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values that are shared among all primitive asset attributes templates.
# The values are tuples with the first entry being the value,the second being
# whether the property is editable and the third being the type.
def build_dict_of_prim_attrs(prim_template):
    res_dict = build_dict_of_Default_attrs(prim_template)
    res_dict["use_texture_coords"] = (prim_template.use_texture_coords, True, "boolean")
    res_dict["use_tangents"] = (prim_template.use_tangents, True, "boolean")
    res_dict["num_rings"] = (prim_template.num_rings, True, "int")
    res_dict["num_segments"] = (prim_template.num_segments, True, "int")
    res_dict["half_length"] = (prim_template.half_length, True)
    # Read-only values
    res_dict["prim_obj_class_name"] = (
        prim_template.prim_obj_class_name,
        False,
        "string",
    )
    res_dict["prim_obj_type"] = (prim_template.prim_obj_type, False, "int")
    res_dict["is_valid_template"] = (prim_template.is_valid_template, False, "boolean")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed capsule primitive template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_Capsule_prim_attrs(capsule_template):
    res_dict = build_dict_of_prim_attrs(capsule_template)
    res_dict["hemisphere_rings"] = (capsule_template.hemisphere_rings, True, "int")
    res_dict["cylinder_rings"] = (capsule_template.cylinder_rings, True, "int")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed cone primitive template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_Cone_prim_attrs(cone_template):
    res_dict = build_dict_of_prim_attrs(cone_template)
    res_dict["use_cap_end"] = (cone_template.use_cap_end, True, "boolean")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed cube primitive template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_Cube_prim_attrs(cube_template):
    res_dict = build_dict_of_prim_attrs(cube_template)
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed cylinder primitive template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_Cylinder_prim_attrs(cylinder_template):
    res_dict = build_dict_of_prim_attrs(cylinder_template)
    res_dict["use_cap_ends"] = (cylinder_template.use_cap_ends, True, "boolean")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed icosphere primitive template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_Icosphere_prim_attrs(icosphere_template):
    res_dict = build_dict_of_prim_attrs(icosphere_template)
    res_dict["subdivisions"] = (icosphere_template.subdivisions, True, "int")
    return res_dict


# This method will build a dict containing k-v pairs of attribute property names
# and values for the passed UV-Sphere primitive template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_UVSphere_prim_attrs(uvsphere_template):
    res_dict = build_dict_of_prim_attrs(uvsphere_template)
    return res_dict


# This method will deduce the appropriate template type and build the subsequent
# dictionary containing containing k-v pairs of template property names
# and values for the passed template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_from_template(template):
    template_class = template.template_class
    if "ObjectAttributes" in template_class:
        return build_dict_of_Object_attrs(template)
    if "StageAttributes" in template_class:
        return build_dict_of_Stage_attrs(template)
    if "PhysicsManagerAttributes" in template_class:
        return build_dict_of_PhysicsSim_attrs(template)
    if "CapsulePrimitiveAttributes" in template_class:
        return build_dict_of_Capsule_prim_attrs(template)
    if "ConePrimitiveAttributes" in template_class:
        return build_dict_of_Cone_prim_attrs(template)
    if "CubePrimitiveAttributes" in template_class:
        return build_dict_of_Cube_prim_attrs(template)
    if "CylinderPrimitiveAttributes" in template_class:
        return build_dict_of_Cylinder_prim_attrs(template)
    if "IcospherePrimitiveAttributes" in template_class:
        return build_dict_of_Icosphere_prim_attrs(template)
    if "UVSpherePrimitiveAttributes" in template_class:
        return build_dict_of_UVSphere_prim_attrs(template)
    print("Unknown template type : %s " % template_class)
    return None


# This will set a template's attributes from the passed dictionary
def set_template_properties_from_dict(template, template_dict):
    for k, v in template_dict.items():
        setattr(template, k, v[0])
    return template


# This will display all the properties of an attributes template
def show_template_properties(template):
    template_dict = build_dict_from_template(template)
    print("Template {} has : ".format(template.handle))
    for k, v in template_dict.items():
        print(
            "\tProperty {} has value {} of type {} that is editable : {}".format(
                k, v[0], v[2], v[1]
            )
        )


# %%
# @title Define Configuration Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown This cell defines a number of utility functions used throughout the tutorial to make simulator reconstruction easy:
# @markdown - make_cfg
# @markdown - make_default_settings
# @markdown - make_simulator_from_settings


def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene_id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]
    # Optional; Specify the location of an existing scene dataset configuration
    # that describes the locations and configurations of all the assets to be used
    if "scene_dataset_config" in settings:
        sim_cfg.scene_dataset_config_file = settings["scene_dataset_config"]

    # Note: all sensors must have the same resolution
    sensor_specs = []
    if settings["color_sensor_1st_person"]:
        color_sensor_1st_person_spec = habitat_sim.CameraSensorSpec()
        color_sensor_1st_person_spec.uuid = "color_sensor_1st_person"
        color_sensor_1st_person_spec.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor_1st_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        color_sensor_1st_person_spec.position = [0.0, settings["sensor_height"], 0.0]
        color_sensor_1st_person_spec.orientation = [
            settings["sensor_pitch"],
            0.0,
            0.0,
        ]
        color_sensor_1st_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(color_sensor_1st_person_spec)
    if settings["depth_sensor_1st_person"]:
        depth_sensor_1st_person_spec = habitat_sim.CameraSensorSpec()
        depth_sensor_1st_person_spec.uuid = "depth_sensor_1st_person"
        depth_sensor_1st_person_spec.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor_1st_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        depth_sensor_1st_person_spec.position = [0.0, settings["sensor_height"], 0.0]
        depth_sensor_1st_person_spec.orientation = [
            settings["sensor_pitch"],
            0.0,
            0.0,
        ]
        depth_sensor_1st_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(depth_sensor_1st_person_spec)
    if settings["semantic_sensor_1st_person"]:
        semantic_sensor_1st_person_spec = habitat_sim.CameraSensorSpec()
        semantic_sensor_1st_person_spec.uuid = "semantic_sensor_1st_person"
        semantic_sensor_1st_person_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
        semantic_sensor_1st_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        semantic_sensor_1st_person_spec.position = [
            0.0,
            settings["sensor_height"],
            0.0,
        ]
        semantic_sensor_1st_person_spec.orientation = [
            settings["sensor_pitch"],
            0.0,
            0.0,
        ]
        semantic_sensor_1st_person_spec.sensor_subtype = (
            habitat_sim.SensorSubType.PINHOLE
        )
        sensor_specs.append(semantic_sensor_1st_person_spec)
    if settings["color_sensor_3rd_person"]:
        color_sensor_3rd_person_spec = habitat_sim.CameraSensorSpec()
        color_sensor_3rd_person_spec.uuid = "color_sensor_3rd_person"
        color_sensor_3rd_person_spec.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor_3rd_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        color_sensor_3rd_person_spec.position = [
            0.0,
            settings["sensor_height"] + 0.2,
            0.2,
        ]
        color_sensor_3rd_person_spec.orientation = [-math.pi / 4, 0, 0]
        color_sensor_3rd_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(color_sensor_3rd_person_spec)

    # Here you can specify the amount of displacement in a forward action and the turn angle
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


def make_default_settings():
    settings = {
        "width": 720,  # Spatial resolution of the observations
        "height": 544,
        "scene": "./data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",  # Scene path
        "scene_dataset": "./data/scene_datasets/mp3d_example/mp3d.scene_dataset_config.json",  # mp3d scene dataset
        "default_agent": 0,
        "sensor_height": 1.5,  # Height of sensors in meters
        "sensor_pitch": -math.pi / 8.0,  # sensor pitch (x rotation in rads)
        "color_sensor_1st_person": True,  # RGB sensor
        "color_sensor_3rd_person": False,  # RGB sensor 3rd person
        "depth_sensor_1st_person": False,  # Depth sensor
        "semantic_sensor_1st_person": False,  # Semantic sensor
        "seed": 1,
        "enable_physics": True,  # enable dynamics simulation
    }
    return settings


def make_simulator_from_settings(sim_settings):
    cfg = make_cfg(sim_settings)
    # clean-up the current simulator instance if it exists
    global sim
    global obj_attr_mgr
    global prim_attr_mgr
    global stage_attr_mgr
    global rigid_obj_mgr

    if sim != None:
        sim.close()
    # initialize the simulator
    sim = habitat_sim.Simulator(cfg)
    # Managers of various Attributes templates
    obj_attr_mgr = sim.get_object_template_manager()
    obj_attr_mgr.load_configs(str(os.path.join(data_path, "objects/example_objects")))
    prim_attr_mgr = sim.get_asset_template_manager()
    stage_attr_mgr = sim.get_stage_template_manager()
    # Manager providing access to rigid objects
    rigid_obj_mgr = sim.get_rigid_object_manager()

    # UI-populated handles used in various cells.  Need to initialize to valid
    # value in case IPyWidgets are not available.
    # Holds the user's desired file-based object template handle
    global sel_file_obj_handle
    sel_file_obj_handle = obj_attr_mgr.get_file_template_handles()[0]
    # Holds the user's desired primitive-based object template handle
    global sel_prim_obj_handle
    sel_prim_obj_handle = obj_attr_mgr.get_synth_template_handles()[0]
    # Holds the user's desired primitive asset template handle
    global sel_asset_handle
    sel_asset_handle = prim_attr_mgr.get_template_handles()[0]


# %%
# @title Define Simulation Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown - simulate
# @markdown - init_camera_track_config
# @markdown - restore_camera_track_config
# @markdown - camera_track_simulate


def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())
    return observations


# Save sensor and agent state to a dictionary, so that initial values can be reset
# Used at beginning of cell that directly modifies camera (i.e. tracking an object)
def init_camera_track_config(sim, sensor_name="color_sensor_1st_person", agent_ID=0):
    init_state = {}
    visual_sensor = sim._sensors[sensor_name]
    # save ref to sensor being used
    init_state["visual_sensor"] = visual_sensor
    init_state["position"] = np.array(visual_sensor._spec.position)
    init_state["orientation"] = np.array(visual_sensor._spec.orientation)
    # set the color sensor transform to be the agent transform
    visual_sensor._spec.position = np.array([0, 0, 0])
    visual_sensor._spec.orientation = np.array([0, 0, 0])
    visual_sensor._sensor_object.set_transformation_from_spec()
    # save ID of agent being modified
    init_state["agent_ID"] = agent_ID
    # save agent initial state
    init_state["agent_state"] = sim.get_agent(agent_ID).get_state()
    # boost the agent off the floor
    sim.get_agent(agent_ID).scene_node.translation += np.array([0, 1.5, 0])
    return init_state


# Reset sensor and agent state using dictionary built in init_camera_track_config
# Used at end of cell that directly modifies camera (i.e. tracking an object)
def restore_camera_track_config(sim, init_state):
    visual_sensor = init_state["visual_sensor"]
    agent_ID = init_state["agent_ID"]
    # reset the sensor state for other examples
    visual_sensor._spec.position = init_state["position"]
    visual_sensor._spec.orientation = init_state["orientation"]
    visual_sensor._sensor_object.set_transformation_from_spec()
    # restore the agent's state to what was savedd in init_camera_track_config
    sim.get_agent(agent_ID).set_state(init_state["agent_state"])


# Simulate scene while having camera track COM of objects of interest
def camera_track_simulate(sim, objects, dt=2.0, get_frames=True, agent_ID=0):
    start_time = sim.get_world_time()
    observations = []
    num_objs = len(objects)
    if num_objs == 0:
        print("camera_track_simulate : Aborting, no objects sent to track")
        return observations
    agent = sim.get_agent(agent_ID)
    # define up vector for tracking calc
    up_vec = np.array([0, 1.0, 0])
    # partition to speed up
    time_step = 1.0 / 60.0
    # process for multiple objects
    while sim.get_world_time() - start_time < dt:
        sim.step_physics(time_step)
        # set agent state to look at object
        camera_position = agent.scene_node.translation
        camera_look_at = objects[0].translation
        for i in range(1, num_objs):
            camera_look_at += objects[i].translation
        camera_look_at /= len(objects)
        agent.scene_node.rotation = mn.Quaternion.from_matrix(
            mn.Matrix4.look_at(camera_position, camera_look_at, up_vec).rotation()  # up
        )
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations


# Set an object transform relative to the agent state
def set_object_state_from_agent(
    sim,
    obj,
    offset=np.array([0, 2.0, -1.5]),
    orientation=mn.Quaternion(((0, 0, 0), 1)),
):
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    ob_translation = agent_transform.transform_point(offset)
    obj.translation = ob_translation
    obj.rotation = orientation


# %%
# @title Define Visualization Utility Function { display-mode: "form" }
# @markdown (double click to show code)
# @markdown - display_sample


# Change to do something like this maybe: https://stackoverflow.com/a/41432704
def display_sample(
    rgb_obs, semantic_obs=np.array([]), depth_obs=np.array([]), key_points=None
):
    from habitat_sim.utils.common import d3_40_colors_rgb

    rgb_img = Image.fromarray(rgb_obs, mode="RGBA")

    arr = [rgb_img]
    titles = ["rgb"]
    if semantic_obs.size != 0:
        semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
        semantic_img.putpalette(d3_40_colors_rgb.flatten())
        semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
        semantic_img = semantic_img.convert("RGBA")
        arr.append(semantic_img)
        titles.append("semantic")

    if depth_obs.size != 0:
        depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
        arr.append(depth_img)
        titles.append("depth")

    plt.figure(figsize=(12, 8))
    for i, data in enumerate(arr):
        ax = plt.subplot(1, 3, i + 1)
        ax.axis("off")
        ax.set_title(titles[i])
        # plot points on images
        if key_points is not None:
            for point in key_points:
                plt.plot(point[0], point[1], marker="o", markersize=10, alpha=0.8)
        plt.imshow(data)

    plt.show(block=False)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.display
    display = args.display
    make_video = args.make_video
else:
    show_video = False
    make_video = False
    display = False


# %%
# @title Define Colab GUI Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown This cell provides utility functions to build and manage IPyWidget interactive components.


# Event handler for dropdowns displaying file-based object handles
def on_file_obj_ddl_change(ddl_values):
    global sel_file_obj_handle
    sel_file_obj_handle = ddl_values["new"]
    return sel_file_obj_handle


# Event handler for dropdowns displaying prim-based object handles
def on_prim_obj_ddl_change(ddl_values):
    global sel_prim_obj_handle
    sel_prim_obj_handle = ddl_values["new"]
    return sel_prim_obj_handle


# Event handler for dropdowns displaying asset handles
def on_prim_ddl_change(ddl_values):
    global sel_asset_handle
    sel_asset_handle = ddl_values["new"]
    return sel_asset_handle


# Build a dropdown list holding obj_handles and set its event handler
def set_handle_ddl_widget(obj_handles, handle_types, sel_handle, on_change):
    sel_handle = obj_handles[0]
    descStr = handle_types + " Template Handles:"
    style = {"description_width": "300px"}
    obj_ddl = widgets.Dropdown(
        options=obj_handles,
        value=sel_handle,
        description=descStr,
        style=style,
        disabled=False,
        layout={"width": "max-content"},
    )

    obj_ddl.observe(on_change, names="value")
    return obj_ddl, sel_handle


def set_button_launcher(desc):
    button = widgets.Button(
        description=desc,
        layout={"width": "max-content"},
    )
    return button


def make_sim_and_vid_button(prefix, dt=1.0):
    if not HAS_WIDGETS:
        return

    def on_sim_click(b):
        observations = simulate(sim, dt=dt)
        vut.make_video(
            observations,
            "color_sensor_1st_person",
            "color",
            output_path + prefix,
            open_vid=show_video,
        )

    sim_and_vid_btn = set_button_launcher("Simulate and Make Video")
    sim_and_vid_btn.on_click(on_sim_click)
    ipydisplay(sim_and_vid_btn)


def make_clear_all_objects_button():
    if not HAS_WIDGETS:
        return

    def on_clear_click(b):
        rigid_obj_mgr.remove_all_objects()

    clear_objs_button = set_button_launcher("Clear all objects")
    clear_objs_button.on_click(on_clear_click)
    ipydisplay(clear_objs_button)


# Builds widget-based UI components
def build_widget_ui(obj_attr_mgr, prim_attr_mgr):
    # Holds the user's desired file-based object template handle
    global sel_file_obj_handle
    sel_file_obj_handle = ""

    # Holds the user's desired primitive-based object template handle
    global sel_prim_obj_handle
    sel_prim_obj_handle = ""

    # Holds the user's desired primitive asset template handle
    global sel_asset_handle
    sel_asset_handle = ""

    # Construct DDLs and assign event handlers
    # All file-based object template handles
    file_obj_handles = obj_attr_mgr.get_file_template_handles()
    # All primitive asset-based object template handles
    prim_obj_handles = obj_attr_mgr.get_synth_template_handles()
    # All primitive asset handles template handles
    prim_asset_handles = prim_attr_mgr.get_template_handles()
    # If not using widgets, set as first available handle
    if not HAS_WIDGETS:
        sel_file_obj_handle = file_obj_handles[0]
        sel_prim_obj_handle = prim_obj_handles[0]
        sel_asset_handle = prim_asset_handles[0]
        return

    # Build widgets
    file_obj_ddl, sel_file_obj_handle = set_handle_ddl_widget(
        file_obj_handles,
        "File-based Object",
        sel_file_obj_handle,
        on_file_obj_ddl_change,
    )
    prim_obj_ddl, sel_prim_obj_handle = set_handle_ddl_widget(
        prim_obj_handles,
        "Primitive-based Object",
        sel_prim_obj_handle,
        on_prim_obj_ddl_change,
    )
    prim_asset_ddl, sel_asset_handle = set_handle_ddl_widget(
        prim_asset_handles, "Primitive Asset", sel_asset_handle, on_prim_ddl_change
    )
    # Display DDLs
    ipydisplay(file_obj_ddl)
    ipydisplay(prim_obj_ddl)
    ipydisplay(prim_asset_ddl)


# %% [markdown]
# # Advanced Features
# This section contains advanced examples/demos of Habitat-sim interactivity.

# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }
sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb"
sim_settings["sensor_pitch"] = 0

make_simulator_from_settings(sim_settings)

# %%
# @title Select target object from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %% [markdown]
# ## Advanced Topic : Motion Tracking Camera
#
# While agents in Habitat sim usually act to modify the environment around them, playing the role of active and often primary participants in simulation, there are cases where we may want agents to play the role of the passive observer. In this case we may want an agent to simply "hold the camera" and record the results of simulation.

# %%
# @markdown This example demonstrates updating the agent state to follow the motion of an object during simulation.

rigid_obj_mgr.remove_all_objects()
visual_sensor = sim._sensors["color_sensor_1st_person"]
initial_sensor_position = np.array(visual_sensor._spec.position)
initial_sensor_orientation = np.array(visual_sensor._spec.orientation)
# set the color sensor transform to be the agent transform
visual_sensor._spec.position = np.array([0, 0, 0])
visual_sensor._spec.orientation = np.array([0, 0, 0])
visual_sensor._sensor_object.set_transformation_from_spec()

# boost the agent off the floor
sim.get_agent(0).scene_node.translation += np.array([0, 1.5, 0])
observations = []

# @markdown ---
# @markdown ### Set example parameters:
seed = 23  # @param {type:"integer"}
random.seed(seed)
sim.seed(seed)
np.random.seed(seed)

# add an object and position the agent
sel_file_obj = rigid_obj_mgr.add_object_by_template_handle(sel_file_obj_handle)
rand_position = np.random.uniform(
    np.array([-0.4, -0.3, -1.0]), np.array([0.4, 0.3, -0.5])
)
set_object_state_from_agent(sim, sel_file_obj, rand_position, ut.random_quaternion())

# simulate with updated camera at each frame
start_time = sim.get_world_time()
while sim.get_world_time() - start_time < 2.0:
    sim.step_physics(1.0 / 60.0)
    # set agent state to look at object
    camera_position = sim.get_agent(0).scene_node.translation
    camera_look_at = sel_file_obj.translation
    sim.get_agent(0).scene_node.rotation = mn.Quaternion.from_matrix(
        mn.Matrix4.look_at(
            camera_position, camera_look_at, np.array([0, 1.0, 0])  # up
        ).rotation()
    )
    observations.append(sim.get_sensor_observations())

# video rendering with embedded 1st person view
video_prefix = "motion tracking"
if make_video:
    vut.make_video(
        observations,
        "color_sensor_1st_person",
        "color",
        output_path + video_prefix,
        open_vid=show_video,
    )

# reset the sensor state for other examples
visual_sensor._spec.position = initial_sensor_position
visual_sensor._spec.orientation = initial_sensor_orientation
visual_sensor._sensor_object.set_transformation_from_spec()
# put the agent back
sim.reset()
rigid_obj_mgr.remove_all_objects()


# %% [markdown]
# ## Advanced Topic : 3D to 2D Key-point Projection
#
# The Habitat-sim visual-sensor API makes it easy to project 3D points into 2D for use cases such as generating ground-truth for image space key-points.


# %%
# @markdown ###Display 2D Projection of Object COMs
# fmt: off
# @markdown First define the projection function using the current state of a chosen VisualSensor to set camera parameters and then projects the 3D point.
# fmt: on
# project a 3D point into 2D image space for a particular sensor
def get_2d_point(sim, sensor_name, point_3d):
    # get the scene render camera and sensor object
    render_camera = sim._sensors[sensor_name]._sensor_object.render_camera

    # use the camera and projection matrices to transform the point onto the near plane
    projected_point_3d = render_camera.projection_matrix.transform_point(
        render_camera.camera_matrix.transform_point(point_3d)
    )
    # convert the 3D near plane point to integer pixel space
    point_2d = mn.Vector2(projected_point_3d[0], -projected_point_3d[1])
    point_2d = point_2d / render_camera.projection_size()[0]
    point_2d += mn.Vector2(0.5)
    point_2d *= render_camera.viewport
    return mn.Vector2i(point_2d)


# fmt: off
# @markdown Use this function to compute the projected object center of mass (COM) 2D projection and display on the image.
# fmt: on
# @markdown ---
# @markdown ### Set example parameters:
seed = 27  # @param {type:"integer"}
random.seed(seed)
sim.seed(seed)
np.random.seed(seed)

rigid_obj_mgr.remove_all_objects()

# add an object and plot the COM on the image
sel_file_obj = rigid_obj_mgr.add_object_by_template_handle(sel_file_obj_handle)
rand_position = np.random.uniform(
    np.array([-0.4, 1.2, -1.0]), np.array([0.4, 1.8, -0.5])
)
set_object_state_from_agent(sim, sel_file_obj, rand_position, ut.random_quaternion())

obs = sim.get_sensor_observations()

com_2d = get_2d_point(
    sim, sensor_name="color_sensor_1st_person", point_3d=sel_file_obj.translation
)
if display:
    display_sample(obs["color_sensor_1st_person"], key_points=[com_2d])
rigid_obj_mgr.remove_all_objects()

# %% [markdown]
# ## Advanced Topic: Configurable Semantic IDs
#
# Semantic information can be loaded into the Simulator for scenes directly. This can be provided to models and visualized with the SEMANTIC sensor.
#
# What if you want to add new objects to the scene with semantic ids or modify the semantic ids at runtime?
#
# Habitat-sim provides three methods for modifying semantic ids which are not provided via vertex colors:
#
# 1. Configure the semantic id before object instancing via the object template (programmatically or in the json)
# 2. Set the semantic_id of all SceneNodes used to render an object with **Simulator.set_object_semantic_id**.
# 3. Set the **SceneNode.semantic_id** property directly.
#
#
#
#

# %%
# @markdown ###Configuring Object Semantic IDs:

sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb"
sim_settings["sensor_pitch"] = 0
sim_settings["semantic_sensor_1st_person"] = True

make_simulator_from_settings(sim_settings)

# fmt: off
# @markdown In this example, we load a box asset with each face as a separate component with its own SceneNode. We demonstrate the result of modiyfing the associated semantic ids via object templates, the Simulator API, and the SceneNode property.
# fmt: on

rigid_obj_mgr.remove_all_objects()
observations = []

# @markdown Set the initial object orientation via local Euler angle (degrees):
orientation_x = 45  # @param {type:"slider", min:-180, max:180, step:1}
orientation_y = 45  # @param {type:"slider", min:-180, max:180, step:1}
orientation_z = 45  # @param {type:"slider", min:-180, max:180, step:1}


# compose the rotations
rotation_x = mn.Quaternion.rotation(mn.Deg(orientation_x), mn.Vector3(1.0, 0, 0))
rotation_y = mn.Quaternion.rotation(mn.Deg(orientation_y), mn.Vector3(0, 1.0, 0))
rotation_z = mn.Quaternion.rotation(mn.Deg(orientation_z), mn.Vector3(0, 0, 1.0))
object_orientation = rotation_z * rotation_y * rotation_x
print(object_orientation)

# fmt: off
# @markdown We can configure the semantic id in the object template. This id will be applied to the object (instead of the default 0) upon instantiation:
# fmt: on

# add a box with default semanticId configured in the template
# Note: each face of this box asset is a separate component
box_template = habitat_sim.attributes.ObjectAttributes()
box_template.render_asset_handle = str(
    os.path.join(data_path, "test_assets/objects/transform_box.glb")
)
box_template.scale = np.array([0.2, 0.2, 0.2])
# set the default semantic id for this object template
box_template.semantic_id = 10  # @param{type:"integer"}
box_template_id = obj_attr_mgr.register_template(box_template, "box")

box_obj = rigid_obj_mgr.add_object_by_template_id(box_template_id)
set_object_state_from_agent(
    sim, box_obj, mn.Vector3(0.0, 1.5, -0.75), orientation=object_orientation
)
observations.append(sim.get_sensor_observations())
# fmt: off
# @markdown We can set the semantic id for all components of the object via the Simulator API at any time:
# fmt: on
# override the configured id with a new id
box_semantic_id_override = 20  # @param{type:"integer"}
box_obj.semantic_id = box_semantic_id_override
observations.append(sim.get_sensor_observations())

# @markdown We can also set the semantic id for any single SceneNode directly:
# set semantic id for specific SceneNode components of the box object
box_visual_nodes = box_obj.visual_scene_nodes
box_visual_nodes[6].semantic_id = 3  # @param{type:"integer"}
box_visual_nodes[7].semantic_id = 4  # @param{type:"integer"}
observations.append(sim.get_sensor_observations())

# display the resulting observations:
if display:
    for obs in observations:
        display_sample(
            obs["color_sensor_1st_person"],
            semantic_obs=obs["semantic_sensor_1st_person"],
        )
rigid_obj_mgr.remove_all_objects()

# %% [markdown]
# ## Advanced Topic : Object and Primitive Asset Customization
#
# ![bananas.png](data:image/png;base64,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)
#
#

# %% [markdown]
# *   Objects are defined by their attributes.
# *   These attributes are collected in editable Attributes Templates that provide a recipe for object construction.
# *   These templates are managed by an Attributes Manager, which is accessible from the simulator and is used to create, retrieve and delete templates.
# *   The physics world, scenes, objects and primitive-based assets are all described by Attributes Templates tailored to their construction and managed by their own Attributes Managers.
# *   This section of the tutorial focuses on Object and Primitive Asset Attributes.
#

# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }
# @markdown (load the apartment_1 scene for object and primitive asset customization in an open space)
sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/habitat-test-scenes/apartment_1.glb"
sim_settings["sensor_pitch"] = 0

make_simulator_from_settings(sim_settings)

# Object Attributes Manager
obj_attr_mgr = sim.get_object_template_manager()


# %%
# @title Select target object from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %% [markdown]
# ### File-based Object Template modification.
#

# %%
# @title 1 Simple File-based Object Template modification. { display-mode: "form" }
# @markdown Running this will demonstrate how to create objects of varying size from a file-based template by iteratively modifying the template's scale value.  This will also demonstrate how to delete unwanted templates from the library.

# clear all objects and observations
rigid_obj_mgr.remove_all_objects()
observations = []
# save initial camera state
init_config = init_camera_track_config(sim)

# Use selected handle as object handle
obj_template_handle = sel_file_obj_handle
# Get File-based template for object using its handle - retrieves
# a copy of this object's attributes template.
obj_template = obj_attr_mgr.get_template_by_handle(obj_template_handle)

# Add object instantiated by desired template using template handle.
file_obj = rigid_obj_mgr.add_object_by_template_handle(obj_template_handle)

# Set desired offset from agent location to place object
offset = np.array([-1.2, 0.1, -1.5])
# Move object to be in front of the agent
set_object_state_from_agent(sim, file_obj, offset=offset)

# Templates have editable fields that will directly affect the instanced
# objects built from them.  Here we iteratively modify and re-register the
# template, instancing a new object each time.
# Bigger Bananas!
objs = [file_obj]
for i in range(5):
    # Increase the template scale value (object size)
    obj_template.scale *= 1.5
    # Make a new handle for the modified template, so we don't overwrite
    new_obj_template_handle = obj_template_handle + "_new_" + str(i)
    # Register modified template with new handle, returns template ID
    new_tmplt_id = obj_attr_mgr.register_template(obj_template, new_obj_template_handle)
    # Object creation can occur using template ID or handle
    if i % 2 == 0:
        # Add another object instantiated by modified template using handle
        new_obj = rigid_obj_mgr.add_object_by_template_id(new_tmplt_id)
    else:
        # Add another object instantiated by modified template using handle
        new_obj = rigid_obj_mgr.add_object_by_template_handle(new_obj_template_handle)
    # Move object to the right of previous object
    offset[0] += 0.4
    objs.append(new_obj)
    set_object_state_from_agent(sim, new_obj, offset=offset)

# Clean-up - remove modified templates from template library
# Get all modified template handles through keyword search
mod_template_handles = obj_attr_mgr.get_template_handles("_new_")
# Show number of modified templates
print(
    "Before delete, there are {} modified templates.".format(len(mod_template_handles))
)
# Display modified template handles
print(*mod_template_handles, sep="\n")
# Remove modified templates
for handle in mod_template_handles:
    obj_attr_mgr.remove_template_by_handle(handle)
# Verify removal - get template handles through keyword search
mod_template_handles = obj_attr_mgr.get_template_handles("_new_")
# Show number of modified templates now
print(
    "After deleting added templates, there are {} modified templates.".format(
        len(mod_template_handles)
    )
)
# Display modified template handles after delete
print(*mod_template_handles, sep="\n")


example_type = "Adding edited objects"
# Run camera-tracking simulation displaying modified objects
observations = camera_track_simulate(sim, objs, dt=3.0)

if make_video:
    vut.make_video(
        observations,
        "color_sensor_1st_person",
        "color",
        output_path + example_type,
        open_vid=show_video,
    )

# restore camera tracking position for future cells
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()


# %%
# @title 2 Object Template Modification in Detail. { display-mode: "form" }
# @markdown The following example lists all the fields available in an object template that can be modified, allows for them to be edited and enables creating an object with the newly modified template.
# @markdown Two objects will be created in this cell, one to the left with the original template and one to the right with the edited configuration.

# clear all objects and observations
rigid_obj_mgr.remove_all_objects()
observations = []
# save initial camera state for tracking
init_config = init_camera_track_config(sim)

# @markdown ###Editable fields in Object Templates :
# Get a new template
new_template = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)

# These are all the available attributes and their values for this object
show_template_properties(new_template)

# @markdown The desired mass of the object
mass = 1.0  # @param {type:"slider", min:0.1, max:50, step:0.1}
new_template.mass = mass

# @markdown The x,y,z components for the scale of the object.
scale_X = 2  # @param {type:"slider", min:0.1, max:10, step:0.1}
scale_Y = 8  # @param {type:"slider", min:0.1, max:10, step:0.1}
scale_Z = 2  # @param {type:"slider", min:0.1, max:10, step:0.1}
new_template.scale = mn.Vector3(scale_X, scale_Y, scale_Z)

# @markdown The x,y,z components for the desired location of the center of mass.
com_X = 0  # @param {type:"slider", min:-1.0, max:1, step:0.1}
com_Y = 0  # @param {type:"slider", min:-1.0, max:1, step:0.1}
com_Z = 0  # @param {type:"slider", min:-1.0, max:1, step:0.1}
new_template.com = mn.Vector3(com_X, com_Y, com_Z)

# @markdown If true, simulator sets COM as center of bounding box, and ignores any specified COM.
compute_COM_from_shape = False  # @param {type:"boolean"}
new_template.compute_COM_from_shape = compute_COM_from_shape

# @markdown Sets the collision margin
margin = 0.4  # @param {type:"slider", min:0.0, max:10, step:0.1}
new_template.margin = margin

# @markdown Friction for object contact
friction_coefficient = 0.5  # @param {type:"slider", min:0.0, max:1.0, step:0.1}
new_template.friction_coefficient = friction_coefficient

# @markdown Fraction of original relative velocity retained by an object after contact.  1 is perfectly elastic contact.
restitution_coefficient = 0.3  # @param {type:"slider", min:0.0, max:1.0, step:0.1}
new_template.restitution_coefficient = restitution_coefficient

# @markdown Whether the object should be lit via Phong shading.
force_flat_shading = True  # @param {type:"boolean"}
new_template.force_flat_shading = force_flat_shading

# @markdown The x,y,z components of the intertia matrix diagonal

inertia_X = 1.0  # @param {type:"slider", min:0.1, max:10, step:0.1}
inertia_Y = 1  # @param {type:"slider", min:0.1, max:10, step:0.1}
inertia_Z = 1.0  # @param {type:"slider", min:0.1, max:10, step:0.1}
new_template.inertia = mn.Vector3(inertia_X, inertia_Y, inertia_Z)

# @markdown Rate of linear momentum dissipation
linear_damping = 0.1  # @param {type:"slider", min:0.0, max:5.0, step:0.1}
new_template.linear_damping = linear_damping

# @markdown Rate of angular momentum dissipation
angular_damping = 0.2  # @param {type:"slider", min:0.0, max:5.0, step:0.1}

new_template.angular_damping = angular_damping

# @markdown Use bounding box for collision instead of collision mesh.
bounding_box_collisions = False  # @param {type:"boolean"}
new_template.bounding_box_collisions = bounding_box_collisions

# @markdown Whether compound collision meshes should be merged into a single convex hull.
join_collision_meshes = False  # @param {type:"boolean"}
new_template.join_collision_meshes = join_collision_meshes

# Construct a new handle to save this template with
new_template_handle = sel_file_obj_handle + "_new"


# register new template and get its new id
new_template_id = obj_attr_mgr.register_template(new_template, new_template_handle)


# Add object instantiated by original template using template handle
original_template = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)
orig_obj = rigid_obj_mgr.add_object_by_template_handle(original_template.handle)

# Set desired offset from agent location to place object
offset = np.array([-0.5, 0.3, -1.5])
# Move object to be in front of the agent
set_object_state_from_agent(sim, orig_obj, offset=offset)

# Add new object instantiated by desired template using template handle
new_obj = rigid_obj_mgr.add_object_by_template_id(new_template_id)

# Set desired offset from agent location to place object
offset[0] += 1.0
# Move object to be in front of the agent
set_object_state_from_agent(sim, new_obj, offset=offset)

example_type = "Adding customized objects"
# Run camera-tracking simulation displaying modified objects
observations = camera_track_simulate(sim, [orig_obj, new_obj], dt=2.5)

if make_video:
    vut.make_video(
        observations,
        "color_sensor_1st_person",
        "color",
        output_path + example_type,
        open_vid=show_video,
    )

# restore camera tracking position
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()


# %% [markdown]
# ### Primitive Asset-based Templates For Object Customization

# %%
# @title Display available file-based and primitive-based object and asset attributes : { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %%
# @title ###1 Simple Primitive-based Object instantiation. { display-mode: "form" }
# @markdown Habitat-Sim has 6 built-in primitives available (Capsule, Cone, Cube, Cylinder, Icosphere and UVSphere), which are available as both solid and wireframe meshes.  Default objects are synthesized from these primitives automatically and are always available. Primitives are desirable due to being very fast to simulate.

# @markdown This example shows the primitives that are available.  One of each type is instanced with default values and simulated.

# clear all objects and observations
rigid_obj_mgr.remove_all_objects()
observations = []
# save initial camera state for tracking
init_config = init_camera_track_config(sim)
# Get Primitive-based solid object template handles
prim_solid_obj_handles = obj_attr_mgr.get_synth_template_handles("solid")
# Get Primitive-based wireframe object template handles
prim_wf_obj_handles = obj_attr_mgr.get_synth_template_handles("wireframe")

# Set desired offset from agent location to place object
offset_solid = np.array([-1.1, 0.6, -1.8])
offset_wf = np.array([-1.1, 0.6, -1.0])
objs_to_sim = []
for i in range(6):
    # Create object from template handle
    obj_solid = rigid_obj_mgr.add_object_by_template_handle(prim_solid_obj_handles[i])
    obj_wf = rigid_obj_mgr.add_object_by_template_handle(prim_wf_obj_handles[i])
    objs_to_sim.append(obj_solid)
    objs_to_sim.append(obj_wf)

    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_solid, offset=offset_solid)
    set_object_state_from_agent(sim, obj_wf, offset=offset_wf)

    # Move offset for next object
    offset_solid[0] += 0.4
    offset_wf[0] += 0.4


example_type = "Adding primitive-based objects"
# Run camera-tracking simulation displaying primitive-based objects
observations = camera_track_simulate(sim, objs_to_sim, dt=2.0)
if make_video:
    vut.make_video(
        observations,
        "color_sensor_1st_person",
        "color",
        output_path + example_type,
        open_vid=show_video,
    )

# restore camera tracking position
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()

# %%
# @title ###2 Primitive Asset Template Modifications in Detail
# @markdown Many of the geometric properties of the available primitive meshes can be controlled by modifying the Primitive Asset Attributes templates that describe them.  Any objects instantiated with these modified template will then exhibit these properties.
# @markdown Note 1 : Solid and Wireframe Cubes and Wireframe Icospheres do not have any editable geometric properties.
# @markdown Note 2 : Since many of the modifiable parameters controlling primitives are restricted in what values they can be, each Primitive Asset Template has a read-only boolean property, "is_valid_template", which describes whether or not the template is in a valid state.  If it is invalid, it will not be able to be used to create primitives.
# @markdown Note 3 : Primitive Asset Templates name themselves based on their settings, so an edited template does not have to be actively renamed by the user.
# @markdown Note 4 : There often are different modifiable properties available for solid and wireframe versions of the same primitive.

# @markdown ###Primitive Asset Template Properties
# @markdown The flollowing examples will illustrate all the modifiable properties for each of the different types of primitives, and allow for their synthesis.


# Primitive Asset Attributes Manager, provides access to AssetAttributesTemplates
prim_attr_mgr = sim.get_asset_template_manager()


# This will register a primitive template if valid, and add it to a passed
# dictionary of handles; If not valid it will give a message
def register_prim_template_if_valid(
    make_modified, template, dict_of_handles, handle_key
):
    if make_modified:
        if template.is_valid_template:
            prim_attr_mgr.register_template(template)
            dict_of_handles[handle_key] = template.handle
            print(
                "Primitive Template named {} is registered for {}.".format(
                    template.handle, handle_key
                )
            )
        else:
            print(
                "Primitive Template configuration is invalid for {}, so unable to register.".format(
                    handle_key
                )
            )
    else:
        dict_of_handles[handle_key] = prim_attr_mgr.get_template_handles(handle_key)[0]
        print("Default Primitive Template used at key {}.".format(handle_key))


# Build a dictionary of templates to use to construct objects
# Configure dictionaries to hold handles of attributes to use to build objects
# Solid and Wireframe cube primitives lack customizable attributes, as does wireframe icosphere
solid_handles_to_use = {"cubeSolid": prim_attr_mgr.get_template_handles("cubeSolid")[0]}
wireframe_handles_to_use = {
    "cubeWireframe": prim_attr_mgr.get_template_handles("cubeWireframe")[0]
}
wireframe_handles_to_use["icosphereWireframe"] = prim_attr_mgr.get_template_handles(
    "icosphereWireframe"
)[0]


# %% [markdown]
# #### 2.1 Capsule Primitive : Cylinder with hemispherical caps of radius 1.0, oriented along y axis, centered at origin.
#

# %%
# @title ###2.1.1 Solid Capsule :{ display-mode: "form" }
# Acquire default template
capsule_solid_template = prim_attr_mgr.get_default_capsule_template(False)


def edit_solid_capsule(edit_template):
    # @markdown Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # @markdown Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # @markdown  Number of (face) rings for each hemisphere. Must be larger or equal to 1
    hemisphere_rings = 6  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.hemisphere_rings = hemisphere_rings
    # @markdown Number of (face) rings for cylinder. Must be larger or equal to 1.
    cylinder_rings = 1  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.cylinder_rings = cylinder_rings
    # @markdown  Number of (face) segments. Must be larger or equal to 3.
    num_segments = 16  # @param {type:"slider", min:3, max:30, step:1}
    edit_template.num_segments = num_segments
    # @markdown Half the length of cylinder part.
    half_length = 0.95  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # @markdown Do you want to make a solid capsule using your above modifications?
    make_modified_solid_capsule = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_solid_capsule,
        edit_template,
        solid_handles_to_use,
        "capsule3DSolid",
    )


edit_solid_capsule(capsule_solid_template)


# %%
# @title ####2.1.2 Wireframe Capsule :{ display-mode: "form" }
# Acquire default template
capsule_wireframe_template = prim_attr_mgr.get_default_capsule_template(True)


def edit_wf_capsule(edit_template):
    # @markdown Number of (line) rings for each hemisphere. Must be larger or equal to 1
    hemisphere_rings = 7  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.hemisphere_rings = hemisphere_rings

    # @markdown Number of (line) rings for cylinder. Must be larger or equal to 1.
    cylinder_rings = 10  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.cylinder_rings = cylinder_rings

    # @markdown Number of line segments. Must be larger or equal to 4 and multiple of 4
    num_segments = 16  # @param {type:"slider", min:4, max:40, step:4}
    edit_template.num_segments = num_segments

    # @markdown Half the length of cylinder part.
    half_length = 0.85  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length

    # @markdown Do you want to make a wireframe capsule using your above modifications?
    make_modified_wireframe_capsule = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_wireframe_capsule,
        edit_template,
        wireframe_handles_to_use,
        "capsule3DWireframe",
    )


edit_wf_capsule(capsule_wireframe_template)

# %% [markdown]
# ####2.2 Cone Primitive : Cone of radius 1.0f along the Y axis, centered at origin.
#

# %%
# @title ####2.2.1 Solid Cone :{ display-mode: "form" }
# Acquire default template
cone_solid_template = prim_attr_mgr.get_default_cone_template(False)


def edit_solid_cone(edit_template):
    # @markdown Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # @markdown Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # @markdown Number of (face) rings. Must be larger or equal to 1.
    num_rings = 6  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.num_rings = num_rings
    # @markdown Number of (face) segments. Must be larger or equal to 3.
    num_segments = 20  # @param {type:"slider", min:3, max:40, step:1}
    edit_template.num_segments = num_segments
    # @markdown Half the cone length
    half_length = 1.25  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # @markdown Whether to cap the base of the cone
    use_cap_end = True  # @param {type:"boolean"}
    edit_template.use_cap_end = use_cap_end
    # @markdown Do you want to make a solid cone using your above modifications?
    make_modified_solid_cone = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_solid_cone, edit_template, solid_handles_to_use, "coneSolid"
    )


edit_solid_cone(cone_solid_template)


# %%
# @title ####2.2.2 Wireframe Cone :{ display-mode: "form" }
# Acquire default template
cone_wireframe_template = prim_attr_mgr.get_default_cone_template(True)


def edit_wireframe_cone(edit_template):
    # @markdown Number of (line) segments. Must be larger or equal to 4 and multiple of 4.
    num_segments = 32  # @param {type:"slider", min:4, max:40, step:4}
    edit_template.num_segments = num_segments
    # @markdown Half the cone length
    half_length = 1.25  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # @markdown Do you want to make a wireframe cone using your above modifications?
    make_modified_wireframe_cone = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_wireframe_cone,
        edit_template,
        wireframe_handles_to_use,
        "coneWireframe",
    )


edit_wireframe_cone(cone_wireframe_template)

# %% [markdown]
# ####2.3 Cylinder Primitive : Cylinder of radius 1.0f along the Y axis, centered at origin.

# %%
# @title ####2.3.1 Solid Cylinder : { display-mode: "form" }
# Acquire default template
cylinder_solid_template = prim_attr_mgr.get_default_cylinder_template(False)


def edit_solid_cylinder(edit_template):
    # @markdown Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # @markdown Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # @markdown Number of (face) rings. Must be larger or equal to 1.
    num_rings = 4  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.num_rings = num_rings
    # @markdown Number of (face) segments. Must be larger or equal to 3.
    num_segments = 14  # @param {type:"slider", min:3, max:30, step:1}
    edit_template.num_segments = num_segments
    # @markdown Half the cylinder length
    half_length = 1  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # @markdown Whether to cap each end of the cylinder
    use_cap_ends = True  # @param {type:"boolean"}
    edit_template.use_cap_ends = use_cap_ends
    # @markdown Do you want to make a solid cylinder using your above modifications?
    make_modified_solid_cylinder = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_solid_cylinder,
        edit_template,
        solid_handles_to_use,
        "cylinderSolid",
    )


edit_solid_cylinder(cylinder_solid_template)


# %%
# @title ####2.3.2 Wireframe Cylinder : { display-mode: "form" }
# Acquire default template
cylinder_wireframe_template = prim_attr_mgr.get_default_cylinder_template(True)


def edit_wireframe_cylinder(edit_template):
    # @markdown Number of (face) rings. Must be larger or equal to 1.
    num_rings = 1  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.num_rings = num_rings
    # @markdown Number of (line) segments. Must be larger or equal to 4 and multiple of 4.
    num_segments = 28  # @param {type:"slider", min:4, max:64, step:4}
    edit_template.num_segments = num_segments
    # @markdown Half the cylinder length
    half_length = 0.7  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # @markdown Do you want to make a wireframe cylinder using your above modifications?
    make_modified_wireframe_cylinder = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_wireframe_cylinder,
        edit_template,
        wireframe_handles_to_use,
        "cylinderWireframe",
    )


edit_wireframe_cylinder(cylinder_wireframe_template)

# %% [markdown]
# ####2.4 Icosphere Primitive : Icosahedron-based sphere of radius 1.0f, centered at the origin.
#
# Only solid icospheres have any editable geometric parameters.

# %%
# @title ####2.4.1 Solid Icosphere : { display-mode: "form" }
# Acquire default template
icosphere_solid_template = prim_attr_mgr.get_default_icosphere_template(False)


def edit_solid_icosphere(edit_template):
    # @markdown Describes the depth of recursive subdivision for each icosphere triangle.
    subdivisions = 3  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.subdivisions = subdivisions
    # @markdown Do you want to make a solid icosphere using your above modifications?
    make_modified_solid_icosphere = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_solid_icosphere,
        edit_template,
        solid_handles_to_use,
        "icosphereSolid",
    )


edit_solid_icosphere(icosphere_solid_template)

# %% [markdown]
# ####2.5 UVSphere Primitive : Sphere of radius 1.0f, centered at the origin.

# %%
# @title ####2.5.1 Solid UVSphere : { display-mode: "form" }
# Acquire default template
UVSphere_solid_template = prim_attr_mgr.get_default_UVsphere_template(False)


def edit_solid_UVSphere(edit_template):
    # @markdown Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # @markdown Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # @markdown Number of (face) rings. Must be larger or equal to 2.
    num_rings = 13  # @param {type:"slider", min:2, max:30, step:1}
    edit_template.num_rings = num_rings
    # @markdown Number of (face) segments. Must be larger or equal to 3.
    num_segments = 8  # @param {type:"slider", min:3, max:30, step:1}
    edit_template.num_segments = num_segments

    # @markdown Do you want to make a solid UVSphere using your above modifications?
    make_modified_solid_UVSphere = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_solid_UVSphere,
        edit_template,
        solid_handles_to_use,
        "uvSphereSolid",
    )


edit_solid_UVSphere(UVSphere_solid_template)


# %%
# @title ####2.5.2 Wireframe UVSphere : { display-mode: "form" }
# Acquire default template
UVSphere_wireframe_template = prim_attr_mgr.get_default_UVsphere_template(True)


def edit_wireframe_UVSphere(edit_template):
    # @markdown Number of (line) rings. Must be larger or equal to 2 and multiple of 2.
    num_rings = 16  # @param {type:"slider", min:2, max:64, step:2}
    edit_template.num_rings = num_rings
    # @markdown Number of (line) segments. Must be larger or equal to 4 and multiple of 4.
    num_segments = 40  # @param {type:"slider", min:4, max:64, step:4}
    edit_template.num_segments = num_segments
    # @markdown Do you want to make a UVSphere cylinder using your above modifications?
    make_modified_wireframe_UVSphere = True  # @param {type:"boolean"}
    # if make is set to true, save modified template.
    register_prim_template_if_valid(
        make_modified_wireframe_UVSphere,
        edit_template,
        wireframe_handles_to_use,
        "uvSphereWireframe",
    )


edit_wireframe_UVSphere(UVSphere_wireframe_template)

# %% [markdown]
# ####2.6 Instancing Objects with Modified Primitive-Asset Attributes.

# %%
# @title ####Using the modifications set in the previous cells, instantiate examples of all available solid and wireframe primitives.{ display-mode: "form" }
# clear all objects and observations
rigid_obj_mgr.remove_all_objects()
observations = []
# save initial camera state for tracking
init_config = init_camera_track_config(sim)
# Previous cells configured solid_handles_to_use and wireframe_handles_to_use

# Set desired offset from agent location to place object
offset_solid = np.array([-1.1, 0.6, -1.8])
objs_to_sim = []
# Create primitive-attributes based object templates for solid and wireframe objects
for solidHandle in solid_handles_to_use.values():
    # Create object template with passed handle
    obj_template = obj_attr_mgr.create_template(solidHandle)
    # Create object from object template handle
    print("Solid Object being made using handle :{}".format(solidHandle))
    obj_solid = rigid_obj_mgr.add_object_by_template_handle(solidHandle)
    objs_to_sim.append(obj_solid)
    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_solid, offset=offset_solid)
    # Move offset for next object
    offset_solid[0] += 0.4

offset_wf = np.array([-1.1, 0.6, -1.0])

for wireframeHandle in wireframe_handles_to_use.values():
    # Create object template with passed handle
    obj_template = obj_attr_mgr.create_template(wireframeHandle)
    # Create object from object template handle
    print("Wireframe Object being made using handle :{}".format(wireframeHandle))
    obj_wf = rigid_obj_mgr.add_object_by_template_handle(wireframeHandle)
    objs_to_sim.append(obj_wf)
    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_wf, offset=offset_wf)
    # Move offset for next object
    offset_wf[0] += 0.4

example_type = "Adding customized primitive-based objects"
observations = camera_track_simulate(sim, objs_to_sim, dt=2.0)
if make_video:
    vut.make_video(
        observations,
        "color_sensor_1st_person",
        "color",
        output_path + example_type,
        open_vid=show_video,
    )

# restore camera tracking position
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()

# %% [markdown]
# ## Additional Advanced Topics:
#
# In addition to the topics we covered here, visit the Habitat-sim [python docs](https://aihabitat.org/docs/habitat-sim/) to explore more topics.
#
# ###Custom Lighting Setups
#
# Habitat-sim allows for both Phong and Flat shading options and configurable lighting groups for objects and scenes to be customized. See our [Working with Lights tutorial page](https://aihabitat.org/docs/habitat-sim/lighting-setups.html) to learn more.
#
# ###Interactive Rigid Objects
#
# For more details on the rigid object API, see our [Interactive Rigid Objects tutorial](https://aihabitat.org/docs/habitat-sim/rigid-object-tutorial.html).
#
