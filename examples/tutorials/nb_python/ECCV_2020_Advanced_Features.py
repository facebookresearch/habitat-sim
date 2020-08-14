# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     collapsed_sections: []
#     name: ECCV_2020_Advanced_Features.ipynb
#     private_outputs: true
#     provenance: []
#     toc_visible: true
#   jupytext:
#     cell_metadata_filter: -all
#     formats: nb_python//py:percent,colabs//ipynb
#     notebook_metadata_filter: all
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.5.2
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %% [markdown]
# #Habitat-sim Advanced Features
#
# This tutorial presents a number of advanced feature examples for using Habitat-sim, including:
# - Object configuration via template libraries
# - Tracking object motion with a camera
# - Configuring semantic ids for objects
# - Generating a cluttered scene from the NavMesh

# %%
# @title Installation { display-mode: "form" }
# @markdown (double click to show code).

# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/master/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s
# !wget -c http://dl.fbaipublicfiles.com/habitat/mp3d_example.zip && unzip -o mp3d_example.zip -d /content/habitat-sim/data/scene_datasets/mp3d/

# %%
# @title Path Setup and Imports { display-mode: "form" }
# @markdown (double click to show code).

# %cd /content/habitat-sim
## [setup]
import math
import os
import random
import sys
import time

import cv2
import git
import magnum as mn
import numpy as np

try:
    import ipywidgets as widgets
    from IPython.display import display as ipydisplay
    # For using jupyter/ipywidget IO components
    from ipywidgets import fixed, interact, interact_manual, interactive

    HAS_WIDGETS = True
except ImportError:
    HAS_WIDGETS = False
# %matplotlib inline
from matplotlib import pyplot as plt
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as ut
from habitat_sim.utils import viz_utils as vut

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")
# fmt: off
output_directory = "examples/tutorials/advanced_features_output/"  # @param {type:"string"}
# fmt: on
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

# define some globals the first time we run.
if not "sim" in globals():
    global sim
    sim = None
    global obj_attr_mgr
    obj_attr_mgr = None
    global prim_attr_mgr
    obj_attr_mgr = None
    global scene_attr_mgr
    scene_attr_mgr = None


# %%
# @title Define Template Dictionary Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown This cell defines utility functions that expose Attribute template object properties.

# This method builds a dictionary of k-v pairs of attribute property names and
# values shared by all attribute template types.  The values are tuples with the
# first entry being the value and the second being whether the property is
# editable and the third being the type.
def build_dict_of_Default_attrs(template):
    res_dict = {}
    res_dict["handle"] = (template.handle, True, "string")
    # Read-only values
    res_dict["ID"] = (template.ID, False, "int")
    res_dict["template_class"] = (template.template_class, False, "string")
    res_dict["file_directory"] = (template.file_directory, False, "string")
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
    res_dict["requires_lighting"] = (
        phys_obj_template.requires_lighting,
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
def build_dict_of_Scene_attrs(scene_template):
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
    if "PhysicsObjectAttributes" in template_class:
        return build_dict_of_Object_attrs(template)
    if "PhysicsSceneAttributes" in template_class:
        return build_dict_of_Scene_attrs(template)
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
    for k, v in template_map.items():
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
    sim_cfg.scene.id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]

    # Note: all sensors must have the same resolution
    sensors = {
        "color_sensor_1st_person": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "orientation": [settings["sensor_pitch"], 0.0, 0.0],
        },
        "depth_sensor_1st_person": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "orientation": [settings["sensor_pitch"], 0.0, 0.0],
        },
        "semantic_sensor_1st_person": {
            "sensor_type": habitat_sim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "orientation": [settings["sensor_pitch"], 0.0, 0.0],
        },
        # configure the 3rd person cam specifically:
        "color_sensor_3rd_person": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"] + 0.2, 0.2],
            "orientation": np.array([-math.pi / 4, 0, 0]),
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            sensor_spec = habitat_sim.SensorSpec()
            sensor_spec.uuid = sensor_uuid
            sensor_spec.sensor_type = sensor_params["sensor_type"]
            sensor_spec.resolution = sensor_params["resolution"]
            sensor_spec.position = sensor_params["position"]
            sensor_spec.orientation = sensor_params["orientation"]

            sensor_specs.append(sensor_spec)

    # Here you can specify the amount of displacement in a forward action and the turn angle
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


def make_default_settings():
    settings = {
        "width": 720,  # Spatial resolution of the observations
        "height": 544,
        "scene": "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",  # Scene path
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
    global scene_attr_mgr
    if sim != None:
        sim.close()
    # initialize the simulator
    sim = habitat_sim.Simulator(cfg)
    # Managers of various Attributes templates
    obj_attr_mgr = sim.get_object_template_manager()
    prim_attr_mgr = sim.get_asset_template_manager()
    scene_attr_mgr = sim.get_scene_template_manager()
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

# @markdown - remove_all_objects
# @markdown - simulate
# @markdown - init_camera_track_config
# @markdown - restore_camera_track_config
# @markdown - camera_track_simulate
# @markdown - sample_object_state


def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


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
def camera_track_simulate(sim, obj_ids, dt=2.0, get_frames=True, agent_ID=0):
    start_time = sim.get_world_time()
    observations = []
    num_objs = len(obj_ids)
    if 0 == num_objs:
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
        camera_look_at = sim.get_translation(obj_ids[0])
        for i in range(1, num_objs):
            camera_look_at += sim.get_translation(obj_ids[i])
        camera_look_at /= len(obj_ids)
        agent.scene_node.rotation = mn.Quaternion.from_matrix(
            mn.Matrix4.look_at(camera_position, camera_look_at, up_vec).rotation()  # up
        )
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations


# Set an object transform relative to the agent state
def set_object_state_from_agent(
    sim,
    ob_id,
    offset=np.array([0, 2.0, -1.5]),
    orientation=mn.Quaternion(((0, 0, 0), 1)),
):
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    ob_translation = agent_transform.transform_point(offset)
    sim.set_translation(ob_translation, ob_id)
    sim.set_rotation(orientation, ob_id)


# %%
# @title Define Visualization Utility Functions { display-mode: "form" }
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
            for pix, point in enumerate(key_points):
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
    button = widgets.Button(description=desc, layout={"width": "max-content"},)
    return button


def make_sim_and_vid_button(prefix, dt=1.0):
    if not HAS_WIDGETS:
        return

    def on_sim_click(b):
        observations = simulate(sim, dt=dt)
        make_video_cv2(observations, prefix=prefix, open_vid=True, multi_obs=False)

    sim_and_vid_btn = set_button_launcher("Simulate and Make Video")
    sim_and_vid_btn.on_click(on_sim_click)
    ipydisplay(sim_and_vid_btn)


def make_clear_all_objects_button():
    if not HAS_WIDGETS:
        return

    def on_clear_click(b):
        remove_all_objects(sim)

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
        sel_prim_obj_handle = prim_asset_handles[0]
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


# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }
sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"
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

remove_all_objects(sim)
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
obj_id_1 = sim.add_object_by_handle(sel_file_obj_handle)
rand_position = np.random.uniform(
    np.array([-0.4, -0.3, -1.0]), np.array([0.4, 0.3, -0.5])
)
set_object_state_from_agent(sim, obj_id_1, rand_position, ut.random_quaternion())

# simulate with updated camera at each frame
start_time = sim.get_world_time()
while sim.get_world_time() - start_time < 2.0:
    sim.step_physics(1.0 / 60.0)
    # set agent state to look at object
    camera_position = sim.get_agent(0).scene_node.translation
    camera_look_at = sim.get_translation(obj_id_1)
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
        observations, "color_sensor_1st_person", "color", output_path + video_prefix
    )

# reset the sensor state for other examples
visual_sensor._spec.position = initial_sensor_position
visual_sensor._spec.orientation = initial_sensor_orientation
visual_sensor._sensor_object.set_transformation_from_spec()
# put the agent back
sim.reset()
remove_all_objects(sim)


# %% [markdown]
# ## Advanced Topic : 3D to 2D Key-point Projection
#
# The Habitat-sim visual-sensor API makes it easy to project 3D points into 2D for use cases such as generating ground-truth for image space key-points.

# %%
# @markdown ###Display 2D Projection of Object COMs
# @markdown First define the projection function using the current state of a chosen
# @markdown VisualSensor to set camera parameters and then projects the 3D point.
# project a 3D point into 2D image space for a particular sensor
def get_2d_point(sim, sensor_name, point_3d):
    # get the scene render camera and sensor object
    visual_sensor = sim._sensors[sensor_name]
    scene_graph = sim.get_active_scene_graph()
    scene_graph.set_default_render_camera_parameters(visual_sensor._sensor_object)
    render_camera = scene_graph.get_default_render_camera()

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


# @markdown Use this function to compute the projected object center of mass (COM)
# @markdown 2D projection and display on the image.

# @markdown ---
# @markdown ### Set example parameters:
seed = 27  # @param {type:"integer"}
random.seed(seed)
sim.seed(seed)
np.random.seed(seed)

remove_all_objects(sim)

# add an object and plot the COM on the image
obj_id_1 = sim.add_object_by_handle(sel_file_obj_handle)
rand_position = np.random.uniform(
    np.array([-0.4, 1.2, -1.0]), np.array([0.4, 1.8, -0.5])
)
set_object_state_from_agent(sim, obj_id_1, rand_position, ut.random_quaternion())

obs = sim.get_sensor_observations()

com_2d = get_2d_point(
    sim, sensor_name="color_sensor_1st_person", point_3d=sim.get_translation(obj_id_1)
)
if display:
    display_sample(obs["color_sensor_1st_person"], key_points=[com_2d])
remove_all_objects(sim)

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
sim_settings["scene"] = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"
sim_settings["sensor_pitch"] = 0
sim_settings["semantic_sensor_1st_person"] = True

make_simulator_from_settings(sim_settings)

# @markdown In this example, we load a box asset with each face as a separate
# @markdown component with its own SceneNode. We demonstrate the result of modiyfing
# @markdown the associated semantic ids via object templates, the Simulator API,
# @markdown and the SceneNode property.

remove_all_objects(sim)
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

# @markdown We can configure the semantic id in the object template. This id
# @markdown will be applied to the object (instead of the default 0) upon instantiation:

# add a box with default semanticId configured in the template
# Note: each face of this box asset is a separate component
box_template = habitat_sim.attributes.PhysicsObjectAttributes()
box_template.render_asset_handle = str(
    os.path.join(data_path, "test_assets/objects/transform_box.glb")
)
box_template.scale = np.array([0.2, 0.2, 0.2])
# set the default semantic id for this object template
box_template.semantic_id = 10  # @param{type:"integer"}
box_template_id = obj_attr_mgr.register_template(box_template, "box")

box_id = sim.add_object(box_template_id)
set_object_state_from_agent(
    sim, box_id, mn.Vector3(0.0, 1.5, -0.75), orientation=object_orientation
)
observations.append(sim.get_sensor_observations())

# @markdown We can set the semantic id for all components of the object via the Simulator
# @markdown API at any time:
# override the configured id with a new id
box_semantic_id_override = 20  # @param{type:"integer"}
sim.set_object_semantic_id(box_semantic_id_override, box_id)
observations.append(sim.get_sensor_observations())

# @markdown We can also set the semantic id for any single SceneNode directly:
# set semantic id for specific SceneNode components of the box object
box_visual_nodes = sim.get_object_visual_scene_nodes(box_id)
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
remove_all_objects(sim)

# %% [markdown]
# ## Advanced Topic : Asset and Object Customization
#
#
#
#

# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }
# @markdown (load the apartment_1 scene for primitive asset and object customization in an open space)
sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/habitat-test-scenes/apartment_1.glb"
sim_settings["sensor_pitch"] = 0

make_simulator_from_settings(sim_settings)

# %%
# @title Select target object from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %%
# @title 1 Simple File-based Object Template modification. { display-mode: "form" }
# @markdown Running this will demonstrate how to create objects of varying size from a file-based template by iteratively modifying the template's scale value.  This will also demonstrate how to delete unwanted templates from the library.

# clear all objects and observations
remove_all_objects(sim)
observations = []
# save initial camera state
init_config = init_camera_track_config(sim)
# Get File-based template for banana using its handle
obj_template_handle = sel_file_obj_handle

obj_template = obj_attr_mgr.get_template_by_handle(obj_template_handle)

# Add object instantiated by desired template using template handle
obj_id = sim.add_object_by_handle(obj_template_handle)

# Set desired offset from agent location to place object
offset = np.array([-1.2, 0.1, -1.5])
# Move object to be in front of the agent
set_object_state_from_agent(sim, obj_id, offset=offset)

# Templates have editable fields that will directly affect the instanced
# objects built from them.  Here we iteratively modify and re-register the
# template, instancing a new object each time.
# Bigger Bananas!
obj_ids = [obj_id]
for i in range(5):
    # Increase the template scale value (object size)
    obj_template.scale *= 1.5
    # Make a new handle for the modified template, so we don't overwrite
    new_obj_template_handle = obj_template_handle + "_new_" + str(i)
    # Register modified template with new handle, returns template ID
    new_tmplt_ID = obj_attr_mgr.register_template(obj_template, new_obj_template_handle)
    # Object creation can occur using template ID or handle
    if i % 2 == 0:
        # Add another object instantiated by modified template using handle
        new_obj = sim.add_object(new_tmplt_ID)
    else:
        # Add another object instantiated by modified template using handle
        new_obj = sim.add_object_by_handle(new_obj_template_handle)
    # Move object to the right of previous object
    offset[0] += 0.4
    obj_ids.append(new_obj)
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
# Display modified template handles
print(*mod_template_handles, sep="\n")

example_type = "Adding edited objects"

observations = camera_track_simulate(sim, obj_ids, dt=3.0)

if make_video:
    vut.make_video(
        observations, "color_sensor_1st_person", "color", output_path + example_type
    )

# restore camera tracking position
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()


# %%
# @title 2 Object Template Modification in Detail. { display-mode: "form" }
# @markdown The following example lists all the fields available in an object template that can be modified, allows for them to be edited and enables creating an object with the newly modified template.
# @markdown Two objects will be created in this cell, one to the left with the original template and one to the right with the edited configuration.

# clear all objects and observations
remove_all_objects(sim)
observations = []
# save initial camera state for tracking
init_config = init_camera_track_config(sim)

# @markdown ###Editable fields in Object Templates :
# Get a new template
new_template = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)

# @markdown The desired mass of the object
mass = 5.9  # @param {type:"slider", min:0.1, max:50, step:0.1}
new_template.mass = mass

# @markdown The x,y,z components for the scale of the object.
scale_X = 5  # @param {type:"slider", min:0.1, max:10, step:0.1}
scale_Y = 10  # @param {type:"slider", min:0.1, max:10, step:0.1}
scale_Z = 5  # @param {type:"slider", min:0.1, max:10, step:0.1}
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
requires_lighting = False  # @param {type:"boolean"}
new_template.requires_lighting = requires_lighting

# @markdown The x,y,z components of the intertia matrix diagonal

inertia_X = 1.0  # @param {type:"slider", min:0.1, max:10, step:0.1}
inertia_Y = 1.0  # @param {type:"slider", min:0.1, max:10, step:0.1}
inertia_Z = 1.0  # @param {type:"slider", min:0.1, max:10, step:0.1}
new_template.inertia = mn.Vector3(inertia_X, inertia_Y, inertia_Z)

# @markdown Rate of linear momentum dissipation
linear_damping = 0.2  # @param {type:"slider", min:0.0, max:5.0, step:0.1}
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
new_template_ID = obj_attr_mgr.register_template(new_template, new_template_handle)


# Add object instantiated by original template using template handle
original_template = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)
orig_obj_id = sim.add_object_by_handle(original_template.handle)

# Set desired offset from agent location to place object
offset = np.array([-0.5, 0.3, -1.5])
# Move object to be in front of the agent
set_object_state_from_agent(sim, orig_obj_id, offset=offset)

# Add new object instantiated by desired template using template handle
obj_id = sim.add_object(new_template_ID)

# Set desired offset from agent location to place object
offset[0] += 1.0
# Move object to be in front of the agent
set_object_state_from_agent(sim, obj_id, offset=offset)

example_type = "Adding customized objects"
observations = camera_track_simulate(sim, [orig_obj_id, obj_id], dt=2.5)

if make_video:
    vut.make_video(
        observations, "color_sensor_1st_person", "color", output_path + example_type
    )

# restore camera tracking position
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()


# %%
# @title 3 Simple Primitive-based Object instantiation. { display-mode: "form" }
# @markdown Habitat-Sim has 6 built-in primitives available (Capsule, Cone, Cube, Cylinder, Icosphere and UVSphere), which are available as both solid and wireframe meshes.  Default objects are synthesized from these primitives automatically and are always available. Primitives are desirable due to being very fast to simulate.

# @markdown This example shows the primitives that are available.  One of each type is instanced with default values and simulated.

# clear all objects and observations
remove_all_objects(sim)
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
    obj_solid_id = sim.add_object_by_handle(prim_solid_obj_handles[i])
    obj_wf_id = sim.add_object_by_handle(prim_wf_obj_handles[i])
    objs_to_sim.append(obj_solid_id)
    objs_to_sim.append(obj_wf_id)

    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_solid_id, offset=offset_solid)
    set_object_state_from_agent(sim, obj_wf_id, offset=offset_wf)

    # Move offset for next object
    offset_solid[0] += 0.4
    offset_wf[0] += 0.4


example_type = "Adding primitive-based objects"

observations = camera_track_simulate(sim, objs_to_sim, dt=2.0)
if make_video:
    vut.make_video(
        observations, "color_sensor_1st_person", "color", output_path + example_type
    )
# restore camera tracking position
restore_camera_track_config(sim, init_config)
make_clear_all_objects_button()


# %% [markdown]
# ### 4 Modifying Primitive Asset Templates to Customize Objects.

# %%
# @markdown Many of the geometric properties of the available primitive meshes can be controlled by modifying the Primitive Asset Attributes templates that describe them.  Any objects instantiated with these modified template will then exhibit these properties.
# @markdown Note 1 : Solid and Wireframe Cubes and Wireframe Icospheres do not have any editable geometric properties.
# @markdown Note 2 : Since many of the modifiable parameters controlling primitives are restricted in what values they can be, each Primitive Asset Template has a read-only boolean property, "is_valid_template", which describes whether or not the template is in a valid state.  If it is invalid, it will not be able to be used to create primitives.
# @markdown Note 3 : Primitive Asset Templates name themselves based on their settings, so an edited template does not have to be actively renamed by the user.
# @markdown Note 4 : There often are different modifiable properties available for solid and wireframe versions of the same primitive.

# @markdown ###Primitive Asset Template Properties
# @markdown The flollowing examples will illustrate all the modifiable properties for each of the different types of primitives, and allow for their synthesis.

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
solid_handles_to_use = {}
solid_handles_to_use["cubeSolid"] = prim_attr_mgr.get_template_handles("cubeSolid")[0]
wireframe_handles_to_use = {}
wireframe_handles_to_use["cubeWireframe"] = prim_attr_mgr.get_template_handles(
    "cubeWireframe"
)[0]
wireframe_handles_to_use["icosphereWireframe"] = prim_attr_mgr.get_template_handles(
    "icosphereWireframe"
)[0]


# %% [markdown]
# #### 4.1 Capsule : Cylinder with hemispherical caps of radius 1.0, oriented along y axis, centered at origin.
#

# %%
# @title ####4.1.1 Solid Capsule :{ display-mode: "form" }
# Acquire default template
capsule_solid_template = prim_attr_mgr.get_default_capsule_template(False)


def edit_solid_capsule(edit_template):
    # Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # Number of (face) rings for each hemisphere. Must be larger or equal to 1
    hemisphere_rings = 6  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.hemisphere_rings = hemisphere_rings
    # Number of (face) rings for cylinder. Must be larger or equal to 1.
    cylinder_rings = 1  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.cylinder_rings = cylinder_rings
    # Number of (face) segments. Must be larger or equal to 3.
    num_segments = 16  # @param {type:"slider", min:3, max:30, step:1}
    edit_template.num_segments = num_segments
    # Half the length of cylinder part.
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

# @title ####4.1.2 Wireframe Capsule :
# Acquire default template
capsule_wireframe_template = prim_attr_mgr.get_default_capsule_template(True)


# %%
# @title ####4.1.2 Wireframe Capsule :{ display-mode: "form" }
# Acquire default template
capsule_wireframe_template = prim_attr_mgr.get_default_capsule_template(True)


def edit_wf_capsule(edit_template):
    # Number of (line) rings for each hemisphere. Must be larger or equal to 1
    hemisphere_rings = 4  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.hemisphere_rings = hemisphere_rings

    # Number of (line) rings for cylinder. Must be larger or equal to 1.
    cylinder_rings = 4  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.cylinder_rings = cylinder_rings

    # Number of line segments. Must be larger or equal to 4 and multiple of 4
    num_segments = 16  # @param {type:"slider", min:4, max:40, step:4}
    edit_template.num_segments = num_segments

    # Half the length of cylinder part.
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
# #### 4.2 Cone : Cone of radius 1.0f along the Y axis, centered at origin.
#

# %%
# @title ####4.2.1 Solid Cone :{ display-mode: "form" }
# Acquire default template
cone_solid_template = prim_attr_mgr.get_default_cone_template(False)


def edit_solid_cone(edit_template):
    # Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # Number of (face) rings. Must be larger or equal to 1.
    num_rings = 6  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.num_rings = num_rings
    # Number of (face) segments. Must be larger or equal to 3.
    num_segments = 20  # @param {type:"slider", min:3, max:40, step:1}
    edit_template.num_segments = num_segments
    # Half the cone length
    half_length = 1.25  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # Whether to cap the base of the cone
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
# @title ####4.2.2 Wireframe Cone :{ display-mode: "form" }
# Acquire default template
cone_wireframe_template = prim_attr_mgr.get_default_cone_template(True)


def edit_wireframe_cone(edit_template):
    # Number of (line) segments. Must be larger or equal to 4 and multiple of 4.
    num_segments = 32  # @param {type:"slider", min:4, max:40, step:4}
    edit_template.num_segments = num_segments
    # Half the cone length
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
# #### 4.3 Cylinders : Cylinder of radius 1.0f along the Y axis, centered at origin.

# %%
# @title ####4.3.1 Solid Cylinder : { display-mode: "form" }
# Acquire default template
cylinder_solid_template = prim_attr_mgr.get_default_cylinder_template(False)


def edit_solid_cylinder(edit_template):
    # Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # Number of (face) rings. Must be larger or equal to 1.
    num_rings = 4  # @param {type:"slider", min:1, max:10, step:1}
    edit_template.num_rings = num_rings
    # Number of (face) segments. Must be larger or equal to 3.
    num_segments = 16  # @param {type:"slider", min:3, max:30, step:1}
    edit_template.num_segments = num_segments
    # Half the cylinder length
    half_length = 1  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
    edit_template.half_length = half_length
    # Whether to cap each end of the cylinder
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
# @title ####4.3.2 Wireframe Cylinder : { display-mode: "form" }
# Acquire default template
cylinder_wireframe_template = prim_attr_mgr.get_default_cylinder_template(True)


def edit_wireframe_cylinder(edit_template):
    # Number of (face) rings. Must be larger or equal to 1.
    num_rings = 4  # @param {type:"slider", min:1, max:10, step:1}
    # Number of (line) segments. Must be larger or equal to 4 and multiple of 4.
    num_segments = 32  # @param {type:"slider", min:4, max:64, step:4}
    edit_template.num_segments = num_segments
    # Half the cylinder length
    half_length = 1.0  # @param {type:"slider", min:0.05, max:2.0, step:0.05}
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
# #### 4.4 Icosphere : Sphere of radius 1.0f, centered at the origin.

# %%
# @title ####4.4.1 Solid Icosphere : { display-mode: "form" }
# @markdown Only solid icospheres have any editable geometric parameters.
# Acquire default template
icosphere_solid_template = prim_attr_mgr.get_default_icosphere_template(False)


def edit_solid_icosphere(edit_template):
    # Describes the depth of recursive subdivision for each icosphere triangle.
    subdivisions = 4  # @param {type:"slider", min:1, max:10, step:1}
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
# #### 4.5 UVSpheres : Sphere of radius 1.0f, centered at the origin.

# %%
# @title ####4.5.1 Solid UVSphere : { display-mode: "form" }
# Acquire default template
UVSphere_solid_template = prim_attr_mgr.get_default_UVsphere_template(False)


def edit_solid_UVSphere(edit_template):
    # Whether to build with texture coordinate support
    use_texture_coords = False  # @param {type:"boolean"}
    edit_template.use_texture_coords = use_texture_coords
    # Whether to build tangents
    use_tangents = False  # @param {type:"boolean"}
    edit_template.use_tangents = use_tangents
    # Number of (face) rings. Must be larger or equal to 2.
    num_rings = 13  # @param {type:"slider", min:2, max:30, step:1}
    edit_template.num_rings = num_rings
    # Number of (face) segments. Must be larger or equal to 3.
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
# @title ####4.5.2 Wireframe UVSphere : { display-mode: "form" }
# Acquire default template
UVSphere_wireframe_template = prim_attr_mgr.get_default_UVsphere_template(True)


def edit_wireframe_UVSphere(edit_template):
    # Number of (line) rings. Must be larger or equal to 2 and multiple of 2.
    num_rings = 20  # @param {type:"slider", min:2, max:64, step:2}
    # Number of (line) segments. Must be larger or equal to 4 and multiple of 4.
    num_segments = 32  # @param {type:"slider", min:4, max:64, step:4}
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
# #### 4.6 Instancing objects with modified primitive attributes templates.

# %%
# @title Instantiate primitive-based objects { display-mode: "form" }
# @markdown Using the modifications set in the previous examples in section 4,

# clear all objects and observations
remove_all_objects(sim)
observations = []
# save initial camera state for tracking
init_config = init_camera_track_config(sim)
# Previous cells configured solid_handles_to_use and wireframe_handles_to_use

# Set desired offset from agent location to place object
offset_solid = np.array([-1.1, 0.6, -1.8])
objs_to_sim = []
# Create primitive-attributes based object templates for solid and wireframe objects
for k, solidHandle in solid_handles_to_use.items():
    # Create object template with passed handle
    obj_template = obj_attr_mgr.create_template(solidHandle)
    # Create object from object template handle
    print("Solid Object being made using handle :{}".format(solidHandle))
    obj_solid_id = sim.add_object_by_handle(solidHandle)
    objs_to_sim.append(obj_solid_id)
    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_solid_id, offset=offset_solid)
    # Move offset for next object
    offset_solid[0] += 0.4

offset_wf = np.array([-1.1, 0.6, -1.0])

for k, wireframeHandle in wireframe_handles_to_use.items():
    # Create object template with passed handle
    obj_template = obj_attr_mgr.create_template(wireframeHandle)
    # Create object from object template handle
    print("Wireframe Object being made using handle :{}".format(wireframeHandle))
    obj_wf_id = sim.add_object_by_handle(wireframeHandle)
    objs_to_sim.append(obj_wf_id)
    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_wf_id, offset=offset_wf)
    # Move offset for next object
    offset_wf[0] += 0.4

example_type = "Adding customized primitive-based objects"
observations = camera_track_simulate(sim, objs_to_sim, dt=2.0)
if make_video:
    vut.make_video(
        observations, "color_sensor_1st_person", "color", output_path + example_type
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
