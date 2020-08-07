# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     collapsed_sections: []
#     name: 'ECCV 2020: Interactivity.ipynb'
#     provenance: []
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
# #Habitat-sim Interactivity and Advanced Features
#
# This use-case driven tutorial covers Habitat-sim interactivity and advanced features, including:
# - Adding new objects to a scene
# - Kinematic object manipulation
# - Physics simulation API
# - Agent embodiment
# - Object configuration via template libraries

# %%
# @title Installation { display-mode: "form" }
# @markdown (double click to show code).

# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/master/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s
# !wget -c http://dl.fbaipublicfiles.com/habitat/mp3d_example.zip && unzip -o mp3d_example.zip -d /content/habitat-sim/data/scene_datasets/mp3d/

# %%
# @title Path Setup and Imports { display-mode: "form" }
# @markdown (double click to show code).

import functools
# %cd /content/habitat-sim
## [setup]
import math
import os
import random
import sys
import time

import cv2
import git
import ipywidgets as widgets
import magnum as mn
import numpy as np
from IPython.display import display as ipydisplay
# For using jupyter/ipywidget IO components
from ipywidgets import fixed, interact, interact_manual, interactive
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
output_directory = "examples/tutorials/interactivity_output/"  # @param {type:"string"}
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
        "height": 540,
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


# %%
# @title Define Simulation Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown - remove_all_objects
# @markdown - simulate
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


# sample a random valid state for the object from the scene bounding box or navmesh
def sample_object_state(
    sim, object_id, from_navmesh, maintain_object_up=True, max_tries=100, bb=None
):
    # check that the object is not STATIC
    if sim.get_object_motion_type(object_id) is habitat_sim.physics.MotionType.STATIC:
        print("sample_object_state : Object is STATIC, aborting.")
    if from_navmesh:
        if not sim.pathfinder.is_loaded:
            print("sample_object_state : No pathfinder, aborting.")
            return False
    elif not bb:
        print(
            "sample_object_state : from_navmesh not specified and no bounding box provided, aborting."
        )
        return False
    tries = 0
    valid_placement = False
    # get scene bounding box
    # scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
    while not valid_placement and tries < max_tries:
        tries += 1
        # initialize sample location to random point in scene bounding box
        sample_location = np.array([0, 0, 0])
        if from_navmesh:
            # query random navigable point
            sample_location = sim.pathfinder.get_random_navigable_point()
        else:
            sample_location = np.random.uniform(bb.min, bb.max)
        # set the test state
        sim.set_translation(sample_location, object_id)
        if maintain_object_up:
            # random rotation only on the Y axis
            y_rotation = mn.Quaternion.rotation(
                mn.Rad(random.random() * 2 * math.pi), mn.Vector3(0, 1.0, 0)
            )
            sim.set_rotation(y_rotation * sim.get_rotation(object_id), object_id)
        else:
            # unconstrained random rotation
            sim.set_rotation(ut.random_quaternion(), object_id)

        # raise object such that lowest bounding box corner is above the navmesh sample point.
        if from_navmesh:
            obj_node = sim.get_object_scene_node(object_id)
            xform_bb = habitat_sim.geo.get_transformed_bb(
                obj_node.cumulative_bb, obj_node.transformation
            )
            # also account for collision margin of the scene
            scene_collision_margin = 0.04
            y_translation = mn.Vector3(
                0, xform_bb.size_y() / 2.0 + scene_collision_margin, 0
            )
            sim.set_translation(
                y_translation + sim.get_translation(object_id), object_id
            )

        # test for penetration with the environment
        if not sim.contact_test(object_id):
            valid_placement = True

    if not valid_placement:
        return False
    return True


# %%
# @title Define Template Dictionary Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown This cell defines utility functions to easily peak into Attribute template contents.

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
    # New fields, uncomment upon updating conda 8/4/20
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
    # New fields, uncomment upon updating conda 8/4/20
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
    # New fields, uncomment upon updating conda 8/4/20
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
# dictionary containing containing k-v pairs of attribute property names
# and values for the passed template. The values are tuples with
# the first entry being the value,the second being whether the property is
# editable and the third being the type.
def build_dict_of_attrs(template):
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


# This will set a template's attributes from the passed map
def set_template_vals_from_map(template, template_map):
    for k, v in template_map.items():
        setattr(template, k, v[0])
    return template


# This will display all the known values of an attributes template
def show_template_map_vals(template):
    template_map = build_dict_of_attrs(template)
    print("Template {} has : ".format(template.handle))
    for k, v in template_map.items():
        print(
            "\t key {} with value {} of type {} that is editable : {}".format(
                k, v[0], v[2], v[1]
            )
        )


# %%
# @title Define Visualization Utility Functions { display-mode: "form" }
# @markdown (double click to show code)
# @markdown - make_video_cv2
# @markdown - display_sample
def make_video_cv2(observations, prefix="", open_vid=True, multi_obs=False, fps=60):
    sensor_keys = list(observations[0])
    videodims = observations[0][sensor_keys[0]].shape
    videodims = (videodims[1], videodims[0])  # flip to w,h order
    print(videodims)
    video_file = output_path + prefix + ".mp4"
    print("Encoding the video: %s " % video_file)
    writer = vut.get_fast_video_writer(video_file, fps=fps)
    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150
    for ob in observations:

        # If in RGB/RGBA format, remove the alpha channel
        rgb_im_1st_person = cv2.cvtColor(
            ob["color_sensor_1st_person"], cv2.COLOR_RGBA2RGB
        )

        if multi_obs:
            # embed the 1st person RBG frame into the 3rd person frame
            rgb_im_3rd_person = cv2.cvtColor(
                ob["color_sensor_3rd_person"], cv2.COLOR_RGBA2RGB
            )
            resized_1st_person_rgb = cv2.resize(
                rgb_im_1st_person, thumb_size, interpolation=cv2.INTER_AREA
            )
            x_offset = 50
            y_offset_rgb = 50
            rgb_im_3rd_person[
                y_offset_rgb - 1 : y_offset_rgb + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            rgb_im_3rd_person[
                y_offset_rgb : y_offset_rgb + resized_1st_person_rgb.shape[0],
                x_offset : x_offset + resized_1st_person_rgb.shape[1],
            ] = resized_1st_person_rgb

            # embed the 1st person DEPTH frame into the 3rd person frame
            # manually normalize depth into [0, 1] so that images are always consistent
            d_im = np.clip(ob["depth_sensor_1st_person"], 0, 10)
            d_im /= 10.0
            bgr_d_im = cv2.cvtColor((d_im * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB)
            resized_1st_person_depth = cv2.resize(
                bgr_d_im, thumb_size, interpolation=cv2.INTER_AREA
            )
            y_offset_d = y_offset_rgb + 10 + thumb_size[1]
            rgb_im_3rd_person[
                y_offset_d - 1 : y_offset_d + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            rgb_im_3rd_person[
                y_offset_d : y_offset_d + resized_1st_person_depth.shape[0],
                x_offset : x_offset + resized_1st_person_depth.shape[1],
            ] = resized_1st_person_depth
            if rgb_im_3rd_person.shape[:2] != videodims:
                rgb_im_3rd_person = cv2.resize(
                    rgb_im_3rd_person, videodims, interpolation=cv2.INTER_AREA
                )
            # write the video frame
            writer.append_data(rgb_im_3rd_person)
        else:
            if rgb_im_1st_person.shape[:2] != videodims:
                rgb_im_1st_person = cv2.resize(
                    rgb_im_1st_person, videodims, interpolation=cv2.INTER_AREA
                )
            # write the 1st person observation to video
            writer.append_data(rgb_im_1st_person)
    writer.close()

    if open_vid:
        print("Displaying video")
        vut.display_video(video_file)


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

    plt.show()


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
    def on_sim_click(b):
        observations = simulate(sim, dt=dt)
        make_video_cv2(observations, prefix=prefix, open_vid=True, multi_obs=False)

    sim_and_vid_btn = set_button_launcher("Simulate and Make Video")
    sim_and_vid_btn.on_click(on_sim_click)
    ipydisplay(sim_and_vid_btn)


def make_clear_all_objects_button():
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
    file_obj_ddl, sel_file_obj_handle = set_handle_ddl_widget(
        file_obj_handles,
        "File-based Object",
        sel_file_obj_handle,
        on_file_obj_ddl_change,
    )
    # All primitive asset-based object template handles
    prim_obj_handles = obj_attr_mgr.get_synth_template_handles()
    prim_obj_ddl, sel_prim_obj_handle = set_handle_ddl_widget(
        prim_obj_handles,
        "Primitive-based Object",
        sel_prim_obj_handle,
        on_prim_obj_ddl_change,
    )
    # All primitive asset handles template handles
    prim_asset_handles = prim_attr_mgr.get_template_handles()
    prim_asset_ddl, sel_asset_handle = set_handle_ddl_widget(
        prim_asset_handles, "Primitive Asset", sel_asset_handle, on_prim_ddl_change
    )
    # Display DDLs
    ipydisplay(file_obj_ddl)
    ipydisplay(prim_obj_ddl)
    ipydisplay(prim_asset_ddl)


# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }

# convienience functions defined in Utility cell manage global variables
sim_settings = make_default_settings()
# set globals: sim,
make_simulator_from_settings(sim_settings)

# %% [markdown]
# #Interactivity in Habitat-sim
#
# This tutorial section covers how to configure and use the Habitat-sim object manipulation API to setup and run physical interaction simulations.
#
# ## Outline:
# This section is divided into three use-case driven sub-sections:
# 1.   Introduction to Interactivity
# 2.   Physical Reasoning
# 3.   Continuous Embodied Navigation
#
# For more tutorial examples and details see the [Interactive Rigid Objects tutorial](https://aihabitat.org/docs/habitat-sim/rigid-object-tutorial.html) also available for Colab [here](https://github.com/facebookresearch/habitat-sim/blob/master/examples/tutorials/colabs/rigid_object_tutorial.ipynb).
#
#
#
#

# %% [markdown]
# ## Introduction to Interactivity
#
# ####Easily add an object and simulate!
#
#

# %%
# @title Select a Simulation Object: { display-mode: "form" }
# @markdown Use the dropdown menu below to select an object for use in the
# @markdown following examples.

# @markdown File-based objects are loaded from and named after an asset file (e.g. banana.glb).

# @markdown Primitive objects are generated programmatically (e.g. uv_sphere) with
# @markdown handles (name/key for reference) uniquely generated from a specific parameterization.

# @markdown See the [Advanced Features](https://colab.research.google.com/drive/10iSSLQZiKJi86intkenN53iOQ-w5By1z?authuser=1#scrollTo=JSrIT5nz-1Os)
# @markdown section for more details about asset configuration.

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %%
# @title Add either a File-based or Primitive Asset-based object to the scene at a user-specified location.
# @markdown Running this will add a physically-modelled object of the selected
# @markdown type to the scene at the location specified by user, simulate forward for a few seconds and save a
# @markdown movie of the results.

# @markdown Choose either the primitive or file-based asset recently selected in the dropdown:
obj_template_handle = sel_file_obj_handle
asset_tempalte_handle = sel_asset_handle
object_type = "File-based"  # @param ["File-based","Primitive-based"]
if "File" in object_type:
    # Handle File-based object handle
    obj_template_handle = sel_file_obj_handle
elif "Primitive" in object_type:
    # Handle Primitive-based object handle
    obj_template_handle = sel_prim_obj_handle
else:
    # Unknown - defaults to file-based
    pass

# @markdown Configure the initial object location (local offset from the agent body node):
# default : offset=np.array([0,2.0,-1.5]), orientation=np.quaternion(1,0,0,0)
offset_x = 0.5  # @param {type:"slider", min:-2, max:2, step:0.1}
offset_y = 1.4  # @param {type:"slider", min:0, max:3.0, step:0.1}
offset_z = -1.5  # @param {type:"slider", min:-3, max:0, step:0.1}
offset = np.array([offset_x, offset_y, offset_z])

# @markdown Configure the initial object orientation via local Euler angle (degrees):
orientation_x = 0  # @param {type:"slider", min:-180, max:180, step:1}
orientation_y = 0  # @param {type:"slider", min:-180, max:180, step:1}
orientation_z = 0  # @param {type:"slider", min:-180, max:180, step:1}

# compose the rotations
rotation_x = mn.Quaternion.rotation(mn.Deg(orientation_x), mn.Vector3(1.0, 0, 0))
rotation_y = mn.Quaternion.rotation(mn.Deg(orientation_y), mn.Vector3(1.0, 0, 0))
rotation_z = mn.Quaternion.rotation(mn.Deg(orientation_z), mn.Vector3(1.0, 0, 0))
orientation = rotation_z * rotation_y * rotation_x

# Add object instantiated by desired template using template handle
obj_id_1 = sim.add_object_by_handle(obj_template_handle)

# @markdown Note: agent local coordinate system is Y up and -Z forward.
# Move object to be in front of the agent
set_object_state_from_agent(sim, obj_id_1, offset=offset, orientation=orientation)

# display a still frame of the scene after the object is added if RGB sensor is enabled
observations = sim.get_sensor_observations()
if display:
    if sim_settings["color_sensor_1st_person"]:
        display_sample(observations["color_sensor_1st_person"])

example_type = "adding objects test"
make_sim_and_vid_button(example_type)
make_clear_all_objects_button()


# %% [markdown]
#
#
#
#
# ## Physical Reasoning

# %% [markdown]
# This section demonstrates simple setups for physical reasoning tasks in Habitat-sim with a fixed camera position collecting data:
# - [Scripted vs. Dynamic Motion](https://colab.research.google.com/drive/10iSSLQZiKJi86intkenN53iOQ-w5By1z?authuser=1#scrollTo=oUlgE5P-_F65&line=1&uniqifier=1)
# - [Object Permanence](https://colab.research.google.com/drive/10iSSLQZiKJi86intkenN53iOQ-w5By1z?authuser=1#scrollTo=GTPL4fzY_GZt&line=1&uniqifier=1)
# - [Physical plausibility classification](https://colab.research.google.com/drive/10iSSLQZiKJi86intkenN53iOQ-w5By1z?authuser=1#scrollTo=DH3mLjq5PabM&line=1&uniqifier=1)
# - [Trajectory Prediction]()

# %%
# @title Select objects from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %%
# @title Scripted vs. Dynamic Motion
# @markdown A quick script to generate video data for AI classification of
# @markdown dynamically dropping vs. kinematically moving objects.
remove_all_objects(sim)
# @markdown Set the scene as dynamic or kinematic:
scenario_is_kinematic = False  # @param {type:"boolean"}

# add the selected object
obj_id_1 = sim.add_object_by_handle(sel_file_obj_handle)

# place the object
set_object_state_from_agent(
    sim, obj_id_1, offset=np.array([0, 2.0, -1.0]), orientation=ut.random_quaternion()
)

if scenario_is_kinematic:
    # use the velocity control struct to setup a constant rate kinematic motion
    sim.set_object_motion_type(habitat_sim.MotionType.KINEMATIC, obj_id_1)
    vel_control = sim.get_object_velocity_control(obj_id_1)
    vel_control.controlling_lin_vel = True
    vel_control.linear_velocity = np.array([0, -1.0, 0])

# simulate and collect observations
example_type = "kinematic vs dynamic"
simulate_and_make_vid(prefix=example_type, dt=2.0)
remove_all_objects(sim)


# %%
# @title Object Permanence
# @markdown This example script demonstrates a possible object permanence task.
# @markdown Two objects are dropped behind an occluder. One is removed while occluded.
remove_all_objects(sim)

# @markdown 1. Add the two dynamic objects.
# add the selected objects
obj_id_1 = sim.add_object_by_handle(sel_file_obj_handle)
obj_id_2 = sim.add_object_by_handle(sel_file_obj_handle)

# place the objects
set_object_state_from_agent(
    sim, obj_id_1, offset=np.array([0.5, 2.0, -1.0]), orientation=ut.random_quaternion()
)
set_object_state_from_agent(
    sim,
    obj_id_2,
    offset=np.array([-0.5, 2.0, -1.0]),
    orientation=ut.random_quaternion(),
)

# @markdown 2. Configure and add an occluder from a scaled cube primitive.
# Get a default cube primitive template
obj_attr_mgr = sim.get_object_template_manager()
cube_handle = obj_attr_mgr.get_template_handles("cube")[0]
cube_template_cpy = obj_attr_mgr.get_template_by_handle(cube_handle)
# Modify the template's configured scale.
cube_template_cpy.scale = np.array([0.32, 0.075, 0.01])
# Register the modified template under a new name.
obj_attr_mgr.register_template(cube_template_cpy, "occluder_cube")
# Instance and place the occluder object from the template.
occluder_id = sim.add_object_by_handle("occluder_cube")
set_object_state_from_agent(sim, occluder_id, offset=np.array([0.0, 1.4, -0.4]))
sim.set_object_motion_type(habitat_sim.MotionType.KINEMATIC, occluder_id)

# @markdown 3. Simulate at 60Hz, removing one object when it's center of mass
# @markdown drops below that of the occluder.
# Simulate and remove object when it passes the midpoint of the occluder
dt = 2.0
print("Simulating " + str(dt) + " world seconds.")
observations = []
# simulate at 60Hz to the nearest fixed timestep
start_time = sim.get_world_time()
while sim.get_world_time() < start_time + dt:
    sim.step_physics(1.0 / 60.0)
    # remove the object once it passes the occluder center
    if obj_id_2 in sim.get_existing_object_ids():
        if sim.get_translation(obj_id_2)[1] <= sim.get_translation(occluder_id)[1]:
            sim.remove_object(obj_id_2)
    observations.append(sim.get_sensor_observations())

example_type = "object permanence"
make_video_cv2(observations, prefix=example_type, open_vid=True, multi_obs=False)
remove_all_objects(sim)


# %%
# @title Physical Plausibility Classification
# @markdown This example demonstrates a physical plausibility expirement. A sphere
# @markdown is dropped onto the back of a couch to roll onto the floor. Optionally,
# @markdown an invisible plane is introduced for the sphere to roll onto producing
# @markdown non-physical motion.

introduce_surface = True  # @param{type:"boolean"}

remove_all_objects(sim)

# add a rolling object
obj_attr_mgr = sim.get_object_template_manager()
sphere_handle = obj_attr_mgr.get_template_handles("uvSphereSolid")[0]
obj_id_1 = sim.add_object_by_handle(sphere_handle)
set_object_state_from_agent(sim, obj_id_1, offset=np.array([1.0, 1.6, -1.95]))

if introduce_surface:
    # optionally add invisible surface
    cube_handle = obj_attr_mgr.get_template_handles("cube")[0]
    cube_template_cpy = obj_attr_mgr.get_template_by_handle(cube_handle)
    # Modify the template.
    cube_template_cpy.scale = np.array([1.0, 0.04, 1.0])
    surface_is_visible = False  # @param{type:"boolean"}
    cube_template_cpy.is_visibile = surface_is_visible
    # Register the modified template under a new name.
    obj_attr_mgr.register_template(cube_template_cpy, "invisible_surface")

    # Instance and place the surface object from the template.
    surface_id = sim.add_object_by_handle("invisible_surface")
    set_object_state_from_agent(sim, surface_id, offset=np.array([0.4, 0.88, -1.6]))
    sim.set_object_motion_type(habitat_sim.MotionType.STATIC, surface_id)


example_type = "physical plausibility"
observations = simulate(sim, dt=3.0)
if make_video:
    make_video_cv2(
        observations, prefix=example_type, open_vid=show_video, multi_obs=False
    )
remove_all_objects(sim)


# %%
# @title Trajectory Prediction
# @markdown This example demonstrates setup of a trajectory prediction task.
# @markdown Boxes are placed in a target zone and a sphere is given an initial
# @markdown velocity with the goal of knocking the boxes off the counter.

# @markdown ---
# @markdown Configure Parameters:

obj_attr_mgr = sim.get_object_template_manager()
remove_all_objects(sim)

seed = 2  # @param{type:"integer"}
random.seed(seed)
sim.seed(seed)
np.random.seed(seed)

# setup agent state manually to face the bar
agent_state = sim.agents[0].state
agent_state.position = np.array([-1.97496, 0.072447, -2.0894])
agent_state.rotation = ut.quat_from_coeffs([0, -1, 0, 0])
sim.agents[0].set_state(agent_state)

# load the target objects
cheezit_handle = obj_attr_mgr.get_template_handles("cheezit")[0]
# create range from center and half-extent
target_zone = mn.Range3D.from_center(
    mn.Vector3(-2.07496, 1.07245, -0.2894), mn.Vector3(0.5, 0.05, 0.1)
)
num_targets = 9  # @param{type:"integer"}
for target in range(num_targets):
    obj_id = sim.add_object_by_handle(cheezit_handle)
    rotate = mn.Quaternion.rotation(mn.Rad(-mn.math.pi_half), mn.Vector3(1.0, 0, 0))
    sim.set_rotation(rotate, obj_id)
    # sample state from the target zone
    if not sample_object_state(sim, obj_id, False, True, 100, target_zone):
        sim.remove_object(obj_id)


show_target_zone = False  # @param{type:"boolean"}
if show_target_zone:
    # Get and modify the wire cube template from the range
    cube_handle = obj_attr_mgr.get_template_handles("cubeWireframe")[0]
    cube_template_cpy = obj_attr_mgr.get_template_by_handle(cube_handle)
    cube_template_cpy.scale = target_zone.size()
    cube_template_cpy.is_collidable = False
    # Register the modified template under a new name.
    obj_attr_mgr.register_template(cube_template_cpy, "target_zone")
    # instance and place the object from the template
    target_zone_id = sim.add_object_by_handle("target_zone")
    sim.set_translation(target_zone.center(), target_zone_id)
    sim.set_object_motion_type(habitat_sim.MotionType.STATIC, target_zone_id)
    # print("target_zone_center = " + str(sim.get_translation(target_zone_id)))

# @markdown ---
# @markdown ###Ball properties:
# load the ball
sphere_handle = obj_attr_mgr.get_template_handles("uvSphereSolid")[0]
sphere_template_cpy = obj_attr_mgr.get_template_by_handle(sphere_handle)
# @markdown Mass:
ball_mass = 5.01  # @param {type:"slider", min:0.01, max:50.0, step:0.01}
sphere_template_cpy.mass = ball_mass
obj_attr_mgr.register_template(sphere_template_cpy, "ball")

ball_id = sim.add_object_by_handle("ball")
set_object_state_from_agent(sim, ball_id, offset=np.array([0, 1.4, 0]))

# @markdown Initial linear velocity (m/sec):
lin_vel_x = -1  # @param {type:"slider", min:-10, max:10, step:0.1}
lin_vel_y = 1  # @param {type:"slider", min:-10, max:10, step:0.1}
lin_vel_z = 5  # @param {type:"slider", min:0, max:10, step:0.1}
initial_linear_velocity = mn.Vector3(lin_vel_x, lin_vel_y, lin_vel_z)
sim.set_linear_velocity(initial_linear_velocity, ball_id)

# @markdown Initial angular velocity (rad/sec):
ang_vel_x = 0  # @param {type:"slider", min:-100, max:100, step:0.1}
ang_vel_y = 0  # @param {type:"slider", min:-100, max:100, step:0.1}
ang_vel_z = 0  # @param {type:"slider", min:-100, max:100, step:0.1}
initial_angular_velocity = mn.Vector3(ang_vel_x, ang_vel_y, ang_vel_z)
sim.set_angular_velocity(initial_angular_velocity, ball_id)

example_type = "trajectory prediction"
observations = simulate(sim, dt=3.0)
if make_video:
    make_video_cv2(
        observations, prefix=example_type, open_vid=show_video, multi_obs=False
    )
remove_all_objects(sim)

# %% [markdown]
# ## Embodied Continuous Navigation

# %% [markdown]
# The following example demonstrates setup and excecution of an embodied navigation and interaction scenario. An object and an agent embodied by a rigid locobot mesh are placed randomly on the NavMesh. A path is computed for the agent to reach the object which is executed by a continuous path-following controller. The object is then kinematically gripped by the agent and a second path is computed for the agent to reach a goal location, also executed by a continuous controller. The gripped object is then released and thrown in front of the agent.
#
# Note: for a more detailed explanation of the NavMesh see [this(TODO: link)]() tutorial.

# %%
# @title Select target object from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)


# %%
# @title Continuous Path Follower Example
# @markdown A python Class to provide waypoints along a path given agent states


class ContinuousPathFollower(object):
    def __init__(self, sim, path, agent_scene_node, waypoint_threshold):
        self._sim = sim
        self._points = path.points[:]
        assert len(self._points) > 0
        self._length = path.geodesic_distance
        self._node = agent_scene_node
        self._threshold = waypoint_threshold
        self._step_size = 0.01
        self.progress = 0  # geodesic distance -> [0,1]
        self.waypoint = path.points[0]

        # setup progress waypoints
        _point_progress = [0]
        _segment_tangents = []
        _length = self._length
        for ix, point in enumerate(self._points):
            if ix > 0:
                segment = point - self._points[ix - 1]
                segment_length = np.linalg.norm(segment)
                segment_tangent = segment / segment_length
                _point_progress.append(
                    segment_length / _length + _point_progress[ix - 1]
                )
                # t-1 -> t
                _segment_tangents.append(segment_tangent)
        self._point_progress = _point_progress
        self._segment_tangents = _segment_tangents
        # final tangent is duplicated
        self._segment_tangents.append(self._segment_tangents[-1])

        print("self._length = " + str(self._length))
        print("num points = " + str(len(self._points)))
        print("self._point_progress = " + str(self._point_progress))
        print("self._segment_tangents = " + str(self._segment_tangents))

    def pos_at(self, progress):
        if progress <= 0:
            return self._points[0]
        elif progress >= 1.0:
            return self._points[-1]

        path_ix = 0
        for ix, prog in enumerate(self._point_progress):
            if prog > progress:
                path_ix = ix
                break

        segment_distance = self._length * (progress - self._point_progress[path_ix - 1])
        return (
            self._points[path_ix - 1]
            + self._segment_tangents[path_ix - 1] * segment_distance
        )

    def update_waypoint(self):
        if self.progress < 1.0:
            wp_disp = self.waypoint - self._node.absolute_translation
            wp_dist = np.linalg.norm(wp_disp)
            node_pos = self._node.absolute_translation
            step_size = self._step_size
            threshold = self._threshold
            while wp_dist < threshold:
                self.progress += step_size
                self.waypoint = self.pos_at(self.progress)
                if self.progress >= 1.0:
                    break
                wp_disp = self.waypoint - node_pos
                wp_dist = np.linalg.norm(wp_disp)


def setup_path_visualization(sim, path_follower, vis_ids, vis_samples=100):
    sphere_handle = obj_attr_mgr.get_template_handles("uvSphereSolid")[0]
    sphere_template_cpy = obj_attr_mgr.get_template_by_handle(sphere_handle)
    sphere_template_cpy.scale *= 0.2
    obj_attr_mgr.register_template(sphere_template_cpy, "mini-sphere")
    vis_ids.append(sim.add_object_by_handle(sphere_handle))

    for point in path_follower._points:
        cp_id = sim.add_object_by_handle(sphere_handle)
        sim.set_translation(point, cp_id)
        vis_ids.append(cp_id)

    for i in range(vis_samples):
        cp_id = sim.add_object_by_handle("mini-sphere")
        sim.set_translation(path_follower.pos_at(float(i / vis_samples)), cp_id)
        vis_ids.append(cp_id)

    for id in vis_ids:
        sim.set_object_motion_type(habitat_sim.MotionType.KINEMATIC, id)


def track_waypoint(waypoint, rs, vc, dt=1.0 / 60.0):
    angular_error_threshold = 0.5
    max_linear_speed = 1.0
    max_turn_speed = 1.0
    glob_forward = rs.rotation.transform_vector(mn.Vector3(0, 0, -1.0)).normalized()
    glob_right = rs.rotation.transform_vector(mn.Vector3(-1.0, 0, 0)).normalized()
    to_waypoint = mn.Vector3(waypoint) - rs.translation
    u_to_waypoint = to_waypoint.normalized()
    angle_error = float(mn.math.angle(glob_forward, u_to_waypoint))
    if angle_error < angular_error_threshold:
        # move forward
        vc.linear_velocity = mn.Vector3(0, 0, -max_linear_speed)
    else:
        vc.linear_velocity = mn.Vector3(0)
    if angle_error > 0.2:
        rot_dir = 1.0
        if mn.math.dot(glob_right, u_to_waypoint) < 0:
            rot_dir = -1.0
        vc.angular_velocity = mn.Vector3(
            0, np.clip(rot_dir * angle_error / dt, -max_turn_speed, max_turn_speed), 0
        )
    else:
        vc.angular_velocity = mn.Vector3(0)


# grip/release and sync gripped object state kineamtically
class ObjectGripper(object):
    def __init__(
        self, sim, agent_scene_node, end_effector_offset,
    ):
        self._sim = sim
        self._node = agent_scene_node
        self._offset = end_effector_offset
        self._gripped_obj_id = -1
        self._gripped_obj_buffer = 0  # bounding box y dimension offset of the offset

    def sync_states(self):
        if self._gripped_obj_id != -1:
            agent_t = self._node.absolute_transformation_matrix()
            agent_t.translation += self._offset + mn.Vector3(
                0, self._gripped_obj_buffer, 0.0
            )
            sim.set_transformation(agent_t, self._gripped_obj_id)

    def grip(self, obj_id):
        if self._gripped_obj_id != -1:
            print("Oops, can't carry more than one item.")
            return
        self._gripped_obj_id = obj_id
        sim.set_object_motion_type(habitat_sim.MotionType.KINEMATIC, obj_id)
        object_node = sim.get_object_scene_node(obj_id)
        self._gripped_obj_buffer = object_node.cumulative_bb.size_y() / 2.0
        self.sync_states()

    def release(self):
        sim.set_object_motion_type(habitat_sim.MotionType.DYNAMIC, self._gripped_obj_id)
        sim.set_linear_velocity(
            self._node.absolute_transformation_matrix().transform_vector(
                mn.Vector3(0, 0, -1.0)
            )
            + mn.Vector3(0, 2.0, 0),
            self._gripped_obj_id,
        )
        self._gripped_obj_id = -1


# %%
# @title Embodied Continuous Navigation Example
# @markdown This example cell runs the object retrieval task.

import faulthandler

faulthandler.enable()

# @markdown First the Simulator is re-initialized with:
# @markdown - a 3rd person camera view
# @markdown - modified 1st person sensor placement
sim_settings = make_default_settings()
sim_settings[
    "scene"
] = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"  # @param{type:"string"}
sim_settings["sensor_pitch"] = 0
sim_settings["sensor_height"] = 0.6
sim_settings["color_sensor_3rd_person"] = True
sim_settings["depth_sensor_1st_person"] = True

make_simulator_from_settings(sim_settings)

# @markdown ---
# @markdown ### Set other example parameters:
seed = 14  # @param {type:"integer"}
random.seed(seed)
sim.seed(seed)
np.random.seed(seed)

sim.config.sim_cfg.allow_sliding = True  # @param {type:"boolean"}

print(sel_file_obj_handle)
# load a selected target object and place it on the NavMesh
obj_id_1 = sim.add_object_by_handle(sel_file_obj_handle)

if not sample_object_state(
    sim, obj_id_1, from_navmesh=True, maintain_object_up=True, max_tries=1000
):
    print("Couldn't find an initial object placement. Aborting.")

# load the locobot_merged asset
locobot_template_handle = obj_attr_mgr.get_file_template_handles("locobot")[0]

# add robot object to the scene with the agent/camera SceneNode attached
locobot_id = sim.add_object_by_handle(locobot_template_handle, sim.agents[0].scene_node)

# set the agent's body to kinematic since we will be updating position manually
sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, locobot_id)

# create and configure a new VelocityControl structure
# Note: this is NOT the object's VelocityControl, so it will not be consumed automatically in sim.step_physics
vel_control = habitat_sim.physics.VelocityControl()
vel_control.controlling_lin_vel = True
vel_control.lin_vel_is_local = True
vel_control.controlling_ang_vel = True
vel_control.ang_vel_is_local = True

# reset observations and robot state
sim.set_translation(sim.pathfinder.get_random_navigable_point(), locobot_id)
observations = []

# get shortest path to the object from the agent position
path1 = habitat_sim.ShortestPath()
path1.requested_start = sim.get_translation(locobot_id)
path1.requested_end = sim.get_translation(obj_id_1)
path2 = habitat_sim.ShortestPath()
path2.requested_start = path1.requested_end
path2.requested_end = sim.pathfinder.get_random_navigable_point()

found_path = sim.pathfinder.find_path(path1) and sim.pathfinder.find_path(path2)

if not found_path:
    print("Could not find path to object, aborting!")
vis_ids = []

gripper = ObjectGripper(
    sim, sim.get_object_scene_node(locobot_id), np.array([0.0, 0.6, 0.0])
)
continuous_path_follower = ContinuousPathFollower(
    sim, path1, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.3
)

time_step = 1.0 / 30.0
for i in range(2):
    if i == 1:
        gripper.grip(obj_id_1)
        continuous_path_follower = ContinuousPathFollower(
            sim, path2, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.3
        )

    show_waypoint_indicators = False  # @param {type:"boolean"}
    if show_waypoint_indicators:
        for id in vis_ids:
            sim.remove_object(id)
        setup_path_visualization(sim, continuous_path_follower, vis_ids)

    # manually control the object's kinematic state via velocity integration
    start_time = sim.get_world_time()
    max_time = 30.0
    while (
        continuous_path_follower.progress < 1.0
        and sim.get_world_time() - start_time < max_time
    ):
        continuous_path_follower.update_waypoint()
        if show_waypoint_indicators:
            sim.set_translation(continuous_path_follower.waypoint, vis_ids[0])

        previous_rigid_state = sim.get_rigid_state(locobot_id)

        # set velocities based on relative waypoint position/direction
        track_waypoint(
            continuous_path_follower.waypoint,
            previous_rigid_state,
            vel_control,
            dt=time_step,
        )

        # manually integrate the rigid state
        target_rigid_state = vel_control.integrate_transform(
            time_step, previous_rigid_state
        )

        # snap rigid state to navmesh and set state to object/agent
        end_pos = sim.step_filter(
            previous_rigid_state.translation, target_rigid_state.translation
        )
        sim.set_translation(end_pos, locobot_id)
        sim.set_rotation(target_rigid_state.rotation, locobot_id)

        # Check if a collision occured
        dist_moved_before_filter = (
            target_rigid_state.translation - previous_rigid_state.translation
        ).dot()
        dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

        # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
        # collision _didn't_ happen. One such case is going up stairs.  Instead,
        # we check to see if the the amount moved after the application of the filter
        # is _less_ than the amount moved before the application of the filter
        EPS = 1e-5
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

        gripper.sync_states()
        # run any dynamics simulation
        sim.step_physics(time_step)

        # render observation
        observations.append(sim.get_sensor_observations())

# release
gripper.release()
start_time = sim.get_world_time()
while sim.get_world_time() - start_time < 2.0:
    sim.step_physics(time_step)
    observations.append(sim.get_sensor_observations())

# video rendering with embedded 1st person view
video_prefix = "fetch"
make_video_cv2(
    observations,
    prefix=video_prefix,
    open_vid=True,
    multi_obs=True,
    fps=1.0 / time_step,
)

# remove locobot while leaving the agent node for later use
sim.remove_object(locobot_id, delete_object_node=False)
remove_all_objects(sim)

# %% [markdown]
# # Advanced Features
# This section contains advanced examples/demos of Habitat-sim interactivity.

# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }
sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"
sim_settings["sensor_pitch"] = 0

make_simulator_from_settings(sim_settings)

# %% [markdown]
# ## Advanced Topic : Asset and Object Customization
#
# TODO: a Colab form to edit all fields of an object template and register it.

# %%
# @title Select target object from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %%
# @title 1. Simple File-based Object Template modification. { display-mode: "form" }
# @markdown Running this will demonstrate how to create objects of varying size from a file-based template by iteratively modifying the template's scale value.  This will also demonstrate how to delete unwanted templates from the library.

# Get File-based template for banana using its handle
obj_template_handle = "./data/objects/banana.phys_properties.json"

obj_template = obj_attr_mgr.get_template_by_handle(obj_template_handle)

# Add object instantiated by desired template using template handle
obj_id = sim.add_object_by_handle(obj_template_handle)

# Set desired offset from agent location to place object
offset = np.array([-1.2, 1.4, -1.5])
# Move object to be in front of the agent
set_object_state_from_agent(sim, obj_id, offset=offset)

# Templates have editable fields that will directly affect the instanced
# objects built from them.  Here we iteratively modify and re-register the
# template, instancing a new object each time.
# Bigger Bananas!
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
observations = simulate(sim, dt=1.0)
if make_video:
    make_video_cv2(
        observations, prefix=example_type, open_vid=show_video, multi_obs=False
    )
make_clear_all_objects_button()


# %%
# @title 2. All Possible File-based Object Template fields modification. { display-mode: "form" }
# @markdown This lists all the possible modifiable fields available in an object template, allows for them to be edited and enables creating an object with the new template.

# @markdown ###2.1 Fields to edit for a new template
# Get a new template
new_template = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)

new_template_orig_map = build_dict_of_attrs(new_template)

# @markdown The desired mass of the object
mass = 1  # @param {type:"slider", min:0.1, max:50, step:0.1}
new_template.mass = mass

# @markdown The x,y,z components for the scale of the object.
scale_X = 10  # @param {type:"slider", min:0.1, max:10, step:0.1}
scale_Y = 10  # @param {type:"slider", min:0.1, max:10, step:0.1}
scale_Z = 10  # @param {type:"slider", min:0.1, max:10, step:0.1}
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


# %%
# @markdown ###2.2 Create an object and display it with the edited template.
# Add object instantiated by original template using template handle
original_template = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)
orig_obj_id = sim.add_object_by_handle(original_template.handle)

# Set desired offset from agent location to place object
offset = np.array([-0.5, 1.5, -1.5])
# Move object to be in front of the agent
set_object_state_from_agent(sim, orig_obj_id, offset=offset)

# Add new object instantiated by desired template using template handle
obj_id = sim.add_object(new_template_ID)

# Set desired offset from agent location to place object
offset[0] += 1.0
# Move object to be in front of the agent
set_object_state_from_agent(sim, obj_id, offset=offset)


example_type = "Adding customized objects"
# Either
simulate_and_make_vid(example_type, dt=2.5)
# or
# make_sim_and_vid_button(example_type, dt=2.5)
make_clear_all_objects_button()

# %%
# @title 3.0 Simple Primitive-based Object instantiation. { display-mode: "form" }
# @markdown Habitat-Sim has 6 built-in primitives available (Capsule, Cone, Cube, Cylinder, Icosphere and UVSphere), which are available as both solid and wireframe meshes.  Default objects are synthesized from these primitives automatically and are always available.

# Get Primitive-based solid object template handles
prim_solid_obj_handles = obj_attr_mgr.get_synth_template_handles("solid")
# Get Primitive-based wireframe object template handles
prim_wf_obj_handles = obj_attr_mgr.get_synth_template_handles("wireframe")

# Set desired offset from agent location to place object
offset_solid = np.array([-1.1, 1.6, -1.8])
offset_wf = np.array([-1.1, 1.6, -1.0])

for i in range(6):
    # Create object from template handle
    obj_solid_id = sim.add_object_by_handle(prim_solid_obj_handles[i])
    obj_wf_id = sim.add_object_by_handle(prim_wf_obj_handles[i])

    # Place object in scene relative to agent
    set_object_state_from_agent(sim, obj_solid_id, offset=offset_solid)
    set_object_state_from_agent(sim, obj_wf_id, offset=offset_wf)

    # Move offset for next object
    offset_solid[0] += 0.4
    offset_wf[0] += 0.4


example_type = "Adding primitive-basedd objects"
# Either
simulate_and_make_vid(example_type)
# or
# make_sim_and_vid_button(example_type, dt=1.5)
make_clear_all_objects_button()

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
make_video_cv2(
    observations, prefix=video_prefix, open_vid=True, multi_obs=False, fps=60.0
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
# ## Advanced Topic : Generating Clutter with STATIC objects included in NavMesh
#
# The NavMesh can be used to place objects on surfaces in the scene. Once objects are placed they can be set to MotionType::STATIC, indiciating that they are not moveable (kinematics and dynamics are disabled for STATIC objects). The NavMesh can then be recomputed including STATIC object meshes in the voxelization.
#
# This example demonstrates using the NavMesh to generate a cluttered scene for navigation. In this script we will:
#
# - Place objects off the NavMesh
# - Set them to MotionType::STATIC
# - Recompute the NavMesh including STATIC objects
# - Visualize the results

# %%
# @title Initialize Simulator and Load Scene { display-mode: "form" }
# @markdown (load the apartment_1 scene for clutter generation in an open space)
sim_settings = make_default_settings()
sim_settings["scene"] = "./data/scene_datasets/habitat-test-scenes/apartment_1.glb"
sim_settings["sensor_pitch"] = 0

make_simulator_from_settings(sim_settings)

# %%
# @title Select clutter object from the GUI: { display-mode: "form" }

build_widget_ui(obj_attr_mgr, prim_attr_mgr)

# %%
# @title Clutter Generation Script
# @markdown Configure some example parameters:

# position the agent
sim.agents[0].scene_node.translation = mn.Vector3(0.5, -1.60025, 6.15)
print(sim.agents[0].scene_node.rotation)
agent_orientation_y = -23  # @param{type:"integer"}
sim.agents[0].scene_node.rotation = mn.Quaternion.rotation(
    mn.Deg(agent_orientation_y), mn.Vector3(0, 1.0, 0)
)

num_objects = 10  # @param {type:"slider", min:0, max:20, step:1}
object_scale = 5  # @param {type:"slider", min:1.0, max:10.0, step:0.1}

# scale up the selected object
sel_obj_template_cpy = obj_attr_mgr.get_template_by_handle(sel_file_obj_handle)
sel_obj_template_cpy.scale = mn.Vector3(object_scale)
obj_attr_mgr.register_template(sel_obj_template_cpy, "scaled_sel_obj")

# add the selected object
sim.navmesh_visualization = True
remove_all_objects(sim)
fails = 0
for obj in range(num_objects):
    obj_id_1 = sim.add_object_by_handle("scaled_sel_obj")

    # place the object
    placement_success = sample_object_state(
        sim, obj_id_1, from_navmesh=True, maintain_object_up=True, max_tries=100
    )
    if not placement_success:
        fails += 1
        sim.remove_object(obj_id_1)
    else:
        # set the objects to STATIC so they can be added to the NavMesh
        sim.set_object_motion_type(habitat_sim.MotionType.STATIC, obj_id_1)

print("Placement fails = " + str(fails) + "/" + str(num_objects))

# recompute the NavMesh with STATIC objects
navmesh_settings = habitat_sim.NavMeshSettings()
navmesh_settings.set_defaults()
navmesh_success = sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)

# simulate and collect observations
example_type = "clutter generation"
observations = simulate(sim, dt=2.0)
if make_video:
    make_video_cv2(
        observations, prefix=example_type, open_vid=show_video, multi_obs=False
    )
remove_all_objects(sim)
sim.navmesh_visualization = False

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
