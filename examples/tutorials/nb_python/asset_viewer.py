# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     collapsed_sections: []
#     name: Habitat-sim Asset Viewer
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
#       jupytext_version: 1.13.0
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %% [markdown]
# <a href="https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/asset_viewer.ipynb" target="_parent"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a>

# %% [markdown]
# #Habitat-sim Asset Viewer
#
# This utility provides a user with the ability to view assets rendered by the Habitat-Sim engine.
#

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
import sys

import git
import numpy as np

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
output_directory = "examples/tutorials/asset_viewer_output/"  # @param {type:"string"}
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
    if "override_scene_light_defaults" in settings:
        sim_cfg.override_scene_light_defaults = settings[
            "override_scene_light_defaults"
        ]
    if "scene_light_setup" in settings:
        sim_cfg.scene_light_setup = settings["scene_light_setup"]

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
        "width": 1280,  # Spatial resolution of the observations
        "height": 720,
        "scene": "./data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",  # Scene path
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
    global metadata_mediator

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
    # get metadata_mediator
    metadata_mediator = sim.metadata_mediator

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


# [/setup]


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
    res_dict["template_id"] = (template.template_id, False, "int")
    res_dict["template_class"] = (template.template_class, False, "string")
    res_dict["file_directory"] = (template.file_directory, False, "string")
    res_dict["num_user_configs"] = (template.num_user_configs, False, "int")
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
    res_dict["shader_type"] = (phys_obj_template.shader_type, True, "int")
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
    res_dict["is_collidable"] = (phys_obj_template.is_collidable, True, "boolean")
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
    res_dict["is_visible"] = (obj_template.is_visible, True, "boolean")
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
    res_dict["frustum_culling"] = (scene_template.frustum_culling, True, "boolean")
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


# @title Define Simulation Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown - simulate
def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating {:.3f} world seconds.".format(dt))
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())
    return observations


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

# %% [markdown]
# # View Assets in Habitat-sim
# Use the code in this section to view assets in the Habitat-sim engine.

# %%
# [initialize]
# @title Initialize Simulator and Load Objects { display-mode: "form" }

sim_settings = make_default_settings()
sim_settings["scene"] = "none"
sim_settings["sensor_pitch"] = 0
sim_settings["override_scene_light_defaults"] = True
sim_settings["scene_light_setup"] = ""

# use 3rd person camera
sim_settings["color_sensor_3rd_person"] = True

make_simulator_from_settings(sim_settings)
# [/initialize]

# [specify_object]
# @markdown Drag a stage or object asset into a directory at the left,
# @markdown and then load it below by setting object_to_view_path
# @markdown Put the full path to the asset you would like to view here :
# fmt: off
object_to_view_path = "./data/test_assets/scenes/simple_room.glb"  # @param {type:"string"}
# fmt: on

# this is the name to save the resultant video with
clip_short_name = object_to_view_path.split("/")[-1].split(".")[0]

# [/specify_object]

# %% [markdown]
# ## Synthesize Carousel View
# This cell will make a carousel view of the loaded stage.

# %%
# [build_carousel_view]

# @markdown This cell loads the object, centers it, and then moves a camera in a circle around
# @markdown the center of the scene, recording observations, which are subsequently stitched
# @markdown together to build a video of the object

# check if desired object actually exists
if os.path.exists(object_to_view_path) and os.path.isfile(object_to_view_path):

    # Acquire the sensor being used
    visual_sensor = sim._sensors["color_sensor_3rd_person"]
    initial_sensor_position = np.array(visual_sensor._spec.position)
    initial_sensor_orientation = np.array(visual_sensor._spec.orientation)

    # load an object template and instantiate an object to view
    object_template = obj_attr_mgr.create_new_template(str(object_to_view_path), False)
    # if using a stage and it displays sideways, you may need to reorient it via its attributes for it to display properly.

    # @markdown If the asset being displayed is on its side, enable orientation_correction below :
    orientation_correction = False  # @param {type: "boolean"}
    # This will correct the orientation (Dependent on PR : )
    if orientation_correction:
        object_template.orient_up = (0.0, 0.0, 1.0)
        object_template.orient_front = (0.0, 1.0, 0.0)

    # modify template here if desired and then register it
    obj_temp_id = obj_attr_mgr.register_template(object_template)

    # create object
    obj = rigid_obj_mgr.add_object_by_template_id(obj_temp_id)
    # place object in center - must be done before setting to static
    # get bb of object
    obj_bbox = obj.root_scene_node.compute_cumulative_bb()
    # find center of bb and move to scene origin - this centers object
    obj.translation = -obj_bbox.center()
    # get max dim to use as scale for sensor placement
    bb_scale = max(obj_bbox.max)
    # determine sensor placement based on size of object
    sensor_pos = bb_scale * np.array([0, 1, 2])
    # set object to be static
    obj.motion_type = habitat_sim.physics.MotionType.STATIC

    # initialize an agent and set its intial state
    agent = sim.initialize_agent(sim_settings["default_agent"])
    agent_state = habitat_sim.AgentState()
    agent_state.position = np.array([0.0, 0.0, 0.0])  # in world space
    agent.set_state(agent_state)

    # set the sensor to be behind and above the agent's initial loc
    # distance is scaled by size of largest object dimension
    visual_sensor._spec.position = agent_state.position + sensor_pos
    visual_sensor._spec.orientation = np.array([-0.5, 0, 0])
    visual_sensor._sensor_object.set_transformation_from_spec()

    # Create observations array
    observations = []

    # @markdown Set how long the resutlant video should be, in seconds.  The object will make 1 full revolution during this time.
    video_length = 4.8  # @param {type:"slider", min:1.0, max:20.0, step:0.1}
    # Sim time step
    time_step = 1.0 / 60.0
    # Amount to rotate per frame to make 1 full rotation
    rot_amount = 2 * math.pi / (video_length / time_step)

    # simulate with updated camera at each frame
    start_time = sim.get_world_time()
    while sim.get_world_time() - start_time < video_length:
        sim.step_physics(time_step)
        # rotate the agent to rotate the camera
        agent_state.rotation *= ut.quat_from_angle_axis(
            rot_amount, np.array([0, 1.0, 0])
        )
        agent.set_state(agent_state)

        observations.append(sim.get_sensor_observations())

    # video rendering of carousel view
    video_prefix = clip_short_name + "_scene_view"
    if make_video:
        vut.make_video(
            observations,
            "color_sensor_3rd_person",
            "color",
            output_path + video_prefix,
            open_vid=show_video,
            video_dims=[1280, 720],
        )

    # reset the sensor state for other examples
    visual_sensor._spec.position = initial_sensor_position
    visual_sensor._spec.orientation = initial_sensor_orientation
    visual_sensor._sensor_object.set_transformation_from_spec()

    # remove added objects
    rigid_obj_mgr.remove_all_objects()
else:
    print(
        "\nChosen File : '{}' does not exist or cannot be found. Aborting.\n".format(
            object_to_view_path
        )
    )
# [/build_carousel_view]
# %%
