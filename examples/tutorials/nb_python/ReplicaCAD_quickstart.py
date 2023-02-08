# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     collapsed_sections: []
#     name: Habitat-sim ReplicaCAD Quickstart
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
# <a href="https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ReplicaCAD_quickstart.ipynb" target="_parent"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a>

# %% [markdown]
# #Habitat-sim ReplicaCAD Quickstart
#
# This brief Colab tutorial demonstrates loading the ReplicaCAD dataset in Habitat-sim from a SceneDataset and rendering a short video of agent navigation with physics simulation.
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
import os
import sys

import git
import magnum as mn

import habitat_sim
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
output_directory = "examples/tutorials/replica_cad_output/"  # @param {type:"string"}
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
    sim_cfg.scene_dataset_config_file = settings["scene_dataset"]
    sim_cfg.scene_id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]
    # Specify the location of the scene dataset
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

    # Here you can specify the amount of displacement in a forward action and the turn angle
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


def make_default_settings():
    settings = {
        "width": 1280,  # Spatial resolution of the observations
        "height": 720,
        "scene_dataset": "data/replica_cad/replicaCAD.scene_dataset_config.json",  # dataset path
        "scene": "NONE",  # Scene path
        "default_agent": 0,
        "sensor_height": 1.5,  # Height of sensors in meters
        "sensor_pitch": 0,  # sensor pitch (x rotation in rads)
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
    # Holds the user's desired scene handle
    global selected_scene
    selected_scene = "NONE"


# [/setup]


# %%
# @title Define Simulation Utility Function { display-mode: "form" }
# @markdown (double click to show code)
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
def on_scene_ddl_change(ddl_values):
    global selected_scene
    selected_scene = ddl_values["new"]
    return selected_scene


# Build a dropdown list holding obj_handles and set its event handler
def set_handle_ddl_widget(scene_handles, sel_handle, on_change):
    descStr = "Available Scenes:"
    style = {"description_width": "300px"}
    obj_ddl = widgets.Dropdown(
        options=scene_handles,
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


# Builds widget-based UI components
def build_widget_ui(metadata_mediator):
    # Holds the user's desired scene
    global selected_scene

    # All file-based object template handles
    scene_handles = metadata_mediator.get_scene_handles()
    # Set default as first available valid handle, or NONE scene if none are available
    if len(scene_handles) == 0:
        selected_scene = "NONE"
    else:
        # Set default selection to be first valid non-NONE scene (for python consumers)
        for scene_handle in scene_handles:
            if "NONE" not in scene_handle:
                selected_scene = scene_handle
                break

    if not HAS_WIDGETS:
        # If no widgets present, return, using default
        return

    # Construct DDLs and assign event handlers
    # Build widgets
    scene_obj_ddl, selected_scene = set_handle_ddl_widget(
        scene_handles,
        selected_scene,
        on_scene_ddl_change,
    )

    # Display DDLs
    ipydisplay(scene_obj_ddl)


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
# # View ReplicaCAD in Habitat-sim
# Use the code in this section to view assets in the Habitat-sim engine.

# %%
# [initialize]
# @title Initialize Simulator{ display-mode: "form" }

sim_settings = make_default_settings()
make_simulator_from_settings(sim_settings)
# [/initialize]

# %%
# @title Select a SceneInstance: { display-mode: "form" }
# @markdown Select a scene from the dropdown and then run the next cell to load and simulate that scene and produce a visualization of the result.

build_widget_ui(sim.metadata_mediator)

# %% [markdown]
# ## Load the Select Scene and Simulate!
# This cell will load the scene selected above, simulate, and produce a visualization.

# %%
global selected_scene
if sim_settings["scene"] != selected_scene:
    sim_settings["scene"] = selected_scene
    make_simulator_from_settings(sim_settings)

observations = []
start_time = sim.get_world_time()
while sim.get_world_time() < start_time + 4.0:
    sim.agents[0].scene_node.rotate(mn.Rad(mn.math.pi_half / 60.0), mn.Vector3(0, 1, 0))
    sim.step_physics(1.0 / 60.0)
    if make_video:
        observations.append(sim.get_sensor_observations())

# video rendering of carousel view
video_prefix = "ReplicaCAD_scene_view"
if make_video:
    vut.make_video(
        observations,
        "color_sensor_1st_person",
        "color",
        output_path + video_prefix,
        open_vid=show_video,
        video_dims=[1280, 720],
    )
