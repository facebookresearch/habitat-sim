# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     collapsed_sections: []
#     name: 'ECCV 2020: Navigation'
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
# #Habitat-sim Basics for Navigation
#
# The Habitat platform relies on a number of key abstractions that model the domain of embodied agents and tasks that can be carried out in three-dimensional indoor environments.
#
# - **Agent**: a physically embodied agent (e.g., a robot) with a suite of Sensors. Can observe the environment and is capable of taking actions that change agent or environment state.
# - **Sensor**: associated with a specific Agent, capable of returning observation data from the environment at a specified frequency.
# - **SceneGraph**: a hierarchical representation of a 3D environment that organizes the environment into regions and objects. Can be programmatically manipulated. The scene mesh, objects, agents, and sensors are all present on the SceneGraph.
# - **Simulator**: an instance of a simulator backend. Given actions for a set of configured Agents and SceneGraphs, can update the state of the Agents and SceneGraphs, and provide observations for all active Sensors possessed by the Agents.
#
# This tutorial covers the basics of using Habitat-sim for navigation tasks, including:
# - configuration of a Simulator, Sensors, and Agents.
# - taking actions and retrieving observations
# - pathfinding and navigation on the NavMesh

# %%
# @title Installation

# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/master/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s
# !wget -c http://dl.fbaipublicfiles.com/habitat/mp3d_example.zip && unzip -o mp3d_example.zip -d /content/habitat-sim/data/scene_datasets/mp3d/

# %%
# @title Colab Setup and Imports { display-mode: "form" }
# @markdown (double click to see the code)

import math
import os
import random
import sys

import cv2
import git
import magnum as mn
import numpy as np
# %matplotlib inline
from matplotlib import pyplot as plt
# function to display the topdown map
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut

# %cd /content/habitat-sim


if "google.colab" in sys.modules:
    # This tells imageio to use the system FFMPEG that has hardware acceleration.
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"


# %%
# @title Define Observation Display Utility Function { display-mode: "form" }

# @markdown A convenient function that displays sensor observations with matplotlib.

# @markdown (double click to see the code)


# Change to do something like this maybe: https://stackoverflow.com/a/41432704
def display_sample(rgb_obs, semantic_obs=np.array([]), depth_obs=np.array([])):
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

# %% [markdown]
# # "Hello, World!"
#
# Habitat simulator for navigation consists of **3** important concepts:
# - configurable embodied agents
# - multiple sensors
# - generic 3D dataset handling (e.g., Matterport, Gibson, and Replica datasets).
#
# In the 1st example, we demonstrate how to setup 1 agent with only 1 sensor (RGB visual sensor), place it in a scene, instruct it to navigate and collect the observations.

# %% [markdown]
# ### Basic settings
#
# To begin with, we specify a scene we are going to load, designate a default agent, and describe a couple of basic sensor parameters, such as the type, position, resolution of the obeservation (width and height).

# %%
# This is the scene we are going to load.
# we support a variety of mesh formats, such as .glb, .gltf, .obj, .ply
test_scene = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"

sim_settings = {
    "scene": test_scene,  # Scene path
    "default_agent": 0,  # Index of the default agent
    "sensor_height": 1.5,  # Height of sensors in meters, relative to the agent
    "width": 256,  # Spatial resolution of the observations
    "height": 256,
}


# %% [markdown]
# ### Configurations for the simulator
#
# To run the simulator, we need to create a configuration that are understandable by our simulator.\
# Such a configuration consists of **2** parts:
# - **One for the simulator backend.** It specifies parameters that are required to start and run the simulator. For example, the scene to be loaded, whether to load the semantic mesh, to enable physics or not. (Details: [code](https://github.com/facebookresearch/habitat-sim/blob/5820e1adc3ab238d2f564241d4705da5755542c9/src/esp/sim/Simulator.h#L44))
# - **One for the agent.** It describes parameters to initialize an agent, such as height, mass, as well as the configs for the attached sensors. User can also define the amount of displacement e.g., in a forward action and the turn angle.
# (Details: [code](https://github.com/facebookresearch/habitat-sim/blob/5820e1adc3ab238d2f564241d4705da5755542c9/src/esp/agent/Agent.h#L52))

# %%
# This function generates a config for the simulator.
# It contains two parts:
# one for the simulator backend
# one for the agent, where you can attach a bunch of sensors
def make_simple_cfg(settings):
    # simulator backend
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene.id = settings["scene"]

    # agent
    agent_cfg = habitat_sim.agent.AgentConfiguration()

    # In the 1st example, we attach only one sensor,
    # a RGB visual sensor, to the agent
    rgb_sensor_spec = habitat_sim.SensorSpec()
    rgb_sensor_spec.uuid = "color_sensor"
    rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_sensor_spec.resolution = [settings["height"], settings["width"]]
    rgb_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]

    agent_cfg.sensor_specifications = [rgb_sensor_spec]

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


cfg = make_simple_cfg(sim_settings)

# %% [markdown]
# ### Create a simulator instance

# %%
try:  # Needed to handle out of order cell run in Colab
    sim.close()
except NameError:
    pass
sim = habitat_sim.Simulator(cfg)

# %% [markdown]
# ### Initialize the agent
#
# After we initialize the simulator, we could put the agent in the scene, set and query its state, such as position and orientation.

# %%
# initialize an agent
agent = sim.initialize_agent(sim_settings["default_agent"])

# Set agent state
agent_state = habitat_sim.AgentState()
agent_state.position = np.array([-0.6, 0.0, 0.0])  # in world space
agent.set_state(agent_state)

# Get agent state
agent_state = agent.get_state()
print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)

# %% [markdown]
# ### Navigate and see

# %%
# obtain the default, discrete actions that an agent can perform
action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())
print("Discrete action space: ", action_names)


def navigateAndSee(action=""):
    if action in action_names:
        observations = sim.step(action)
        print("action: ", action)
        if display:
            display_sample(observations["color_sensor"])


action = "turn_right"
navigateAndSee(action)

action = "turn_right"
navigateAndSee(action)

action = "move_forward"
navigateAndSee(action)

action = "turn_left"
navigateAndSee(action)

# action = "move_backward"   // #illegal, no such action in the default action space
# navigateAndSee(action)

# %% [markdown]
# ### Take away and look ahead
#
# - **The basics of Habitat-Sim**: config and start the simulator, load a scene, setup an agent with a sensor, instruct the agent to navigate and see, obtain the observations.
# - In the following section, we will present a comprehensive example to demonstrate:
#     - **different types of sensors** ("rgb", "semantic", "depth" etc.) and their configurations
#     - **the semantic scene**, and its annotation information
#     - how to define and config the **action space**

# %% [markdown]
# # Full Config

# %%
# @title Configure Sim Settings

test_scene = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"

rgb_sensor = True  # @param {type:"boolean"}
depth_sensor = True  # @param {type:"boolean"}
semantic_sensor = True  # @param {type:"boolean"}

sim_settings = {
    "width": 256,  # Spatial resolution of the observations
    "height": 256,
    "scene": test_scene,  # Scene path
    "default_agent": 0,
    "sensor_height": 1.5,  # Height of sensors in meters
    "color_sensor": rgb_sensor,  # RGB sensor
    "depth_sensor": depth_sensor,  # Depth sensor
    "semantic_sensor": semantic_sensor,  # Semantic sensor
    "seed": 1,  # used in the random navigation
    "enable_physics": True,
}


# %%
def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene.id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]

    # Note: all sensors must have the same resolution
    sensors = {
        "color_sensor": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "depth_sensor": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "semantic_sensor": {
            "sensor_type": habitat_sim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
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

            sensor_specs.append(sensor_spec)

    # Here you can specify the amount of displacement in a forward action and the turn angle
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left", habitat_sim.agent.ActuationSpec(amount=30.0)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=30.0)
        ),
    }

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


# %%
cfg = make_cfg(sim_settings)
# Needed to handle out of order cell run in Colab
try:  # Got to make initialization idiot proof
    sim.close()
except NameError:
    pass
sim = habitat_sim.Simulator(cfg)


# %%
def print_scene_recur(scene, limit_output=10):
    print(
        f"House has {len(scene.levels)} levels, {len(scene.regions)} regions and {len(scene.objects)} objects"
    )
    print(f"House center:{scene.aabb.center} dims:{scene.aabb.sizes}")

    count = 0
    for level in scene.levels:
        print(
            f"Level id:{level.id}, center:{level.aabb.center},"
            f" dims:{level.aabb.sizes}"
        )
        for region in level.regions:
            print(
                f"Region id:{region.id}, category:{region.category.name()},"
                f" center:{region.aabb.center}, dims:{region.aabb.sizes}"
            )
            for obj in region.objects:
                print(
                    f"Object id:{obj.id}, category:{obj.category.name()},"
                    f" center:{obj.aabb.center}, dims:{obj.aabb.sizes}"
                )
                count += 1
                if count >= limit_output:
                    return None


# Print semantic annotation information (id, category, bounding box details)
# about levels, regions and objects in a hierarchical fashion
scene = sim.semantic_scene
print_scene_recur(scene)

# %%
# the randomness is needed when choosing the actions
random.seed(sim_settings["seed"])
sim.seed(sim_settings["seed"])

# Set agent state
agent = sim.initialize_agent(sim_settings["default_agent"])
agent_state = habitat_sim.AgentState()
agent_state.position = np.array([-0.6, 0.0, 0.0])  # world space
agent.set_state(agent_state)

# Get agent state
agent_state = agent.get_state()
print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)

# %%
total_frames = 0
action_names = list(cfg.agents[sim_settings["default_agent"]].action_space.keys())

max_frames = 5

while total_frames < max_frames:
    action = random.choice(action_names)
    print("action", action)
    observations = sim.step(action)
    rgb = observations["color_sensor"]
    semantic = observations["semantic_sensor"]
    depth = observations["depth_sensor"]

    if display:
        display_sample(rgb, semantic, depth)

    total_frames += 1


# %% [markdown]
# # Working with the NavMesh

# %% [markdown]
# Habitat-sim provides pathfinding and navigability constraints via intergration with [Recast Navigation | Detour](https://masagroup.github.io/recastdetour/) through the [nav module](https://aihabitat.org/docs/habitat-sim/habitat_sim.nav.html).
#
# This tutorial section demonstrates loading/recomputing/saving a NavMesh for a static scene and using it for a discrete navigation task. This module can also be applied to continuous navigation tasks, which is covered in the [advanced features tutorial (TODO: link this)](https://).
#
#
#

# %% [markdown]
# ##What is a NavMesh?

# %% [markdown]
# A navigation mesh (navmesh) is a collection of two-dimensional convex polygons (i.e., a polygon mesh) that define which areas of an environment are traversable by an agent with a particular embodiement. In other words, an agent could freely navigate around within these areas unobstructed by objects, walls, furniture, or other barriers that are part of the environment. Adjacent polygons are connected to each other in a graph enabling efficient pathfinding algorithms to chart routes between points on the navmesh as visualized below.
# <div>
# <img src="https://masagroup.github.io/recastdetour/recast_intro.png" width="300"/>
# </div>
#
# Using a NavMesh approximation of navigability, an agent is embodied as a rigid cylinder aligned with the gravity direction. The NavMesh is then computed by voxelizing the static scene and generating polygons on the top surfaces of solid voxels where the cylinder would sit without intersection or overhanging and respecting configured constraints such as maximum climeable slope and step-height.

# %% [markdown]
# ##NavMesh utilities:

# %% [markdown]
# ## Visualizing the NavMesh: Topdown Map
#
# The PathFinder API makes it easy to produce a topdown map of navigability in a scene. Since the NavMesh is a 3D mesh, and scenes can have multiple floor or levels vertically, we need to slice the NavMesh at specific world height (y coordinate). The map is then generated by sampling the NavMesh at a configurable resolution (meters_per_pixel) with 0.5 meters of vertical slack.
#
# The following example cell defines a matplotlib function to display top down map with optional key points and trajectory overlay. It then generates a topdown map of the current scene using the minimum y coordinate of the scene bounding box as the height, or an optionally configured custom height. Note that this height is in scene global coordinates, so we cannot assume that 0 is the bottom floor.

# %%
# small utilities for convenience
def unit_vector(vector):
    return vector / np.linalg.norm(vector)


# convert 3d points to 2d topdown coordinates
def convert_points_to_topdown(pathfinder, points, meters_per_pixel):
    points_topdown = []
    bounds = pathfinder.get_bounds()
    for point in points:
        # convert 3D x,z to topdown x,y
        px = (point[0] - bounds[0][0]) / meters_per_pixel
        py = (point[2] - bounds[0][2]) / meters_per_pixel
        points_topdown.append(np.array([px, py]))
    return points_topdown


def display_topdown_map(topdown_map, trajectory=None, key_points=None):
    size = topdown_map.shape[::-1]
    databytes = np.packbits(topdown_map, axis=1)
    databytes = databytes.copy(order="C")
    topdown_img = Image.frombytes(mode="1", size=size, data=databytes)
    plt.figure(figsize=(12, 8))
    ax = plt.subplot(1, 1, 1)
    ax.axis("off")
    # ax.set_title('topdown map')
    # plot a trajectory (image space) on the map
    if trajectory is not None:
        if len(trajectory) > 1:
            # plot the trajectory
            ax.plot(
                [p[0] for p in trajectory],
                [p[1] for p in trajectory],
                linewidth=2,
                color="firebrick",
            )
            # plot the agent icon
            tangent = unit_vector(trajectory[1] - trajectory[0])
            initial_angle = math.atan2(tangent[1], tangent[0]) * 180 / math.pi
            plt.plot(
                trajectory[0][0],
                trajectory[0][1],
                marker=(3, 0, -initial_angle - 90),
                markersize=20,
                linestyle="None",
                alpha=0.8,
            )
    # plot points on map
    if key_points is not None:
        for pix, point in enumerate(key_points):
            plt.plot(point[0], point[1], marker="o", markersize=10, alpha=0.8)
    # draw axis
    x = [1, 10]
    y = [1, 1]
    # ax.plot(x,y, linewidth=2, color='red')
    # ax.plot(y,x, linewidth=2, color='green')

    plt.imshow(topdown_img)
    plt.show(block=False)


try:
    # @markdown ###Configure Example Parameters:
    # @markdown Configure the map resolution:
    meters_per_pixel = 0.1  # @param {type:"slider", min:0.01, max:1.0, step:0.01}
    # @markdown ---
    # @markdown Customize the map slice height (global y coordinate):
    custom_height = False  # @param {type:"boolean"}
    height = 2.8  # @param {type:"slider", min:-10, max:10, step:0.1}
    # @markdown If not using custom height, default to scene lower limit.
    # @markdown (Cell output provides scene height range from bounding box for reference.)

    # get scene bounding box for specified height slice
    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
    print("The Scene asset height range is: " + str(scene_bb.y()))

    if not custom_height:
        height = scene_bb.y().min

    # This map is a 2D boolean array
    topdown_map = sim.pathfinder.get_topdown_view(meters_per_pixel, height)
    if display:
        display_topdown_map(topdown_map)

except:
    print("Error - aborting")
    pass

# %%
# @markdown ## Querying the NavMesh

if not sim.pathfinder.is_loaded:
    print("Pathfinder not initialized, aborting.")
else:
    # @markdown NavMesh area and bounding box can be queried via *navigable_area* and *get_bounds* respectively.
    print("NavMesh area = " + str(sim.pathfinder.navigable_area))
    print("Bounds = " + str(sim.pathfinder.get_bounds()))

    # @markdown A random point on the NavMesh can be queried with *get_random_navigable_point*.
    pathfinder_seed = 0  # @param {type:"integer"}
    sim.pathfinder.seed(pathfinder_seed)
    nav_point = sim.pathfinder.get_random_navigable_point()
    print("Random navigable point : " + str(nav_point))
    print("Is point navigable? " + str(sim.pathfinder.is_navigable(nav_point)))

    # @markdown The radius of the minimum containing circle (with vertex centroid origin) for the isolated navigable island of a point can be queried with *island_radius*.
    # @markdown This is analogous to the size of the point's connected component and
    # @markdown can be used to check that a queried navigable point is on an interesting surface (e.g. the floor), rather than a small surface (e.g. a table-top).
    print("Nav island radius : " + str(sim.pathfinder.island_radius(nav_point)))

    # @markdown The closest boundary point can also be queried (within some radius).
    max_search_radius = 2.0  # @param {type:"number"}
    print(
        "Distance to obstacle: "
        + str(sim.pathfinder.distance_to_closest_obstacle(nav_point, max_search_radius))
    )
    hit_record = sim.pathfinder.closest_obstacle_surface_point(
        nav_point, max_search_radius
    )
    print("Closest obstacle HitRecord:")
    print(" point: " + str(hit_record.hit_pos))
    print(" normal: " + str(hit_record.hit_normal))
    print(" distance: " + str(hit_record.hit_dist))

    vis_points = [nav_point]

    # HitRecord will have infinite distance if no valid point was found:
    if math.isinf(hit_record.hit_dist):
        print("No obstacle found within search radius.")
    else:
        # @markdown Points near the boundary or above the NavMesh can be snapped onto it.
        perturbed_point = hit_record.hit_pos - hit_record.hit_normal * 0.2
        print("Perturbed point : " + str(perturbed_point))
        print(
            "Is point navigable? " + str(sim.pathfinder.is_navigable(perturbed_point))
        )
        snapped_point = sim.pathfinder.snap_point(perturbed_point)
        print("Snapped point : " + str(snapped_point))
        print("Is point navigable? " + str(sim.pathfinder.is_navigable(snapped_point)))
        vis_points.append(snapped_point)

    # @markdown ---
    # @markdown ### Visualization
    # @markdown Running this cell generates a topdown visualization of the NavMesh with sampled points overlayed.
    meters_per_pixel = 0.1  # @param {type:"slider", min:0.01, max:1.0, step:0.01}

    # use the y coordinate of the sampled nav_point for the map height slice
    topdown_map = sim.pathfinder.get_topdown_view(meters_per_pixel, height=nav_point[1])
    xy_vis_points = convert_points_to_topdown(
        sim.pathfinder, vis_points, meters_per_pixel
    )
    if display:
        display_topdown_map(topdown_map, key_points=xy_vis_points)

# %%
# @markdown ## Pathfinding Queries on NavMesh

# @markdown The shortest path between valid points on the NavMesh can be queried
# @markdown as shown in this example.

# @markdown With a valid PathFinder instance:
if sim.pathfinder.is_loaded:
    seed = 0  # @param {type:"integer"}
    sim.pathfinder.seed(seed)

    # @markdown 1. Sample valid points on the NavMesh for agent spawn location and
    # @markdown pathfinding goal.
    sample1 = sim.pathfinder.get_random_navigable_point()
    sample2 = sim.pathfinder.get_random_navigable_point()

    # @markdown 2. Use ShortestPath module to compute path between samples.
    path = habitat_sim.ShortestPath()
    path.requested_start = sample1
    path.requested_end = sample2
    found_path = sim.pathfinder.find_path(path)
    geodesic_distance = path.geodesic_distance
    path_points = path.points
    # @markdown - Success, geodesic path length, and 3D points can be queried.
    print("found_path : " + str(found_path))
    print("geodesic_distance : " + str(geodesic_distance))
    print("path_points : " + str(path_points))

    # @markdown 3. Display trajectory (if found) on a topdown map of ground floor
    if found_path:
        meters_per_pixel = 0.1
        scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
        height = scene_bb.y().min
        topdown_map = sim.pathfinder.get_topdown_view(meters_per_pixel, height)
        trajectory = convert_points_to_topdown(
            sim.pathfinder, path_points, meters_per_pixel
        )
        if display:
            display_topdown_map(topdown_map, trajectory)

        display_path_agent_renders = False  # @param{type:"boolean"}
        # @markdown 4. (optional) Place agent and render images at trajectory points (if found).
        if display_path_agent_renders:
            print("Rendering observations at path points:")
            tangent = path_points[1] - path_points[0]
            agent_state = habitat_sim.AgentState()
            for ix, point in enumerate(path_points):
                if ix < len(path_points) - 1:
                    tangent = path_points[ix + 1] - point
                    agent_state.position = point
                    tangent_orientation_matrix = mn.Matrix4.look_at(
                        point, point + tangent, np.array([0, 1.0, 0])
                    )
                    tangent_orientation_q = mn.Quaternion.from_matrix(
                        tangent_orientation_matrix.rotation()
                    )
                    agent_state.rotation = utils.quat_from_magnum(tangent_orientation_q)
                    agent.set_state(agent_state)

                    observations = sim.get_sensor_observations()
                    rgb = observations["color_sensor"]
                    semantic = observations["semantic_sensor"]
                    depth = observations["depth_sensor"]

                    if display:
                        display_sample(rgb, semantic, depth)


else:
    print("No pathfinder loaded, aborting.")


# %% [markdown]
# ##Loading a NavMesh for a scene:

# %% [markdown]
#
# To load a pre-computed NavMesh for a scene, simply include it with the scene asset you are loading using the `.navmesh` file-ending.
#
# E.g.
# ```
# habitat-test-scenes/
#     apartment_1.glb
#     apartment_1.navmesh
# ```
#
#

# %%
# initialize a new simulator with the apartment_1 scene
# this will automatically load the accompanying .navmesh file
sim_settings["scene"] = "./data/scene_datasets/habitat-test-scenes/apartment_1.glb"
cfg = make_cfg(sim_settings)
try:  # Got to make initialization idiot proof
    sim.close()
except NameError:
    pass
sim = habitat_sim.Simulator(cfg)

# the navmesh can also be explicitly loaded
sim.pathfinder.load_nav_mesh(
    "./data/scene_datasets/habitat-test-scenes/apartment_1.navmesh"
)

# %% [markdown]
# ## Recompute the NavMesh at runtime
#
# When computing the NavMesh at runtime, configuration options are available to customize the result based on the intended use case.
#
# These settings include (all quantities in world units):
# - **Voxelization parameters**:
#
#   *Decrease these for better accuracy at the cost of higher compute cost.*
#
#   **Note:** most continuous parameters are converted to multiples of cell dimensions, so these should be compatible values for best accuracy.
#   - **cell_size** - xz-plane voxel dimensions. [Limit: >= 0]
#   - **cell_height** - y-axis voxel dimension. [Limit: >= 0]
#
# - **Agent parameters**:
#
#   - **agent_height** - Height of the agent. Used to cull navigable cells with obstructions.
#   - **agent_radius** - Radius of the agent. Used as distance to erode/shrink the computed heightfield. [Limit: >=0]
#   - **agent_max_climb** - Maximum ledge height that is considered to still be traversable. [Limit: >=0]
#   - **agent_max_slope** - The maximum slope that is considered navigable. [Limits: 0 <= value < 85] [Units: Degrees]
#
# - **Navigable area filtering options** (default active):
#   - **filter_low_hanging_obstacles** - Marks navigable spans as non-navigable if the clearence above the span is less than the specified height.
#   - **filter_ledge_spans** - Marks spans that are ledges as non-navigable. This filter reduces the impact of the overestimation of conservative voxelization so the resulting mesh will not have regions hanging in the air over ledges.
#   - **filter_walkable_low_height_spans** - Marks navigable spans as non-navigable if the clearence above the span is less than the specified height. Allows the formation of navigable regions that will flow over low lying objects such as curbs, and up structures such as stairways.
#
# - **Detail mesh generation parameters**:
#   - **region_min_size** - Minimum number of cells allowed to form isolated island areas.
#   - **region_merge_size** - Any 2-D regions with a smaller span (cell count) will, if possible, be merged with larger regions. [Limit: >=0]
#   - **edge_max_len** - The maximum allowed length for contour edges along the border of the mesh. Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature. [Limit: >=0] [ / cell_size]
#   - **edge_max_error** - The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0]
#   - **verts_per_poly** - The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.[Limit: >= 3]
#   - **detail_sample_dist** - Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >= 0.9] [x cell_size]
#   - **detail_sample_max_error** - The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.) [Limit: >=0] [x cell_height]
#
#
#
#

# %%
# @markdown ## Recompute NavMesh:
navmesh_settings = habitat_sim.NavMeshSettings()

# @markdown Choose Habitat-sim defaults (e.g. for point-nav tasks), or custom settings.
use_custom_settings = True  # @param {type:"boolean"}
sim.navmesh_visualization = True  # @param {type:"boolean"}
if not use_custom_settings:
    navmesh_settings.set_defaults()
else:
    # fmt: off
    #@markdown ## Configure custom settings (if use_custom_settings):
    #@markdown Configure the following NavMeshSettings for customized NavMesh recomputation.
    #@markdown **Voxelization parameters**:
    navmesh_settings.cell_size = 0.05 #@param {type:"slider", min:0.01, max:0.2, step:0.01}
    #default = 0.05
    navmesh_settings.cell_height = 0.2 #@param {type:"slider", min:0.01, max:0.4, step:0.01}
    #default = 0.2

    #@markdown **Agent parameters**:
    navmesh_settings.agent_height = 1.5 #@param {type:"slider", min:0.01, max:3.0, step:0.01}
    #default = 1.5
    navmesh_settings.agent_radius = 0.1 #@param {type:"slider", min:0.01, max:0.5, step:0.01}
    #default = 0.1
    navmesh_settings.agent_max_climb = 0.2 #@param {type:"slider", min:0.01, max:0.5, step:0.01}
    #default = 0.2
    navmesh_settings.agent_max_slope = 45 #@param {type:"slider", min:0, max:85, step:1.0}
    # default = 45.0
    # fmt: on
    # @markdown **Navigable area filtering options**:
    navmesh_settings.filter_low_hanging_obstacles = True  # @param {type:"boolean"}
    # default = True
    navmesh_settings.filter_ledge_spans = True  # @param {type:"boolean"}
    # default = True
    navmesh_settings.filter_walkable_low_height_spans = True  # @param {type:"boolean"}
    # default = True

    # fmt: off
    #@markdown **Detail mesh generation parameters**:
    #@markdown For more details on the effects
    navmesh_settings.region_min_size = 20 #@param {type:"slider", min:0, max:50, step:1}
    #default = 20
    navmesh_settings.region_merge_size = 20 #@param {type:"slider", min:0, max:50, step:1}
    #default = 20
    navmesh_settings.edge_max_len = 12.0 #@param {type:"slider", min:0, max:50, step:1}
    #default = 12.0
    navmesh_settings.edge_max_error = 1.3 #@param {type:"slider", min:0, max:5, step:0.1}
    #default = 1.3
    navmesh_settings.verts_per_poly = 6.0 #@param {type:"slider", min:3, max:6, step:1}
    #default = 6.0
    navmesh_settings.detail_sample_dist = 6.0 #@param {type:"slider", min:0, max:10.0, step:0.1}
    #default = 6.0
    navmesh_settings.detail_sample_max_error = 1.0 #@param {type:"slider", min:0, max:10.0, step:0.1}
    # default = 1.0
    # fmt: on

# @markdown ---
# @markdown You can also include MotionType::STATIC objects as NavMesh obstacles:

include_static_objects = True  # @param {type:"boolean"}

navmesh_success = sim.recompute_navmesh(
    sim.pathfinder, navmesh_settings, include_static_objects
)

observations = sim.get_sensor_observations()
rgb = observations["color_sensor"]
semantic = observations["semantic_sensor"]
depth = observations["depth_sensor"]

if display:
    display_sample(rgb, semantic, depth)


# %%
# @markdown ##Saving the NavMesh

# fmt: off
# @markdown An existing NavMesh can be saved with *Pathfinder.save_nav_mesh(filename)*
if sim.pathfinder.is_loaded:
    navmesh_save_path = "/content/habitat-sim/data/test_saving.navmesh" #@param {type:"string"}
    sim.pathfinder.save_nav_mesh(navmesh_save_path)
    print('Saved NavMesh to "' + navmesh_save_path + '"')
    sim.pathfinder.load_nav_mesh(navmesh_save_path)
# fmt: on

# %%
# @title Define Simulation and Video Utlities { display-mode: "form" }


repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")
# @markdown Optionally configure the save path for video output:
output_directory = "examples/tutorials/nav_output/"  # @param {type:"string"}
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)


def make_video_cv2(observations, prefix="", open_vid=True, fps=60, multi_obs=False):
    videodims = (720, 544)
    video_file = output_path + prefix + ".mp4"
    print("Encoding the video: %s " % video_file)
    writer = vut.get_fast_video_writer(video_file, fps=fps)

    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150
    for ob in observations:

        # If in RGB/RGBA format, remove the alpha channel
        rgb_im_1st_person = cv2.cvtColor(ob["color_sensor"], cv2.COLOR_RGBA2RGB)

        if multi_obs:
            # embed the 1st person RBG frame into the 3rd person frame
            rgb_im_3rd_person = cv2.cvtColor(
                ob["rgba_camera_3rdperson"], cv2.COLOR_RGBA2RGB
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
            d_im = np.clip(ob["depth_camera_1stperson"], 0, 10)
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


# %% [markdown]
# #Taking Actions on the NavMesh:
#
# The following example demonstrates taking random agent actions on the NavMesh. Both continuous and discrete action spaces are available. Sliding vs. non-sliding scenarios are compared.
#
# ## What is sliding?
# Most game engines allow agents to slide along obstacles when commanding actions which collide with the environment. While this is a reasonable behavior in games, it does not accuractely reflect the result of collisions between robotic agents and the environment.
#
# We note that **allowing sliding** makes training easier and results in higher simulation performance, but **hurts sim-2-real transfer** of trained policies.
#
# For a more detailed exposition of this subject see our paper:
# ["Are We Making Real Progress in Simulated Environments? Measuring the Sim2Real Gap in Embodied Visual Navigation"](https://arxiv.org/abs/1912.06321).
#

# %%
# @title Discrete and Continuous Navigation:

# @markdown Running this cell

# @markdown ---
# @markdown ### Set example parameters:
seed = 7  # @param {type:"integer"}
# @markdown Optionally navigate on the currently configured scene and NavMesh instead of re-loading with defaults:
use_current_scene = False  # @param {type:"boolean"}


sim_settings["seed"] = seed
if not use_current_scene:
    # reload a default nav scene
    sim_settings["scene"] = "./data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"
    cfg = make_cfg(sim_settings)
    try:  # Got to make initialization idiot proof
        sim.close()
    except NameError:
        pass
    sim = habitat_sim.Simulator(cfg)
random.seed(sim_settings["seed"])
sim.seed(sim_settings["seed"])
# set new initial state
sim.initialize_agent(agent_id=0)
agent = sim.agents[0]

# @markdown Seconds to simulate:
sim_time = 10  # @param {type:"integer"}

# @markdown Optional continuous action space parameters:
continuous_nav = True  # @param {type:"boolean"}

# defaults for discrete control
# control frequency (actions/sec):
control_frequency = 3
# observation/integration frames per action
frame_skip = 1
if continuous_nav:
    control_frequency = 5  # @param {type:"slider", min:1, max:30, step:1}
    frame_skip = 12  # @param {type:"slider", min:1, max:30, step:1}


fps = control_frequency * frame_skip
print("fps = " + str(fps))
control_sequence = []
for action in range(int(sim_time * control_frequency)):
    if continuous_nav:
        # allow forward velocity and y rotation to vary
        control_sequence.append(
            {
                "forward_velocity": random.random() * 2.0,  # [0,2)
                "rotation_velocity": (random.random() - 0.5) * 2.0,  # [-1,1)
            }
        )
    else:
        control_sequence.append(random.choice(action_names))

# create and configure a new VelocityControl structure
vel_control = habitat_sim.physics.VelocityControl()
vel_control.controlling_lin_vel = True
vel_control.lin_vel_is_local = True
vel_control.controlling_ang_vel = True
vel_control.ang_vel_is_local = True

# try 2 variations of the control experiment
for iteration in range(2):
    # reset observations and robot state
    observations = []

    video_prefix = "nav_sliding"
    sim.config.sim_cfg.allow_sliding = True
    # turn sliding off for the 2nd pass
    if iteration == 1:
        sim.config.sim_cfg.allow_sliding = False
        video_prefix = "nav_no_sliding"

    print(video_prefix)

    # manually control the object's kinematic state via velocity integration
    time_step = 1.0 / (frame_skip * control_frequency)
    print("time_step = " + str(time_step))
    for action in control_sequence:

        # apply actions
        if continuous_nav:
            # update the velocity control
            # local forward is -z
            vel_control.linear_velocity = np.array([0, 0, -action["forward_velocity"]])
            # local up is y
            vel_control.angular_velocity = np.array([0, action["rotation_velocity"], 0])

        else:  # discrete action navigation
            discrete_action = agent.agent_config.action_space[action]

            did_collide = False
            if agent.controls.is_body_action(discrete_action.name):
                did_collide = agent.controls.action(
                    agent.scene_node,
                    discrete_action.name,
                    discrete_action.actuation,
                    apply_filter=True,
                )
            else:
                for _, v in agent._sensors.items():
                    habitat_sim.errors.assert_obj_valid(v)
                    agent.controls.action(
                        v.object,
                        discrete_action.name,
                        discrete_action.actuation,
                        apply_filter=False,
                    )

        # simulate and collect frames
        for frame in range(frame_skip):
            if continuous_nav:
                # Integrate the velocity and apply the transform.
                # Note: this can be done at a higher frequency for more accuracy
                agent_state = agent.state
                previous_rigid_state = habitat_sim.RigidState(
                    utils.quat_to_magnum(agent_state.rotation), agent_state.position
                )

                # manually integrate the rigid state
                target_rigid_state = vel_control.integrate_transform(
                    time_step, previous_rigid_state
                )

                # snap rigid state to navmesh and set state to object/agent
                end_pos = sim.step_filter(
                    previous_rigid_state.translation, target_rigid_state.translation
                )

                # set the computed state
                agent_state.position = end_pos
                agent_state.rotation = utils.quat_from_magnum(
                    target_rigid_state.rotation
                )
                agent.set_state(agent_state)

                # Check if a collision occured
                dist_moved_before_filter = (
                    target_rigid_state.translation - previous_rigid_state.translation
                ).dot()
                dist_moved_after_filter = (
                    end_pos - previous_rigid_state.translation
                ).dot()

                # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
                # collision _didn't_ happen. One such case is going up stairs.  Instead,
                # we check to see if the the amount moved after the application of the filter
                # is _less_ than the amount moved before the application of the filter
                EPS = 1e-5
                collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

            # run any dynamics simulation
            sim.step_physics(time_step)

            # render observation
            observations.append(sim.get_sensor_observations())

    print("frames = " + str(len(observations)))
    # video rendering with embedded 1st person view
    if make_video:
        make_video_cv2(
            observations,
            prefix=video_prefix,
            open_vid=show_video,
            fps=fps,
            multi_obs=False,
        )
    sim.reset()

# [/embodied_agent_navmesh]
if __name__ == "__main__":
    sys.exit(0)
