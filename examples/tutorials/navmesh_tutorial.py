# [setup]
import math
import os
import random

import cv2
import magnum as mn
import numpy as np
from matplotlib import pyplot as plt

import habitat_sim
from habitat_sim.utils import common as ut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "navmesh_tutorial_output/")

save_index = 0


def make_video_cv2(observations, prefix="", open_vid=True, multi_obs=False):
    videodims = (720, 540)
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    video = cv2.VideoWriter(output_path + prefix + ".mp4", fourcc, 60, videodims)
    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150
    for ob in observations:

        # If in RGB/RGBA format, change first to RGB and change to BGR
        bgr_im_1st_person = ob["rgba_camera_1stperson"][..., 0:3][..., ::-1]

        if multi_obs:
            # embed the 1st person RBG frame into the 3rd person frame
            bgr_im_3rd_person = ob["rgba_camera_3rdperson"][..., 0:3][..., ::-1]
            resized_1st_person_rgb = cv2.resize(
                bgr_im_1st_person, thumb_size, interpolation=cv2.INTER_AREA
            )
            x_offset = 50
            y_offset_rgb = 50
            bgr_im_3rd_person[
                y_offset_rgb - 1 : y_offset_rgb + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            bgr_im_3rd_person[
                y_offset_rgb : y_offset_rgb + resized_1st_person_rgb.shape[0],
                x_offset : x_offset + resized_1st_person_rgb.shape[1],
            ] = resized_1st_person_rgb

            # embed the 1st person DEPTH frame into the 3rd person frame
            # manually normalize depth into [0, 1] so that images are always consistent
            d_im = np.clip(ob["depth_camera_1stperson"], 0, 10)
            d_im /= 10.0
            bgr_d_im = cv2.cvtColor((d_im * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            resized_1st_person_depth = cv2.resize(
                bgr_d_im, thumb_size, interpolation=cv2.INTER_AREA
            )
            y_offset_d = y_offset_rgb + 10 + thumb_size[1]
            bgr_im_3rd_person[
                y_offset_d - 1 : y_offset_d + outline_frame.shape[0] - 1,
                x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
            ] = outline_frame
            bgr_im_3rd_person[
                y_offset_d : y_offset_d + resized_1st_person_depth.shape[0],
                x_offset : x_offset + resized_1st_person_depth.shape[1],
            ] = resized_1st_person_depth

            # write the video frame
            video.write(bgr_im_3rd_person)
        else:
            # write the 1st person observation to video
            video.write(bgr_im_1st_person)
    video.release()
    if open_vid:
        os.system("open " + output_path + prefix + ".mp4")


def show_img(data, save):
    # display rgb and semantic images side-by-side
    plt.figure(figsize=(12, 12))
    plt.imshow(data, interpolation="nearest")
    plt.axis("off")
    plt.show(block=False)
    if save:
        global save_index
        plt.savefig(
            output_path + str(save_index) + ".jpg",
            bbox_inches="tight",
            pad_inches=0,
            quality=50,
        )
        save_index += 1
    plt.pause(1)


def get_obs(sim, show, save):
    # render sensor ouputs and optionally show them
    rgb_obs = sim.get_sensor_observations()["rgba_camera_1stperson"]
    if show:
        show_img(rgb_obs, save)


def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [-0.15, -0.7, 1.0]
    # agent_state.position = [-0.15, -1.6, 1.0]
    agent_state.rotation = np.quaternion(-0.83147, 0, 0.55557, 0)
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    backend_cfg.enable_physics = True

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    camera_resolution = [540, 720]
    sensors = {
        "rgba_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 0.6, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
        "depth_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": camera_resolution,
            "position": [0.0, 0.6, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
        "rgba_camera_3rdperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 1.0, 0.3],
            "orientation": [-45, 0.0, 0.0],
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = habitat_sim.SensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]
        sensor_spec.orientation = sensor_params["orientation"]
        sensor_specs.append(sensor_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


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


# [/setup]

# This is wrapped such that it can be added to a unit test
def main(make_video=True, show_video=True):
    if make_video:
        if not os.path.exists(output_path):
            os.mkdir(output_path)

    # [initialize]
    # create the simulator
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)

    # get the primitive assets attributes manager
    prim_templates_mgr = sim.get_asset_template_manager()

    # get the physics object attributes manager
    obj_templates_mgr = sim.get_object_template_manager()

    # [/initialize]

    # [basics]

    # turn on navmesh visualization and render the scene
    sim.toggle_navmesh_visualization()
    get_obs(sim, show_video, True)

    # load some object templates from configuration files
    chair_template_id = sim.load_object_configs(
        str(os.path.join(data_path, "test_assets/objects/chair"))
    )[0]

    # add a STATIC object to the scene blocking a path
    id_1 = sim.add_object(chair_template_id)
    sim.set_translation(np.array([2.0, -1.1, -0.5]), id_1)
    sim.set_object_motion_type(habitat_sim.physics.MotionType.STATIC, id_1)

    # recompute the NavMesh
    navmesh_settings = habitat_sim.NavMeshSettings()
    navmesh_settings.set_defaults()
    sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
    get_obs(sim, show_video, True)

    # [/basics]

    remove_all_objects(sim)

    # TODO: add more after colab conversion


if __name__ == "__main__":
    main(make_video=True, show_video=True)
