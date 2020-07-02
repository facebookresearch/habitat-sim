# [setup]
import math
import os
from pathlib import Path

import cv2
import magnum as mn
import numpy as np

import examples.tutorials.tutorial_functions as tutFuncs
import habitat_sim
import habitat_sim.utils.common as ut

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "rigid_object_primitive_tutorial_output/")


def make_video_cv2(
    observations, camera_res=[540, 720], prefix="", open_vid=True, multi_obs=False
):
    videodims = (camera_res[1], camera_res[0])
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    video = cv2.VideoWriter(output_path + prefix + ".mp4", fourcc, 60, videodims)
    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150

    if multi_obs:
        for ob in observations:
            embed_image_data_list = [
                (ob["rgba_camera_1stperson"], False),
                (ob["depth_camera_1stperson"], True),
            ]

            res_image = tutFuncs.build_multi_obs_image(
                ob["rgba_camera_3rdperson"],
                embed_image_data_list,
                thumb_size,
                outline_frame,
            )
            # write the desired image to video
            video.write(res_image)
    else:
        for ob in observations:
            res_image = ob["rgba_camera_1stperson"][..., 0:3][..., ::-1]

            # write the desired image to video
            video.write(res_image)
    video.release()
    if open_vid:
        os.system("open " + output_path + prefix + ".mp4")


def make_tutorial_configuration(camera_resolution=[540, 720]):
    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    sensors_config_dict = {
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

    return tutFuncs.make_configuration("apartment_1.glb", True, sensors_config_dict)


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
    # camera y by x
    camera_resolution = [720, 1024]
    cfg = make_tutorial_configuration(camera_resolution)
    sim = habitat_sim.Simulator(cfg)
    agent_transform = tutFuncs.place_agent(
        sim, pos=[-0.15, -0.7, 1.0], rot=np.quaternion(-0.83147, 0, 0.55557, 0)
    )

    # get the primitive assets attributes manager
    prim_templates_mgr = sim.get_asset_template_manager()

    # get the physics object attributes manager
    obj_templates_mgr = sim.get_object_template_manager()

    # [/initialize]

    # [basics]

    # get the handles of the default object template for solid Cylinders
    # only a single template will have been made
    cylinder_template_handle = obj_templates_mgr.get_synth_template_handles(
        "CylinderSolid"
    )[0]

    # add a default cylinder to the scene
    id_1 = sim.add_object_by_handle(cylinder_template_handle)
    sim.set_translation(np.array([2.50, 0, 0.2]), id_1)

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        make_video_cv2(
            observations,
            camera_resolution,
            prefix="prim_obj_basics",
            open_vid=show_video,
        )

    # [/basics]
    # remove all instanced objects
    tutFuncs.remove_all_objects(sim)

    # [asset_customization]

    # get default primitive asset-based object template for solid icosphere
    orig_icosphere_handles = obj_templates_mgr.get_synth_template_handles(
        "icosphereSolid"
    )

    # add a default icosphere to the scene
    id_0 = sim.add_object_by_handle(orig_icosphere_handles[0])
    sim.set_translation(np.array([1.80, 0, 0.4]), id_0)

    # customize and reinstantiate
    # get default solid icosphere template
    icosphere_template = prim_templates_mgr.get_default_icosphere_template(
        is_wireframe=False
    )

    # modify asset template to have subdivision level 2, with 320 triangles
    icosphere_template.subdivisions = 10

    # register new asset template in asset library
    new_asset_template_id = prim_templates_mgr.register_template(icosphere_template, "")

    # use new asset template to instantiate primitive-based object template
    new_icosphere_obj_template = obj_templates_mgr.create_template(
        icosphere_template.handle
    )

    # add a modified sphere ot the scene
    id_1 = sim.add_object(new_icosphere_obj_template.ID)
    sim.set_translation(np.array([1.80, 0, 0.0]), id_1)

    # simulate
    observations = simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        make_video_cv2(
            observations,
            camera_resolution,
            prefix="prim_obj_asset_customization",
            open_vid=show_video,
        )

    # [/asset_customization]

    # remove all instanced objects
    tutFuncs.remove_all_objects(sim)


if __name__ == "__main__":
    main(make_video=True, show_video=True)
