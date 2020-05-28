# [setup]
import math
import os

import cv2
import magnum as mn
import numpy as np

import habitat_sim
import habitat_sim.utils.common as ut
import tutorial_functions as tutFuncs

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

    # [/initialize]

    # [basics]

    # get the handles of the default object template for solid Cylinders
    primtitive_template_handles = sim.get_synth_template_handles("CylinderSolid")

    # only a single template will have been made with this
    primitive_cylinder_template_handle = primtitive_template_handles[0]

    # add a default cylinder to the scene
    id_1 = sim.add_object_by_handle(primitive_cylinder_template_handle)
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

    tutFuncs.remove_all_objects(sim)

    # [asset_customization]

    # build default icosphere and add to simulation
    orig_icosphere_handles = sim.get_synth_template_handles("icosphereSolid")

    # add a default icosphere to the scene
    id_0 = sim.add_object_by_handle(orig_icosphere_handles[0])
    sim.set_translation(np.array([1.80, 0, 0.4]), id_0)

    # get the handle of an icosphere template to customize
    prim_assset_template_handles = sim.get_prim_asset_template_handles("icosphereSolid")

    # get specific asset template for icosphereSolid asset
    icosphere_template = sim.get_primitive_asset_template_by_handle(
        prim_assset_template_handles[0]
    )

    # modify asset template to have subdivision level 2, with 320 triangles
    icosphere_template.set_subdivisions(10)

    # register new asset template in asset library
    new_asset_template_id = sim.register_primitive_asset_template(icosphere_template)

    # use new asset template to instantiate primitive-based object template
    new_icosphere_obj_template_id = sim.build_and_register_prim_object_template(
        new_asset_template_id
    )

    # add a modified sphere ot the scene
    id_1 = sim.add_object_by_ID(new_icosphere_obj_template_id)
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

    tutFuncs.remove_all_objects(sim)


if __name__ == "__main__":
    main(make_video=True, show_video=True)
