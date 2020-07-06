# [setup]
import math
import os
from pathlib import Path

import magnum as mn
import numpy as np


import habitat_sim
import habitat_sim.utils.common as ut
import habitat_sim.utils.tutorial_functions as tutFuncs

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "rigid_object_primitive_tutorial_output/")


def make_tutorial_configuration(camera_list, camera_resolution=[540, 720]):
    """
    Set sensor configuration specific to this tutorial and 
    then build simulator, sensor and agent configurations. 

    Parameters
    ----------
    camera_list : list of tuples
        Each tuple value holds :
          idx 0 : name of camera
          idx 1 : whether camera is depth or not
          idx 2 : whether camera is 1st person or 3rd person

    camera_resolution : list of ints
            Y,X resolution of sensor image.


    Returns
    -------
    cfg : habitat_sim.Configuration
        Configuration required to instance necessary Habitat components.

    """

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup 2 rgb sensors for 1st and 3rd person views
    sensors_config_dict = {}
    for camera in camera_list:
        camera_name = camera[0]
        temp_dict = {}
        # if depth else color
        temp_dict["sensor_type"] = (
            habitat_sim.SensorType.DEPTH
            if "depth" in camera[1].lower()
            else habitat_sim.SensorType.COLOR
        )
        temp_dict["resolution"] = camera_resolution
        # check if 1st or 3rd person camera
        if "firstperson" in camera[2].lower():
            temp_dict["position"] = [0.0, 0.6, 0.0]
            temp_dict["orientation"] = [0.0, 0.0, 0.0]
        else:
            temp_dict["position"] = [0.0, 1.0, 0.3]
            temp_dict["orientation"] = [-45, 0.0, 0.0]
        sensors_config_dict[camera_name] = temp_dict

    return tutFuncs.make_configuration("apartment_1.glb", True, sensors_config_dict)


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

    # camera names and povs to use
    camera_list = [
        ("rgba_camera_1stperson", "Color", "FirstPerson"),
        ("depth_camera_1stperson", "Depth", "FirstPerson"),
        ("rgba_camera_3rdperson", "Color", "ThirdPerson"),
    ]

    # build tutorial simulator configuration
    cfg = make_tutorial_configuration(camera_list, camera_resolution)
    # construct an instance of simulator with desired configuration
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
    observations = tutFuncs.simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        tutFuncs.make_video_cv2(
            observations,
            camera_list[0][0],
            output_path=output_path,
            file_name="prim_obj_basics",
            camera_res=camera_resolution,
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
    observations = tutFuncs.simulate(sim, dt=1.5, get_frames=make_video)

    if make_video:
        tutFuncs.make_video_cv2(
            observations,
            camera_list[0][0],
            output_path=output_path,
            file_name="prim_obj_asset_customization",
            camera_res=camera_resolution,
            open_vid=show_video,
        )

    # [/asset_customization]

    # remove all instanced objects
    tutFuncs.remove_all_objects(sim)


if __name__ == "__main__":
    main(make_video=True, show_video=True)
