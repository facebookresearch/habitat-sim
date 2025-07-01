import os

import magnum as mn

from habitat_sim import Simulator
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.settings import default_sim_settings, make_cfg


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


if __name__ == "__main__":
    import argparse

    output_path = "output_videos/"
    os.makedirs(output_path, exist_ok=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.add_argument("--dataset", dest="dataset", type=str, default=None)
    parser.add_argument("--scene", dest="scene", type=str, default=None)
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.display
    display = args.display
    make_video = args.make_video

    sim_settings = default_sim_settings.copy()
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["scene"] = args.scene
    sim_settings["enable_hbao"] = True
    cfg = make_cfg(sim_settings)
    with Simulator(cfg) as sim:
        observations = []
        start_time = sim.get_world_time()
        while sim.get_world_time() < start_time + 4.0:
            sim.agents[0].scene_node.rotate(
                mn.Rad(mn.math.pi_half / 60.0), mn.Vector3(0, 1, 0)
            )
            sim.step_physics(1.0 / 60.0)
            if make_video:
                observations.append(sim.get_sensor_observations())

        # video rendering of carousel view
        video_prefix = f"HSSD_scene_{sim_settings['scene']}_preview"
        if make_video:
            vut.make_video(
                observations,
                "color_sensor",
                "color",
                output_path + video_prefix,
                open_vid=show_video,
                video_dims=[1280, 720],
            )
