from magnum import gl
from magnum.platform.glfw import Application
from settings import default_sim_settings, make_cfg

import habitat_sim

class SkeletonPythonViewer(Application):
    def __init__(self, sim_settings):
        configuration = self.Configuration()
        configuration.title = "Skeleton Viewer Application"
        Application.__init__(self, configuration)

        # Set proper viewport size
        self.viewport_size = gl.default_framebuffer.viewport.size() #viewport size
        sim_settings["width"] = self.viewport_size[0]
        sim_settings["height"] = self.viewport_size[1]
        
        self.cfg = make_cfg(sim_settings)
        self.sim = habitat_sim.Simulator(self.cfg)
        self.agent_id = sim_settings["default_agent"]
        self.step = -1

    def draw_event(self):
        gl.default_framebuffer.clear(
            gl.FramebufferClear.COLOR | gl.FramebufferClear.DEPTH
        )

        self.sim._sensors["color_sensor"].draw_observation() # attempt to draw_observation

        # added to use blit_rgba_to_default()
        render_target = self.sim.agents[0]._sensors["color_sensor"].render_target
        render_target.blit_rgba_to_default()

        self.swap_buffers()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    # necessary arguments
    parser.add_argument(
        'scene',
        type=str, 
        help='scene/stage file to load ("default" for default scene)',
    )

    # optional arguments
    parser.add_argument(        
        '--dataset',
        type=str,
        metavar='DATASET', 
        help='scene/stage file to load ("default" for default scene)',
    )

    args = parser.parse_args()

    # Setting up sim_settings
    sim_settings = default_sim_settings
    sim_settings['scene'] = args.scene
    sim_settings['scene_dataset_config_file'] = args.dataset or 'default'

    # 'default' option added for now to make testing easier
    if args.scene == 'default':
        sim_settings[
            'scene'
        ] = 'data/scene_datasets/habitat-test-scenes/skokloster-castle.glb'
try:
    exit(SkeletonPythonViewer(sim_settings).exec())
except Exception:
    import traceback

    print(str(traceback.format_exc()))
