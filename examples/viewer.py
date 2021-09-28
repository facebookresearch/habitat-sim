from magnum import gl
from magnum.platform.glfw import Application
from settings import default_sim_settings, make_cfg

import habitat_sim

# Setting filepath to load
sim_settings = default_sim_settings
sim_settings[
    "scene"
] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"


class SkeletonPythonViewer(Application):
    def __init__(self):
        configuration = self.Configuration()
        configuration.title = "Skeleton Viewer Application"
        Application.__init__(self, configuration)
        self.viewport_size = gl.default_framebuffer.viewport.size() #viewport size

        # attempt to set proper width and height
        sim_settings["width"] = self.viewport_size[1]
        sim_settings["height"] = self.viewport_size[0]
        
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


try:
    exit(SkeletonPythonViewer().exec())
except Exception:
    import traceback

    print(str(traceback.format_exc()))
