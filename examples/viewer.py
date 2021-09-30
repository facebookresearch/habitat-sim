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
        self.print_help_text()

    def draw_event(self):
        gl.default_framebuffer.clear(
            gl.FramebufferClear.COLOR | gl.FramebufferClear.DEPTH
        )

        self.sim._sensors["color_sensor"].draw_observation() # attempt to draw_observation

        # added to use blit_rgba_to_default()
        render_target = self.sim.agents[0]._sensors["color_sensor"].render_target
        render_target.blit_rgba_to_default()

        self.swap_buffers()

    # TODO: Find out why using a mouse-click to close window doesn't utilize 
    #       this function, leading to zsh: abort 
    # Override exit() method to first close the simulator before exiting
    def exit(self, arg0):
        self.sim.close(destroy=True)
        super().exit(arg0)

    def key_press_event(self, event):
        key = event.key
        pressed = Application.KeyEvent.Key

        if key == pressed.ESC:
            self.exit(0)
            return
        elif key == pressed.H:
            self.print_help_text()
            
    def print_help_text(self):
        print(
'''
=====================================================
Welcome to the Habitat-sim Python Viewer application!
=====================================================
Key Commands:
-------------
esc: Exit the application.
'h': Display this help message.
=====================================================
'''
        )           

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    
    # optional arguments
    parser.add_argument(
        '--scene',
        default="NONE",
        type=str, 
        help='scene/stage file to load (default: "NONE")',
    )
    parser.add_argument(        
        '--dataset',
        default='default',
        type=str,
        metavar='DATASET', 
        help='dataset configuration file to use (default: default)',
    )
    
    args = parser.parse_args()
    
    # TODO: Find a way to get this to print right before 
    #       the help text. Currently prints before huge 
    #       console logging blob
    # If no arguments are passed, print usage for user
    if vars(args) == vars(parser.parse_args([])):
        parser.print_help()

    # Setting up sim_settings
    sim_settings = default_sim_settings
    sim_settings['scene'] = args.scene
    sim_settings['scene_dataset_config_file'] = args.dataset

SkeletonPythonViewer(sim_settings).exec()