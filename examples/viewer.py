from magnum import gl
from magnum.platform.glfw import Application
from settings import default_sim_settings, make_cfg

import habitat_sim
import time
import math

class SkeletonPythonViewer(Application):
    def __init__(self, sim_settings):
        configuration = self.Configuration()
        configuration.title = "Skeleton Viewer Application"
        Application.__init__(self, configuration)

        # Set proper viewport size
        self.viewport_size = gl.default_framebuffer.viewport.size()
        sim_settings["width"] = self.viewport_size[0]
        sim_settings["height"] = self.viewport_size[1]

        # Set up movement dict
        key = Application.KeyEvent.Key
        self.pressed = {
            key.LEFT : False, key.RIGHT : False,
            key.UP : False,   key.DOWN : False,
            key.A : False,    key.D : False,
            key.S : False,    key.W : False,
            key.X : False,    key.Z : False
        }

        self.agent_id = sim_settings["default_agent"] # 0

        # Configure our sim_settings but then set agent to our default
        self.cfg = make_cfg(sim_settings)
        self.cfg.agents[self.agent_id] = self.default_agent_config()
        self.sim = habitat_sim.Simulator(self.cfg)

        self.active_scene_graph = self.sim.get_active_scene_graph()
        self.default_agent = self.sim.get_agent(self.agent_id)
        self.agent_body_node = self.default_agent.scene_node
        self.render_camera = self.agent_body_node.node_sensor_suite.get("color_sensor")

        self.time_since_last_simulation = 0.0
        Timer.start()

        self.step = -1
        self.print_help_text()
     
    def draw_event(self):
        agent_acts_per_sec = 60.0

        gl.default_framebuffer.clear(
            gl.FramebufferClear.COLOR | gl.FramebufferClear.DEPTH
        )

        # Agent actions should occur at a fixed rate per second
        self.time_since_last_simulation += Timer.prev_frame_duration
        num_agent_actions = (self.time_since_last_simulation * agent_acts_per_sec)

        self.move_and_look(int(num_agent_actions)) 
        if self.time_since_last_simulation >= 1.0 / 60.0:
            self.time_since_last_simulation = math.fmod(
                    self.time_since_last_simulation, 1.0 / 60.0)
        self.sim._sensors["color_sensor"].draw_observation()
        
        # added to use blit_rgba_to_default()
        sensor_render_target = self.render_camera.render_target
        sensor_render_target.blit_rgba_to_default()

        gl.default_framebuffer.bind()

        self.swap_buffers()
        Timer.next_frame()
        self.redraw()

    def move_and_look(self, repetitions):
        key = Application.KeyEvent.Key
        agent = self.sim.agents[self.agent_id]

        for _ in range(int(repetitions)):
            # No Movement
            if self.pressed[key.LEFT]:  agent.act('turn_left')
            if self.pressed[key.RIGHT]: agent.act('turn_right')
            if self.pressed[key.UP]:    agent.act('look_up')
            if self.pressed[key.DOWN]:  agent.act('look_down')

            # Yes Movement
            if self.pressed[key.A]: agent.act('move_left')
            if self.pressed[key.D]: agent.act('move_right')
            if self.pressed[key.S]: agent.act('move_backward')
            if self.pressed[key.W]: agent.act('move_forward')
            if self.pressed[key.X]: agent.act('move_down')
            if self.pressed[key.Z]: agent.act('move_up')

    def key_press_event(self, event):
        key = event.key
        pressed = Application.KeyEvent.Key

        if key == pressed.ESC:
            self.exit(0)
            return
        elif key == pressed.H:
            self.print_help_text()

        if key in self.pressed.keys():
            self.pressed[key] = True
        self.redraw()

    def key_release_event(self, event):
        key = event.key
        if key in self.pressed.keys():
                self.pressed[key] = False
        self.redraw()

    # Set up our own agent and agent controls
    def default_agent_config(self):
        ActionSpec = habitat_sim.agent.ActionSpec
        ActuationSpec = habitat_sim.agent.ActuationSpec
        MOVE = 0.07
        LOOK = 0.9
        action_space = {
            'turn_left' : ActionSpec('turn_left', ActuationSpec(amount=LOOK)),
            'turn_right' : ActionSpec('turn_right', ActuationSpec(amount=LOOK)),
            'look_up' : ActionSpec('look_up', ActuationSpec(amount=LOOK)),
            'look_down' : ActionSpec('look_down', ActuationSpec(amount=LOOK)),
            'move_left' : ActionSpec('move_left', ActuationSpec(amount=MOVE)),
            'move_right' : ActionSpec('move_right', ActuationSpec(amount=MOVE)),
            'move_backward' : ActionSpec('move_backward', ActuationSpec(amount=MOVE)),
            'move_forward' : ActionSpec('move_forward', ActuationSpec(amount=MOVE)),
            'move_down' : ActionSpec('move_down', ActuationSpec(amount=MOVE)),
            'move_up' : ActionSpec('move_up', ActuationSpec(amount=MOVE)),
        }

        sensor_spec = self.cfg.agents[self.agent_id].sensor_specifications
        
        agent_config = habitat_sim.agent.AgentConfiguration(
            height = 1.5,
            radius = 0.1,
            sensor_specifications = sensor_spec, # : typing.List[sensor.SensorSpec]
            action_space = action_space,         #: typing.Dict[typing.Any, ActionSpec]
            body_type= 'cylinder' 
        )
        return agent_config

    # TODO: Find out why using a mouse-click to close window doesn't utilize 
    #       this function, leading to zsh: abort 
    # Override exit() method to first close the simulator before exiting
    def exit(self, arg0):
        self.sim.close(destroy=True)
        super().exit(arg0)
            
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

    Agent Controls:
    'wasd':         Move the agent's body forward/backward, left/right.
    'zx':           Move the agent's body up/down.
    arrow keys:     Turn the agent's body left/right and camera look up/down.
=====================================================
'''
        )

class Timer:
    start_time = 0.0
    prev_frame_time = 0.0
    prev_frame_duration = 0.0
    running = False

    @staticmethod
    def start():
        Timer.running = True
        Timer.start_time = time.time()
        Timer.prev_frame_time = Timer.start_time
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def stop():
        Timer.running = False
        Timer.start_time = 0.0
        Timer.prev_frame_time = 0.0
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def next_frame():
        if not Timer.running:
            return
        Timer.prev_frame_duration = time.time() - Timer.prev_frame_time
        Timer.prev_frame_time = time.time()

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
