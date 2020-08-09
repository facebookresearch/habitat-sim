from magnum import gl, shaders
from magnum.platform.glfw import Application

import habitat_sim

settings = {
    "max_frames": 10,
    "width": 256,  # Spatial resolution of the observations
    "height": 256,
    "scene": "data/scene_datasets/habitat-test-scenes/apartment_1.glb",  # Scene path
    "default_agent": 0,
    "sensor_height": 1.5,  # Height of sensors in meters
    "rgb": True,  # RGB sensor
    "seed": 1,
    "enable_physics": True,
    "physics_config_file": "data/default.phys_scene_config.json",
    "silent": False,
    "num_objects": 10,
    "compute_shortest_path": False,
    "compute_action_shortest_path": False,
    "save_png": True,
    "render_to_ui": True,
}

navmesh_settings = habitat_sim.NavMeshSettings()
navmesh_settings.set_defaults()
navmesh_settings.agent_radius = 0.1
navmesh_settings.agent_height = 0.5
navmesh_settings.agent_max_climb = 0.0
navmesh_settings.agent_max_slope = 0.0


def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene.id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]
    sim_cfg.physics_config_file = settings["physics_config_file"]

    # Note: all sensors must have the same resolution
    sensors = {
        "rgb": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        }
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
            "move_forward", habitat_sim.agent.ActuationSpec(amount=0.1)
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
        "look_down": habitat_sim.agent.ActionSpec(
            "look_down", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
        "look_up": habitat_sim.agent.ActionSpec(
            "look_up", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
    }
    return habitat_sim.Configuration(
        sim_cfg, [agent_cfg], render_to_ui=settings["render_to_ui"]
    )


def init_agent_state(sim, navmesh_settings, agent_id):
    sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)

    # initialize the agent at a random start state
    agent = sim.initialize_agent(agent_id)
    start_state = agent.get_state()

    # force starting position on first floor (try 100 samples)
    num_start_tries = 0
    while start_state.position[1] > 0.5 and num_start_tries < 100:
        start_state.position = sim.pathfinder.get_random_navigable_point()
        num_start_tries += 1
    agent.set_state(start_state)

    return start_state


class Viewer(Application):
    def __init__(self):
        configuration = self.Configuration()
        configuration.title = "Magnum Python Triangle Example"
        Application.__init__(self, configuration)

        self.viewport_size = gl.default_framebuffer.viewport.size()

        self.cfg = make_cfg(settings)
        self.sim = habitat_sim.Simulator(self.cfg)
        self.agent_id = settings["default_agent"]
        self.step = -1

        init_agent_state(self.sim, navmesh_settings, self.agent_id)

    def draw_event(self):
        gl.default_framebuffer.clear(
            gl.FramebufferClear.COLOR | gl.FramebufferClear.DEPTH
        )
        obs = self.sim.get_sensor_observations()
        self.swap_buffers()

    def key_press_event(self, event: Application.KeyEvent):
        if event.key == Application.KeyEvent.Key.W:
            self.sim.step("move_forward")
        elif event.key == Application.KeyEvent.Key.UP:
            self.sim.step("look_up")
        elif event.key == Application.KeyEvent.Key.DOWN:
            self.sim.step("look_down")
        elif event.key == Application.KeyEvent.Key.RIGHT:
            self.sim.step("turn_right")
        elif event.key == Application.KeyEvent.Key.LEFT:
            self.sim.step("turn_left")
        elif event.key == Application.KeyEvent.Key.R:
            self.sim.recompute_navmesh(
                self.sim.pathfinder, navmesh_settings, include_static_objects=True
            )
        elif event.key == Application.KeyEvent.Key.N:
            self.sim._sim.toggle_navmesh_visualization()
        else:
            pass

        self.redraw()


try:
    exit(Viewer().exec())
except Exception as e:
    import traceback

    print(str(traceback.format_exc()))
