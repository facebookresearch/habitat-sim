import habitat_sim.bindings as hsim
import habitat_sim
import habitat_sim.utils
import pytest
import os.path as osp


_test_scene = osp.abspath(
    osp.join(osp.dirname(__file__), "17DRP5sb8fy", "17DRP5sb8fy.glb")
)


def make_cfg(settings):
    sim_cfg = hsim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene.id = _test_scene

    # define default sensor parameters (see src/esp/Sensor/Sensor.h)
    sensors = {
        "color_sensor": {  # active if sim_settings["color_sensor"]
            "sensor_type": hsim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "depth_sensor": {  # active if sim_settings["depth_sensor"]
            "sensor_type": hsim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "semantic_sensor": {  # active if sim_settings["semantic_sensor"]
            "sensor_type": hsim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
    }

    # create sensor specifications
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            sensor_spec = hsim.SensorSpec()
            sensor_spec.uuid = sensor_uuid
            sensor_spec.sensor_type = sensor_params["sensor_type"]
            sensor_spec.resolution = sensor_params["resolution"]
            sensor_spec.position = sensor_params["position"]
            sensor_specs.append(sensor_spec)

    # create agent specifications
    agent_cfg = habitat_sim.agent.AgentConfig()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


# Any test globally can take `sim` as an arguement and will get
# the single instance of the Simulator
@pytest.fixture(scope="session")
def sim():
    settings = dict(
        height=480,
        width=640,
        sensor_height=1.5,
        color_sensor=True,
        semantic_sensor=True,
        depth_sensor=True,
    )

    return habitat_sim.Simulator(make_cfg(settings))
