# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
from numpy import ndarray
import quaternion as qt

import habitat_sim
import habitat_sim.sim
import habitat_sim.sensor
import habitat_sim._ext.habitat_sim_bindings as hsim_bindings

from datetime import datetime

def printTime():
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    print("Current Time =", current_time)

def main():
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = (
        "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb"
    )
    backend_cfg.scene_dataset_config_file = ("data/scene_datasets/mp3d_example/17DRP5sb8fy/scene_dataset_config.json")
    backend_cfg.enable_physics = False

    agent_config = habitat_sim.AgentConfiguration()

    cfg = habitat_sim.Configuration(backend_cfg, [agent_config])

    sim = habitat_sim.Simulator(cfg)

    # create the acoustic configs
    acoustics_config = hsim_bindings.HabitatAcousticsConfiguration()
    acoustics_config.dumpWaveFiles = True
    acoustics_config.enableMaterials = False
    acoustics_config.writeIrToFile = True

    # create channel layout
    channel_layout = hsim_bindings.HabitatAcousticsChannelLayout()
    channel_layout.channelType = hsim_bindings.HabitatAcousticsChannelLayoutType.Binaural
    channel_layout.channelCount = 2

    # create the Audio sensor specs
    audio_sensor_spec = habitat_sim.AudioSensorSpec()
    audio_sensor_spec.uuid = "audio_sensor"
    audio_sensor_spec.outputDirectory = "/home/sangarg/AudioSimulation"
    audio_sensor_spec.acousticsConfig = acoustics_config
    audio_sensor_spec.channelLayout = channel_layout

    # add the audio sensor
    sim.add_sensor(audio_sensor_spec)

    # Get the audio sensor object
    audio_sensor = sim.get_agent(0)._sensors["audio_sensor"]

    # set audio source location, no need to set the agent location, will be set implicitly
    audio_sensor.setAudioSourceTransform(np.array([3.1035, 1.57245, -4.15972]))

    # run the simulation
    for i in range (1):
        print(i)
        print("Start Time : ")
        printTime()
        obs = sim.get_sensor_observations()["audio_sensor"]

        # print the audio observations
        print (obs)

        print("End Time : ")
        printTime()

    sim.close()

if __name__ == "__main__":
    main()
