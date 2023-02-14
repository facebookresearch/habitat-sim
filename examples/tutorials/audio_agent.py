# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from datetime import datetime

import numpy as np
import quaternion  # noqa: F401
from numpy import ndarray  # noqa: F401

import habitat_sim
import habitat_sim.sensor
import habitat_sim.sim


def printTime():
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    print("Current Time =", current_time)


def main():
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "17DRP5sb8fy.glb"
    backend_cfg.scene_dataset_config_file = (
        "data/scene_datasets/mp3d_example/mp3d.scene_dataset_config.json"
    )
    backend_cfg.load_semantic_mesh = True
    backend_cfg.enable_physics = False

    agent_config = habitat_sim.AgentConfiguration()

    cfg = habitat_sim.Configuration(backend_cfg, [agent_config])

    with habitat_sim.Simulator(cfg) as sim:
        # create the acoustic configs
        acoustics_config = habitat_sim.sensor.RLRAudioPropagationConfiguration()
        acoustics_config.enableMaterials = True

        # create channel layout
        channel_layout = habitat_sim.sensor.RLRAudioPropagationChannelLayout()
        channel_layout.channelType = (
            habitat_sim.sensor.RLRAudioPropagationChannelLayoutType.Binaural
        )
        channel_layout.channelCount = 2

        # create the Audio sensor specs, assign the acoustics_config and the channel_layout.
        # note that the outputDirectory should already exist for each iteration step.
        # for the example below, folders /home/AudioSimulation0, /home/AudioSimulation1 ... should
        # exist based on the number of iterations
        audio_sensor_spec = habitat_sim.AudioSensorSpec()
        audio_sensor_spec.uuid = "audio_sensor"
        audio_sensor_spec.outputDirectory = "/tmp/AudioSimulation"
        audio_sensor_spec.acousticsConfig = acoustics_config
        audio_sensor_spec.channelLayout = channel_layout

        # add the audio sensor
        sim.add_sensor(audio_sensor_spec)

        # Get the audio sensor object
        audio_sensor = sim.get_agent(0)._sensors["audio_sensor"]

        # set audio source location, no need to set the agent location, will be set implicitly
        audio_sensor.setAudioSourceTransform(np.array([3.1035, 1.57245, -4.15972]))

        # optionally, set the audio materials json
        audio_sensor.setAudioMaterialsJSON(
            "src/deps/rlr-audio-propagation/RLRAudioPropagationPkg/data/mp3d_material_config.json"
        )

        # run the simulation, currently only 1 iteration is run
        for i in range(1):
            print(i)
            print("Start Time : ")
            printTime()
            obs = sim.get_sensor_observations()["audio_sensor"]

            # optional - print the audio observations or write them to the desired location
            print(obs)

            # optional - write the observations to a file, make sure the folder path p (below) exists
            # p = audio_sensor_spec.outputDirectory + str(i) + "/ir"

            # for channelIndex in range(0, len(obs)):
            #     filePath = p + str(channelIndex) + ".txt"
            #     f = open(filePath, "w")
            #     print("Writing file : ", filePath)
            #     for sampleIndex in range(0, len(obs[channelIndex])):
            #         f.write(
            #             str(sampleIndex) + "\t" + str(obs[channelIndex][sampleIndex]) + "\n"
            #         )
            #     f.close()

            print("End Time : ")
            printTime()


if __name__ == "__main__":
    main()
