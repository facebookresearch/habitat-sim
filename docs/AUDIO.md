# Audio Sensor

The C++ Audio sensor class uses the [RLRAudioPropagation](https://github.com/facebookresearch/rlr-audio-propagation) library. This is a bi-directional ray tracer based audio simulator. Given a source location, listener location, mesh, audio materials and some parameters will simulate how audio waves travel from the source to the listener. The output of the library is an impulse response for the given setup. 

The C++ implementation is exposed for python user via py-bindings. This document explains the various python apis, structs and enums.



## List of sections
- [Acoustics configuration - RLRAudioPropagationConfiguration()](#acoustics-configuration)
- [Channel layout - RLRAudioPropagationChannelLayout()](#channel-layout)
- [Audio sensor specs - AudioSensorSpec()](#audio-sensor-specs)
- [APIs](#apis)
- [Steps to run audio simulation in python](#steps-to-run-audio-simulation-in-python)



### - Acoustics configuration

The RLRAudioPropagationConfiguration() exposes various audio configuration that can be used to modify the audio simulation. This section lists all the available configs, their data type, default values and explains what the config does


|Config name|Data Type|Default Value|Usage|
|-----------|---------|-------------|-----|
| sampleRate | int | 44100 | Sample rate for the simulated audio |
| frequencyBands | int |  4 | Number of frequency bands in the simulated audio simulation |
| directSHOrder | int | 3 | todo sangarg : Talk to Carl |
| indirectSHOrder | int | 1 |  todo sangarg : Talk to Carl  |
| threadCount | int | 1 | Number of thread the audio simulation will use |
| updateDt | float | 0.02f | Simulation time step the audio simulation will use |
| irTime | float | 4.f | Total render time for the audio simulation |
| unitScale | float | 1.f | Unit scale for the scene, mesh and positions are multiplied by this factor |
| globalVolume | float | 4.f | Total initial pressure value |
| indirectRayCount | int | 5000 | Number of indirect rays that the ray tracer will use |
| indirectRayDepth | int | 200 | Depth of each indirect ray that the ray tracer will use |
| sourceRayCount | int | 200 |  Number of direct rays that the ray tracer will use |
| sourceRayDepth | int | 10 |  Depth of each direct ray that the ray tracer will use |
| maxDiffractionOrder | int | 10 | todo sagnarg : Talk to Carl |
| direct | bool | true | Enable contribution from the direct rays |
| indirect | bool | true | Enable contribution from the indirect rays |
| diffraction | bool | true | Enable diffraction for the simulation |
| transmission | bool | false | Enable transmission of rays |
| meshSimplification | bool | false | Simplify mesh before running the audio simulation. todo sangarg : Add which simplification algo is used|
| temporalCoherence | bool | false | todo sangarg : Talk to Carl |
| dumpWaveFiles | bool | false | Write the wave files for different bands. Will be writted to the AudioSensorSpec's outputDirectory |
| enableMaterials | bool | true | Enable audio materials |
| writeIrToFile | bool | false | Write the final impulse response to a file |



### - Channel layout

This section talks about the channel layout struct. The channel layout defines what the output will look like.

|Config name|Data Type|Default Value|Usage|
|-----------|---------|-------------|-----|
| channelType | enum | [RLRAudioPropagationChannelLayoutType](#RLRAudioPropagationChannelLayoutType).Binaural | Channel type for the simulated audio |
| channelCount | int |  2 | Number of output channels in simulated audio |



- ##### RLRAudioPropagationChannelLayoutType

Lets look at the available values for RLRAudioPropagationChannelLayoutType

|Enum|Usage|
|-----------|---------|
Unknown | Unknown channel layout type |
Mono | Monaural channel layout that does not have any spatial information. This layout usually has 1 channel |
Stereo | Channel layout with 2 channels (e.g. speakers) that does not use any HRTF |
Binaural | Channel layout with 2 channels that spatializes audio using an HRTF |
Quad | Channel layout with 4 channels (speakers) arranged at +-30 and +-95 degrees in the horizontal plane |
Surround_5_1 | Channel layout with 6 channels (speakers) arranged at 0, +-30, and +-110 degrees in the horizontal plane, with unpositioned low frequency channel |
Surround_7_1 | Channel layout with 8 channels (speakers) arranged at 0, +-30, +-90, and +-135 degrees in the horizontal plane, with unpositioned low frequency channel |
Ambisonics | Channel layout that encodes fully spherical spatial audio as a set of spherical harmonic basis function coefficients |


### - Audio sensor specs

|Config name|Data Type|Default Value|Usage|
|-----------|---------|-------------|-----|
|uuid|string|""|uuid to identify the sensor object|
| outputDirectory | string | "" | Output directory prefix for the simulation. Folders with outputDirectory + i should be created if you want to dump the wave files. (i = 0 indexed simulation iteration |
| acousticsConfig | [RLRAudioPropagationConfiguration()](#acoustics-configuration) |  Defined in the relevant section | Acoustic configuration struct that defines simulation parameters |
| channelLayout | [RLRAudioPropagationChannelLayout()](#channel-layout) |  Defined in the relevant section | Channel layout for simulated output audio |



### - APIs

The audio sensor is build in C++ and exposed on pythom via py-bindings. Import the following to get access to the audio sensor


|python imports|
|------|
|import habitat_sim|
|import habitat_sim._ext.habitat_sim_bindings as hsim_bindings|
|import habitat_sim.sensor|
|import habitat_sim.sim|


The various structs and enums can be accessed as part of the hsim_bindings.

|struct/enum in hsim_bindings|notes|
|------|-----|
|hsim_bindings.RLRAudioPropagationConfiguration() | acoustic configs struct |
|hsim_bindings.RLRAudioPropagationChannelLayout() | channel layout struct |
|hsim_bindings.RLRAudioPropagationChannelLayoutType.Binaural | channel type enum |

The acoustic sensor spec is part of habitat_sim

|struct/enum in habitat_sim|notes|
|------|-----|
|habitat_sim.AudioSensorSpec() | acoustic sensor spec |


To call apis on the audio sensor, get access to the audio sensor object using the uuid.

|apis for audio_sensor|notes|
|------|-----|
|audio_sensor = sim.get_agent(0)._sensors["audio_sensor"]| get the audio sensor object from the habitat sim object|
| audio_sensor.setAudioSourceTransform(np.array([x, y, z])) | set the audio source location where x,y,z are floats|
|audio_sensor.reset() | User can reset the simulation object and start fresh. This is same as deleting the audio sensor.|

Relevant apis on the sim object

|apis for the habitat sim object|notes|
|------|-----|
| sim.add_sensor(audio_sensor_spec)| Add the audio sensor. This is similar to adding any other sensors|
|obs = sim.get_sensor_observations()["audio_sensor"]|Get the impulse response. obs is a n-d array where n = channel count|



### - Steps to run audio simulation in python

Please see the [audio_agent.py](https://github.com/facebookresearch/habitat-sim/tree/main/examples/tutorials/audio_agent.py) for a sample script on how to use the python audio sensor. Follow the steps and refer to the python script for the code.

1. Create the habitat sim object and configuration.
1. Create the Acoustic configuration ([RLRAudioPropagationConfiguration](#acoustics-configuration)) object. Set the various simulation parameters.
1. Create the channel layout ([RLRAudioPropagationChannelLayout](#channel-layout)) object. Set the channel layout
1. Create the [AudioSensorSpec()](#audio-sensor-specs). Assign the acoustic configuration and the channel layout.
1. Add the audio sensor spec to the simulation. This will create th e C++ AudioSensor object. 
1. Get the audio_sensor object from the list of sensors on the agent. (see examples/tutorials/audio_agent.py script). The identifier is set under AudioSensorSpec -> uuid config.
1. Set the location of the audio source by calling audio_sensor.setAudioSourceTransform
1. Run the simulation step and get the audio sensor output sim.get_sensor_observations()["audio_sensor"]. Use the uuid defined. The output is a n-d array of floats where n is the channel count defined in RLRAudioPropagationChannelLayout

