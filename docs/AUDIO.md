# Audio Sensor

The C++ Audio sensor class uses the [RLRAudioPropagation](https://github.com/facebookresearch/rlr-audio-propagation) library. This is a bi-directional ray tracer based audio simulator. Given a source location, listener location, scene geometry (3D mesh), audio materials, and some parameters (described below) it will simulate how audio waves travel from the source to arrive at the listener. The output of this simulation is an impulse response for the listener.

The C++ implementation is exposed for python users via pybind11. This document explains the various python APIs, structs, and enums. Also see the relevant [Habitat-sim python API](https://aihabitat.org/docs/habitat-sim/classes.html) doc pages.

Please refer to the [BUILD_FROM_SOURCE](https://github.com/facebookresearch/habitat-sim/blob/main/BUILD_FROM_SOURCE.md) doc for information on how to build habitat with the audio sensor. This feature was added as part of the following PR : https://github.com/facebookresearch/habitat-sim/pull/1646

## List of sections
- [Acoustics configuration - RLRAudioPropagationConfiguration()](#acoustics-configuration)
- [Channel layout - RLRAudioPropagationChannelLayout()](#channel-layout)
- [Audio sensor specs - AudioSensorSpec()](#audio-sensor-specs)
- [APIs](#apis)
- [Steps to run audio simulation in python](#steps-to-run-audio-simulation-in-python)



### - Acoustics configuration

The RLRAudioPropagationConfiguration() exposes various configuration options that can be used to customize the audio simulation. This section describes the available config settings including data types and default values.


|Config name|Data Type|Default Value|Usage|
|-----------|---------|-------------|-----|
| sampleRate | int | 44100 | Sample rate for the simulated audio |
| frequencyBands | int |  4 | Number of frequency bands in the audio simulation |
| directSHOrder | int | 3 | The spherical harmonic order used for calculating direct sound spatialization for non-point sources (those with non-zero radii). It is not recommended to go above order 9. |
| indirectSHOrder | int | 1 |  The spherical harmonic order used for calculating the spatialization of indirect sound (reflections, reverb). It is not recommended to go above order 5. Increasing this value requires more rays to be traced for the results to converge properly, and uses substantially more memory (scales quadratically).  |
| threadCount | int | 1 | Number of CPU thread the simulation will use |
| updateDt | float | 0.02f | Simulation time step |
| irTime | float | 4.f | Maximum render time budget for the audio simulation |
| unitScale | float | 1.f | Unit scale for the scene. Mesh and positions are multiplied by this factor |
| globalVolume | float | 0.25f | Total initial pressure value |
| indirectRayCount | int | 5000 | Number of indirect rays that the ray tracer will use |
| indirectRayDepth | int | 200 | Maximum depth of each indirect ray cast by the ray tracer |
| sourceRayCount | int | 200 | Number of direct rays that the ray tracer will use |
| sourceRayDepth | int | 10 | Maximum depth of direct rays cast by the ray tracer |
| maxDiffractionOrder | int | 10 | The maximum number of edge diffraction events that can occur between a source and listener. This value cannot exceed 10 (compile-time limit). |
| direct | bool | true | Enable contribution from the direct rays |
| indirect | bool | true | Enable contribution from the indirect rays |
| diffraction | bool | true | Enable diffraction for the simulation |
| transmission | bool | false | Enable transmission of rays |
| meshSimplification | bool | false | Uses a series of mesh simplification operations to reduce the mesh complexity for ray tracing. Vertex welding is applied, followed by simplification using the edge collapse algorithm. |
| temporalCoherence | bool | false | Turn on/off temporal smoothing of the impulse response. This uses the impulse response from the previous simulation time step as a starting point for the next time step. This reduces the number of rays required by about a factor of 10, resulting in faster simulations, but should not be used if the motion of sources/listeners is not continuous. |
| dumpWaveFiles | bool | false | Write the wave files for different bands. Will be writted to the AudioSensorSpec's [outputDirectory](#outputDirectory) |
| enableMaterials | bool | true | Enable audio materials |
| writeIrToFile | bool | false | Write the final impulse response to a file. Will be writted to the AudioSensorSpec's [outputDirectory](#outputDirectory) |



### - Channel layout

This section describes the channel layout struct, which defines what the output will look like.

|Config name|Data Type|Default Value|Usage|
|-----------|---------|-------------|-----|
| channelType | enum | [RLRAudioPropagationChannelLayoutType](#RLRAudioPropagationChannelLayoutType).Binaural | Channel type for the simulated audio |
| channelCount | int |  2 | Number of output channels in simulated audio |



- ##### RLRAudioPropagationChannelLayoutType

The channel layout describes how the audio output will be experienced by the listener. Lets look at channel layout types that are currently supported.

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
|uuid|string|""|unique identifier string to name and refer to this sensor object|
| outputDirectory(#outputDirectory) | string | "" | Output directory prefix for the simulation. Folders with outputDirectory + i should be created if you want to dump the wave files. (i = 0 indexed simulation iteration |
| acousticsConfig | [RLRAudioPropagationConfiguration()](#acoustics-configuration) |  Defined in the relevant section | Acoustic configuration struct that defines simulation parameters |
| channelLayout | [RLRAudioPropagationChannelLayout()](#channel-layout) |  Defined in the relevant section | Channel layout for simulated output audio |



### - APIs

The audio sensor is implemented in C++ and exposed to python via pybind11. Import the following to get access to the audio sensor:

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


To call APIs on the audio sensor, get access to the audio sensor object using the uuid.

|APIs for audio_sensor|notes|
|------|-----|
|audio_sensor = sim.get_agent(0)._sensors["audio_sensor"]| get the audio sensor object from the habitat sim object|
| audio_sensor.setAudioSourceTransform(np.array([x, y, z])) | set the audio source location where x,y,z are floats|
|audio_sensor.reset() | Reset the simulation object to restart from a fresh context. This is the same as deleting the audio sensor and re-creating it.|

Relevant APIs on the Simulator object

|APIs for the Habitat Simulator object|notes|
|------|-----|
| sim.add_sensor(audio_sensor_spec)| Add the audio sensor. This is similar to adding any other sensors|
|obs = sim.get_sensor_observations()["audio_sensor"]|Get the impulse response. obs is a n-d array where n = channel count|



### - Steps to run audio simulation in python

Please see the [audio_agent.py](https://github.com/facebookresearch/habitat-sim/tree/main/examples/tutorials/audio_agent.py) for an example of how to use the python audio sensor. Follow these steps and refer to the python script for the code.

1. Create the habitat sim object and configuration.
1. Create the Acoustic configuration ([RLRAudioPropagationConfiguration](#acoustics-configuration)) object. Set the various simulation parameters.
1. Create the channel layout ([RLRAudioPropagationChannelLayout](#channel-layout)) object. Set the channel layout
1. Create the [AudioSensorSpec()](#audio-sensor-specs). Assign the acoustic configuration and the channel layout.
1. Add the audio sensor spec to the simulation. This will create the C++ AudioSensor object.
1. Get the audio_sensor object from the list of sensors on the agent. (see examples/tutorials/audio_agent.py script). The identifier is set under AudioSensorSpec -> uuid config.
1. Set the location of the audio source by calling audio_sensor.setAudioSourceTransform
1. Run the simulation step and get the audio sensor output sim.get_sensor_observations()["audio_sensor"]. Use the uuid defined. The output is a n-d array of floats where n is the channel count defined in RLRAudioPropagationChannelLayout
