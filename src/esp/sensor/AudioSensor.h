// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <memory>
#include <vector>
#include <cstdint>
#include <fstream>

#include "esp/core/Esp.h"
#include "esp/sensor/Sensor.h"
#include "esp/assets/MeshData.h"
#include "esp/scene/SemanticScene.h"

// todo sangarg : Cleanup header include. Not sure why we need the full path
#include "audio/HabitatAcousticsPkg/headers/HabitatAcoustics.h"

#ifndef ESP_SENSOR_AUDIOSENSOR_H_
#define ESP_SENSOR_AUDIOSENSOR_H_

namespace esp {
namespace sensor {

struct AudioSensorSpec : public SensorSpec {
 public:
  AudioSensorSpec();
  void sanityCheck() const override;
  bool isVisualSensorSpec() const override { return false; }

 public:
  // Data
  HabitatAcoustics::Configuration acousticsConfig_;
  HabitatAcoustics::ChannelLayout channelLayout_;
  std::string outputDirectory_;

 public:
  ESP_SMART_POINTERS(AudioSensorSpec)

 private:
  const std::string logHeader_ = "[Audio] ";
};

class AudioSensor : public Sensor {
 public:
  // Constructor
  explicit AudioSensor(scene::SceneNode& node, AudioSensorSpec::ptr spec);
  // Destructor
  virtual ~AudioSensor();

  /**
   * @brief Return that this is not a visual sensor
   */
  bool isVisualSensor() const override { return false; }

  // functions to implement the audio sensor

  /**
   * @brief Clear the audio simulator object
   * */
  void reset();

  /**
   * @brief Set the audio source transform
   * @param sourcePos = vec3 source position
   * */
  void setAudioSourceTransform(const vec3f& sourcePos);

  /**
   * @brief Set the audio listener position and orientation
   * @param agentPos = vec3 agent position
   * @param agentRotQuat = vec4 agent rotation quaternion
   * */
  void setAudioListenerTransform(const vec3f& agentPos, const vec4f& agentRotQuat);

  /**
   * @brief Run the audio simulation. This will run the HabitatAcoustics code to get the impulse response
   * */
  void runSimulation(sim::Simulator& sim);

 // Sensor class overrides
 public:
  bool getObservation(sim::Simulator& sim, Observation& obs) override;
  bool getObservationSpace(ObservationSpace& obsSpace) override;
 private:
  bool displayObservation(sim::Simulator& sim) override;

private:
  /**
   * @brief Create the audio simulator object
   * */
  void createAudioSimulator();

  /**
   * @brief Load the semantic mesh for the Simulator object
   * */
  void loadSemanticMesh(sim::Simulator& sim);

  /**
   * @brief Load the non-sematic mesh for the Simulator object
   * */
  void loadMesh(sim::Simulator& sim);

  /**
   * @brief Get the simulation folder path.
   * This path is based on the AudioSensorSpec->outputDirectoryPrefix and the current simulation count
   * */
  std::string getSimulationFolder();

  /**
   * @brief Dump the impulse response to a file
   * */
  void writeIRFile(const Observation& obs);

private:
  AudioSensorSpec::ptr audioSensorSpec_ =
      std::dynamic_pointer_cast<AudioSensorSpec>(spec_);
  std::unique_ptr<HabitatAcoustics::Simulator> audioSimulator_ = nullptr;
  esp::assets::MeshData::ptr sceneMesh_;

  std::int32_t currentSimCount_ = -1; // track the number of simulations
  vec3f lastSourcePos_; // track the source position
  vec3f lastAgentPos_; // track the agent orientation
  vec4f lastAgentRot_; // track the agent rotation

  bool newInitialization_ = false;
  bool newSource_ = false;

  const std::string logHeader_ = "[Audio] ";

 public:
  ESP_SMART_POINTERS(AudioSensor)
};  // class AudioSensor

} // namespace sensor
} // namespace esp

#endif  // ESP_SENSOR_AUDIOSENSOR_H_
