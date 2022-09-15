// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <cstdint>
#include <fstream>
#include <memory>
#include <vector>

#include "esp/assets/MeshData.h"
#include "esp/core/Esp.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/Sensor.h"

#ifdef ESP_BUILD_WITH_AUDIO
#include "rlr-audio-propagation/RLRAudioPropagationPkg/headers/RLRAudioPropagation.h"
#else
#include "esp/sensor/AudioSensorStubs.h"
#endif  // ESP_BUILD_WITH_AUDIO

#ifndef ESP_SENSOR_AUDIOSENSOR_H_
#define ESP_SENSOR_AUDIOSENSOR_H_

namespace esp {
namespace sensor {

struct AudioSensorSpec : public SensorSpec {
 public:
  AudioSensorSpec();
  void sanityCheck() const override;
  bool isVisualSensorSpec() const override { return false; }

#ifdef ESP_BUILD_WITH_AUDIO
 public:
  // Data
  RLRAudioPropagation::Configuration acousticsConfig_;
  RLRAudioPropagation::ChannelLayout channelLayout_;
  std::string outputDirectory_;
#endif  // ESP_BUILD_WITH_AUDIO

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
  ~AudioSensor() override;

  /**
   * @brief Return that this is not a visual sensor
   */
  bool isVisualSensor() const override { return false; }

  // ------- functions to implement the audio sensor ------

#ifdef ESP_BUILD_WITH_AUDIO
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
  void setAudioListenerTransform(const vec3f& agentPos,
                                 const vec4f& agentRotQuat);

  /**
   * @brief Run the audio simulation. This will run the RLRAudioPropagation code
   * to get the impulse response
   * */
  void runSimulation(sim::Simulator& sim);

  /**
   * @brief Set the audio materials database from a json file
   * */
  void setAudioMaterialsJSON(const std::string& jsonPath);

  /**
   * @brief Return the last impulse response.
   * */
  const std::vector<std::vector<float>>& getIR();
#endif  // ESP_BUILD_WITH_AUDIO

  // ------ Sensor class overrides ------
 public:
  bool getObservation(sim::Simulator&, Observation& obs) override;
  bool getObservationSpace(ObservationSpace& obsSpace) override;

 private:
  bool displayObservation(sim::Simulator&) override;

#ifdef ESP_BUILD_WITH_AUDIO
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
   * This path is based on the AudioSensorSpec->outputDirectoryPrefix and the
   * current simulation count
   * */
  std::string getSimulationFolder();

  /**
   * @brief Dump the impulse response to a file
   * */
  void writeIRFile(const Observation& obs);
#endif  // ESP_BUILD_WITH_AUDIO

 private:
  AudioSensorSpec::ptr audioSensorSpec_ =
      std::dynamic_pointer_cast<AudioSensorSpec>(spec_);

#ifdef ESP_BUILD_WITH_AUDIO
  std::unique_ptr<RLRAudioPropagation::Simulator> audioSimulator_ = nullptr;
#endif  // ESP_BUILD_WITH_AUDIO

  esp::assets::MeshData::ptr sceneMesh_;

  //! track the number of simulations
  std::int32_t currentSimCount_ = -1;
  //! track the source position
  vec3f lastSourcePos_;
  //! track the agent orientation
  vec3f lastAgentPos_;
  //! track the agent rotation
  vec4f lastAgentRot_;
  //! audio materials json path
  std::string audioMaterialsJSON_;

  bool audioMaterialsJsonSet_ = false;
  bool newInitialization_ = false;
  bool newSource_ = false;

  const std::string logHeader_ = "[Audio] ";

  std::vector<std::vector<float>> impulseResponse_;

 public:
  ESP_SMART_POINTERS(AudioSensor)
};  // class AudioSensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_AUDIOSENSOR_H_
