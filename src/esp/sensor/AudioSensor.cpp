// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "AudioSensor.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

void CHECK_AUDIO_FLAG() {
#ifndef ESP_BUILD_WITH_AUDIO
  CORRADE_ASSERT_UNREACHABLE(
      "Audio sensor is not enabled, cannot create Audio Sensor Spec. Build "
      "with flag --audio", );
#endif  // ESP_BUILD_WITH_AUDIO
}

AudioSensorSpec::AudioSensorSpec() : SensorSpec() {
  ESP_DEBUG() << logHeader_ << "AudioSensorSpec constructor";
  uuid = "audio";
  sensorType = SensorType::Audio;
  sensorSubType = SensorSubType::ImpulseResponse;
}

void AudioSensorSpec::sanityCheck() const {
  CHECK_AUDIO_FLAG();
  ESP_DEBUG() << logHeader_ << "SanityCheck the audio sensor spec";
  SensorSpec::sanityCheck();

  CORRADE_ASSERT(sensorType == SensorType::Audio,
                 "AudioSensorSpec::sanityCheck(): sensorType must be Audio", );

  CORRADE_ASSERT(
      sensorSubType == SensorSubType::ImpulseResponse,
      "AudioSensorSpec::sanityCheck(): sensorSubType must be Audio", );
}

AudioSensor::AudioSensor(scene::SceneNode& node, AudioSensorSpec::ptr spec)
    : Sensor{node, std::move(spec)} {
#ifdef ESP_BUILD_WITH_AUDIO
  ESP_DEBUG() << logHeader_ << "AudioSensor constructor";
  audioSensorSpec_->sanityCheck();
  ESP_DEBUG() << logHeader_
              << "Acoustics Configs : " << audioSensorSpec_->acousticsConfig_;
  ESP_DEBUG() << logHeader_
              << "Channel Layout : " << audioSensorSpec_->channelLayout_;

  // If the output directory is not defined, use /home/AudioSimulation
  if (audioSensorSpec_->outputDirectory_.size() == 0) {
    ESP_DEBUG()
        << logHeader_
        << "output directory not provided, will use /home/AudioSimulation";
    audioSensorSpec_->outputDirectory_ = "/home/AudioSimulation";
  }
  ESP_DEBUG() << logHeader_
              << "OutputDirectory : " << audioSensorSpec_->outputDirectory_;
#endif  // ESP_BUILD_WITH_AUDIO
}

AudioSensor::~AudioSensor() {
  CHECK_AUDIO_FLAG();
#ifdef ESP_BUILD_WITH_AUDIO
  ESP_DEBUG() << logHeader_ << "Destroying the audio sensor";
  audioSimulator_ = nullptr;
  impulseResponse_.clear();
#endif  // ESP_BUILD_WITH_AUDIO
}

#ifdef ESP_BUILD_WITH_AUDIO
void AudioSensor::reset() {
  audioSimulator_ = nullptr;
  impulseResponse_.clear();
}

void AudioSensor::setAudioSourceTransform(const vec3f& sourcePos) {
  ESP_DEBUG() << logHeader_
              << "Setting the audio source position : " << sourcePos << "]";
  lastSourcePos_ = sourcePos;
  // track if the source has changed
  newSource_ = true;
}

void AudioSensor::setAudioListenerTransform(const vec3f& agentPos,
                                            const vec4f& agentRotQuat) {
  ESP_DEBUG() << logHeader_ << "Setting the agent transform : position ["
              << agentPos << "], rotQuat[" << agentRotQuat << "]";

  createAudioSimulator();

  CORRADE_ASSERT(audioSimulator_,
                 "setAudioListenerTransform : audioSimulator_ should exist", );

  // If its a new simulation object or the agent position or orientation
  // changed,
  //    add a listener
  if (newInitialization_ || (lastAgentPos_ != agentPos) ||
      !(lastAgentRot_.isApprox(agentRotQuat))) {
    audioSimulator_->AddListener(
        RLRAudioPropagation::Vector3f{agentPos(0), agentPos(1), agentPos(2)},
        RLRAudioPropagation::Quaternion{agentRotQuat(0), agentRotQuat(1),
                                        agentRotQuat(2), agentRotQuat(3)},
        audioSensorSpec_->channelLayout_);
  }
}

void AudioSensor::runSimulation(sim::Simulator& sim) {
  CORRADE_ASSERT(audioSimulator_,
                 "runSimulation: audioSimulator_ should exist", );
  ESP_DEBUG() << logHeader_ << "Running the audio simulator";

  if (newInitialization_) {
    // If its a new initialization, upload the geometry
    newInitialization_ = false;
    ESP_DEBUG() << logHeader_
                << "New initialization, will upload geometry and add the "
                   "source at position : "
                << lastSourcePos_;

    if (audioSensorSpec_->acousticsConfig_.enableMaterials &&
        sim.semanticSceneExists()) {
      ESP_DEBUG() << logHeader_ << "Loading semantic scene";
      loadSemanticMesh(sim);
    } else {
      // load the normal render mesh without any material info
      ESP_DEBUG() << logHeader_
                  << "Semantic scene does not exist or materials are disabled, "
                     "will use default material";
      loadMesh(sim);
    }
  }

  if (newSource_) {
    // [NOTE] Currently, only one source is supported
    // If its a new source, add/replace the audio source.
    // A new initialization should always come with a new source
    // Mark that the source has been added
    newSource_ = false;
    ESP_DEBUG() << logHeader_
                << "Adding source at position : " << lastSourcePos_;
    audioSimulator_->AddSource(RLRAudioPropagation::Vector3f{
        lastSourcePos_(0), lastSourcePos_(1), lastSourcePos_(2)});
  }

  // Run the audio simulation
  const std::string simFolder = getSimulationFolder();
  ESP_DEBUG() << "Running simulation, folder : " << simFolder;
  audioSimulator_->RunSimulation(simFolder);

  // Each time simulation is run, clear the impulse response
  impulseResponse_.clear();
}

void AudioSensor::setAudioMaterialsJSON(const std::string& jsonPath) {
  if (audioMaterialsJsonSet_) {
    // audioMaterialsJsonSet_ is set to true when in loadSemanticMesh.
    // Once the mesh is loaded, we cannot change the materials database. Ignore
    // any new material database files.
    ESP_WARNING() << "[WARNING] Audio material database is already set. Will "
                     "ignore the new file:"
                  << jsonPath;
    return;
  }

  ESP_DEBUG() << "Set audio materials database to json file : " << jsonPath;
  audioMaterialsJSON_ = jsonPath;
}

const std::vector<std::vector<float>>& AudioSensor::getIR() {
  if (impulseResponse_.empty()) {
    ObservationSpace obsSpace;
    getObservationSpace(obsSpace);

    impulseResponse_.resize(obsSpace.shape[0],
                            std::vector<float>(obsSpace.shape[1], 0.0));

    for (std::size_t channelIndex = 0; channelIndex < obsSpace.shape[0];
         ++channelIndex) {
      const float* ir =
          audioSimulator_->GetImpulseResponseForChannel(channelIndex);
      for (std::size_t sampleIndex = 0; sampleIndex < obsSpace.shape[1];
           ++sampleIndex) {
        impulseResponse_[channelIndex][sampleIndex] = ir[sampleIndex];
      }
    }
  }

  return impulseResponse_;
}
#endif  // ESP_BUILD_WITH_AUDIO

bool AudioSensor::getObservation(sim::Simulator&, Observation& obs) {
  CHECK_AUDIO_FLAG();

  // Note : Addressing clang-tidy error for unused param
  obs.buffer = buffer_;

#ifdef ESP_BUILD_WITH_AUDIO
  CORRADE_ASSERT(audioSimulator_,
                 "getObservation : audioSimulator_ should exist", false);

  ObservationSpace obsSpace;
  getObservationSpace(obsSpace);

  if (obsSpace.shape[0] == 0 || obsSpace.shape[1] == 0) {
    ESP_ERROR() << "Channel count or sample count is 0, no IR to return.";
    return false;
  }

  if (buffer_ == nullptr) {
    buffer_ = core::Buffer::create(obsSpace.shape, obsSpace.dataType);
  }

  obs.buffer = buffer_;

  std::uint64_t bufIndex = 0;
  const std::size_t sizeToCopy = sizeof(float) * obsSpace.shape[1];
  // write the simulation output to the observation buffer
  // IR samples are packed per channel
  for (std::size_t channelIndex = 0; channelIndex < obsSpace.shape[0];
       ++channelIndex) {
    const float* ir =
        audioSimulator_->GetImpulseResponseForChannel(channelIndex);
    // Copy the ir for the specific channel into the data buffer
    memcpy(obs.buffer->data + bufIndex, ir, sizeToCopy);
    bufIndex += sizeToCopy;
  }

  if (audioSensorSpec_->acousticsConfig_.writeIrToFile) {
    writeIRFile(obs);
  }
#endif  // ESP_BUILD_WITH_AUDIO
  return true;
}

bool AudioSensor::getObservationSpace(ObservationSpace& obsSpace) {
  CHECK_AUDIO_FLAG();

  // Update the spaceType
  // Note : Called outside the #ifdef to address clang-tidy error
  obsSpace.spaceType = ObservationSpaceType::Tensor;

#ifdef ESP_BUILD_WITH_AUDIO
  CORRADE_ASSERT(audioSimulator_,
                 "getObservationSpace: audioSimulator_ should exist", false);

  // shape is a 2 ints
  //    index 0 = channel count
  //    index 1 = sample count
  obsSpace.shape = {audioSimulator_->GetChannelCount(),
                    audioSimulator_->GetSampleCount()};

  obsSpace.dataType = core::DataType::DT_FLOAT;

  ESP_DEBUG() << logHeader_
              << "getObservationSpace -> [ChannelCount] : " << obsSpace.shape[0]
              << ", [SampleCount] : " << obsSpace.shape[1];

#endif  // ESP_BUILD_WITH_AUDIO
  return true;
}

bool AudioSensor::displayObservation(sim::Simulator&) {
  ESP_ERROR() << logHeader_
              << "Display observation for audio sensor is not used. This "
                 "function should be unreachable";
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  return false;
}

#ifdef ESP_BUILD_WITH_AUDIO
void AudioSensor::createAudioSimulator() {
  ++currentSimCount_;
  ESP_DEBUG() << logHeader_
              << "Create audio simulator iteration: " << currentSimCount_;

  // If the audio simulator already exists, no need to create it
  if (audioSimulator_)
    return;

  newInitialization_ = true;
  audioSimulator_ = std::make_unique<RLRAudioPropagation::Simulator>();
  lastAgentPos_ = {__FLT_MIN__, __FLT_MIN__, __FLT_MIN__};

  audioSimulator_->Configure(audioSensorSpec_->acousticsConfig_);
}

void AudioSensor::loadSemanticMesh(sim::Simulator& sim) {
  ESP_DEBUG() << logHeader_ << "Loading semantic mesh";

  CORRADE_ASSERT(audioSimulator_,
                 "loadSemanticMesh: audioSimulator_ should exist", );

  // Load the audio materials JSON if it was set
  if (!audioMaterialsJsonSet_ && audioMaterialsJSON_.size() > 0) {
    auto errorCode =
        audioSimulator_->LoadAudioMaterialJSON(audioMaterialsJSON_);
    if (errorCode != RLRAudioPropagation::ErrorCodes::Success) {
      ESP_ERROR() << "Audio material json could not be loaded. ErrorCode: "
                  << static_cast<int>(errorCode)
                  << ". Please check if the file exists and the format is "
                     "correct. FilePath:"
                  << audioMaterialsJSON_;
      CORRADE_ASSERT(false, "", );
    }
    audioMaterialsJsonSet_ = true;
  }

  std::vector<std::uint16_t> objectIds;

  sceneMesh_ = sim.getJoinedSemanticMesh(objectIds);

  std::shared_ptr<scene::SemanticScene> semanticScene = sim.getSemanticScene();

  const std::vector<std::shared_ptr<scene::SemanticCategory>>& categories =
      semanticScene->categories();
  const std::vector<std::shared_ptr<scene::SemanticObject>>& objects =
      semanticScene->objects();

  RLRAudioPropagation::VertexData vertices;

  vertices.vertices = sceneMesh_->vbo.data();
  vertices.byteOffset = 0;
  vertices.vertexCount = sceneMesh_->vbo.size();
  vertices.vertexStride = 0;

  // Send the vertex data
  audioSimulator_->LoadMeshVertices(vertices);

  std::unordered_map<std::string, std::vector<uint32_t>> categoryNameToIndices;

  auto& ibo = sceneMesh_->ibo;
  for (std::size_t iboIdx = 0; iboIdx < ibo.size(); iboIdx += 3) {
    // For each index in the ibo
    //  get the object id
    //    get the object using the id
    //      get the category name from the object
    const std::string cat1 =
        objects[objectIds[ibo[iboIdx]]]
            ? objects[objectIds[ibo[iboIdx]]]->category()->name()
            : "default";
    const std::string cat2 =
        objects[objectIds[ibo[iboIdx + 1]]]
            ? objects[objectIds[ibo[iboIdx + 1]]]->category()->name()
            : "default";
    const std::string cat3 =
        objects[objectIds[ibo[iboIdx + 2]]]
            ? objects[objectIds[ibo[iboIdx + 2]]]->category()->name()
            : "default";

    std::string catToUse;

    if (cat1 == cat2 && cat1 == cat3) {
      // If all 3 categories are the same, save the indices to cat1 in the
      // categoryNameToIndices map
      catToUse = cat1;
    } else if (cat1 != cat2 && cat1 != cat3) {
      // If cat1 != 2 and 3
      // then either all 3 are different, or cat1 is different while 2==3
      // Either case, use 1
      // reason : if all 3 are different, we cant determine which one is correct
      // if this is the odd one out, then the triangle is actually of this
      // category
      catToUse = cat1;
    } else if (cat1 == cat2) {
      // if we reach here, cat 1 == 2 but != 3
      // use 3
      catToUse = cat3;
    } else {
      // else 1 == 3 but != 2
      // use 2
      catToUse = cat2;
    }

    categoryNameToIndices[catToUse].push_back(ibo[iboIdx]);
    categoryNameToIndices[catToUse].push_back(ibo[iboIdx + 1]);
    categoryNameToIndices[catToUse].push_back(ibo[iboIdx + 2]);
  }

  std::size_t indicesLoaded = 0;
  std::size_t totalIndicesLoaded = 0;

  // Send indices by category
  for (auto catToIndices : categoryNameToIndices) {
    RLRAudioPropagation::IndexData indices;

    indices.indices = catToIndices.second.data();
    indices.byteOffset = 0;
    indices.indexCount = catToIndices.second.size();

    ++indicesLoaded;
    const bool lastUpdate = (indicesLoaded == categoryNameToIndices.size());

    ESP_DEBUG() << logHeader_ << "Vertex count : " << vertices.vertexCount
                << ", Index count : " << indices.indexCount
                << ", Material : " << catToIndices.first
                << ", LastUpdate : " << lastUpdate;

    // Send all indices for this particular category
    audioSimulator_->LoadMeshIndices(indices, catToIndices.first);

    totalIndicesLoaded += indices.indexCount;
  }

  if (totalIndicesLoaded != sceneMesh_->ibo.size()) {
    ESP_ERROR() << logHeader_
                << "totalIndicesLoaded != sceneMesh_->ibo.size() : ("
                << totalIndicesLoaded << " != " << sceneMesh_->ibo.size()
                << ")";
    CORRADE_ASSERT(false, "totalIndicesLoaded != sceneMesh_->ibo.size()", );
  }

  audioSimulator_->UploadMesh();
}

void AudioSensor::loadMesh(sim::Simulator& sim) {
  ESP_DEBUG() << logHeader_ << "Loading non-semantic mesh";
  CORRADE_ASSERT(audioSimulator_, "loadMesh: audioSimulator_ should exist", );
  sceneMesh_ = sim.getJoinedMesh(true);

  RLRAudioPropagation::VertexData vertices;

  vertices.vertices = sceneMesh_->vbo.data();
  vertices.byteOffset = 0;
  vertices.vertexCount = sceneMesh_->vbo.size();
  vertices.vertexStride = 0;

  RLRAudioPropagation::IndexData indices;

  indices.indices = sceneMesh_->ibo.data();
  indices.byteOffset = 0;
  indices.indexCount = sceneMesh_->ibo.size();

  ESP_DEBUG() << "Vertex count : " << vertices.vertexCount
              << ", Index count : " << indices.indexCount;
  audioSimulator_->LoadMeshData(vertices, indices);
}

std::string AudioSensor::getSimulationFolder() {
  return audioSensorSpec_->outputDirectory_ + std::to_string(currentSimCount_);
}

void AudioSensor::writeIRFile(const Observation& obs) {
  ESP_DEBUG() << logHeader_ << "Write IR samples to file";
  CORRADE_ASSERT(audioSimulator_, "loadMesh: audioSimulator_ should exist", );

  const std::string folderPath = getSimulationFolder();
  std::size_t bufIndex = 0;
  for (std::size_t channelIndex = 0; channelIndex < obs.buffer->shape[0];
       ++channelIndex) {
    std::ofstream file;
    std::string fileName =
        folderPath + "/ir" + std::to_string(channelIndex) + ".txt";
    file.open(fileName);

    for (std::size_t sampleIndex = 0; sampleIndex < obs.buffer->shape[1];
         ++sampleIndex) {
      file << sampleIndex << "\t" << *(float*)(obs.buffer->data + bufIndex)
           << std::endl;
      bufIndex += sizeof(float);
    }

    file.close();
    ESP_DEBUG() << logHeader_ << "File written : " << fileName;
  }
}

#endif  // ESP_BUILD_WITH_AUDIO

}  // namespace sensor
}  // namespace esp
