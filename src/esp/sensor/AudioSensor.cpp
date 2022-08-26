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

#ifdef ESP_BUILD_WITH_AUDIO
  // Initialize default configuration.
  acousticsConfig_.thisSize = sizeof(RLRA_ContextConfiguration);
  RLRA_ContextConfigurationDefault( &acousticsConfig_ );

  channelLayout_.type = RLRA_ChannelLayoutType_Binaural;
  channelLayout_.channelCount = 2;
  enableMaterials_ = false;
#endif // ESP_BUILD_WITH_AUDIO
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
#endif  // ESP_BUILD_WITH_AUDIO
}

AudioSensor::~AudioSensor() {
  CHECK_AUDIO_FLAG();
#ifdef ESP_BUILD_WITH_AUDIO
  ESP_DEBUG() << logHeader_ << "Destroying the audio sensor";
  if ( context )
  {
    RLRA_DestroyContext( context );
    context = nullptr;
  }
  impulseResponse_.clear();
#endif  // ESP_BUILD_WITH_AUDIO
}

#ifdef ESP_BUILD_WITH_AUDIO
void AudioSensor::reset() {
  ESP_DEBUG() << logHeader_ << "Resetting the audio sensor";
  if ( context )
  {
    RLRA_DestroyContext( context );
    context = nullptr;
  }
  impulseResponse_.clear();
}

void AudioSensor::setAudioSourceTransform(const vec3f& sourcePos) {
  ESP_DEBUG() << logHeader_
              << "Setting the audio source position : " << sourcePos << "]";

  // Make sure the context is created.
  createAudioSimulator();
  CORRADE_ASSERT(context, "setAudioSourceTransform : context should exist", );

  const float pos[3] = { sourcePos[0], sourcePos[1], sourcePos[2] };
  if ( RLRA_SetSourcePosition( context, 0, pos ) != RLRA_Success )
  {
    ESP_ERROR() << "Error setting audio source position";
    return;
  }
}

void AudioSensor::setAudioListenerTransform(const vec3f& agentPos,
                                            const vec4f& agentRotQuat) {
  ESP_DEBUG() << logHeader_ << "Setting the agent transform : position ["
              << agentPos << "], rotQuat[" << agentRotQuat << "]";

  // Make sure the context is created.
  createAudioSimulator();
  CORRADE_ASSERT(context, "setAudioListenerTransform : context should exist", );

  const float pos[3] = { agentPos[0], agentPos[1], agentPos[2] };
  if ( RLRA_SetListenerPosition(context, 0, pos) != RLRA_Success )
  {
    ESP_ERROR() << "Error setting agent position";
    return;
  }

  const float rot[4] = { agentRotQuat[0], agentRotQuat[1], agentRotQuat[2],
      agentRotQuat[3] };
  if ( RLRA_SetListenerOrientationQuaternion(context, 0, rot) != RLRA_Success )
  {
    ESP_ERROR() << "Error setting agent rotation";
    return;
  }
}

void AudioSensor::runSimulation(sim::Simulator& sim) {
  CORRADE_ASSERT(context, "runSimulation: context should exist", );
  ESP_DEBUG() << logHeader_ << "Running the audio simulator";

  if (newInitialization_) {
    // If its a new initialization, upload the geometry
    newInitialization_ = false;
    ESP_DEBUG() << logHeader_
                << "New initialization, will upload geometry";

    if (audioSensorSpec_->enableMaterials_ && sim.semanticSceneExists()) {
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

  // Run the audio simulation
  if ( RLRA_Simulate( context ) != RLRA_Success ) {
    ESP_ERROR() << "Error while running audio simulation";
  }

  // Each time simulation is run, clear the impulse response
  impulseResponse_.clear();
}

void AudioSensor::setAudioMaterialsJSON(const std::string& jsonPath) {
  if (audioMaterialsJsonSet_) {
    // audioMaterialsJsonSet_ is set to true when in loadSemanticMesh.
    // Once the mesh is loaded, we cannot change the materials database. Ignore
    // any new material database files.
    ESP_WARNING() << "[WARNING] Audio material database is already set. Will "
                     "ignore the new file:" << jsonPath;
    return;
  }

  ESP_DEBUG() << "Set audio materials database to json file : " << jsonPath;
  audioMaterialsJSON_ = jsonPath;
}

void AudioSensor::setListenerHRTF(const std::string& hrtfPath) {
  // Make sure the context is created.
  createAudioSimulator();
  CORRADE_ASSERT(context, "setListenerHRTF: context should exist", );
  ESP_DEBUG() << "Set listener HRTF to file : " << hrtfPath;

  if ( RLRA_SetListenerHRTF( context, 0, hrtfPath.c_str() ) != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't load custom audio listener HRTF";
    return;
  }
}

float AudioSensor::getRayEfficiency() const {
  CORRADE_ASSERT(context, "getRayEfficiency: context should exist", 0.0f );
  return RLRA_GetIndirectRayEfficiency( context );
}

const std::vector<std::vector<float>>& AudioSensor::getIR() {
  CORRADE_ASSERT(context, "getIR: context should exist", impulseResponse_ );

  if (impulseResponse_.empty()) {
    ObservationSpace obsSpace;
    getObservationSpace(obsSpace);

    impulseResponse_.resize(obsSpace.shape[0],
                            std::vector<float>(obsSpace.shape[1], 0.0));

    for (std::size_t channelIndex = 0; channelIndex < obsSpace.shape[0];
         ++channelIndex) {
      const float* ir = RLRA_GetIRChannel(context, 0, 0, channelIndex);
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
#ifdef ESP_BUILD_WITH_AUDIO
  CORRADE_ASSERT(context, "getObservation : context should exist", false);

  // Get observation dimensions.
  ObservationSpace obsSpace;
  getObservationSpace(obsSpace);
  if (obsSpace.shape[0] == 0 || obsSpace.shape[1] == 0) {
    ESP_ERROR() << "Channel count or sample count is 0, no IR to return.";
    return false;
  }

  // Create new buffer to store observation.
  obs.buffer = core::Buffer::create(obsSpace.shape, obsSpace.dataType);

  // Copy the IR samples to the observation buffer.
  std::uint64_t bufIndex = 0;
  const std::size_t sizeToCopy = sizeof(float) * obsSpace.shape[1];
  for (std::size_t channelIndex = 0; channelIndex < obsSpace.shape[0];
       ++channelIndex) {
    const float* ir = RLRA_GetIRChannel(context, 0, 0, channelIndex);
    // Copy the ir for the specific channel into the data buffer
    memcpy(obs.buffer->data + bufIndex, ir, sizeToCopy);
    bufIndex += sizeToCopy;
  }

#else // ESP_BUILD_WITH_AUDIO
  // Note : Addressing clang-tidy error for unused param
  (void)obs;
#endif // ESP_BUILD_WITH_AUDIO
  return true;
}

bool AudioSensor::getObservationSpace(ObservationSpace& obsSpace) {
  CHECK_AUDIO_FLAG();
#ifdef ESP_BUILD_WITH_AUDIO
  CORRADE_ASSERT(context,"getObservationSpace: context should exist", false);

  // shape is a 2 ints
  //    index 0 = channel count
  //    index 1 = sample count
  obsSpace.shape = {RLRA_GetIRChannelCount(context, 0, 0),
                    RLRA_GetIRSampleCount(context, 0, 0)};
  obsSpace.dataType = core::DataType::DT_FLOAT;
  obsSpace.spaceType = ObservationSpaceType::Tensor;
#else // ESP_BUILD_WITH_AUDIO
  // Note : Addressing clang-tidy error for unused param
  (void)obsSpace;
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
  ESP_DEBUG() << logHeader_ << "Create audio simulator";

  // If the audio simulator already exists, no need to create it
  if (context)
    return;

  newInitialization_ = true;

  // Create context.
  if ( RLRA_CreateContext( &context, &audioSensorSpec_->acousticsConfig_ )
        != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't create audio context";
    return;
  }

  // Create listener.
  if ( RLRA_AddListener( context, &audioSensorSpec_->channelLayout_ )
        != RLRA_Success )
  {
    RLRA_DestroyContext( context );
    context = nullptr;
    ESP_ERROR() << "Couldn't add audio listener";
    return;
  }

  // Create source.
  if ( RLRA_AddSource( context ) != RLRA_Success )
  {
    RLRA_DestroyContext( context );
    context = nullptr;
    ESP_ERROR() << "Couldn't add audio source";
    return;
  }
}

void AudioSensor::loadSemanticMesh(sim::Simulator& sim) {
  ESP_DEBUG() << logHeader_ << "Loading semantic mesh" << sim.semanticSceneExists();

  CORRADE_ASSERT(context, "loadSemanticMesh: context should exist", );

  // Load the audio materials JSON if it was set
  if (!audioMaterialsJsonSet_ && audioMaterialsJSON_.size() > 0) {
    RLRA_Error error = RLRA_SetMaterialDatabaseJSON(context,
        audioMaterialsJSON_.c_str());
    if (error != RLRA_Success) {
      ESP_ERROR() << "Audio material json could not be loaded. ErrorCode: "
                  << static_cast<int>(error)
                  << ". Please check if the file exists and the format is "
                     "correct. FilePath:" << audioMaterialsJSON_;
      CORRADE_ASSERT(false, "", );
    }
    audioMaterialsJsonSet_ = true;
  }

  std::vector<std::uint16_t> objectIds;
  esp::assets::MeshData::ptr sceneMesh = sim.getJoinedSemanticMesh(objectIds);

  std::shared_ptr<scene::SemanticScene> semanticScene = sim.getSemanticScene();
  const std::vector<std::shared_ptr<scene::SemanticCategory>>& categories =
      semanticScene->categories();
  const std::vector<std::shared_ptr<scene::SemanticObject>>& objects =
      semanticScene->objects();

  // Submit the vertex data
  static_assert( sizeof(vec3f) == 3*sizeof(float) );
  RLRA_Error error = RLRA_AddMeshVertices( context,
      (const float*)sceneMesh->vbo.data(), sceneMesh->vbo.size() );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't add vertices to audio mesh, error: "
                << static_cast<int>(error);
    return;
  }
  std::unordered_map<std::string, std::vector<uint32_t>> categoryNameToIndices;

  auto& ibo = sceneMesh->ibo;
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
      // if we reach here, cat 1 == 2 but != 3, use 3
      catToUse = cat3;
    } else {
      // else 1 == 3 but != 2, use 2
      catToUse = cat2;
    }

    categoryNameToIndices[catToUse].push_back(ibo[iboIdx]);
    categoryNameToIndices[catToUse].push_back(ibo[iboIdx + 1]);
    categoryNameToIndices[catToUse].push_back(ibo[iboIdx + 2]);
  }

  std::size_t totalIndicesLoaded = 0;

  // Send indices by category
  for (auto catToIndices : categoryNameToIndices) {
    ESP_DEBUG() << logHeader_ << " Index count: " << catToIndices.second.size()
                << ", Material: " << catToIndices.first;

    // Submit all indices for this particular category
    error = RLRA_AddMeshIndices( context,
        catToIndices.second.data(), catToIndices.second.size(), 3,
        catToIndices.first.c_str() );
    if ( error != RLRA_Success )
    {
      ESP_ERROR() << "Couldn't submit audio mesh indices";
      continue;
    }

    totalIndicesLoaded += catToIndices.second.size();
  }

  if (totalIndicesLoaded != sceneMesh->ibo.size()) {
    ESP_ERROR() << logHeader_
                << "totalIndicesLoaded != sceneMesh->ibo.size() : ("
                << totalIndicesLoaded << " != " << sceneMesh->ibo.size()
                << ")";
    CORRADE_ASSERT(false, "totalIndicesLoaded != sceneMesh->ibo.size()", );
  }

  // Add a new object for the mesh.
  const size_t objectIndex = RLRA_GetObjectCount( context );
  error = RLRA_AddObject( context );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't add audio object for mesh, error: "
                << static_cast<int>(error);
    return;
  }

  // Upload mesh into object.
  error = RLRA_FinalizeObjectMesh( context, objectIndex );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't upload mesh into object, error: "
                << static_cast<int>(error);
    return;
  }
}

void AudioSensor::loadMesh(sim::Simulator& sim) {
  ESP_DEBUG() << logHeader_ << "Loading non-semantic mesh";
  CORRADE_ASSERT(context, "loadMesh: context should exist", );
  esp::assets::MeshData::ptr sceneMesh = sim.getJoinedMesh(true);

  ESP_DEBUG() << "Vertex count : " << sceneMesh->vbo.size()
              << ", Index count : " << sceneMesh->ibo.size();

  // Submit the vertex data
  static_assert( sizeof(vec3f) == 3*sizeof(float) );
  RLRA_Error error = RLRA_AddMeshVertices( context,
      (const float*)sceneMesh->vbo.data(), sceneMesh->vbo.size() );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't add vertices to audio mesh, error: "
                << static_cast<int>(error);
    return;
  }

  // Submit the index data with default material.
  error = RLRA_AddMeshIndices( context,
      sceneMesh->ibo.data(), sceneMesh->ibo.size(), 3,
      nullptr );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't submit audio mesh indices";
  }

  // Add a new object for the mesh.
  const size_t objectIndex = RLRA_GetObjectCount( context );
  error = RLRA_AddObject( context );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't add audio object for mesh, error: "
                << static_cast<int>(error);
    return;
  }

  // Upload mesh into object.
  error = RLRA_FinalizeObjectMesh( context, objectIndex );
  if ( error != RLRA_Success )
  {
    ESP_ERROR() << "Couldn't upload mesh into object, error: "
                << static_cast<int>(error);
    return;
  }
}

#endif  // ESP_BUILD_WITH_AUDIO

}  // namespace sensor
}  // namespace esp
