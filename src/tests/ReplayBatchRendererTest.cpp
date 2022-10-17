// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "configure.h"

#include "esp/agent/Agent.h"  // todo: try to avoid using this
#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"
#include "esp/sim/ReplayBatchRenderer.h"
#include "esp/sim/Simulator.h"

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/ImageView.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::scene::SceneManager;
using esp::sim::ReplayBatchRenderer;
using esp::sim::ReplayBatchRendererConfiguration;
using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;

namespace {

const std::string screenshotDir =
    Cr::Utility::Path::join(TEST_ASSETS, "screenshots/");

struct ReplayBatchRendererTest : Cr::TestSuite::Tester {
  explicit ReplayBatchRendererTest();

  void testIntegration();

  const Magnum::Float maxThreshold = 255.f;
  const Magnum::Float meanThreshold = 0.75f;

  esp::logging::LoggingContext loggingContext;

};  // struct ReplayBatchRendererTest

Mn::MutableImageView2D getRGBView(int width,
                                  int height,
                                  std::vector<char>& buffer) {
  Mn::Vector2i size(width, height);
  constexpr int pixelSize = 4;

  buffer.resize(static_cast<size_type>(width * height * pixelSize));

  auto view = Mn::MutableImageView2D(Mn::PixelFormat::RGB8Unorm, size, buffer);

  return view;
}

ReplayBatchRendererTest::ReplayBatchRendererTest() {
  addTests({&ReplayBatchRendererTest::testIntegration});
}  // ctor

// test recording and playback through the simulator interface
void ReplayBatchRendererTest::testIntegration() {
  const std::string vangogh = Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/van-gogh-room.glb");
  constexpr int numEnvs = 4;
  std::string sensorName = "my_rgb";
  std::string userPrefix = "sensor_";

  std::vector<std::string> serKeyframes;
  for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
    SimulatorConfiguration simConfig{};
    simConfig.activeSceneName = vangogh;
    simConfig.enableGfxReplaySave = true;
    simConfig.createRenderer = false;

    auto sim = Simulator::create_unique(simConfig);

    // add and pose objects
    {
      auto objAttrMgr = sim->getObjectAttributesManager();
      objAttrMgr->loadAllJSONConfigsFromPath(
          Cr::Utility::Path::join(TEST_ASSETS, "objects/donut"), true);

      auto handles = objAttrMgr->getObjectHandlesBySubstring("donut");
      CORRADE_VERIFY(!handles.empty());

      auto rigidObj0 =
          sim->getRigidObjectManager()->addBulletObjectByHandle(handles[0]);
      rigidObj0->setTranslation(
          Mn::Vector3(1.5f, 1.0f + envIndex * 0.2f, 0.7f));
      rigidObj0->setRotation(
          Mn::Quaternion::rotation(Mn::Deg(45.f - envIndex * 5.f),
                                   Mn::Vector3(1.f, 0.f, 0.f).normalized()));

      auto rigidObj1 =
          sim->getRigidObjectManager()->addBulletObjectByHandle(handles[0]);
      rigidObj1->setTranslation(
          Mn::Vector3(1.5f, 1.2f + envIndex * 0.2f, -0.7f));
      rigidObj1->setRotation(
          Mn::Quaternion::rotation(Mn::Deg(30.f + envIndex * 5.f),
                                   Mn::Vector3(0.f, 0.f, 1.f).normalized()));
    }

    auto& recorder = *sim->getGfxReplayManager()->getRecorder().get();
    recorder.addUserTransformToKeyframe(
        userPrefix + sensorName, Mn::Vector3(3.3f, 1.3f + envIndex * 0.1f, 0.f),
        Mn::Quaternion::rotation(Mn::Deg(80.f + envIndex * 5.f),
                                 Mn::Vector3(0.f, 1.f, 0.f)));

    std::string serKeyframe = esp::gfx::replay::Recorder::keyframeToString(
        recorder.extractKeyframe());
    serKeyframes.emplace_back(std::move(serKeyframe));
  }

  std::vector<esp::sensor::SensorSpec::ptr> sensorSpecifications;
  {
    auto pinholeCameraSpec = esp::sensor::CameraSensorSpec::create();
    pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
    pinholeCameraSpec->sensorType = esp::sensor::SensorType::Color;
    pinholeCameraSpec->position = {0.0f, 0.f, 0.0f};
    pinholeCameraSpec->resolution = {512, 384};
    pinholeCameraSpec->uuid = sensorName;
    sensorSpecifications = {pinholeCameraSpec};
  }

  ReplayBatchRendererConfiguration batchRendererConfig;
  batchRendererConfig.sensorSpecifications = std::move(sensorSpecifications);
  batchRendererConfig.numEnvironments = numEnvs;
  ReplayBatchRenderer batchRenderer(batchRendererConfig);
  auto renderer = batchRenderer.getRenderer();

  std::vector<std::vector<char>> buffers(numEnvs);
  std::vector<Mn::MutableImageView2D> imageViews;

  for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
    auto& sensorMap = batchRenderer.getEnvironmentSensors(envIndex);
    CORRADE_INTERNAL_ASSERT(sensorMap.size() == 1);
    for (auto& kv : sensorMap) {
      CORRADE_VERIFY(kv.second.get().isVisualSensor());
      auto& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(kv.second.get());
      renderer->bindRenderTarget(visualSensor);

      const int width = visualSensor.framebufferSize().x();
      const int height = visualSensor.framebufferSize().y();
      imageViews.emplace_back(getRGBView(width, height, buffers[envIndex]));
    }
  }

  for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
    batchRenderer.setEnvironmentKeyframe(envIndex, serKeyframes[envIndex]);
    batchRenderer.setSensorTransformsFromKeyframe(envIndex, userPrefix);

    auto& sensorMap = batchRenderer.getEnvironmentSensors(envIndex);
    CORRADE_INTERNAL_ASSERT(sensorMap.size() == 1);
    for (auto& kv : sensorMap) {
      CORRADE_VERIFY(kv.second.get().isVisualSensor());
      auto& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(kv.second.get());

      auto& sceneGraph = batchRenderer.getSceneGraph(envIndex);

      // todo: investigate flags (frustum culling?)
      renderer->enqueueAsyncDrawJob(visualSensor, sceneGraph,
                                    imageViews[envIndex],
                                    esp::gfx::RenderCamera::Flags{});
    }
  }

  renderer->startDrawJobs();
  renderer->waitDrawJobs();

  for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
    std::string groundTruthImageFile =
        "ReplayBatchRendererTest_env" + std::to_string(envIndex) + ".png";
    CORRADE_COMPARE_WITH(
        Mn::ImageView2D{imageViews[envIndex]},
        Cr::Utility::Path::join(screenshotDir, groundTruthImageFile),
        (Mn::DebugTools::CompareImageToFile{maxThreshold, meanThreshold}));
  }
}

}  // namespace

CORRADE_TEST_MAIN(ReplayBatchRendererTest)
