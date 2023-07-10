// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Corrade/Containers/EnumSet.h"
#include "Corrade/Containers/StridedArrayView.h"
#include "Magnum/GL/Context.h"
#include "configure.h"

#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"
#include "esp/sim/AbstractReplayRenderer.h"
#include "esp/sim/BatchReplayRenderer.h"
#include "esp/sim/ClassicReplayRenderer.h"
#include "esp/sim/Simulator.h"

#include <esp/gfx_batch/RendererStandalone.h>
#include <esp/sim/BatchPlayerImplementation.h>

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/ImageView.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::scene::SceneManager;
using esp::sim::ReplayRendererConfiguration;
using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;

namespace {

const std::string screenshotDir =
    Cr::Utility::Path::join(TEST_ASSETS, "screenshots/");

enum class TestFlag : Magnum::UnsignedInt { Color = 1 << 0, Depth = 1 << 1 };
typedef Corrade::Containers::EnumSet<TestFlag> TestFlags;

struct BatchReplayRendererTest : Cr::TestSuite::Tester {
  explicit BatchReplayRendererTest();

  void testIntegration();
  void testUnproject();
  void testBatchPlayerDeletion();
  void testClose();

  const Magnum::Float maxThreshold = 255.f;
  const Magnum::Float meanThreshold = 0.75f;

  esp::logging::LoggingContext loggingContext;

};  // struct BatchReplayRendererTest

Mn::MutableImageView2D getRGBView(int width,
                                  int height,
                                  std::vector<char>& buffer) {
  Mn::Vector2i size(width, height);
  constexpr int pixelSize = 4;

  buffer.resize(std::size_t(width * height * pixelSize));

  auto view = Mn::MutableImageView2D(Mn::PixelFormat::RGB8Unorm, size, buffer);

  return view;
}

Mn::MutableImageView2D getDepthView(int width,
                                    int height,
                                    std::vector<char>& buffer) {
  Mn::Vector2i size(width, height);
  constexpr int pixelSize = 4;

  buffer.resize(std::size_t(width * height * pixelSize));

  auto view = Mn::MutableImageView2D(Mn::PixelFormat::R32F, size, buffer);

  return view;
}

esp::sensor::SensorSpec::ptr getDefaultSensorSpecs(
    const std::string& sensorName,
    const esp::sensor::SensorType sensorType) {
  auto pinholeCameraSpec = esp::sensor::CameraSensorSpec::create();
  pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
  pinholeCameraSpec->sensorType = sensorType;
  pinholeCameraSpec->position = {0.0f, 0.f, 0.0f};
  pinholeCameraSpec->resolution = {512, 384};
  pinholeCameraSpec->uuid = sensorName;
  return pinholeCameraSpec;
}

std::vector<esp::sensor::SensorSpec::ptr> getDefaultSensorSpecs(
    TestFlags flags) {
  std::vector<esp::sensor::SensorSpec::ptr> sensorSpecifications{};
  if (flags & TestFlag::Color) {
    sensorSpecifications.push_back(
        getDefaultSensorSpecs("rgb", esp::sensor::SensorType::Color));
  }
  if (flags & TestFlag::Depth) {
    sensorSpecifications.push_back(
        getDefaultSensorSpecs("depth", esp::sensor::SensorType::Depth));
  }
  return sensorSpecifications;
}

const struct {
  const char* name;
  Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> (*create)(
      const ReplayRendererConfiguration& configuration);
} TestUnprojectData[]{
    {"classic",
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::ClassicReplayRenderer{configuration}};
     }},
    // temp only enable testUnproject for classic
    //{"batch", [](const ReplayRendererConfiguration& configuration) {
    //   return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
    //       new esp::sim::BatchReplayRenderer{configuration}};
    // }
    //},
};

const struct {
  const char* name;
  TestFlags testFlags;
  Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> (*create)(
      const ReplayRendererConfiguration& configuration);
} TestIntegrationData[]{
    {"rgb - classic", TestFlag::Color,
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::ClassicReplayRenderer{configuration}};
     }},
    {"rgb - batch", TestFlag::Color,
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::BatchReplayRenderer{configuration}};
     }},
    {"depth - classic", TestFlag::Depth,
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::ClassicReplayRenderer{configuration}};
     }},
    {"depth - batch", TestFlag::Depth,
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::BatchReplayRenderer{configuration}};
     }},
};

const struct {
  const char* name;
  Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> (*create)(
      const ReplayRendererConfiguration& configuration);
} TestCloseData[]{
    {"classic",
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::ClassicReplayRenderer{configuration}};
     }},
    {"batch",
     [](const ReplayRendererConfiguration& configuration) {
       return Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer>{
           new esp::sim::BatchReplayRenderer{configuration}};
     }},
};

BatchReplayRendererTest::BatchReplayRendererTest() {
  addInstancedTests({&BatchReplayRendererTest::testUnproject},
                    Cr::Containers::arraySize(TestUnprojectData));

  addInstancedTests({&BatchReplayRendererTest::testIntegration},
                    Cr::Containers::arraySize(TestIntegrationData));

  addTests({&BatchReplayRendererTest::testBatchPlayerDeletion});

  addInstancedTests({&BatchReplayRendererTest::testClose},
                    Cr::Containers::arraySize(TestCloseData));
}  // ctor

// test recording and playback through the simulator interface
void BatchReplayRendererTest::testUnproject() {
  auto&& data = TestIntegrationData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  std::vector<esp::sensor::SensorSpec::ptr> sensorSpecifications;
  auto pinholeCameraSpec = esp::sensor::CameraSensorSpec::create();
  pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
  pinholeCameraSpec->sensorType = esp::sensor::SensorType::Color;
  pinholeCameraSpec->position = {1.0f, 2.f, 3.0f};
  pinholeCameraSpec->resolution = {512, 384};
  pinholeCameraSpec->uuid = "my_rgb";
  sensorSpecifications = {pinholeCameraSpec};

  ReplayRendererConfiguration batchRendererConfig;
  batchRendererConfig.sensorSpecifications = std::move(sensorSpecifications);
  batchRendererConfig.numEnvironments = 1;
  {
    Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> renderer =
        data.create(batchRendererConfig);

    const int h = pinholeCameraSpec->resolution.x();
    const int w = pinholeCameraSpec->resolution.y();
    constexpr int envIndex = 0;
    auto ray = renderer->unproject(envIndex, {0, 0});
    CORRADE_COMPARE(Mn::Vector3(ray.origin),
                    Mn::Vector3(pinholeCameraSpec->position));
    CORRADE_COMPARE(Mn::Vector3(ray.direction),
                    Mn::Vector3(-0.51544, 0.68457, -0.51544));

    // Ug, these tests reveal an off-by-one bug in our implementation in
    // RenderCamera::unproject. Depending on your convention, we would expect
    // some of these various corners to have exactly mirrored results, but they
    // don't.
    ray = renderer->unproject(envIndex, {w, 0});
    CORRADE_COMPARE(Mn::Vector3(ray.direction),
                    Mn::Vector3(0.51544, 0.68457, -0.51544));
    ray = renderer->unproject(envIndex, {w - 1, 0});
    CORRADE_COMPARE(Mn::Vector3(ray.direction),
                    Mn::Vector3(0.513467, 0.68551, -0.51615));

    ray = renderer->unproject(envIndex, {0, h});
    CORRADE_COMPARE(Mn::Vector3(ray.direction),
                    Mn::Vector3(-0.51355, -0.68740, -0.51355));
    ray = renderer->unproject(envIndex, {0, h - 1});
    CORRADE_COMPARE(Mn::Vector3(ray.direction),
                    Mn::Vector3(-0.51449, -0.68599, -0.51450));
  }
}

// test recording and playback through the simulator interface
void BatchReplayRendererTest::testIntegration() {
  auto&& data = TestIntegrationData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  const auto sensorSpecs = getDefaultSensorSpecs(data.testFlags);

  const std::string vangogh = Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/van-gogh-room.glb");
  constexpr int numEnvs = 4;
  const std::string userPrefix = "sensor_";
  const std::string screenshotPrefix = "ReplayBatchRendererTest_env";

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

    auto& recorder = *sim->getGfxReplayManager()->getRecorder();
    for (const auto& sensor : sensorSpecs) {
      recorder.addUserTransformToKeyframe(
          userPrefix + sensor->uuid,
          Mn::Vector3(3.3f, 1.3f + envIndex * 0.1f, 0.f),
          Mn::Quaternion::rotation(Mn::Deg(80.f + envIndex * 5.f),
                                   Mn::Vector3(0.f, 1.f, 0.f)));
    }

    std::string serKeyframe = esp::gfx::replay::Recorder::keyframeToString(
        recorder.extractKeyframe());
    serKeyframes.emplace_back(std::move(serKeyframe));
  }

  ReplayRendererConfiguration batchRendererConfig;
  batchRendererConfig.sensorSpecifications = sensorSpecs;
  batchRendererConfig.numEnvironments = numEnvs;
  {
    Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> renderer =
        data.create(batchRendererConfig);

    // Check that the context is properly created
    CORRADE_VERIFY(Mn::GL::Context::hasCurrent());

    std::vector<std::vector<char>> colorBuffers(numEnvs);
    std::vector<std::vector<char>> depthBuffers(numEnvs);
    std::vector<Mn::MutableImageView2D> colorImageViews;
    std::vector<Mn::MutableImageView2D> depthImageViews;

    for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
      if (data.testFlags & TestFlag::Color) {
        colorImageViews.emplace_back(getRGBView(
            renderer->sensorSize(envIndex).x(),
            renderer->sensorSize(envIndex).y(), colorBuffers[envIndex]));
      }
      if (data.testFlags & TestFlag::Depth) {
        depthImageViews.emplace_back(getDepthView(
            renderer->sensorSize(envIndex).x(),
            renderer->sensorSize(envIndex).y(), depthBuffers[envIndex]));
      }
    }

    for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
      renderer->setEnvironmentKeyframe(envIndex, serKeyframes[envIndex]);
      renderer->setSensorTransformsFromKeyframe(envIndex, userPrefix);
    }

    renderer->render(colorImageViews, depthImageViews);

    for (int envIndex = 0; envIndex < numEnvs; envIndex++) {
      CORRADE_ITERATION(envIndex);
      // Test color output
      if (data.testFlags & TestFlag::Color) {
        std::string groundTruthImageFile =
            screenshotPrefix + std::to_string(envIndex) + ".png";
        CORRADE_COMPARE_WITH(
            Mn::ImageView2D{colorImageViews[envIndex]},
            Cr::Utility::Path::join(screenshotDir, groundTruthImageFile),
            (Mn::DebugTools::CompareImageToFile{maxThreshold, meanThreshold}));
      }
      // Test depth output
      if (data.testFlags & TestFlag::Depth) {
        const auto depth = depthImageViews[envIndex];
        std::string groundTruthImageFile =
            screenshotPrefix + std::to_string(envIndex) + ".exr";
        CORRADE_COMPARE_WITH(
            Mn::ImageView2D{depthImageViews[envIndex]},
            Cr::Utility::Path::join(screenshotDir, groundTruthImageFile),
            (Mn::DebugTools::CompareImageToFile{maxThreshold, meanThreshold}));
      }
    }

    const auto colorPtr = renderer->getCudaColorBufferDevicePointer();
    const auto depthPtr = renderer->getCudaDepthBufferDevicePointer();
    bool isBatchRenderer =
        dynamic_cast<esp::sim::BatchReplayRenderer*>(renderer.get());
#ifdef ESP_BUILD_WITH_CUDA
    if (isBatchRenderer) {
      CORRADE_VERIFY(colorPtr);
      CORRADE_VERIFY(depthPtr);
    } else {
      // Not implemented in ClassicReplayRenderer
      CORRADE_VERIFY(!colorPtr);
      CORRADE_VERIFY(!depthPtr);
    }
#else
    CORRADE_VERIFY(!colorPtr);
    CORRADE_VERIFY(!depthPtr);
#endif
  }
  // Check that the context is properly deleted
  CORRADE_VERIFY(!Mn::GL::Context::hasCurrent());
}

// test batch replay renderer node deletion
void BatchReplayRendererTest::testBatchPlayerDeletion() {
  const std::string assetPath =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/sphere.glb");
  const auto assetInfo = esp::assets::AssetInfo::fromPath(assetPath);

  esp::assets::RenderAssetInstanceCreationInfo::Flags flags;
  flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD;
  flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic;
  auto creationInfo = esp::assets::RenderAssetInstanceCreationInfo();
  creationInfo.filepath = assetPath;
  creationInfo.flags = flags;

  std::vector<esp::gfx::replay::Keyframe> keyframes;

  // Record sequence
  {
    SimulatorConfiguration simConfig{};
    simConfig.enableGfxReplaySave = true;
    simConfig.createRenderer = false;
    auto sim = Simulator::create_unique(simConfig);
    auto& sceneRoot = sim->getActiveSceneGraph().getRootNode();
    auto& recorder = *sim->getGfxReplayManager()->getRecorder();

    {
      // Frame 0
      auto* sphereX =
          sim->loadAndCreateRenderAssetInstance(assetInfo, creationInfo);
      auto* sphereY =
          sim->loadAndCreateRenderAssetInstance(assetInfo, creationInfo);
      CORRADE_VERIFY(sphereX);
      CORRADE_VERIFY(sphereY);
      sphereX->setTranslation(Mn::Vector3(1.0f, 0.0f, 0.0f));
      sphereY->setTranslation(Mn::Vector3(0.0f, 1.0f, 0.0f));
      keyframes.emplace_back(std::move(recorder.extractKeyframe()));

      // Frame 1
      delete sphereX;
      auto sphereZ =
          sim->loadAndCreateRenderAssetInstance(assetInfo, creationInfo);
      CORRADE_VERIFY(sphereZ);
      sphereZ->setTranslation(Mn::Vector3(0.0f, 0.0f, 1.0f));
      delete sphereZ;
      keyframes.emplace_back(std::move(recorder.extractKeyframe()));

      // Frame 2
      delete sphereY;
      keyframes.emplace_back(std::move(recorder.extractKeyframe()));
    }
  }
  CORRADE_COMPARE(keyframes.size(), 3);

  // Play sequence
  {
    // clang-format off
    esp::gfx_batch::RendererStandalone renderer{
        esp::gfx_batch::RendererConfiguration{}
            .setTileSizeCount({16, 16}, {1, 1}),
        esp::gfx_batch::RendererStandaloneConfiguration{}
            .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
    };
    // clang-format on
    auto batchPlayer =
        std::make_shared<esp::sim::BatchPlayerImplementation>(renderer, 0);
    esp::gfx::replay::Player player(batchPlayer);
    player.debugSetKeyframes(std::move(keyframes));
    CORRADE_COMPARE(player.getNumKeyframes(), 3);
    constexpr size_t transformsPerInstance = 2;

    // Frame 0
    player.setKeyframeIndex(0);
    CORRADE_COMPARE(renderer.transformations(0).size(),
                    2 * transformsPerInstance);
    CORRADE_COMPARE(renderer.transformations(0)[0],
                    Mn::Matrix4::translation(Mn::Vector3(1.0f, 0.0f, 0.0f)));
    CORRADE_COMPARE(renderer.transformations(0)[1 * transformsPerInstance],
                    Mn::Matrix4::translation(Mn::Vector3(0.0f, 1.0f, 0.0f)));

    // Frame 2
    player.setKeyframeIndex(1);
    CORRADE_COMPARE(renderer.transformations(0).size(),
                    1 * transformsPerInstance);
    CORRADE_COMPARE(renderer.transformations(0)[0],
                    Mn::Matrix4::translation(Mn::Vector3(0.0f, 1.0f, 0.0f)));

    // Frame 2
    player.setKeyframeIndex(2);
    CORRADE_COMPARE(renderer.transformations(0).size(), 0);
  }
}

void BatchReplayRendererTest::testClose() {
  auto&& data = TestIntegrationData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  auto sensorSpecifications = getDefaultSensorSpecs(TestFlag::Color);
  ReplayRendererConfiguration batchRendererConfig;
  batchRendererConfig.sensorSpecifications = std::move(sensorSpecifications);
  batchRendererConfig.numEnvironments = 4;
  batchRendererConfig.standalone = true;
  Cr::Containers::Pointer<esp::sim::AbstractReplayRenderer> renderer =
      data.create(batchRendererConfig);

  // Verify that a context exists.
  CORRADE_VERIFY(renderer);
  CORRADE_VERIFY(Mn::GL::Context::hasCurrent());

  // Verify that the context is released.
  renderer->close();
  CORRADE_VERIFY(!Mn::GL::Context::hasCurrent());

  // Verify that closing multiple times is safe.
  renderer->close();
  renderer.reset();
}

}  // namespace

CORRADE_TEST_MAIN(BatchReplayRendererTest)
