// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/sim/SimulatorWithAgents.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::agent::AgentConfiguration;
using esp::nav::PathFinder;
using esp::scene::SceneConfiguration;
using esp::sensor::Observation;
using esp::sensor::ObservationSpace;
using esp::sensor::ObservationSpaceType;
using esp::sensor::SensorSpec;
using esp::sensor::SensorType;
using esp::sim::SimulatorConfiguration;
using esp::sim::SimulatorWithAgents;

const std::string vangogh =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "habitat-test-scenes/van-gogh-room.glb");
const std::string skokloster =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "habitat-test-scenes/skokloster-castle.glb");

// TODO(MM): move following to common test utils module
template <typename T, typename U>
bool equalWithTolerance(const T& lhs, const T& rhs, U tolerance) {
  return std::abs(rhs - lhs) <= std::abs(tolerance);
}

bool pixelEqualWithChannelTolerance(const Mn::Color4ub& lhs,
                                    const Mn::Color4ub& rhs,
                                    int tolerance) {
  return equalWithTolerance(lhs.r(), rhs.r(), tolerance) &&
         equalWithTolerance(lhs.g(), rhs.g(), tolerance) &&
         equalWithTolerance(lhs.b(), rhs.b(), tolerance) &&
         equalWithTolerance(lhs.a(), rhs.a(), tolerance);
}

TEST(SimTest, Basic) {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  SimulatorWithAgents simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  ASSERT_NE(pathfinder, nullptr);
}

TEST(SimTest, Reconfigure) {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  SimulatorWithAgents simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  simulator.reconfigure(cfg);
  ASSERT_EQ(pathfinder, simulator.getPathFinder());
  SimulatorConfiguration cfg2;
  cfg2.scene.id = skokloster;
  simulator.reconfigure(cfg2);
  ASSERT_NE(pathfinder, simulator.getPathFinder());
}

TEST(SimTest, Reset) {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  SimulatorWithAgents simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  simulator.reset();
  ASSERT_EQ(pathfinder, simulator.getPathFinder());
}

TEST(SimTest, GetPinholeCameraRGBAObservation) {
  SimulatorConfiguration simConfig{};
  simConfig.scene.id = vangogh;

  // do not rely on default SensorSpec default constructor to remain constant
  auto pinholeCameraSpec = SensorSpec::create();
  pinholeCameraSpec->sensorSubtype = "pinhole";
  pinholeCameraSpec->sensorType = SensorType::COLOR;
  pinholeCameraSpec->position = {0.0f, 1.5f, 5.0f};
  pinholeCameraSpec->resolution = {100, 100};

  SimulatorWithAgents simulator(simConfig);
  AgentConfiguration agentConfig{};
  agentConfig.sensorSpecifications = {pinholeCameraSpec};
  simulator.addAgent(agentConfig);

  Observation observation;
  ObservationSpace obsSpace;
  ASSERT_TRUE(simulator.getAgentObservation(
      simConfig.defaultAgentId, pinholeCameraSpec->uuid, observation));
  ASSERT_TRUE(simulator.getAgentObservationSpace(
      simConfig.defaultAgentId, pinholeCameraSpec->uuid, obsSpace));

  std::vector<size_t> expectedShape{
      {static_cast<size_t>(pinholeCameraSpec->resolution[0]),
       static_cast<size_t>(pinholeCameraSpec->resolution[1]), 4}};

  ASSERT_EQ(obsSpace.spaceType, ObservationSpaceType::TENSOR);
  ASSERT_EQ(obsSpace.dataType, esp::core::DataType::DT_UINT8);
  ASSERT_EQ(obsSpace.shape, expectedShape);
  ASSERT_EQ(observation.buffer->shape, expectedShape);

  // Compare one pixel (center of image) with previously rendered ground truth
  // TODO: compare more than one pixel
  Mn::ImageView2D image{
      Mn::PixelFormat::RGBA8Unorm,
      {pinholeCameraSpec->resolution[0], pinholeCameraSpec->resolution[1]},
      observation.buffer->data};

  Mn::Color4ub pixel = image.pixels<Mn::Color4ub>()[50][50];

  // Ground truth: hardcoded from previous render
  // TODO: move various sensor configurations and expected ground truths to
  // common test util
  Mn::Color4ub expectedPixel{0x40, 0x6C, 0x46, 0xB5};

  ASSERT_TRUE(pixelEqualWithChannelTolerance(pixel, expectedPixel, 5));
}
