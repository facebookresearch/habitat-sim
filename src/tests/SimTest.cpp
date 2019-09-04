// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/sim/SimulatorWithAgents.h"

#include "configure.h"

namespace Cr = Corrade;

using esp::gfx::SimulatorConfiguration;
using esp::nav::PathFinder;
using esp::scene::SceneConfiguration;
using esp::sim::SimulatorWithAgents;

const std::string vangogh =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "habitat-test-scenes/van-gogh-room.glb");
const std::string skokloster =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "habitat-test-scenes/skokloster-castle.glb");

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
