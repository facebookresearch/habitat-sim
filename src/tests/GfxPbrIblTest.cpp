// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Optional.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>

#include "esp/assets/ResourceManager.h"
#include "esp/core/Logging.h"
#include "esp/metadata/MetadataMediator.h"

#include "configure.h"

namespace {
namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::gfx::LightInfo;
using esp::gfx::LightPositionModel;
using esp::gfx::LightSetup;
using esp::metadata::MetadataMediator;
using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;

struct GfxPbrIblTest : Cr::TestSuite::Tester {
  explicit GfxPbrIblTest();

  esp::logging::LoggingContext loggingContext;

};  // struct GfxReplayTest

GfxPbrIblTest::GfxPbrIblTest() {}

}  // namespace

CORRADE_TEST_MAIN(GfxPbrIblTest)
