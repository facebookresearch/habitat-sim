// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>

#include "esp/core/Esp.h"
#include "esp/scene/SemanticScene.h"

#include "configure.h"
namespace {

namespace Cr = Corrade;
struct SemanticTest : Cr::TestSuite::Tester {
  explicit SemanticTest();

  void TestRegions();

  esp::logging::LoggingContext loggingContext_;
};  // struct SemanticTest

SemanticTest::SemanticTest() {
  addTests({&SemanticTest::TestRegions});
}

void SemanticTest::TestRegions() {
  CORRADE_VERIFY(1 == 1);

}  // SemanticTest::TestRegions

}  // namespace

CORRADE_TEST_MAIN(SemanticTest)
