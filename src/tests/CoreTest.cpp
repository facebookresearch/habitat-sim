// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include "esp/core/Configuration.h"
#include "esp/core/Esp.h"

using namespace esp::core::config;

namespace {

struct CoreTest : Cr::TestSuite::Tester {
  explicit CoreTest();

  void TestConfiguration();

  esp::logging::LoggingContext loggingContext_;
};  // struct CoreTest

CoreTest::CoreTest() {
  addTests({&CoreTest::TestConfiguration});
}

void CoreTest::TestConfiguration() {
  Configuration cfg;
  cfg.set("myInt", 10);
  cfg.set("myString", "test");
  CORRADE_VERIFY(cfg.hasValue("myInt"));
  CORRADE_VERIFY(cfg.hasValue("myString"));
  CORRADE_COMPARE(cfg.get<int>("myInt"), 10);
  CORRADE_COMPARE(cfg.get<std::string>("myString"), "test");
}

}  // namespace

CORRADE_TEST_MAIN(CoreTest)
