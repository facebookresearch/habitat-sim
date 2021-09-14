// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/core/logging.h"

#include <sstream>

#include <Corrade/Containers/StaticArray.h>
#include <Corrade/Containers/String.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/Containers/StringView.h>
#include <Corrade/TestSuite/Tester.h>

namespace Cr = Corrade;

namespace esp {

namespace sim {
namespace test {
namespace {
void debug(const Cr::Containers::StringView statement) {
  ESP_DEBUG() << statement;
}
void warning(const Cr::Containers::StringView statement) {
  ESP_WARNING() << statement;
}
}  // namespace
}  // namespace test
}  // namespace sim

namespace gfx {
namespace test {
namespace {
void debug(const Cr::Containers::StringView statement) {
  ESP_DEBUG() << statement;
}
void warning(const Cr::Containers::StringView statement) {
  ESP_WARNING() << statement;
}
}  // namespace
}  // namespace test
}  // namespace gfx
}  // namespace esp
namespace {

struct LoggingTest : Cr::TestSuite::Tester {
  explicit LoggingTest();

  void envVarTest();
};

constexpr const struct {
  const char* envString;
  const char* expected;
} EnvVarTestData[]{
    {nullptr,
     "[Default] LoggingTest.cpp(99)::envVarTest : DebugDefault\n[Default] "
     "LoggingTest.cpp(100)::envVarTest : WarningDefault\n"
     "[Sim] LoggingTest.cpp(23)::debug : DebugSim\n[Sim] "
     "LoggingTest.cpp(26)::warning : WarningSim\n"
     "[Gfx] LoggingTest.cpp(36)::debug : DebugGfx\n[Gfx] "
     "LoggingTest.cpp(39)::warning : WarningGfx\n"},
    {"debug",
     "[Default] LoggingTest.cpp(99)::envVarTest : DebugDefault\n[Default] "
     "LoggingTest.cpp(100)::envVarTest : WarningDefault\n"
     "[Sim] LoggingTest.cpp(23)::debug : DebugSim\n[Sim] "
     "LoggingTest.cpp(26)::warning : WarningSim\n"
     "[Gfx] LoggingTest.cpp(36)::debug : DebugGfx\n[Gfx] "
     "LoggingTest.cpp(39)::warning : WarningGfx\n"},
    {"quiet", ""},
    {"error", ""},
    {"quiet:Sim,Gfx=verbose",
     "[Sim] LoggingTest.cpp(23)::debug : DebugSim\n[Sim] "
     "LoggingTest.cpp(26)::warning : WarningSim\n[Gfx] "
     "LoggingTest.cpp(36)::debug : DebugGfx\n[Gfx] "
     "LoggingTest.cpp(39)::warning : WarningGfx\n"},
    {"warning:Gfx=debug",
     "[Default] LoggingTest.cpp(100)::envVarTest : WarningDefault\n"
     "[Sim] LoggingTest.cpp(26)::warning : WarningSim\n[Gfx] "
     "LoggingTest.cpp(36)::debug : DebugGfx\n[Gfx] "
     "LoggingTest.cpp(39)::warning : WarningGfx\n"},
};  // EnvVarTestData

LoggingTest::LoggingTest() {
  addInstancedTests({&LoggingTest::envVarTest},
                    Cr::Containers::arraySize(EnvVarTestData));
}

void LoggingTest::envVarTest() {
  auto&& data = EnvVarTestData[testCaseInstanceId()];

  esp::logging::LoggingContext ctx{data.envString};

  std::ostringstream out;
  Cr::Utility::Debug debugCapture{&out};
  Cr::Utility::Warning warnCapture{&out};

  ESP_DEBUG() << "DebugDefault";
  ESP_WARNING() << "WarningDefault";

  esp::sim::test::debug("DebugSim");
  esp::sim::test::warning("WarningSim");

  esp::gfx::test::debug("DebugGfx");
  esp::gfx::test::warning("WarningGfx");

  CORRADE_COMPARE(Cr::Containers::StringView{out.str()},
                  Cr::Containers::StringView{data.expected});
}

}  // namespace
CORRADE_TEST_MAIN(LoggingTest)
