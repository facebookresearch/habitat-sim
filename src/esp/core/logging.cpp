// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "logging.h"
#include "Check.h"

#include <algorithm>
#include <cstdlib>

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/StaticArray.h>
#include <Corrade/Containers/String.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/Format.h>

namespace Cr = Corrade;
using Cr::Containers::Literals::operator""_s;

namespace esp {
namespace logging {

constexpr const char* LoggingContext::LOGGING_ENV_VAR_NAME;
constexpr LoggingLevel LoggingContext::DEFAULT_LEVEL;

static_assert(uint8_t(Subsystem::NumSubsystems) ==
                  (sizeof(subsystemNames) / sizeof(subsystemNames[0])),
              "Missing a subsystem name.");

Subsystem subsystemFromName(const Corrade::Containers::StringView name) {
  const Cr::Containers::String lowerCaseName =
      Cr::Utility::String::lowercase(name);
  for (uint8_t i = 0; i < uint8_t(Subsystem::NumSubsystems); ++i)
    if (lowerCaseName == Cr::Utility::String::lowercase(
                             Cr::Containers::StringView{subsystemNames[i]}))
      return Subsystem(i);

  CORRADE_ASSERT_UNREACHABLE("Unknown subsystem '"
                                 << Cr::Utility::Debug::nospace << name
                                 << Cr::Utility::Debug::nospace << "'",
                             {});
}

LoggingLevel levelFromName(const Corrade::Containers::StringView name) {
  const Cr::Containers::String lowerCaseName =
      Cr::Utility::String::lowercase(name);
#define CASE(level, name)         \
  if (lowerCaseName == #name##_s) \
  return LoggingLevel::level

  CASE(VeryVerbose, veryverbose);
  CASE(Verbose, verbose);
  CASE(Debug, debug);
  CASE(Warning, warning);
  CASE(Quiet, quiet);
  CASE(Error, error);

#undef CASE
  CORRADE_ASSERT_UNREACHABLE("Unknown logging level name '"
                                 << Cr::Utility::Debug::nospace << name
                                 << Cr::Utility::Debug::nospace << "'",
                             {});
}

namespace {
#if defined(MAGNUM_BUILD_STATIC_UNIQUE_GLOBALS) && \
    !defined(CORRADE_TARGET_WINDOWS)
/* On static builds that get linked to multiple shared libraries and then used
   in a single app we want to ensure there's just one global symbol. On Linux
   it's apparently enough to just export, macOS needs the weak attribute.
   Windows handled differently below. */
CORRADE_VISIBILITY_EXPORT
#ifdef __GNUC__
__attribute__((weak))
#else
/* uh oh? the test will fail, probably */
#endif
#endif
const LoggingContext* currentLoggingContext = nullptr;
}  // namespace

bool LoggingContext::hasCurrent() {
  return currentLoggingContext != nullptr;
}

const LoggingContext& LoggingContext::current() {
  ESP_CHECK(hasCurrent(),
            "esp::logging::LoggingContext: No current logging context.");

  return *currentLoggingContext;
}

LoggingContext::LoggingContext(Corrade::Containers::StringView envString)
    : loggingLevels_{Cr::DirectInit, uint8_t(Subsystem::NumSubsystems),
                     DEFAULT_LEVEL},
      prevContext_{currentLoggingContext} {
  currentLoggingContext = this;
  for (const Cr::Containers::StringView setLevelCommand :
       envString.split(':')) {
    if (setLevelCommand.contains("=")) {
      const auto parts = setLevelCommand.partition('=');
      LoggingLevel lvl = levelFromName(parts[2]);

      for (const Cr::Containers::StringView subsystemName : parts[0].split(','))
        loggingLevels_[uint8_t(subsystemFromName(subsystemName))] = lvl;
    } else {
      std::fill(loggingLevels_.begin(), loggingLevels_.end(),
                levelFromName(setLevelCommand));
    }
  }
}

LoggingContext::LoggingContext()
    : LoggingContext{std::getenv(LOGGING_ENV_VAR_NAME)} {}

LoggingContext::~LoggingContext() {
  currentLoggingContext = prevContext_;
}

LoggingLevel LoggingContext::levelFor(Subsystem subsystem) const {
  return loggingLevels_[uint8_t(subsystem)];
}

bool isLevelEnabled(Subsystem subsystem, LoggingLevel level) {
  return level >= LoggingContext::current().levelFor(subsystem);
}

Cr::Containers::String buildMessagePrefix(Subsystem subsystem,
                                          const std::string& filename,
                                          const std::string& function,
                                          int line) {
  auto baseFileName = Cr::Utility::Directory::filename(filename);
  return ""_s.join({"["_s, subsystemNames[uint8_t(subsystem)], "] "_s,
                    baseFileName, "("_s, std::to_string(line), ")::"_s,
                    function, " : "_s});
}

}  // namespace logging
}  // namespace esp
