// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "logging.h"
#include "Check.h"

#include <algorithm>
#include <cstdlib>

#include <Corrade/Containers/StaticArray.h>
#include <Corrade/Containers/String.h>

namespace Cr = Corrade;
using Cr::Containers::Literals::operator""_s;

namespace esp {
namespace logging {

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
#ifdef CORRADE_BUILD_MULTITHREADED
CORRADE_THREAD_LOCAL
#endif
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
LoggingContext* currentLoggingContext = nullptr;
}  // namespace

bool LoggingContext::hasCurrent() {
  return currentLoggingContext != nullptr;
}

LoggingContext& LoggingContext::current() {
  ESP_CHECK(hasCurrent(),
            "esp::logging::LoggingContext: No current logging context.");

  return *currentLoggingContext;
}

LoggingContext::LoggingContext(Corrade::Containers::StringView envString)
    : loggingLevels_{Cr::DirectInit, uint8_t(Subsystem::NumSubsystems),
                     DEFAULT_LEVEL},
      prevContext_{currentLoggingContext} {
  currentLoggingContext = this;
  processEnvString(envString);
}

LoggingContext::LoggingContext()
    : LoggingContext{std::getenv(LOGGING_ENV_VAR_NAME)} {}

LoggingContext::~LoggingContext() {
  currentLoggingContext = prevContext_;
}

void LoggingContext::processEnvString(
    const Cr::Containers::StringView envString) {
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

void LoggingContext::reinitializeFromEnv() {
  std::fill(loggingLevels_.begin(), loggingLevels_.end(), DEFAULT_LEVEL);
  processEnvString(std::getenv(LOGGING_ENV_VAR_NAME));
}

LoggingLevel LoggingContext::levelFor(Subsystem subsystem) const {
  return loggingLevels_[uint8_t(subsystem)];
}

namespace {
template <class Logger>
Logger outputForImpl(Subsystem subsystem, LoggingLevel level) {
  if (level >= LoggingContext::current().levelFor(subsystem)) {
    Logger logger{};
    logger << "[" << Logger::nospace << subsystemNames[uint8_t(subsystem)]
           << Logger::nospace << "]";
    return logger;
  } else
    return Logger{nullptr};
}
}  // namespace

Cr::Utility::Debug debugOutputFor(Subsystem subsystem) {
  return outputForImpl<Cr::Utility::Debug>(subsystem, LoggingLevel::Debug);
}

Cr::Utility::Warning warningOutputFor(Subsystem subsystem) {
  return outputForImpl<Cr::Utility::Warning>(subsystem, LoggingLevel::Warning);
}

Cr::Utility::Error errorOutputFor(Subsystem subsystem) {
  return outputForImpl<Cr::Utility::Error>(subsystem, LoggingLevel::Error);
}

}  // namespace logging
}  // namespace esp
