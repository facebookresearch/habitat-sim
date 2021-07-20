// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_LOGGING_H_
#define ESP_CORE_LOGGING_H_

#include "esp/core/configure.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/StringView.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/String.h>

namespace esp {
namespace logging {
// These are all subsystems that have different logging levels
// They correspond to namespaces.  When adding a new subsystem, make sure
// to add an appropriate loggingSubsystem function (see the ADD_SUBSYSTEM_FN
// helper bellow) and add it to @ref subsystemFromName()
enum class Subsystem : uint8_t {
  // This is the catch all subsystem
  Default,
  gfx,
  scene,
  sim,
  physics,
  nav,
  metadata,
  geo,
  io,
  URDF,

  // Must always be last
  NumSubsystems,
};

constexpr const char* subsystemNames[] = {"Default", "Gfx", "Scene",    "Sim",
                                          "Physics", "Nav", "Metadata", "Geo",
                                          "IO",      "URDF"};

Subsystem subsystemFromName(Corrade::Containers::StringView name);

}  // namespace logging
}  // namespace esp

// This is the catch all for subsystems without a specialization
// Using c style namespacing that way logging can work outside our namespace
inline esp::logging::Subsystem espLoggingSubsystem() {
  return esp::logging::Subsystem::Default;
}

#define ADD_SUBSYSTEM_FN(subsystemName)             \
  namespace esp {                                   \
  namespace subsystemName {                         \
  inline logging::Subsystem espLoggingSubsystem() { \
    return logging::Subsystem::subsystemName;       \
  }                                                 \
  }                                                 \
  }

ADD_SUBSYSTEM_FN(gfx);
ADD_SUBSYSTEM_FN(scene);
ADD_SUBSYSTEM_FN(sim);
ADD_SUBSYSTEM_FN(physics);
ADD_SUBSYSTEM_FN(nav);
ADD_SUBSYSTEM_FN(metadata);
ADD_SUBSYSTEM_FN(geo);
ADD_SUBSYSTEM_FN(io);

namespace esp {
namespace io {
namespace URDF {
inline logging::Subsystem espLoggingSubsystem() {
  return logging::Subsystem::URDF;
}
}  // namespace URDF
}  // namespace io
}  // namespace esp

namespace esp {
namespace logging {

enum class LoggingLevel : uint8_t {
  Debug,
  Warning,
  Error,

  // Verbose enables debug and higher
  // Note:  If you add something lower prio than Debug,
  // Verbose should be set equal to that
  Verbose = Debug,
  // Quiet is errors and higher
  Quiet = Error,
};

LoggingLevel levelFromName(Corrade::Containers::StringView name);

class LoggingContext {
 public:
  constexpr static const char* LOGGING_ENV_VAR_NAME = "HABITAT_SIM_LOG";
  constexpr static LoggingLevel DEFAULT_LEVEL = LoggingLevel::Verbose;

  /**
   * @brief Convenience constructor that grabs the configuration string from the
   * environment variable and calls @ref
   * LoggingContext(Corrade::Containers::StringView)
   */
  LoggingContext();

  /**
   * @brief Constructor
   *
   * @param[in] envString Configuration string. See @ref processEnvString for
   * details
   */
  explicit LoggingContext(Corrade::Containers::StringView envString);

  ~LoggingContext();

  LoggingContext(LoggingContext&&) = delete;
  LoggingContext(LoggingContext&) = delete;
  LoggingContext& operator=(LoggingContext&) = delete;
  LoggingContext& operator=(LoggingContext&&) = delete;

  /**
   * @brief Processes the environment variable string that configures the
   * habitat-sim logging levels for various subsystems
   *
   * This environment string has a fairly simple grammar that is as follows
   *
   *    FilterString: SetLevelCommand (COLON SetLevelCommand)*
   *    SetLevelCommand: (SUBSYSTEM (COMMA SUBSYSTEM)* EQUALS)? LOGGING_LEVEL
   *
   * where SUBSYSTEM is a known logging subsystem and LOGGING_LEVEL is one of
   * the logging levels.  If a SetLevelCommand does not contain a list of
   * subsystems, that level is applied to all subsystems.
   *
   *
   * A logging statement is printed if it's logging level has a higher or equal
   * priority to the current logging level for it's subsystem.  verbose is the
   * lowest level. quiet disables debug and warnings.
   */
  void processEnvString(Corrade::Containers::StringView envString);

  /**
   * @brief Retrieves the configuration string from the environment variable
   * and reconfigures levels.  Useful for allowing the environment variable
   * to be changed after module level initialization.
   */
  void reinitializeFromEnv();

  LoggingLevel levelFor(Subsystem subsystem) const;

  static bool hasCurrent();
  static LoggingContext& current();

 private:
  Corrade::Containers::Array<LoggingLevel> loggingLevels_;
  LoggingContext* prevContext_;
};

Corrade::Utility::Debug debugOutputFor(Subsystem subsystem);
Corrade::Utility::Warning warningOutputFor(Subsystem subsystem);
Corrade::Utility::Error errorOutputFor(Subsystem subsystem);
}  // namespace logging
}  // namespace esp

#define ESP_DEBUG() esp::logging::debugOutputFor(espLoggingSubsystem())
#define ESP_WARNING() esp::logging::warningOutputFor(espLoggingSubsystem())
#define ESP_ERROR() esp::logging::errorOutputFor(espLoggingSubsystem())

#if defined(ESP_BUILD_GLOG_SHIM)

#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>

// Our own shims "emulating" GLOG
// TODO shims are hack to get things compiling, implement expected behaviors

class LogMessageVoidify {
 public:
  LogMessageVoidify() {}
  // This has to be an operator with a precedence lower than << but
  // higher than ?:
  void operator&(Corrade::Utility::Debug&) {}
};

#define GLOG_INFO \
  Corrade::Utility::Debug {}
#define GLOG_WARNING \
  Corrade::Utility::Warning {}
#define GLOG_ERROR \
  Corrade::Utility::Error {}
#define GLOG_FATAL \
  Corrade::Utility::Fatal {}
#define LOG(severity) GLOG_##severity
#define LOG_IF(severity, condition) \
  !(condition) ? (void)0 : LogMessageVoidify() & LOG(severity)

#define VLOG_LEVEL 0

#define VLOG_IS_ON(verboselevel) (VLOG_LEVEL >= (verboselevel))
#define VLOG(verboselevel) LOG_IF(INFO, VLOG_IS_ON(verboselevel))
#define VLOG_IF(verboselevel, condition) \
  LOG_IF(INFO, (condition) && VLOG_IS_ON(verboselevel))
#define VLOG_EVERY_N(verboselevel, n) \
  LOG_IF_EVERY_N(INFO, VLOG_IS_ON(verboselevel), n)
#define VLOG_IF_EVERY_N(verboselevel, condition, n) \
  LOG_IF_EVERY_N(INFO, (condition) && VLOG_IS_ON(verboselevel), n)

#define CHECK(condition) \
  LOG_IF(ERROR, !(condition)) << "Check failed: " #condition " "
#define CHECK_EQ(a, b) CHECK(a == b)
#define CHECK_GE(a, b) CHECK(a >= b)
#define CHECK_LT(a, b) CHECK(a < b)
#define CHECK_LE(a, b) CHECK(a <= b)

#else
// stl_logging.h needs to be before logging.h because template magic.
#include <glog/logging.h>
#include <glog/stl_logging.h>
#endif

#define ASSERT(x, ...)                                              \
  do {                                                              \
    if (!(x)) {                                                     \
      ESP_ERROR() << "Assert failed: " #x << "," << __FILE__ << ":" \
                  << __LINE__;                                      \
      exit(-1);                                                     \
    }                                                               \
  } while (false)

#endif  // ESP_CORE_LOGGING_H_
