// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_LOGGING_H_
#define ESP_CORE_LOGGING_H_

#include "esp/core/configure.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/String.h>
#include <Corrade/Containers/StringView.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/String.h>
#include <Magnum/Magnum.h> /* for Magnum::Debug alias, mainly */

namespace esp {
namespace logging {
/**
 * @brief Habitat-Sim logging subsystems.
 *
 * These are all subsystems that have different logging levels. They correspond
 * to namespaces.  When adding a new subsystem, make sure to add an appropriate
 * loggingSubsystem function (see the @ref ESP_ADD_SUBSYSTEM_FN helper bellow)
 * and add it to @ref subsystemNames.
 */
enum class Subsystem : uint8_t {
  /**
   * @brief Catch all subsystem that is used when no subsystem is specified
   */
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
  core,
  assets,
  sensor,
  agent,

  /**
   * @brief Not a subsystem.  Simply tracks how many subsystems there are.
   *
   * Must always be last
   */
  NumSubsystems,
};

constexpr const char* subsystemNames[] = {
    "Default", "Gfx", "Scene", "Sim",  "Physics", "Nav",    "Metadata",
    "Geo",     "IO",  "URDF",  "Core", "Assets",  "Sensor", "Agent",
};

Subsystem subsystemFromName(Corrade::Containers::StringView name);

}  // namespace logging
}  // namespace esp

/**
 * @brief Top level logging subsystem function allowing logging macros to work
 * outside the esp namespace
 *
 * Uses c style namespacing that way logging can work outside our namespace.
 *
 *
 * There will be many of these functions in different namespaces as we use
 * namespace resolution to determine the logging subsystem for a given logging
 * macro.
 */
inline esp::logging::Subsystem espLoggingSubsystem() {
  return esp::logging::Subsystem::Default;
}

/**
 * @brief Helper macro to add functions for determining the current subsystem.
 *
 * This macro assumes that the namespace for a subsystem is esp::<subsystemName>
 */
#define ESP_ADD_SUBSYSTEM_FN(subsystemName)         \
  namespace esp {                                   \
  namespace subsystemName {                         \
  inline logging::Subsystem espLoggingSubsystem() { \
    return logging::Subsystem::subsystemName;       \
  }                                                 \
  }                                                 \
  }

ESP_ADD_SUBSYSTEM_FN(gfx)
ESP_ADD_SUBSYSTEM_FN(scene)
ESP_ADD_SUBSYSTEM_FN(sim)
ESP_ADD_SUBSYSTEM_FN(physics)
ESP_ADD_SUBSYSTEM_FN(nav)
ESP_ADD_SUBSYSTEM_FN(metadata)
ESP_ADD_SUBSYSTEM_FN(geo)
ESP_ADD_SUBSYSTEM_FN(io)
ESP_ADD_SUBSYSTEM_FN(core)
ESP_ADD_SUBSYSTEM_FN(assets)
ESP_ADD_SUBSYSTEM_FN(sensor)
ESP_ADD_SUBSYSTEM_FN(agent)

namespace esp {
namespace metadata {
namespace URDF {
inline logging::Subsystem espLoggingSubsystem() {
  return logging::Subsystem::URDF;
}
}  // namespace URDF
}  // namespace metadata
}  // namespace esp

namespace esp {
namespace logging {

/**
 * @brief Habitat-Sim logging levels. If the logging level for log macro is
 * greater than or equal to the logging level within that subsystem, the log
 * will be printed.
 *
 * The convenience levels, @ref Verbose, @ref Default and @ref Quiet are the primary user facing
 * options.  @ref Quiet turns off logging for all but errors, @ref Default is the default and
 * shows warnings and errors, while @ref Verbose gives debug messages along with warnings and
 * errors. These names exist such that logging levels can be added and removed
 * without users needing to change what they set logging to.
 */
enum class LoggingLevel : uint8_t {
  /**
   * Logging level to be used to provide in depth information about a process
   * and its result.
   */
  VeryVerbose,

  /**
   * Logging level to be used to specify the result of some process.
   */
  Debug,
  /**
   * Logging level to be used to denote that some process has yielded a result
   * that may not be appropriate or expected.
   */
  Warning,
  /**
   * Logging level to be used to denote that some process has failed.
   */
  Error,

  /**
   * Enables all logging except for the most verbose statements. Habitat-Sim
   * will be fairly verbose in this setting, but not overly so.
   */
  Verbose = Debug,
  /**
   * The default logging level. Enables all warning and error messages but hides
   * debug and verbose messages.
   */
  Default = Warning,
  /**
   * Turns of all non-critical logging messages, i.e. those that indicate that
   * something has gone wrong.
   */
  Quiet = Error,
};

LoggingLevel levelFromName(Corrade::Containers::StringView name);

/**
 * @brief Logging context that tracks which logging statements are enabled.
 *
 * The logging system in habitat-sim can be configured in a variety of ways to
 * determine which logging statements should be printed.
 *
 *
 * Due to the use of @ref ESP_LOG_IF, logging statements that are not printed
 * are as close to free as reasonably possible.
 */
class LoggingContext {
 public:
  constexpr static const char* LOGGING_ENV_VAR_NAME = "HABITAT_SIM_LOG";
  constexpr static LoggingLevel DEFAULT_LEVEL = LoggingLevel::Default;

  /**
   * @brief Convenience constructor that grabs the configuration string from the
   * environment variable and calls @ref
   * LoggingContext(Corrade::Containers::StringView)
   */
  LoggingContext();

  /**
   * @brief Constructor.
   *
   * Processes the environment variable string that configures the habitat-sim
   * logging levels for various subsystems
   *
   * This environment string has a fairly simple grammar that is as follows
   *
   * @code
   *    FilterString: SetLevelCommand (COLON SetLevelCommand)*
   *    SetLevelCommand: (SUBSYSTEM (COMMA SUBSYSTEM)* EQUALS)? LOGGING_LEVEL
   * @endcode
   *
   * where SUBSYSTEM is a known logging subsystem and LOGGING_LEVEL is one of
   * the logging levels.  If a SetLevelCommand does not contain a list of
   * subsystems, that level is applied to all subsystems.
   *
   *
   * A logging statement is printed if it's logging level has a higher or equal
   * priority to the current logging level for it's subsystem.  verbose is the
   * lowest level. quiet disables debug and warnings.
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
   * @brief Retrieves the logging level for the given subsystem
   */
  LoggingLevel levelFor(Subsystem subsystem) const;

  /**
   * @brief Whether or not there is a current context or not.  Whichever context
   * class was most recently constructed is the current one.
   */
  static bool hasCurrent();
  /**
   * @brief Retrieves the current logging context.  Throws an error if there
   * isn't one.
   */
  static const LoggingContext& current();

 private:
  Corrade::Containers::Array<LoggingLevel> loggingLevels_;
  const LoggingContext* prevContext_;
};

/**
 * @brief Determine if the specified logging level is enabled within a given
 * subsystem
 *
 * Requires an active @ref LoggingContext
 *
 * @param[in] subsystem The name of the subsystem. Typically this is returned by
 * @ref espLoggingSubsystem
 * @param[in] level The logging level
 */
bool isLevelEnabled(Subsystem subsystem, LoggingLevel level);

/**
 * @brief Build appropriate prefix for logging messages, including
 * subsystem/namespace, file, line number and function name
 */
Corrade::Containers::String buildMessagePrefix(Subsystem subsystem,
                                               const std::string& msgLevel,
                                               const std::string& filename,
                                               const std::string& function,
                                               int line);

namespace impl {
class LogMessageVoidify {
 public:
  // This has to be an operator with a precedence lower than << but
  // higher than ?:
  void operator&(Corrade::Utility::Debug&) {}
};
}  // namespace impl
}  // namespace logging
}  // namespace esp

/**
 * @brief Magic macro that creates a conditional logging statement
 *
 * The intent of this macro is actually rather simple (although the
 * implementation is quite opaque).  This macro is just
 *
 * @code
 *   if (condition) output
 * @endcode
 *
 * This is a "so simple it's obvious" way to implement a conditional logging
 * macro.  However, the if statement implementation would potentially interact
 * with other parts of the code.  While certianly an edge case, the following
 * should not be valid code:
 *
 * @code
 *   ESP_DEBUG() << ... ;
 *   else {
 *      ...
 *   }
 * @endcode
 *
 * So instead, a ternary is used to encapsulate everything as a single
 * expression.  Since a ternary requires both a true and false clause and the
 * false clause comes second, the condition is negated.
 *
 * The final piece of the puzzle is how to have both the true and false clause
 * return the same thing (a return of void is used to avoid unused value
 * compiler warnings). The `LogMessageVoidify{} & output` does this as
 * `LogMessageVoidify` has an `operator&` overload that returns void.  Any
 * operator that is lower precedence than `operator<<` and higher than `?:`
 * would work for this overload.  That way the logging statements get evaluated
 * first, then the voidify, then the ternary.
 */

// No formatting on this macro as the NOLINT for clang
#define ESP_LOG_IF(condition, output)                                         \
  !(condition)                                                                \
      ? static_cast<void>(0) /* NOLINTNEXTLINE(bugprone-macro-parentheses) */ \
      : esp::logging::impl::LogMessageVoidify{} & (output)

// This ends with a nospace since the space is baked in to subsystemPrefix for
// the case that the logger was created with a nospace flag.
#define ESP_SUBSYS_LOG_IF(subsystem, level, output, levelMsg)                  \
  ESP_LOG_IF(esp::logging::isLevelEnabled((subsystem), (level)), (output))     \
      << esp::logging::buildMessagePrefix((subsystem), (levelMsg), (__FILE__), \
                                          (__FUNCTION__), (__LINE__))          \
      << Corrade::Utility::Debug::nospace

#define ESP_LOG_LEVEL_ENABLED(level) \
  esp::logging::isLevelEnabled(espLoggingSubsystem(), (level))

/**
 * @brief Very verbose level logging macro.
 */
#define ESP_VERY_VERBOSE(...)                                            \
  ESP_SUBSYS_LOG_IF(                                                     \
      espLoggingSubsystem(), esp::logging::LoggingLevel::VeryVerbose,    \
      (Corrade::Utility::Debug{Corrade::Utility::Debug::defaultOutput(), \
                               __VA_ARGS__}),                            \
      "Verbose")
/**
 * @brief Debug level logging macro.
 */
#define ESP_DEBUG(...)                                                        \
  ESP_SUBSYS_LOG_IF(espLoggingSubsystem(), esp::logging::LoggingLevel::Debug, \
                    Corrade::Utility::Debug{__VA_ARGS__}, "Debug")
/**
 * @brief Warning level logging macro.
 */
#define ESP_WARNING(...)                                 \
  ESP_SUBSYS_LOG_IF(espLoggingSubsystem(),               \
                    esp::logging::LoggingLevel::Warning, \
                    Corrade::Utility::Warning{__VA_ARGS__}, "Warning")
/**
 * @brief Error level logging macro.
 */
#define ESP_ERROR(...)                                                        \
  ESP_SUBSYS_LOG_IF(espLoggingSubsystem(), esp::logging::LoggingLevel::Error, \
                    Corrade::Utility::Error{__VA_ARGS__}, "Error")

#endif  // ESP_CORE_LOGGING_H_
