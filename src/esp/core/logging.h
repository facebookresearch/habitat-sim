// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/configure.h"

#if defined(USE_GLOG_SHIM)

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
#define GLOG_ERROR \
  Corrade::Utility::Error {}
#define GLOG_WARNING \
  Corrade::Utility::Warning {}
#define LOG(severity) GLOG_##severity
#define VLOG(severity) LOG(INFO)
#define LOG_IF(severity, condition) \
  !(condition) ? (void)0 : LogMessageVoidify() & LOG(severity)
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
      LOG(ERROR) << "Assert failed: " #x << ", " << __FILE__ << ":" \
                 << __LINE__;                                       \
      exit(-1);                                                     \
    }                                                               \
  } while (false)
