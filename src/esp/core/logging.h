// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#if defined(USE_GLOG_SHIM)

#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>

// Our own shims "emulating" GLOG
// TODO shims are hack to get things compiling, implement expected behaviors
#define INFO Corrade::Utility::Debug{}
#define ERROR Corrade::Utility::Error{}
#define WARNING Corrade::Utility::Warning{}
#define LOG(severity) severity
#define VLOG(severity) LOG(INFO)
#define CHECK(condition) { if (condition) ERROR << "Error"; }
#define CHECK_EQ(a, b) { if (a != b) ERROR << "Error"; }
#define CHECK_GE(a, b) { if (a < b) ERROR << "Error"; }
#define CHECK_LT(a, b) { if (a >= b) ERROR << "Error"; }
#define CHECK_LE(a, b) { if (a > b) ERROR << "Error"; }

#else
// stl_logging.h needs to be before logging.h because template magic.
#include <glog/stl_logging.h>
#include <glog/logging.h>
#endif

#define ASSERT(x, ...)                                                         \
  do {                                                                         \
    if (!(x)) {                                                                \
      LOG(ERROR) << "Assert failed: " #x << ", " << __FILE__ << ":" << __LINE__; \
      exit(-1);                                                                \
    }                                                                          \
  } while (false)
