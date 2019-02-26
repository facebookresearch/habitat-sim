// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <iostream>

// stl_logging.h needs to be before logging.h because template magic.
#include <glog/logging.h>
#include <glog/stl_logging.h>

#define ASSERT(x, ...)                                                         \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::cout << "Assert failed: " #x << ", " << __FILE__ << ":" << __LINE__ \
                << std::endl;                                                  \
      exit(-1);                                                                \
    }                                                                          \
  } while (false)
