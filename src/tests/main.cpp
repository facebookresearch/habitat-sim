// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include "esp/core/esp.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  LOG(INFO) << "Running all tests";

  return RUN_ALL_TESTS();
}
