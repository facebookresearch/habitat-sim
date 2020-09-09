// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>

#include "esp/core/Configuration.h"
#include "esp/core/esp.h"

using namespace esp::core;

TEST(CoreTest, ConfigurationTest) {
  Configuration cfg;
  cfg.set("myInt", 10);
  cfg.set("myString", "test");
  EXPECT_TRUE(cfg.hasValue("myInt"));
  EXPECT_TRUE(cfg.hasValue("myString"));
  EXPECT_EQ(cfg.get<int>("myInt"), 10);
  EXPECT_EQ(cfg.get<std::string>("myString"), "test");
}
