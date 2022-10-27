// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_RANDOM_H_
#define ESP_CORE_RANDOM_H_

#include <random>

#include "Esp.h"

namespace esp {
namespace core {

class Random {
 public:
  explicit Random(unsigned int seed = std::random_device()())
      : gen_(seed),
        uniform_float_01_(0, 1),
        uniform_int_(),
        uniform_uint32_(),
        normal_float_01_(0, 1) {}

  //! Seed the random generator state with the given number
  void seed(uint32_t newSeed) { gen_.seed(newSeed); }

  //! Return randomly sampled int distributed uniformly in [0,
  //! std::numeric_limits<int>::max()]
  int uniform_int() { return uniform_int_(gen_); }

  //! Return randomly sampled uint32_t distributed uniformly in [0,
  //! std::numeric_limits<uint32_t>::max()]
  uint32_t uniform_uint() { return uniform_uint32_(gen_); }

  //! Return randomly sampled float distributed uniformly in [0, 1)
  float uniform_float_01() { return uniform_float_01_(gen_); }

  //! Return randomly sampled float distributed normally (mean=0, std=1)
  float normal_float_01() { return normal_float_01_(gen_); }

  //! Return randomly sampled float distributed uniformly in [a, b)
  float uniform_float(float a, float b) {
    return uniform_float_01_(gen_) * (b - a) + a;
  }

  //! Return randomly sampled int distributed uniformly in [a, b)
  int uniform_int(int a, int b) {
    return static_cast<int>(
        uniform_float(static_cast<float>(a), static_cast<float>(b)));
  }

 protected:
  std::default_random_engine gen_;
  std::uniform_real_distribution<float> uniform_float_01_;
  std::uniform_int_distribution<int> uniform_int_;
  std::uniform_int_distribution<uint32_t> uniform_uint32_;
  std::normal_distribution<float> normal_float_01_;

  ESP_SMART_POINTERS(Random)
};

}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_RANDOM_H_
