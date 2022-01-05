// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_PROFILINGSCOPE_H_
#define ESP_BATCHEDSIM_PROFILINGSCOPE_H_

#include <cstdint>

namespace esp {
namespace batched_sim {

class ProfilingScope {
 public:
  ProfilingScope(const char* name);
  ~ProfilingScope();
 private:
  uint64_t handle_ = uint64_t(-1);
};


}  // namespace batched_sim
}  // namespace esp

#endif
