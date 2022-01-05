// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ProfilingScope.h"

#define ENABLE_NVTX

#ifdef ENABLE_NVTX
#include <nvtx3/nvToolsExt.h>
#endif

namespace esp {
namespace batched_sim {

ProfilingScope::ProfilingScope(const char* name) {
#ifdef ENABLE_NVTX
  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.message.ascii = name;
  auto retVal = nvtxRangeStartEx(&eventAttrib);
  static_assert(sizeof(retVal) == sizeof(handle_));
  handle_ = retVal;
#endif
}

ProfilingScope::~ProfilingScope() {
#ifdef ENABLE_NVTX
  nvtxRangeEnd(handle_);
#endif
}


}  // namespace batched_sim
}  // namespace esp
