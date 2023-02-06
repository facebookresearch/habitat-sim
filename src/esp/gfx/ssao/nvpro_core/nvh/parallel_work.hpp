/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

namespace nvh {
// distributes batches of loops over BATCHSIZE items across
// multiple threads. numItems reflects the total number
// of items to process.
// batches: fn (uint64_t itemIndex, uint32_t threadIndex)
//          callback does single item
// ranges:  fn (uint64_t itemBegin, uint64_t itemEnd, uint32_t threadIndex)
//          callback does loop `for (uint64_t itemIndex = itemBegin; itemIndex <
//          itemEnd; itemIndex++)`

template <uint64_t BATCHSIZE = 128>
inline void parallel_batches(uint64_t numItems,
                             std::function<void(uint64_t)> fn,
                             uint32_t numThreads) {
  if (numThreads <= 1 || numItems < numThreads || numItems < BATCHSIZE) {
    for (uint64_t idx = 0; idx < numItems; idx++) {
      fn(idx);
    }
  } else {
    std::atomic_uint64_t counter = 0;

    auto worker = [&]() {
      uint64_t idx;
      while ((idx = counter.fetch_add(BATCHSIZE)) < numItems) {
        uint64_t last = std::min(numItems, idx + BATCHSIZE);
        for (uint64_t i = idx; i < last; i++) {
          fn(i);
        }
      }
    };

    std::vector<std::thread> threads(numThreads);
    for (uint32_t i = 0; i < numThreads; i++) {
      threads[i] = std::thread(worker);
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      threads[i].join();
    }
  }
}

template <uint64_t BATCHSIZE = 128>
inline void parallel_batches(
    uint64_t numItems,
    std::function<void(uint64_t, uint32_t threadIdx)> fn,
    uint32_t numThreads) {
  if (numThreads <= 1 || numItems < numThreads || numItems < BATCHSIZE) {
    for (uint64_t idx = 0; idx < numItems; idx++) {
      fn(idx, 0);
    }
  } else {
    std::atomic_uint64_t counter = 0;

    auto worker = [&](uint32_t threadIdx) {
      uint64_t idx;
      while ((idx = counter.fetch_add(BATCHSIZE)) < numItems) {
        uint64_t last = std::min(numItems, idx + BATCHSIZE);
        for (uint64_t i = idx; i < last; i++) {
          fn(i, threadIdx);
        }
      }
    };

    std::vector<std::thread> threads(numThreads);
    for (uint32_t i = 0; i < numThreads; i++) {
      threads[i] = std::thread(worker, i);
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      threads[i].join();
    }
  }
}

template <uint64_t BATCHSIZE = 128>
inline void parallel_ranges(
    uint64_t numItems,
    std::function<void(uint64_t idxBegin, uint64_t idxEnd, uint32_t threadIdx)>
        fn,
    uint32_t numThreads) {
  if (numThreads <= 1 || numItems < numThreads || numItems < BATCHSIZE) {
    fn(0, numItems, 0);
  } else {
    std::atomic_uint64_t counter = 0;

    auto worker = [&](uint32_t threadIdx) {
      uint64_t idx;
      while ((idx = counter.fetch_add(BATCHSIZE)) < numItems) {
        uint64_t last = std::min(numItems, idx + BATCHSIZE);
        fn(idx, last, threadIdx);
      }
    };

    std::vector<std::thread> threads(numThreads);
    for (uint32_t i = 0; i < numThreads; i++) {
      threads[i] = std::thread(worker, i);
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      threads[i].join();
    }
  }
}
}  // namespace nvh
