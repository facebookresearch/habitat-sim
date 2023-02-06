/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NV_MISC_INCLUDED
#define NV_MISC_INCLUDED

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <vector>

#include "nvprint.hpp"

/**
 # functions in nvh

 - mipMapLevels : compute number of mip maps
 - stringFormat : sprintf for std::string
 - frand : random float using rand()
 - permutation : fills uint vector with random permutation of values [0...
 vec.size-1]
 */

namespace nvh {

inline std::string stringFormat(const char* msg, ...) {
  va_list list;

  if (msg == 0)
    return std::string();

  // Speculate needed string size and vsnprintf to std::string.
  // If it was too small, we resize and try for the second (and final) time.
  std::string str;
  str.resize(64);

  for (int i = 0; i < 2; ++i) {
    va_start(list, msg);
    size_t charsNeeded = static_cast<size_t>(
        vsnprintf(const_cast<char*>(str.data()), str.size(), msg,
                  list));  // charsNeeded doesn't count \0
    va_end(list);

    if (charsNeeded < str.size()) {  // Not <= due to \0 terminator (which we
                                     // trim out of std::string)
      str.resize(charsNeeded);
      return str;
    } else {
      str.resize(charsNeeded + 1);  // Leave room for \0
    }
  }

  assert(!"String should have been resized perfectly second try");
  return std::string();
}

inline float frand() {
  return float(rand() % RAND_MAX) / float(RAND_MAX);
}

inline int mipMapLevels(int size) {
  int num = 0;
  while (size) {
    num++;
    size /= 2;
  }
  return num;
}

// permutation creates a random permutation of all integer values
// 0..data.size-1 occuring once within data.

inline void permutation(std::vector<unsigned int>& data) {
  size_t size = data.size();
  assert(size < RAND_MAX);

  for (size_t i = 0; i < size; i++) {
    data[i] = (unsigned int)(i);
  }

  for (size_t i = size - 1; i > 0; i--) {
    size_t other = rand() % (i + 1);
    std::swap(data[i], data[other]);
  }
}
}  // namespace nvh

#endif
