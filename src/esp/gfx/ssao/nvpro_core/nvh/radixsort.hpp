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

#ifndef NV_RADIXSORT_INCLUDED
#define NV_RADIXSORT_INCLUDED

namespace nvh {

/**
      \fn nvh::radixsort

      The radixsort function sorts the provided keys based on
      BYTES many bytes stored inside TKey starting at BYTEOFFSET.
      The sorting result is returned as indices into the keys array.

      For example:

      \code{.cpp}
      struct MyData {
        uint32_t objectIdentifier;
        uint16_t objectSortKey;
      };


      // 4-byte offset of objectSortKey within MyData
      // 2-byte size of sorting key

      result = radixsort<4,2>(keys, indicesIn, indicesTemp);

      // after sorting the following is true

      keys[result[i]].objectSortKey < keys[result[i + 1]].objectSortKey

      // result can point either to indicesIn or indicesTemp (we swap the arrays
      // after each byte iteration)
      \endcode
    */

template <uint32_t BYTEOFFSET, uint32_t BYTES, typename TKey>
uint32_t* radixsort(uint32_t numIndices,
                    const TKey* keys,
                    uint32_t* indicesIn,
                    uint32_t* indicesTemp) {
  uint32_t histogram[BYTES][256] = {0};

  for (uint32_t i = 0; i < numIndices; i++) {
    uint32_t idx = indicesIn[i];
    const uint8_t* bytes = (const uint8_t*)&keys[idx];
    for (uint32_t p = 0; p < BYTES; p++) {
      uint8_t curbyte = bytes[BYTEOFFSET + p];
      histogram[p][curbyte]++;
    }
  }

  uint32_t* tempIn = indicesIn;
  uint32_t* tempOut = indicesTemp;

  for (uint32_t p = 0; p < BYTES; p++) {
    uint32_t offset = 0;
    for (int32_t i = 0; i < 256; i++) {
      uint32_t numBin = histogram[p][i];
      histogram[p][i] = offset;
      offset += numBin;
    }

    for (uint32_t i = 0; i < numIndices; i++) {
      uint32_t idx = tempIn[i];
      const uint8_t* bytes = (const uint8_t*)&keys[idx];
      uint8_t curbyte = bytes[BYTEOFFSET + p];
      uint32_t pos = histogram[p][curbyte]++;
      tempOut[pos] = idx;
    }

    assert(histogram[p][255] == offset);

    // swap
    uint32_t* temp = tempIn;
    tempIn = tempOut;
    tempOut = temp;
  }

  // post swap tempIn is last tempOut
  return tempIn;
}

}  // namespace nvh

#endif
