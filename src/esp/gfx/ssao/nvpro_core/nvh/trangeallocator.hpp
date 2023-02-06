/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <algorithm>

#include <NvFoundation.h>  // for NV_X86 and NV_X64

#if (defined(NV_X86) || defined(NV_X64)) && defined(_MSC_VER)
#include <intrin.h>
#endif

namespace nvh {

/**
  \class nvh::TRangeAllocator

  The nvh::TRangeAllocator<GRANULARITY> template allows to sub-allocate ranges
  from a fixed maximum size. Ranges are allocated at GRANULARITY and are merged
  back on freeing. Its primary use is within allocators that sub-allocate from
  fixed-size blocks.

  The implementation is based on [MakeID by Emil
  Persson](http://www.humus.name/3D/MakeID.h).

  Example :

  \code{.cpp}
  TRangeAllocator<256> range;

  // initialize to a certain range
  range.init(range.alignedSize(128 * 1024 * 1024));

  ...

  // allocate a sub range
  // example
  uint32_t size = vertexBufferSize;
  uint32_t alignment = vertexAlignment;

  uint32_t allocOffset;
  uint32_t allocSize;
  uint32_t alignedOffset;

  if (range.subAllocate(size, alignment, allocOffset, alignedOffset, allocSize))
  {
    ... use the allocation space
    // [alignedOffset + size] is guaranteed to be within [allocOffset +
  allocSize]
  }

  // give back the memory range for re-use
  range.subFree(allocOffset, allocSize);

  ...

  // at the end cleanup
  range.deinit();
  \endcode
*/

// GRANULARITY must be power of two
template <uint32_t GRANULARITY = 256>
class TRangeAllocator {
 private:
  uint32_t m_size;
  uint32_t m_used;

 public:
  TRangeAllocator() : m_size(0), m_used(0) {}
  TRangeAllocator(uint32_t size) { init(size); }

  ~TRangeAllocator() { deinit(); }

  static uint32_t alignedSize(uint32_t size) {
    return (size + GRANULARITY - 1) & (~(GRANULARITY - 1));
  }

  void init(uint32_t size) {
    assert(size % GRANULARITY == 0 &&
           "managed total size must be aligned to GRANULARITY");

    uint32_t pages = ((size + GRANULARITY - 1) / GRANULARITY);
    rangeInit(pages - 1);
    m_used = 0;
    m_size = size;
  }
  void deinit() { rangeDeinit(); }

  bool isEmpty() const { return m_used == 0; }

  bool isAvailable(uint32_t size, uint32_t align) const {
    uint32_t alignRest = align - 1;
    uint32_t sizeReserved = size;

    if (m_used >= m_size) {
      return false;
    }

    if (m_used != 0 && align > GRANULARITY) {
      sizeReserved += alignRest;
    }

    uint32_t countReserved = (sizeReserved + GRANULARITY - 1) / GRANULARITY;
    return isRangeAvailable(countReserved);
  }

  bool subAllocate(uint32_t size,
                   uint32_t align,
                   uint32_t& outOffset,
                   uint32_t& outAligned,
                   uint32_t& outSize) {
    if (align == 0) {
      align = 1;
    }
    uint32_t alignRest = align - 1;
    uint32_t sizeReserved = size;
#if (defined(NV_X86) || defined(NV_X64)) && defined(_MSC_VER)
    bool alignIsPOT = __popcnt(align) == 1;
#else
    bool alignIsPOT = __builtin_popcount(align) == 1;
#endif

    if (m_used >= m_size) {
      outSize = 0;
      outOffset = 0;
      outAligned = 0;
      return false;
    }

    if (m_used != 0 && (alignIsPOT ? (align > GRANULARITY)
                                   : ((alignRest + size) > GRANULARITY))) {
      sizeReserved += alignRest;
    }

    uint32_t countReserved = (sizeReserved + GRANULARITY - 1) / GRANULARITY;

    uint32_t startID;
    if (createRangeID(startID, countReserved)) {
      outOffset = startID * GRANULARITY;
      outAligned = ((outOffset + alignRest) / align) * align;

      // due to custom alignment, we may be able to give
      // pages back that we over-allocated
      //
      // reserved:   [     |     |     |     ] (GRANULARITY spacing)
      // used:                [      ]         (custom alignment/size)
      // corrected:        [     |     ]       (GRANULARITY spacing)

      // correct start (warning could yield more fragmentation)

      uint32_t skipFront = (outAligned - outOffset) / GRANULARITY;
      if (skipFront) {
        destroyRangeID(startID, skipFront);
        outOffset += skipFront * GRANULARITY;
        startID += skipFront;
        countReserved -= skipFront;
      }

      assert(outOffset <= outAligned);

      // correct end
      uint32_t outLast = alignedSize(outAligned + size);
      outSize = outLast - outOffset;

      uint32_t usedCount = outSize / GRANULARITY;
      assert(usedCount <= countReserved);

      if (usedCount < countReserved) {
        destroyRangeID(startID + usedCount, countReserved - usedCount);
      }

      assert((outAligned + size) <= (outOffset + outSize));

      m_used += outSize;

      // checkRanges();

      return true;
    } else {
      outSize = 0;
      outOffset = 0;
      outAligned = 0;
      return false;
    }
  }

  void subFree(uint32_t offset, uint32_t size) {
    assert(offset % GRANULARITY == 0);
    assert(size % GRANULARITY == 0);

    m_used -= size;
    destroyRangeID(offset / GRANULARITY, size / GRANULARITY);

    // checkRanges();
  }

  TRangeAllocator& operator=(const TRangeAllocator& other) {
    m_size = other.m_size;
    m_used = other.m_used;

    m_Ranges = other.m_Ranges;
    m_Count = other.m_Count;
    m_Capacity = other.m_Capacity;
    m_MaxID = other.m_MaxID;

    if (m_Ranges) {
      m_Ranges = static_cast<Range*>(::malloc(m_Capacity * sizeof(Range)));
      memcpy(m_Ranges, other.m_Ranges, m_Capacity * sizeof(Range));
    }

    return *this;
  }

  TRangeAllocator(const TRangeAllocator& other) {
    m_size = other.m_size;
    m_used = other.m_used;

    m_Ranges = other.m_Ranges;
    m_Count = other.m_Count;
    m_Capacity = other.m_Capacity;
    m_MaxID = other.m_MaxID;

    if (m_Ranges) {
      m_Ranges = static_cast<Range*>(::malloc(m_Capacity * sizeof(Range)));
      assert(m_Ranges);  // Make sure allocation succeeded
      memcpy(m_Ranges, other.m_Ranges, m_Capacity * sizeof(Range));
    }
  }

  TRangeAllocator& operator=(TRangeAllocator&& other) {
    m_size = other.m_size;
    m_used = other.m_used;

    m_Ranges = other.m_Ranges;
    m_Count = other.m_Count;
    m_Capacity = other.m_Capacity;
    m_MaxID = other.m_MaxID;

    other.m_Ranges = nullptr;

    return *this;
  }

  TRangeAllocator(TRangeAllocator&& other) {
    m_size = other.m_size;
    m_used = other.m_used;

    m_Ranges = other.m_Ranges;
    m_Count = other.m_Count;
    m_Capacity = other.m_Capacity;
    m_MaxID = other.m_MaxID;

    other.m_Ranges = nullptr;
  }

 private:
  //////////////////////////////////////////////////////////////////////////
  // most of the following code is taken from Emil Persson's MakeID
  // http://www.humus.name/3D/MakeID.h (v1.02)

  struct Range {
    uint32_t m_First;
    uint32_t m_Last;
  };

  Range* m_Ranges = nullptr;  // Sorted array of ranges of free IDs
  uint32_t m_Count = 0;       // Number of ranges in list
  uint32_t m_Capacity = 0;    // Total capacity of range list
  uint32_t m_MaxID = 0;

 public:
  void rangeInit(const uint32_t max_id) {
    // Start with a single range, from 0 to max allowed ID (specified)
    m_Ranges = static_cast<Range*>(::malloc(sizeof(Range)));
    assert(m_Ranges != nullptr);  // Make sure allocation succeeded
    m_Ranges[0].m_First = 0;
    m_Ranges[0].m_Last = max_id;
    m_Count = 1;
    m_Capacity = 1;
    m_MaxID = max_id;
  }

  void rangeDeinit() {
    if (m_Ranges) {
      ::free(m_Ranges);
      m_Ranges = nullptr;
    }
  }

  bool createID(uint32_t& id) {
    if (m_Ranges[0].m_First <= m_Ranges[0].m_Last) {
      id = m_Ranges[0].m_First;

      // If current range is full and there is another one, that will become the
      // new current range
      if (m_Ranges[0].m_First == m_Ranges[0].m_Last && m_Count > 1) {
        destroyRange(0);
      } else {
        ++m_Ranges[0].m_First;
      }
      return true;
    }

    // No availble ID left
    return false;
  }

  bool createRangeID(uint32_t& id, const uint32_t count) {
    uint32_t i = 0;
    do {
      const uint32_t range_count = 1 + m_Ranges[i].m_Last - m_Ranges[i].m_First;
      if (count <= range_count) {
        id = m_Ranges[i].m_First;

        // If current range is full and there is another one, that will become
        // the new current range
        if (count == range_count && i + 1 < m_Count) {
          destroyRange(i);
        } else {
          m_Ranges[i].m_First += count;
        }
        return true;
      }
      ++i;
    } while (i < m_Count);

    // No range of free IDs was large enough to create the requested continuous
    // ID sequence
    return false;
  }

  bool destroyID(const uint32_t id) { return destroyRangeID(id, 1); }

  bool destroyRangeID(const uint32_t id, const uint32_t count) {
    const uint32_t end_id = id + count;

    assert(end_id <= m_MaxID + 1);

    // Binary search of the range list
    uint32_t i0 = 0;
    uint32_t i1 = m_Count - 1;

    for (;;) {
      const uint32_t i = (i0 + i1) / 2;

      if (id < m_Ranges[i].m_First) {
        // Before current range, check if neighboring
        if (end_id >= m_Ranges[i].m_First) {
          if (end_id != m_Ranges[i].m_First)
            return false;  // Overlaps a range of free IDs, thus (at least
                           // partially) invalid IDs

          // Neighbor id, check if neighboring previous range too
          if (i > i0 && id - 1 == m_Ranges[i - 1].m_Last) {
            // Merge with previous range
            m_Ranges[i - 1].m_Last = m_Ranges[i].m_Last;
            destroyRange(i);
          } else {
            // Just grow range
            m_Ranges[i].m_First = id;
          }
          return true;
        } else {
          // Non-neighbor id
          if (i != i0) {
            // Cull upper half of list
            i1 = i - 1;
          } else {
            // Found our position in the list, insert the deleted range here
            insertRange(i);
            m_Ranges[i].m_First = id;
            m_Ranges[i].m_Last = end_id - 1;
            return true;
          }
        }
      } else if (id > m_Ranges[i].m_Last) {
        // After current range, check if neighboring
        if (id - 1 == m_Ranges[i].m_Last) {
          // Neighbor id, check if neighboring next range too
          if (i < i1 && end_id == m_Ranges[i + 1].m_First) {
            // Merge with next range
            m_Ranges[i].m_Last = m_Ranges[i + 1].m_Last;
            destroyRange(i + 1);
          } else {
            // Just grow range
            m_Ranges[i].m_Last += count;
          }
          return true;
        } else {
          // Non-neighbor id
          if (i != i1) {
            // Cull bottom half of list
            i0 = i + 1;
          } else {
            // Found our position in the list, insert the deleted range here
            insertRange(i + 1);
            m_Ranges[i + 1].m_First = id;
            m_Ranges[i + 1].m_Last = end_id - 1;
            return true;
          }
        }
      } else {
        // Inside a free block, not a valid ID
        return false;
      }
    }
  }

  bool isRangeAvailable(uint32_t searchCount) const {
    uint32_t i = 0;
    do {
      uint32_t count = m_Ranges[i].m_Last - m_Ranges[i].m_First + 1;
      if (count >= searchCount)
        return true;

      ++i;
    } while (i < m_Count);

    return false;
  }

  void printRanges() const {
    uint32_t i = 0;
    for (;;) {
      if (m_Ranges[i].m_First < m_Ranges[i].m_Last)
        printf("%u-%u", m_Ranges[i].m_First, m_Ranges[i].m_Last);
      else if (m_Ranges[i].m_First == m_Ranges[i].m_Last)
        printf("%u", m_Ranges[i].m_First);
      else
        printf("-");

      ++i;
      if (i >= m_Count) {
        printf("\n");
        return;
      }

      printf(", ");
    }
  }

  void checkRanges() const {
    for (uint32_t i = 0; i < m_Count; i++) {
      assert(m_Ranges[i].m_Last <= m_MaxID);

      if (m_Ranges[i].m_First == m_Ranges[i].m_Last + 1) {
        continue;
      }
      assert(m_Ranges[i].m_First <= m_Ranges[i].m_Last);
      assert(m_Ranges[i].m_First <= m_MaxID);
    }
  }

  void insertRange(const uint32_t index) {
    if (m_Count >= m_Capacity) {
      m_Capacity += m_Capacity;
      m_Ranges = (Range*)realloc(m_Ranges, m_Capacity * sizeof(Range));
      assert(m_Ranges);  // Make sure reallocation succeeded
    }

    ::memmove(m_Ranges + index + 1, m_Ranges + index,
              (m_Count - index) * sizeof(Range));
    ++m_Count;
  }

  void destroyRange(const uint32_t index) {
    --m_Count;
    ::memmove(m_Ranges + index, m_Ranges + index + 1,
              (m_Count - index) * sizeof(Range));
  }
};

}  // namespace nvh
