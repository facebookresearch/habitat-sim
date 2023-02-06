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

#ifndef NV_BITARRAY_H__
#define NV_BITARRAY_H__

#include <algorithm>
#include "../nvp/platform.h"
#if (defined(NV_X86) || defined(NV_X64)) && defined(_MSC_VER)
#include <intrin.h>
#endif

namespace nvh {

//////////////////////////////////////////////////////////////////////////
/**
    \class nvh::BitArray

    \brief The nvh::BitArray class implements a tightly packed boolean array
   using single bits stored in uint64_t values. Whenever you want large boolean
   arrays this representation is preferred for cache-efficiency. The Visitor and
   OffsetVisitor traversal mechanisms make use of cpu intrinsics to speed up
   iteration over bits.

    Example:
    \code{.cpp}
    BitArray modifiedObjects(1024);

    // set some bits
    modifiedObjects.setBit(24,true);
    modifiedObjects.setBit(37,true);

    // iterate over all set bits using the built-in traversal mechanism

    struct MyVisitor {
    void operator()( size_t index ){
        // called with the index of a set bit
        myObjects[index].update();
      }
    };

    MyVisitor visitor;
    modifiedObjects.traverseBits(visitor);
    \endcode
  */

/** \brief Visitor which forwards the visitor operator with a fixed offset **/
template <typename Visitor>
struct OffsetVisitor {
  inline OffsetVisitor(Visitor& visitor, size_t offset)
      : m_visitor(visitor), m_offset(offset) {}

  inline void operator()(size_t index) { m_visitor(index + m_offset); }

 private:
  Visitor& m_visitor;
  size_t m_offset;
};

#if (defined(NV_X86) || defined(NV_X64)) && defined(_MSC_VER)
template <typename Visitor>
inline void bitTraverse(uint32_t bits, Visitor& visitor) {
  unsigned long localIndex;
  while (_BitScanForward(&localIndex, bits)) {
    visitor(localIndex);
    bits ^= 1 << localIndex;  // clear the current bit so that the next one is
                              // being found by the bitscan
  }
}

template <typename Visitor>
inline void bitTraverse(uint64_t bits, Visitor& visitor) {
  unsigned long localIndex;
  while (_BitScanForward64(&localIndex, bits)) {
    visitor(localIndex);
    bits ^=
        uint64_t(1) << localIndex;  // clear the current bit so that the next
                                    // one is being found by the bitscan
  }
}

inline size_t ctz(uint64_t bits) {
  unsigned long localIndex;
  return _BitScanForward64(&localIndex, bits) ? localIndex : 64;
}

inline size_t ctz(uint32_t bits) {
  unsigned long localIndex;
  return _BitScanForward(&localIndex, bits) ? localIndex : 32;
}
#else
inline size_t ctz(uint64_t bits) {
  return (bits != 0) ? __builtin_ctzl(bits) : 64;
}

inline size_t ctz(uint32_t bits) {
  return (bits != 0) ? __builtin_ctz(bits) : 32;
}

// TODO implement GCC version!
template <typename BitType, typename Visitor>
inline void bitTraverse(BitType bits, Visitor visitor) {
  size_t index = 0;
  while (bits) {
    if (bits & 0xff)  // skip ifs if the byte is 0
    {
      if (bits & 0x01)
        visitor(index + 0);
      if (bits & 0x02)
        visitor(index + 1);
      if (bits & 0x04)
        visitor(index + 2);
      if (bits & 0x08)
        visitor(index + 3);
      if (bits & 0x10)
        visitor(index + 4);
      if (bits & 0x20)
        visitor(index + 5);
      if (bits & 0x40)
        visitor(index + 6);
      if (bits & 0x80)
        visitor(index + 7);
    }
    bits >>= 8;
    index += 8;
  }
}
#endif

/** \brief Call visitor(index) for each bit set **/
template <typename BitType, typename Visitor>
inline void bitTraverse(BitType* elements,
                        size_t numberOfElements,
                        Visitor& visitor) {
  size_t baseIndex = 0;
  for (size_t elementIndex = 0; elementIndex < numberOfElements;
       ++elementIndex) {
    OffsetVisitor<Visitor> offsetVisitor(visitor, baseIndex);
    bitTraverse(elements[elementIndex], offsetVisitor);
    baseIndex += sizeof(*elements) * 8;
  }
}

class BitArray {
 public:
  typedef uint64_t BitStorageType;
  enum { StorageBitsPerElement = sizeof(BitStorageType) * 8 };

  BitArray();
  BitArray(size_t size);
  BitArray(const BitArray& rhs);
  ~BitArray();

  BitArray& operator=(const BitArray& rhs);
  bool operator==(const BitArray& rhs);
  BitArray operator^(BitArray const& rhs);
  BitArray operator&(BitArray const& rhs);
  BitArray operator|(BitArray const& rhs);
  BitArray& operator^=(BitArray const& rhs);
  BitArray& operator&=(BitArray const& rhs);
  BitArray& operator|=(BitArray const& rhs);

  void clear();
  void fill();

  /** \brief Change the number of bits in this array. The state of remaining
    bits is being kept. New bits will be initialized to false. \param size New
    number of bits in this array \param defaultValue The new default value for
    the new bits
    **/
  void resize(size_t size, bool defaultValue = false);

  size_t getSize() const { return m_size; }

  // inline functions
  void enableBit(size_t index);
  void disableBit(size_t index);
  void setBit(size_t index, bool value);
  bool getBit(size_t index) const;

  BitStorageType const* getBits() const;

  template <typename Visitor>
  void traverseBits(Visitor visitor);

  size_t countLeadingZeroes() const;

 private:
  size_t m_size;
  BitStorageType* NV_RESTRICT m_bits;

  void determineBitPosition(size_t index, size_t& element, size_t& bit) const;
  size_t determineNumberOfElements() const;

  /** \brief Clear the last unused bits in the last element.
        \remarks Clear bits whose number is >= m_size. those are traversed
    unconditional and would produce invalid results. restrict shifting range to
    0 to StorageBitsPerElement - 1 to handle the case usedBitsInLastElement==0
                 which would result in shifting StorageBitsPerElement which is
    undefined by the standard and not the desired operation.
    **/
  void clearUnusedBits();

  /** \brief Set the last unused bits in the last element.
        \remarks Set bits whose number is >= m_size. This is required when
    expanding the vector with the bits set to true.
    **/
  void setUnusedBits();
};

/** \brief Determine the element / bit for the given index **/
inline void BitArray::determineBitPosition(size_t index,
                                           size_t& element,
                                           size_t& bit) const {
  element = index / StorageBitsPerElement;
  bit = index % StorageBitsPerElement;
}

inline size_t BitArray::determineNumberOfElements() const {
  return (m_size + StorageBitsPerElement - 1) / StorageBitsPerElement;
}

inline void BitArray::enableBit(size_t index) {
  NV_ASSERT(index < m_size);
  size_t element;
  size_t bit;
  determineBitPosition(index, element, bit);
  m_bits[element] |= BitStorageType(1) << bit;
}

inline void BitArray::disableBit(size_t index) {
  NV_ASSERT(index < m_size);

  size_t element;
  size_t bit;
  determineBitPosition(index, element, bit);
  m_bits[element] &= ~(BitStorageType(1) << bit);
}

inline void BitArray::setBit(size_t index, bool value) {
  NV_ASSERT(index < m_size);
  if (value) {
    enableBit(index);
  } else {
    disableBit(index);
  }
}

inline BitArray::BitStorageType const* BitArray::getBits() const {
  return m_bits;
}

inline bool BitArray::getBit(size_t index) const {
  NV_ASSERT(index < m_size);
  size_t element;
  size_t bit;
  determineBitPosition(index, element, bit);
  return !!(m_bits[element] & (BitStorageType(1) << bit));
}

/** \brief call Visitor( size_t index ) on all bits which are set. **/
template <typename Visitor>
inline void BitArray::traverseBits(Visitor visitor) {
  bitTraverse(m_bits, determineNumberOfElements(), visitor);
}

inline void BitArray::clearUnusedBits() {
  if (m_size) {
    size_t usedBitsInLastElement = m_size % StorageBitsPerElement;
    m_bits[determineNumberOfElements() - 1] &=
        ~BitStorageType(0) >> ((StorageBitsPerElement - usedBitsInLastElement) &
                               (StorageBitsPerElement - 1));
  }
}

inline void BitArray::setUnusedBits() {
  if (m_size) {
    size_t usedBitsInLastElement = m_size % StorageBitsPerElement;
    m_bits[determineNumberOfElements() - 1] |= ~BitStorageType(0)
                                               << usedBitsInLastElement;
  }
}
}  // namespace nvh

#endif
