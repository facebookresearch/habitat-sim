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

#include "bitarray.hpp"

namespace nvh {
/** \brief Create a new BitVector.
 **/
BitArray::BitArray() : m_size(0), m_bits(NULL) {}

/** \brief Create a new BitVector with all bits set to false
      \param size Number of Bits in the Array
  **/
BitArray::BitArray(size_t size)
    : m_size(size), m_bits(new BitStorageType[determineNumberOfElements()]) {
  clear();
}

BitArray::BitArray(const BitArray& rhs)
    : m_size(rhs.m_size),
      m_bits(new BitStorageType[determineNumberOfElements()]) {
  std::copy(rhs.m_bits, rhs.m_bits + determineNumberOfElements(), m_bits);
}

BitArray::~BitArray() {
  delete[] m_bits;
}

void BitArray::resize(size_t newSize, bool defaultValue) {
  // if the default value for the new bits is true enabled the unused bits in
  // the last element.
  if (defaultValue) {
    setUnusedBits();
  }

  size_t oldNumberOfElements = determineNumberOfElements();
  m_size = newSize;
  size_t newNumberOfElements = determineNumberOfElements();

  // the number of elements has changed, reallocate array
  if (oldNumberOfElements != newNumberOfElements) {
    BitStorageType* NV_RESTRICT newBits =
        new BitStorageType[newNumberOfElements];
    if (newNumberOfElements < oldNumberOfElements) {
      std::copy(m_bits, m_bits + newNumberOfElements, newBits);
    } else {
      std::copy(m_bits, m_bits + oldNumberOfElements, newBits);
      std::fill(newBits + oldNumberOfElements, newBits + newNumberOfElements,
                defaultValue ? ~BitStorageType(0) : BitStorageType(0));
    }
    delete[] m_bits;
    m_bits = newBits;
  }
  clearUnusedBits();
}

BitArray& BitArray::operator=(const BitArray& rhs) {
  if (m_size != rhs.m_size) {
    m_size = rhs.m_size;
    delete[] m_bits;
    m_bits = new BitStorageType[determineNumberOfElements()];
  }
  std::copy(rhs.m_bits, rhs.m_bits + determineNumberOfElements(), m_bits);

  return *this;
}

bool BitArray::operator==(const BitArray& rhs) {
  return (m_size == rhs.m_size)
             ? std::equal(m_bits, m_bits + determineNumberOfElements(),
                          rhs.m_bits)
             : false;
}

BitArray BitArray::operator^(BitArray const& rhs) {
  NV_ASSERT(getSize() == rhs.getSize());

  BitArray result(getSize());
  for (size_t index = 0; index < determineNumberOfElements(); ++index) {
    result.m_bits[index] = m_bits[index] ^ rhs.m_bits[index];
  }
  clearUnusedBits();

  return result;
}

BitArray BitArray::operator|(BitArray const& rhs) {
  NV_ASSERT(getSize() == rhs.getSize());

  BitArray result(getSize());
  for (size_t index = 0; index < determineNumberOfElements(); ++index) {
    result.m_bits[index] = m_bits[index] | rhs.m_bits[index];
  }
  clearUnusedBits();

  return result;
}

BitArray BitArray::operator&(BitArray const& rhs) {
  NV_ASSERT(getSize() == rhs.getSize());

  BitArray result(getSize());
  for (size_t index = 0; index < determineNumberOfElements(); ++index) {
    result.m_bits[index] = m_bits[index] & rhs.m_bits[index];
  }
  clearUnusedBits();

  return result;
}

BitArray& BitArray::operator^=(BitArray const& rhs) {
  NV_ASSERT(getSize() == rhs.getSize());

  for (size_t index = 0; index < determineNumberOfElements(); ++index) {
    m_bits[index] ^= rhs.m_bits[index];
  }
  clearUnusedBits();

  return *this;
}

BitArray& BitArray::operator|=(BitArray const& rhs) {
  NV_ASSERT(getSize() == rhs.getSize());

  for (size_t index = 0; index < determineNumberOfElements(); ++index) {
    m_bits[index] |= rhs.m_bits[index];
  }

  return *this;
}

BitArray& BitArray::operator&=(BitArray const& rhs) {
  NV_ASSERT(getSize() == rhs.getSize());

  for (size_t index = 0; index < determineNumberOfElements(); ++index) {
    m_bits[index] &= rhs.m_bits[index];
  }

  return *this;
}

void BitArray::clear() {
  std::fill(m_bits, m_bits + determineNumberOfElements(), 0);
}

void BitArray::fill() {
  if (determineNumberOfElements()) {
    std::fill(m_bits, m_bits + determineNumberOfElements(), ~0);

    clearUnusedBits();
  }
}

size_t BitArray::countLeadingZeroes() const {
  size_t index = 0;

  // first count
  while (index < determineNumberOfElements() && !m_bits[index]) {
    ++index;
  }

  size_t leadingZeroes = index * StorageBitsPerElement;
  if (index < determineNumberOfElements()) {
    leadingZeroes += ctz(m_bits[index]);
  }

  return std::min(leadingZeroes, getSize());
}

}  // namespace nvh
