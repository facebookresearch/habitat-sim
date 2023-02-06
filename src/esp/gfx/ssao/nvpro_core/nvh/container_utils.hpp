#ifndef NVPRO_CORE_NVH_CONTAINER_UTILS_HPP_
#define NVPRO_CORE_NVH_CONTAINER_UTILS_HPP_

#include <stddef.h>
#include <stdint.h>
#include <array>
#include <cassert>
#include <vector>

/// \nodoc (keyword to exclude this file from automatic README.md generation)

// constexpr array size functions for C and C++ style arrays.
// Truncated to 32-bits (with error checking) to support the common case in
// Vulkan.
template <typename T, size_t size>
constexpr uint32_t arraySize(const T (&)[size]) {
  constexpr uint32_t u32_size = static_cast<uint32_t>(size);
  static_assert(size == u32_size, "32-bit overflow");
  return u32_size;
}

template <typename T, size_t size>
constexpr uint32_t arraySize(const std::array<T, size>&) {
  constexpr uint32_t u32_size = static_cast<uint32_t>(size);
  static_assert(size == u32_size, "32-bit overflow");
  return u32_size;
}

// Checked 32-bit array size function for vectors.
template <typename T, typename Allocator>
constexpr uint32_t arraySize(const std::vector<T, Allocator>& vector) {
  auto size = vector.size();
  uint32_t u32_size = static_cast<uint32_t>(size);
  if (u32_size != size) {
    assert(!"32-bit overflow");
  }
  return u32_size;
}

#endif
