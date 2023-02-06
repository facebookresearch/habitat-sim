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
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

namespace nvh {

struct Bbox {
  Bbox() = default;
  Bbox(nvmath::vec3f _min, nvmath::vec3f _max) : m_min(_min), m_max(_max) {}
  Bbox(const std::vector<nvmath::vec3f>& corners) {
    for (auto& c : corners) {
      insert(c);
    }
  }

  void insert(const nvmath::vec3f& v) {
    m_min = {std::min(m_min.x, v.x), std::min(m_min.y, v.y),
             std::min(m_min.z, v.z)};
    m_max = {std::max(m_max.x, v.x), std::max(m_max.y, v.y),
             std::max(m_max.z, v.z)};
  }

  void insert(const Bbox& b) {
    insert(b.m_min);
    insert(b.m_max);
  }

  inline Bbox& operator+=(float v) {
    m_min -= v;
    m_max += v;
    return *this;
  }

  inline bool isEmpty() const {
    return m_min == nvmath::vec3f{std::numeric_limits<float>::max()} ||
           m_max == nvmath::vec3f{std::numeric_limits<float>::lowest()};
  }

  inline uint32_t rank() const {
    uint32_t result{0};
    result += m_min.x < m_max.x;
    result += m_min.y < m_max.y;
    result += m_min.z < m_max.z;
    return result;
  }
  inline bool isPoint() const { return m_min == m_max; }
  inline bool isLine() const { return rank() == 1u; }
  inline bool isPlane() const { return rank() == 2u; }
  inline bool isVolume() const { return rank() == 3u; }
  inline nvmath::vec3f min() { return m_min; }
  inline nvmath::vec3f max() { return m_max; }
  inline nvmath::vec3f extents() { return m_max - m_min; }
  inline nvmath::vec3f center() { return (m_min + m_max) * 0.5f; }
  inline float radius() { return nvmath::length(m_max - m_min) * 0.5f; }

  Bbox transform(nvmath::mat4f mat) {
    // Make sure this is a 3D transformation + translation:
    auto r = mat.row(3);
    const float epsilon = 1e-6f;
    assert(fabs(r.x) < epsilon && fabs(r.y) < epsilon && fabs(r.z) < epsilon &&
           fabs(r.w - 1.0f) < epsilon);

    std::vector<nvmath::vec3f> corners(8);
    corners[0] = nvmath::vec3f(mat * nvmath::vec3f(m_min.x, m_min.y, m_min.z));
    corners[1] = nvmath::vec3f(mat * nvmath::vec3f(m_min.x, m_min.y, m_max.z));
    corners[2] = nvmath::vec3f(mat * nvmath::vec3f(m_min.x, m_max.y, m_min.z));
    corners[3] = nvmath::vec3f(mat * nvmath::vec3f(m_min.x, m_max.y, m_max.z));
    corners[4] = nvmath::vec3f(mat * nvmath::vec3f(m_max.x, m_min.y, m_min.z));
    corners[5] = nvmath::vec3f(mat * nvmath::vec3f(m_max.x, m_min.y, m_max.z));
    corners[6] = nvmath::vec3f(mat * nvmath::vec3f(m_max.x, m_max.y, m_min.z));
    corners[7] = nvmath::vec3f(mat * nvmath::vec3f(m_max.x, m_max.y, m_max.z));

    Bbox result(corners);
    return result;
  }

 private:
  nvmath::vec3f m_min{std::numeric_limits<float>::max()};
  nvmath::vec3f m_max{-std::numeric_limits<float>::max()};
};

template <typename T, typename TFlag>
inline bool hasFlag(T a, TFlag flag) {
  return (a & flag) == flag;
}

}  // namespace nvh
