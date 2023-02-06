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
 * SPDX-FileCopyrightText: Copyright (c) 2014-2022 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include <array>

#define _USE_MATH_DEFINES
#include <math.h>

#include "primitives.hpp"

namespace nvh {
static uint32_t addPos(PrimitiveMesh& mesh, nvmath::vec3f p) {
  PrimitiveVertex v{};
  v.p = p;
  mesh.vertices.emplace_back(v);
  return static_cast<uint32_t>(mesh.vertices.size()) - 1;
}

static void addTriangle(PrimitiveMesh& mesh,
                        uint32_t a,
                        uint32_t b,
                        uint32_t c) {
  mesh.indices.push_back(a);
  mesh.indices.push_back(b);
  mesh.indices.push_back(c);
}

static void addTriangle(PrimitiveMesh& mesh,
                        nvmath::vec3f a,
                        nvmath::vec3f b,
                        nvmath::vec3f c) {
  mesh.indices.push_back(addPos(mesh, a));
  mesh.indices.push_back(addPos(mesh, b));
  mesh.indices.push_back(addPos(mesh, c));
}

static void faceted(PrimitiveMesh& mesh) {
  auto num_indices = static_cast<uint32_t>(mesh.indices.size());
  for (uint32_t i = 0; i < num_indices; i += 3) {
    auto& v0 = mesh.vertices[mesh.indices[i + 0]];
    auto& v1 = mesh.vertices[mesh.indices[i + 1]];
    auto& v2 = mesh.vertices[mesh.indices[i + 2]];

    auto n = nvmath::cross(nvmath::normalize(v1.p - v0.p),
                           nvmath::normalize(v2.p - v0.p));

    v0.n = n;
    v1.n = n;
    v2.n = n;
  }
}

PrimitiveMesh tetrahedron() {
  PrimitiveMesh mesh;

  // choose coordinates on the unit sphere
  float a = 1.0F / 3.0F;
  float b = sqrt(8.0F / 9.0F);
  float c = sqrt(2.0F / 9.0F);
  float d = sqrt(2.0F / 3.0F);

  // 4 vertices
  nvmath::vec3f v0 = nvmath::vec3f{0.0F, 1.0F, 0.0F} * 0.5F;
  nvmath::vec3f v1 = nvmath::vec3f{-c, -a, d} * 0.5F;
  nvmath::vec3f v2 = nvmath::vec3f{-c, -a, -d} * 0.5F;
  nvmath::vec3f v3 = nvmath::vec3f{b, -a, 0.0F} * 0.5F;

  // 4 triangles
  addTriangle(mesh, v0, v2, v1);
  addTriangle(mesh, v0, v3, v2);
  addTriangle(mesh, v0, v1, v3);
  addTriangle(mesh, v3, v1, v2);

  faceted(mesh);

  return mesh;
}

PrimitiveMesh icosahedron() {
  PrimitiveMesh mesh;

  float sq5 = sqrt(5.0F);
  float a = 2.0F / (1.0F + sq5);
  float b = sqrt((3.0F + sq5) / (1.0F + sq5));
  a /= b;
  float r = 0.5F;

  std::vector<nvmath::vec3f> v;
  v.emplace_back(0.0F, r * a, r / b);
  v.emplace_back(0.0F, r * a, -r / b);
  v.emplace_back(0.0F, -r * a, r / b);
  v.emplace_back(0.0F, -r * a, -r / b);
  v.emplace_back(r * a, r / b, 0.0F);
  v.emplace_back(r * a, -r / b, 0.0F);
  v.emplace_back(-r * a, r / b, 0.0F);
  v.emplace_back(-r * a, -r / b, 0.0F);
  v.emplace_back(r / b, 0.0F, r * a);
  v.emplace_back(r / b, 0.0F, -r * a);
  v.emplace_back(-r / b, 0.0F, r * a);
  v.emplace_back(-r / b, 0.0F, -r * a);

  addTriangle(mesh, v[1], v[6], v[4]);
  addTriangle(mesh, v[0], v[4], v[6]);
  addTriangle(mesh, v[0], v[10], v[2]);
  addTriangle(mesh, v[0], v[2], v[8]);
  addTriangle(mesh, v[1], v[9], v[3]);
  addTriangle(mesh, v[1], v[3], v[11]);
  addTriangle(mesh, v[2], v[7], v[5]);
  addTriangle(mesh, v[3], v[5], v[7]);
  addTriangle(mesh, v[6], v[11], v[10]);
  addTriangle(mesh, v[7], v[10], v[11]);
  addTriangle(mesh, v[4], v[8], v[9]);
  addTriangle(mesh, v[5], v[9], v[8]);
  addTriangle(mesh, v[0], v[6], v[10]);
  addTriangle(mesh, v[0], v[8], v[4]);
  addTriangle(mesh, v[1], v[11], v[6]);
  addTriangle(mesh, v[1], v[4], v[9]);
  addTriangle(mesh, v[3], v[7], v[11]);
  addTriangle(mesh, v[3], v[9], v[5]);
  addTriangle(mesh, v[2], v[10], v[7]);
  addTriangle(mesh, v[2], v[5], v[8]);

  faceted(mesh);

  return mesh;
}

PrimitiveMesh octahedron() {
  PrimitiveMesh mesh;

  std::vector<nvmath::vec3f> v;
  v.emplace_back(0.5F, 0.0F, 0.0F);
  v.emplace_back(-0.5F, 0.0F, 0.0F);
  v.emplace_back(0.0F, 0.5F, 0.0F);
  v.emplace_back(0.0F, -0.5F, 0.0F);
  v.emplace_back(0.0F, 0.0F, 0.5F);
  v.emplace_back(0.0F, 0.0F, -0.5F);

  addTriangle(mesh, v[0], v[2], v[4]);
  addTriangle(mesh, v[0], v[4], v[3]);
  addTriangle(mesh, v[0], v[5], v[2]);
  addTriangle(mesh, v[0], v[3], v[5]);
  addTriangle(mesh, v[1], v[4], v[2]);
  addTriangle(mesh, v[1], v[3], v[4]);
  addTriangle(mesh, v[1], v[5], v[3]);
  addTriangle(mesh, v[2], v[5], v[1]);

  faceted(mesh);
  return mesh;
}

PrimitiveMesh plane(uint32_t steps, float width, float depth) {
  PrimitiveMesh mesh;

  float increment = 1.0F / static_cast<float>(steps);
  for (uint32_t sz = 0; sz <= steps; sz++) {
    for (uint32_t sx = 0; sx <= steps; sx++) {
      PrimitiveVertex v{};

      v.p = nvmath::vec3f(-0.5F + (static_cast<float>(sx) * increment), 0.0F,
                          -0.5F + (static_cast<float>(sz) * increment));
      v.p *= nvmath::vec3f(width, 1.0F, depth);
      v.n = nvmath::vec3f(0.0F, 1.0F, 0.0F);
      v.t = nvmath::vec2f(
          static_cast<float>(sx) / static_cast<float>(steps),
          static_cast<float>(steps - sz) / static_cast<float>(steps));
      mesh.vertices.emplace_back(v);
    }
  }

  for (uint32_t sz = 0; sz < steps; sz++) {
    for (uint32_t sx = 0; sx < steps; sx++) {
      addTriangle(mesh, sx + sz * (steps + 1), sx + 1 + (sz + 1) * (steps + 1),
                  sx + 1 + sz * (steps + 1));
      addTriangle(mesh, sx + sz * (steps + 1), sx + (sz + 1) * (steps + 1),
                  sx + 1 + (sz + 1) * (steps + 1));
    }
  }

  return mesh;
}

PrimitiveMesh cube(float width /*= 1*/,
                   float height /*= 1*/,
                   float depth /*= 1*/) {
  PrimitiveMesh mesh;

  nvmath::vec3f s = nvmath::vec3f(width, height, depth) * 0.5F;
  std::vector<nvmath::vec3f> pnt = {{-s.x, -s.y, -s.z}, {-s.x, -s.y, s.z},
                                    {-s.x, s.y, -s.z},  {-s.x, s.y, s.z},
                                    {s.x, -s.y, -s.z},  {s.x, -s.y, s.z},
                                    {s.x, s.y, -s.z},   {s.x, s.y, s.z}};
  std::vector<nvmath::vec3f> nrm = {{-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F},
                                    {1.0F, 0.0F, 0.0F},  {0.0F, 0.0F, -1.0F},
                                    {0.0F, -1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};
  std::vector<nvmath::vec2f> uv = {
      {0.0F, 0.0F}, {0.0F, 1.0F}, {1.0F, 1.0F}, {1.0F, 0.0F}};

  // cube topology
  std::vector<std::vector<int>> cube_polygons = {{0, 1, 3, 2}, {1, 5, 7, 3},
                                                 {5, 4, 6, 7}, {4, 0, 2, 6},
                                                 {4, 5, 1, 0}, {2, 3, 7, 6}};

  for (int i = 0; i < 6; ++i) {
    auto index = static_cast<int32_t>(mesh.vertices.size());
    for (int j = 0; j < 4; ++j)
      mesh.vertices.push_back({pnt[cube_polygons[i][j]], nrm[i], uv[j]});
    addTriangle(mesh, index, index + 1, index + 2);
    addTriangle(mesh, index, index + 2, index + 3);
  }

  return mesh;
}

PrimitiveMesh sphere(float radius, int sectors, int stacks) {
  PrimitiveMesh mesh;

  float omega{0.0F};                 // rotation around the X axis
  float phi{0.0F};                   // rotation around the Y axis
  float length_inv = 1.0F / radius;  // vertex normal

  const float math_pi = static_cast<float>(M_PI);
  float sector_step = 2.0F * math_pi / static_cast<float>(sectors);
  float stack_step = math_pi / static_cast<float>(stacks);
  float sector_angle{0.0F};
  float stack_angle{0.0F};

  for (int i = 0; i <= stacks; ++i) {
    stack_angle =
        math_pi / 2.0F -
        static_cast<float>(i) * stack_step;  // starting from pi/2 to -pi/2
    phi = radius * cosf(stack_angle);        // r * cos(u)
    omega = radius * sinf(stack_angle);      // r * sin(u)

    // add (sectorCount+1) vertices per stack
    // the first and last vertices have same position and normal, but different
    // tex coords
    for (int j = 0; j <= sectors; ++j) {
      PrimitiveVertex v{};

      sector_angle =
          static_cast<float>(j) * sector_step;  // starting from 0 to 2pi

      // vertex position (x, y, z)
      v.p.x = phi * cosf(sector_angle);  // r * cos(u) * cos(v)
      v.p.z = phi * sinf(sector_angle);  // r * cos(u) * sin(v)
      v.p.y = omega;

      // normalized vertex normal
      v.n = v.p * length_inv;

      // vertex tex coord (s, t) range between [0, 1]
      v.t.x = 1.0F - static_cast<float>(j) / static_cast<float>(sectors);
      v.t.y = static_cast<float>(i) / static_cast<float>(stacks);

      mesh.vertices.emplace_back(v);
    }
  }

  // indices
  //  k2---k2+1
  //  | \  |
  //  |  \ |
  //  k1---k1+1
  uint32_t k1{0};
  uint32_t k2{0};
  for (int i = 0; i < stacks; ++i) {
    k1 = i * (sectors + 1);  // beginning of current stack
    k2 = k1 + sectors + 1;   // beginning of next stack

    for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
      // 2 triangles per sector excluding 1st and last stacks
      if (i != 0) {
        addTriangle(mesh, k1, k1 + 1, k2);  // k1---k2---k1+1
      }

      if (i != (stacks - 1)) {
        addTriangle(mesh, k1 + 1, k2 + 1, k2);  // k1+1---k2---k2+1
      }
    }
  }

  return mesh;
}

PrimitiveMesh cone(float radius, int sectors) {
  PrimitiveMesh mesh;

  const float math_pi = static_cast<float>(M_PI);
  float sector_step = 2.0F * math_pi / static_cast<float>(sectors);
  float sector_angle{0.0F};

  // length of the flank of the cone
  float flank_len = sqrtf(radius * radius + 1.0F);
  // unit vector along the flank of the cone
  float cone_x = radius / flank_len;
  float cone_y = -1.0F / flank_len;

  nvmath::vec3f tip = {0.0F, 0.5F, 0.0F};

  // Sides
  for (int i = 0; i <= sectors; ++i) {
    PrimitiveVertex v{};
    sector_angle = static_cast<float>(i) * sector_step;

    // Position
    v.p.x = radius * cosf(sector_angle);  // r * cos(u) * cos(v)
    v.p.z = radius * sinf(sector_angle);  // r * cos(u) * sin(v)
    v.p.y = -0.5F;
    // Normal
    v.n.x = -cone_y * cosf(sector_angle);
    v.n.y = cone_x;
    v.n.z = -cone_y * sinf(sector_angle);
    // TexCoord
    v.t.x = static_cast<float>(i) / static_cast<float>(sectors);
    v.t.y = 0.0F;
    mesh.vertices.emplace_back(v);

    // Tip point
    v.p = tip;
    // Normal
    sector_angle += 0.5F * sector_step;  // Half way to next triangle
    v.n.x = -cone_y * cosf(sector_angle);
    v.n.y = cone_x;
    v.n.z = -cone_y * sinf(sector_angle);
    // TexCoord
    v.t.x += 0.5F / static_cast<float>(sectors);
    v.t.y = 1.0F;

    mesh.vertices.emplace_back(v);
  }

  for (int j = 0; j < sectors; ++j) {
    uint32_t k1 = j * 2;
    addTriangle(mesh, k1, k1 + 1, k1 + 2);
  }

  // Bottom plate (normal are different)
  for (int i = 0; i <= sectors; ++i) {
    PrimitiveVertex v{};
    sector_angle =
        static_cast<float>(i) * sector_step;  // starting from 0 to 2pi

    v.p.x = radius * cosf(sector_angle);  // r * cos(u) * cos(v)
    v.p.z = radius * sinf(sector_angle);  // r * cos(u) * sin(v)
    v.p.y = -0.5F;
    //
    v.n = {0.0F, -1.0F, 0.0F};
    //
    v.t.x = static_cast<float>(i) / static_cast<float>(sectors);
    v.t.y = 0.0F;
    mesh.vertices.emplace_back(v);

    v.p = -tip;
    v.t.x += 0.5F / static_cast<float>(sectors);
    v.t.y = 1.0F;
    mesh.vertices.emplace_back(v);
  }

  for (int j = 0; j < sectors; ++j) {
    uint32_t k1 = (j + sectors + 1) * 2;
    addTriangle(mesh, k1, k1 + 2, k1 + 1);
  }

  return mesh;
}

}  // namespace nvh
