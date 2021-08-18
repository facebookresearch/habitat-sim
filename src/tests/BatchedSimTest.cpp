// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>

#include "esp/batched_sim/BatchedSimulator.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/GlmUtils.h"

#include <glm/gtx/transform.hpp>

#include <cuda_runtime.h>

#include <bps3D.hpp>

// FIXME
// #define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

using namespace esp::batched_sim;

namespace {
template <typename T>
static std::vector<T> copyToHost(const T* dev_ptr,
                                 uint32_t width,
                                 uint32_t height,
                                 uint32_t num_channels) {
  uint64_t num_pixels = width * height * num_channels;

  std::vector<T> buffer(num_pixels);

  cudaMemcpy(buffer.data(), dev_ptr, sizeof(T) * num_pixels,
             cudaMemcpyDeviceToHost);

  return buffer;
}

void saveFrame(const char* fname,
               const float* dev_ptr,
               uint32_t width,
               uint32_t height,
               uint32_t num_channels) {
  auto buffer = copyToHost(dev_ptr, width, height, num_channels);

  std::vector<uint8_t> sdr_buffer(buffer.size());
  for (unsigned i = 0; i < buffer.size(); i++) {
    float v = buffer[i];
    if (v < 0)
      v = 0;
    if (v > 1)
      v = 1;
    sdr_buffer[i] = v * 255;
  }

  stbi_write_bmp(fname, width, height, num_channels, sdr_buffer.data());
}

void saveFrame(const char* fname,
               const uint8_t* dev_ptr,
               uint32_t width,
               uint32_t height,
               uint32_t num_channels) {
  auto buffer = copyToHost(dev_ptr, width, height, num_channels);

  stbi_write_bmp(fname, width, height, num_channels, buffer.data());
}

}  // namespace

class BatchedSimulatorTest : public ::testing::Test {};

TEST_F(BatchedSimulatorTest, basic) {
  BatchedSimulator bsim;

  bsim.stepPhysics();
  bsim.startRender();
  bsim.waitForFrame();

  uint8_t* base_color_ptr = bsim.debugGetBpsRenderer()->getColorPointer();
  float* base_depth_ptr = bsim.debugGetBpsRenderer()->getDepthPointer();

  // temp hack copied from BpsWrapper internals
  uint32_t batch_size = 11;
  glm::u32vec2 out_dim(1024, 1024);

  for (uint32_t batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    saveFrame(("./out_color_" + std::to_string(batch_idx) + ".bmp").c_str(),
              base_color_ptr + batch_idx * out_dim.x * out_dim.y * 4, out_dim.x,
              out_dim.y, 4);
    saveFrame(("./out_depth_" + std::to_string(batch_idx) + ".bmp").c_str(),
              base_depth_ptr + batch_idx * out_dim.x * out_dim.y, out_dim.x,
              out_dim.y, 1);
  }
}
