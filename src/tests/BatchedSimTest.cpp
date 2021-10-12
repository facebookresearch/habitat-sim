// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/BatchedSimulator.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/GlmUtils.h"

#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <bps3D.hpp>
#include <glm/gtx/transform.hpp>

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
  esp::logging::LoggingContext loggingContext;

  BatchedSimulatorConfig config{
      .numEnvs = 1, .sensor0 = {.width = 256, .height = 128, .hfov = 45}};
  BatchedSimulator bsim(config);

  for (int i = 0; i < 100; i++) {
    bsim.autoResetOrStepPhysics();
    bsim.getRewards();
    bsim.getDones();
  }
  bsim.startRender();
  bsim.waitForFrame();

  uint8_t* base_color_ptr = bsim.getBpsRenderer().getColorPointer();
  float* base_depth_ptr = bsim.getBpsRenderer().getDepthPointer();

  // temp hack copied from BpsWrapper internals
  uint32_t batch_size = 1;
  glm::u32vec2 out_dim(config.sensor0.width, config.sensor0.height);

  for (int b = 0; b < config.numEnvs; b++) {
    saveFrame(("./out_color_" + std::to_string(b) + ".bmp").c_str(),
              base_color_ptr + b * out_dim.x * out_dim.y * 4, out_dim.x,
              out_dim.y, 4);
    saveFrame(("./out_depth_" + std::to_string(b) + ".bmp").c_str(),
              base_depth_ptr + b * out_dim.x * out_dim.y, out_dim.x, out_dim.y,
              1);
  }
}
