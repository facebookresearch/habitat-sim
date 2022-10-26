// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_REDWOODNOISEMODEL_CUH_
#define ESP_SENSOR_REDWOODNOISEMODEL_CUH_

namespace esp {
namespace sensor {
namespace impl {

struct CurandStates;

CurandStates* getCurandStates();

void freeCurandStates(CurandStates* curandStates);

void simulateFromCPU(const int maxThreadsPerBlock,
                     const int warpSize,
                     const float* __restrict__ depth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     const float noiseMultiplier,
                     float* __restrict__ noisyDepth);

void simulateFromGPU(const int maxThreadsPerBlock,
                     const int warpSize,
                     const float* __restrict__ devDepth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     const float noiseMultiplier,
                     float* __restrict__ devNoisyDepth);
}  // namespace impl
}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_REDWOODNOISEMODEL_CUH_
