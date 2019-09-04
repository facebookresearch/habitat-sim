#pragma once

namespace esp {
namespace sensor {
namespace impl {

struct CurandStates;

CurandStates* getCurandStates();
void freeCurandStates(CurandStates* curandStates);
void simulateFromCPU(const float* __restrict__ depth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     float* __restrict__ noisyDepth);

void simulateFromGPU(const float* __restrict__ devDepth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     float* __restrict__ devNoisyDepth);
}  // namespace impl
}  // namespace sensor
}  // namespace esp
