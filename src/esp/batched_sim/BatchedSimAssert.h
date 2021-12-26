// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_BATCHEDSIMASSERT_H_
#define ESP_BATCHEDSIM_BATCHEDSIMASSERT_H_

#include <Corrade/Utility/Assert.h>

#ifdef NDEBUG
#define BATCHED_SIM_ASSERT(expr) do {} while(0)
#else
#define BATCHED_SIM_ASSERT(expr) CORRADE_INTERNAL_ASSERT(expr)
#endif

#define BATCHED_SIM_ASSERT_VECTOR_ACCESS(vec, i)  BATCHED_SIM_ASSERT(i >= 0 && i < vec.size())

template<typename VectorType, typename IndexType>
typename VectorType::value_type& safeVectorGet(VectorType& vec, IndexType index) {
  BATCHED_SIM_ASSERT_VECTOR_ACCESS(vec, index);
  return vec[index];
}

template<typename VectorType, typename IndexType>
const typename VectorType::value_type& safeVectorGet(const VectorType& vec, IndexType index) {
  BATCHED_SIM_ASSERT_VECTOR_ACCESS(vec, index);
  return vec[index];
}
#endif

