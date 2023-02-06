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

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#pragma once

#ifndef NVH_ALIGNEMENT_HPP
#define NVH_ALIGNEMENT_HPP 1

#include <stddef.h>  // for size_t

namespace nvh {
template <class integral>
constexpr bool is_aligned(integral x, size_t a) noexcept {
  return (x & (integral(a) - 1)) == 0;
}

template <class integral>
constexpr integral align_up(integral x, size_t a) noexcept {
  return integral((x + (integral(a) - 1)) & ~integral(a - 1));
}

template <class integral>
constexpr integral align_down(integral x, size_t a) noexcept {
  return integral(x & ~integral(a - 1));
}
}  // namespace nvh

#endif  // !NVH_ALIGNEMENT_HPP
