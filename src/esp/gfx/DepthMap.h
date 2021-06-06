// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_DEPTHMAP_H_
#define ESP_GFX_PBR_DEPTHMAP_H_

#include "DrawableGroup.h"

namespace esp {
namespace gfx {
class DepthMap {
 public:
  /**
   * @brief Constructor
   */
  explicit DepthMap();

  /** @brief Copying is not allowed */
  DepthMap(const DepthMap&) = delete;

  /** @brief Move constructor */
  DepthMap(DepthMap&&) noexcept = default;

  /** @brief Copying is not allowed */
  DepthMap& operator=(const DepthMap&) = delete;

  /** @brief Move assignment */
  DepthMap& operator=(DepthMap&&) noexcept = default;

  /**
   * @brief for each drawable from the sourceGroup, create a new DepthMap
   * drawable and insert it to the depthMapGroup.
   * @param[in] sourceGroup, source drawable group
   * @param[in] depthMapGroup, depth map group that will contain the new
   * DepthMapDrawables
   */
  void fillDepthDrawableGroups(const gfx::DrawableGroup& sourceGroup,
                               gfx::DrawableGroup& depthMapGroup);
};

}  // namespace gfx
}  // namespace esp

#endif
