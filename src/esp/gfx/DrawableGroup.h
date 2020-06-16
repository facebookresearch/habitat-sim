// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/FeatureGroup.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <functional>
#include <unordered_map>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class RenderCamera;
class Drawable;

/**
 * @brief Group of drawables, and shared group parameters.
 */
class DrawableGroup : public Magnum::SceneGraph::DrawableGroup3D {
 public:
  virtual ~DrawableGroup() {
    LOG(INFO) << "Haha, Deconstructing drawableGroup";  // XXX
  };

  /**
   * @brief Given drawable id, returns if drawable is in the group
   * @param id, drawable id
   * @return true if the drawable is in the group, otherwise false
   */
  bool hasDrawable(uint64_t id) const;

  /**
   * @brief Given drawable id, returns nullptr if the id is not in this
   * drawable group, otherwise the raw pointer to the object
   *
   * @param id, drawable id
   * @return raw pointer to the drawable, or nullptr
   */
  Drawable* getDrawable(uint64_t id) const;

  /**
   * @brief Prepare to draw group with given @ref RenderCamera
   *
   * @return Whether the @ref DrawableGroup is in a valid state to be drawn
   */
  virtual bool prepareForDraw(const RenderCamera&) { return true; }

 protected:
  /**
   * Why a friend class here?
   * class Drawable has to update idToDrawable_, and it is the ONLY class that
   * can do it.
   */
  friend class Drawable;
  /**
   * a map, that maps a drawable id to the drawable object
   * NOTE: operations on this structure are all in the class Drawable
   *
   */
  std::unordered_map<uint64_t, Drawable*> idToDrawable_;
  ESP_SMART_POINTERS(DrawableGroup)
};

}  // namespace gfx
}  // namespace esp
