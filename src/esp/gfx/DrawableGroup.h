// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DRAWABLEGROUP_H_
#define ESP_GFX_DRAWABLEGROUP_H_

#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/FeatureGroup.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <unordered_map>

#include <functional>
#include "esp/core/Esp.h"

namespace esp {
namespace gfx {

class RenderCamera;
class Drawable;

/**
 * @brief Group of drawables, and shared group parameters.
 */
class DrawableGroup : public Magnum::SceneGraph::DrawableGroup3D {
 public:
  ~DrawableGroup() override;

  /**
   * @brief Add a drawable to the group.
   * @return Reference to self (for method chaining)
   *
   * If the drawable is part of another group, it is removed from it.
   */
  DrawableGroup& add(Drawable& drawable);
  /**
   * @brief Remove a drawable from the group.
   * @return Reference to self (for method chaining)
   *
   * The feature must be part of the group.
   */
  DrawableGroup& remove(Drawable& drawable);

  /**
   * @brief Given drawable id, returns if drawable is in the group
   * @param id drawable id
   * @return true if the drawable is in the group, otherwise false
   */
  bool hasDrawable(uint64_t id) const;

  /**
   * @brief Given drawable id, returns nullptr if the id is not in this
   * drawable group, otherwise the raw pointer to the object
   *
   * @param id drawable id
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
   * @brief Add the drawable to the lookup table, and update the state in the
   * drawable
   * @return return true, if the drawable was newly registered, otherwise false
   */
  bool registerDrawable(Drawable& drawable);
  /**
   * @brief Remove the drawable from the lookup table, and update the state in
   * the drawable
   * @return return true, if the drawable was in the group, otherwise false
   */
  bool unregisterDrawable(Drawable& drawable);
  /**
   * a lookup table, that maps a drawable id to the drawable object
   */
  std::unordered_map<uint64_t, Drawable*> idToDrawable_;
  ESP_SMART_POINTERS(DrawableGroup)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_DRAWABLEGROUP_H_
