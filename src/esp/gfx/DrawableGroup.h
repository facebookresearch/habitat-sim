// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Containers/Optional.h>
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
  /*
  disable the dctor as it trigues weird error saying that:
  FeatureGroup.h:
In instantiation of ‘Magnum::SceneGraph::FeatureGroup<<anonymous>,
<template-parameter-1-2>, <template-parameter-1-3> >::~FeatureGroup() [with
unsigned int dimensions = 3; Feature = Magnum::SceneGraph::Drawable<3, float>; T
= float]’: habitat-sim/src/esp/gfx/DrawableGroup.h:25:27:   required
from here
habitat-sim/src/deps/magnum/src/Magnum/SceneGraph/FeatureGroup.h:177:65:
error: invalid static_cast from type ‘Magnum::SceneGraph::AbstractFeature<3,
float>’ to type ‘Magnum::SceneGraph::Drawable<3, float>&’ for(auto i:
AbstractFeatureGroup<dimensions, T>::_features)
static_cast<Feature&>(i.get())._group = nullptr;
  */
  // virtual ~DrawableGroup(){};

  /**
   * @brief Prepare to draw group with given @ref RenderCamera
   *
   * @return Whether the @ref DrawableGroup is in a valid state to be drawn
   */
  virtual bool prepareForDraw(const RenderCamera&) { return true; }

  /**
   * @brief Given drawable object id, returns if drawable is in the group
   */
  bool hasDrawable(uint64_t id);

  // Corrade::Containers::Optional<std::reference_wrapper<Drawable>>
  // getDrawable(uint64_t id);
  Drawable* getDrawable(uint64_t id);

 protected:
  friend class Drawable;
  /**
   * a map, that maps a drawable id to the drawable object
   *
   * NOTE: operations on this structure are all in the class Drawable
   */
  std::unordered_map<uint64_t, Drawable*> drawableLookUp_;

  ESP_SMART_POINTERS(DrawableGroup)
};

}  // namespace gfx
}  // namespace esp
