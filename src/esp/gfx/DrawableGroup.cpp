// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "DrawableGroup.h"
#include "Drawable.h"

namespace esp {
namespace gfx {

class Drawable;
DrawableGroup& DrawableGroup::add(Drawable& drawable) {
  if (registerDrawable(drawable)) {
    this->Magnum::SceneGraph::DrawableGroup3D::add(drawable);
  }
  return *this;
}

DrawableGroup& DrawableGroup::remove(Drawable& drawable) {
  if (unregisterDrawable(drawable)) {
    this->Magnum::SceneGraph::DrawableGroup3D::remove(drawable);
  }
  return *this;
}

DrawableGroup::~DrawableGroup() = default;

bool DrawableGroup::hasDrawable(uint64_t id) const {
  return (idToDrawable_.find(id) != idToDrawable_.end());
}

Drawable* DrawableGroup::getDrawable(uint64_t id) const {
  auto it = idToDrawable_.find(id);
  if (it != idToDrawable_.end()) {
    return it->second;
  }

  return nullptr;
}

bool DrawableGroup::registerDrawable(Drawable& drawable) {
  // if it is already registered, emplace will do nothing
  return idToDrawable_.emplace(drawable.getDrawableId(), &drawable).second;
}
bool DrawableGroup::unregisterDrawable(Drawable& drawable) {
  // if it is not registered, erase will do nothing
  return idToDrawable_.erase(drawable.getDrawableId()) != 0;
}

}  // namespace gfx
}  // namespace esp
