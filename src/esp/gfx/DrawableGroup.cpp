// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "DrawableGroup.h"
#include "Drawable.h"

namespace esp {
namespace gfx {

class Drawable;
DrawableGroup& DrawableGroup::add(Drawable& drawable) {
  registerDrawable(drawable);
  this->Magnum::SceneGraph::DrawableGroup3D::add(drawable);
  return *this;
}

DrawableGroup& DrawableGroup::remove(Drawable& drawable) {
  unregisterDrawable(drawable);
  this->Magnum::SceneGraph::DrawableGroup3D::remove(drawable);
  return *this;
}

DrawableGroup::~DrawableGroup() {
  // has to unregister any currently existing drawables
  for (int iDrawable = 0; iDrawable < this->size(); ++iDrawable) {
    unregisterDrawable(static_cast<Drawable&>((*this)[iDrawable]));
  }
}

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

DrawableGroup& DrawableGroup::registerDrawable(Drawable& drawable) {
  uint64_t id = drawable.getDrawableId();

  // if it is already registered, emplace will do nothing
  if (idToDrawable_.emplace(id, &drawable).second) {
    drawable.attachedToGroup_ = true;
  }
  return *this;
}
DrawableGroup& DrawableGroup::unregisterDrawable(Drawable& drawable) {
  uint64_t id = drawable.getDrawableId();

  if (idToDrawable_.erase(id) != 0) {
    drawable.attachedToGroup_ = false;
  }
  return *this;
}

}  // namespace gfx
}  // namespace esp
