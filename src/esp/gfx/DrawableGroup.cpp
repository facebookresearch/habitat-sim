// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "DrawableGroup.h"
#include "Drawable.h"

namespace esp {
namespace gfx {

class Drawable;
auto DrawableGroup::add(Drawable& drawable) -> DrawableGroup& {
  if (registerDrawable(drawable)) {
    this->Magnum::SceneGraph::DrawableGroup3D::add(drawable);
  }
  return *this;
}

auto DrawableGroup::remove(Drawable& drawable) -> DrawableGroup& {
  if (unregisterDrawable(drawable)) {
    this->Magnum::SceneGraph::DrawableGroup3D::remove(drawable);
  }
  return *this;
}

DrawableGroup::~DrawableGroup() = default;

auto DrawableGroup::hasDrawable(uint64_t id) const -> bool {
  return (idToDrawable_.find(id) != idToDrawable_.end());
}

auto DrawableGroup::getDrawable(uint64_t id) const -> Drawable* {
  auto it = idToDrawable_.find(id);
  if (it != idToDrawable_.end()) {
    return it->second;
  }

  return nullptr;
}

auto DrawableGroup::registerDrawable(Drawable& drawable) -> bool {
  // if it is already registered, emplace will do nothing
  if (idToDrawable_.emplace(drawable.getDrawableId(), &drawable).second) {
    return true;
  }
  return false;
}
auto DrawableGroup::unregisterDrawable(Drawable& drawable) -> bool {
  // if it is not registered, erase will do nothing
  if (idToDrawable_.erase(drawable.getDrawableId()) == 0) {
    return false;
  }
  return true;
}

}  // namespace gfx
}  // namespace esp
