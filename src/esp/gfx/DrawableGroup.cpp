// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "DrawableGroup.h"
#include "Drawable.h"

namespace esp {
namespace gfx {

class Drawable;

DrawableGroup::~DrawableGroup() {
  // has to inform any currently existing drawables, do not query anything
  // regarding this group
  for (int iDrawable = 0; iDrawable < this->size(); ++iDrawable) {
    static_cast<Drawable&>((*this)[iDrawable]).groupIsNull_ = true;
  }
}

bool DrawableGroup::hasDrawable(uint64_t id) const {
  return (idToDrawable_.find(id) != idToDrawable_.end());
}

Drawable* DrawableGroup::getDrawable(uint64_t id) const {
  if (hasDrawable(id)) {
    return idToDrawable_.at(id);
  }

  return nullptr;
}

}  // namespace gfx
}  // namespace esp
