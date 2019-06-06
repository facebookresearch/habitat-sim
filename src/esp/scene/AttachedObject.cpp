// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AttachedObject.h"
#include "SceneNode.h"

namespace esp {
namespace scene {

AttachedObject::AttachedObject(SceneNode& node, AttachedObjectType type)
    : Magnum::SceneGraph::AbstractFeature3D{node}, objectType_(type) {}

SceneNode& AttachedObject::object() {
  return static_cast<SceneNode&>(
      Magnum::SceneGraph::AbstractFeature3D::object());
}

const SceneNode& AttachedObject::object() const {
  return static_cast<const SceneNode&>(
      Magnum::SceneGraph::AbstractFeature3D::object());
}

}  // namespace scene
}  // namespace esp
