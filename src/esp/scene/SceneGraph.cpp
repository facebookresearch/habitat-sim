// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "SceneGraph.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>

namespace esp {
namespace scene {

SceneGraph::SceneGraph() : rootNode_{world_} {
  // For now, just create one drawable group with empty string uuid
  createDrawableGroup(std::string{});
}

bool SceneGraph::isRootNode(SceneNode& node) {
  auto* parent = node.parent();
  // if the parent is null, it means the node is the world_ node.
  CORRADE_ASSERT(parent != nullptr,
                 "SceneGraph::isRootNode: the node is illegal.", false);
  return (parent->parent() == nullptr);
}

gfx::DrawableGroup* SceneGraph::getDrawableGroup(const std::string& id) {
  auto it = drawableGroups_.find(id);
  return it == drawableGroups_.end() ? nullptr : &it->second;
}

const gfx::DrawableGroup* SceneGraph::getDrawableGroup(
    const std::string& id) const {
  auto it = drawableGroups_.find(id);
  return it == drawableGroups_.end() ? nullptr : &it->second;
}

bool SceneGraph::deleteDrawableGroup(const std::string& id) {
  return drawableGroups_.erase(id) != 0u;
}

}  // namespace scene
}  // namespace esp
