// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Drawable.h"
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Assert.h>
#include "DrawableGroup.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
uint64_t Drawable::drawableIdCounter = 0;
Drawable::Drawable(scene::SceneNode& node,
                   Magnum::GL::Mesh* mesh,
                   DrawableType type,
                   DrawableConfiguration& cfg,
                   Mn::Resource<LightSetup> lightSetup)
    : Magnum::SceneGraph::Drawable3D{node, cfg.group_},
      type_(type),
      node_(node),
      drawableId_(drawableIdCounter++),
      lightSetup_(std::move(lightSetup)),
      skinData_(cfg.getSkinData()),
      jointTransformations_(),
      mesh_(mesh) {
  if (cfg.group_) {
    cfg.group_->registerDrawable(*this);
  }
}

Drawable::~Drawable() {
  DrawableGroup* group = drawables();
  if (group) {
    group->unregisterDrawable(*this);
  }
}

DrawableGroup* Drawable::drawables() {
  auto* group = Magnum::SceneGraph::Drawable3D::drawables();
  if (!group) {
    return nullptr;
  }
  CORRADE_ASSERT(dynamic_cast<DrawableGroup*>(group),
                 "Drawable must only be used with esp::gfx::DrawableGroup!",
                 {});
  return static_cast<DrawableGroup*>(group);
}

void Drawable::buildSkinJointTransforms() {
  if (!skinData_) {
    return;
  }
  // Gather joint transformations
  const auto& skin = skinData_->skinData->skin;
  const auto& transformNodes = skinData_->jointIdToTransformNode;

  ESP_CHECK(jointTransformations_.size() == skin->joints().size(),
            "Joint transformation count doesn't match bone count.");

  // Undo root node transform so that the model origin matches the root
  // articulated object link.
  const auto invRootTransform =
      skinData_->rootArticulatedObjectNode->absoluteTransformationMatrix()
          .inverted();

  for (std::size_t i = 0; i != jointTransformations_.size(); ++i) {
    const auto jointNodeIt = transformNodes.find(skin->joints()[i]);
    if (jointNodeIt != transformNodes.end()) {
      jointTransformations_[i] =
          invRootTransform *
          jointNodeIt->second->absoluteTransformationMatrix() *
          skin->inverseBindMatrices()[i];
    } else {
      // Joint not found, use placeholder matrix.
      jointTransformations_[i] = Mn::Matrix4{Mn::Math::IdentityInit};
    }
  }

}  // Drawable::buildSkinJointTransforms

void Drawable::resizeJointTransformArray(Mn::UnsignedInt jointCount) {
  if (jointTransformations_.size() != jointCount) {
    Corrade::Containers::arrayResize(jointTransformations_, jointCount);
  }
}

}  // namespace gfx
}  // namespace esp
