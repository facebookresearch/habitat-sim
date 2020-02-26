// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Shaders/Shaders.h>

#include "Drawable.h"

namespace esp {
namespace gfx {

class GenericDrawable : public Drawable {
 public:
  //! Create a GenericDrawable for the given object using shader and mesh.
  //! Adds drawable to given group and uses provided texture, objectId, and
  //! color for textured, object id buffer and color shader output respectively
  explicit GenericDrawable(scene::SceneNode& node,
                           Magnum::Shaders::Flat3D& shader,
                           Magnum::GL::Mesh& mesh,
                           DrawableGroup* group = nullptr,
                           Magnum::GL::Texture2D* texture = nullptr,
                           int objectId = ID_UNDEFINED,
                           const Magnum::Color4& color = Magnum::Color4{1});

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;

  Magnum::GL::Texture2D* texture_;
  int objectId_;
  Magnum::Color4 color_;
};

}  // namespace gfx
}  // namespace esp
