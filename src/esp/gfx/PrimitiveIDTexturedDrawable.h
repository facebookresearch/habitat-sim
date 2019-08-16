// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Drawable.h"

namespace esp {
namespace gfx {

class PrimitiveIDTexturedShader;

class PrimitiveIDTexturedDrawable : public Drawable {
 public:
  //! Create a PrimitiveIDTexturedDrawable for the given object using shader
  //! and mesh. Adds drawable to given group and uses provided texture,
  //! objectId, and color for textured, object id buffer and color shader
  //! output respectively
  explicit PrimitiveIDTexturedDrawable(
      scene::SceneNode& node,
      PrimitiveIDTexturedShader& shader,
      Magnum::GL::Mesh& mesh,
      Magnum::SceneGraph::DrawableGroup3D* group = nullptr,
      Magnum::GL::Texture2D* texture = nullptr);

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;

  Magnum::GL::Texture2D* texture_;
};

}  // namespace gfx
}  // namespace esp
