// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Drawable.h"

namespace esp {
namespace gfx {

class PrimitiveIDShader;

class PrimitiveIDDrawable : public Drawable {
 public:
  //! Create a PrimitiveIDDrawable for the given object using shader
  //! and mesh. Adds drawable to given group and uses provided texture,
  //! objectId, and color for textured, object id buffer and color shader
  //! output respectively
  explicit PrimitiveIDDrawable(scene::SceneNode& node,
                               PrimitiveIDShader& shader,
                               Magnum::GL::Mesh& mesh,
                               DrawableGroup* group = nullptr);

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;
};

}  // namespace gfx
}  // namespace esp
