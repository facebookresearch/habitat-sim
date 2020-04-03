// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Drawable.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {

class TriangleIDShader;

class TriangleIDDrawable : public Drawable {
 public:
  //! Create a TriangleIDDrawable for the given object using shader
  //! and mesh. Adds drawable to given group and uses provided texture,
  //! triangleId, and color for textured, triangle id buffer and color shader
  //! output respectively
  explicit TriangleIDDrawable(scene::SceneNode& node,
                              Magnum::GL::Mesh& mesh,
                              ShaderManager& shaderManager,
                              DrawableGroup* group = nullptr);

  static constexpr char SHADER_KEY[] = "TriangleIDShader";

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;

  TriangleIDShader* shader_ = nullptr;
};

}  // namespace gfx
}  // namespace esp
