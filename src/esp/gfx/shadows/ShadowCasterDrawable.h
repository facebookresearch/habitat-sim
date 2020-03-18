#pragma once
/*
    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include <Magnum/GL/Mesh.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/Object.h>

#include "esp/gfx/Drawable.h"
#include "esp/gfx/DrawableGroup.h"
#include "esp/gfx/ShaderManager.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

class ShadowCasterShader;

class ShadowCasterDrawable : public Drawable {
 public:
  explicit ShadowCasterDrawable(scene::SceneNode& node,
                                Magnum::GL::Mesh& mesh,
                                ShaderManager& shaderManager,
                                DrawableGroup* group = nullptr);

  static constexpr char SHADER_KEY[] = "ShadowCasterShader";

  void draw(const Magnum::Matrix4& transformationMatrix,
            Magnum::SceneGraph::Camera3D& shadowCamera) override;

 private:
  ShadowCasterShader* shader_ = nullptr;
};

}  // namespace gfx
}  // namespace esp
