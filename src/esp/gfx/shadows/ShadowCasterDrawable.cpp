/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ShadowCasterDrawable.h"

#include <Magnum/SceneGraph/Camera.h>

#include "ShadowCasterShader.h"

namespace esp {
namespace gfx {

// static constexpr arrays require redundant definitions until C++17
constexpr char ShadowCasterDrawable::SHADER_KEY[];

namespace Mn = Magnum;

ShadowCasterDrawable::ShadowCasterDrawable(scene::SceneNode& node,
                                           Magnum::GL::Mesh& mesh,
                                           ShaderManager& shaderManager,
                                           DrawableGroup* group)
    : Drawable{node, mesh, group} {
  auto shaderResource =
      shaderManager.get<Magnum::GL::AbstractShaderProgram, ShadowCasterShader>(
          SHADER_KEY);

  if (!shaderResource) {
    shaderManager.set<Magnum::GL::AbstractShaderProgram>(
        shaderResource.key(), new ShadowCasterShader{});
  }
  shader_ = &(*shaderResource);
}

void ShadowCasterDrawable::draw(const Mn::Matrix4& transformationMatrix,
                                Mn::SceneGraph::Camera3D& shadowCamera) {
  shader_->setTransformationMatrix(shadowCamera.projectionMatrix() *
                                   transformationMatrix);
  mesh_.draw(*shader_);
}

}  // namespace gfx
}  // namespace esp
