// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Shader.h"

#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/Shaders.h>

#include "esp/gfx/PrimitiveIDShader.h"
#ifdef ESP_BUILD_PTEX_SUPPORT
#include "esp/gfx/PTexMeshShader.h"
#endif
#include "esp/gfx/Drawable.h"
#include "esp/gfx/RenderCamera.h"

namespace esp {
namespace gfx {

namespace {
// TODO: don't hardcode this
std::unique_ptr<Magnum::GL::AbstractShaderProgram> shaderProgramFactory(
    ShaderType type,
    const ShaderConfiguration& cfg) {
  switch (type) {
    case ShaderType::INSTANCE_MESH_SHADER: {
      return std::make_unique<gfx::PrimitiveIDShader>();
    } break;
#ifdef ESP_BUILD_PTEX_SUPPORT
    case ShaderType::PTEX_MESH_SHADER: {
      return std::make_unique<gfx::PTexMeshShader>();
    } break;
#endif

    case ShaderType::FLAT_SHADER: {
      Magnum::Shaders::Flat3D::Flags flags =
          Magnum::Shaders::Flat3D::Flag::ObjectId;
      if (cfg.vertexColored)
        flags |= Magnum::Shaders::Flat3D::Flag::VertexColor;
      if (cfg.textured)
        flags |= Magnum::Shaders::Flat3D::Flag::Textured;

      return std::make_unique<Magnum::Shaders::Flat3D>(flags);
    } break;

    case ShaderType::PHONG_SHADER: {
      Magnum::Shaders::Phong::Flags flags =
          Magnum::Shaders::Phong::Flag::ObjectId;
      if (cfg.vertexColored)
        flags |= Magnum::Shaders::Phong::Flag::VertexColor;
      if (cfg.textured)
        flags |= Magnum::Shaders::Phong::Flag::DiffuseTexture;
      std::unique_ptr<Magnum::Shaders::Phong> shaderProgram =
          std::make_unique<Magnum::Shaders::Phong>(flags, 3 /*lights*/);

      // TODO: don't hardcode this
      // NOLINTNEXTLINE(google-build-using-namespace)
      using namespace Magnum::Math::Literals;
      shaderProgram
          ->setLightPositions({Magnum::Vector3{10.0f, 10.0f, 10.0f} * 100.0f,
                               Magnum::Vector3{-5.0f, -5.0f, 10.0f} * 100.0f,
                               Magnum::Vector3{0.0f, 10.0f, -10.0f} * 100.0f})
          .setLightColors({0xffffff_rgbf * 0.8f, 0xffcccc_rgbf * 0.8f,
                           0xccccff_rgbf * 0.8f})
          .setSpecularColor(0x11111100_rgbaf)
          .setShininess(80.0f);

      return shaderProgram;
    } break;

    default:
      return nullptr;
      break;
  }
}
}  // namespace

Shader::Shader(const ShaderConfiguration& config) : config_{config} {
  setConfiguration(config);
}

void Shader::setConfiguration(const ShaderConfiguration& config) {
  config_ = config;
  // update program now that we have a new config
  // this may throw if config is invalid for specific ShaderType
  std::unique_ptr<Magnum::GL::AbstractShaderProgram> newProgram =
      shaderProgramFactory(config.type, config);
  if (!newProgram) {
    throw std::invalid_argument(
        "Invalid shader type! No factory registered for type (TODO)");
  }
  shaderProgram_ = std::move(newProgram);
}

void Shader::prepareForDraw(const RenderCamera& camera) {
  // shaderProgram_->prepareforDraw(camera);
}

template <class Drawable>
bool draw(const Drawable& drawable,
          const Magnum::Matrix4& transformationMatrix,
          Magnum::SceneGraph::Camera3D& camera) {
  // shaderProgram_->draw(drawable, transformationMatrix, camera);
}

}  // namespace gfx
}  // namespace esp
