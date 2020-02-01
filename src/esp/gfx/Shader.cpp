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
std::shared_ptr<Magnum::GL::AbstractShaderProgram> shaderProgramFactory(
    ShaderType type,
    const ShaderConfiguration& cfg) {
  switch (type) {
    case ShaderType::INSTANCE_MESH_SHADER: {
      // TODO: reuse shaders
      return std::make_shared<gfx::PrimitiveIDShader>();
      ;
    } break;
#ifdef ESP_BUILD_PTEX_SUPPORT
    case ShaderType::PTEX_MESH_SHADER: {
      // TODO: reuse shaders
      return std::make_shared<gfx::PTexMeshShader>();
    } break;
#endif

    case ShaderType::FLAT_SHADER: {
      Magnum::Shaders::Flat3D::Flags flags =
          Magnum::Shaders::Flat3D::Flag::ObjectId;
      if (cfg.vertexColored)
        flags |= Magnum::Shaders::Flat3D::Flag::VertexColor;
      if (cfg.textured)
        flags |= Magnum::Shaders::Flat3D::Flag::Textured;
      // TODO: cache and share this
      return std::make_shared<Magnum::Shaders::Flat3D>(flags);
    } break;

    case ShaderType::PHONG_SHADER: {
      Magnum::Shaders::Phong::Flags flags =
          Magnum::Shaders::Phong::Flag::ObjectId;
      if (cfg.vertexColored)
        flags |= Magnum::Shaders::Phong::Flag::VertexColor;
      if (cfg.textured)
        flags |= Magnum::Shaders::Phong::Flag::DiffuseTexture;
      // TODO: cache and share this
      auto shaderProgram =
          std::make_shared<Magnum::Shaders::Phong>(flags, 3 /*lights*/);

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

Shader::Shader(const ShaderConfiguration& config) {
  setConfiguration(config);
}

void Shader::setConfiguration(const ShaderConfiguration& config) {
  // update program now that we have a new config
  // we could wait until prepareForDraw to do this, but creating now outside of
  // the render pipeline will give us a cached program for use during render

  // this may throw if config is invalid for specified ShaderType
  std::shared_ptr<Magnum::GL::AbstractShaderProgram> newProgram =
      shaderProgramFactory(config.type, config);
  if (!newProgram) {
    throw std::invalid_argument(
        "Invalid shader type! No factory registered for type (TODO)");
  }
  config_ = config;
  shaderProgram_ = std::move(newProgram);
}

void Shader::prepareForDraw(const RenderCamera& camera) {
  // since we update shaderProgram in setConfiguration, shaderProgram_ supports
  // the current config. So skip factory call and update directly
  // shaderProgram_->updateConfig(config);

  // shaderProgram_->setCamera(camera);
}

}  // namespace gfx
}  // namespace esp
