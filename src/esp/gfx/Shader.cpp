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
    const ShaderConfiguration& cfg) {
  std::unique_ptr<Magnum::GL::AbstractShaderProgram> shaderProgram;
  switch (cfg.type) {
    case ShaderType::INSTANCE_MESH_SHADER: {
      return std::make_unique<gfx::PrimitiveIDShader>();
    } break;
#ifdef ESP_BUILD_PTEX_SUPPORT
    case ShaderType::PTEX_MESH_SHADER: {
      return std::make_unique<gfx::PTexMeshShader>();
    } break;
#endif

    case ShaderType::COLORED_SHADER: {
      return std::make_unique<Magnum::Shaders::Flat3D>(
          Magnum::Shaders::Flat3D::Flag::ObjectId);
    } break;

    case ShaderType::VERTEX_COLORED_SHADER: {
      return std::make_unique<Magnum::Shaders::Flat3D>(
          Magnum::Shaders::Flat3D::Flag::ObjectId |
          Magnum::Shaders::Flat3D::Flag::VertexColor);
    } break;

    case ShaderType::TEXTURED_SHADER: {
      return std::make_unique<Magnum::Shaders::Flat3D>(
          Magnum::Shaders::Flat3D::Flag::ObjectId |
          Magnum::Shaders::Flat3D::Flag::Textured);
    } break;

    case ShaderType::COLORED_SHADER_PHONG: {
      shaderProgram = std::make_unique<Magnum::Shaders::Phong>(
          Magnum::Shaders::Phong::Flag::ObjectId, 3 /*lights*/);
    } break;

    case ShaderType::VERTEX_COLORED_SHADER_PHONG: {
      shaderProgram = std::make_unique<Magnum::Shaders::Phong>(
          Magnum::Shaders::Phong::Flag::ObjectId |
              Magnum::Shaders::Phong::Flag::VertexColor,
          3 /*lights*/);
    } break;

    case ShaderType::TEXTURED_SHADER_PHONG: {
      shaderProgram = std::make_unique<Magnum::Shaders::Phong>(
          Magnum::Shaders::Phong::Flag::ObjectId |
              Magnum::Shaders::Phong::Flag::DiffuseTexture,
          3 /*lights*/);
    } break;

    default:
      return nullptr;
      break;
  }

  /* Default setup for Phong, shared by all models */
  if (cfg.type == ShaderType::COLORED_SHADER_PHONG ||
      cfg.type == ShaderType::VERTEX_COLORED_SHADER_PHONG ||
      cfg.type == ShaderType::TEXTURED_SHADER_PHONG) {
    using namespace Magnum::Math::Literals;

    static_cast<Magnum::Shaders::Phong&>(*shaderProgram)
        .setLightPositions({Magnum::Vector3{10.0f, 10.0f, 10.0f} * 100.0f,
                            Magnum::Vector3{-5.0f, -5.0f, 10.0f} * 100.0f,
                            Magnum::Vector3{0.0f, 10.0f, -10.0f} * 100.0f})
        .setLightColors(
            {0xffffff_rgbf * 0.8f, 0xffcccc_rgbf * 0.8f, 0xccccff_rgbf * 0.8f})
        .setSpecularColor(0x11111100_rgbaf)
        .setShininess(80.0f);
  }
  return shaderProgram;
}
}  // namespace

Shader::Shader(const ShaderConfiguration& config)
    : config_{config}, shaderProgram_{shaderProgramFactory(config_)} {}

void Shader::setConfiguration(const ShaderConfiguration& config) {
  config_ = config;
  // update program now that we have a new config
  // TODO: error check if config is invalid
  shaderProgram_ = shaderProgramFactory(config);
}

bool Shader::prepareForDraw(const DrawableGroup& drawables,
                            const RenderCamera& camera) {
  // TODO
  return false;
}

bool draw(const Drawable& drawable) {
  // TODO
  return false;
}

}  // namespace gfx
}  // namespace esp
