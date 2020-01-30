// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class Drawable;
class DrawableGroup;
class RenderCamera;

// TODO: string type to allow for dynamically added shader types
enum class ShaderType {
  /**
   * Shader program for instance mesh data. See @ref gfx::PrimitiveIDShader,
   * @ref GenericInstanceMeshData, @ref Mp3dInstanceMeshData, @ref
   * AssetType::INSTANCE_MESH, @ref loadInstanceMeshData.
   */
  INSTANCE_MESH_SHADER = 0,

  /**
   * Shader program for PTex mesh data. See @ref gfx::PTexMeshShader, @ref
   * gfx::PTexMeshDrawable, @ref loadPTexMeshData, @ref PTexMeshData.
   */
  PTEX_MESH_SHADER = 1,

  /**
   * Shader program for flat shading with uniform color. Used to render object
   * identifier or semantic types (e.g. chair, table, etc...). Also the
   * default shader for assets with unidentified rendering parameters. See
   * @ref Magnum::Shaders::Flat3D.
   */
  FLAT_SHADER = 2,

  /**
   * Shader program for vertex color shading. Used to render meshes with
   * per-vertex colors defined.
   */
  PHONG_SHADER = 3,
};

struct ShaderConfiguration {
  ShaderType type = ShaderType::FLAT_SHADER;
  // JSON options;
  bool textured = true;
  bool vertexColored = false;
};

/**
 * @brief Shader class
 */
class Shader {
 public:
  /**
   * @brief Construct a shader with given configuration
   */
  explicit Shader(const ShaderConfiguration& config = {});

  /**
   * @brief Get the current shader configuration
   */
  const ShaderConfiguration& getConfiguration() const { return config_; }

  /**
   * @brief Update the configuration for this shader
   * @throw std::invalid_argument if the configuration is not valid
   */
  void setConfiguration(const ShaderConfiguration& config);

  /**
   * @brief Prepare to draw with given @ref RenderCamera
   */
  void prepareForDraw(const RenderCamera& camera);

  /**
   * @brief Draw the @ref Drawable with current shader configuration.
   *
   * The @ref DrawableGroup and @ref RenderCamera parameters from the last call
   * to @ref prepareForDraw will be used to draw the given @ref Drawable.
   *
   * This function is templated to allow for derived drawable types not to have
   * their types erased before being passed to Shader Programs for rendering
   *
   * @return Whether the specified @ref Drawable was drawn
   */
  template <class Drawable>
  bool draw(const Drawable& drawable,
            const Magnum::Matrix4& transformationMatrix,
            Magnum::SceneGraph::Camera3D& camera);

 private:
  ShaderConfiguration config_;

  std::unique_ptr<Magnum::GL::AbstractShaderProgram> shaderProgram_;

  ESP_SMART_POINTERS(Shader);
};

}  // namespace gfx
}  // namespace esp
