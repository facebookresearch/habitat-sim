// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DRAWABLE_H_
#define ESP_GFX_DRAWABLE_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/GL/GL.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/Trade/MaterialData.h>
#include "esp/core/Esp.h"
#include "esp/gfx/DrawableConfiguration.h"

namespace esp {
namespace scene {
class SceneNode;
}
namespace gfx {

struct InstanceSkinData;
class DrawableGroup;

enum class DrawableType : uint8_t {
  None = 0,
  Generic = 1,
  Pbr = 2,
  MeshVisualizer = 3
};

/**
 * @brief Drawable for use with @ref DrawableGroup.
 *
 * Drawable will retrieve its shader from its group, and draw
 * itself with the shader.
 */
class Drawable : public Magnum::SceneGraph::Drawable3D {
 public:
  /** @brief Flag
   * It will not be used directly in the base class "Drawable" but
   * it will be used in a couple of sub-classes
   */
  enum class Flag {
    /**
     * indicates the mesh data has tangent attribute
     */
    HasTangent = 1 << 0,

    /**
     * indicates the mesh data has separate bi-tangent attribute
     */
    HasSeparateBitangent = 1 << 1,

    /**
     * indicates whether the mesh is vertex colored
     */
    HasVertexColor = 1 << 2

  };
  /** @brief Flags */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /// @brief Key template for entry in shader map
  static constexpr const char* SHADER_KEY_TEMPLATE =
      "{}-lights={}-flags={}-joints={}";

  /**
   * @brief Constructor
   *
   * @param node Node which will be made drawable.
   * @param mesh Mesh to draw when on render.
   * @param type the type of this drawable
   * @param group Drawable group this drawable will be added to.
   */
  Drawable(scene::SceneNode& node,
           Magnum::GL::Mesh* mesh,
           DrawableType type,
           DrawableConfiguration& cfg,
           Mn::Resource<LightSetup> lightSetup);
  ~Drawable() override;

  virtual scene::SceneNode& getSceneNode() { return node_; }

  /**
   * @brief Get the @ref DrawableGroup this drawable is in.
   *
   * This overrides Magnum::SceneGraph::Drawable so that the derived @ref
   * DrawableGroup can be used
   */
  DrawableGroup* drawables();

  /**
   * @brief Get the drawable id
   */
  uint64_t getDrawableId() const { return drawableId_; }

  /**
   * @brief setup the lights.
   * NOTE: sub-class should override this function
   */
  virtual void setLightSetup(
      CORRADE_UNUSED const Magnum::ResourceKey& lightSetup){};
  /**
   * @brief the the scene node
   */
  virtual scene::SceneNode& getSceneNode() const { return node_; }

  /** @brief get the GL mesh */
  Magnum::GL::Mesh& getMesh() const {
    CORRADE_ASSERT(
        mesh_ != nullptr,
        "Drawable::getMesh() : Attempting to get the GL mesh when none exists",
        *mesh_);
    return *mesh_;
  }

  /** @brief get the drawable type */
  DrawableType getDrawableType() const { return type_; }

  /**
   * @brief Get the Magnum GL mesh for visualization, highlighting (e.g., used
   * in object picking)
   * See MeshVisualizer3D in Magnum library for more details.
   *
   * @return mesh_ by default.
   * NOTE: sub-class should override this function if the "visualizer mesh" is
   * different from mesh_
   */
  virtual Magnum::GL::Mesh& getVisualizerMesh() {
    CORRADE_INTERNAL_ASSERT(mesh_ != nullptr);
    return *mesh_;
  }

  /**
   * Change this drawable's @ref Magnum::Trade::MaterialData values from passed material, keeping existing values if not overwritten
   * This is only pertinent for material-equipped drawables.
   * @param material material values to set
   */

  void setMaterialValues(
      const Magnum::Resource<Magnum::Trade::MaterialData,
                             Magnum::Trade::MaterialData>& material) {
    setMaterialValuesInternal(material, false);
  }

  /**
   * Reset this drawable's @ref Magnum::Trade::MaterialData values from passed material, completely replacing the existing values
   * This is only pertinent for material-equipped drawables.
   * @param material material values to set
   * @param reset whether to reset underlying material or to write over it
   */
  void resetMaterialValues(
      const Magnum::Resource<Magnum::Trade::MaterialData,
                             Magnum::Trade::MaterialData>& material) {
    setMaterialValuesInternal(material, true);
  }

 private:
  /**
   * Set or change this drawable's @ref Magnum::Trade::MaterialData values from passed material.
   * This is only pertinent for material-equipped drawables.
   * @param material material values to set
   * @param reset whether to reset underlying material or to write over it
   */
  virtual void setMaterialValuesInternal(
      CORRADE_UNUSED const Magnum::Resource<Magnum::Trade::MaterialData,
                                            Magnum::Trade::MaterialData>&
          material,
      CORRADE_UNUSED bool reset) {}

 protected:
  /**
   * @brief resize the jointTransformArray_
   */
  void resizeJointTransformArray(Mn::UnsignedInt jointCount);
  /**
   * @brief Draw the object using given camera
   *
   * @param transformationMatrix  Transformation relative to camera.
   * @param camera                Camera to draw from.
   *
   * Each derived drawable class needs to implement this draw() function.
   * It's nothing more than drawing itself with its group's shader.
   */
  void draw(CORRADE_UNUSED const Magnum::Matrix4& transformationMatrix,
            CORRADE_UNUSED Magnum::SceneGraph::Camera3D& camera) override = 0;

  /**
   * @brief Derive the shader key appropriate for the calling child drawable
   * @tparam FlagsType is the underlying type of the flags used by the calling
   * drawable.
   * @param shaderType Name of shader class (i.e. "Phong" or "PBR")
   * @param lightCount Number of lights used by drawable
   * @param flags The flags used by the owning drawable.
   * @param jointCount The number of joints if an articulated object with a
   * skin, 0 otherwise
   */
  template <class FlagsType>
  std::string getShaderKey(const std::string& shaderType,
                           Magnum::UnsignedInt lightCount,
                           FlagsType flags,
                           Magnum::UnsignedInt jointCount) const {
    return Corrade::Utility::formatString(SHADER_KEY_TEMPLATE, shaderType,
                                          lightCount, flags, jointCount);
  }

  /**
   * @brief Build the joint transformations on every draw if skinData exists
   */
  void buildSkinJointTransforms();

  /**
   * @brief Update lighting-related parameters on every draw call
   * @tparam ShaderType is the type of shader being passed (wrapped in a
   * Magnum::Resource).
   * @param transformationMatrix The transformation matrix passed to this
   * drawables draw function.
   * @param camera The camera passed to this drawable's draw function
   * @param shader The shader this drawable consumes.
   * @param getLightPosition The function to query for each light to acquire its
   * proper position in the world. Flat/Phong objects query
   * esp::gfx::getLightPositionRelativeToCamera(), while PBR objects query
   * esp::gfx::getLightPositionRelativeToWorld()
   */
  template <class ShaderType>
  void updateShaderLightingParameters(
      const Mn::Matrix4& transformationMatrix,
      Mn::SceneGraph::Camera3D& camera,
      ShaderType shader,
      const std::function<Magnum::Vector4(const LightInfo&,
                                          const Magnum::Matrix4&,
                                          const Magnum::Matrix4&)>&
          getLightPosition);

  /**
   * @brief Drawable-specific update called at the end of
   * updateShaderLightingParameters
   */
  virtual void updateShaderLightingParametersInternal() {}

  DrawableType type_ = DrawableType::None;

  scene::SceneNode& node_;

  static uint64_t drawableIdCounter;
  uint64_t drawableId_;

  Mn::Resource<LightSetup> lightSetup_;

  std::shared_ptr<InstanceSkinData> skinData_{nullptr};

  Corrade::Containers::Array<Magnum::Matrix4> jointTransformations_;

  bool glMeshExists() const { return mesh_ != nullptr; }

 private:
  Magnum::GL::Mesh* mesh_ = nullptr;
};

CORRADE_ENUMSET_OPERATORS(Drawable::Flags)

template <class ShaderType>
void Drawable::updateShaderLightingParameters(
    const Mn::Matrix4& transformationMatrix,
    Mn::SceneGraph::Camera3D& camera,
    ShaderType shader,
    const std::function<Magnum::Vector4(const LightInfo&,
                                        const Magnum::Matrix4&,
                                        const Magnum::Matrix4&)>&
        getLightPosition) {
  const Mn::Matrix4 cameraMatrix = camera.cameraMatrix();

  std::vector<Mn::Vector4> lightPositions;
  lightPositions.reserve(lightSetup_->size());
  std::vector<Mn::Color3> lightColors;
  lightColors.reserve(lightSetup_->size());
  // TODO derive ranges appropriately?
  constexpr float dummyRange = Mn::Constants::inf();
  std::vector<float> lightRanges(lightSetup_->size(), dummyRange);

  // std::vector<Mn::Color3> lightSpecularColors;
  // lightSpecularColors.reserve(lightSetup_->size());

  for (Mn::UnsignedInt i = 0; i < lightSetup_->size(); ++i) {
    const auto& lightInfo = (*lightSetup_)[i];
    Mn::Vector4 pos =
        getLightPosition(lightInfo, transformationMatrix, cameraMatrix);
    // flip directional lights to facilitate faster, non-forking calc in
    // shader.  Leave non-directional lights unchanged
    pos *= (pos[3] * 2) - 1;
    lightPositions.emplace_back(pos);

    const auto& lightColor = (*lightSetup_)[i].color;
    lightColors.emplace_back(lightColor);

    // In general, a light's specular color should match its base color.
    // However, negative lights have zero (black) specular.
    // constexpr Mn::Color3 blackColor(0.0, 0.0, 0.0);
    // bool isNegativeLight = (lightColor.r() < 0 || lightColor.g() < 0 ||
    // lightColor.b() < 0); lightSpecularColors.emplace_back(isNegativeLight ?
    // blackColor : lightColor);
  }
  (*shader)
      //.setLightSpecularColors(lightSpecularColors)
      .setLightPositions(lightPositions)
      .setLightColors(lightColors)
      .setLightRanges(lightRanges);

  updateShaderLightingParametersInternal();
}

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_DRAWABLE_H_
