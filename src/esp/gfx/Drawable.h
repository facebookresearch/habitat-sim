// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DRAWABLE_H_
#define ESP_GFX_DRAWABLE_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Assert.h>
#include <Magnum/GL/GL.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/Trade/MaterialData.h>
#include "esp/core/Esp.h"

namespace esp {
namespace scene {
class SceneNode;
}
namespace gfx {

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
           DrawableGroup* group = nullptr);
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

  DrawableType type_ = DrawableType::None;

  scene::SceneNode& node_;

  static uint64_t drawableIdCounter;
  uint64_t drawableId_;

  bool glMeshExists() const { return mesh_ != nullptr; }

 private:
  Magnum::GL::Mesh* mesh_ = nullptr;
};

CORRADE_ENUMSET_OPERATORS(Drawable::Flags)

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_DRAWABLE_H_
