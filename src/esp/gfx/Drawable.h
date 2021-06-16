// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DRAWABLE_H_
#define ESP_GFX_DRAWABLE_H_

#include <Corrade/Containers/EnumSet.h>

#include "esp/core/esp.h"
#include "magnum.h"

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
  PTexMesh = 3,
  MeshVisualizer = 4,
  VarianceShadowMap = 6,
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
           Magnum::GL::Mesh& mesh,
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
  Magnum::GL::Mesh& getMesh() const { return mesh_; }

  /** @brief get the drawable type */
  DrawableType getDrawableType() const { return type_; }

  /**
   * @brief Get the Magnum GL mesh for visualization, highlighting (e.g., used
   * in object picking)
   * See MeshVisualizer3D in Magnum library for more details.
   *
   * @return mesh_ by default.
   * NOTE: sub-class should override this function if the "visualizer mesh" is
   * different from mesh_ (check the example in the PTexMeshDrawable class)
   */
  virtual Magnum::GL::Mesh& getVisualizerMesh() { return mesh_; }

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
  Magnum::GL::Mesh& mesh_;

  static uint64_t drawableIdCounter;
  uint64_t drawableId_;
};

CORRADE_ENUMSET_OPERATORS(Drawable::Flags)

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_DRAWABLE_H_
