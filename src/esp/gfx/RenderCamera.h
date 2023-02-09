// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_RENDERCAMERA_H_
#define ESP_GFX_RENDERCAMERA_H_

#include <Magnum/SceneGraph/Camera.h>
#include "esp/core/Esp.h"
#include "esp/geo/Geo.h"
#include "esp/gfx/magnum.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

class RenderCamera : public MagnumCamera {
 public:
  /**
   * @brief Rendering Flags
   */
  enum class Flag : unsigned int {
    /**
     * Cull Drawables with bounding boxes not intersecting the camera frustum.
     */
    FrustumCulling = 1 << 0,

    /**
     * Cull Drawables not attached to @ref SceneNodes with @ref
     * scene::SceneNodeType::OBJECT.
     */
    ObjectsOnly = 1 << 1,
    /**
     * Use drawable id as the object id in the following rendering pass
     * Internally, it is not a state machine, which means user needs to set it
     * every frame if she needs the drawable ids.
     * If not set, by default, it would use the semantic id (if "per vertex
     * object id" is not set)
     */
    UseDrawableIdAsObjectId = 1 << 2,

    /**
     * Clear depth, used in the sub-class CubeMapCamera
     */
    ClearDepth = 1 << 3,

    /**
     * Clear color, used in the sub-class CubeMapCamera
     */
    ClearColor = 1 << 4,

    /**
     * Clear object id, used in the sub-class CubeMapCamera
     */
    ClearObjectId = 1 << 5,
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;
  CORRADE_ENUMSET_FRIEND_OPERATORS(Flags)

  typedef std::vector<
      std::pair<std::reference_wrapper<Magnum::SceneGraph::Drawable3D>,
                Magnum::Matrix4>>
      DrawableTransforms;

  /**
   * @brief Constructor
   * @param node the scene node to which the camera is attached
   */
  explicit RenderCamera(scene::SceneNode& node);
  /**
   * @brief Constructor
   * @param node the scene node to which the camera is attached
   * @param eye the eye position (in PARENT node space)
   * @param target the target position (in PARENT node space)
   * @param up the up direction (in PARENT node space)
   * NOTE: it will override any relative transformation w.r.t its parent node
   */
  RenderCamera(scene::SceneNode& node,
               const vec3f& eye,
               const vec3f& target,
               const vec3f& up);
  /**
   * @brief Constructor
   * @param node the scene node to which the camera is attached
   * @param eye the eye position (in PARENT node space)
   * @param target the target position (in PARENT node space)
   * @param up the up direction (in PARENT node space)
   * NOTE: it will override any relative transformation w.r.t its parent node
   */
  RenderCamera(scene::SceneNode& node,
               const Magnum::Vector3& eye,
               const Magnum::Vector3& target,
               const Magnum::Vector3& up);
  /**
   * @brief Reset the initial viewing parameters of the camera
   * @param eye the eye position (in PARENT node space)
   * @param target the target position (in PARENT node space)
   * @param up the up direction (in PARENT node space)
   * @return Reference to self (for method chaining)
   * NOTE: it will override any relative transformation w.r.t its parent node
   */
  virtual RenderCamera& resetViewingParameters(const Magnum::Vector3& eye,
                                               const Magnum::Vector3& target,
                                               const Magnum::Vector3& up);
  /**
   * @brief Tell if the camera is attached to the scene graph
   * @return true if it is attached to this scene graph, otherwise false
   */
  bool isInSceneGraph(const scene::SceneGraph& sceneGraph);

  /**
   * @brief destructor
   * do nothing, let magnum handle the camera
   */
  ~RenderCamera() override = default;

  /**
   * @brief Get the scene node being attached to.
   */
  scene::SceneNode& node() { return object(); }

  /**
   * @brief Get a const ref to the scene node being attached to.
   */
  const scene::SceneNode& node() const { return object(); }

  /** @overload */
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(MagnumCamera::object());
  }

  /** @overload */
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(MagnumCamera::object());
  }

  /**
   * @brief Set precalculated projection matrix for this RenderCamera
   * @param width The width of the viewport
   * @param height The height of the viewport
   * @param projMat The projection matrix to use.
   * @return A reference to this RenderCamera
   */
  RenderCamera& setProjectionMatrix(int width,
                                    int height,
                                    Mn::Matrix4& projMat) {
    MagnumCamera::setProjectionMatrix(projMat).setViewport(
        Magnum::Vector2i(width, height));
    invertedProjectionMatrix = projectionMatrix().inverted();
    return *this;
  }

  /**
   * @brief Set precalculated projection matrix for this RenderCamera
   * @param width The width of the viewport
   * @param height The height of the viewport
   * @param znear The location of the near clipping plane
   * @param zfar The location of the far clipping plane
   * @param hfov The horizontal field of view.
   * @return A reference to this RenderCamera
   */
  RenderCamera& setProjectionMatrix(int width,
                                    int height,
                                    float znear,
                                    float zfar,
                                    Mn::Deg hfov);

  /**
   * @brief Set projection matrix for Orthographic camera
   * @param width The width of the viewport
   * @param height The height of the viewport
   * @param znear The location of the near clipping plane
   * @param zfar The location of the far clipping plane
   * @param scale A multiplier to scale the size of the resultant image.
   * @return A reference to this RenderCamera
   */
  RenderCamera& setOrthoProjectionMatrix(int width,
                                         int height,
                                         float znear,
                                         float zfar,
                                         float scale);

  /**
   * @brief Overload function to render the drawables
   * @param drawables a drawable group containing all the drawables
   * @param flags state flags to direct drawing
   * @return the number of drawables that are drawn
   */
  uint32_t draw(MagnumDrawableGroup& drawables, Flags flags = {});

  uint32_t draw(DrawableTransforms& drawableTransforms, Flags flags = {});

  /**
   * @brief performs the frustum culling
   * @param drawableTransforms a vector of pairs of Drawable3D object and its
   * absolute transformation
   * @return the number of drawables that are not culled
   *
   * NOTE: user are not encouraged to call this function directly.
   * The preferred way is to enable the frustum culling by calling @ref
   * setFrustumCullingEnabled and then call @ref draw
   */
  size_t cull(DrawableTransforms& drawableTransforms);

  /**
   * @brief Cull Drawables for SceneNodes which are not OBJECT type.
   *
   * @param drawableTransforms a vector of pairs of Drawable3D object and its
   * absolute transformation
   * @return the number of drawables that are not culled
   */
  size_t removeNonObjects(DrawableTransforms& drawableTransforms);

  size_t filterTransforms(DrawableTransforms& drawableTransforms,
                          Flags flags = {});

  /**
   * @brief if the "immediate" following rendering pass is to use drawable ids
   * as the object ids.
   * By default, it uses the semantic_id, stored in the drawable's scene graph
   * node, if no "per-vertex" object id is used.
   * @return true, if it is to use drawable ids as the object ids in the
   * following rendering pass, otherwise false
   */
  bool useDrawableIds() const { return useDrawableIds_; }

  /**
   * @brief Unproject a 2D viewport point to a 3D ray with origin at camera
   * position. Ray direction is optionally normalized. Non-normalized rays
   * originate at the camera location and terminate at a view plane one unit
   * down the Z axis.
   *
   * @param viewportPosition The 2D point on the viewport to unproject
   * ([0,width], [0,height]).
   * @param normalized If true(default), normalize ray direction.
   * @return a @ref esp::geo::Ray with unit length direction or zero direction
   * if failed.
   */
  esp::geo::Ray unproject(const Mn::Vector2i& viewportPosition,
                          bool normalized = true);

  /**
   * @brief Query the cached number of Drawables visible after frustum culling
   * for the most recent render pass.
   */
  size_t getPreviousNumVisibleDrawables() const {
    return previousNumVisibleDrawables_;
  }

 protected:
  //! cached inverted projection matrix to save compute on repeated calls (e.g.
  //! to unproject) without moving the camera
  Mn::Matrix4 invertedProjectionMatrix;
  size_t previousNumVisibleDrawables_ = 0;
  bool useDrawableIds_ = false;
  ESP_SMART_POINTERS(RenderCamera)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_RENDERCAMERA_H_
