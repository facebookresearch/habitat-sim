#pragma once
/*
    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/TextureArray.h>
#include <Magnum/Resource.h>
#include <Magnum/SceneGraph/AbstractFeature.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>

#include <esp/gfx/magnum.h>
#include <esp/scene/SceneNode.h>

namespace esp {
namespace gfx {

/**
@brief A special camera used to render shadow maps

The object it's attached to should face the direction that the light travels.
*/
class ShadowLight : public MagnumCamera {
 public:
  static std::vector<Magnum::Vector3> cameraFrustumCorners(
      MagnumCamera& mainCamera,
      Magnum::Float z0 = -1.0f,
      Magnum::Float z1 = 1.0f);

  static std::vector<Magnum::Vector3> frustumCorners(
      const Magnum::Matrix4& imvp,
      Magnum::Float z0,
      Magnum::Float z1);

  explicit ShadowLight(scene::SceneNode& parent);

  // Get the scene node being attached to.
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(MagnumCamera::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(MagnumCamera::object());
  }

  /**
   * @brief Initialize the shadow map texture array and framebuffers
   *
   * Should be called before @ref setupSplitDistances().
   */
  void setupShadowmaps(Magnum::Int numShadowLevels,
                       const Magnum::Vector2i& size);

  /**
   * @brief Set up the distances we should cut the view frustum along
   *
   * The distances will be distributed along a power series. Should be
   * called after @ref setupShadowmaps().
   */
  void setupSplitDistances(Magnum::Float cameraNear,
                           Magnum::Float cameraFar,
                           Magnum::Float power);

  /**
   * @brief Computes all the matrices for the shadow map splits
   * @param lightDirection    Direction of travel of the light
   * @param screenDirection   Crossed with light direction to determine
   *      orientation of the shadow maps. Use the forward direction of
   *      the camera for best resolution use, or use a constant value for
   *      more stable shadows.
   * @param mainCamera        The camera to use to determine the optimal
   *      splits (normally, the main camera that the shadows will be
   *      rendered to)
   *
   * Should be called whenever your camera moves.
   */
  void setTarget(const Magnum::Vector3& lightDirection,
                 const Magnum::Vector3& screenDirection,
                 MagnumCamera& mainCamera);

  /**
   * @brief Render a group of shadow-casting drawables to the shadow maps
   */
  void generateShadowMaps(MagnumDrawableGroup& drawables);

  std::vector<Magnum::Vector3> layerFrustumCorners(MagnumCamera& mainCamera,
                                                   Magnum::Int layer);

  Magnum::Float cutZ(Magnum::Int layer) const;

  Magnum::Float cutDistance(Magnum::Float zNear,
                            Magnum::Float zFar,
                            Magnum::Int layer) const;

  std::size_t layerCount() const { return layers_.size(); }

  const Magnum::Matrix4& layerMatrix(Magnum::Int layer) const {
    return layers_[layer].shadowMatrix;
  }

  std::vector<Magnum::Vector4> calculateClipPlanes();

  Magnum::GL::Texture2DArray& shadowTexture() { return shadowTexture_; }

  const Magnum::Vector3& getLightDirection() const { return lightDirection_; }

 private:
  struct ShadowLayerData {
    Magnum::GL::Framebuffer shadowFramebuffer;
    Magnum::Matrix4 shadowCameraMatrix;
    Magnum::Matrix4 shadowMatrix;
    Magnum::Vector2 orthographicSize;
    Magnum::Float orthographicNear, orthographicFar;
    Magnum::Float cutPlane;

    explicit ShadowLayerData(const Magnum::Vector2i& size);
  };

  void generateLayerShadowMap(ShadowLayerData& d,
                              MagnumDrawableGroup& drawables);

  bool isCulledByPlanes(const Magnum::Range3D& bb,
                        const std::vector<Magnum::Vector4>& clipPlanes) const;

  Magnum::GL::Texture2DArray shadowTexture_;

  std::vector<ShadowLayerData> layers_;

  Magnum::Vector3 lightDirection_;

  ESP_SMART_POINTERS(ShadowLight)
};

}  // namespace gfx
}  // namespace esp
