/*
    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include "ShadowLight.h"

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Frustum.h>
#include <Magnum/SceneGraph/FeatureGroup.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <algorithm>

#include "esp/geo/geo.h"
#include "esp/gfx/shadows/ShadowCasterDrawable.h"
#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;
namespace esp {
namespace gfx {

ShadowLight::ShadowLight(scene::SceneNode& parent)
    : Mn::SceneGraph::Camera3D{parent}, shadowTexture_{Mn::NoCreate} {
  setAspectRatioPolicy(Mn::SceneGraph::AspectRatioPolicy::NotPreserved);
}

void ShadowLight::setupShadowmaps(Mn::Int numShadowLevels,
                                  const Mn::Vector2i& size) {
  layers_.clear();

  (shadowTexture_ = Mn::GL::Texture2DArray{})
      .setImage(0, Mn::GL::TextureFormat::DepthComponent,
                Mn::ImageView3D{Mn::GL::PixelFormat::DepthComponent,
                                Mn::GL::PixelType::Float,
                                {size, numShadowLevels}})
      .setMaxLevel(0)
      .setCompareFunction(Mn::GL::SamplerCompareFunction::LessOrEqual)
      .setCompareMode(Mn::GL::SamplerCompareMode::CompareRefToTexture)
      .setMinificationFilter(Mn::GL::SamplerFilter::Linear,
                             Mn::GL::SamplerMipmap::Base)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear);

  for (std::int_fast32_t i = 0; i < numShadowLevels; ++i) {
    layers_.emplace_back(size);
    Mn::GL::Framebuffer& shadowFramebuffer = layers_.back().shadowFramebuffer;
    shadowFramebuffer
        .attachTextureLayer(Mn::GL::Framebuffer::BufferAttachment::Depth,
                            shadowTexture_, 0, i)
        .mapForDraw(Mn::GL::Framebuffer::DrawAttachment::None)
        .bind();
    CORRADE_INTERNAL_ASSERT(
        shadowFramebuffer.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
        Mn::GL::Framebuffer::Status::Complete);
  }
}

ShadowLight::ShadowLayerData::ShadowLayerData(const Mn::Vector2i& size)
    : shadowFramebuffer{{{}, size}} {}

void ShadowLight::setTarget(const Mn::Vector3& lightDirection,
                            const Mn::Vector3& screenDirection,
                            MagnumCamera& mainCamera) {
  Mn::Matrix4 cameraMatrix =
      Mn::Matrix4::lookAt({}, -lightDirection, screenDirection);
  lightDirection_ = lightDirection;
  const Mn::Matrix3x3 cameraRotationMatrix = cameraMatrix.rotation();
  const Mn::Matrix3x3 inverseCameraRotationMatrix =
      cameraRotationMatrix.inverted();

  for (std::size_t layerIndex = 0; layerIndex != layers_.size(); ++layerIndex) {
    std::vector<Mn::Vector3> mainCameraFrustumCorners =
        layerFrustumCorners(mainCamera, Mn::Int(layerIndex));
    ShadowLayerData& layer = layers_[layerIndex];

    /* Calculate the AABB in shadow-camera space */
    Mn::Vector3 min{std::numeric_limits<Mn::Float>::max()},
        max{std::numeric_limits<Mn::Float>::lowest()};
    for (Mn::Vector3 worldPoint : mainCameraFrustumCorners) {
      Mn::Vector3 cameraPoint = inverseCameraRotationMatrix * worldPoint;
      min = Mn::Math::min(min, cameraPoint);
      max = Mn::Math::max(max, cameraPoint);
    }

    /* Place the shadow camera at the mid-point of the camera box */
    const Mn::Vector3 mid = (min + max) * 0.5f;
    const Mn::Vector3 cameraPosition = cameraRotationMatrix * mid;

    const Mn::Vector3 range = max - min;
    /* Set up the initial extends of the shadow map's render volume. Note
       we will adjust this later when we render. */
    layer.orthographicSize = range.xy();
    layer.orthographicNear = -0.5f * range.z();
    layer.orthographicFar = 0.5f * range.z();
    cameraMatrix.translation() = cameraPosition;
    layer.shadowCameraMatrix = cameraMatrix;
  }
}

Mn::Float ShadowLight::cutZ(const Mn::Int layer) const {
  return layers_[layer].cutPlane;
}

void ShadowLight::setupSplitDistances(const Mn::Float zNear,
                                      const Mn::Float zFar,
                                      const Mn::Float power) {
  /* props http://stackoverflow.com/a/33465663 */
  for (std::size_t i = 0; i != layers_.size(); ++i) {
    const Mn::Float linearDepth =
        zNear +
        std::pow(Mn::Float(i + 1) / layers_.size(), power) * (zFar - zNear);
    const Mn::Float nonLinearDepth =
        (zFar + zNear - 2.0f * zNear * zFar / linearDepth) / (zFar - zNear);
    layers_[i].cutPlane = (nonLinearDepth + 1.0f) / 2.0f;
  }
}

Mn::Float ShadowLight::cutDistance(const Mn::Float zNear,
                                   const Mn::Float zFar,
                                   const Mn::Int layer) const {
  const Mn::Float depthSample = 2.0f * layers_[layer].cutPlane - 1.0f;
  const Mn::Float zLinear =
      2.0f * zNear * zFar / (zFar + zNear - depthSample * (zFar - zNear));
  return zLinear;
}

std::vector<Mn::Vector3> ShadowLight::layerFrustumCorners(
    MagnumCamera& mainCamera,
    const Mn::Int layer) {
  const Mn::Float z0 = layer == 0 ? 0 : layers_[layer - 1].cutPlane;
  const Mn::Float z1 = layers_[layer].cutPlane;
  return cameraFrustumCorners(mainCamera, z0, z1);
}

std::vector<Mn::Vector3> ShadowLight::cameraFrustumCorners(
    MagnumCamera& mainCamera,
    const Mn::Float z0,
    const Mn::Float z1) {
  const Mn::Matrix4 imvp =
      (mainCamera.projectionMatrix() * mainCamera.cameraMatrix()).inverted();
  return frustumCorners(imvp, z0, z1);
}

std::vector<Mn::Vector3> ShadowLight::frustumCorners(const Mn::Matrix4& imvp,
                                                     const Mn::Float z0,
                                                     const Mn::Float z1) {
  return {imvp.transformPoint({-1, -1, z0}), imvp.transformPoint({1, -1, z0}),
          imvp.transformPoint({-1, 1, z0}),  imvp.transformPoint({1, 1, z0}),
          imvp.transformPoint({-1, -1, z1}), imvp.transformPoint({1, -1, z1}),
          imvp.transformPoint({-1, 1, z1}),  imvp.transformPoint({1, 1, z1})};
}

bool ShadowLight::isCulledByPlanes(
    const Mn::Range3D& bb,
    const std::vector<Magnum::Vector4>& clipPlanes) const {
  const Mn::Vector3 center = bb.center();
  const Mn::Vector3 extent = bb.max() - bb.min();
  return std::any_of(clipPlanes.begin(), clipPlanes.end(),
                     [&center, &extent](const Magnum::Vector4& plane) {
                       const Mn::Vector3 absPlaneNormal =
                           Mn::Math::abs(plane.xyz());

                       const float d = Mn::Math::dot(center, plane.xyz());
                       const float r = Mn::Math::dot(extent, absPlaneNormal);
                       return (d + r < -2.0 * plane.w());
                     });
}

void ShadowLight::generateLayerShadowMap(ShadowLayerData& d,
                                         MagnumDrawableGroup& drawables) {
  Mn::Float orthographicNear = d.orthographicNear;
  const Mn::Float orthographicFar = d.orthographicFar;

  /* Move this whole object to the right place to render each layer */
  object().setTransformation(d.shadowCameraMatrix).setClean();
  setProjectionMatrix(Mn::Matrix4::orthographicProjection(
      d.orthographicSize, orthographicNear, orthographicFar));

  // const std::vector<Mn::Vector4> clipPlanes = calculateClipPlanes();
  // camera frustum in camera coordinates
  const Mn::Frustum frustum = Mn::Frustum::fromMatrix(projectionMatrix());
  // Skip out the near plane because we need to include shadow casters
  // traveling the direction the camera is facing.
  const std::vector<Mn::Vector4> clipPlanes{frustum.left(), frustum.right(),
                                            frustum.top(), frustum.bottom(),
                                            frustum.far()};
  std::vector<std::pair<std::reference_wrapper<Magnum::SceneGraph::Drawable3D>,
                        Magnum::Matrix4>>
      drawableTransforms = drawableTransformations(drawables);

  /* Rebuild the list of objects we will draw by clipping them with the
     shadow camera's planes */
  std::size_t transformationsOutIndex = 0;
  drawableTransforms.erase(
      std::remove_if(drawableTransforms.begin(), drawableTransforms.end(),
                     [&clipPlanes, &orthographicNear,
                      this](const std::pair<
                            std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                            Mn::Matrix4>& drawableTransform) {
                       auto& node = static_cast<scene::SceneNode&>(
                           drawableTransform.first.get().object());

                       // get this objects bounding box relative to camera
                       Magnum::Range3D range = geo::getTransformedBB(
                           node.getMeshBB(), drawableTransform.second);

                       if (isCulledByPlanes(range, clipPlanes))
                         return true;

                       /* If this object extends in front of the near plane,
                          extend the near plane. We negate the z because the
                          negative z is forward away from the camera, but the
                          near/far planes are measured forwards. */
                       // TODO: need to implement proper distance
                       const Mn::Float nearestPoint = -100.f;
                       orthographicNear =
                           Mn::Math::min(orthographicNear, nearestPoint);
                       return false;
                     }),
      drawableTransforms.end());

  /* Recalculate the projection matrix with new near plane. */
  const Mn::Matrix4 shadowCameraProjectionMatrix =
      Mn::Matrix4::orthographicProjection(d.orthographicSize, orthographicNear,
                                          orthographicFar);

  /* Projecting world points normalized device coordinates means they range
  -1 -> 1. Use this bias matrix so we go straight from world -> texture
  space */
  constexpr const Mn::Matrix4 bias{{0.5f, 0.0f, 0.0f, 0.0f},
                                   {0.0f, 0.5f, 0.0f, 0.0f},
                                   {0.0f, 0.0f, 0.5f, 0.0f},
                                   {0.5f, 0.5f, 0.5f, 1.0f}};
  d.shadowMatrix = bias * shadowCameraProjectionMatrix * cameraMatrix();
  setProjectionMatrix(shadowCameraProjectionMatrix);

  d.shadowFramebuffer.clear(Mn::GL::FramebufferClear::Depth).bind();
  draw(drawableTransforms);
}

void ShadowLight::generateShadowMaps(MagnumDrawableGroup& drawables) {
  /* Compute transformations of all objects in the group relative to the
   * camera
   */
  Mn::GL::Renderer::setDepthMask(true);

  for (auto& layer : layers_) {
    generateLayerShadowMap(layer, drawables);
  }

  Mn::GL::defaultFramebuffer.bind();
}

}  // namespace gfx
}  // namespace esp
