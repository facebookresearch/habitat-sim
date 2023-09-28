// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "CubeMapSensorBase.h"
#include "esp/core/Check.h"
#include "esp/sim/Simulator.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace sensor {

CubeMapSensorBaseSpec::CubeMapSensorBaseSpec() : VisualSensorSpec() {
  uuid = "cubemap_sensor_base";
  sensorSubType = SensorSubType::None;
}

void CubeMapSensorBaseSpec::sanityCheck() const {
  VisualSensorSpec::sanityCheck();
  if (cubemapSize != Cr::Containers::NullOpt) {
    ESP_CHECK(*cubemapSize > 0,
              "CubeMapSensorBaseSpec::sanityCheck(): the size of the cubemap,"
                  << *cubemapSize << "is illegal.");
  }
}

int computeCubemapSize(const esp::vec2i& resolution,
                       const Cr::Containers::Optional<int>& cubemapSize) {
  int size = (resolution[0] < resolution[1] ? resolution[0] : resolution[1]);
  // if user sets the size of the cubemap, use it
  if (cubemapSize != Corrade::Containers::NullOpt) {
    size = *cubemapSize;
  }
  return size;
}

CubeMapSensorBase::CubeMapSensorBase(scene::SceneNode& cameraNode,
                                     const CubeMapSensorBaseSpec::ptr& spec)
    : VisualSensor(cameraNode, spec),
      cubeMapCamera_(new gfx::CubeMapCamera(cameraNode)),
      mesh_(Mn::GL::Mesh()) {
  // initialize a cubemap
  int size = computeCubemapSize(cubeMapSensorBaseSpec_->resolution,
                                cubeMapSensorBaseSpec_->cubemapSize);
  gfx::CubeMap::Flags cubeMapFlags = {};
  switch (cubeMapSensorBaseSpec_->sensorType) {
    case SensorType::Color:
      cubeMapFlags |= gfx::CubeMap::Flag::ColorTexture;
      break;
    case SensorType::Depth:
      cubeMapFlags |= gfx::CubeMap::Flag::DepthTexture;
      break;
    case SensorType::Semantic:
      cubeMapFlags |= gfx::CubeMap::Flag::ObjectIdTexture;
      break;
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
  cubeMap_ = esp::gfx::CubeMap{size, cubeMapFlags};

  // Sets the cubemap camera, it attaches to the same node as the sensor
  // You do not have to release it in the dtor since magnum scene graph will
  // handle it. The cubemap is initialized in the member list above.
  cubeMapCamera_->setProjectionMatrix(size, cubeMapSensorBaseSpec_->near,
                                      cubeMapSensorBaseSpec_->far);

  // setup shader flags
  switch (cubeMapSensorBaseSpec_->sensorType) {
    case SensorType::Color:
      cubeMapShaderBaseFlags_ |= gfx::CubeMapShaderBase::Flag::ColorTexture;
      break;
    case SensorType::Depth:
      cubeMapShaderBaseFlags_ |= gfx::CubeMapShaderBase::Flag::DepthTexture;
      break;
    case SensorType::Semantic:
      cubeMapShaderBaseFlags_ |= gfx::CubeMapShaderBase::Flag::ObjectIdTexture;
      break;
    // sensor type list is too long, have to use default
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }

  // prepare a big triangle mesh to cover the screen
  // initialied in member initializer list
  mesh_.setCount(3);
}

bool CubeMapSensorBaseSpec::operator==(const CubeMapSensorBaseSpec& a) const {
  return (VisualSensorSpec::operator==(a) && cubemapSize == a.cubemapSize);
}

bool CubeMapSensorBase::renderToCubemapTexture(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  // in case the fisheye sensor resolution changed at runtime
  {
    int size = computeCubemapSize(cubeMapSensorBaseSpec_->resolution,
                                  cubeMapSensorBaseSpec_->cubemapSize);
    bool reset = cubeMap_->reset(size);
    if (reset) {
      cubeMapCamera_->setProjectionMatrix(size, cubeMapSensorBaseSpec_->near,
                                          cubeMapSensorBaseSpec_->far);
    }
  }

  esp::gfx::RenderCamera::Flags flags = {
      gfx::RenderCamera::Flag::ClearColor |
      gfx::RenderCamera::Flag::ClearDepth |
      gfx::RenderCamera::Flag::ClearObjectId};
  if (sim.isFrustumCullingEnabled()) {
    flags |= gfx::RenderCamera::Flag::FrustumCulling;
  }

  // generate the cubemap texture
  const char* defaultDrawableGroupName = "";
  if (cubeMapSensorBaseSpec_->sensorType == SensorType::Semantic) {
    bool twoSceneGraphs =
        (&sim.getActiveSemanticSceneGraph() != &sim.getActiveSceneGraph());

    if (twoSceneGraphs) {
      VisualSensor::MoveSemanticSensorNodeHelper helper(*this, sim);
      cubeMap_->renderToTexture(*cubeMapCamera_,
                                sim.getActiveSemanticSceneGraph(),
                                defaultDrawableGroupName, flags);
    } else {
      cubeMap_->renderToTexture(*cubeMapCamera_,
                                sim.getActiveSemanticSceneGraph(),
                                defaultDrawableGroupName, flags);
    }

    if (twoSceneGraphs) {
      flags |= gfx::RenderCamera::Flag::ObjectsOnly;
      // Incremental rendering:
      // BE AWARE that here "ClearColor", "ClearDepth" and "ClearObjectId" are
      // NOT set!! Rendering happens on top of whatever existing there.
      flags &= ~gfx::RenderCamera::Flag::ClearColor;
      flags &= ~gfx::RenderCamera::Flag::ClearDepth;
      flags &= ~gfx::RenderCamera::Flag::ClearObjectId;
      cubeMap_->renderToTexture(*cubeMapCamera_, sim.getActiveSceneGraph(),
                                defaultDrawableGroupName, flags);
    }
  } else {
    cubeMap_->renderToTexture(*cubeMapCamera_, sim.getActiveSceneGraph(),
                              defaultDrawableGroupName, flags);
  }

  return true;
}

void CubeMapSensorBase::drawWith(gfx::CubeMapShaderBase& shader) {
  if (cubeMapSensorBaseSpec_->sensorType == SensorType::Color) {
    shader.bindColorTexture(
        cubeMap_->getTexture(gfx::CubeMap::TextureType::Color));
  }
  if (cubeMapSensorBaseSpec_->sensorType == SensorType::Depth) {
    shader.bindDepthTexture(
        cubeMap_->getTexture(gfx::CubeMap::TextureType::Depth));
  }
  if (cubeMapSensorBaseSpec_->sensorType == SensorType::Semantic) {
    shader.bindObjectIdTexture(
        cubeMap_->getTexture(gfx::CubeMap::TextureType::ObjectId));
  }

  renderTarget().renderEnter();
  shader.draw(mesh_);
  // TODO: Add support for CubeMap-based sensors in HBAO
  // if (cubeMapSensorBaseSpec_->sensorType == SensorType::Color) {
  //   renderTarget().tryDrawHbao();
  // }
  renderTarget().renderExit();
}

}  // namespace sensor
}  // namespace esp
