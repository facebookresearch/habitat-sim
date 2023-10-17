// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchPlayerImplementation.h"

#include <esp/gfx_batch/Renderer.h>

#include <Corrade/Containers/StringStl.h>

namespace {
bool isSupportedRenderAsset(const Corrade::Containers::StringView& filepath) {
  // Primitives aren't directly supported in the Magnum batch renderer. See
  // https://docs.google.com/document/d/1ngA73cXl3YRaPfFyICSUHONZN44C-XvieS7kwyQDbkI/edit#bookmark=id.yq39718gqbwz

  const std::array<Corrade::Containers::StringView, 12> primNamePrefixes = {
      "capsule3DSolid",     "capsule3DWireframe", "coneSolid",
      "coneWireframe",      "cubeSolid",          "cubeWireframe",
      "cylinderSolid",      "cylinderWireframe",  "icosphereSolid",
      "icosphereWireframe", "uvSphereSolid",      "uvSphereWireframe"};

  // primitive render asset filepaths start with one of the above prefixes.
  // Examples: icosphereSolid_subdivs_1
  // capsule3DSolid_hemiRings_4_cylRings_1_segments_12_halfLen_3.25_useTexCoords_false_useTangents_false
  for (const auto& primNamePrefix : primNamePrefixes) {
    if (filepath.size() < primNamePrefix.size()) {
      continue;
    }

    if (filepath.prefix(primNamePrefix.size()) == primNamePrefix) {
      return false;
    }
  }

  return true;
}
}  // namespace

namespace esp {
namespace sim {

BatchPlayerImplementation::BatchPlayerImplementation(
    gfx_batch::Renderer& renderer,
    Mn::UnsignedInt sceneId)
    : renderer_{renderer}, sceneId_{sceneId} {}

gfx::replay::NodeHandle
BatchPlayerImplementation::loadAndCreateRenderAssetInstance(
    const esp::assets::AssetInfo& assetInfo,
    const esp::assets::RenderAssetInstanceCreationInfo& creation) {
  // TODO anything to use creation.flags for?
  // TODO is creation.lightSetupKey actually mapping to anything in the
  //  replay file?

  if (!::isSupportedRenderAsset(creation.filepath)) {
    ESP_WARNING() << "Unsupported render asset: " << creation.filepath;
    return nullptr;
  }

  /* If no such name is known yet, add as a file */
  if (!renderer_.hasNodeHierarchy(creation.filepath)) {
    ESP_WARNING()
        << creation.filepath
        << "not found in any composite file, loading from the filesystem";

    ESP_CHECK(
        renderer_.addFile(creation.filepath,
                          gfx_batch::RendererFileFlag::Whole |
                              gfx_batch::RendererFileFlag::GenerateMipmap),
        "addFile failed for " << creation.filepath);
    CORRADE_INTERNAL_ASSERT(renderer_.hasNodeHierarchy(creation.filepath));
  }

  return reinterpret_cast<gfx::replay::NodeHandle>(
      renderer_.addNodeHierarchy(
          sceneId_, creation.filepath,
          /* Baking the initial scaling and coordinate frame into the
             transformation */
          Mn::Matrix4::scaling(creation.scale ? *creation.scale
                                              : Mn::Vector3{1.0f}) *
              Mn::Matrix4::from(
                  Mn::Quaternion{assetInfo.frame.rotationFrameToWorld()}
                      .toMatrix(),
                  {}))
      /* Returning incremented by 1 because 0 (nullptr) is treated as an
         error */
      + 1);
}

void BatchPlayerImplementation::deleteAssetInstance(
    const gfx::replay::NodeHandle node) {
  // TODO actually remove from the scene instead of setting a zero scale
  renderer_.transformations(sceneId_)[reinterpret_cast<std::size_t>(node) - 1] =
      Mn::Matrix4{Mn::Math::ZeroInit};
}

void BatchPlayerImplementation::deleteAssetInstances(
    const std::unordered_map<gfx::replay::RenderAssetInstanceKey,
                             gfx::replay::NodeHandle>&) {
  renderer_.clear(sceneId_);
}

void BatchPlayerImplementation::setNodeTransform(
    const gfx::replay::NodeHandle node,
    const Mn::Vector3& translation,
    const Mn::Quaternion& rotation) {
  renderer_.transformations(sceneId_)[reinterpret_cast<std::size_t>(node) - 1] =
      Mn::Matrix4::from(rotation.toMatrix(), translation);
}

void BatchPlayerImplementation::setNodeTransform(
    const gfx::replay::NodeHandle node,
    const Mn::Matrix4& transform) {
  renderer_.transformations(sceneId_)[reinterpret_cast<std::size_t>(node) - 1] =
      transform;
}

Mn::Matrix4 BatchPlayerImplementation::hackGetNodeTransform(
    const gfx::replay::NodeHandle node) const {
  return renderer_.transformations(
      sceneId_)[reinterpret_cast<std::size_t>(node) - 1];
}

void BatchPlayerImplementation::changeLightSetup(
    const esp::gfx::LightSetup& lights) {
  if (!renderer_.maxLightCount()) {
    ESP_WARNING() << "Attempted to change" << lights.size()
                  << "lights for scene" << sceneId_
                  << "but the renderer is configured without lights";
    return;
  }

  renderer_.clearLights(sceneId_);
  for (std::size_t i = 0; i != lights.size(); ++i) {
    const gfx::LightInfo& light = lights[i];
    CORRADE_INTERNAL_ASSERT(light.model == gfx::LightPositionModel::Global);

    const std::size_t nodeId = renderer_.addEmptyNode(sceneId_);

    std::size_t lightId;  // NOLINT
    if (light.vector.w()) {
      renderer_.transformations(sceneId_)[nodeId] =
          Mn::Matrix4::translation(light.vector.xyz());
      lightId = renderer_.addLight(sceneId_, nodeId,
                                   gfx_batch::RendererLightType::Point);
    } else {
      /* The matrix will be partially NaNs if the light vector is in the
         direction of the Y axis, but that's fine -- we only really care
         about the Z axis direction, which is always the "target" vector
         normalized. */
      // TODO for more robustness use something that "invents" some
      //  arbitrary orthogonal axes instead of the NaNs, once Magnum has
      //  such utility
      renderer_.transformations(sceneId_)[nodeId] =
          Mn::Matrix4::lookAt({}, light.vector.xyz(), Mn::Vector3::yAxis());
      lightId = renderer_.addLight(sceneId_, nodeId,
                                   gfx_batch::RendererLightType::Directional);
    }

    renderer_.lightColors(sceneId_)[lightId] = light.color;
    // TODO use gfx::getAmbientLightColor(lights) once it's not hardcoded
    //  to an arbitrary value and once it's possible to change the ambient
    //  factor in the renderer at runtime (and not just in
    //  RendererConfiguration::setAmbientFactor())
    // TODO range, once Habitat has that
  }
}

void BatchPlayerImplementation::createRigInstance(
    int,
    const std::vector<std::string>&) {
  // Not implemented.
}

void BatchPlayerImplementation::deleteRigInstance(int) {
  // Not implemented.
}

void BatchPlayerImplementation::setRigPose(
    int,
    const std::vector<gfx::replay::Transform>&) {
  // Not implemented.
}
}  // namespace sim
}  // namespace esp
