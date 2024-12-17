// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneLoader.h"

#include <iostream>
#include <string>
#include <vector>

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/Path.h>
#include "esp/assets/GenericSemanticMeshData.h"
#include "esp/core/Esp.h"
#include "esp/geo/Geo.h"
#include "esp/metadata/attributes/AttributesEnumMaps.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Concatenate.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/SceneTools/Hierarchy.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/MeshData.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace assets {

SceneLoader::SceneLoader()
#ifdef MAGNUM_BUILD_STATIC
    :  // avoid using plugins that might depend on different library versions
      importerManager_("nonexistent")
#endif
{
}

MeshData SceneLoader::load(const AssetInfo& info) {
  MeshData mesh;

  Cr::Containers::Pointer<Importer> importer = importerManager_.loadAndInstantiate("AnySceneImporter");
  ESP_CHECK(importer && importer->openFile(info.filepath),
      "Error opening" << info.filepath);

  if (info.type == metadata::attributes::AssetType::InstanceMesh) {
    // dummy colormap
    std::vector<Magnum::Vector3ub> dummyColormap;
    Cr::Containers::Optional<Mn::Trade::MeshData> meshData = importer->mesh(0);
    ESP_CHECK(meshData, "Error loading mesh data from" << info.filepath);

    GenericSemanticMeshData::uptr instanceMeshData =
        GenericSemanticMeshData::buildSemanticMeshData(*meshData, info.filepath,
                                                       dummyColormap, false);

    const auto& vbo = instanceMeshData->getVertexBufferObjectCPU();
    const auto& cbo = instanceMeshData->getColorBufferObjectCPU();
    const auto& ibo = instanceMeshData->getIndexBufferObjectCPU();

    mesh.vbo.resize(vbo.size());
    Cr::Utility::copy(vbo, Cr::Containers::arrayCast<Mn::Vector3>(
                               Cr::Containers::arrayView(mesh.vbo)));
    mesh.ibo = ibo;
    for (const auto& c : cbo) {
      auto clr = Mn::EigenIntegration::cast<esp::vec3uc>(c);
      mesh.cbo.emplace_back(clr.cast<float>() / 255.0f);
    }
  } else {
    /* Get all meshes */
    Cr::Containers::Array<Mn::Trade::MeshData> meshes;
    Cr::Containers::Optional<Mn::Trade::MeshData> meshData;
    for(Mn::UnsignedInt i = 0; i != importer->meshCount(); ++i) {
      Cr::Containers::Optional<Mn::Trade::MeshData> meshData = importer->mesh(i);
      ESP_CHECK(meshData, "Error loading mesh data from" << info.filepath);
      arrayAppend(meshes, *std::move(meshData));
    }

    /* Get absolute transformations for all objects with a mesh assigned */
    Cr::Containers::Optional<Mn::Trade::SceneData> scene = importer->scene(0);
    ESP_CHECK(scene, "Error loading scene data from" << info.filepath);

    Cr::Containers::Array<Cr::Containers::Pair<Mn::UnsignedInt, Cr::Containers::Pair<Mn::UnsignedInt, Mn::Int>>> meshesMaterials = scene->meshesMaterialsAsArray();
    /* Add an extra transform to align to habitat's gravity. Keeping the
       original Eigen expression just to avoid some silly error, worthy of a
       future cleanup. */
    const quatf alignSceneToEspGravityEigen =
        quatf::FromTwoVectors(info.frame.gravity(), esp::geo::ESP_GRAVITY);
    const Mn::Matrix4 alignSceneToEspGravity = Mn::Matrix4::from(Mn::Quaternion{alignSceneToEspGravityEigen}.toMatrix(), {});
    Cr::Containers::Array<Mn::Matrix4> transformations = Mn::SceneTools::absoluteFieldTransformations3D(*scene, Mn::Trade::SceneField::Mesh, alignSceneToEspGravity);

    /* Apply those transforms to meshes, concatenate them all together */
    Cr::Containers::Array<Mn::Trade::MeshData> flattenedMeshes;
    for(std::size_t i = 0; i != meshesMaterials.size(); ++i) {
        arrayAppend(flattenedMeshes, Mn::MeshTools::transform3D(
            meshes[meshesMaterials[i].second().first()], transformations[i]));
    }
    Mn::Trade::MeshData concatenated = Mn::MeshTools::concatenate(flattenedMeshes);

    /* Extract data from the nice and tidy Magnum MeshData into a bunch of STL
       vectors, worthy of a future cleanup as well */
    {
      mesh.vbo.resize(concatenated.vertexCount());
      concatenated.positions3DInto(Cr::Containers::arrayCast<Mn::Vector3>(Cr::Containers::arrayView(mesh.vbo)));
    }
    if(concatenated.hasAttribute(Mn::Trade::MeshAttribute::Normal)) {
      mesh.nbo.resize(concatenated.vertexCount());
      concatenated.normalsInto(Cr::Containers::arrayCast<Mn::Vector3>(Cr::Containers::arrayView(mesh.nbo)));
    }
    if(concatenated.hasAttribute(Mn::Trade::MeshAttribute::TextureCoordinates)) {
      mesh.tbo.resize(concatenated.vertexCount());
      concatenated.textureCoordinates2DInto(Cr::Containers::arrayCast<Mn::Vector2>(Cr::Containers::arrayView(mesh.tbo)));
    }
    if(concatenated.hasAttribute(Mn::Trade::MeshAttribute::Color)) {
      mesh.cbo.resize(concatenated.vertexCount());
      /* The colors are generally four-component, copy just the first 3
         components */
      Cr::Containers::Array<Mn::Color4> colors = concatenated.colorsAsArray();
      Cr::Utility::copy(stridedArrayView(colors).slice(&Mn::Color4::rgb),
        Cr::Containers::arrayCast<Mn::Color3>(Cr::Containers::arrayView(mesh.cbo)));
    }
  }

  ESP_DEBUG() << "Loaded" << mesh.vbo.size() << "vertices," << mesh.ibo.size()
              << "indices";

  return mesh;
};

}  // namespace assets
}  // namespace esp
