// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneLoader.h"

#include <iostream>
#include <string>
#include <vector>

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Path.h>
#include "esp/assets/GenericSemanticMeshData.h"
#include "esp/core/Esp.h"
#include "esp/core/Utility.h"
#include "esp/geo/Geo.h"
#include "esp/metadata/attributes/AttributesEnumMaps.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <Magnum/Trade/AbstractImporter.h>

namespace Cr = Corrade;

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
  if (!Cr::Utility::Path::exists(info.filepath)) {
    ESP_ERROR() << "Could not find file" << info.filepath;
    return mesh;
  }

  if (info.type == metadata::attributes::AssetType::InstanceMesh) {
    Cr::Containers::Pointer<Importer> importer;
    CORRADE_INTERNAL_ASSERT_OUTPUT(
        importer = importerManager_.loadAndInstantiate("StanfordImporter"));
    // dummy colormap
    std::vector<Magnum::Vector3ub> dummyColormap;
    Cr::Containers::Optional<Mn::Trade::MeshData> meshData;

    ESP_CHECK(
        (importer->openFile(info.filepath) && (meshData = importer->mesh(0))),
        Cr::Utility::formatString(
            "Error loading instance mesh data from file {}", info.filepath));

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
      mesh.cbo.emplace_back(c / 255.0f);
    }
  } else {
    const aiScene* scene;
    Assimp::Importer Importer;

    // Flags for loading the mesh
    static const int assimpFlags =
        aiProcess_Triangulate | aiProcess_PreTransformVertices;

    scene = Importer.ReadFile(info.filepath.c_str(), assimpFlags);

    const Mn::Quaternion alignSceneToEspGravity = core::quatRotFromTwoVectors(
        info.frame.gravity(), esp::geo::ESP_GRAVITY);

    // Iterate through all meshes in the file and extract the vertex components
    for (uint32_t m = 0, indexBase = 0; m < scene->mNumMeshes; ++m) {
      const aiMesh& assimpMesh = *scene->mMeshes[m];
      for (uint32_t v = 0; v < assimpMesh.mNumVertices; ++v) {
        // Use Eigen::Map to convert ASSIMP vectors to eigen vectors
        const Mn::Vector3 xyz_scene =
            Mn::Vector3::from(&assimpMesh.mVertices[v].x);
        const Mn::Vector3 xyz_esp =
            alignSceneToEspGravity.transformVectorNormalized(xyz_scene);
        mesh.vbo.push_back(xyz_esp);

        if (assimpMesh.mNormals) {
          const Mn::Vector3 normal_scene =
              Mn::Vector3::from(&assimpMesh.mNormals[v].x);
          const Mn::Vector3 normal_esp =
              alignSceneToEspGravity.transformVectorNormalized(normal_scene);
          mesh.nbo.push_back(normal_esp);
        }

        if (assimpMesh.HasTextureCoords(0)) {
          const Mn::Vector2 texCoord =
              Mn::Vector2::from(&assimpMesh.mTextureCoords[0][v].x);
          mesh.tbo.push_back(texCoord);
        }

        if (assimpMesh.HasVertexColors(0)) {
          const Mn::Vector3 color =
              Mn::Vector3::from(&assimpMesh.mColors[0][v].r);
          mesh.cbo.push_back(color);
        }
      }  // vertices

      // Generate and append index buffer for mesh
      for (uint32_t f = 0; f < assimpMesh.mNumFaces; ++f) {
        const aiFace& face = assimpMesh.mFaces[f];
        for (uint32_t i = 0; i < face.mNumIndices; ++i) {
          mesh.ibo.push_back(face.mIndices[i] + indexBase);
        }
      }  // faces
      indexBase += assimpMesh.mNumVertices;
    }  // meshes
  }

  ESP_DEBUG() << "Loaded" << mesh.vbo.size() << "vertices," << mesh.ibo.size()
              << "indices";

  return mesh;
};

}  // namespace assets
}  // namespace esp
