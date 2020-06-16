// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneLoader.h"

#include <iostream>
#include <string>
#include <vector>

#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/io/io.h"

#include <sophus/so3.hpp>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <Magnum/Trade/AbstractImporter.h>

namespace Cr = Corrade;

namespace esp {
namespace assets {

SceneLoader::SceneLoader()
    :
#ifdef MAGNUM_BUILD_STATIC
      // avoid using plugins that might depend on different library versions
      importerManager_("nonexistent")
#endif
          {};

MeshData SceneLoader::load(const AssetInfo& info) {
  MeshData mesh;
  if (!esp::io::exists(info.filepath)) {
    LOG(ERROR) << "Could not find file " << info.filepath;
    return mesh;
  }

  if (info.type == AssetType::INSTANCE_MESH) {
    Cr::Containers::Pointer<Importer> importer;
    CORRADE_INTERNAL_ASSERT_OUTPUT(
        importer = importerManager_.loadAndInstantiate("StanfordImporter"));
    GenericInstanceMeshData::ptr instanceMeshData =
        GenericInstanceMeshData::fromPLY(*importer, info.filepath);

    const auto& vbo = instanceMeshData->getVertexBufferObjectCPU();
    const auto& cbo = instanceMeshData->getColorBufferObjectCPU();
    const auto& ibo = instanceMeshData->getIndexBufferObjectCPU();
    mesh.vbo = vbo;
    mesh.ibo = ibo;
    for (const auto& c : cbo) {
      mesh.cbo.emplace_back(c.cast<float>() / 255.0f);
    }
  } else {
    const aiScene* scene;
    Assimp::Importer Importer;

    // Flags for loading the mesh
    static const int assimpFlags =
        aiProcess_Triangulate | aiProcess_PreTransformVertices;

    scene = Importer.ReadFile(info.filepath.c_str(), assimpFlags);

    const quatf alignSceneToEspGravity =
        quatf::FromTwoVectors(info.frame.gravity(), esp::geo::ESP_GRAVITY);

    // Iterate through all meshes in the file and extract the vertex components
    for (uint32_t m = 0, indexBase = 0; m < scene->mNumMeshes; ++m) {
      const aiMesh& assimpMesh = *scene->mMeshes[m];
      for (uint32_t v = 0; v < assimpMesh.mNumVertices; ++v) {
        // Use Eigen::Map to convert ASSIMP vectors to eigen vectors
        const Eigen::Map<const vec3f> xyz_scene(&assimpMesh.mVertices[v].x);
        const vec3f xyz_esp = alignSceneToEspGravity * xyz_scene;
        mesh.vbo.push_back(xyz_esp);

        if (assimpMesh.mNormals) {
          const Eigen::Map<const vec3f> normal_scene(&assimpMesh.mNormals[v].x);
          const vec3f normal_esp = alignSceneToEspGravity * normal_scene;
          mesh.nbo.push_back(normal_esp);
        }

        if (assimpMesh.HasTextureCoords(0)) {
          const Eigen::Map<const vec2f> texCoord(
              &assimpMesh.mTextureCoords[0][v].x);
          mesh.tbo.push_back(texCoord);
        }

        if (assimpMesh.HasVertexColors(0)) {
          const Eigen::Map<const vec3f> color(&assimpMesh.mColors[0][v].r);
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

  LOG(INFO) << "Loaded " << mesh.vbo.size() << " vertices, " << mesh.ibo.size()
            << " indices";

  return mesh;
};

}  // namespace assets
}  // namespace esp
