// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <iostream>
#include <string>
#include <unordered_map>

#include "SceneLoader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include "Mp3dInstanceMeshData.h"
#include "esp/core/Esp.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/SemanticScene.h"

using esp::assets::AssetInfo;
using esp::assets::MeshData;
using esp::assets::Mp3dInstanceMeshData;
using esp::assets::SceneLoader;
using esp::nav::NavMeshSettings;
using esp::nav::PathFinder;
using esp::scene::SemanticScene;

int createNavMesh(const std::string& meshFile, const std::string& navmeshFile) {
  SceneLoader loader;
  const AssetInfo info = AssetInfo::fromPath(meshFile);
  const MeshData mesh = loader.load(info);
  NavMeshSettings bs;
  bs.setDefaults();
  PathFinder pf;
  if (!pf.build(bs, mesh)) {
    ESP_ERROR() << "Failed to build navmesh";
    return 2;
  }
  if (!pf.saveNavMesh(navmeshFile)) {
    ESP_ERROR() << "Failed to save navmesh";
    return 3;
  }
  return 0;
}

int createGibsonSemanticMesh(const std::string& objFile,
                             const std::string& idsFile,
                             const std::string& semMeshFile) {
  ESP_DEBUG() << "createGibsonSemanticMesh";
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  std::string warn;
  std::string err;

  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                              objFile.c_str());
  if (!warn.empty()) {
    ESP_WARNING() << warn;
  }
  if (!err.empty()) {
    ESP_ERROR() << err;
  }
  if (!ret) {
    ESP_ERROR() << "Failed to load" << objFile;
    return 1;
  }

  std::ifstream file(idsFile, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<unsigned short> objectId(size / sizeof(short));
  file.read(reinterpret_cast<char*>(objectId.data()), size);
  if (!file) {
    ESP_ERROR() << "Failed to load" << idsFile;
    return 2;
  }

  size_t numVerts = attrib.vertices.size() / 3;

  std::ofstream f(semMeshFile, std::ios::out | std::ios::binary);
  f << "ply" << std::endl;
  f << "format binary_little_endian 1.0" << std::endl;
  f << "element vertex " << numVerts << std::endl;
  f << "property float x" << std::endl;
  f << "property float y" << std::endl;
  f << "property float z" << std::endl;
  f << "property uchar red" << std::endl;
  f << "property uchar green" << std::endl;
  f << "property uchar blue" << std::endl;
  f << "element face " << shapes[0].mesh.num_face_vertices.size() << std::endl;
  f << "property list uchar int vertex_indices" << std::endl;
  f << "property ushort object_id" << std::endl;
  f << "end_header" << std::endl;

  // We need to rotate to match .glb where -Z is gravity
  const auto transform =
      esp::quatf::FromTwoVectors(esp::vec3f::UnitY(), esp::vec3f::UnitZ());
  for (size_t i = 0; i < numVerts; ++i) {
    unsigned char gray[] = {0x80, 0x80, 0x80};
    float* components = &attrib.vertices[i * 3];
    Eigen::Map<esp::vec3f> vertex{components};
    vertex = transform * vertex;
    f.write(reinterpret_cast<char*>(components), sizeof(float) * 3);
    f.write(reinterpret_cast<char*>(gray), sizeof(gray));
  }

  size_t index_offset = 0;
  for (size_t i = 0; i < shapes[0].mesh.num_face_vertices.size(); ++i) {
    unsigned char fv = shapes[0].mesh.num_face_vertices[i];
    f.put(fv);

    for (size_t j = 0; j < fv; ++j) {
      tinyobj::index_t idx = shapes[0].mesh.indices[index_offset + j];
      f.write(reinterpret_cast<char*>(&idx.vertex_index),
              sizeof(idx.vertex_index));
    }
    index_offset += fv;
    f.write(reinterpret_cast<char*>(&objectId[i]), sizeof(objectId[i]));
  }

  f.close();

  return 0;
}

int createMp3dSemanticMesh(const std::string& plyFile,
                           const std::string& houseFile,
                           const std::string& semMeshFile) {
  SemanticScene semanticScene;
  bool success = SemanticScene::loadMp3dHouse(houseFile, semanticScene);
  if (!success) {
    ESP_ERROR() << "Failed loading MP3D house file" << houseFile;
    return 1;
  }
  const std::unordered_map<int, int>& objectIdMap =
      semanticScene.getSemanticIndexMap();

  Mp3dInstanceMeshData mp3dMesh;
  success = mp3dMesh.loadMp3dPLY(plyFile);
  if (!success) {
    ESP_ERROR() << "Failed parsing MP3D segmenation PLY" << plyFile;
    return 1;
  }

  success =
      mp3dMesh.saveSemMeshPLY(semMeshFile, semanticScene.getSemanticIndexMap());
  if (!success) {
    ESP_ERROR() << "Failed saving MP3D semantic mesh PLY" << plyFile;
    return 1;
  }

  return 0;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cout << "Usage: Datatool task input_file output_file" << std::endl;
    return 64;
  }
  const std::string task = argv[1];
  if (task == "create_navmesh") {
    createNavMesh(argv[2], argv[3]);
  } else if (task == "create_mp3d_semantic_mesh") {
    if (argc < 5) {
      std::cout << "Usage: Datatool create_mp3d_semantic_mesh input_ply "
                   "input_house output_mesh"
                << std::endl;
      return 64;
    }
    createMp3dSemanticMesh(argv[2], argv[3], argv[4]);
  } else if (task == "create_gibson_semantic_mesh") {
    if (argc < 5) {
      std::cout << "Usage: Datatool create_gibson_semantic_mesh input_obj "
                   "input_ids output_mesh"
                << std::endl;
      return 64;
    }
    createGibsonSemanticMesh(argv[2], argv[3], argv[4]);
  } else {
    ESP_ERROR() << "Unrecognized task" << task;
    return 1;
  }

  ESP_DEBUG() << "task: \"" << task << "\" done";
  return 0;
}
