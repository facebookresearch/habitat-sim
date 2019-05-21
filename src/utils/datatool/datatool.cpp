// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <iostream>
#include <string>
#include <unordered_map>

#include "SceneLoader.h"

#include "esp/assets/Mp3dInstanceMeshData.h"
#include "esp/core/esp.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/SemanticScene.h"

using namespace esp::assets;
using namespace esp::scene;
using namespace esp::nav;

int createNavMesh(const std::string& meshFile, const std::string& navmeshFile) {
  SceneLoader loader;
  const AssetInfo info = AssetInfo::fromPath(meshFile);
  const MeshData mesh = loader.load(info);
  NavMeshSettings bs;
  bs.setDefaults();
  PathFinder pf;
  if (!pf.build(bs, mesh)) {
    LOG(ERROR) << "Failed to build navmesh";
    return 2;
  }
  if (!pf.saveNavMesh(navmeshFile)) {
    LOG(ERROR) << "Failed to save navmesh";
    return 3;
  }
  return 0;
}

int createMp3dSemanticMesh(const std::string& plyFile,
                           const std::string& houseFile,
                           const std::string& semMeshFile) {
  SemanticScene semanticScene;
  bool success = SemanticScene::loadMp3dHouse(houseFile, semanticScene);
  if (!success) {
    LOG(ERROR) << "Failed loading MP3D house file " << houseFile;
    return 1;
  }
  const std::unordered_map<int, int>& objectIdMap =
      semanticScene.getSemanticIndexMap();

  Mp3dInstanceMeshData mp3dMesh;
  success = mp3dMesh.loadMp3dPLY(plyFile);
  if (!success) {
    LOG(ERROR) << "Failed parsing MP3D segmenation PLY " << plyFile;
    return 1;
  }

  success =
      mp3dMesh.saveSemMeshPLY(semMeshFile, semanticScene.getSemanticIndexMap());
  if (!success) {
    LOG(ERROR) << "Failed saving MP3D semantic mesh PLY " << plyFile;
    return 1;
  }

  return 0;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cout << "Usage: datatool task input_file output_file" << std::endl;
    return 64;
  }
  const std::string task = argv[1];
  if (task == "create_navmesh") {
    createNavMesh(argv[2], argv[3]);
  } else if (task == "create_mp3d_semantic_mesh") {
    if (argc < 5) {
      std::cout << "Usage: datatool create_mp3d_semantic_mesh input_ply "
                   "input_house output_mesh"
                << std::endl;
      return 64;
    }
    createMp3dSemanticMesh(argv[2], argv[3], argv[4]);
  } else {
    LOG(ERROR) << "Unrecognized task " << task;
    return 1;
  }

  LOG(INFO) << "task: \"" << task << "\" done";
  return 0;
}
