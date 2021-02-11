// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticScene.h"
#include "GibsonSemanticScene.h"
#include "Mp3dSemanticScene.h"
#include "ReplicaSemanticScene.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <sophus/se3.hpp>
#include <sstream>
#include <string>

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
namespace scene {

bool SemanticScene::
    loadSemanticSceneDescriptor(const std::string& houseFilename, SemanticScene& scene, const quatf& rotation /* = quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY) */) {
  bool success = false;
  bool exists = checkFileExists(houseFilename, "loadSemanticSceneDescriptor");
  if (exists) {
    // TODO: we need to investigate the possibility of adding an identifying tag
    // to the SSD config files.
    // Try file load mechanisms for various types
    // first try Mp3d
    try {
      // only returns false if file does not exist, or attempting to open it
      // fails
      success = loadMp3dHouse(houseFilename, scene, rotation);
      if (!success) {
        // if not successful then attempt to load known json files
        const auto& jsonDoc = io::parseJsonFile(houseFilename);
        // if no error thrown, then we loaded a json file of given name
        if (jsonDoc.HasMember("objects") && jsonDoc["objects"].IsArray()) {
          // check if also has "classes" tag, otherwise will assume it is a
          // gibson file
          if (jsonDoc.HasMember("classes") && jsonDoc["classes"].IsArray()) {
            // load replica
            success = loadReplicaHouse(houseFilename, scene, rotation);
          } else {
            // load gibson
            success = loadGibsonHouse(houseFilename, scene, rotation);
          }
        }
      }
      if (success) {
        // if successfully loaded, return true;
        return true;
      }
    } catch (...) {
      // if error thrown, assume it is because file load attempt fails
      success = false;
    }
  }
  // should only reach here if not successfully loaded
  namespace FileUtil = Cr::Utility::Directory;
  // check if constructed replica file exists in directory of passed
  // houseFilename
  const std::string tmpFName =
      FileUtil::join(FileUtil::path(houseFilename), "info_semantic.json");
  if (FileUtil::exists(tmpFName)) {
    success = scene::SemanticScene::loadReplicaHouse(tmpFName, scene);
  }
  return success;

}  // SemanticScene::loadSemanticSceneDescriptor

}  // namespace scene
}  // namespace esp
