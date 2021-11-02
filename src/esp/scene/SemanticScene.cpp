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

#include "esp/io/Io.h"
#include "esp/io/Json.h"

namespace esp {
namespace scene {

bool SemanticScene::
    loadSemanticSceneDescriptor(const std::string& ssdFileName, SemanticScene& scene, const quatf& rotation /* = quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY) */) {
  bool success = false;
  bool exists = checkFileExists(ssdFileName, "loadSemanticSceneDescriptor");
  if (exists) {
    // TODO: we need to investigate the possibility of adding an identifying tag
    // to the SSD config files.
    // Try file load mechanisms for various types
    // first try Mp3d
    try {
      // only returns false if file does not exist, or attempting to open it
      // fails
      // open stream and determine house format version
      try {
        std::ifstream ifs = std::ifstream(ssdFileName);
        std::string header;
        std::getline(ifs, header);
        if (header == "ASCII 1.1") {
          success = buildMp3dHouse(ifs, scene, rotation);
        } else if (header == "HM3D Semantic Annotations") {
          success = buildHM3DHouse(ifs, scene, rotation);
        }
      } catch (...) {
        success = false;
      }
      if (!success) {
        // if not successful then attempt to load known json files
        const io::JsonDocument& jsonDoc = io::parseJsonFile(ssdFileName);
        // if no error thrown, then we have loaded a json file of given name
        bool hasCorrectObjects =
            (jsonDoc.HasMember("objects") && jsonDoc["objects"].IsArray());
        // check if also has "classes" tag, otherwise will assume it is a
        // gibson file
        if (jsonDoc.HasMember("classes") && jsonDoc["classes"].IsArray()) {
          // attempt to load replica or replicaCAD if has classes (replicaCAD
          // does not have objects in SSDescriptor)
          success =
              buildReplicaHouse(jsonDoc, scene, hasCorrectObjects, rotation);
        } else if (hasCorrectObjects) {
          // attempt to load gibson if has objects but not classes
          success = buildGibsonHouse(jsonDoc, scene, rotation);
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
  // ssdFileName
  const std::string tmpFName =
      FileUtil::join(FileUtil::path(ssdFileName), "info_semantic.json");
  if (FileUtil::exists(tmpFName)) {
    success = scene::SemanticScene::loadReplicaHouse(tmpFName, scene);
  }
  return success;

}  // SemanticScene::loadSemanticSceneDescriptor

}  // namespace scene
}  // namespace esp
