// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticScene.h"
#include "GibsonSemanticScene.h"
#include "Mp3dSemanticScene.h"
#include "ReplicaSemanticScene.h"

#include <Corrade/Utility/FormatStl.h>

#include <fstream>
#include <map>
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
        if (header.find("ASCII 1.1") != std::string::npos) {
          success = buildMp3dHouse(ifs, scene, rotation);
        } else if (header.find("HM3D Semantic Annotations") !=
                   std::string::npos) {
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

namespace {
/**
 * @brief Build an AABB for a given set of vertex indices in @p verts list, and
 * return in a std::pair, along with the count of verts used to build the AABB.
 * @param colorInt Semantic Color of object
 * @param verts The mesh's vertex buffer.
 * @param setOfIDXs set of vertex IDXs in the vertex buffer being used to build
 * the resultant AABB.
 */
CCSemanticObject::ptr buildCCSemanticObjForSetOfVerts(
    uint32_t colorInt,
    const std::vector<Mn::Vector3>& verts,
    const std::set<uint32_t>& setOfIDXs) {
  Mn::Vector3 vertMax{-Mn::Constants::inf(), -Mn::Constants::inf(),
                      -Mn::Constants::inf()};
  Mn::Vector3 vertMin{Mn::Constants::inf(), Mn::Constants::inf(),
                      Mn::Constants::inf()};

  for (uint32_t idx : setOfIDXs) {
    Mn::Vector3 vert = verts[idx];
    vertMax = Mn::Math::max(vertMax, vert);
    vertMin = Mn::Math::min(vertMin, vert);
  }

  Mn::Vector3 center = .5f * (vertMax + vertMin);
  Mn::Vector3 dims = vertMax - vertMin;

  auto obj = std::make_shared<CCSemanticObject>(
      CCSemanticObject(setOfIDXs.size(), colorInt));
  // set obj's bounding box
  obj->setObb(Mn::EigenIntegration::cast<esp::vec3f>(center),
              Mn::EigenIntegration::cast<esp::vec3f>(dims), quatf::Identity());
  return obj;
}  // buildCCSemanticObjForSetOfVerts

}  // namespace

std::unordered_map<uint32_t, std::vector<CCSemanticObject::ptr>>
SemanticScene::buildCCBasedSemanticObjs(
    const std::vector<Mn::Vector3>& verts,
    const std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>&
        clrsToComponents,
    const std::shared_ptr<SemanticScene>& semanticScene) {
  // build color-keyed map of lists of pairs of vert-count/bboxes
  std::unordered_map<uint32_t, std::vector<CCSemanticObject::ptr>>
      semanticCCObjsByVertTag(clrsToComponents.size());
  for (const auto& elem : clrsToComponents) {
    const uint32_t colorKey = elem.first;
    const std::vector<std::set<uint32_t>>& vectorOfSets = elem.second;
    auto findIter = semanticCCObjsByVertTag.find(colorKey);
    if (findIter == semanticCCObjsByVertTag.end()) {
      // not found already - initialize new entry
      findIter = semanticCCObjsByVertTag
                     .emplace(colorKey, std::vector<CCSemanticObject::ptr>{})
                     .first;
    }
    for (const std::set<uint32_t>& vertSet : vectorOfSets) {
      findIter->second.emplace_back(
          buildCCSemanticObjForSetOfVerts(colorKey, verts, vertSet));
    }
  }

  // only map to semantic ID if semanticScene exists, otherwise return map with
  // objects keyed by hex color
  if (!semanticScene) {
    return semanticCCObjsByVertTag;
  }
  // if semanticScene is provided, assume vert color was tag used to determine
  // CC membership
  const auto& semanticObjs = semanticScene->objects();
  // build temp map of colors to SemanticObject IDs
  std::unordered_map<uint32_t, uint32_t> mapColorIntsToSemanticObjIDXs;
  mapColorIntsToSemanticObjIDXs.reserve(semanticObjs.size());
  for (uint32_t i = 0; i < semanticObjs.size(); ++i) {
    mapColorIntsToSemanticObjIDXs.insert({semanticObjs[i]->getColorAsInt(), i});
  }
  // build map with key being semanticObject IDX in semantic Objects array
  std::unordered_map<uint32_t, std::vector<scene::CCSemanticObject::ptr>>
      results;
  for (auto& elem : mapColorIntsToSemanticObjIDXs) {
    const auto objID = elem.second;
    const auto colorAsInt = elem.first;
    auto mapEntry = semanticCCObjsByVertTag.find(colorAsInt);
    if (mapEntry != semanticCCObjsByVertTag.end()) {
      results.emplace(objID, std::move(mapEntry->second));
    }
  }
  return results;
}  // SemanticScene::buildCCBasedBBoxes

void SemanticScene::buildSemanticOBBsFromCCs(
    const std::vector<Mn::Vector3>& verts,
    const std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>&
        clrsToComponents,
    const std::shared_ptr<SemanticScene>& semanticScene,
    const std::string& msgPrefix) {
  // get map of semantic ID to vector of CCSemanticObjs.
  const auto perIDMapOfCCSemanticObjs =
      buildCCBasedSemanticObjs(verts, clrsToComponents, semanticScene);

  // get all semantic objects
  const auto& ssdObjs = semanticScene->objects();
  std::multimap<float, std::shared_ptr<CCSemanticObject>> perSemanticObjCCs;
  for (const auto& ccObj : perIDMapOfCCSemanticObjs) {
    // get vector of CCSemanticObjs
    const auto& vecOfCCSemanticObjs = ccObj.second;
    // if only one element, skip
    if (vecOfCCSemanticObjs.size() == 1) {
      continue;
    }
    uint32_t objIdx = ccObj.first;
    // do not process Unknown
    if (objIdx == 0) {
      continue;
    }
    // get object with given semantic ID
    auto& ssdObj = *ssdObjs[objIdx];
    // build temp multimap keyed by volume
    perSemanticObjCCs.clear();
    for (const auto& CCObj : vecOfCCSemanticObjs) {
      perSemanticObjCCs.emplace(CCObj->obb().volume(), CCObj);
    }
    // after all are emplaced, reverse order will be in order of decreasing
    // volume
    ESP_DEBUG() << Cr::Utility::formatString(
        "\nmap size {} : vec size {} Displaying elements in decreasing order "
        "for objIDX : {} | name : {} | vol : {}",
        perSemanticObjCCs.size(), vecOfCCSemanticObjs.size(), objIdx,
        ssdObj.id(), ssdObj.obb().volume());
    for (auto elem = perSemanticObjCCs.crbegin();
         elem != perSemanticObjCCs.crend(); ++elem) {
      ESP_DEBUG() << Cr::Utility::formatString(
          "\tvol:{} : #pts {}", elem->first, elem->second->getNumSrcVerts());
    }
    // Set Largest volume element as new OBB
    ssdObj.setObb(perSemanticObjCCs.crbegin()->second->obb());
  }

}  // SemanticScene::buildSemanticOBBsFromCCs

void SemanticScene::buildSemanticOBBs(
    const std::vector<Mn::Vector3>& vertices,
    const std::vector<uint16_t>& vertSemanticIDs,
    const std::vector<std::shared_ptr<esp::scene::SemanticObject>>& ssdObjs,
    const std::string& msgPrefix) {
  // build per-SSD object vector of known semantic IDs
  // doing this in case semanticIDs are not contiguous.
  std::size_t numSSDObjs = ssdObjs.size();
  std::vector<int> semanticIDToSSOBJidx(numSSDObjs, -1);
  for (int i = 0; i < numSSDObjs; ++i) {
    const auto& ssdObj = *ssdObjs[i];
    int semanticID = ssdObj.semanticID();
    // should not happen unless semantic ids are not sequential
    if (semanticIDToSSOBJidx.size() <= semanticID) {
      semanticIDToSSOBJidx.resize(semanticID + 1);
    }
    semanticIDToSSOBJidx[semanticID] = i;
  }

  // aggegates of per-semantic ID mins and maxes
  std::vector<Mn::Vector3> vertMax(
      semanticIDToSSOBJidx.size(),
      {-Mn::Constants::inf(), -Mn::Constants::inf(), -Mn::Constants::inf()});
  std::vector<Mn::Vector3> vertMin(
      semanticIDToSSOBJidx.size(),
      {Mn::Constants::inf(), Mn::Constants::inf(), Mn::Constants::inf()});
  std::vector<int> vertCounts(semanticIDToSSOBJidx.size());

  // for each vertex, map vert min and max for each known semantic ID
  // Known semantic IDs are expected to be contiguous, and correspond to the
  // number of unique ssdObjs mappings.

  for (int vertIdx = 0; vertIdx < vertSemanticIDs.size(); ++vertIdx) {
    // semantic ID on vertex - valid values are 1->semanticIDToSSOBJidx.size().
    // Invalid/unknown semantic ids are > semanticIDToSSOBJidx.size()
    const auto semanticID = vertSemanticIDs[vertIdx];
    if ((semanticID >= 0) && (semanticID < semanticIDToSSOBJidx.size())) {
      const auto vert = vertices[vertIdx];
      // FOR VERT-BASED OBB CALC
      // only support bbs for known colors that map to semantic objects
      vertMax[semanticID] = Mn::Math::max(vertMax[semanticID], vert);
      vertMin[semanticID] = Mn::Math::min(vertMin[semanticID], vert);
      vertCounts[semanticID] += 1;
    }
  }

  // with mins/maxs per ID, map to objs
  // give each ssdObj the values to build its OBB
  for (int semanticID = 0; semanticID < semanticIDToSSOBJidx.size();
       ++semanticID) {
    int objIdx = semanticIDToSSOBJidx[semanticID];
    if (objIdx == -1) {
      continue;
    }
    // get object with given semantic ID
    auto& ssdObj = *ssdObjs[objIdx];
    Mn::Vector3 center{};
    Mn::Vector3 dims{};

    if (vertCounts[semanticID] == 0) {
      ESP_DEBUG() << Cr::Utility::formatString(
          "{} Semantic ID : {} : color : {} tag : {} present in {} "
          "verts | No verts have specified Semantic ID.",
          msgPrefix, semanticID, geo::getColorAsString(ssdObj.getColor()),
          ssdObj.id(), vertCounts[semanticID]);
    } else {
      center = .5f * (vertMax[semanticID] + vertMin[semanticID]);
      dims = vertMax[semanticID] - vertMin[semanticID];
      ESP_DEBUG() << Cr::Utility::formatString(
          "{} Semantic ID : {} : color : {} tag : {} present in {} verts | BB "
          "Center [{} {} {}] Dims [{} {} {}]",
          msgPrefix, semanticID, geo::getColorAsString(ssdObj.getColor()),
          ssdObj.id(), vertCounts[semanticID], center.x(), center.y(),
          center.z(), dims.x(), dims.y(), dims.z());
    }
    ssdObj.setObb(Mn::EigenIntegration::cast<esp::vec3f>(center),
                  Mn::EigenIntegration::cast<esp::vec3f>(dims));
  }
}  // SemanticScene::buildSemanticOBBs

}  // namespace scene
}  // namespace esp
