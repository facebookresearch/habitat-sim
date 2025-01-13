// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticScene.h"
#include "GibsonSemanticScene.h"
#include "Mp3dSemanticScene.h"
#include "ReplicaSemanticScene.h"

#include <Corrade/Containers/Pair.h>
#include <Corrade/Utility/FormatStl.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "esp/io/Io.h"
#include "esp/io/Json.h"
#include "esp/metadata/attributes/SemanticAttributes.h"

namespace esp {
namespace scene {

bool SemanticScene::loadSemanticSceneDescriptor(
    const std::shared_ptr<metadata::attributes::SemanticAttributes>&
        semanticAttr,
    SemanticScene& scene,
    const Mn::Quaternion& rotation) {
  const std::string ssdFileName =
      semanticAttr != nullptr ? semanticAttr->getSemanticDescriptorFullPath()
                              : "";

  bool loadSuccess = false;
  if (ssdFileName != "") {
    bool fileExists =
        checkFileExists(ssdFileName, "loadSemanticSceneDescriptor");
    if (fileExists) {
      // TODO: we need to investigate the possibility of adding an identifying
      // tag to the SSD config files. Try file load mechanisms for various types
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
            loadSuccess = buildMp3dHouse(ifs, scene, rotation);
          } else if (header.find("HM3D Semantic Annotations") !=
                     std::string::npos) {
            loadSuccess = buildHM3DHouse(ifs, scene, rotation);
          }
        } catch (...) {
          loadSuccess = false;
        }
        if (!loadSuccess) {
          // if not successful then attempt to load known json files
          const io::JsonDocument& jsonDoc = io::parseJsonFile(ssdFileName);
          // if no error thrown, then we have loaded a json file of given name

          io::JsonGenericValue::ConstMemberIterator hasJsonObjIter =
              jsonDoc.FindMember("objects");

          bool hasCorrectObjects = (hasJsonObjIter != jsonDoc.MemberEnd() &&
                                    hasJsonObjIter->value.IsArray());

          io::JsonGenericValue::ConstMemberIterator hasJsonClassIter =
              jsonDoc.FindMember("classes");

          // check if also has "classes" tag, otherwise will assume it is a
          // gibson file
          if (hasJsonClassIter != jsonDoc.MemberEnd() &&
              hasJsonClassIter->value.IsArray()) {
            // attempt to load replica or replicaCAD if has classes (replicaCAD
            // does not have objects in SSDescriptor)
            loadSuccess =
                buildReplicaHouse(jsonDoc, scene, hasCorrectObjects, rotation);
          } else if (hasCorrectObjects) {
            // attempt to load gibson if has objects but not classes
            loadSuccess = buildGibsonHouse(jsonDoc, scene, rotation);
          }
        }
      } catch (...) {
        // if error thrown, assume it is because file load attempt fails
        loadSuccess = false;
      }
    }
    if (!loadSuccess) {
      // should only reach here if either specified file exists but was not
      // loaded successfully or file does not exist.

      // attempt to look for specified file failed, so attempt to build new file
      // name by searching in path specified of specified file for
      // info_semantic.json file for replica dataset
      namespace FileUtil = Cr::Utility::Path;
      // check if constructed replica file exists in directory of passed
      // ssdFileName
      const std::string constructedFilename = FileUtil::join(
          FileUtil::split(ssdFileName).first(), "info_semantic.json");
      if (FileUtil::exists(constructedFilename)) {
        loadSuccess =
            scene::SemanticScene::loadReplicaHouse(constructedFilename, scene);

        if (loadSuccess) {
          ESP_DEBUG(Mn::Debug::Flag::NoSpace)
              << "SSD for Replica using constructed file : `"
              << constructedFilename << "` in directory with `" << ssdFileName
              << "` loaded successfully";
        } else {
          // here if constructed file exists but does not correspond to
          // appropriate SSD or some loading error occurred.
          ESP_ERROR(Mn::Debug::Flag::NoSpace)
              << "SSD Load Failure! Replica file with constructed name `"
              << ssdFileName << "` exists but failed to load.";
        }
      } else {
        // neither provided non-empty filename nor constructed filename
        // exists. This is probably due to an incorrect naming in the
        // SemanticAttributes
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "SSD File Naming Issue! Neither "
               "SemanticAttributes-provided name : `"
            << ssdFileName << "` nor constructed filename : `"
            << constructedFilename << "` exist on disk.";
        loadSuccess = false;
      }
    }

    if (loadSuccess) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "SSD with SemanticAttributes-provided name `" << ssdFileName
          << "` successfully found and loaded.";
    } else {
      // here if provided file exists but does not correspond to appropriate
      // SSD
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "SSD Load Failure! File with "
             "SemanticAttributes-provided name `"
          << ssdFileName << "` exists but failed to load.";
    }
  }

  if (semanticAttr != nullptr) {
    if (semanticAttr->getNumRegionInstances() > 0) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Semantic Attributes : `" << semanticAttr->getHandle() << "` has "
          << semanticAttr->getNumRegionInstances() << " regions defined.";
      // Build Semantic regions for each SemanticRegion attributes instance
      const auto regionInstances = semanticAttr->getRegionInstances();

      scene.regions_.clear();
      scene.regions_.reserve(regionInstances.size());
      int idx = 0;
      for (const auto& regionInstance : regionInstances) {
        auto regionPtr = SemanticRegion::create();
        // Unique name
        regionPtr->name_ = regionInstance->getHandle();
        // Build a category
        regionPtr->category_ =
            LoopRegionCategory::create(idx++, regionInstance->getLabel());
        // Set y heights
        regionPtr->extrusionHeight_ = regionInstance->getExtrusionHeight();
        regionPtr->floorHeight_ = regionInstance->getFloorHeight();
        // Set bbox
        const Mn::Vector3 min = regionInstance->getMinBounds();
        const Mn::Vector3 max = regionInstance->getMaxBounds();
        regionPtr->setBBox(min, max);
        // Set polyloop points and precalc polyloop edge vectors
        const auto loopPoints = regionInstance->getPolyLoop();

        std::size_t numPts = loopPoints.size();
        regionPtr->polyLoopPoints_ = std::vector<Mn::Vector2>(numPts);
        regionPtr->visEdges_ =
            std::vector<std::vector<Mn::Vector3>>(4 * numPts);

        // Save points and edges
        int eIdx = 0;
        float yExtrusion = static_cast<float>(regionPtr->extrusionHeight_ +
                                              regionPtr->floorHeight_);

        float polyArea = 0.0f;
        for (std::size_t i = 0; i < numPts; ++i) {
          Mn::Vector3 currPoint = loopPoints[i];
          regionPtr->polyLoopPoints_[i] = {currPoint.x(), currPoint.z()};
          // Edges
          Mn::Vector3 currExtPt = {currPoint.x(), yExtrusion, currPoint.z()};

          std::size_t nextIdx = ((i + 1) % numPts);
          Mn::Vector3 nextPoint = loopPoints[nextIdx];
          Mn::Vector3 nextExtPt = {nextPoint.x(), yExtrusion, nextPoint.z()};
          // Find region projection area based on Green's Theorem (.5 * abs(sum
          // (xprod of sequential edges)))
          polyArea +=
              currPoint.x() * nextPoint.z() - nextPoint.x() * currPoint.z();
          // Horizontal edge
          regionPtr->visEdges_[eIdx++] = {currPoint, nextPoint};
          // Vertical edge
          regionPtr->visEdges_[eIdx++] = {currPoint, currExtPt};
          // Extruded horizontal edge
          regionPtr->visEdges_[eIdx++] = {currExtPt, nextExtPt};
          // Diagonal edge
          regionPtr->visEdges_[eIdx++] = {currPoint, nextExtPt};
        }
        // Set the polyloop area
        regionPtr->area_ = .5 * abs(polyArea);
        scene.regions_.emplace_back(std::move(regionPtr));
      }
    } else {  // if semantic attributes specifes region annotations
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Semantic Attributes : `" << semanticAttr->getHandle()
          << "` does not have any regions defined.";
    }
  } else {
    ESP_DEBUG(Mn::Debug::Flag::NoSpace) << "Semantic attributes do not exist.";
  }  // if semanticAttrs exist or not

  return loadSuccess;

}  // SemanticScene::loadSemanticSceneDescriptor

bool SemanticRegion::contains(const Mn::Vector3& pt) const {
  auto checkPt = [&](float x, float x0, float x1, float y, float y0,
                     float y1) -> bool {
    float interp = ((y - y0) / (y1 - y0));
    return ((y < y0) != (y < y1)) && (x < x0 + interp * (x1 - x0));
  };

  // First check height
  if ((pt.y() < floorHeight_) || (pt.y() > (floorHeight_ + extrusionHeight_))) {
    return false;
  }
  // Next check bbox
  if (!bbox_.contains(pt)) {
    return false;
  }
  // Lastly, count casts across edges.
  int count = 0;
  int numPts = polyLoopPoints_.size();
  for (int i = 0; i < numPts; ++i) {
    const auto stPt = polyLoopPoints_[i];
    const auto endPt = polyLoopPoints_[(i + 1) % numPts];
    if (stPt == endPt) {
      // Skip points that are equal.
      continue;
    }
    // use x-z in point (i.e. projecting to ground plane)
    bool checkCrossing =
        checkPt(pt.x(), stPt.x(), endPt.x(), pt.z(), stPt.y(), endPt.y());
    if (checkCrossing) {
      ++count;
    }
  }

  // Want odd crossings for being inside
  return (count % 2 == 1);
}  // SemanticRegion::contains

void SemanticRegion::setBBox(const Mn::Vector3& min, const Mn::Vector3& max) {
  bbox_ = {min, max};
}  // SemanticRegion::setBBox

namespace {
/**
 * @brief Build an AABB for a given set of vertex indices in @p verts list,
 * and return in a std::pair, along with the count of verts used to build the
 * AABB.
 * @param colorInt Semantic Color of object
 * @param verts The mesh's vertex buffer.
 * @param setOfIDXs set of vertex IDXs in the vertex buffer being used to
 * build the resultant AABB.
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

  auto obj =
      std::make_shared<CCSemanticObject>(CCSemanticObject(colorInt, setOfIDXs));
  // set obj's bounding box
  obj->setObb(center, dims, Mn::Quaternion(Mn::Math::IdentityInit));
  return obj;
}  // buildCCSemanticObjForSetOfVerts

/**
 * @brief build per-SSD object vector of known semantic IDs - doing this in
 * case semanticIDs are not contiguous.
 */

std::vector<int> getObjsIdxToIDMap(
    const std::vector<std::shared_ptr<esp::scene::SemanticObject>>& ssdObjs) {
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
  return semanticIDToSSOBJidx;
}

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

  // only map to semantic ID if semanticScene exists, otherwise return map
  // with objects keyed by hex color
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
    mapColorIntsToSemanticObjIDXs.emplace(semanticObjs[i]->getColorAsInt(), i);
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

std::vector<uint32_t> SemanticScene::buildSemanticOBBsFromCCs(
    const std::vector<Mn::Vector3>& verts,
    const std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>&
        clrsToComponents,
    const std::shared_ptr<SemanticScene>& semanticScene,
    float maxVolFraction,
    const std::string& msgPrefix) {
  // Semantic scene is required to map color annotations to semantic objects
  if (!semanticScene) {
    ESP_WARNING() << "Attempting to build CC-based semantic bboxes but no "
                     "Semantic Scene Descriptor is provided, so skipping. NO "
                     "SEMANTIC BBOXES WILL BE CREATED.";
    return {};
  }

  // get map of semantic ID to vector of CCSemanticObjs.
  const auto perIDMapOfCCSemanticObjs =
      buildCCBasedSemanticObjs(verts, clrsToComponents, semanticScene);

  // get all semantic objects
  const auto& ssdObjs = semanticScene->objects();

  // build per-SSD object vector of known semantic IDs
  // doing this in case semanticIDs are not contiguous.
  std::vector<int> semanticIDToSSOBJidx = getObjsIdxToIDMap(ssdObjs);

  std::vector<uint32_t> unMappedObjectIDXs;

  std::multimap<float, std::shared_ptr<CCSemanticObject>> perSemanticObjCCs;
  std::multimap<float, uint32_t> semanticObjVolToSetIDX;
  for (int semanticID = 0; semanticID < semanticIDToSSOBJidx.size();
       ++semanticID) {
    // no index corresponds with given semantic ID
    if (semanticIDToSSOBJidx[semanticID] == -1) {
      continue;
    }

    uint32_t objIdx = semanticIDToSSOBJidx[semanticID];
    // get object with given semantic ID
    auto& ssdObj = *ssdObjs[objIdx];

    // get vector of CCSemanticObjs for given ID
    const std::unordered_map<uint32_t,
                             std::vector<CCSemanticObject::ptr>>::const_iterator
        semanticCCsPerID = perIDMapOfCCSemanticObjs.find(objIdx);

    if ((semanticCCsPerID == perIDMapOfCCSemanticObjs.end()) ||
        (semanticCCsPerID->second.empty())) {
      // keep a record of semantic IDs without any corresponding verts
      unMappedObjectIDXs.emplace_back(objIdx);
      // ESP_DEBUG() << "\n\t\t!!!!!!" << msgPrefix
      //             << "Note : Semantic Scene Annotation ID :" << objIdx
      //             << "is not present in the mesh - there are no vertices "
      //             << "with this annotation's color assigned to them; "
      //             << "therefore, this semantic object will have no BBox.";
      continue;
    }

    std::vector<CCSemanticObject::ptr> vecOfCCSemanticObjs =
        semanticCCsPerID->second;
    ESP_VERY_VERBOSE() << Cr::Utility::formatString(
        "{}Semantic CC vec size {} Displaying elements in decreasing order "
        "for objIDX : {} | name : {} | current (pre-CC calc) vol : {} | "
        "Fraction of max vol to use "
        "{}",
        msgPrefix, vecOfCCSemanticObjs.size(), objIdx, ssdObj.id(),
        ssdObj.obb().volume(), maxVolFraction);
    if (vecOfCCSemanticObjs.size() == 1) {
      // if only one element, always use specified CC's bbox, built off all
      // verts with semantic annotation
      ssdObj.setObb(vecOfCCSemanticObjs[0]->obb());
    } else {
      // Multiple elements, use some fraction of CCs based on volume
      // build temp multimap keyed by volume
      perSemanticObjCCs.clear();
      semanticObjVolToSetIDX.clear();
      for (uint32_t idx = 0; idx < vecOfCCSemanticObjs.size(); ++idx) {
        const auto& CCObj = vecOfCCSemanticObjs[idx];
        const float vol = CCObj->obb().volume();
        perSemanticObjCCs.emplace(vol, CCObj);
        semanticObjVolToSetIDX.emplace(vol, idx);
      }
      auto largestElement = perSemanticObjCCs.crbegin();
      // the OBB/AABB to use for the specified semantic object
      auto obbToUse = largestElement->second->obb();
      if (maxVolFraction == 1.0f) {
        // Only use largest CC for semantic bbox
        // after all are emplaced, reverse order will be in order of
        // decreasing volume
        for (auto elem = largestElement; elem != perSemanticObjCCs.crend();
             ++elem) {
          ESP_VERY_VERBOSE()
              << Cr::Utility::formatString("\tvol:{} : #pts {}", elem->first,
                                           elem->second->getNumSrcVerts());
        }

      } else {
        // TODO determine which collection of CC bboxes to merge to build the
        // final bbox.
        // use some subset of the sets of verts to determine OBB/AABB
        float maxVol = largestElement->first;
        int numElems = perSemanticObjCCs.size();
        int currElemNum = 1;
        // after all are emplaced, reverse order will be in order of
        // decreasing volume

        std::set<uint32_t> setOfIDXs = largestElement->second->getVertSet();
        const uint32_t clrOfVerts = largestElement->second->getColorAsInt();

        for (auto elem = std::next(largestElement);
             elem != perSemanticObjCCs.crend(); ++elem) {
          float fraction = elem->first / maxVol;
          if (fraction < maxVolFraction) {
            ESP_VERY_VERBOSE() << Cr::Utility::formatString(
                "\tCC Built using {} of {} elements. Remaining CCs' volumes "
                "too small, so skipping.",
                currElemNum, numElems);
            break;
          }
          ESP_VERY_VERBOSE() << Cr::Utility::formatString(
              "\tCC Vol:{} : #pts {} : fraction of max vol : {}", elem->first,
              elem->second->getNumSrcVerts(), fraction);
          setOfIDXs.insert(elem->second->getVertSet().begin(),
                           elem->second->getVertSet().end());
          ++currElemNum;
        }

        // build CCSemanticObj, which will build
        auto ccObbPtr =
            buildCCSemanticObjForSetOfVerts(clrOfVerts, verts, setOfIDXs);
        // get merged sets' obb
        obbToUse = ccObbPtr->obb();
      }
      // Set Largest volume element, or aggregate element, as new OBB
      ssdObj.setObb(obbToUse);

      ESP_VERY_VERBOSE() << Cr::Utility::formatString(
          "After setting from largest cc, obj {} volume :{}", ssdObj.id(),
          ssdObj.obb().volume());
    }
  }  // for each semantic ID currently mapped to a vector of CCs
  // return listing of semantic object idxs that have no presence in the mesh
  return unMappedObjectIDXs;
}  // SemanticScene::buildSemanticOBBsFromCCs

std::vector<uint32_t> SemanticScene::buildSemanticOBBs(
    const std::vector<Mn::Vector3>& vertices,
    const std::vector<uint16_t>& vertSemanticIDs,
    const std::vector<std::shared_ptr<esp::scene::SemanticObject>>& ssdObjs,
    const std::string& msgPrefix) {
  // build per-SSD object vector of known semantic IDs
  // doing this in case semanticIDs are not contiguous.
  std::vector<int> semanticIDToSSOBJidx = getObjsIdxToIDMap(ssdObjs);

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
    // semantic ID on vertex - valid values are
    // 1->semanticIDToSSOBJidx.size(). Invalid/unknown semantic ids are >
    // semanticIDToSSOBJidx.size()
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

  std::vector<uint32_t> unMappedObjectIDXs;

  // with mins/maxs per ID, map to objs
  // give each ssdObj the values to build its OBB
  for (int semanticID = 0; semanticID < semanticIDToSSOBJidx.size();
       ++semanticID) {
    // no index corresponds with given semantic ID
    if (semanticIDToSSOBJidx[semanticID] == -1) {
      continue;
    }

    uint32_t objIdx = semanticIDToSSOBJidx[semanticID];
    // get object with given semantic ID
    auto& ssdObj = *ssdObjs[objIdx];
    Mn::Vector3 center{};
    Mn::Vector3 dims{};

    if (vertCounts[semanticID] == 0) {
      // keep a record of semantic IDs without any corresponding verts
      unMappedObjectIDXs.emplace_back(objIdx);
      // ESP_DEBUG() << Cr::Utility::formatString(
      //     "{} Semantic ID : {} : color : {} tag : {} | No verts have color
      //     for " "specified Semantic ID.", msgPrefix, semanticID,
      //     geo::getColorAsString(ssdObj.getColor()), ssdObj.id());
    } else {
      center = .5f * (vertMax[semanticID] + vertMin[semanticID]);
      dims = vertMax[semanticID] - vertMin[semanticID];
      ESP_VERY_VERBOSE() << Cr::Utility::formatString(
          "{} Semantic ID : {} : color : {} tag : {} present in {} verts | "
          "BB "
          "Center [{} {} {}] Dims [{} {} {}]",
          msgPrefix, semanticID, geo::getColorAsString(ssdObj.getColor()),
          ssdObj.id(), vertCounts[semanticID], center.x(), center.y(),
          center.z(), dims.x(), dims.y(), dims.z());
    }
    ssdObj.setObb(center, dims);
  }
  // return listing of semantic object idxs that have no presence in the mesh
  return unMappedObjectIDXs;
}  // SemanticScene::buildSemanticOBBs

std::vector<int> SemanticScene::getRegionsForPoint(
    const Mn::Vector3& point) const {
  std::vector<int> containingRegions;
  for (int rix = 0; rix < regions_.size(); ++rix) {
    if (regions_[rix]->contains(point)) {
      containingRegions.push_back(rix);
    }
  }
  return containingRegions;
}  // SemanticScene::getRegionsForPoint

std::vector<std::pair<int, double>> SemanticScene::getWeightedRegionsForPoint(
    const Mn::Vector3& point) const {
  std::vector<int> containingRegions = getRegionsForPoint(point);
  if (containingRegions.size() == 0) {
    return {};
  }

  std::vector<std::pair<int, double>> containingRegionWeights;
  containingRegionWeights.reserve(containingRegions.size());
  // Only 1 containing region, so return region idx and weight of 1
  if (containingRegions.size() == 1) {
    containingRegionWeights.emplace_back(
        std::pair<int, double>(containingRegions[0], 1.0f));
    return containingRegionWeights;
  }

  // Sum up all areas containing point
  double ttlArea = 0.0f;
  for (int rix : containingRegions) {
    ttlArea += regions_[rix]->getArea();
  }

  for (int rix : containingRegions) {
    containingRegionWeights.emplace_back(std::pair<int, double>(
        rix, 1.0f - (regions_[rix]->getArea() / ttlArea)));
  }

  std::sort(containingRegionWeights.begin(), containingRegionWeights.end(),
            [](const std::pair<int, double>& a, std::pair<int, double>& b) {
              return a.second > b.second;
            });
  return containingRegionWeights;

}  // SemanticScene::getWeightedRegionsForPoint

std::vector<std::pair<int, double>> SemanticScene::getRegionsForPoints(
    const std::vector<Mn::Vector3>& points) const {
  // Weights for every point for every region
  std::vector<std::vector<double>> regAreaWeightsForPoints;
  regAreaWeightsForPoints.reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    // Get this point's weighted regions
    auto regWeightsForPoint = getWeightedRegionsForPoint(points[i]);
    // Initialize all region weights to be 0
    std::vector<double> allRegionWeights(regions_.size(), 0);
    // Set the weight for the containing region with the smallest area (if
    // nested regions) for this particular point.
    // Set the vote for each region to be equal.
    for (const std::pair<int, double>& regionWeight : regWeightsForPoint) {
      allRegionWeights[regionWeight.first] = 1.0;
    }
    // Save this points region weight vector
    regAreaWeightsForPoints.emplace_back(allRegionWeights);
  }

  std::vector<std::pair<int, double>> containingRegionWeights;
  // Will only have at max the number of regions in the scene
  containingRegionWeights.reserve(regions_.size());
  for (int rix = 0; rix < regions_.size(); ++rix) {
    double containmentWeight = 0;
    for (int i = 0; i < points.size(); ++i) {
      std::vector<double> regWtsForPoint = regAreaWeightsForPoints[i];
      containmentWeight += regWtsForPoint[rix];
    }
    if (containmentWeight > 0) {
      containingRegionWeights.emplace_back(
          std::pair<int, double>(rix, containmentWeight / points.size()));
    }
  }
  // Free up unused capacity - every region probably does not contain a tested
  // point
  containingRegionWeights.shrink_to_fit();
  std::sort(containingRegionWeights.begin(), containingRegionWeights.end(),
            [](const std::pair<int, double>& a, std::pair<int, double>& b) {
              return a.second > b.second;
            });
  return containingRegionWeights;
}  // SemanticScene::getRegionsForPoints
}  // namespace scene
}  // namespace esp
