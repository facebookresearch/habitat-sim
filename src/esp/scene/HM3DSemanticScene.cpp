// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "HM3DSemanticScene.h"
#include "SemanticScene.h"

#include <Corrade/Utility/FormatStl.h>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace Cr = Corrade;
namespace esp {
namespace scene {

bool SemanticScene::loadHM3DHouse(
    const std::string& houseFilename,
    SemanticScene& scene,
    const quatf& rotation /* = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                       geo::ESP_GRAVITY) */ ) {
  if (!checkFileExists(houseFilename, "loadHM3DHouse")) {
    return false;
  }
  // open stream and determine house format version
  std::ifstream ifs = std::ifstream(houseFilename);
  std::string header;
  std::getline(ifs, header);

  if (header.find("HM3D Semantic Annotations") == std::string::npos) {
    ESP_ERROR() << "Unsupported HM3D House format header" << header
                << "in file name" << houseFilename;
    return false;
  }

  return buildHM3DHouse(ifs, scene, rotation);
}  // SemanticScene::loadHM3DHouse

namespace {

// These structs are here to make constructing the actual Semantic objects
// easier, due to the format of the HM3D ssd files.

struct TempHM3DObject {
  int objInstanceID;
  int objCatID;
  std::string categoryName;
  std::string objInstanceName;
  unsigned colorInt;
  std::shared_ptr<SemanticRegion> region;
};

struct TempHM3DRegion {
  int regionID;
  std::vector<TempHM3DObject*> objInstances;
};
struct TempHM3DCategory {
  int ID;
  std::string name;
  std::vector<TempHM3DObject*> objInstances;

  std::shared_ptr<HM3DObjectCategory> category_;
};

void buildInstanceRegionCategory(
    int instanceID,
    const unsigned colorInt,
    const std::string& objCategoryName,
    int regionID,
    std::map<int, TempHM3DObject>& objInstance,
    std::map<int, TempHM3DRegion>& regions,
    std::unordered_map<std::string, TempHM3DCategory>& categories) {
  // build initial temp object
  TempHM3DObject obj{
      instanceID, 0, objCategoryName,
      Cr::Utility::formatString("{}_{}", objCategoryName, instanceID),
      colorInt};
  objInstance[instanceID] = std::move(obj);
  // find category, build if dne
  TempHM3DCategory tmpCat{
      static_cast<int>(categories.size()), objCategoryName, {}};
  auto categoryIter = categories.emplace(objCategoryName, std::move(tmpCat));
  categoryIter.first->second.objInstances.push_back(&objInstance[instanceID]);
  // find region, build if dne
  TempHM3DRegion tmpRegion{regionID, {}};
  auto regionIter = regions.emplace(regionID, std::move(tmpRegion));
  regionIter.first->second.objInstances.push_back(&objInstance[instanceID]);
}  // buildInstanceRegionCategory

}  // namespace

bool SemanticScene::buildHM3DHouse(std::ifstream& ifs,
                                   SemanticScene& scene,
                                   const quatf& /*rotation*/) {
  // temp constructs
  std::map<int, TempHM3DObject> objInstance;
  std::map<int, TempHM3DRegion> regions;
  std::unordered_map<std::string, TempHM3DCategory> categories;
  // build an unknown object
  buildInstanceRegionCategory(0, 0, "Unknown", -1, objInstance, regions,
                              categories);

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }

    // Each line corresponds to a single instance of an annotated object.
    // Line :
    // Unique Instance ID (int), color (hex RGB), category name (may be multiple
    // tokens) (string), room/region ID (int) unique instance ID is first token

    // NOTE : label (idx 2) can include commas, so split on quotes, which will
    // always be around category name
    // idx 0 will be "<ID>,<color>,"
    // idx 1 will be "<category with possible commas>"
    // idx 2 will be ",<region ID>"
    const std::vector<std::string> tokens =
        Cr::Utility::String::splitWithoutEmptyParts(line, '"');
    // sub tokens are ID and color
    const std::vector<std::string> subtokens =
        Cr::Utility::String::splitWithoutEmptyParts(tokens[0], ',');
    // ID is integer
    int instanceID = std::stoi(Cr::Utility::String::trim(subtokens[0]));
    // semantic color is 2nd token, as hex string
    const unsigned colorInt =
        std::stoul(Cr::Utility::String::trim(subtokens[1]), nullptr, 16);
    // convert a hex string containing 3 bytes to a vector3ub (represents a
    // color)
    Mn::Vector3ub colorVec = {uint8_t((colorInt >> 16) & 0xff),
                              uint8_t((colorInt >> 8) & 0xff),
                              uint8_t(colorInt & 0xff)};
    // object category will possibly have commas
    const std::string& objCategoryName = tokens[1];
    // room/region is always last token - get rid of first comma
    int regionID =
        std::stoi(Cr::Utility::String::trim(tokens[tokens.size() - 1], " ,"));

    buildInstanceRegionCategory(instanceID, colorInt, objCategoryName, regionID,
                                objInstance, regions, categories);

  }  // while

  // construct object instance names for each object instance, by setting the
  // instance ID as the idx in the category list
  for (auto& item : categories) {
    std::vector<TempHM3DObject*>& objsWithCat = item.second.objInstances;
    int objCatIdx = 0;
    for (TempHM3DObject* objItem : objsWithCat) {
      objItem->objCatID = objCatIdx++;
    }
    // build category item and place within temp map
    item.second.category_ = std::make_shared<HM3DObjectCategory>(
        HM3DObjectCategory(item.second.ID, item.second.name));
  }

  // build all categories
  // object categories
  scene.categories_.clear();
  scene.categories_.reserve(categories.size());
  for (const auto& catItem : categories) {
    scene.categories_.emplace_back(catItem.second.category_);
  }
  // build all regions
  // regions
  scene.regions_.clear();
  scene.regions_.reserve(regions.size());
  for (auto& regionItem : regions) {
    auto regionPtr = SemanticRegion::create();
    regionPtr->index_ = regionItem.second.regionID;
    for (TempHM3DObject* objItem : regionItem.second.objInstances) {
      objItem->region = regionPtr;
    }
    scene.regions_.emplace_back(std::move(regionPtr));
  }

  // build all object instances
  // object instances
  scene.objects_.clear();
  scene.objects_.reserve(objInstance.size());
  for (auto& item : objInstance) {
    TempHM3DObject obj = item.second;
    auto objPtr = std::make_shared<HM3DObjectInstance>(HM3DObjectInstance(
        obj.objInstanceID, obj.objCatID, obj.objInstanceName, obj.colorInt));
    objPtr->category_ = categories[obj.categoryName].category_;
    // set region
    objPtr->parentIndex_ = obj.region->index_;
    objPtr->region_ = obj.region;
    objPtr->region_->objects_.emplace_back(objPtr);
    scene.objects_.emplace_back(std::move(objPtr));
  }
  scene.hasVertColors_ = true;

  // build colormap from colors defined in ssd
  scene.semanticColorMapBeingUsed_.clear();
  scene.semanticColorMapBeingUsed_.resize(objInstance.size());
  scene.semanticColorToIdAndRegion_.clear();
  scene.semanticColorToIdAndRegion_.reserve(objInstance.size());
  // build the color map with first maxSemanticID elements in proper order
  // to match provided semantic IDs (so that ID is IDX of semantic color in
  // map).  Any overflow colors will be uniquely mapped 1-to-1 to unmapped
  // semantic IDs as their index.
  for (const auto& objPtr : scene.objects_) {
    int idx = objPtr->semanticID();
    if (scene.semanticColorMapBeingUsed_.size() <= idx) {
      // both are same size, so resize both at same time
      scene.semanticColorMapBeingUsed_.resize(idx + 1);
      scene.semanticColorToIdAndRegion_.reserve(idx + 1);
    }
    scene.semanticColorMapBeingUsed_[idx] = objPtr->getColor();
    scene.semanticColorToIdAndRegion_.insert(
        std::make_pair<uint32_t, std::pair<int, int>>(
            objPtr->getColorAsInt(),
            {objPtr->semanticID(), objPtr->region()->getIndex()}));
  }
  // we need to build the bbox on load for each semantic annotation
  // TODO: eventually BBoxes will be pre-generated and stored in semantic text
  // file.
  scene.needBBoxFromVertColors_ = true;

  // TODO: eventually set this via customizable config value.
  // TODO: Using float to potentially allow for this value to include multiple
  // CC's verts via their BBox volume, where float value denotes fraction of
  // largest bbox's volume to include in final BBox.

  // Currently, if this value is > 0.0f, semantic BBox is
  // an AABB in world space built by largest CC vertset by volume sharing
  // semantic color annotation; if 0.0, it uses all verts assigned specific
  // color annotations. Using a float value to
  //
  scene.ccLargestVolToUseForBBox_ = 1.0f;

  // Not used for Hm3d currently
  scene.levels_.clear();
  return true;

}  // SemanticScene::buildHM3DHouse

}  // namespace scene
}  // namespace esp
