// Copyright (c) Facebook, Inc. and its affiliates.
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
  if (header != "HM3D Semantic Annotations") {
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
  std::string categoryName;
  std::string objInstanceName;
  Mn::Vector3ub color;
  std::shared_ptr<SemanticRegion> region;

  void setObjInstanceAndName(int _objInstanceID) {
    objInstanceID = _objInstanceID;
    objInstanceName =
        Cr::Utility::formatString("{}_{}", categoryName, objInstanceID);
  }
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

}  // namespace

bool SemanticScene::buildHM3DHouse(std::ifstream& ifs,
                                   SemanticScene& scene,
                                   const quatf& /*rotation*/) {
  // convert a hex string containing 3 bytes to a vector3ub (represents a color)
  auto getVec3ub = [](const std::string& hexValStr) -> Mn::Vector3ub {
    const unsigned val = std::stoul(hexValStr, nullptr, 16);
    return {uint8_t((val >> 16) & 0xff), uint8_t((val >> 8) & 0xff),
            uint8_t((val >> 0) & 0xff)};
  };

  // temp constructs
  std::map<int, TempHM3DObject> objInstance;
  std::map<int, TempHM3DRegion> regions;
  std::unordered_map<std::string, TempHM3DCategory> categories;

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> tokens =
        Cr::Utility::String::splitWithoutEmptyParts(line, ',');
    // Each line corresponds to a single instance of an annotated object.
    // Line :
    // Unique Instance ID (int), color (hex RGB), category name
    // (string), room/region ID (int)
    // instance ID is first token
    int instanceID = std::stoi(tokens[0]);
    // semantic color is 2nd token, as hex string
    auto colorVec = getVec3ub(tokens[1]);
    // object category name
    // construct instance name by counting instances of same category
    // get rid of quotes
    std::string objCategoryName =
        Cr::Utility::String::replaceAll(tokens[2], "\"", "");
    // room/region
    int regionID = std::stoi(tokens[3]);
    // build initial temp object
    TempHM3DObject obj{0, objCategoryName, "", colorVec};
    objInstance[instanceID] = obj;
    // find category, build if dne
    TempHM3DCategory tmpCat{
        static_cast<int>(categories.size()), objCategoryName, {}};
    auto categoryIter = categories.emplace(objCategoryName, tmpCat);
    categoryIter.first->second.objInstances.push_back(&objInstance[instanceID]);
    // find region, build if dne
    TempHM3DRegion tmpRegion{regionID, {}};
    auto regionIter = regions.emplace(regionID, tmpRegion);
    regionIter.first->second.objInstances.push_back(&objInstance[instanceID]);
  }  // while

  // construct object instance names for each object instance, by setting the
  // instance ID as the idx in the category list
  for (auto& item : categories) {
    std::vector<TempHM3DObject*>& objsWithCat = item.second.objInstances;
    int objCatIdx = 0;
    for (TempHM3DObject* objItem : objsWithCat) {
      objItem->setObjInstanceAndName(objCatIdx++);
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
    auto regionPtr = HM3DSemanticRegion::create();
    regionPtr->index_ = regionItem.second.regionID;
    for (TempHM3DObject* objItem : regionItem.second.objInstances) {
      objItem->region = regionPtr;
    }
    scene.regions_.emplace_back(regionPtr);
  }

  // build all object instances
  // object instances
  scene.objects_.clear();
  scene.objects_.reserve(objInstance.size());
  for (auto& item : objInstance) {
    TempHM3DObject obj = item.second;
    auto objPtr = std::make_shared<HM3DObjectInstance>(HM3DObjectInstance(
        item.first, obj.objInstanceID, obj.objInstanceName, obj.color));
    objPtr->category_ = categories[obj.categoryName].category_;
    // set region
    objPtr->parentIndex_ = obj.region->index_;
    objPtr->region_ = obj.region;
    objPtr->region_->objects_.push_back(objPtr);
    scene.objects_.emplace_back(objPtr);
  }
  scene.hasVertColors_ = true;

  scene.levels_.clear();  // Not used for Hm3d currently

  return true;

}  // SemanticScene::buildHM3DHouse

}  // namespace scene
}  // namespace esp
