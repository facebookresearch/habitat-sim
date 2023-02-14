// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Mp3dSemanticScene.h"
#include "SemanticScene.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace esp {
namespace scene {

static const std::map<char, std::string> kRegionCategoryMap = {
    {'a', "bathroom"},  // with toilet and sink
    {'b', "bedroom"},
    {'c', "closet"},
    {'d', "dining room"},  // includes “breakfast rooms”, rooms to eat in
    {'e', "entryway/foyer/lobby"},  // should be the front door, not any door
    {'f',
     "familyroom/lounge"},  // room where family hangs out, not just couches
    {'g', "garage"},
    {'h', "hallway"},
    {'i', "library"},  // like a library at a university, not individual study
    {'j', "laundryroom/mudroom"},  // place where people do laundry, etc.
    {'k', "kitchen"},
    {'l', "living room"},  // main “showcase” living room, not just couches
    {'m', "meetingroom/conferenceroom"},
    {'n', "lounge"},  // relax in comfy chairs/couches, not family/living room
    {'o', "office"},  // usually for an individual, or a small set of people
    {'p', "porch/terrace/deck"},  // must be outdoors on ground level
    {'r',
     "rec/game"},  // should have recreational objects, like pool table, etc.
    {'s', "stairs"},
    {'t', "toilet"},  // should be a small room with ONLY a toilet
    {'u', "utilityroom/toolroom"},
    {'v', "tv"},  // must have theater-style seating
    {'w', "workout/gym/exercise"},
    {'x', "outdoor"},  // outdoor areas containing grass, plants, bushes, trees
    {'y', "balcony"},  // must be outside and must not be on ground floor
    {'z', "other room"},  // it is clearly a room, but the function is not clear
    {'B', "bar"},
    {'C', "classroom"},
    {'D', "dining booth"},
    {'S', "spa/sauna"},
    {'Z', "junk"}  // reflections of mirrors, points floating in space, etc.
};

int Mp3dObjectCategory::index(const std::string& mapping) const {
  if (mapping == "" || mapping == "mpcat40") {
    return mpcat40Index_;
  } else if (mapping == "raw") {
    return categoryMappingIndex_;
  } else {
    ESP_ERROR() << "Unknown SemanticCategory mapping" << mapping;
    return ID_UNDEFINED;
  }
}

std::string Mp3dObjectCategory::name(const std::string& mapping) const {
  if (mapping == "" || mapping == "mpcat40") {
    return mpcat40Name_;
  } else if (mapping == "raw") {
    return categoryMappingName_;
  } else {
    ESP_ERROR() << "Unknown SemanticCategory mapping" << mapping;
    return "";
  }
}

int Mp3dRegionCategory::index(const std::string&) const {
  return std::distance(kRegionCategoryMap.begin(),
                       kRegionCategoryMap.find(labelCode_));
}

std::string Mp3dRegionCategory::name(const std::string&) const {
  return kRegionCategoryMap.at(labelCode_);
}

bool SemanticScene::loadMp3dHouse(
    const std::string& houseFilename,
    SemanticScene& scene,
    const quatf& rotation /* = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                       geo::ESP_GRAVITY) */ ) {
  if (!checkFileExists(houseFilename, "loadMp3dHouse")) {
    return false;
  }

  // open stream and determine house format version
  std::ifstream ifs = std::ifstream(houseFilename);
  std::string header;
  std::getline(ifs, header);
  if (header != "ASCII 1.1") {
    ESP_ERROR() << "Unsupported Mp3d House format header" << header
                << "in file name" << houseFilename;
    return false;
  }

  return buildMp3dHouse(ifs, scene, rotation);
}  // SemanticScene::loadMp3dHouse

bool SemanticScene::buildMp3dHouse(std::ifstream& ifs,
                                   SemanticScene& scene,
                                   const quatf& rotation) {
  const bool hasWorldRotation = !rotation.isApprox(quatf::Identity());

  auto getVec3f = [&](const std::vector<std::string>& tokens, int offset,
                      bool applyRotation = true) -> vec3f {
    const float x = std::stof(tokens[offset]);
    const float y = std::stof(tokens[offset + 1]);
    const float z = std::stof(tokens[offset + 2]);
    vec3f p = vec3f(x, y, z);
    if (applyRotation && hasWorldRotation) {
      p = rotation * p;
    }
    return p;
  };

  auto getBBox = [&](const std::vector<std::string>& tokens,
                     int offset) -> box3f {
    // Get the bounding box without rotating as rotating min/max is odd
    box3f sceneBox{getVec3f(tokens, offset, /*applyRotation=*/false),
                   getVec3f(tokens, offset + 3, /*applyRotation=*/false)};
    if (!hasWorldRotation)
      return sceneBox;

    // Apply the rotation to center/sizes
    const vec3f worldCenter = rotation * sceneBox.center();
    const vec3f worldHalfSizes =
        (rotation * sceneBox.sizes()).array().abs().matrix() / 2.0f;
    // Then remake the box with min/max computed from rotated center/size
    return box3f{(worldCenter - worldHalfSizes).eval(),
                 (worldCenter + worldHalfSizes).eval()};
  };

  auto getOBB = [&](const std::vector<std::string>& tokens, int offset) {
    const vec3f center = getVec3f(tokens, offset);

    // Don't need to apply rotation here, it'll already be added in by getVec3f
    mat3f boxRotation;
    boxRotation.col(0) << getVec3f(tokens, offset + 3);
    boxRotation.col(1) << getVec3f(tokens, offset + 6);
    boxRotation.col(2) << boxRotation.col(0).cross(boxRotation.col(1));

    // Don't apply the world rotation here, that'll get added by boxRotation
    const vec3f radius = getVec3f(tokens, offset + 9, /*applyRotation=*/false);

    return geo::OBB(center, 2 * radius, quatf(boxRotation));
  };

  scene.categories_.clear();
  scene.levels_.clear();
  scene.regions_.clear();
  scene.objects_.clear();

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> tokens =
        Cr::Utility::String::splitWithoutEmptyParts(line, ' ');
    switch (line[0]) {
      case 'H': {  // house
        // H name label #images #panoramas #vertices #surfaces #segments
        //   #objects #categories #regions #portals #levels  0 0 0 0 0
        //   xlo ylo zlo xhi yhi zhi  0 0 0 0 0
        scene.name_ = tokens[1];
        scene.label_ = tokens[2];
        scene.elementCounts_["images"] = std::stoi(tokens[3]);
        scene.elementCounts_["panoramas"] = std::stoi(tokens[4]);
        scene.elementCounts_["vertices"] = std::stoi(tokens[5]);
        scene.elementCounts_["surfaces"] = std::stoi(tokens[6]);
        scene.elementCounts_["segments"] = std::stoi(tokens[7]);
        scene.elementCounts_["objects"] = std::stoi(tokens[8]);
        scene.elementCounts_["categories"] = std::stoi(tokens[9]);
        scene.elementCounts_["regions"] = std::stoi(tokens[10]);
        scene.elementCounts_["portals"] = std::stoi(tokens[11]);
        scene.elementCounts_["levels"] = std::stoi(tokens[12]);
        scene.bbox_ = getBBox(tokens, 18);
        break;
      }
      case 'L': {  // level
        // L level_index #regions label  px py pz  xlo ylo zlo xhi yhi zhi  0 0
        //   0 0 0
        scene.levels_.emplace_back(SemanticLevel::create());
        auto& level = scene.levels_.back();
        level->index_ = std::stoi(tokens[1]);
        // NOTE tokens[2] is number of regions in level which we don't need
        level->labelCode_ = tokens[3];
        level->position_ = getVec3f(tokens, 4);
        level->bbox_ = getBBox(tokens, 7);
        break;
      }
      case 'R': {  // region
        // R region_index level_index 0 0 label  px py pz  xlo ylo zlo xhi yhi
        //   zhi height  0 0 0 0
        scene.regions_.emplace_back(SemanticRegion::create());
        auto& region = scene.regions_.back();
        region->index_ = std::stoi(tokens[1]);
        region->parentIndex_ = std::stoi(tokens[2]);
        region->category_ = std::make_shared<Mp3dRegionCategory>(tokens[5][0]);
        region->position_ = getVec3f(tokens, 6);
        region->bbox_ = getBBox(tokens, 9);
        if (region->parentIndex_ >= 0) {
          region->level_ = scene.levels_[region->parentIndex_];
          region->level_->regions_.push_back(region);
        }
        break;
      }
      // NOLINTNEXTLINE(bugprone-branch-clone)
      case 'P': {  // portal or panorama
        // P portal_index region0_index region1_index label  xlo ylo zlo xhi
        //   yhi zhi  0 0 0 0
        // P name  panorama_index region_index 0  px py pz  0 0 0 0 0
        break;
      }
      // NOLINTNEXTLINE(bugprone-branch-clone)
      case 'S': {  // surface
        // S surface_index region_index 0 label px py pz  nx ny nz  xlo ylo
        // zlo
        //   xhi yhi zhi  0 0 0 0 0
        break;
      }
      // NOLINTNEXTLINE(bugprone-branch-clone)
      case 'V': {  // vertex
        // V vertex_index surface_index label  px py pz  nx ny nz  0 0 0
        break;
      }
      // NOLINTNEXTLINE(bugprone-branch-clone)
      case 'I': {  // image
        // I image_index panorama_index  name camera_index yaw_index e00 e01
        //   e02 e03 e10 e11 e12 e13 e20 e21 e22 e23 e30 e31 e32 e33  i00 i01
        //   i02  i10 i11 i12 i20 i21 i22  width height  px py pz  0 0 0 0 0
        break;
      }
      case 'C': {  // category
        // C category_index category_mapping_index category_mapping_name
        //   mpcat40_index mpcat40_name 0 0 0 0 0
        scene.categories_.emplace_back(std::make_shared<Mp3dObjectCategory>());
        auto& category =
            static_cast<Mp3dObjectCategory&>(*scene.categories_.back());
        category.index_ = std::stoi(tokens[1]);
        category.categoryMappingIndex_ = std::stoi(tokens[2]);
        std::string catName = tokens[3];
        std::replace(catName.begin(), catName.end(), '#', ' ');
        category.categoryMappingName_ = catName;
        category.mpcat40Index_ = std::stoi(tokens[4]);
        category.mpcat40Name_ = tokens[5];
        break;
      }
      case 'O': {  // object
        // O object_index region_index category_index px py pz  a0x a0y a0z
        //   a1x a1y a1z  r0 r1 r2 0 0 0 0 0 0 0 0
        scene.objects_.emplace_back(SemanticObject::create());
        auto& object = scene.objects_.back();
        object->index_ = std::stoi(tokens[1]);
        object->parentIndex_ = std::stoi(tokens[2]);
        int categoryIndex = std::stoi(tokens[3]);
        if (categoryIndex < 0) {  // no category
          object->category_ = std::make_shared<Mp3dObjectCategory>();
        } else {
          object->category_ = scene.categories_[categoryIndex];
        }
        object->obb_ = getOBB(tokens, 4);
        if (object->parentIndex_ >= 0) {
          object->region_ = scene.regions_[object->parentIndex_];
          object->region_->objects_.push_back(object);
        }
        break;
      }
      case 'E': {  // segment
        // E segment_index object_index id area px py pz xlo ylo zlo xhi yhi
        // zhi 0 0 0 0 0
        const int objectIndex = std::stoi(tokens[2]);
        const int segmentId = std::stoi(tokens[3]);
        // NOTE: segmentId = regionIndex * 1000000 + segmentId
        scene.segmentToObjectIndex_[segmentId] = objectIndex;
        break;
      }
      default: {
        break;
      }
    }
  }
  scene.hasVertColors_ = true;
  return true;

}  // SemanticScene::buildMp3dHouse

}  // namespace scene
}  // namespace esp
