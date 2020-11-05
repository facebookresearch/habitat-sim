// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SuncgSemanticScene.h"
#include "SemanticScene.h"
#include "SuncgObjectCategoryMap.h"

#include <map>
#include <string>

#include <Corrade/Utility/String.h>

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
namespace scene {

std::string SuncgSemanticObject::id() const {
  return nodeId_;
}

std::string SuncgSemanticRegion::id() const {
  return nodeId_;
}

int SuncgObjectCategory::index(const std::string&) const {
  return ID_UNDEFINED;
}

std::string SuncgObjectCategory::name(const std::string& mapping) const {
  if (mapping == "model_id") {
    return modelId_;
  } else if (mapping == "node_id") {
    return nodeId_;
  } else if (mapping == "category" || mapping == "") {
    if (kSuncgObjectCategoryMap.count(modelId_) > 0) {
      return kSuncgObjectCategoryMap.at(modelId_);
    } else {
      // This is a non-model object where modelId stores type (e.g., Wall)
      return modelId_;
    }
  } else {
    LOG(ERROR) << "Unknown mapping type: " << mapping;
    return "UNKNOWN";
  }
}

int SuncgRegionCategory::index(const std::string&) const {
  // NOTE: SUNCG regions are not linearized
  return ID_UNDEFINED;
}
std::string SuncgRegionCategory::name(const std::string& mapping) const {
  if (mapping == "node_id") {
    return nodeId_;
  } else if (mapping == "" || mapping == "category") {
    return Corrade::Utility::String::join(categories_, ',');
  } else {
    LOG(ERROR) << "Unknown mapping type: " << mapping;
    return "UNKNOWN";
  }
}

bool SemanticScene::loadSuncgHouse(
    const std::string& houseFilename,
    SemanticScene& scene,
    const quatf& worldRotation /* = quatf::Identity() */) {
  if (!io::exists(houseFilename)) {
    LOG(ERROR) << "Could not load file " << houseFilename;
    return false;
  }

  const bool hasWorldRotation = !worldRotation.isApprox(quatf::Identity());

  auto getVec3f = [&](const io::JsonDocument::ValueType& v) {
    const float x = v[0].GetFloat();
    const float y = v[1].GetFloat();
    const float z = v[2].GetFloat();
    vec3f p = vec3f(x, y, z);
    if (hasWorldRotation) {
      p = worldRotation * p;
    }
    return p;
  };

  auto getBBox = [&](const io::JsonDocument::ValueType& v) {
    return box3f(getVec3f(v["min"]), getVec3f(v["max"]));
  };

  scene.categories_.clear();
  scene.levels_.clear();
  scene.regions_.clear();
  scene.objects_.clear();

  // top-level scene
  VLOG(1) << "Parsing " << houseFilename;
  const auto& json = io::parseJsonFile(houseFilename);
  VLOG(1) << "Parsed.";
  scene.name_ = json["id"].GetString();
  const auto& levels = json["levels"].GetArray();
  scene.elementCounts_["levels"] = static_cast<int>(levels.Size());
  scene.bbox_ = getBBox(json["bbox"]);

  // store nodeIds to obtain linearized index for semantic masks
  std::vector<std::string> nodeIds;

  int iLevel = 0;
  for (const auto& jsonLevel : levels) {
    VLOG(1) << "Parsing level iLevel=" << iLevel;
    const std::string levelId = jsonLevel["id"].GetString();
    const auto& nodes = jsonLevel["nodes"].GetArray();

    scene.levels_.emplace_back(SemanticLevel::create());
    auto& level = scene.levels_.back();
    level->index_ = iLevel;
    if (jsonLevel.HasMember("bbox")) {
      level->bbox_ = getBBox(jsonLevel["bbox"]);
    }

    // maps from JSON node index to created object index (per level)
    std::vector<int> nodeToObjectIndex(nodes.Size(), ID_UNDEFINED);

    // room index
    int jRoom = 0;

    // level nodes
    for (int jNode = 0; jNode < nodes.Size(); ++jNode) {
      const auto& node = nodes[jNode];
      const std::string nodeId = node["id"].GetString();
      const std::string nodeType = node["type"].GetString();
      const int valid = node["valid"].GetInt();
      if (valid == 0) {
        continue;
      }

      VLOG(1) << "Parsing node jNode=" << jNode << " nodeId=" << nodeId
              << " nodeType=" << nodeType;

      // helper for creating objects
      auto createObjectFunc = [&](const std::string& nodeId,
                                  const std::string& modelId) {
        const int nodeIndex = nodeIds.size();
        nodeIds.push_back(nodeId);
        auto object = SuncgSemanticObject::create();
        object->nodeId_ = nodeId;
        object->index_ = nodeIndex;
        object->category_ =
            std::make_shared<SuncgObjectCategory>(nodeId, modelId);
        object->obb_ = geo::OBB(getBBox(node["bbox"]));
        scene.objects_.push_back(object);
        nodeToObjectIndex[jNode] = level->objects_.size();
        level->objects_.push_back(object);
      };

      if (nodeType == "Room") {
        // create room region
        auto region = SuncgSemanticRegion::create();
        region->nodeId_ = nodeId;
        region->index_ = jRoom;
        region->parentIndex_ = iLevel;
        std::vector<std::string> roomTypes;
        roomTypes.reserve(node["roomTypes"].Size());

        for (int jRoomType = 0; jRoomType < node["roomTypes"].Size();
             ++jRoomType) {
          roomTypes.push_back(node["roomTypes"][jRoomType].GetString());
        }
        for (int iChildNode = 0; iChildNode < node["nodeIndices"].Size();
             ++iChildNode) {
          region->nodeIndicesInLevel_.push_back(
              node["nodeIndices"][iChildNode].GetInt());
        }
        region->category_ =
            std::make_shared<SuncgRegionCategory>(nodeId, roomTypes);
        region->bbox_ = getBBox(node["bbox"]);
        region->level_ = scene.levels_[region->parentIndex_];
        region->level_->regions_.push_back(region);
        scene.regions_.push_back(region);

        const int hideCeiling = node["hideCeiling"].GetInt();
        const int hideFloor = node["hideFloor"].GetInt();
        const int hideWalls = node["hideWalls"].GetInt();

        if (hideCeiling != 1) {
          createObjectFunc(nodeId + "c", "Ceiling");
        }
        if (hideWalls != 1) {
          createObjectFunc(nodeId + "w", "Wall");
        }
        if (hideFloor != 1) {
          createObjectFunc(nodeId + "f", "Floor");
        }
        jRoom++;
      } else if (nodeType == "Object") {
        createObjectFunc(nodeId, node["modelId"].GetString());
      } else if (nodeType == "Box") {
        createObjectFunc(nodeId, "Box");
      } else if (nodeType == "Ground") {
        createObjectFunc(nodeId, "Ground");
      } else {
        LOG(ERROR) << "Unrecognized SUNCG house node type " << nodeType;
      }
    }  // for node

    // now hook up nodes in this level to their room regions
    VLOG(1) << "Connecting child nodes to parent regions";
    for (int jRoom = 0; jRoom < level->regions().size(); ++jRoom) {
      auto room = std::static_pointer_cast<SuncgSemanticRegion>(
          level->regions()[jRoom]);
      VLOG(1) << "jRoom=" << jRoom;
      for (int nodeIndexInLevel : room->nodeIndicesInLevel_) {
        const int objectIndexInLevel = nodeToObjectIndex[nodeIndexInLevel];
        VLOG(1) << "nodeIndexInLevel=" << nodeIndexInLevel << " -> "
                << objectIndexInLevel;
        if (objectIndexInLevel == ID_UNDEFINED) {
          // node was not valid and ignored, so skip here also
          continue;
        }
        ASSERT(objectIndexInLevel >= 0 &&
               objectIndexInLevel < level->objects().size());
        auto& object = level->objects()[objectIndexInLevel];
        object->parentIndex_ = jRoom;
        object->region_ = room;
        room->objects_.push_back(object);
      }
    }

    iLevel++;
  }  // for level
  return true;
}

}  // namespace scene
}  // namespace esp
