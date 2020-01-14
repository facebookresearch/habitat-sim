// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "esp/core/esp.h"
#include "esp/geo/OBB.h"

namespace esp {
namespace scene {

//! Represents a semantic category
class SemanticCategory {
 public:
  virtual ~SemanticCategory() = default;
  //! Return index of SemanticCategory under given mapping
  virtual int index(const std::string& mapping = "") const = 0;
  //! Return name of SemanticCategory under given mapping
  virtual std::string name(const std::string& mapping = "") const = 0;

  ESP_SMART_POINTERS(SemanticCategory);
};

// forward declarations
class SemanticObject;
class SemanticRegion;
class SemanticLevel;

//! Represents a scene with containing semantically annotated
//! levels, regions and objects
class SemanticScene {
 public:
  ~SemanticScene() { LOG(INFO) << "Deconstructing SemanticScene"; }
  //! return axis aligned bounding box of this House
  box3f aabb() const { return bbox_; }

  //! return total number of given element type
  int count(const std::string& element) const {
    return elementCounts_.at(element);
  }

  //! return all SemanticCategories of objects in this House
  const std::vector<std::shared_ptr<SemanticCategory>>& categories() const {
    return categories_;
  }

  //! return all Levels in this House
  const std::vector<std::shared_ptr<SemanticLevel>>& levels() const {
    return levels_;
  }

  //! return all Regions in this House
  const std::vector<std::shared_ptr<SemanticRegion>>& regions() const {
    return regions_;
  }

  //! return all Objects in this House
  const std::vector<std::shared_ptr<SemanticObject>>& objects() const {
    return objects_;
  }

  const std::unordered_map<int, int>& getSemanticIndexMap() const {
    return segmentToObjectIndex_;
  }

  //! convert semantic mesh mask index to object index or ID_UNDEFINED if
  //! not mapped
  inline int semanticIndexToObjectIndex(int maskIndex) const {
    if (segmentToObjectIndex_.count(maskIndex) > 0) {
      return segmentToObjectIndex_.at(maskIndex);
    } else {
      return ID_UNDEFINED;
    }
  }

  //! load SemanticScene from a Gibson house format file
  static bool loadGibsonHouse(const std::string& filename,
                              SemanticScene& scene,
                              const quatf& rotation = quatf::Identity());

  //! load SemanticScene from a Matterport3D House format filename
  static bool loadMp3dHouse(
      const std::string& filename,
      SemanticScene& scene,
      const quatf& rotation = quatf::FromTwoVectors(-vec3f::UnitZ(),
                                                    geo::ESP_GRAVITY));

  //! load SemanticScene from a SUNCG house format file
  static bool loadReplicaHouse(const std::string& filename,
                               SemanticScene& scene,
                               const quatf& rotation = quatf::Identity());

  //! load SemanticScene from a SUNCG house format file
  static bool loadSuncgHouse(const std::string& filename,
                             SemanticScene& scene,
                             const quatf& rotation = quatf::Identity());

 protected:
  std::string name_;
  std::string label_;
  box3f bbox_;
  std::map<std::string, int> elementCounts_;
  std::vector<std::shared_ptr<SemanticCategory>> categories_;
  std::vector<std::shared_ptr<SemanticLevel>> levels_;
  std::vector<std::shared_ptr<SemanticRegion>> regions_;
  std::vector<std::shared_ptr<SemanticObject>> objects_;
  //! map from combined region-segment id to objectIndex for semantic mesh
  std::unordered_map<int, int> segmentToObjectIndex_;

  ESP_SMART_POINTERS(SemanticScene)
};

//! Represents a level of a SemanticScene
class SemanticLevel {
 public:
  virtual ~SemanticLevel() = default;
  virtual std::string id() const { return std::to_string(index_); }

  const std::vector<std::shared_ptr<SemanticRegion>>& regions() const {
    return regions_;
  }

  const std::vector<std::shared_ptr<SemanticObject>>& objects() const {
    return objects_;
  }

  box3f aabb() const { return bbox_; }

 protected:
  int index_;
  std::string labelCode_;
  vec3f position_;
  box3f bbox_;
  std::vector<std::shared_ptr<SemanticObject>> objects_;
  std::vector<std::shared_ptr<SemanticRegion>> regions_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SemanticLevel)
};

//! Represents a region (typically room) in a level of a house
class SemanticRegion {
 public:
  virtual ~SemanticRegion() = default;
  virtual std::string id() const {
    if (level_ != nullptr) {
      return level_->id() + "_" + std::to_string(index_);
    } else {
      return "_" + std::to_string(index_);
    }
  }

  const SemanticLevel::ptr level() const { return level_; }

  const std::vector<std::shared_ptr<SemanticObject>>& objects() const {
    return objects_;
  }

  box3f aabb() const { return bbox_; }

  const SemanticCategory::ptr category() const { return category_; }

 protected:
  int index_;
  int parentIndex_;
  std::shared_ptr<SemanticCategory> category_;
  vec3f position_;
  box3f bbox_;
  vec3f floorNormal_;
  std::vector<vec3f> floorPoints_;
  std::vector<std::shared_ptr<SemanticObject>> objects_;
  std::shared_ptr<SemanticLevel> level_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SemanticRegion)
};

//! Represents a distinct semantically annotated object
class SemanticObject {
 public:
  virtual ~SemanticObject() = default;
  virtual std::string id() const {
    if (region_ != nullptr) {
      return region_->id() + "_" + std::to_string(index_);
    } else {
      return "_" + std::to_string(index_);
    }
  }

  const SemanticRegion::ptr region() const { return region_; }

  box3f aabb() const { return obb_.toAABB(); }

  geo::OBB obb() const { return obb_; }

  const SemanticCategory::ptr category() const { return category_; }

 protected:
  int index_;
  int parentIndex_;
  std::shared_ptr<SemanticCategory> category_;
  geo::OBB obb_;
  std::shared_ptr<SemanticRegion> region_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SemanticObject)
};

}  // namespace scene
}  // namespace esp
