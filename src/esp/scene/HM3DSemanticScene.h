// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_HM3DSEMANTICSCENE_H_
#define ESP_SCENE_HM3DSEMANTICSCENE_H_

#include "SemanticScene.h"

namespace esp {
namespace scene {
class HM3DObjectCategory : public SemanticCategory {
 public:
  HM3DObjectCategory(const int id, const std::string& name)
      : id_(id), name_(name) {}

  int index(const std::string& /*mapping*/) const override { return id_; }

  std::string name(const std::string& mapping) const override {
    if (mapping == "category" || mapping == "") {
      return name_;
    } else {
      ESP_ERROR() << "Unknown mapping type:" << mapping;
      return "UNKNOWN";
    }
  }

 protected:
  int id_;
  std::string name_;
  ESP_SMART_POINTERS(HM3DObjectCategory)
};

class HM3DSemanticRegion : public SemanticRegion {
 public:
  int getIndex() const { return index_; }
  ESP_SMART_POINTERS(HM3DSemanticRegion)
};  // class HM3DSemanticRegion

class HM3DObjectInstance : public SemanticObject {
 public:
  HM3DObjectInstance(int objInstanceID,
                     int objCatID,
                     const std::string& name,
                     const Mn::Vector3ub& clr)
      : SemanticObject(), objCatID_(objCatID), name_(name), color_(clr) {
    index_ = objInstanceID;
  }
  Mn::Vector3ub getColor() const { return color_; }

  std::string id() const override { return name_; }

 protected:
  // idx of this particular object instance within the category it belongs in
  const int objCatID_;
  // unique name for this instance
  const std::string name_;
  // specified color for this object instance.
  const Mn::Vector3ub color_;
  ESP_SMART_POINTERS(HM3DObjectInstance)
};

}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_HM3DSEMANTICSCENE_H_
