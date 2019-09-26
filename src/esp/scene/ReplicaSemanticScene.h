// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "SemanticScene.h"

namespace esp {
namespace scene {

struct ReplicaObjectCategory : public SemanticCategory {
  ReplicaObjectCategory(const int id, const std::string& name)
      : id_(id), name_(name) {}

  int index(const std::string& mapping) const override { return id_; }

  std::string name(const std::string& mapping) const override {
    if (mapping == "category" || mapping == "") {
      return name_;
    } else {
      LOG(ERROR) << "Unknown mapping type: " << mapping;
      return "UNKNOWN";
    }
  }

 protected:
  int id_;
  std::string name_;
  friend SemanticScene;

  ESP_SMART_POINTERS(ReplicaObjectCategory)
};

}  // namespace scene
}  // namespace esp
