// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_SUNCGSEMANTICSCENE_H_
#define ESP_SCENE_SUNCGSEMANTICSCENE_H_

#include "SemanticScene.h"

#include <utility>

namespace esp {
namespace scene {

class SuncgSemanticObject : public SemanticObject {
 public:
  std::string id() const override;

 protected:
  std::string nodeId_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SuncgSemanticObject)
};

class SuncgSemanticRegion : public SemanticRegion {
 public:
  std::string id() const override;

 protected:
  std::string nodeId_;
  std::vector<int> nodeIndicesInLevel_;
  friend SemanticScene;
  ESP_SMART_POINTERS(SuncgSemanticRegion)
};

struct SuncgObjectCategory : public SemanticCategory {
  SuncgObjectCategory(std::string nodeId, std::string modelId)
      : nodeId_(std::move(nodeId)), modelId_(std::move(modelId)) {}

  int index(const std::string& mapping) const override;

  std::string name(const std::string& mapping) const override;

 protected:
  std::string nodeId_;
  std::string modelId_;
  friend SemanticScene;

  ESP_SMART_POINTERS(SuncgObjectCategory)
};

struct SuncgRegionCategory : public SemanticCategory {
  SuncgRegionCategory(std::string nodeId, std::vector<std::string> categories)
      : nodeId_(std::move(nodeId)), categories_(std::move(categories)) {}

  int index(const std::string& mapping) const override;

  std::string name(const std::string& mapping) const override;

 protected:
  std::string nodeId_;
  std::vector<std::string> categories_;

  ESP_SMART_POINTERS(SuncgRegionCategory)
};
}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_SUNCGSEMANTICSCENE_H_
