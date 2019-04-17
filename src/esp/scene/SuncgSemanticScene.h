
#pragma once

#include "esp/scene/SemanticScene.h"

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
  SuncgObjectCategory(const std::string& nodeId, const std::string& modelId)
      : nodeId_(nodeId), modelId_(modelId) {}

  int index(const std::string& mapping) const override;

  std::string name(const std::string& mapping) const override;

 protected:
  std::string nodeId_;
  std::string modelId_;
  friend SemanticScene;

  ESP_SMART_POINTERS(SuncgObjectCategory)
};

struct SuncgRegionCategory : public SemanticCategory {
  SuncgRegionCategory(const std::string& nodeId,
                      const std::vector<std::string> categories)
      : nodeId_(nodeId), categories_(categories) {}

  int index(const std::string& mapping) const override;

  std::string name(const std::string& mapping) const override;

 protected:
  std::string nodeId_;
  std::vector<std::string> categories_;

  ESP_SMART_POINTERS(SuncgRegionCategory)
};
}  // namespace scene
}  // namespace esp
