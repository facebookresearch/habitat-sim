// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_MP3DSEMANTICSCENE_H_
#define ESP_SCENE_MP3DSEMANTICSCENE_H_

#include "SemanticScene.h"

namespace esp {
namespace scene {
struct Mp3dObjectCategory : public SemanticCategory {
  int index(const std::string& mapping) const override;

  std::string name(const std::string& mapping) const override;

 protected:
  int index_ = ID_UNDEFINED;
  int categoryMappingIndex_ = ID_UNDEFINED;
  int mpcat40Index_ = ID_UNDEFINED;
  std::string categoryMappingName_ = "";
  std::string mpcat40Name_ = "";
  friend SemanticScene;

  ESP_SMART_POINTERS(Mp3dObjectCategory)
};

struct Mp3dRegionCategory : public SemanticCategory {
  explicit Mp3dRegionCategory(const char labelCode) : labelCode_(labelCode) {}

  int index(const std::string& mapping) const override;

  std::string name(const std::string& mapping) const override;

 protected:
  char labelCode_;

  ESP_SMART_POINTERS(Mp3dRegionCategory)
};
}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_MP3DSEMANTICSCENE_H_
