// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <string>
#include "esp/core/esp.h"

namespace esp {
namespace scene {

struct SceneConfiguration {
  //! uuid dataset to which this scene belongs
  std::string dataset = "";
  //! uuid of the specific scene
  std::string id = "";
  //! map of (filetype,filepath) key-value pairs
  std::map<std::string, std::string> filepaths;
  //! up direction of this scene in global coordinates
  vec3f sceneUpDir = vec3f(0, 0, 0);
  //! front direction of this scene in global coordinates
  vec3f sceneFrontDir = vec3f(0, 0, 0);
  //! scale of scene (this many virtual units per meter)
  float sceneScaleUnit = 1.0f;
  ESP_SMART_POINTERS(SceneConfiguration)
};

bool operator==(const SceneConfiguration& a, const SceneConfiguration& b);
bool operator!=(const SceneConfiguration& a, const SceneConfiguration& b);

}  // namespace scene
}  // namespace esp
