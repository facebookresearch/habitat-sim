// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_OBJECTCONTROLS_H_
#define ESP_SCENE_OBJECTCONTROLS_H_

#include <functional>
#include <map>
#include <string>

#include "esp/core/Esp.h"
#include "esp/core/EspEigen.h"

namespace esp {
namespace scene {

// forward declaration
class SceneNode;

class ObjectControls {
 public:
  ObjectControls();

  typedef std::function<SceneNode&(SceneNode&, float)> MoveFunc;
  typedef std::function<vec3f(const vec3f&, const vec3f&)> MoveFilterFunc;
  ObjectControls& setMoveFilterFunction(MoveFilterFunc filterFunc);

  ObjectControls& action(SceneNode& object,
                         const std::string& actName,
                         float distance,
                         bool applyFilter = true);
  ObjectControls& operator()(SceneNode& object,
                             const std::string& actName,
                             float distance,
                             bool applyFilter = true) {
    return action(object, actName, distance, applyFilter);
  }

  inline const std::map<std::string, MoveFunc>& getMoveFuncMap() const {
    return moveFuncMap_;
  }

 protected:
  MoveFilterFunc moveFilterFunc_ = [](const vec3f& /*start*/,
                                      const vec3f& end) { return end; };
  std::map<std::string, MoveFunc> moveFuncMap_;

  ESP_SMART_POINTERS(ObjectControls)
};

}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_OBJECTCONTROLS_H_
