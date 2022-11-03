// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectControls.h"

#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

#include "SceneNode.h"
#include "esp/core/Esp.h"

using Magnum::EigenIntegration::cast;

namespace esp {
namespace scene {

SceneNode& moveRight(SceneNode& object, float distance) {
  // TODO: this assumes no scale is applied
  object.translateLocal(object.transformation().right() * distance);
  return object;
}

SceneNode& moveLeft(SceneNode& object, float distance) {
  return moveRight(object, -distance);
}

SceneNode& moveUp(SceneNode& object, float distance) {
  // TODO: this assumes no scale is applied
  // Note: this is not a body action and is applied to the sensor rather than
  // the Agent, so it will move the sensor in Agent's +Y (up) direction
  object.translate(Magnum::Vector3(0, 1, 0) * distance);
  return object;
}

SceneNode& moveDown(SceneNode& object, float distance) {
  return moveUp(object, -distance);
}

SceneNode& moveBackward(SceneNode& object, float distance) {
  // TODO: this assumes no scale is applied
  object.translateLocal(object.transformation().backward() * distance);
  return object;
}

SceneNode& moveForward(SceneNode& object, float distance) {
  return moveBackward(object, -distance);
}

SceneNode& turnLeft(SceneNode& object, float angleInDegrees) {
  object.rotateYLocal(Magnum::Deg(angleInDegrees));
  object.setRotation(object.rotation().normalized());
  return object;
}

SceneNode& turnRight(SceneNode& object, float angleInDegrees) {
  return turnLeft(object, -angleInDegrees);
}

SceneNode& lookUp(SceneNode& object, float angleInDegrees) {
  object.rotateXLocal(Magnum::Deg(angleInDegrees));
  object.setRotation(object.rotation().normalized());
  return object;
}

SceneNode& lookDown(SceneNode& object, float angleInDegrees) {
  return lookUp(object, -angleInDegrees);
}

ObjectControls::ObjectControls() {
  moveFuncMap_["moveRight"] = &moveRight;
  moveFuncMap_["moveLeft"] = &moveLeft;
  moveFuncMap_["moveUp"] = &moveUp;
  moveFuncMap_["moveDown"] = &moveDown;
  moveFuncMap_["moveForward"] = &moveForward;
  moveFuncMap_["moveBackward"] = &moveBackward;
  moveFuncMap_["turnLeft"] = &turnLeft;
  moveFuncMap_["turnRight"] = &turnRight;
  moveFuncMap_["lookUp"] = &lookUp;
  moveFuncMap_["lookDown"] = &lookDown;
}

ObjectControls& ObjectControls::setMoveFilterFunction(
    MoveFilterFunc filterFunc) {
  moveFilterFunc_ = std::move(filterFunc);
  return *this;
}

ObjectControls& ObjectControls::action(SceneNode& object,
                                       const std::string& actName,
                                       float distance,
                                       bool applyFilter /* = true */) {
  auto moveFuncMapIter = moveFuncMap_.find(actName);
  if (moveFuncMapIter != moveFuncMap_.end()) {
    if (applyFilter) {
      // TODO: use magnum math for the filter func as well?
      const auto startPosition =
          cast<vec3f>(object.absoluteTransformation().translation());
      moveFuncMapIter->second(object, distance);
      const auto endPos =
          cast<vec3f>(object.absoluteTransformation().translation());
      const vec3f filteredEndPosition = moveFilterFunc_(startPosition, endPos);
      object.translate(Magnum::Vector3(vec3f(filteredEndPosition - endPos)));
    } else {
      moveFuncMapIter->second(object, distance);
    }
  } else {
    ESP_ERROR() << "Tried to perform unknown action with name" << actName;
  }

  return *this;
}

}  // namespace scene
}  // namespace esp
