// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectControls.h"

#include "SceneNode.h"
#include "esp/core/esp.h"

namespace esp {
namespace scene {

SceneNode& moveRight(SceneNode& object, float distance) {
  // TODO: this assumes no scale is applied
  const vec3f& x = object.getTransformation().col(0).head(3);
  object.translateLocal(x * distance);
  return object;
}

SceneNode& moveLeft(SceneNode& object, float distance) {
  return moveRight(object, -distance);
}

SceneNode& moveUp(SceneNode& object, float distance) {
  // TODO: this assumes no scale is applied
  const vec3f& y = object.getTransformation().col(1).head(3);
  object.translateLocal(y * distance);
  return object;
}

SceneNode& moveDown(SceneNode& object, float distance) {
  return moveUp(object, -distance);
}

SceneNode& moveBackward(SceneNode& object, float distance) {
  // TODO: this assumes no scale is applied
  const vec3f& z = object.getTransformation().col(2).head(3);
  object.translateLocal(z * distance);
  return object;
}

SceneNode& moveForward(SceneNode& object, float distance) {
  return moveBackward(object, -distance);
}

SceneNode& lookLeft(SceneNode& object, float angleInDegrees) {
  // TODO pull out into proper utility
  const float angleInRad = angleInDegrees * 0.0174533f;
  object.rotateYLocal(angleInRad);
  object.setRotation(object.getRotation().normalized());
  return object;
}

SceneNode& lookRight(SceneNode& object, float angleInDegrees) {
  return lookLeft(object, -angleInDegrees);
}

SceneNode& lookUp(SceneNode& object, float angleInDegrees) {
  // TODO pull out into proper utility
  const float angleInRad = angleInDegrees * 0.0174533f;
  object.rotateXLocal(angleInRad);
  object.setRotation(object.getRotation().normalized());
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
  moveFuncMap_["lookLeft"] = &lookLeft;
  moveFuncMap_["lookRight"] = &lookRight;
  moveFuncMap_["lookUp"] = &lookUp;
  moveFuncMap_["lookDown"] = &lookDown;

  // TODO Do we need a different function for turnLeft vs. lookLeft?
  // Those should just be body vs. sensor, but should check
  moveFuncMap_["turnLeft"] = &lookLeft;
  moveFuncMap_["turnRight"] = &lookRight;
}

ObjectControls& ObjectControls::setMoveFilterFunction(
    MoveFilterFunc filterFunc) {
  moveFilterFunc_ = filterFunc;
  return *this;
}

ObjectControls& ObjectControls::action(SceneNode& object,
                                       const std::string& actName,
                                       float distance,
                                       bool applyFilter /* = true */) {
  if (moveFuncMap_.count(actName)) {
    if (applyFilter) {
      const vec3f startPosition = object.getAbsolutePosition();
      moveFuncMap_[actName](object, distance);
      const vec3f endPos = object.getAbsolutePosition();
      const vec3f filteredEndPosition = moveFilterFunc_(startPosition, endPos);
      object.translate(filteredEndPosition - endPos);
    } else {
      moveFuncMap_[actName](object, distance);
    }
  } else {
    LOG(ERROR) << "Tried to perform unknown action with name " << actName;
  }

  return *this;
}

}  // namespace scene
}  // namespace esp
