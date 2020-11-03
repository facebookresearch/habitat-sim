// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectControls.h"

#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

#include "SceneNode.h"
#include "esp/core/esp.h"

using Magnum::EigenIntegration::cast;

namespace esp {
namespace scene {

auto moveRight(SceneNode& object, float distance) -> SceneNode& {
  // TODO: this assumes no scale is applied
  object.translateLocal(object.transformation().right() * distance);
  return object;
}

auto moveLeft(SceneNode& object, float distance) -> SceneNode& {
  return moveRight(object, -distance);
}

auto moveUp(SceneNode& object, float distance) -> SceneNode& {
  // TODO: this assumes no scale is applied
  object.translateLocal(object.transformation().up() * distance);
  return object;
}

auto moveDown(SceneNode& object, float distance) -> SceneNode& {
  return moveUp(object, -distance);
}

auto moveBackward(SceneNode& object, float distance) -> SceneNode& {
  // TODO: this assumes no scale is applied
  object.translateLocal(object.transformation().backward() * distance);
  return object;
}

auto moveForward(SceneNode& object, float distance) -> SceneNode& {
  return moveBackward(object, -distance);
}

auto turnLeft(SceneNode& object, float angleInDegrees) -> SceneNode& {
  object.rotateYLocal(Magnum::Deg(angleInDegrees));
  object.setRotation(object.rotation().normalized());
  return object;
}

auto turnRight(SceneNode& object, float angleInDegrees) -> SceneNode& {
  return turnLeft(object, -angleInDegrees);
}

auto lookUp(SceneNode& object, float angleInDegrees) -> SceneNode& {
  object.rotateXLocal(Magnum::Deg(angleInDegrees));
  object.setRotation(object.rotation().normalized());
  return object;
}

auto lookDown(SceneNode& object, float angleInDegrees) -> SceneNode& {
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

auto ObjectControls::setMoveFilterFunction(MoveFilterFunc filterFunc)
    -> ObjectControls& {
  moveFilterFunc_ = std::move(filterFunc);
  return *this;
}

auto ObjectControls::action(SceneNode& object,
                            const std::string& actName,
                            float distance,
                            bool applyFilter /* = true */) -> ObjectControls& {
  if (moveFuncMap_.count(actName)) {
    if (applyFilter) {
      // TODO: use magnum math for the filter func as well?
      const auto startPosition =
          cast<vec3f>(object.absoluteTransformation().translation());
      moveFuncMap_[actName](object, distance);
      const auto endPos =
          cast<vec3f>(object.absoluteTransformation().translation());
      const vec3f filteredEndPosition = moveFilterFunc_(startPosition, endPos);
      object.translate(Magnum::Vector3(vec3f(filteredEndPosition - endPos)));
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
