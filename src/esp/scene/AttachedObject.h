// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/SceneGraph/AbstractFeature.h>

#include "esp/core/esp.h"

namespace esp {
namespace scene {

class SceneNode;

// Base class, that provides minimal interface for any object (sensor, agent
// etc.) that can be attached to a scene node;

// it is user's responsibility to create new scene nodes, to which the objects
// are attached.
// this class will NOT create any new scene node
// it only attaches the object to an existing node provided by the user

// object type needs to be added only when the object class is *directly*
// derived from the base class
// (e.g., you do not need to specify PINHOLE_CAMERA_SENSOR here since it
//  is a sub-type in the SENSOR category)

// Future types may include e.g., "LIGHT"
enum class AttachedObjectType {
  NONE = 0,
  SENSOR = 1,
  AGENT = 2,
  CAMERA = 3,
};

class AttachedObject : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  // constructor
  AttachedObject(SceneNode& node,
                 AttachedObjectType type = AttachedObjectType::NONE);

  // get the type of the attached object
  AttachedObjectType getObjectType() { return objectType_; }
  void setObjectType(AttachedObjectType type) { objectType_ = type; }

  // Get the scene node being attached to. Expects the object is valid, call
  // isValid() before to be sure. For consistency this is named the same as
  // Magnum::SceneGraph::object(), only returning a derived type.
  SceneNode& object();
  const SceneNode& object() const;

 protected:
  // the type of the attached object (e.g., sensor, agent etc.)
  // no setter is provided since it is an internal variable set by the subclass,
  // not by the user
  AttachedObjectType objectType_ = AttachedObjectType::NONE;

  ESP_SMART_POINTERS(AttachedObject)
};

}  // namespace scene
}  // namespace esp
