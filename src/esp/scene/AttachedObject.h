// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include "esp/core/esp.h"

namespace esp {
namespace scene {

class SceneNode;

// Base class, that provides minimal interface for any object (sensor, agent
// etc.) that can be attached to a scene node;

// [SceneNode] <== (two-way pointers) ==> [AttachedObject]

// ONLY AttachedObject is responsible for building/destroying the bridge, the
// two-way pointers illustrated above.

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

class AttachedObject {
 public:
  // constructor
  AttachedObject(AttachedObjectType type = AttachedObjectType::NONE);
  AttachedObject(SceneNode& node,
                 AttachedObjectType type = AttachedObjectType::NONE);

  // destructor
  virtual ~AttachedObject() {
    // when refernce is counted down to 0, no one use this attachment, detach it
    // from the scene node
    detach();
  }

  // attach the object to the scene node
  virtual void attach(SceneNode& node);

  // detach the object from the scene node
  // (make sure detach will be called when the scene node is deconstructed)
  virtual void detach();

  // get the type of the attached object
  AttachedObjectType getObjectType() { return objectType_; }
  void setObjectType(AttachedObjectType type) { objectType_ = type; }

  // get the scene node being attached to (can be nullptr)
  // please use return_policy = reference in pybind
  SceneNode* getSceneNode() { return node_; }

  // object is valid only when it is attached to a scene node.
  // node_ can only be set in functions attach/detach, and the value is either
  // nullptr or an address that is guaranteed accessible
  inline bool isValid() const { return (node_ != nullptr); }

  // ==== get functions ====
  // get local transformation w.r.t. parent's frame
  virtual mat4f getTransformation() const;
  virtual quatf getRotation() const;

  // get global transformation w.r.t. world frame
  virtual vec3f getAbsolutePosition() const;
  virtual mat4f getAbsoluteTransformation() const;

  // ==== set functions ====
  // set local transformation w.r.t. parent's frame
  virtual AttachedObject& setTransformation(
      const Eigen::Ref<const mat4f> transformation);
  virtual AttachedObject& setTransformation(
      const Eigen::Ref<const vec3f> position,
      const Eigen::Ref<const vec3f> target,
      const Eigen::Ref<const vec3f> up);
  virtual AttachedObject& setTranslation(const Eigen::Ref<const vec3f> vector);
  virtual AttachedObject& setRotation(const quatf& quaternion);

  virtual AttachedObject& resetTransformation();

  // ==== rigid body transformations ====
  virtual AttachedObject& translate(const Eigen::Ref<const vec3f> vector);
  virtual AttachedObject& translateLocal(const Eigen::Ref<const vec3f> vector);

  virtual AttachedObject& rotate(float angleInRad,
                                 const Eigen::Ref<const vec3f> normalizedAxis);

  // rotateLocal:
  // It means rotation is applied before all other rotations.

  // Rotate object using axis-angle as a local transformation.
  // normalizedAxis: in parent's frame
  virtual AttachedObject& rotateLocal(
      float angleInRad,
      const Eigen::Ref<const vec3f> normalizedAxis);

  virtual AttachedObject& rotateX(float angleInRad);
  virtual AttachedObject& rotateXInDegree(float angleInDeg);
  virtual AttachedObject& rotateXLocal(float angleInRad);
  virtual AttachedObject& rotateY(float angleInRad);
  virtual AttachedObject& rotateYLocal(float angleInRad);
  virtual AttachedObject& rotateZ(float angleInRad);
  virtual AttachedObject& rotateZLocal(float angleInRad);

 protected:
  // raw pointer, let the scene graph manage the memory
  SceneNode* node_ = nullptr;

  // the type of the attached object (e.g., sensor, agent etc.)
  // no setter is provided since it is an internal variable set by the subclass,
  // not by the user
  AttachedObjectType objectType_ = AttachedObjectType::NONE;

  ESP_SMART_POINTERS(AttachedObject)
};

}  // namespace scene
}  // namespace esp
