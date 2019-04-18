// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AttachedObject.h"
#include "SceneNode.h"

namespace esp {
namespace scene {

AttachedObject::AttachedObject(AttachedObjectType type) : objectType_(type) {}
AttachedObject::AttachedObject(SceneNode& node, AttachedObjectType type)
    : objectType_(type) {
  // since a node is passed, attach it
  attach(node);
}

// detach the object from the scene graph
void AttachedObject::detach() {
  // if the attaching scene node is destroyed earlier, it will inform the
  // attachedObject before it was deconstructed. This must be guaranteed in the
  // destructor of "SceneNode";

  // the above statement also implies that as long as node_ is NOT nullptr,
  // it points to an address that is guaranteed accessible.

  // so if the attached object is destroyed first, it is *safe* to call
  // node_->attachedObject_

  if (node_ != nullptr) {
    node_->attachedObject_ =
        nullptr;      // break the connection (SceneNode --> AttachedObject)
    node_ = nullptr;  // break the connection (SceneNode <-- AttachedObject)
  }
}

void AttachedObject::attach(SceneNode& node) {
  node_ = &node;  // build the connection (SceneNode <-- AttachedObject)
  node_->attachedObject_ =
      this;  // build the connection (SceneNode --> AttachedObject)
}

// ==== get functions ====
// get local transformation w.r.t. parent's frame
mat4f AttachedObject::getTransformation() const {
  ASSERT(isValid());
  return node_->getTransformation();
}
quatf AttachedObject::getRotation() const {
  ASSERT(isValid());
  return node_->getRotation();
}

// get global transformation w.r.t. world frame
vec3f AttachedObject::getAbsolutePosition() const {
  ASSERT(isValid());
  return node_->getAbsolutePosition();
}

mat4f AttachedObject::getAbsoluteTransformation() const {
  ASSERT(isValid());
  return node_->getAbsoluteTransformation();
}

// ==== set functions ====
// set local transformation w.r.t. parent's frame
AttachedObject& AttachedObject::setTransformation(
    const Eigen::Ref<const mat4f> transformation) {
  ASSERT(isValid());
  node_->setTransformation(transformation);
  return *this;
}

AttachedObject& AttachedObject::setTransformation(
    const Eigen::Ref<const vec3f> position,
    const Eigen::Ref<const vec3f> target,
    const Eigen::Ref<const vec3f> up) {
  ASSERT(isValid());
  node_->setTransformation(position, target, up);
  return *this;
}

AttachedObject& AttachedObject::setTranslation(
    const Eigen::Ref<const vec3f> vector) {
  ASSERT(isValid());
  node_->setTranslation(vector);
  return *this;
}

AttachedObject& AttachedObject::setRotation(const quatf& quaternion) {
  ASSERT(isValid());
  node_->setRotation(quaternion);
  return *this;
}

AttachedObject& AttachedObject::resetTransformation() {
  ASSERT(isValid());
  node_->resetTransformation();
  return *this;
}

// ==== rigid body transformations ====
AttachedObject& AttachedObject::translate(
    const Eigen::Ref<const vec3f> vector) {
  ASSERT(isValid());
  node_->translate(vector);
  return *this;
}

AttachedObject& AttachedObject::translateLocal(
    const Eigen::Ref<const vec3f> vector) {
  ASSERT(isValid());
  node_->translateLocal(vector);
  return *this;
}

AttachedObject& AttachedObject::rotate(
    float angleInRad,
    const Eigen::Ref<const vec3f> normalizedAxis) {
  ASSERT(isValid());
  node_->rotate(angleInRad, normalizedAxis);
  return *this;
}

// rotateLocal:
// It means rotation is applied before all other rotations.

// Rotate object using axis-angle as a local transformation.
// normalizedAxis: in parent's frame
AttachedObject& AttachedObject::rotateLocal(
    float angleInRad,
    const Eigen::Ref<const vec3f> normalizedAxis) {
  ASSERT(isValid());
  node_->rotateLocal(angleInRad, normalizedAxis);
  return *this;
}

AttachedObject& AttachedObject::rotateX(float angleInRad) {
  ASSERT(isValid());
  node_->rotateX(angleInRad);
  return *this;
}

AttachedObject& AttachedObject::rotateXInDegree(float angleInDeg) {
  ASSERT(isValid());
  node_->rotateXInDegree(angleInDeg);
  return *this;
}

AttachedObject& AttachedObject::rotateXLocal(float angleInRad) {
  ASSERT(isValid());
  node_->rotateXLocal(angleInRad);
  return *this;
}

AttachedObject& AttachedObject::rotateY(float angleInRad) {
  ASSERT(isValid());
  node_->rotateY(angleInRad);
  return *this;
}

AttachedObject& AttachedObject::rotateYLocal(float angleInRad) {
  ASSERT(isValid());
  node_->rotateYLocal(angleInRad);
  return *this;
}

AttachedObject& AttachedObject::rotateZ(float angleInRad) {
  ASSERT(isValid());
  node_->rotateZ(angleInRad);
  return *this;
}

AttachedObject& AttachedObject::rotateZLocal(float angleInRad) {
  ASSERT(isValid());
  node_->rotateZLocal(angleInRad);
  return *this;
}

}  // namespace scene
}  // namespace esp
