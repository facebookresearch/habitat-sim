// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"
#include "AttachedObject.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

using namespace Magnum;

namespace esp {
namespace scene {

SceneNode::SceneNode(SceneNode& parent) {
  setParent(parent);
  setId(parent.getId());
}

SceneNode::SceneNode(MagnumScene& parentNode) {
  MagnumObject& baseNode = static_cast<MagnumObject&>(*this);
  // call the setParent in the base class,
  // the parameter is a pointer
  baseNode.setParent(&parentNode);
}

SceneNode::~SceneNode() {
  if (attachedObject_ != nullptr) {
    attachedObject_->detach();
  }
}

SceneNode& SceneNode::setParent(SceneNode& parent) {
  MagnumObject::setParent(&parent);
  return *this;
}

SceneNode& SceneNode::createChild() {
  // will set the parent to *this
  SceneNode* node = new SceneNode(*this);
  node->setId(this->getId());
  return *node;
}

mat4f SceneNode::getTransformation() const {
  return EigenIntegration::cast<mat4f>(MagnumObject::transformation());
}

quatf SceneNode::getRotation() const {
  return quatf(MagnumObject::rotation());
}

mat4f SceneNode::getAbsoluteTransformation() const {
  return EigenIntegration::cast<mat4f>(MagnumObject::absoluteTransformation());
}

vec3f SceneNode::getAbsolutePosition() const {
  return getAbsoluteTransformation().col(3).head(3);
}

SceneNode& SceneNode::setTransformation(
    const Eigen::Ref<const mat4f> transformMat) {
  MagnumObject::setTransformation(Matrix4(transformMat));
  return *this;
}

SceneNode& SceneNode::setTransformation(const Eigen::Ref<const vec3f> position,
                                        const Eigen::Ref<const vec3f> target,
                                        const Eigen::Ref<const vec3f> up) {
  MagnumObject::setTransformation(
      Matrix4::lookAt(Vector3(position), Vector3(target), Vector3(up)));
  return *this;
}

SceneNode& SceneNode::setRotation(const quatf& q) {
  MagnumObject::setRotation(Quaternion(q).normalized());
  return *this;
}

SceneNode& SceneNode::setTranslation(const Eigen::Ref<const vec3f> vector) {
  MagnumObject::setTranslation(Vector3(vector));
  return *this;
}

SceneNode& SceneNode::resetTransformation() {
  MagnumObject::resetTransformation();
  return *this;
}

// SceneNode& SceneNode::transform(const mat4f& transformMat) {
//   MagnumObject::transform(Matrix4(transformMat));
//   return *this;
// }

// SceneNode& SceneNode::transformLocal(const mat4f& transformMat) {
//   MagnumObject::transformLocal(Matrix4(transformMat));
//   return *this;
// }

SceneNode& SceneNode::translate(const Eigen::Ref<const vec3f> vector) {
  MagnumObject::translate(Vector3(vector));
  return *this;
}

SceneNode& SceneNode::translateLocal(const Eigen::Ref<const vec3f> vector) {
  MagnumObject::translateLocal(Vector3(vector));
  return *this;
}

SceneNode& SceneNode::rotate(float angleInRad,
                             const Eigen::Ref<const vec3f> normalizedAxis) {
  MagnumObject::rotate(Rad{angleInRad}, Vector3(normalizedAxis));
  return *this;
}

SceneNode& SceneNode::rotateLocal(
    float angleInRad,
    const Eigen::Ref<const vec3f> normalizedAxis) {
  MagnumObject::rotateLocal(Rad{angleInRad}, Vector3(normalizedAxis));
  return *this;
}

SceneNode& SceneNode::rotateXInDegree(float angleInDeg) {
  MagnumObject::rotateX(Deg{angleInDeg});
  return *this;
}

SceneNode& SceneNode::rotateX(float angleInRad) {
  MagnumObject::rotateX(Rad{angleInRad});
  return *this;
}

SceneNode& SceneNode::rotateXLocal(float angleInRad) {
  MagnumObject::rotateXLocal(Rad{angleInRad});
  return *this;
}

SceneNode& SceneNode::rotateY(float angleInRad) {
  MagnumObject::rotateY(Rad{angleInRad});
  return *this;
}

SceneNode& SceneNode::rotateYLocal(float angleInRad) {
  MagnumObject::rotateYLocal(Rad{angleInRad});
  return *this;
}

SceneNode& SceneNode::rotateZ(float angleInRad) {
  MagnumObject::rotateZ(Rad{angleInRad});
  return *this;
}

SceneNode& SceneNode::rotateZLocal(float angleInRad) {
  MagnumObject::rotateZLocal(Rad{angleInRad});
  return *this;
}

}  // namespace scene
}  // namespace esp
