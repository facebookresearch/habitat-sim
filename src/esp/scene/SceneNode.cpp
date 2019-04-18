// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"
#include "AttachedObject.h"

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
  return Eigen::Map<mat4f>(MagnumObject::transformation().data());
}

quatf SceneNode::getRotation() const {
  const auto& tmp = MagnumObject::rotation();
  return quatf(tmp.scalar(), tmp.vector().x(), tmp.vector().y(),
               tmp.vector().z());
}

mat4f SceneNode::getAbsoluteTransformation() const {
  return Eigen::Map<mat4f>(MagnumObject::absoluteTransformation().data());
}

vec3f SceneNode::getAbsolutePosition() const {
  return getAbsoluteTransformation().col(3).head(3);
}

SceneNode& SceneNode::setTransformation(
    const Eigen::Ref<const mat4f> transformMat) {
  const auto magnumMat = Math::Matrix4<Float>::from(transformMat.data());
  MagnumObject::setTransformation(magnumMat);
  return *this;
}

SceneNode& SceneNode::setTransformation(const Eigen::Ref<const vec3f> position,
                                        const Eigen::Ref<const vec3f> target,
                                        const Eigen::Ref<const vec3f> up) {
  MagnumObject::setTransformation(
      Matrix4::lookAt(Vector3::from(position.data()),
                      Vector3::from(target.data()), Vector3::from(up.data())));
  return *this;
}

SceneNode& SceneNode::setRotation(const quatf& q) {
  // TODO: this is a kludge, ideally we'd use a map-like from()
  const Math::Quaternion<Float> mq(Vector3(q.x(), q.y(), q.z()), q.w());
  MagnumObject::setRotation(mq.normalized());
  return *this;
}

SceneNode& SceneNode::setTranslation(const Eigen::Ref<const vec3f> vector) {
  const auto magnumVec = Math::Vector3<Float>::from(vector.data());
  MagnumObject::setTranslation(magnumVec);
  return *this;
}

SceneNode& SceneNode::resetTransformation() {
  MagnumObject::resetTransformation();
  return *this;
}

// SceneNode& SceneNode::transform(const mat4f& transformMat) {
//   const auto magnumMat = Math::Matrix4<Float>::from(transformMat.data());
//   MagnumObject::transform(magnumMat);
//   return *this;
// }

// SceneNode& SceneNode::transformLocal(const mat4f& transformMat) {
//   const auto magnumMat = Math::Matrix4<Float>::from(transformMat.data());
//   MagnumObject::transformLocal(magnumMat);
//   return *this;
// }

SceneNode& SceneNode::translate(const Eigen::Ref<const vec3f> vector) {
  const auto magnumVec = Math::Vector3<Float>::from(vector.data());
  MagnumObject::translate(magnumVec);
  return *this;
}

SceneNode& SceneNode::translateLocal(const Eigen::Ref<const vec3f> vector) {
  const auto magnumVec = Math::Vector3<Float>::from(vector.data());
  MagnumObject::translateLocal(magnumVec);
  return *this;
}

SceneNode& SceneNode::rotate(float angleInRad,
                             const Eigen::Ref<const vec3f> normalizedAxis) {
  const auto magnumVec = Math::Vector3<Float>::from(normalizedAxis.data());
  MagnumObject::rotate(Rad{angleInRad}, magnumVec);
  return *this;
}

SceneNode& SceneNode::rotateLocal(
    float angleInRad,
    const Eigen::Ref<const vec3f> normalizedAxis) {
  const auto magnumVec = Math::Vector3<Float>::from(normalizedAxis.data());
  MagnumObject::rotateLocal(Rad{angleInRad}, magnumVec);
  return *this;
}

SceneNode& SceneNode::rotateXInDegree(float angleInDeg) {
  Deg angle{angleInDeg};
  MagnumObject::rotateX(Rad{angle});
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
