// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/gfx/magnum.h"

// This class provides routines to:
// set and get local rigid body transformation of the current node w.r.t. the
// parent node; get global rigid body transformation

namespace esp {
namespace scene {

class SceneGraph;
class AttachedObject;

class SceneNode : public MagnumObject {
 public:
  // creating a scene node "in the air" is not allowed.
  // it must set an existing node as its parent node.
  // this is to prevent any sub-tree that is "floating in the air", without a
  // terminate node (e.g., "MagnumScene" defined in SceneGraph) as its ancestor
  SceneNode() = delete;
  SceneNode(SceneNode& parent);
  virtual ~SceneNode();

  // CAREFUL:
  // the parameter is a reference,
  // not a pointer, which is different from Magnum library
  virtual SceneNode& setParent(SceneNode& parent);

  //! Create a new child SceneNode and return it. NOTE: this SceneNode owns and
  //! is responsible for deallocating created child
  //! NOTE: child node inherits parent id by default
  SceneNode& createChild();

  //! Returns node id
  virtual int getId() { return id_; }

  //! Sets node id
  virtual void setId(int id) { id_ = id; }

  // ==== get functions ====
  // get local transformation w.r.t. parent's frame
  virtual mat4f getTransformation() const;
  virtual quatf getRotation() const;

  // get global transformation w.r.t. world frame
  virtual vec3f getAbsolutePosition() const;
  virtual mat4f getAbsoluteTransformation() const;

  // ==== set functions ====
  // set local transformation w.r.t. parent's frame
  virtual SceneNode& setTransformation(
      const Eigen::Ref<const mat4f> transformation);
  virtual SceneNode& setTransformation(const Eigen::Ref<const vec3f> position,
                                       const Eigen::Ref<const vec3f> target,
                                       const Eigen::Ref<const vec3f> up);
  virtual SceneNode& setTranslation(const Eigen::Ref<const vec3f> vector);
  virtual SceneNode& setRotation(const quatf& quaternion);

  virtual SceneNode& resetTransformation();

  // ==== rigid body transformations ====
  // SceneNode& transform(const mat4f& transformation);
  // SceneNode& transformLocal(const mat4f& transformation);

  virtual SceneNode& translate(const Eigen::Ref<const vec3f> vector);
  virtual SceneNode& translateLocal(const Eigen::Ref<const vec3f> vector);

  virtual SceneNode& rotate(float angleInRad,
                            const Eigen::Ref<const vec3f> normalizedAxis);

  // rotateLocal:
  // It means rotation is applied before all other rotations.

  // Rotate object using axis-angle as a local transformation.
  // normalizedAxis: in parent's frame
  virtual SceneNode& rotateLocal(float angleInRad,
                                 const Eigen::Ref<const vec3f> normalizedAxis);

  virtual SceneNode& rotateX(float angleInRad);
  virtual SceneNode& rotateXInDegree(float angleInDeg);
  virtual SceneNode& rotateXLocal(float angleInRad);
  virtual SceneNode& rotateY(float angleInRad);
  virtual SceneNode& rotateYLocal(float angleInRad);
  virtual SceneNode& rotateZ(float angleInRad);
  virtual SceneNode& rotateZLocal(float angleInRad);

 protected:
  // why friend class?
  // because it needs to set the attached object directly;

  // then why not specify public set function?
  // because by design, ONLY class 'AttachedObject' can set it, not any other
  // class.

  friend class AttachedObject;

  // no smart pointer, raw pointer only
  // it does not make any sense to let the scene node to take the ownership of
  // the attached object (who creates it, who owns it)

  // no vector, ONE attachment only (if you would like to have multiple attached
  // objects, just create multiple scene nodes, one for each attachment)

  // to connect "AttachedObject" and the "SceneNode", it is done in
  // "AttachedObject" (see attach/detach functions in AttachedObject), not in
  // "SceneNode"; this is to avoid having two functionally equivalent methods in
  // two different classes

  // WARNING: NOT thread-safe.
  // e.g., the scene graph is being deleted in one thread, while the attached
  // object (e.g., a sensor) is being queried in another thread

  AttachedObject* attachedObject_ = nullptr;

  // DO not make the following constructor public!
  // it can ONLY be called from SceneGraph class to initialize the scene graph
  friend class SceneGraph;
  SceneNode(MagnumScene& parentNode);

  int id_ = ID_UNDEFINED;
};

}  // namespace scene
}  // namespace esp
