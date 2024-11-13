// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETARTICULATEDLINK_H_
#define ESP_PHYSICS_BULLET_BULLETARTICULATEDLINK_H_

#include "../ArticulatedLink.h"
#include "BulletBase.h"
#include "objectWrappers/ManagedBulletArticulatedObject.h"

namespace esp {

namespace physics {

class ManagedBulletArticulatedObject;

////////////////////////////////////
// Link
////////////////////////////////////

class BulletArticulatedLink : public ArticulatedLink, public BulletBase {
 public:
  BulletArticulatedLink(scene::SceneNode* bodyNode,
                        const assets::ResourceManager& resMgr,
                        std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
                        int index,
                        std::shared_ptr<std::map<const btCollisionObject*, int>>
                            collisionObjToObjIds)
      : ArticulatedLink(bodyNode, index, resMgr),
        BulletBase(std::move(bWorld), std::move(collisionObjToObjIds)) {}

  Magnum::Range3D getCollisionShapeAabb() const override {
    // TODO: collision object should be linked here
    ESP_WARNING() << "Not implemented.";
    return Magnum::Range3D();
  }

  //! link can't do this.
  void setMotionType(CORRADE_UNUSED MotionType mt) override {
    ESP_WARNING() << "Cannot set MotionType individually for links.";
  }

  std::shared_ptr<ManagedBulletArticulatedObject> getOwningManagedBulletAO()
      const {
    return ArticulatedLink::getOwningManagedAOInternal<
        ManagedBulletArticulatedObject>();
  }

 protected:
  int mbIndex_;

 private:
  ESP_SMART_POINTERS(BulletArticulatedLink)
};
}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETARTICULATEDLINK_H_
