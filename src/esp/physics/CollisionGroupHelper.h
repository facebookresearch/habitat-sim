// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

namespace esp {
namespace physics {

/*
  For reference, here are the standard Bullet groups. It's probably good to
  stay somewhat consistent with them, in case users are already using them.
        enum CollisionFilterGroups
        {
                DefaultFilter = 1,
                StaticFilter = 2,
                KinematicFilter = 4,
                DebrisFilter = 8,
                SensorTrigger = 16,
                CharacterFilter = 32,
                AllFilter = -1  //all bits sets: DefaultFilter | StaticFilter |
  KinematicFilter | DebrisFilter | SensorTrigger
        };
*/

enum class CollisionGroup {
  Default = 1,
  Static = 2,
  Kinematic = 4,
  FreeObject = 8,
  GraspedObject = 16,
  Robot = 32,
  EeMargin = 64,
  SelObj = 128,
  Noncollidable = 256
};

class CollisionGroupHelper {
 public:
  static int getMaskForGroup(CollisionGroup group);
};

}  // end namespace physics
}  // end namespace esp
