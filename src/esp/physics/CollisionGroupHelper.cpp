// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CollisionGroupHelper.h"

namespace esp {
namespace physics {

// initialize the default collision group names
std::map<std::string, CollisionGroup>
    CollisionGroupHelper::collisionGroupNames = {
        {"Default", CollisionGroup::Default},
        {"Static", CollisionGroup::Static},
        {"Kinematic", CollisionGroup::Kinematic},
        {"FreeObject", CollisionGroup::FreeObject},
        {"Robot", CollisionGroup::Robot},
        {"Noncollidable", CollisionGroup::Noncollidable},
        {"UserGroup0", CollisionGroup::UserGroup0},
        {"UserGroup1", CollisionGroup::UserGroup1},
        {"UserGroup2", CollisionGroup::UserGroup2},
        {"UserGroup3", CollisionGroup::UserGroup3},
        {"UserGroup4", CollisionGroup::UserGroup4},
        {"UserGroup5", CollisionGroup::UserGroup5},
        {"UserGroup6", CollisionGroup::UserGroup6},
        {"UserGroup7", CollisionGroup::UserGroup7},
        {"UserGroup8", CollisionGroup::UserGroup8},
        {"UserGroup9", CollisionGroup::UserGroup9},
};

// initialize the default collision group masks
std::map<CollisionGroup, CollisionGroups>
    CollisionGroupHelper::collisionGroupMasks = {
        // everything except Noncollidable
        {CollisionGroup::Default,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // all but Static and Kinematic
        {CollisionGroup::Static,
         ~CollisionGroups(CollisionGroup::Static) &
             ~CollisionGroups(CollisionGroup::Kinematic) &
             ~CollisionGroups(CollisionGroup::Noncollidable)},
        // all but Static and Kinematic
        {CollisionGroup::Kinematic,
         ~CollisionGroups(CollisionGroup::Static) &
             ~CollisionGroups(CollisionGroup::Kinematic) &
             ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::FreeObject,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::Robot,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // nothing
        {CollisionGroup::Noncollidable, ~CollisionGroups()},

        // everything except Noncollidable
        {CollisionGroup::UserGroup0,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup1,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup2,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup3,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup4,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup5,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup6,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup7,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup8,
         ~CollisionGroups(CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::UserGroup9,
         ~CollisionGroups(CollisionGroup::Noncollidable)}};

}  // namespace physics
}  // namespace esp
