// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CollisionGroupHelper.h"

namespace esp {
namespace physics {

// initialize the default collision group names
std::map<std::string, int> CollisionGroupHelper::collisionGroupNames = {
    {"Default", int(CollisionGroup::Default)},
    {"Static", int(CollisionGroup::Static)},
    {"Kinematic", int(CollisionGroup::Kinematic)},
    {"FreeObject", int(CollisionGroup::FreeObject)},
    {"Robot", int(CollisionGroup::Robot)},
    {"Noncollidable", int(CollisionGroup::Noncollidable)},
    {"UserGroup1", int(CollisionGroup::UserGroup1)},
    {"UserGroup2", int(CollisionGroup::UserGroup2)},
    {"UserGroup3", int(CollisionGroup::UserGroup3)},
    {"UserGroup4", int(CollisionGroup::UserGroup4)},
    {"UserGroup5", int(CollisionGroup::UserGroup5)},
    {"UserGroup6", int(CollisionGroup::UserGroup6)},
    {"UserGroup7", int(CollisionGroup::UserGroup7)},
    {"UserGroup8", int(CollisionGroup::UserGroup8)},
    {"UserGroup9", int(CollisionGroup::UserGroup9)},
    {"AllFilter", int(CollisionGroup::AllFilter)},
};

// initialize the default collision group masks
std::map<CollisionGroup, int> CollisionGroupHelper::collisionGroupMasks = {
    // everything except Noncollidable
    {CollisionGroup::Default, -1 & ~int(CollisionGroup::Noncollidable)},
    // all but Static and Kinematic
    {CollisionGroup::Static, -1 & ~int(CollisionGroup::Kinematic) &
                                 ~int(CollisionGroup::Static) &
                                 ~int(CollisionGroup::Noncollidable)},
    // all but Static and Kinematic
    {CollisionGroup::Kinematic, -1 & ~int(CollisionGroup::Kinematic) &
                                    ~int(CollisionGroup::Static) &
                                    ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::FreeObject, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::Robot, -1 & ~int(CollisionGroup::Noncollidable)},
    // nothing
    {CollisionGroup::Noncollidable, 0},

    // everything except Noncollidable
    {CollisionGroup::UserGroup1, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup2, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup3, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup4, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup5, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup6, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup7, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup8, -1 & ~int(CollisionGroup::Noncollidable)},
    // everything except Noncollidable
    {CollisionGroup::UserGroup9, -1 & ~int(CollisionGroup::Noncollidable)},

    // everything except Noncollidable
    {CollisionGroup::AllFilter, -1 & ~int(CollisionGroup::Noncollidable)},
};

}  // namespace physics
}  // namespace esp
