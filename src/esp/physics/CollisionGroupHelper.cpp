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
        {"UserGroup1", CollisionGroup::UserGroup1},
        {"UserGroup2", CollisionGroup::UserGroup2},
        {"UserGroup3", CollisionGroup::UserGroup3},
        {"UserGroup4", CollisionGroup::UserGroup4},
        {"UserGroup5", CollisionGroup::UserGroup5},
        {"UserGroup6", CollisionGroup::UserGroup6},
        {"UserGroup7", CollisionGroup::UserGroup7},
        {"UserGroup8", CollisionGroup::UserGroup8},
        {"UserGroup9", CollisionGroup::UserGroup9},
        {"AllFilter", CollisionGroup::AllFilter},
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
