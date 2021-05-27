// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_COLLISIONGROUPHELPER_H_
#define ESP_PHYSICS_COLLISIONGROUPHELPER_H_

#include <Corrade/Utility/Assert.h>
#include <map>
#include <string>
#include <vector>
#include "esp/core/Check.h"
#include "esp/core/esp.h"

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
  //! Default group for unspecified object (e.g. raycast)
  Default = 1,
  //! STATIC objects should not change state
  Static = 2,
  //! KINEMATIC objects are updated manually
  Kinematic = 4,
  //! Free (i.e., DYNAMIC) objects are integrated by dynamic simulation
  FreeObject = 8,
  //! convenience groups to separate robots from free objects
  Robot = 16,
  //! convenience group for objects which should not collide with anything
  Noncollidable = 32,

  // User configurable groups with "CollisionGroup::Default" behavior unless
  // modified
  UserGroup1 = 64,
  UserGroup2 = 128,
  UserGroup3 = 256,
  UserGroup4 = 512,
  UserGroup5 = 1024,
  UserGroup6 = 2048,
  UserGroup7 = 4096,
  UserGroup8 = 8192,
  UserGroup9 = 16384,

  //! convenience group for objects which should collide with everything except
  //! Noncollidable
  AllFilter = -1
};

class CollisionGroupHelper {
  //! maps custom names to collision groups
  static std::map<std::string, int> collisionGroupNames;
  // maps collision groups to collision group filter masks
  static std::map<CollisionGroup, int> collisionGroupMasks;

 public:
  //! get the mask for a group
  static int getMaskForGroup(CollisionGroup group) {
    return collisionGroupMasks.at(group);
  }

  /**
   * @brief Set the mask for a group defining which other groups it will
   * interactive with.
   *
   * Mask check logic is two ways: AandBCollide = (GroupA & MaskB) && (GroupB &
   * MaskA) Note: Editing default engine group (Default, Static, Kinematic,
   * FreeObject, Robot, Noncollidable, AllFilter) behavior is discouraged.
   *
   * @param group The group to modify
   * @param mask The mask to apply
   */
  static void setMaskForGroup(CollisionGroup group, int mask) {
    collisionGroupMasks.at(group) = mask;
  }

  //! return the group given the name
  static CollisionGroup getGroup(std::string groupName) {
    ESP_CHECK(collisionGroupNames.count(groupName) != 0,
              "Invalid groupName provided. Matches no CollisionGroup.");
    return CollisionGroup(collisionGroupNames.at(groupName));
  }

  // return the group name given the group
  static std::string getGroupName(CollisionGroup group) {
    for (std::map<std::string, int>::iterator it = collisionGroupNames.begin();
         it != collisionGroupNames.end(); ++it) {
      if (int(group) == it->second) {
        return it->first;
      }
    }
    // enum input, so should not get here unless the map is corrupted
    CORRADE_INTERNAL_ASSERT_UNREACHABLE();
    return "";
  }

  //! query all available groups (by name)
  static std::vector<std::string> getAllGroupNames() {
    std::vector<std::string> groupNames(collisionGroupNames.size());
    for (std::map<std::string, int>::iterator it = collisionGroupNames.begin();
         it != collisionGroupNames.end(); ++it) {
      groupNames.push_back(it->first);
    }
    return groupNames;
  }

  static bool setGroupName(CollisionGroup group, std::string newName) {
    auto currentName = getGroupName(group);
    if (collisionGroupNames.count(newName) != 0) {
      LOG(WARNING) << "CollisionGroupHelper::setGroupName - requested group "
                      "name is already in use, aborting.";
      return false;
    }
    collisionGroupNames[newName] = collisionGroupNames.at(currentName);
    collisionGroupNames.erase(currentName);
    return true;
  }
};

}  // end namespace physics
}  // end namespace esp

#endif  // ESP_PHYSICS_COLLISIONGROUPHELPER_H_
