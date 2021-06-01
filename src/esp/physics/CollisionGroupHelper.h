// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_COLLISIONGROUPHELPER_H_
#define ESP_PHYSICS_COLLISIONGROUPHELPER_H_

#include <Corrade/Utility/Assert.h>
#include <Magnum/Magnum.h>
#include <map>
#include <string>
#include <vector>
#include "esp/core/Check.h"
#include "esp/core/esp.h"

/** @file
 * @brief Class @ref esp::physics::CollisionGroupHelper is a convenience class
 * with all static functions providing an interface for customizing collision
 * masking behavior for simulated objects. Enum Class @ref
 * esp::physics::CollisionGroup defines available collision groups.
 */

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

/**
 * @brief Defined available collision groups with bitwise integer correspondance
 * for use in collision filtering masks.
 *
 * Contains several named default groups used by Habitat-sim and a set of unused
 * user groups for custom behavior.
 */
enum class CollisionGroup {
  //! Default group for unspecified object (e.g. raycast)
  Default = 1,
  //! STATIC objects should not change state
  Static = 1 << 1,
  //! KINEMATIC objects are updated manually
  Kinematic = 1 << 2,
  //! Free (i.e., DYNAMIC) objects are integrated by dynamic simulation
  FreeObject = 1 << 3,
  //! convenience groups to separate robots from free objects
  Robot = 1 << 4,
  //! convenience group for objects which should not collide with anything
  Noncollidable = 1 << 5,

  // User configurable groups with "CollisionGroup::Default" behavior unless
  // modified
  UserGroup1 = 1 << 6,
  UserGroup2 = 1 << 7,
  UserGroup3 = 1 << 8,
  UserGroup4 = 1 << 9,
  UserGroup5 = 1 << 10,
  UserGroup6 = 1 << 11,
  UserGroup7 = 1 << 12,
  UserGroup8 = 1 << 13,
  UserGroup9 = 1 << 14,

  //! convenience group for objects which should collide with everything except
  //! Noncollidable
  AllFilter = -1
};

/**
 * @brief A convenience class with all static functions providing an interface
 * for customizing collision masking behavior for simulated objects.
 */
class CollisionGroupHelper {
  //! maps custom names to collision groups
  static std::map<std::string, CollisionGroup> collisionGroupNames;
  //! maps collision groups to collision group filter masks
  static std::map<CollisionGroup, int> collisionGroupMasks;

 public:
  /**
   * @brief Get the collision mask for a group.
   *
   * @param group The collision group being queried.
   * @return The integer collision mask where each bit corresponds to the
   * group's interaction with another group.
   */
  static int getMaskForGroup(const CollisionGroup& group) {
    return collisionGroupMasks.at(group);
  }
  //! convenience override allowing the group to be referenced by name.
  static int getMaskForGroup(const std::string& groupName) {
    return collisionGroupMasks.at(getGroup(groupName));
  }

  /**
   * @brief Set groupA's collision mask to a specific interaction state with
   * respect to groupB.
   *
   * Convenience function equivalent to setting groupB bit in groupA's mask.
   * @param groupA The group for which the mask will be modified.
   * @param groupB The group which will be masked.
   */
  static void setGroupInteractsWith(const CollisionGroup& groupA,
                                    const CollisionGroup& groupB,
                                    bool interacts) {
    int groupAMask = collisionGroupMasks.at(groupA);
    groupAMask =
        interacts ? groupAMask | int(groupB) : groupAMask & ~int(groupB);
    collisionGroupMasks.at(groupA) = groupAMask;
  };

  /**
   * @brief Set the mask for a group defining which other groups it will
   * interactive with.
   *
   * Mask check logic is two ways: AandBCollide = (GroupA & MaskB) && (GroupB &
   * MaskA) Note: Editing default engine group (Default, Static, Kinematic,
   * FreeObject, Robot, Noncollidable, AllFilter) behavior is discouraged.
   *
   * @param group The group to modify.
   * @param mask The mask to apply.
   */
  static void setMaskForGroup(const CollisionGroup& group, int mask) {
    collisionGroupMasks.at(group) = mask;
  }

  /**
   * @brief Get the collision group by its name. Must pass a valid name.
   *
   * @param groupName The collision group's configured name.
   */
  static CollisionGroup getGroup(const std::string& groupName) {
    ESP_CHECK(collisionGroupNames.count(groupName) != 0,
              "Invalid groupName provided. Matches no CollisionGroup.");
    return collisionGroupNames.at(groupName);
  }

  /**
   * @brief Get a collision group's configured name.
   *
   * @param group The collision group.
   * @return The group's configured name.
   */
  static std::string getGroupName(const CollisionGroup& group) {
    for (std::map<std::string, CollisionGroup>::iterator it =
             collisionGroupNames.begin();
         it != collisionGroupNames.end(); ++it) {
      if (group == it->second) {
        return it->first;
      }
    }
    // enum input, so should not get here unless the map is corrupted
    CORRADE_INTERNAL_ASSERT_UNREACHABLE();
    return "";
  }

  /**
   * @brief Set a custom name for a collision group.
   *
   * Fails if the desired name is already in use.
   *
   * @param group The collision group.
   * @return Whether or not the set was successful.
   */
  static bool setGroupName(const CollisionGroup& group,
                           const std::string& newName) {
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

  /**
   * @brief Get a list of all configured collision group names for easy
   * iteration over groups.
   *
   * @return List of configured group names
   */
  static std::vector<std::string> getAllGroupNames() {
    std::vector<std::string> groupNames(collisionGroupNames.size());
    for (std::map<std::string, CollisionGroup>::iterator it =
             collisionGroupNames.begin();
         it != collisionGroupNames.end(); ++it) {
      groupNames.push_back(it->first);
    }
    return groupNames;
  }

  //! get the mask for group split into boolean indicators of interaction with
  //! each CollisionGroup
  /**
   * @brief Get the collision mask for a group split into a map with entries
   * corresponding to whether a mask bit is set indicating interaction with each
   * other group.
   *
   * @param group The collision group for which the mask will be queried.
   * @return The group mask split into per-group booleans indicating interaction
   * with each group.
   */
  static std::map<CollisionGroup, bool> getSplitMask(
      const CollisionGroup& group) {
    std::map<CollisionGroup, bool> splitMask;
    auto groupMask = getMaskForGroup(group);
    for (std::map<CollisionGroup, int>::iterator it2 =
             collisionGroupMasks.begin();
         it2 != collisionGroupMasks.end(); ++it2) {
      splitMask[it2->first] = (groupMask & int(it2->first)) != 0;
    }
    return splitMask;
  }
  //! convenience override allowing the group to be referenced by name.
  static std::map<CollisionGroup, bool> getSplitMask(
      const std::string& groupName) {
    return getSplitMask(getGroup(groupName));
  }
};
}  // end namespace physics
}  // end namespace esp

#endif  // ESP_PHYSICS_COLLISIONGROUPHELPER_H_
