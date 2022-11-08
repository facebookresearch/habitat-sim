// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_COLLISIONGROUPHELPER_H_
#define ESP_PHYSICS_COLLISIONGROUPHELPER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Assert.h>
#include <Magnum/Magnum.h>
#include <map>
#include <string>
#include <vector>
#include "esp/core/Check.h"
#include "esp/core/Esp.h"

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
enum class CollisionGroup : unsigned int {
  //! Default group for unspecified object (e.g. raycast)
  Default = 1,
  //! Static objects should not change state
  Static = 1 << 1,
  //! Kinematic objects are updated manually
  Kinematic = 1 << 2,
  //! Dynamic objects are integrated by dynamic simulation
  Dynamic = 1 << 3,
  //! convenience groups to separate robots from free objects
  Robot = 1 << 4,
  //! convenience group for objects which should not collide with anything
  Noncollidable = 1 << 5,

  // User configurable groups with "CollisionGroup::Default" behavior unless
  // modified
  UserGroup0 = 1 << 6,
  UserGroup1 = 1 << 7,
  UserGroup2 = 1 << 8,
  UserGroup3 = 1 << 9,
  UserGroup4 = 1 << 10,
  UserGroup5 = 1 << 11,
  UserGroup6 = 1 << 12,
  UserGroup7 = 1 << 13,
  UserGroup8 = 1 << 14,
  UserGroup9 = 1 << 15,
};

typedef Corrade::Containers::EnumSet<CollisionGroup> CollisionGroups;
CORRADE_ENUMSET_OPERATORS(CollisionGroups)

/**
 * @brief A convenience class with all static functions providing an interface
 * for customizing collision masking behavior for simulated objects.
 */
class CollisionGroupHelper {
  //! maps custom names to collision groups
  static std::map<std::string, CollisionGroup> collisionGroupNames;
  //! maps collision groups to collision group filter masks
  static std::map<CollisionGroup, CollisionGroups> collisionGroupMasks;

 public:
  /**
   * @brief Get the collision mask for a group.
   *
   * @param group The collision group being queried.
   * @return The integer collision mask where each bit corresponds to the
   * group's interaction with another group.
   */
  static CollisionGroups getMaskForGroup(CollisionGroup group);
  //! convenience override allowing the group to be referenced by name.
  static CollisionGroups getMaskForGroup(const std::string& groupName);

  /**
   * @brief Set groupA's collision mask to a specific interaction state with
   * respect to groupB.
   *
   * Convenience function equivalent to setting groupB bit in groupA's mask.
   * @param groupA The group for which the mask will be modified.
   * @param groupB The group which will be masked.
   */
  static void setGroupInteractsWith(CollisionGroup groupA,
                                    CollisionGroup groupB,
                                    bool interacts);

  /**
   * @brief Set the mask for a group defining which other groups it will
   * interactive with.
   *
   * Mask check logic is two ways: AandBCollide = (GroupA & MaskB) && (GroupB &
   * MaskA) Note: Editing default engine group (Default, Static, Kinematic,
   * Dynamic, Robot, Noncollidable, AllFilter) behavior is discouraged.
   *
   * @param group The group to modify.
   * @param mask The mask to apply.
   */
  static void setMaskForGroup(CollisionGroup group, CollisionGroups mask);

  /**
   * @brief Get the collision group by its name. Must pass a valid name.
   *
   * @param groupName The collision group's configured name.
   */
  static CollisionGroup getGroup(const std::string& groupName);

  /**
   * @brief Get a collision group's configured name.
   *
   * @param group The collision group.
   * @return The group's configured name.
   */
  static std::string getGroupName(CollisionGroup group);

  /**
   * @brief Set a custom name for a collision group.
   *
   * Fails if the desired name is already in use.
   *
   * @param group The collision group.
   * @return Whether or not the set was successful.
   */
  static bool setGroupName(CollisionGroup group, const std::string& newName);

  /**
   * @brief Get a list of all configured collision group names for easy
   * iteration over groups.
   *
   * @return List of configured group names
   */
  static std::vector<std::string> getAllGroupNames();

};  // end CollisionGroupHelper

}  // end namespace physics
}  // end namespace esp

#endif  // ESP_PHYSICS_COLLISIONGROUPHELPER_H_
