// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
        {"Dynamic", CollisionGroup::Dynamic},
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
        {CollisionGroup::Default, ~CollisionGroup::Noncollidable},
        // all but Static and Kinematic
        {CollisionGroup::Static,
         ~(CollisionGroup::Static | CollisionGroup::Kinematic |
           CollisionGroup::Noncollidable)},
        // all but Static and Kinematic
        {CollisionGroup::Kinematic,
         ~(CollisionGroup::Static | CollisionGroup::Kinematic |
           CollisionGroup::Noncollidable)},
        // everything except Noncollidable
        {CollisionGroup::Dynamic, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::Robot, ~CollisionGroup::Noncollidable},
        // nothing
        {CollisionGroup::Noncollidable, CollisionGroups()},

        // everything except Noncollidable
        {CollisionGroup::UserGroup0, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup1, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup2, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup3, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup4, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup5, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup6, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup7, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup8, ~CollisionGroup::Noncollidable},
        // everything except Noncollidable
        {CollisionGroup::UserGroup9, ~CollisionGroup::Noncollidable}};

CollisionGroups CollisionGroupHelper::getMaskForGroup(CollisionGroup group) {
  return collisionGroupMasks.at(group);
}

CollisionGroups CollisionGroupHelper::getMaskForGroup(
    const std::string& groupName) {
  return collisionGroupMasks.at(getGroup(groupName));
}

void CollisionGroupHelper::setGroupInteractsWith(CollisionGroup groupA,
                                                 CollisionGroup groupB,
                                                 bool interacts) {
  CollisionGroups groupAMask = collisionGroupMasks.at(groupA);
  groupAMask = interacts ? groupAMask | groupB : groupAMask & ~groupB;
  collisionGroupMasks.at(groupA) = groupAMask;
}

void CollisionGroupHelper::setMaskForGroup(CollisionGroup group,
                                           CollisionGroups mask) {
  collisionGroupMasks.at(group) = mask;
}

CollisionGroup CollisionGroupHelper::getGroup(const std::string& groupName) {
  auto colGroupNamesIter = collisionGroupNames.find(groupName);
  ESP_CHECK(colGroupNamesIter != collisionGroupNames.end(),
            "Invalid groupName provided. Matches no CollisionGroup.");
  return colGroupNamesIter->second;
}

std::string CollisionGroupHelper::getGroupName(CollisionGroup group) {
  for (auto& it : collisionGroupNames) {
    if (group == it.second) {
      return it.first;
    }
  }
  // enum input, so should not get here unless the map is corrupted
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  return "";
}

bool CollisionGroupHelper::setGroupName(CollisionGroup group,
                                        const std::string& newName) {
  auto currentName = getGroupName(group);
  if (collisionGroupNames.count(newName) != 0) {
    ESP_WARNING() << "Requested group name: " << newName
                  << "is already in use, aborting.";
    return false;
  }
  collisionGroupNames[newName] = collisionGroupNames.at(currentName);
  collisionGroupNames.erase(currentName);
  return true;
}

std::vector<std::string> CollisionGroupHelper::getAllGroupNames() {
  std::vector<std::string> groupNames;
  groupNames.reserve(collisionGroupNames.size());
  for (auto& it : collisionGroupNames) {
    groupNames.push_back(it.first);
  }
  return groupNames;
}

}  // namespace physics
}  // namespace esp
