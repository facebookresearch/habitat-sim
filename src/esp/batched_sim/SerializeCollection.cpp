// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/SerializeCollection.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/io/JsonBuiltinTypes.h"

#include "esp/core/Check.h"
#include "esp/io/json.h"

#include <Corrade/Utility/Directory.h>

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {
namespace serialize {

using esp::io::readMember;

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   Sphere& val) {
  readMember(obj, "origin", val.origin);
  readMember(obj, "radius", val.radius);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   CollisionBox& val) {
  readMember(obj, "min", val.min);
  readMember(obj, "max", val.max);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   FixedObject& val) {
  readMember(obj, "name", val.name);
  readMember(obj, "columnGridFilepaths", val.columnGridFilepaths);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   FreeObject& val) {
  readMember(obj, "name", val.name);
  readMember(obj, "collisionBox", val.collisionBox);
  readMember(obj, "heldRotationIndex", val.heldRotationIndex);
  readMember(obj, "collisionSpheres", val.collisionSpheres);
  readMember(obj, "generateCollisionSpheresTechnique", val.generateCollisionSpheresTechnique);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   RobotGripper& val) {
  readMember(obj, "attachLinkName", val.attachLinkName);
  readMember(obj, "offset", val.offset);
  readMember(obj, "radius", val.radius);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   RobotLink& val) {
  readMember(obj, "linkName", val.linkName);
  readMember(obj, "collisionSpheres", val.collisionSpheres);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   Robot& val) {
  readMember(obj, "urdfFilepath", val.urdfFilepath);
  readMember(obj, "gripper", val.gripper);
  readMember(obj, "links", val.links);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   Collection& val) {
  readMember(obj, "collisionRadiusWorkingSet", val.collisionRadiusWorkingSet);
  readMember(obj, "fixedObjects", val.fixedObjects);
  readMember(obj, "freeObjects", val.freeObjects);
  readMember(obj, "robots", val.robots);
  return true;
}

Collection Collection::loadFromFile(const std::string& filepath) {
  Collection collection;
  ESP_CHECK(Cr::Utility::Directory::exists(filepath), "couldn't find collection file " << filepath);
  auto newDoc = esp::io::parseJsonFile(filepath);
  esp::io::readMember(newDoc, "collection", collection);
  return collection;
}

}

int getCollisionRadiusIndex(const serialize::Collection& collection, float radius) {

  const auto& collisionRadiusWorkingSet = collection.collisionRadiusWorkingSet;
  ESP_CHECK(!collisionRadiusWorkingSet.empty(), 
    "collection.json collisionRadiusWorkingSet is empty");
  auto it = std::find(collisionRadiusWorkingSet.begin(), 
    collisionRadiusWorkingSet.end(), radius);
  ESP_CHECK(it != collisionRadiusWorkingSet.end(), "robot collisionSphere radius " 
    << radius << " not found in collection.json collisionRadiusWorkingSet");
  int radiusIdx = it - collisionRadiusWorkingSet.begin();
  return radiusIdx;
}

float getCollisionRadius(const serialize::Collection& collection, int radiusIdx) {

  const auto& collisionRadiusWorkingSet = collection.collisionRadiusWorkingSet;
  return safeVectorGet(collisionRadiusWorkingSet, radiusIdx);
}

float getMaxCollisionRadius(const serialize::Collection& collection) {

  const auto& collisionRadiusWorkingSet = collection.collisionRadiusWorkingSet;
  const auto it = std::max_element(collisionRadiusWorkingSet.begin(), collisionRadiusWorkingSet.end());
  ESP_CHECK(it != collisionRadiusWorkingSet.end(), "collection.json collisionRadiusWorkingSet is empty");

  return *it;
}

}  // namespace batched_sim
}  // namespace esp
