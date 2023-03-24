// Copyright (c) Meta Platforms, Inc. All Rights Reserved
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_SERIALIZECOLLECTION_H_
#define ESP_BATCHEDSIM_SERIALIZECOLLECTION_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Tags.h>

#include <string>
#include <vector>

namespace esp {
namespace batched_sim {
namespace serialize {

struct Sphere {
  Magnum::Vector3 origin;
  float radius;
};

struct CollisionBox {
  Magnum::Vector3 min;
  Magnum::Vector3 max;
};

struct FixedObject {
  std::string name; // this is also the render asset name
  std::vector<std::string> columnGridFilepaths;
};

struct FreeObject {
  std::string name; // this is also the render asset name
  CollisionBox collisionBox;
  int heldRotationIndex = 0; // temp hack index into startRotations
  std::vector<Sphere> collisionSpheres;
  std::string generateCollisionSpheresTechnique = ""; // "uprightCylinder", "box"
};

struct ContinuousActionSetup {
  int actionIdx = -1;
  float stepMin = -1.f;
  float stepMax = 1.f;
};

struct DiscreteActionSetup {
  int actionIdx = -1;
  std::vector<float> thresholds;
};

struct ActionMap {
  int numActions = -1;
  ContinuousActionSetup baseMove;
  ContinuousActionSetup baseRotate;
  DiscreteActionSetup graspRelease;
  std::vector<std::pair<int,ContinuousActionSetup>> joints;
};

struct RobotGripper {
  std::string attachLinkName;
  Magnum::Vector3 offset{Magnum::Math::ZeroInit};
  float radius = 0.f;
};

struct RobotLink {
  std::string linkName; // from URDF
  std::vector<Sphere> collisionSpheres;
};

struct Robot {
  std::string urdfFilepath;
  std::vector<float> startJointPositions;
  RobotGripper gripper;
  std::vector<RobotLink> links;
  ActionMap actionMap;
};

struct Collection {
  std::vector<float> collisionRadiusWorkingSet;
  std::vector<FreeObject> freeObjects;
  std::vector<FixedObject> fixedObjects;
  std::vector<Robot> robots;

  static Collection loadFromFile(const std::string& filepath);
};

}

// sloppy: put these somewhere else
int getCollisionRadiusIndex(const serialize::Collection& collection, float radius);
float getCollisionRadius(const serialize::Collection& collection, int radiusIdx);
float getMaxCollisionRadius(const serialize::Collection& collection);

}  // namespace batched_sim
}  // namespace esp

#endif
