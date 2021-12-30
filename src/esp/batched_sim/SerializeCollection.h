// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_SERIALIZECOLLECTION_H_
#define ESP_BATCHEDSIM_SERIALIZECOLLECTION_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

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
  std::vector<Sphere> collisionSpheres;
  std::string generateCollisionSpheresTechnique = ""; // "uprightCylinder", "box"
};

struct RobotLink {
  std::string linkName; // from URDF
  std::vector<Sphere> collisionSpheres;
};

struct Robot {
  std::string urdfFilepath;
  std::vector<RobotLink> links;  
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
