// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Attributes.h"
#include <algorithm>

namespace esp {
namespace assets {

Attributes::Attributes() {
  doubleMap_ = std::map<std::string, double>();
  intMap_ = std::map<std::string, int>();
  boolMap_ = std::map<std::string, bool>();
  stringMap_ = std::map<std::string, std::string>();
  magnumVec3Map_ = std::map<std::string, Magnum::Vector3>();
  vecStringsMap_ = std::map<std::string, std::vector<std::string> >();
}

// return true if any container has the key
bool Attributes::exists(const std::string& key) const {
  if (doubleMap_.count(key) > 0)
    return true;
  if (intMap_.count(key) > 0)
    return true;
  if (boolMap_.count(key) > 0)
    return true;
  if (stringMap_.count(key) > 0)
    return true;
  if (magnumVec3Map_.count(key) > 0)
    return true;
  if (vecStringsMap_.count(key) > 0)
    return true;

  return false;
}

// check if an attribute of a specific type exists
bool Attributes::existsAs(DataType t, const std::string& key) const {
  if (t == DOUBLE)
    if (doubleMap_.count(key) > 0)
      return true;
  if (t == INT)
    if (intMap_.count(key) > 0)
      return true;
  if (t == BOOL)
    if (boolMap_.count(key) > 0)
      return true;
  if (t == STRING)
    if (stringMap_.count(key) > 0)
      return true;
  if (t == MAGNUMVEC3)
    if (magnumVec3Map_.count(key) > 0)
      return true;
  if (t == VEC_STRINGS)
    if (vecStringsMap_.count(key) > 0)
      return true;
  return false;
}

// count the number of containers with the key
int Attributes::count(const std::string& key) const {
  int numAttributes = 0;
  if (doubleMap_.count(key) > 0)
    numAttributes++;
  if (intMap_.count(key) > 0)
    numAttributes++;
  if (boolMap_.count(key) > 0)
    numAttributes++;
  if (stringMap_.count(key) > 0)
    numAttributes++;
  if (magnumVec3Map_.count(key) > 0)
    numAttributes++;
  if (vecStringsMap_.count(key) > 0)
    numAttributes++;
  return numAttributes;
}

// erase the key from all maps
void Attributes::eraseAll(const std::string& key) {
  if (doubleMap_.count(key) > 0)
    doubleMap_.erase(key);
  if (intMap_.count(key) > 0)
    intMap_.erase(key);
  if (boolMap_.count(key) > 0)
    boolMap_.erase(key);
  if (stringMap_.count(key) > 0)
    stringMap_.erase(key);
  if (magnumVec3Map_.count(key) > 0)
    magnumVec3Map_.erase(key);
  if (vecStringsMap_.count(key) > 0)
    vecStringsMap_.erase(key);
}

// erase the key from a particular map
void Attributes::eraseAs(const DataType t, const std::string& key) {
  if (t == DOUBLE) {
    if (doubleMap_.count(key) > 0)
      doubleMap_.erase(key);
  } else if (t == INT) {
    if (intMap_.count(key) > 0)
      intMap_.erase(key);
  } else if (t == BOOL) {
    if (boolMap_.count(key) > 0)
      boolMap_.erase(key);
  } else if (t == STRING) {
    if (stringMap_.count(key) > 0)
      stringMap_.erase(key);
  } else if (t == MAGNUMVEC3) {
    if (magnumVec3Map_.count(key) > 0)
      magnumVec3Map_.erase(key);
  } else if (t == VEC_STRINGS) {
    if (vecStringsMap_.count(key) > 0)
      vecStringsMap_.erase(key);
  }
}

// clear all maps
void Attributes::clear() {
  doubleMap_.clear();
  intMap_.clear();
  boolMap_.clear();
  stringMap_.clear();
  magnumVec3Map_.clear();
  vecStringsMap_.clear();
}

// clear only a particular map
void Attributes::clearAs(const DataType t) {
  if (t == DOUBLE) {
    doubleMap_.clear();
  } else if (t == INT) {
    intMap_.clear();
  } else if (t == BOOL) {
    boolMap_.clear();
  } else if (t == STRING) {
    stringMap_.clear();
  } else if (t == MAGNUMVEC3) {
    magnumVec3Map_.clear();
  } else if (t == VEC_STRINGS) {
    vecStringsMap_.clear();
  }
}

//----------------------------------------//
//  Type specific getters/setters
//----------------------------------------//
// return the queried entry in the double map
// will throw an exception if the key does not exist in the double map
double Attributes::getDouble(const std::string& key) const {
  return doubleMap_.at(key);
}

// set a double attribute key->val
void Attributes::setDouble(const std::string& key, const double val) {
  doubleMap_[key] = val;
}

int Attributes::getInt(const std::string& key) const {
  return intMap_.at(key);
}

void Attributes::setInt(const std::string& key, const int val) {
  intMap_[key] = val;
}

bool Attributes::getBool(const std::string& key) const {
  return boolMap_.at(key);
}

void Attributes::setBool(const std::string& key, const bool val) {
  boolMap_[key] = val;
}

const std::string& Attributes::getString(const std::string& key) const {
  return stringMap_.at(key);
}

void Attributes::setString(const std::string& key, const std::string& val) {
  stringMap_[key] = val;
}

const Magnum::Vector3& Attributes::getMagnumVec3(const std::string& key) const {
  return magnumVec3Map_.at(key);
}

void Attributes::setMagnumVec3(const std::string& key,
                               const Magnum::Vector3& val) {
  magnumVec3Map_[key] = val;
}
const std::vector<std::string>& Attributes::getVecStrings(
    const std::string& key) const {
  return vecStringsMap_.at(key);
}

void Attributes::setVecStrings(const std::string& key,
                               const std::vector<std::string>& val) {
  vecStringsMap_[key] = val;
}

// add a string to a string vector (to avoid get/set copying)
void Attributes::appendVecStrings(const std::string& key,
                                  const std::string& val) {
  vecStringsMap_[key].push_back(val);
}

void Attributes::removeFromVecString(const std::string& key,
                                     const std::string& val) {
  std::vector<std::string>& stringVec = vecStringsMap_[key];
  std::vector<std::string>::iterator position =
      std::find(stringVec.begin(), stringVec.end(), val);
  if (position != stringVec.end())  // == .end() means the element was not found
    stringVec.erase(position);
}

// return a formated string exposing the current contents of the attributes maps
std::string Attributes::listAttributes() {
  std::string attributes =
      "List of attributes: \n----------------------------------------\n";

  attributes += "\nDoubles: \n";
  for (auto it : doubleMap_) {
    attributes += it.first + " : " + std::to_string(it.second) + "\n";
  }

  attributes += "\nInts: \n";
  for (auto it : intMap_) {
    attributes += it.first + " : " + std::to_string(it.second) + "\n";
  }

  attributes += "\nBools: \n";
  for (auto it : boolMap_) {
    attributes += it.first + " : " + std::to_string(it.second) + "\n";
  }

  attributes += "\nStrings: \n";
  for (auto it : stringMap_) {
    attributes += it.first + " : " + it.second + "\n";
  }

  attributes += "\nMagnum Vector3s: \n";
  for (auto it : magnumVec3Map_) {
    attributes += it.first + " : [" + std::to_string(it.second[0]) + ", " +
                  std::to_string(it.second[1]) + ", " +
                  std::to_string(it.second[2]) + "]\n";
  }

  attributes += "\nVectors of Strings: \n";
  for (auto it : vecStringsMap_) {
    attributes += it.first + " : [";
    for (auto vs_it = it.second.begin(); vs_it != it.second.end(); ++vs_it) {
      if (vs_it != it.second.begin())
        attributes += ", ";
      attributes += *vs_it;
    }
    attributes += "]\n";
  }

  attributes += "\n----------------------------------------\n\n";
  return attributes;
}

//----------------------------------------//
//  Derived attribute implementations
//----------------------------------------//

PhysicsObjectAttributes::PhysicsObjectAttributes() {
  // fill necessary attribute defaults
  setMass(1.0);
  setMargin(0.01);
  setScale({1.0, 1.0, 1.0});
  setCOM({0, 0, 0});
  setInertia({0, 0, 0});
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setLinearDamping(0.2);
  setAngularDamping(0.2);
  setOriginHandle("");
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
  setBoundingBoxCollisions(false);
  setJoinCollisionMeshes(true);
  setRequiresLighting(true);
}

// center of mass (COM)
void PhysicsObjectAttributes::setCOM(const Magnum::Vector3& com) {
  setMagnumVec3("COM", com);
}
Magnum::Vector3 PhysicsObjectAttributes::getCOM() const {
  return getMagnumVec3("COM");
}

// collision shape inflation margin
void PhysicsObjectAttributes::setMargin(double margin) {
  setDouble("margin", margin);
}
double PhysicsObjectAttributes::getMargin() const {
  return getDouble("margin");
}

void PhysicsObjectAttributes::setMass(double mass) {
  setDouble("mass", mass);
}
double PhysicsObjectAttributes::getMass() const {
  return getDouble("mass");
}

// inertia diagonal
void PhysicsObjectAttributes::setInertia(const Magnum::Vector3& inertia) {
  setMagnumVec3("inertia", inertia);
}
Magnum::Vector3 PhysicsObjectAttributes::getInertia() const {
  return getMagnumVec3("inertia");
}

void PhysicsObjectAttributes::setScale(const Magnum::Vector3& scale) {
  setMagnumVec3("scale", scale);
}
Magnum::Vector3 PhysicsObjectAttributes::getScale() const {
  return getMagnumVec3("scale");
}

void PhysicsObjectAttributes::setFrictionCoefficient(
    double frictionCoefficient) {
  setDouble("frictionCoefficient", frictionCoefficient);
}
double PhysicsObjectAttributes::getFrictionCoefficient() const {
  return getDouble("frictionCoefficient");
}

void PhysicsObjectAttributes::setRestitutionCoefficient(
    double restitutionCoefficient) {
  setDouble("restitutionCoefficient", restitutionCoefficient);
}
double PhysicsObjectAttributes::getRestitutionCoefficient() const {
  return getDouble("restitutionCoefficient");
}

void PhysicsObjectAttributes::setLinearDamping(double linearDamping) {
  setDouble("linDamping", linearDamping);  // TODO: change to "linearDamping"
}
double PhysicsObjectAttributes::getLinearDamping() const {
  return getDouble("linDamping");
}

void PhysicsObjectAttributes::setAngularDamping(double angularDamping) {
  setDouble("angDamping", angularDamping);  // TODO: change to "angularDamping"
}
double PhysicsObjectAttributes::getAngularDamping() const {
  return getDouble("angDamping");
}

void PhysicsObjectAttributes::setOriginHandle(const std::string& originHandle) {
  setString("originHandle", originHandle);
}
std::string PhysicsObjectAttributes::getOriginHandle() const {
  return getString("originHandle");
}

void PhysicsObjectAttributes::setRenderMeshHandle(
    const std::string& renderMeshHandle) {
  setString("renderMeshHandle", renderMeshHandle);
}
std::string PhysicsObjectAttributes::getRenderMeshHandle() const {
  return getString("renderMeshHandle");
}

void PhysicsObjectAttributes::setCollisionMeshHandle(
    const std::string& collisionMeshHandle) {
  setString("collisionMeshHandle", collisionMeshHandle);
}
std::string PhysicsObjectAttributes::getCollisionMeshHandle() const {
  return getString("collisionMeshHandle");
}

// if true override other settings and use render mesh bounding box as collision
// object
void PhysicsObjectAttributes::setBoundingBoxCollisions(
    bool useBoundingBoxForCollision) {
  setBool("useBoundingBoxForCollision", useBoundingBoxForCollision);
}
bool PhysicsObjectAttributes::getBoundingBoxCollisions() const {
  return getBool("useBoundingBoxForCollision");
}

// if true join all mesh components of an asset into a unified collision object
void PhysicsObjectAttributes::setJoinCollisionMeshes(bool joinCollisionMeshes) {
  setBool("joinCollisionMeshes", joinCollisionMeshes);
}
bool PhysicsObjectAttributes::getJoinCollisionMeshes() const {
  return getBool("joinCollisionMeshes");
}

// if true use phong illumination model instead of flat shading
void PhysicsObjectAttributes::setRequiresLighting(bool requiresLighting) {
  setBool("requiresLighting", requiresLighting);
}
bool PhysicsObjectAttributes::getRequiresLighting() const {
  return getBool("requiresLighting");
}

PhysicsSceneAttributes::PhysicsSceneAttributes() {
  setGravity({0, -9.8, 0});
  setFrictionCoefficient(0.4);
  setRestitutionCoefficient(0.05);
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
}

void PhysicsSceneAttributes::setGravity(const Magnum::Vector3& gravity) {
  setMagnumVec3("gravity", gravity);
}
Magnum::Vector3 PhysicsSceneAttributes::getGravity() const {
  return getMagnumVec3("gravity");
}

void PhysicsSceneAttributes::setFrictionCoefficient(
    double frictionCoefficient) {
  setDouble("frictionCoefficient", frictionCoefficient);
}
double PhysicsSceneAttributes::getFrictionCoefficient() const {
  return getDouble("frictionCoefficient");
}

void PhysicsSceneAttributes::setRestitutionCoefficient(
    double restitutionCoefficient) {
  setDouble("restitutionCoefficient", restitutionCoefficient);
}
double PhysicsSceneAttributes::getRestitutionCoefficient() const {
  return getDouble("restitutionCoefficient");
}

void PhysicsSceneAttributes::setRenderMeshHandle(
    const std::string& renderMeshHandle) {
  setString("renderMeshHandle", renderMeshHandle);
}
std::string PhysicsSceneAttributes::getRenderMeshHandle() const {
  return getString("renderMeshHandle");
}

void PhysicsSceneAttributes::setCollisionMeshHandle(
    const std::string& collisionMeshHandle) {
  setString("collisionMeshHandle", collisionMeshHandle);
}
std::string PhysicsSceneAttributes::getCollisionMeshHandle() const {
  return getString("collisionMeshHandle");
}

PhysicsManagerAttributes::PhysicsManagerAttributes() {
  setSimulator("none");
  setTimestep(0.01);
  setMaxSubsteps(10);
}

void PhysicsManagerAttributes::setSimulator(const std::string& simulator) {
  setString("simulator", simulator);
}
std::string PhysicsManagerAttributes::getSimulator() const {
  return getString("simulator");
}

void PhysicsManagerAttributes::setTimestep(double timestep) {
  setDouble("timestep", timestep);
}
double PhysicsManagerAttributes::getTimestep() const {
  return getDouble("timestep");
}

void PhysicsManagerAttributes::setMaxSubsteps(int maxSubsteps) {
  setInt("maxSubsteps", maxSubsteps);
}
int PhysicsManagerAttributes::getMaxSubsteps() const {
  return getInt("maxSubsteps");
}
}  // namespace assets
}  // namespace esp
