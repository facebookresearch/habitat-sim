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

PhysicsObjectAttributes::PhysicsObjectAttributes() {
  // fill necessary attribute defaults
  setDouble("mass", 1.0);
  setDouble("margin", 0.01);
  setDouble("scale", 1.0);
  setMagnumVec3("COM", Magnum::Vector3(0));
  setMagnumVec3("inertia", Magnum::Vector3(0., 0., 0.));
  setDouble("frictionCoefficient", 0.5);
  setDouble("restitutionCoefficient", 0.1);
  setDouble("linDamping", 0.2);
  setDouble("angDamping", 0.2);
  setString("originHandle", "");
  setString("renderMeshHandle", "");
  setString("collisionMeshHandle", "");
}

PhysicsSceneAttributes::PhysicsSceneAttributes() {
  setMagnumVec3("gravity", Magnum::Vector3(0, -9.8, 0));
  setDouble("frictionCoefficient", 0.4);
  setDouble("restitutionCoefficient", 0.05);
  setString("renderMeshHandle", "");
  setString("collisionMeshHandle", "");
}

PhysicsManagerAttributes::PhysicsManagerAttributes() {
  setString("simulator", "none");
  setDouble("timestep", 0.01);
  setInt("maxSubsteps", 10);
}
}  // namespace assets
}  // namespace esp
