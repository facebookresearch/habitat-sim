// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Magnum.h>
#include <map>
#include <string>
#include <vector>
#include "Magnum/Math/Math.h"
#include "Magnum/Types.h"
#include "esp/gfx/magnum.h"

namespace esp {
namespace assets {

// DataTypes supported by the attributes container. Complexity of non
// type-specific operations growths wrt to number of supported types
enum DataType { DOUBLE, STRING, INT, MAGNUMVEC3, VEC_STRINGS };

// Arbitrary map type container for storing and managing various types of
// attribute data
class Attributes {
 public:
  // constructor initializes the maps
  Attributes();

  // return true if any container has the key
  bool exists(std::string key);

  // check if an attribute of a specific type exists
  bool existsAs(DataType t, std::string key);

  // count the number of containers with the key
  int count(std::string key);

  // erase the key from all maps
  void eraseAll(std::string key);

  // erase the key from a particular map
  void eraseAs(DataType t, std::string key);

  // clear all maps
  void clear();

  // clear only a particular map
  void clearAs(DataType t);

  //----------------------------------------//
  //  Type specific getters/setters
  //----------------------------------------//
  // return the queried entry in the double map
  // will throw an exception if the key does not exist in the double map
  double getDouble(std::string key);

  // set a double attribute key->val
  void setDouble(std::string key, double val);

  int getInt(std::string key);

  void setInt(std::string key, int val);

  std::string getString(std::string key);

  void setString(std::string key, std::string val);

  Magnum::Vector3 getMagnumVec3(std::string key);

  void setMagnumVec3(std::string key, Magnum::Vector3 val);
  std::vector<std::string> getVecStrings(std::string key);

  void setVecStrings(std::string key, std::vector<std::string> val);

  // add a string to a string vector (to avoid get/set copying)
  void appendVecStrings(std::string key, std::string val);
  // remove a string from a string vector (to avoid get/set copying)
  void removeFromVecString(std::string key, std::string val);

  // return a formated string exposing the current contents of the attributes
  // maps
  std::string listAttributes();

 private:
  std::map<std::string, double> doubleMap_;
  std::map<std::string, int> intMap_;
  std::map<std::string, std::string> stringMap_;
  std::map<std::string, Magnum::Vector3> magnumVec3Map_;
  std::map<std::string, std::vector<std::string> > vecStringsMap_;

};  // end Attributes class

// Specific Attributes instance which is constructed with a base set of physics
// object required attributes
class PhysicsObjectAttributes : public Attributes {
 public:
  PhysicsObjectAttributes();
};  // end PhysicsObjectAttributes class

// attributes for a single physical scene
class PhysicsSceneAttributes : public Attributes {
 public:
  PhysicsSceneAttributes();
};  // end PhysicsSceneAttributes

// attributes for a single physics manager
class PhysicsManagerAttributes : public Attributes {
 public:
  PhysicsManagerAttributes();
};  // end PhysicsManagerAttributes

}  // namespace assets
}  // namespace esp
