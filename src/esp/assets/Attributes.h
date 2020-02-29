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

/**
 * @brief DataTypes supported by the attributes container. Complexity of non
 * type-specific operations growths wrt to number of supported types
 */
enum DataType { DOUBLE, STRING, INT, MAGNUMVEC3, VEC_STRINGS, BOOL };

/**
 * @brief Arbitrary map type container for storing and managing various types of
 * attribute data
 */
class Attributes {
 public:
  //! constructor initializes the maps
  Attributes();

  //! return true if any container has the key
  bool exists(const std::string& key) const;

  //! check if an attribute of a specific type exists
  bool existsAs(const DataType t, const std::string& key) const;

  //! count the number of containers with the key
  int count(const std::string& key) const;

  //! erase the key from all maps
  void eraseAll(const std::string& key);

  //! erase the key from a particular map
  void eraseAs(const DataType t, const std::string& key);

  //! clear all maps
  void clear();

  //! clear only a particular map
  void clearAs(const DataType t);

  //----------------------------------------//
  //  Type specific getters/setters
  //----------------------------------------//

  /**
   * @brief return the queried entry in the double map
   * will throw an exception if the key does not exist in the double map
   */
  double getDouble(const std::string& key) const;

  //! set a double attribute key->val
  void setDouble(const std::string& key, const double val);

  int getInt(const std::string& key) const;

  void setInt(const std::string& key, const int val);
  bool getBool(const std::string& key) const;

  void setBool(const std::string& key, const bool val);

  const std::string& getString(const std::string& key) const;

  void setString(const std::string& key, const std::string& val);

  const Magnum::Vector3& getMagnumVec3(const std::string& key) const;

  void setMagnumVec3(const std::string& key, const Magnum::Vector3& val);
  const std::vector<std::string>& getVecStrings(const std::string& key) const;

  void setVecStrings(const std::string& key,
                     const std::vector<std::string>& val);

  //! add a string to a string vector (to avoid get/set copying)
  void appendVecStrings(const std::string& key, const std::string& val);
  //! remove a string from a string vector (to avoid get/set copying)
  void removeFromVecString(const std::string& key, const std::string& val);

  /**
   * @brief return a formated string exposing the current contents of the
   * attributes maps
   */
  std::string listAttributes();

 private:
  std::map<std::string, double> doubleMap_;
  std::map<std::string, int> intMap_;
  std::map<std::string, bool> boolMap_;
  std::map<std::string, std::string> stringMap_;
  std::map<std::string, Magnum::Vector3> magnumVec3Map_;
  std::map<std::string, std::vector<std::string> > vecStringsMap_;

};  // end Attributes class

/**
 * @brief Specific Attributes instance which is constructed with a base set of
 * physics object required attributes
 */
class PhysicsObjectAttributes : public Attributes {
 public:
  PhysicsObjectAttributes();
};  // end PhysicsObjectAttributes class

//! attributes for a single physical scene
class PhysicsSceneAttributes : public Attributes {
 public:
  PhysicsSceneAttributes();
};  // end PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public Attributes {
 public:
  PhysicsManagerAttributes();
};  // end PhysicsManagerAttributes

}  // namespace assets
}  // namespace esp
