// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_ATTRIBUTES_ATTRIBUTES_H_
#define ESP_ASSETS_ATTRIBUTES_ATTRIBUTES_H_

#include "esp/core/Configuration.h"

namespace esp {
namespace assets {
namespace attributes {

/**
 * @brief Base class for all implemented attributes.  This abstract base class
 * also illustrates the functionality exectation of @ref AttributesManagerBase
 * class template specializations.
 */
class AbstractAttributes : public esp::core::Configuration {
 public:
  AbstractAttributes(const std::string& attributesClassKey,
                     const std::string& handle)
      : Configuration() {
    setAttributesClassKey(attributesClassKey);
    AbstractAttributes::setHandle(handle);
  }

  virtual ~AbstractAttributes() = default;
  /**
   * @brief Get this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   */
  std::string getClassKey() const { return getString("attributesClassKey"); }

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes;  in
   * such cases this should be overridden with NOP.
   * @param handle the handle to set.
   */
  virtual void setHandle(const std::string& handle) {
    setString("handle", handle);
  }
  std::string getHandle() const { return getString("handle"); }

  /**
   * @brief directory where files used to construct attributes can be found.
   */
  virtual void setFileDirectory(const std::string& fileDirectory) {
    setString("fileDirectory", fileDirectory);
  }
  std::string getFileDirectory() const { return getString("fileDirectory"); }

  void setID(int ID) { setInt("ID", ID); }
  int getID() const { return getInt("ID"); }

  /**
   * @brief Returns configuration to be used with PrimitiveImporter to
   * instantiate Primitives.  Names in getter/setters chosen to match parameter
   * name expectations in PrimitiveImporter.
   *
   * @return a reference to the underlying configuration group for this
   * attributes object
   */
  const Corrade::Utility::ConfigurationGroup& getConfigGroup() const {
    return cfg;
  }

 protected:
  /**
   * @brief Set this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   * @param attributesClassKey the string handle corresponding to the
   * constructors used to make copies of this object in copy constructor map.
   */
  void setAttributesClassKey(const std::string& attributesClassKey) {
    setString("attributesClassKey", attributesClassKey);
  }

 public:
  ESP_SMART_POINTERS(AbstractAttributes)
};  // class AbstractAttributes

}  // namespace attributes
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_ATTRIBUTES_H_
