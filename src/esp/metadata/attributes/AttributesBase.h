// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ATTRIBUTESBASE_H_
#define ESP_METADATA_ATTRIBUTES_ATTRIBUTESBASE_H_

#include <Corrade/Utility/Directory.h>
#include "esp/core/AbstractManagedObject.h"
#include "esp/core/Configuration.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Base class for all implemented attributes.  Inherits from @ref
 * esp::core::AbstractManagedObject so the attributes can be managed by a @ref
 * esp::core::ManagedContainer.
 */
class AbstractAttributes : public esp::core::AbstractManagedObject,
                           public esp::core::Configuration {
 public:
  AbstractAttributes(const std::string& attributesClassKey,
                     const std::string& handle)
      : Configuration() {
    setClassKey(attributesClassKey);
    AbstractAttributes::setHandle(handle);
  }

  ~AbstractAttributes() override = default;
  /**
   * @brief Get this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   */
  std::string getClassKey() const override {
    return getString("attributesClassKey");
  }

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes;  in
   * such cases this should be overridden with NOP.
   * @param handle the handle to set.
   */
  void setHandle(const std::string& handle) override {
    setString("handle", handle);
  }
  std::string getHandle() const override { return getString("handle"); }

  /**
   * @brief This will return a simplified version of the attributes handle. Note
   * : there's no guarantee this handle will be sufficiently unique to identify
   * this attributes, so this should only be used for logging, and not for
   * attempts to search for attributes.
   */
  std::string getSimplifiedHandle() {
    // first parse for file name, and then get rid of extension(s).
    return Corrade::Utility::Directory::splitExtension(
               Corrade::Utility::Directory::splitExtension(
                   Corrade::Utility::Directory::filename(getHandle()))
                   .first)
        .first;
  }

  /**
   * @brief directory where files used to construct attributes can be found.
   */
  void setFileDirectory(const std::string& fileDirectory) override {
    setString("fileDirectory", fileDirectory);
  }
  std::string getFileDirectory() const override {
    return getString("fileDirectory");
  }

  /**
   *  @brief Unique ID referencing attributes
   */
  void setID(int ID) override { setInt("ID", ID); }
  int getID() const override { return getInt("ID"); }

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
  void setClassKey(const std::string& attributesClassKey) override {
    setString("attributesClassKey", attributesClassKey);
  }

 public:
  ESP_SMART_POINTERS(AbstractAttributes)
};  // class AbstractAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ATTRIBUTESBASE_H_
