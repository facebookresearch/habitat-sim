// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ATTRIBUTESBASE_H_
#define ESP_METADATA_ATTRIBUTES_ATTRIBUTESBASE_H_

#include <Corrade/Utility/Directory.h>
#include <deque>
#include "esp/core/Configuration.h"
#include "esp/core/managedContainers/AbstractManagedObject.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief This enum class defines the possible shader options for rendering
 * instances of objects or stages in Habitat-sim.
 */
enum class ObjectInstanceShaderType {
  /**
   * Represents an unknown/unspecified value for the shader type to use. Resort
   * to defaults for object type.
   */
  Unknown = ID_UNDEFINED,
  /**
   * Override any config-specified or default shader-type values to use the
   * material-specified shader.
   */
  Material,
  /**
   * Refers to flat shading, pure color and no lighting.  This is often used for
   * textured objects
   */
  Flat,
  /**
   * Refers to phong shading with pure diffuse color.
   */
  Phong,
  /**
   * Refers to using a shader built with physically-based rendering models.
   */
  PBR,
  /**
   * End cap value - no shader type enums should be defined past this enum.
   */
  EndShaderType,
};

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * ObjectInstanceShaderType values.  This will be used to map values set
 * in json for translation origin to @ref ObjectInstanceShaderType.  Keys
 * must be lowercase.
 */
const extern std::map<std::string, ObjectInstanceShaderType> ShaderTypeNamesMap;

/**
 * @brief This method will convert an int value to the string key it maps to in
 * the ShaderTypeNamesMap
 */
std::string getShaderTypeName(int shaderTypeVal);

/**
 * @brief This enum class describes whether an object instance position is
 * relative to its COM or the asset's local origin.  Depending on this value, we
 * may take certain actions when instantiating a scene described by a scene
 * instance. For example, scene instances exported from Blender will have no
 * conception of an object's configured COM, and so will require adjustment to
 * translations to account for COM location when the object is placed*/
enum class SceneInstanceTranslationOrigin {
  /**
   * @brief Default value - in the case of object instances, this means use the
   * specified scene instance default; in the case of a scene instance, this
   * means do not correct for COM.
   */
  Unknown = -1,
  /**
   * @brief Indicates scene instance objects were placed without knowledge of
   * their COM location, and so need to be corrected when placed in scene in
   * Habitat. For example, they were exported from an external editor like
   * Blender.
   */
  AssetLocal,
  /**
   * @brief Indicates scene instance objects' location were recorded at their
   * COM location, and so do not need correction.  For example they were
   * exported from Habitat-sim.
   */
  COM,
  /**
   * End cap value - no instance translation origin type enums should be defined
   * past this enum.
   */
  EndTransOrigin,
};

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * SceneInstanceTranslationOrigin values.  This will be used to map values set
 * in json for translation origin to @ref SceneInstanceTranslationOrigin.  Keys
 * must be lowercase.
 */
const extern std::map<std::string, SceneInstanceTranslationOrigin>
    InstanceTranslationOriginMap;
/**
 * @brief This method will convert an int value to the string key it maps to in
 * the InstanceTranslationOriginMap
 */
std::string getTranslationOriginName(int translationOrigin);

/**
 * @brief Base class for all implemented attributes.  Inherits from @ref
 * esp::core::AbstractFileBasedManagedObject so the attributes can be managed by
 * a @ref esp::core::ManagedContainer.
 */
class AbstractAttributes : public esp::core::AbstractFileBasedManagedObject,
                           public esp::core::config::Configuration {
 public:
  AbstractAttributes(const std::string& attributesClassKey,
                     const std::string& handle)
      : AbstractFileBasedManagedObject(), Configuration() {
    // set up an existing subgroup for user_defined attributes
    addSubgroup("user_defined");
    AbstractAttributes::setClassKey(attributesClassKey);
    AbstractAttributes::setHandle(handle);
    // set initial vals, will be overwritten when registered
    set("ID", 0);
    set("fileDirectory", "");
  }

  ~AbstractAttributes() override = default;
  /**
   * @brief Get this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   */
  std::string getClassKey() const override {
    return get<std::string>("attributesClassKey");
  }

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes;  in
   * such cases this should be overridden with NOP.
   * @param handle the handle to set.
   */
  void setHandle(const std::string& handle) override { set("handle", handle); }
  std::string getHandle() const override { return get<std::string>("handle"); }

  /**
   * @brief This will return a simplified version of the attributes handle. Note
   * : there's no guarantee this handle will be sufficiently unique to identify
   * this attributes, so this should only be used for logging, and not for
   * attempts to search for attributes.
   */
  virtual std::string getSimplifiedHandle() const {
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
    set("fileDirectory", fileDirectory);
  }
  std::string getFileDirectory() const override {
    return get<std::string>("fileDirectory");
  }

  /**
   *  @brief Unique ID referencing attributes
   */
  void setID(int ID) override { set("ID", ID); }
  int getID() const override { return get<int>("ID"); }

  /**
   * @brief Gets a smart pointer reference to a copy of the user-specified
   * configuration data from config file. Habitat does not parse or process this
   * data, but it will be available to the user via python bindings for each
   * object.
   */
  std::shared_ptr<Configuration> getUserConfiguration() const {
    return getSubconfigCopy<Configuration>("user_defined");
  }

  /**
   * @brief Gets a smart pointer reference to the actual user-specified
   * configuration data from config file. Habitat does not parse or process this
   * data, but it will be available to the user via python bindings for each
   * object.  This method is for editing the configuration.
   */
  std::shared_ptr<Configuration> editUserConfiguration() {
    return editSubconfig<Configuration>("user_defined");
  }

  /**
   * @brief Returns the number of user-defined values (within the "user-defined"
   * sub-ConfigurationGroup) this attributes has.
   */
  int getNumUserDefinedConfigurations() const {
    return getSubconfigNumEntries("user_defined");
  }

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object.
   */
  std::string getObjectInfoHeader() const override {
    return Cr::Utility::formatString("Simplified Name, ID, {}",
                                     getObjectInfoHeaderInternal());
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfo() const override {
    return Cr::Utility::formatString("{},{},{}", getSimplifiedHandle(),
                                     getAsString("ID"),
                                     getObjectInfoInternal());
  }

 protected:
  /**
   * @brief return a vector of shared pointers to @ref AttributesBase
   * sub-configurations.
   */
  template <class T>
  std::vector<std::shared_ptr<const T>> getSubAttributesInternal(
      const std::shared_ptr<Configuration>& subAttrConfig) const;

  /**
   * @brief Add the passed shared pointer to @ref AbstractAttributes , @p
   * objInst , to the appropriate sub-config using the passed name.
   *
   * @tparam The type of smartpointer object instance attributes
   * @param objInst The object instance attributes pointer
   * @param availableIDs The ids available that can be used for the passed
   * attributes
   * @param subconfigKey The string key representing the subconfig to place @p
   * objInst in.
   * @param objInstNamePrefix The prefix to use to construct the key to store
   * the instance in the subconfig.
   */
  template <class T>
  void setSubAttributesInternal(
      std::shared_ptr<T>& objInst,
      std::deque<int>& availableIDs,
      const std::shared_ptr<Configuration>& subAttrConfig,
      const std::string& objInstNamePrefix);

  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */

  virtual std::string getObjectInfoHeaderInternal() const { return ","; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object, type-specific.
   */
  virtual std::string getObjectInfoInternal() const {
    return "no internal attributes specified,";
  };
  /**
   * @brief Set this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   * @param attributesClassKey the string handle corresponding to the
   * constructors used to make copies of this object in copy constructor map.
   */
  void setClassKey(const std::string& attributesClassKey) override {
    set("attributesClassKey", attributesClassKey);
  }

 public:
  ESP_SMART_POINTERS(AbstractAttributes)
};  // class AbstractAttributes

template <class T>
std::vector<std::shared_ptr<const T>>
AbstractAttributes::getSubAttributesInternal(
    const std::shared_ptr<Configuration>& subAttrConfig) const {
  std::vector<std::shared_ptr<const T>> res{};
  // pair of begin/end const iters through subconfig of given name
  int numSubconfigs = subAttrConfig->getNumSubconfigEntries();
  if (numSubconfigs == 0) {
    return {};
  }
  res.reserve(numSubconfigs);
  // get begin/end pair of iterators for subconfiguration
  auto subAttrIter = subAttrConfig->getSubconfigIterator();
  // iterate through subconfig entries, casting appropriately and adding to
  // map if cast successful
  for (auto objIter = subAttrIter.first; objIter != subAttrIter.second;
       ++objIter) {
    auto obj = objIter->second;
    if (std::shared_ptr<T> objPtr = std::dynamic_pointer_cast<T>(obj)) {
      res.emplace_back(std::move(objPtr));
    } else {
      ESP_WARNING() << getClassKey() << ": Subconfig obj with"
                    << obj->getNumEntries()
                    << "entries is not castable to appropriate type const"
                    << typeid(T).name() << " | " << typeid(obj).name() << " | {"
                    << objIter->first << " : " << obj->getAsString("handle")
                    << "}";
    }
  }
  return res;
}

template <class T>
void AbstractAttributes::setSubAttributesInternal(
    std::shared_ptr<T>& objInst,
    std::deque<int>& availableIDs,
    const std::shared_ptr<Configuration>& subAttrConfig,
    const std::string& objInstNamePrefix) {
  // get subconfig for articulated object instances, add this ao instance as a
  // set id
  if (!availableIDs.empty()) {
    // use saved value and then remove from storage
    objInst->setID(availableIDs.front());
    availableIDs.pop_front();
  } else {
    // use size of container to set ID
    objInst->setID(subAttrConfig->getNumSubconfigEntries());
  }
  // get last key
  subAttrConfig->setSubconfigPtr<T>(
      Cr::Utility::formatString("{:.05d}_{}{}", objInst->getID(),
                                objInstNamePrefix,
                                objInst->getSimplifiedHandle()),
      objInst);
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ATTRIBUTESBASE_H_
