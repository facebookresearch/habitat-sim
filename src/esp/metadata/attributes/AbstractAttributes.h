// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTATTRIBUTES_H_

#include <Corrade/Utility/Path.h>
#include <deque>
#include "AttributesEnumMaps.h"
#include "esp/core/Configuration.h"
#include "esp/core/managedContainers/AbstractFileBasedManagedObject.h"

namespace esp {
namespace metadata {
/**
 * @brief A tag to search for in the default_attributes section of the Scene
 * Dataset JSON configuration files denoting that an implementation of the
 * attributes should replace this tag with the base filename (minus all paths
 * and extensions)
 */
static constexpr char CONFIG_NAME_AS_ASSET_FILENAME[] =
    "%%CONFIG_NAME_AS_ASSET_FILENAME%%";

namespace attributes {

/**
 * @brief Base class for all implemented attributes.  Inherits from @ref
 * esp::core::managedContainers::AbstractFileBasedManagedObject so the
 * attributes can be managed by a @ref
 * esp::core::managedContainers::ManagedContainer.
 */
class AbstractAttributes
    : public esp::core::managedContainers::AbstractFileBasedManagedObject,
      public esp::core::config::Configuration {
 public:
  AbstractAttributes(const std::string& attributesClassKey,
                     const std::string& handle);

  AbstractAttributes(const AbstractAttributes& otr) = default;
  AbstractAttributes(AbstractAttributes&& otr) noexcept = default;
  ~AbstractAttributes() override = default;

  AbstractAttributes& operator=(const AbstractAttributes& otr) = default;
  AbstractAttributes& operator=(AbstractAttributes&& otr) noexcept = default;

  /**
   * @brief Support writing to JSON object as required by @ref
   * esp::core::managedContainers::AbstractFileBasedManagedObject
   */
  io::JsonGenericValue writeToJsonObject(
      io::JsonAllocator& allocator) const override {
    return Configuration::writeToJsonObject(allocator);
  }

  /**
   * @brief Get this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   */
  std::string getClassKey() const override {
    return get<std::string>("__attributesClassKey");
  }

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes; in
   * such cases this should be overridden with NOP.
   * @param handle the handle to set.
   */
  void setHandle(const std::string& handle) override { set("handle", handle); }
  std::string getHandle() const override { return get<std::string>("handle"); }

  /**
   * @brief Set the directory where files used to construct ManagedObject can be
   * found.
   */
  void setFileDirectory(const std::string& fileDirectory) override {
    setHidden("__fileDirectory", fileDirectory);
  }

  /**
   * @brief Get directory where files used to construct ManagedObject can be
   * found.
   */
  std::string getFileDirectory() const override {
    return get<std::string>("__fileDirectory");
  }

  /**
   * @brief Set the fully qualified filename of the file used to create or most
   * recently save this ManagedObject.
   */
  void setActualFilename(const std::string& fullFileName) override {
    setHidden("__actualFilename", fullFileName);
  }

  /**
   * @brief Get the fully qualified filename of the file used to create or most
   * recently save this ManagedObject.
   */
  std::string getActualFilename() const override {
    return get<std::string>("__actualFilename");
  }

  /**
   *  @brief Set the unique ID referencing attributes
   */
  void setID(int ID) override { setHidden("__ID", ID); }
  /**
   *  @brief Get the unique ID referencing attributes
   */
  int getID() const override { return get<int>("__ID"); }

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
   * @brief Gets a const smart pointer reference to a view of the user-specified
   * configuration data from config file. Habitat does not parse or process this
   * data, but it will be available to the user via python bindings for each
   * object.
   */
  std::shared_ptr<const Configuration> getUserConfigurationView() const {
    return getSubconfigView("user_defined");
  }

  /**
   * @brief Gets a smart pointer reference to the actual user-specified
   * configuration data from config file. Habitat does not parse or process this
   * data, but it will be available to the user via python bindings for each
   * object. This method is for editing the configuration.
   */
  std::shared_ptr<Configuration> editUserConfiguration() {
    return editSubconfig<Configuration>("user_defined");
  }

  /**
   * @brief Move an existing user_defined subconfiguration into this
   * configuration, overwriting the existing copy if it exists. Habitat does not
   * parse or process this data, but it will be available to the user via python
   * bindings for each object. This method is for editing the configuration.
   */
  void setUserConfiguration(std::shared_ptr<Configuration>& userAttr) {
    setSubconfigPtr("user_defined", userAttr);
  }

  /**
   * @brief Returns the number of user-defined values (within the "user-defined"
   * sub-ConfigurationGroup) this attributes has.
   */
  int getNumUserDefinedConfigurations() const {
    return getSubconfigNumEntries("user_defined");
  }

  /**
   * @brief Returns the number of user-defined values (within the "user-defined"
   * sub-ConfigurationGroup) this attributes has.
   */
  int getTotalNumUserDefinedConfigurations() const {
    return getSubconfigTreeNumEntries("user_defined");
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
                                     getAsString("__ID"),
                                     getObjectInfoInternal());
  }

  /**
   * @brief Check whether filepath-based fields have been set by user input
   * but have not been verified to exist (such verification occurs when the
   * attributes is registered.)
   */
  bool getFilePathsAreDirty() const { return get<bool>("__fileNamesDirty"); }

  /**
   * @brief Clear the flag that specifies that filepath-based fields have been
   * set but not verfified to exist (such verification occurs when the
   * attributes is registered.)
   */
  void setFilePathsAreClean() { setHidden("__fileNamesDirty", false); }

  /**
   * @brief Get whether this ManagedObject has been saved to disk in its current
   * state. Only applicable to registered ManagedObjects
   */
  bool isAttrSaved() const override { return get<bool>("__isAttrSaved"); }

 protected:
  /**
   * @brief Set this ManagedObject's save status (i.e. whether it matches its
   * version on disk or not)
   */
  void setFileSaveStatus(bool _isSaved) override {
    setHidden("__isAttrSaved", _isSaved);
  }

  /**
   * @brief Used internally only. Set the flag that specifies a filepath-based
   * field has been set to some value but has not yet been verified to
   * exist (such verification occurs when the attributes is registered.)
   */
  void setFilePathsAreDirty() { setHidden("__fileNamesDirty", true); }

  /**
   * @brief Changing access to setter so that Configuration bindings cannot be
   * used to set a reserved value to an incorrect type. The inheritors of this
   * class provide an interface to set the values that the consumers of these
   * Attributes depend on and this should be the only mechanism capable of doing
   * so.
   */
  using Configuration::set;

  /**
   * @brief return a vector of shared pointers to const @ref AbstractAttributes
   * sub-configurations.
   * @param subAttrConfig The subconfiguration from which to aquire the
   * subconfigs.
   */
  template <class T>
  std::vector<std::shared_ptr<const T>> getSubAttributesListInternal(
      const std::shared_ptr<Configuration>& subAttrConfig) const;

  /**
   * @brief This function is specifcially designed to copy sub-subconfigs whilst
   * retaining their full type.  Any @ref AbstractAttributes object that gets
   * stored as a subconfig @ref Configuration will retain its full type the
   * config owning it gets copy constructed, in which case the copy will not
   * retain its @ref AbstractFileBasedManagedObject interface characteristics.
   * This function is provided so that any @ref AbstractAttributes object that
   * is copied will have its sub-subconfigs copied with appropriate type.
   * @param srcSubAttrConfig The source of the copy, which retains appropriate
   * type.
   * @param destSubAttrConfig The destination of the copy.
   */
  template <class T>
  void copySubconfigIntoMe(
      const std::shared_ptr<Configuration>& srcSubAttrConfig,
      const std::shared_ptr<Configuration>& destSubAttrConfig);

  /**
   * @brief Return the number of SubAttribute Configurations with passed key
   * @p subConfigNamePrefix in name
   * @param subConfigNamePrefix Name tag to look for to count subconfig.
   * @param subAttrConfig
   */
  int getNumSubAttributesInternal(
      const std::string& subConfigNamePrefix,
      const std::shared_ptr<Configuration>& subAttrConfig) const {
    int res = 0;
    if (subConfigNamePrefix.empty()) {
      return subAttrConfig->getNumSubconfigs();
    }
    // iterator to subAttrConfig's subConfigs
    auto subAttrIter = subAttrConfig->getSubconfigIterator();
    for (auto objIter = subAttrIter.first; objIter != subAttrIter.second;
         ++objIter) {
      const std::string key = objIter->first;
      if (std::string::npos != key.find(subConfigNamePrefix)) {
        ++res;
      }
    }
    return res;
  }

  /**
   * @brief Returns a shared pointer to the named @ref AbstractAttributes
   * sub-configurations member of the passed @p subAttrConfig.
   */
  template <class T>
  std::shared_ptr<const T> getNamedSubAttributesInternal(
      const std::string& name,
      const std::shared_ptr<Configuration>& subAttrConfig) const;

  /**
   * @brief Removes and returns a shared pointer to the named @ref
   * AbstractAttributes sub-configurations member of the passed @p
   * subAttrConfig. The object's ID is freed as part of this process, to be used
   * by other objects.
   * @tparam The desired type of the removed Configuration
   * @param name The name of the object to remove.
   * @param availableIDs A deque of the IDs that this configuration has
   * available for these types of Subconfigurations to use.
   * @param subAttrConfig The
   */
  template <class T>
  std::shared_ptr<T> removeNamedSubAttributesInternal(
      const std::string& name,
      std::deque<int>& availableIDs,
      const std::shared_ptr<Configuration>& subAttrConfig);

  /**
   * @brief Add the passed shared pointer to @ref AbstractAttributes ,
   * @p objInst , to the passed sub-config building a name from the passed
   * @p objInstNamePrefix .
   *
   * @tparam The type of smartpointer object instance attributes
   * @param objInst The subAttributes Configuration pointer
   * @param availableIDs The ids available that can be used for the passed
   * attributes
   * @param subAttrConfig The subconfig to place @p objInst in.
   * @param objInstNamePrefix The prefix to use to construct the key to store
   * the instance in the subconfig. If empty, will use @p objInst->getHandle().
   * @param verifyUnique Verify that the new subconfiguration holds unique data
   * (i.e. no other subconfig exists that has the same data). If this is true
   * and the passed @p objInst is not unique, it will not be saved.
   * @return whether the passed @p objInst was added. Only returns false if
   * @p verifyUnique is true and the objInst is not unique.
   */
  template <class T>
  bool setSubAttributesInternal(
      std::shared_ptr<T>& objInst,
      std::deque<int>& availableIDs,
      const std::shared_ptr<Configuration>& subAttrConfig,
      const std::string& objInstNamePrefix,
      bool verifyUnique);

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
    setHidden("__attributesClassKey", attributesClassKey);
  }

 public:
  ESP_SMART_POINTERS(AbstractAttributes)
};  // class AbstractAttributes

template <class T>
std::vector<std::shared_ptr<const T>>
AbstractAttributes::getSubAttributesListInternal(
    const std::shared_ptr<Configuration>& subAttrConfig) const {
  static_assert(
      std::is_base_of<AbstractAttributes, T>::value,
      "AbstractAttributes : Desired subconfig type must be derived from "
      "esp::metadata::AbstractAttributes");
  std::vector<std::shared_ptr<const T>> res{};
  // pair of begin/end const iters through subconfig of given name
  int numSubconfigs = subAttrConfig->getNumSubconfigs();
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
    std::shared_ptr<T> objPtr;
    CORRADE_ASSERT_OUTPUT(
        objPtr = std::dynamic_pointer_cast<T>(obj),
        Cr::Utility::formatString(
            "{}: Subconfig obj with {} entries is not castable to appropriate "
            "type const {} | Obj type:{} | Subconfig K:V : ( {}, {} )",
            getClassKey(), obj->getNumEntries(), typeid(T).name(),
            typeid(obj).name(), objIter->first, obj->getAsString("handle")),
        res);

    res.emplace_back(std::move(std::const_pointer_cast<const T>(objPtr)));
  }
  return res;
}  // AbstractAttributes::getSubAttributesListInternal

template <class T>
std::shared_ptr<const T> AbstractAttributes::getNamedSubAttributesInternal(
    const std::string& name,
    const std::shared_ptr<Configuration>& subAttrConfig) const {
  static_assert(
      std::is_base_of<AbstractAttributes, T>::value,
      "AbstractAttributes : Desired subconfig type must be derived from "
      "esp::metadata::AbstractAttributes");
  auto obj = subAttrConfig->getSubconfigView(name);
  if (obj == nullptr) {
    // not found - fail quietly
    return nullptr;
  }
  std::shared_ptr<const T> objPtr;
  CORRADE_ASSERT_OUTPUT(
      objPtr = std::dynamic_pointer_cast<const T>(obj),
      Cr::Utility::formatString(
          "{}: Subconfig obj with {} entries is not castable to appropriate "
          "type const {} | Obj type:{} | Handle : {}",
          getClassKey(), obj->getNumEntries(), typeid(T).name(),
          typeid(obj).name(), obj->getAsString("handle")),
      nullptr);
  return objPtr;

}  // AbstractAttributes::getNamedSubAttributesInternal

template <class T>
std::shared_ptr<T> AbstractAttributes::removeNamedSubAttributesInternal(
    const std::string& name,
    std::deque<int>& availableIDs,
    const std::shared_ptr<Configuration>& subAttrConfig) {
  static_assert(
      std::is_base_of<AbstractAttributes, T>::value,
      "AbstractAttributes : Desired subconfig type must be derived from "
      "esp::metadata::AbstractAttributes");
  auto obj = subAttrConfig->removeSubconfig(name);
  if (obj == nullptr) {
    // not found - fail quietly
    return nullptr;
  }
  std::shared_ptr<T> objPtr;
  CORRADE_ASSERT_OUTPUT(
      objPtr = std::dynamic_pointer_cast<T>(obj),
      Cr::Utility::formatString(
          "{}: Subconfig obj with {} entries is not castable to appropriate "
          "type const {} | Obj type:{} | Handle : {}",
          getClassKey(), obj->getNumEntries(), typeid(T).name(),
          typeid(obj).name(), obj->getAsString("handle")),
      nullptr);
  // queue available ID
  availableIDs.emplace_front(obj->get<int>("__ID"));
  return objPtr;

}  // AbstractAttributes::removeNamedSubAttributesInternal

template <class T>
bool AbstractAttributes::setSubAttributesInternal(
    std::shared_ptr<T>& objInst,
    std::deque<int>& availableIDs,
    const std::shared_ptr<Configuration>& subAttrConfig,
    const std::string& objInstNamePrefix,
    bool verifyUnique) {
  static_assert(
      std::is_base_of<AbstractAttributes, T>::value,
      "AbstractAttributes : Desired subconfig type must be derived from "
      "esp::metadata::AbstractAttributes");
  // check uniqueness if verifyUnique is true. If not unique, do not add
  // subconfig
  if (verifyUnique) {
    // Check if subAttrConfig contains a duplicate entry to objInst, other than
    // hidden/internal fields like ID
    if (subAttrConfig->hasSubconfig(objInst)) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "An identical subconfig to subconfig :`" << objInst->getHandle()
          << "` was found in existing subconfig collection, so duplicate was "
             "not added.";
      return false;
    }
  }
  // set id
  if (!availableIDs.empty()) {
    // use saved value and then remove from storage
    objInst->setID(availableIDs.front());
    availableIDs.pop_front();
  } else {
    // use current size of destination config to set ID
    objInst->setID(subAttrConfig->getNumSubconfigs());
  }
  // get last key
  subAttrConfig->setSubconfigPtr<T>(
      objInstNamePrefix.empty()
          ? objInst->getHandle()
          : Cr::Utility::formatString("{:.05d}_{}{}", objInst->getID(),
                                      objInstNamePrefix,
                                      objInst->getSimplifiedHandle()),
      objInst);
  return true;
}  // AbstractAttributes::setSubAttributesInternal

template <class T>
void AbstractAttributes::copySubconfigIntoMe(
    const std::shared_ptr<Configuration>& srcSubAttrConfig,
    const std::shared_ptr<Configuration>& destSubAttrConfig) {
  static_assert(
      std::is_base_of<AbstractAttributes, T>::value,
      "AbstractAttributes : Desired subconfig type must be derived from "
      "esp::metadata::AbstractAttributes");
  // copy configs from srcSubAttrConfig into destSubAttrConfig, with appropriate
  // casting

  // get begin/end pair of iterators for subconfiguration
  auto subAttrIter = srcSubAttrConfig->getSubconfigIterator();

  // iterate through subconfig entries, casting appropriately and adding to
  // map if cast successful
  for (auto objIter = subAttrIter.first; objIter != subAttrIter.second;
       ++objIter) {
    auto obj = objIter->second;
    std::shared_ptr<T> objPtr;
    CORRADE_ASSERT_OUTPUT(
        objPtr = std::dynamic_pointer_cast<T>(obj),
        Cr::Utility::formatString(
            "{}: Subconfig obj with {} entries is not castable to appropriate "
            "type const {} | Obj type:{} | Subconfig K:V : ( {}, {} )",
            getClassKey(), obj->getNumEntries(), typeid(T).name(),
            typeid(obj).name(), objIter->first, obj->getAsString("handle")), );
    destSubAttrConfig->setSubconfigPtr<T>(objIter->first, objPtr);
  }
}  // AbstractAttributes::copySubconfigIntoMe

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTATTRIBUTES_H_
