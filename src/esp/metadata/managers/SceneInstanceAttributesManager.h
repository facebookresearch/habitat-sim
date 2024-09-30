// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_SCENEINSTANCEATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_SCENEINSTANCEATTRIBUTEMANAGER_H_

#include "AbstractAttributesManager.h"
#include "esp/metadata/attributes/SceneInstanceAttributes.h"

namespace esp {
namespace metadata {

namespace managers {
using esp::core::managedContainers::ManagedObjectAccess;

class SceneInstanceAttributesManager
    : public AbstractAttributesManager<attributes::SceneInstanceAttributes,
                                       ManagedObjectAccess::Copy> {
 public:
  SceneInstanceAttributesManager()
      : AbstractAttributesManager<attributes::SceneInstanceAttributes,
                                  ManagedObjectAccess::Copy>::
            AbstractAttributesManager("Scene Instance", "scene_instance.json") {
    // build this manager's copy constructor map
    this->copyConstructorMap_["SceneInstanceAttributes"] =
        &SceneInstanceAttributesManager::createObjCopyCtorMapEntry<
            attributes::SceneInstanceAttributes>;
  }

  /**
   * @brief Creates the descriptive attributes template for a scene instance
   * described by passed string.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param sceneInstanceHandle the origin of the desired dataset template
   * to be created.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the newly-created template.
   */
  attributes::SceneInstanceAttributes::ptr createObject(
      const std::string& sceneInstanceHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Set the curent default PbrShaderAttributes to use on all new scene
   * instances. This can be overridden from within individual scene instances.
   */
  void setDefaultPbrShaderAttrHandle(const std::string& pbrHandle) {
    defaultPbrShaderAttributesHandle_ = pbrHandle;
  }

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::SceneInstanceAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This will return a @ref
   * attributes::SceneObjectInstanceAttributes object with passed handle.
   */
  attributes::SceneObjectInstanceAttributes::ptr createEmptyInstanceAttributes(
      const std::string& handle,
      const std::shared_ptr<attributes::AbstractObjectAttributes>&
          baseObjAttribs = nullptr) {
    if (baseObjAttribs) {
      // SceneObjectInstanceAttributes constructor that takes an attributes is
      // protected
      return std::shared_ptr<attributes::SceneObjectInstanceAttributes>(
          new attributes::SceneObjectInstanceAttributes(handle,
                                                        baseObjAttribs));
    }
    return attributes::SceneObjectInstanceAttributes::create(handle);
  }

  /**
   * @brief This will return a @ref
   * attributes::SceneObjectInstanceAttributes object with passed handle.
   */
  attributes::SceneAOInstanceAttributes::ptr createEmptyAOInstanceAttributes(
      const std::string& handle,
      const std::shared_ptr<attributes::ArticulatedObjectAttributes>&
          aObjAttribs = nullptr) {
    if (aObjAttribs) {
      // SceneAOInstanceAttributes constructor that takes an attributes is
      // protected
      return std::shared_ptr<attributes::SceneAOInstanceAttributes>(
          new attributes::SceneAOInstanceAttributes(handle, aObjAttribs));
    }
    return attributes::SceneAOInstanceAttributes::create(handle);
  }

  /**
   * @brief This function will be called to finalize attributes' paths before
   * registration, moving fully qualified paths to the appropriate hidden
   * attribute fields. This can also be called without registration to make sure
   * the paths specified in an attributes are properly configured.
   * @param attributes The attributes to be filtered.
   */
  void finalizeAttrPathsBeforeRegister(
      CORRADE_UNUSED const attributes::SceneInstanceAttributes::ptr& attributes)
      const override {}

 protected:
  /**
   * @brief Gets the name/key value of the appropriate enum corresponding to the
   * desired Translation Origin used to determine the location of the asset in
   * the scene instance.  The purpose of this value is to know whether to
   * correct placement by location of COM of object when instance is created.
   * @param jsonDoc document where value may be specified.
   * @return the int value to set for translation_origin in instance attributes.
   */
  std::string getTranslationOriginVal(
      const io::JsonGenericValue& jsonDoc) const;

  /**
   * @brief Used Internally.  Create a @ref
   * esp::metadata::attributes::SceneObjectInstanceAttributes object from the
   * passed JSON doc.
   * @param jCell JSON object containing the description of the stage or object
   * instance.
   * @return the constructed @ref
   * esp::metadata::attributes::SceneObjectInstanceAttributes object
   */
  attributes::SceneObjectInstanceAttributes::ptr
  createInstanceAttributesFromJSON(const io::JsonGenericValue& jCell);

  /**
   * @brief Used Internally.  Create a @ref
   * esp::metadata::attributes::SceneAOInstanceAttributes object from the
   * passed JSON doc, describing the initial state of an instance of an
   * articulated object.
   * @param jCell JSON object containing the description of the articulated
   * object instance.
   * @return the constructed @ref
   * esp::metadata::attributes::SceneAOInstanceAttributes object
   */
  attributes::SceneAOInstanceAttributes::ptr createAOInstanceAttributesFromJSON(
      const io::JsonGenericValue& jCell);

  /**
   * @brief Populate an existing @ref
   * metadata::attributes::SceneObjectInstanceAttributes from a JSON config.
   *
   * @param attributes the attributes to populate with JSON values
   * @param jCell JSON document to parse
   */
  void setAbstractObjectAttributesFromJson(
      const attributes::SceneObjectInstanceAttributes::ptr& attributes,
      const io::JsonGenericValue& jCell);

  /**
   * @brief Used Internally.  Create and configure newly-created scene instance
   * attributes with any default values, before any specific values are set.
   *
   * @param sceneInstanceHandle handle name to be assigned to scene instance
   * attributes
   * @param builtFromConfig Whether this scene is being constructed from a
   * config file or from some other source.
   * @return Newly created but unregistered SceneInstanceAttributes pointer,
   * with only default values set.
   */
  attributes::SceneInstanceAttributes::ptr initNewObjectInternal(
      const std::string& sceneInstanceHandle,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * AbstractAttributesManager-specific upon template removal, such as removing
   * a specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called @ref
   * esp::core::managedContainers::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */

  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Any scene-instance-attributes-specific resetting that needs to
   * happen on reset.
   */
  void resetFinalize() override {}

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any essential updating to the managed object
   * before registration is performed. If this updating fails, registration will
   * also fail.
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Should register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(
      CORRADE_UNUSED attributes::SceneInstanceAttributes::ptr object,
      CORRADE_UNUSED const std::string& objectHandle,
      CORRADE_UNUSED bool forceRegistration) override {
    // No pre-registration conditioning performed
    return core::managedContainers::ManagedObjectPreregistration::Success;
  }

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any final manager-related handling after
   * successfully registering an object.
   *
   * See @ref esp::attributes::managers::ObjectAttributesManager for an example.
   *
   * @param objectID the ID of the successfully registered managed object
   * @param objectHandle The name of the managed object
   */
  void postRegisterObjectHandling(
      CORRADE_UNUSED int objectID,
      CORRADE_UNUSED const std::string& objectHandle) override {}

  /**
   * @brief Name of the attributes used for the default Pbr/Ibl shader
   * configuration. This can be overwritten by the SceneInstances.
   */
  std::string defaultPbrShaderAttributesHandle_ = "";

 public:
  ESP_SMART_POINTERS(SceneInstanceAttributesManager)

};  // class SceneInstanceAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_SCENEINSTANCEATTRIBUTEMANAGER_H_
