// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_SCENEATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_SCENEATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"
#include "esp/metadata/attributes/SceneAttributes.h"

namespace esp {
namespace metadata {

namespace managers {
class SceneAttributesManager
    : public AttributesManager<attributes::SceneAttributes> {
 public:
  SceneAttributesManager()
      : AttributesManager<attributes::SceneAttributes>::AttributesManager(
            "Scene Instance",
            "scene_instance.json") {
    buildCtorFuncPtrMaps();
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
  attributes::SceneAttributes::ptr createObject(
      const std::string& sceneInstanceHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::SceneAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

 protected:
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
   * @brief Used Internally.  Create and configure newly-created scene instance
   * attributes with any default values, before any specific values are set.
   *
   * @param sceneInstanceHandle handle name to be assigned to scene instance
   * attributes
   * @param builtFromConfig Whether this scene is being constructed from a
   * config file or from some other source.
   * @return Newly created but unregistered SceneAttributes pointer, with only
   * default values set.
   */
  attributes::SceneAttributes::ptr initNewObjectInternal(
      const std::string& sceneInstanceHandle,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called internally.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */

  void updateObjectHandleLists(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Any scene-instance-attributes-specific resetting that needs to
   * happen on reset.
   */
  void resetFinalize() override {}

  /**
   * @brief Add a @ref std::shared_ptr<attributesType> object to the
   * @ref objectLibrary_.  Verify that render and collision handles have been
   * set properly.  We are doing this since these values can be modified by the
   * user.
   *
   * @param sceneAttributes The attributes template.
   * @param sceneAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of the registered template.
   */
  int registerObjectFinalize(attributes::SceneAttributes::ptr sceneAttributes,
                             const std::string& sceneAttributesHandle,
                             CORRADE_UNUSED bool forceRegistration) override;

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<PhysicsSceneAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["SceneAttributes"] =
        &SceneAttributesManager::createObjectCopy<attributes::SceneAttributes>;
  }  // SceneAttributesManager::buildCtorFuncPtrMaps

  /**
   * @brief This function is meaningless for this manager's ManagedObjects.
   * @param handle Ignored.
   * @return false
   */
  virtual bool isValidPrimitiveAttributes(
      CORRADE_UNUSED const std::string& handle) override {
    return false;
  }

 public:
  ESP_SMART_POINTERS(SceneAttributesManager)

};  // class SceneAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_SCENEATTRIBUTEMANAGER_H_
