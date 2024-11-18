// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_AOATTRIBUTESMANAGER_H_
#define ESP_METADATA_MANAGERS_AOATTRIBUTESMANAGER_H_

#include <utility>

#include "AbstractAttributesManager.h"

#include "esp/metadata/attributes/ArticulatedObjectAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {
using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

class AOAttributesManager
    : public AbstractAttributesManager<attributes::ArticulatedObjectAttributes,
                                       ManagedObjectAccess::Copy> {
 public:
  AOAttributesManager()
      : AbstractAttributesManager<attributes::ArticulatedObjectAttributes,
                                  ManagedObjectAccess::Copy>::
            AbstractAttributesManager("Articulated Object", "ao_config.json") {
    this->copyConstructorMap_["ArticulatedObjectAttributes"] =
        &AOAttributesManager::createObjCopyCtorMapEntry<
            attributes::ArticulatedObjectAttributes>;
  }  // ctor

  /**
   * @brief Creates an instance of an articulated object template described by
   * the passed string. For @ref esp::metadata::attributes::ArticulatedObjectAttributes
   * templates, this is a config file name. Parses global and Habitat-specific
   * articulated object parameters (such as render asset and semantic id) from
   * the specified configuration file.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param aoConfigFilename The configuration file to parse.
   * @param registerTemplate Whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the articulated object attributes parsed from the
   * specified configuration file.
   */
  attributes::ArticulatedObjectAttributes::ptr createObject(
      const std::string& aoConfigFilename,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::ArticulatedObjectAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This is to be deprecated. Provide a map of the articulated object
   * model filenames (.urdf) that have been referenced in the Scene Dataset via
   * paths, either .urdf or .json. To be removed in favor of directly accessing
   * these values through the AOAttributesMaanager.
   */
  std::map<std::string, std::string> getArticulatedObjectModelFilenames() const;

  /**
   * @brief This function will be called to finalize attributes' paths before
   * registration, moving fully qualified paths to the appropriate hidden
   * attribute fields. This can also be called without registration to make sure
   * the paths specified in an attributes are properly configured.
   * @param attributes The attributes to be filtered.
   */
  void finalizeAttrPathsBeforeRegister(
      const attributes::ArticulatedObjectAttributes::ptr& attributes)
      const override;

 protected:
  /**
   * @brief Parse Marker_sets object in json, if present.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   * @return true if tag is found, of appropriate configuration, and holds
   * actual values.
   */
  bool parseMarkerSets(
      const attributes::ArticulatedObjectAttributes::ptr& attribs,
      const io::JsonGenericValue& jsonConfig) const {
    // check for the existing of markersets
    bool hasMarkersets =
        this->parseSubconfigJsonVals("marker_sets", attribs, jsonConfig);
    if (hasMarkersets) {
      // Cast "marker_sets" Configuration to MarkerSets object and rekey all
      // markers to make keys consistent while preserving the natural order of
      // their original keys.
      attribs->rekeyAllMarkerSets();
    }
    return hasMarkersets;
  }

  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   * @param builtFromConfig Whether this ArticulatedObjectAttributes is being
   * built from a config file (i.e. handleName is the name of a configuration
   * file) or from some other source (such as a default being built for an
   * existing URDF that otherwise had no specified JSON config).
   * @return Newly created but unregistered ArticulatedObjectAttributes pointer,
   * with only default values set.
   */
  attributes::ArticulatedObjectAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * AbstractAttributesManager-specific upon template removal, such as removing
   * a specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called @ref
   * esp::core::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief This method will perform any essential updating to the managed
   * object before registration is performed. If this updating fails,
   * registration will also fail.
   *
   * @param AOAttributesTemplate The attributes template.
   * @param AOAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(
      attributes::ArticulatedObjectAttributes::ptr AOAttributesTemplate,
      const std::string& AOAttributesHandle,
      CORRADE_UNUSED bool) override;

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
   * @brief Any articulated-object-attributes-specific resetting that needs to
   * happen on reset.
   */
  void resetFinalize() override {}

  // instance vars

 public:
  ESP_SMART_POINTERS(AOAttributesManager)

};  // AOAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_AOATTRIBUTESMANAGER_H_
