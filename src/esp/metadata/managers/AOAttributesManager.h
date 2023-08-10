// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_AOATTRIBUTESMANAGER_H_
#define ESP_METADATA_MANAGERS_AOATTRIBUTESMANAGER_H_

#include <utility>

#include "AttributesManagerBase.h"

#include "esp/metadata/attributes/ArticulatedObjectAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {
using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

class AOAttributesManager
    : public AttributesManager<attributes::ArticulatedObjectAttributes,
                               ManagedObjectAccess::Copy> {
 public:
  AOAttributesManager()
      : AttributesManager<
            attributes::ArticulatedObjectAttributes,
            ManagedObjectAccess::Copy>::AttributesManager("Articulated Object",
                                                          "ao_config.json") {
    this->copyConstructorMap_["ArticulatedObjectAttributes"] =
        &AOAttributesManager::createObjectCopy<
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
   * @param registerTemplate whether to add this template to the library.
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

 protected:
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
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
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
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::ArticulatedObjectAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param AOAttributesTemplate The attributes template.
   * @param AOAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::ArticulatedObjectAttributes::ptr AOAttributesTemplate,
      const std::string& AOAttributesHandle,
      CORRADE_UNUSED bool) override;

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
