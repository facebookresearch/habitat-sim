// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_SEMANTICATTRIBUTESMANAGER_H_
#define ESP_METADATA_MANAGERS_SEMANTICATTRIBUTESMANAGER_H_

#include <utility>

#include "AttributesManagerBase.h"

#include "esp/metadata/attributes/SemanticAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

class SemanticAttributesManager
    : public AttributesManager<attributes::SemanticAttributes,
                               ManagedObjectAccess::Copy> {
 public:
  SemanticAttributesManager()
      : AttributesManager<attributes::SemanticAttributes,
                          ManagedObjectAccess::Copy>::
            AttributesManager("Semantic Attributes", "semantic_config.json") {
    this->copyConstructorMap_["SemanticAttributes"] =
        &SemanticAttributesManager::createObjectCopy<
            attributes::SemanticAttributes>;
  }  // ctor

  /**
   * @brief Creates an instance of a semantic attributes used to
   * govern which calculations are performed by the PBR shader (such as
   * Image-based Lighting and/or specific PBR layer calculations), as well as to
   * provide some user-configurable shader-wide control values.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param semanticConfigFilename The location of the Semantic Configuratioo.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the physics simulation meta data object parsed from
   * the specified configuration file.
   */
  attributes::SemanticAttributes::ptr createObject(
      const std::string& semanticConfigFilename,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::SemanticAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This will return a @ref
   * attributes::SemanticRegionAttributes object with passed handle.
   */
  attributes::SemanticRegionAttributes::ptr createEmptyRegionAttributes(
      const std::string& handle) {
    return attributes::SemanticRegionAttributes::create(handle);
  }

 protected:
  /**
   * @brief Used Internally.  Create a @ref
   * esp::metadata::attributes::SemanticRegionAttributes object from the
   * passed JSON doc.
   * @param jCell JSON object containing the description of the semantic retion
   * @return the constructed @ref
   * esp::metadata::attributes::SemanticRegionAttributes object
   */
  attributes::SemanticRegionAttributes::ptr createRegionAttributesFromJSON(
      const io::JsonGenericValue& jCell);

  /**
   * @brief Populate an existing @ref
   * metadata::attributes::SemanticRegionAttributes from a JSON config.
   *
   * @param attributes (out) the attributes to populate with JSON values
   * @param jCell JSON document to parse to populate the
   * @ref metadata::attributes::SemanticRegionAttributes
   */
  void setSemanticRegionAttributesFromJson(
      const attributes::SemanticRegionAttributes::ptr& attributes,
      const io::JsonGenericValue& jCell);

  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   * @param builtFromConfig Whether this SemanticAttributes is being built
   * from a config file (i.e. handleName is the name of a configuration file) or
   * from some other source.
   * @return Newly created but unregistered SemanticAttributes pointer,
   * with only default values set.
   */
  attributes::SemanticAttributes::ptr initNewObjectInternal(
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
   * esp::metadata::attributes::SemanticAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param semanticTemplate The attributes template.
   * @param semanticHandle The key for referencing the template in
   * the @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::SemanticAttributes::ptr semanticTemplate,
      const std::string& semanticHandle,
      CORRADE_UNUSED bool forceRegistration) override {
    // adds template to library, and returns either the ID of the existing
    // template referenced by semanticHandle, or the next
    // available ID if not found.
    int semanticAttrID =
        this->addObjectToLibrary(std::move(semanticTemplate), semanticHandle);
    return semanticAttrID;
  }  // SemanticAttributesManager::registerObjectFinalize

  /**
   * @brief Any physics-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  // instance vars

 public:
  ESP_SMART_POINTERS(SemanticAttributesManager)

};  // SemanticAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_SEMANTICATTRIBUTESMANAGER_H_
