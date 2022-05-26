// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_MATERIALATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_MATERIALATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

#include "esp/metadata/attributes/MaterialAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

class MaterialAttributesManager
    : public AttributesManager<attributes::MaterialAttributes,
                               ManagedObjectAccess::Copy> {
 public:
  MaterialAttributesManager()
      : AttributesManager<attributes::MaterialAttributes,
                          ManagedObjectAccess::Copy>::
            AttributesManager("Material", "material_config.json") {
    // build this manager's copy constructor map
    this->copyConstructorMap_["MaterialAttributes"] =
        &MaterialAttributesManager::createObjectCopy<
            attributes::MaterialAttributes>;
  }

  /**
   * @brief Creates one or more instances of MaterialAttributes based on the
   * whether @p materialConfigName is a file or a not.  If it is a file it will
   * consider the contents of that file a layout and will use the file name as
   * the layout name and load all the attributes described and assign them to
   * that layout.  File-based loads will automatically register, regardless of
   * what @p registerTemplate is.
   *
   * If a template/layout exists with this handle, this existing template/layout
   * will be overwritten with the newly created one if registerTemplate is true.
   *
   * @param materialConfigName The configuration file to parse, or the name of
   * the single material's attributes to create.
   * @param registerTemplate whether to add this template to the library.
   * Defaults to false - overridden if @p materialConfigName is a JSON file.
   * @return a reference to the created material attributes.
   */
  attributes::MaterialAttributes::ptr createObject(
      const std::string& materialConfigName,
      bool registerTemplate = false) override;

  /**
   * @brief Function to take an existing MaterialAttributes and set its
   * values from passed json config file.
   * @param materialAttribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::MaterialAttributes::ptr materialAttribs,
                          const io::JsonGenericValue& jsonConfig) override;

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created attributes
   * with any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes\
   * @param builtFromConfig whether this MaterialAttributes is being
   * built from a config file, or from some other source (i.e. handleName
   * contains config file name)
   * @return Newly created but unregistered MaterialAttributes pointer,
   * with only default values set.
   */
  attributes::MaterialAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal.  This should only be
   * called from @ref esp::core::managedContainers::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::MaterialAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param materialAttributesTemplate The attributes template.
   * @param materialAttributesHandle The key for referencing the template in
   * the
   * @ref objectLibrary_.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::MaterialAttributes::ptr materialAttributesTemplate,
      const std::string& materialAttributesHandle,
      CORRADE_UNUSED bool forceRegistration) override;

  /**
   * @brief Any MaterialAttributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

 public:
  ESP_SMART_POINTERS(MaterialAttributesManager)

};  // MaterialAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_MATERIALATTRIBUTEMANAGER_H_
