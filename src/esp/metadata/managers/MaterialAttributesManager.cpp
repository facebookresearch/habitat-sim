// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialAttributesManager.h"
#include "esp/io/Io.h"
#include "esp/io/Json.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
using attributes::MaterialAttributes;
namespace managers {
using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

MaterialAttributes::ptr MaterialAttributesManager::createObject(
    const std::string& materialConfigName,
    bool registerTemplate) {
  std::string msg;
  bool doRegister = registerTemplate;
  // File based attributes are automatically registered.
  std::string jsonAttrFileName = getFormattedJSONFileName(materialConfigName);
  bool jsonFileExists = (Cr::Utility::Path::exists(jsonAttrFileName));
  if (jsonFileExists) {
    // if exists, force registration to be true.
    doRegister = true;
  }
  // build attributes
  MaterialAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      materialConfigName, msg, doRegister);

  if (nullptr != attrs) {
    ESP_DEBUG() << msg << "material attributes created"
                << (doRegister ? "and registered." : ".");
  }
  return attrs;
}  // MaterialAttributesManager::createObject

void MaterialAttributesManager::setValsFromJSONDoc(
    attributes::MaterialAttributes::ptr materialAttribs,
    const io::JsonGenericValue& jsonConfig) {
}  // MaterialAttributesManager::setValsFromJSONDoc

MaterialAttributes::ptr MaterialAttributesManager::initNewObjectInternal(
    const std::string& handleName,
    bool) {
  attributes::MaterialAttributes::ptr newAttributes =
      this->constructFromDefault(handleName);
  // if no default then create new.
  bool createNewAttributes = (nullptr == newAttributes);
  if (createNewAttributes) {
    newAttributes = attributes::MaterialAttributes::create(handleName);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);
  return newAttributes;
}  // MaterialAttributesManager::initNewObjectInternal

int MaterialAttributesManager::registerObjectFinalize(
    MaterialAttributes::ptr materialAttribs,
    const std::string& materialAttribsHandle,
    bool) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by MaterialAttributesHandle, or the next available
  // ID if not found.
  int MaterialAttributesID =
      this->addObjectToLibrary(materialAttribs, materialAttribsHandle);

  return MaterialAttributesID;
}  // MaterialAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
