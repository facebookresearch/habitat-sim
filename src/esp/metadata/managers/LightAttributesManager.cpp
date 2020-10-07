// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightAttributesManager.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;
namespace Cr = Corrade;

namespace esp {
namespace metadata {
using attributes::LightAttributes;
namespace managers {

LightAttributes::ptr LightAttributesManager::createObject(
    const std::string& lightConfigName,
    bool registerTemplate) {
  std::string msg;
  bool doRegister = registerTemplate;
  // create either one or more lighting attributes if an existing file name, and
  // assign these attributes to a layout upon registration, or a single
  // lightinging attributes instance if not a file name.  File based attributes
  // are automatically registered.
  std::string jsonAttrFileName =
      this->convertFilenameToJSON(lightConfigName, this->JSONTypeExt_);
  bool jsonFileExists = (this->isValidFileName(jsonAttrFileName));
  if (jsonFileExists) {
    // if exists, force registration to be true.
    doRegister = true;
  }
  // build attributes (either load multiples from file or build a single if file
  // not found)
  LightAttributes::ptr attrs =
      this->createFromJsonOrDefaultInternal(lightConfigName, msg, doRegister);

  if (nullptr != attrs) {
    LOG(INFO) << msg << " light attributes created"
              << (doRegister ? " and registered." : ".");
  }
  return attrs;
}  // PhysicsAttributesManager::createObject

LightAttributes::ptr LightAttributesManager::buildObjectFromJSONDoc(
    const std::string& layoutNameAndPath,
    const io::JsonGenericValue& jsonConfig) {
  // this will parse jsonConfig for the description of each light, and build an
  // attributes for each.
  // set file directory here, based on layout name
  std::string dirname = Cr::Utility::Directory::path(layoutNameAndPath);
  std::string filenameExt = Cr::Utility::Directory::filename(layoutNameAndPath);
  // remove ".lighting_config.json" from name
  std::string layoutName =
      Cr::Utility::Directory::splitExtension(
          Cr::Utility::Directory::splitExtension(filenameExt).first)
          .first;
  LightAttributes::ptr lightAttribs = nullptr;
  if (jsonConfig.HasMember("lights") && jsonConfig["lights"].IsObject()) {
    const auto& lightCell = jsonConfig["lights"];
    // iterate through objects
    for (rapidjson::Value::ConstMemberIterator it = lightCell.MemberBegin();
         it != lightCell.MemberEnd(); ++it) {
      // create attributes and set its name to be the tag in the JSON for the
      // individual light
      const std::string key = it->name.GetString();
      const auto& obj = it->value;
      // TODO construct name using file name prepended to key
      lightAttribs = this->initNewObjectInternal(layoutName + "_" + key);
      // set file directory here, based on layout name
      if (dirname != "") {
        lightAttribs->setFileDirectory(dirname);
      }
      // set attributes values from JSON doc
      this->setValsFromJSONDoc(lightAttribs, obj);
      // register
      this->postCreateRegister(lightAttribs, true);
      // add ref to object in appropriate layout
      this->addLightToLayout(lightAttribs->getHandle(), layoutName);
      LOG(INFO)
          << "LightAttributesManager::buildObjectFromJSONDoc : Attribs for "
          << key << " created successfully and added to layout " << layoutName
          << ".";
    }
  } else {
    LOG(WARNING)
        << "LightAttributesManager::buildObjectFromJSONDoc : " << layoutName
        << " does not contain a \"lights\" object and so no parsing was "
           "done, "
           "and a single default/blank light attributes is returned.";
    lightAttribs = this->createObject("Bad Config File " + layoutName, false);
  }
  return lightAttribs;
}  // LightAttributesManager::buildObjectFromJSONDoc

void LightAttributesManager::setValsFromJSONDoc(
    LightAttributes::ptr lightAttribs,
    const io::JsonGenericValue& jsonConfig) {
  // jsonConfig here holds the JSON description for a single light attributes.
  // set position
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "position",
      std::bind(&LightAttributes::setPosition, lightAttribs, _1));
  // set direction
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "direction",
      std::bind(&LightAttributes::setDirection, lightAttribs, _1));
  // set color
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "color",
      std::bind(&LightAttributes::setColor, lightAttribs, _1));
  // set intensity
  io::jsonIntoSetter<double>(
      jsonConfig, "intensity",
      std::bind(&LightAttributes::setIntensity, lightAttribs, _1));
  // type of light
  io::jsonIntoSetter<std::string>(
      jsonConfig, "type",
      std::bind(&LightAttributes::setType, lightAttribs, _1));
  // read spotlight params
  if (jsonConfig.HasMember("spot")) {
    if (!jsonConfig["spot"].IsObject()) {
      LOG(WARNING) << "LightAttributesManager::setValsFromJSONDoc : \"spot\" "
                      "cell in JSON config unable to be parsed to set "
                      "spotlight parameters so skipping.";
    } else {
      const auto& spotArea = jsonConfig["spot"];
      // set inner cone angle
      io::jsonIntoSetter<double>(
          spotArea, "innerConeAngle",
          std::bind(&LightAttributes::setInnerConeAngle, lightAttribs, _1));

      // set outer cone angle
      io::jsonIntoSetter<double>(
          spotArea, "outerConeAngle",
          std::bind(&LightAttributes::setOuterConeAngle, lightAttribs, _1));
    }
  }  // if member spot present
}  // LightAttributesManager::setValsFromJSONDoc

LightAttributes::ptr LightAttributesManager::initNewObjectInternal(
    const std::string& handleName) {
  attributes::LightAttributes::ptr newAttributes =
      this->constructFromDefault(handleName);
  // if no default then create new.
  if (nullptr == newAttributes) {
    newAttributes = attributes::LightAttributes::create(handleName);
  }
  return newAttributes;
}  // LightAttributesManager::initNewObjectInternal

int LightAttributesManager::registerObjectFinalize(
    LightAttributes::ptr lightAttribs,
    const std::string& lightAttribsHandle) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by lightAttributesHandle, or the next available ID
  // if not found.
  int lightAttributesID =
      this->addObjectToLibrary(lightAttribs, lightAttribsHandle);

  return lightAttributesID;
}  // LightAttributesManager::registerObjectFinalize

void LightAttributesManager::updateObjectHandleLists(
    int templateID,
    const std::string& lightHandle) {
  // lightHandle has been removed already from templateLibrary. Needs to be
  // removed from all layouts.  Note : empty layouts will remain.
  if (this->layoutsBelongedTo_.count(lightHandle) != 0) {
    // if there are layouts the lightHandle attributes belonged to, get them
    const auto& tmpList = this->layoutsBelongedTo_.at(lightHandle);
    // remove references to this lightHandle from these layouts
    for (auto layoutName : tmpList) {
      this->removeLightFromLayout(lightHandle, layoutName);
    }
    // remove set of layouts that lightHandle attributes belonged to
    this->layoutsBelongedTo_.erase(lightHandle);
  }
}  // LightAttributesManager::updateObjectHandleLists

}  // namespace managers
}  // namespace metadata
}  // namespace esp
