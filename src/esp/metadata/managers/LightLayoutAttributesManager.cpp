// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightLayoutAttributesManager.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;
namespace Cr = Corrade;

namespace esp {
namespace metadata {
using attributes::LightInstanceAttributes;
using attributes::LightLayoutAttributes;
namespace managers {

LightLayoutAttributes::ptr LightLayoutAttributesManager::createObject(
    const std::string& lightConfigName,
    bool registerTemplate) {
  std::string msg;
  bool doRegister = registerTemplate;
  // File based attributes are automatically registered.
  std::string jsonAttrFileName =
      this->convertFilenameToJSON(lightConfigName, this->JSONTypeExt_);
  bool jsonFileExists = (this->isValidFileName(jsonAttrFileName));
  if (jsonFileExists) {
    // if exists, force registration to be true.
    doRegister = true;
  }
  // build attributes
  LightLayoutAttributes::ptr attrs =
      this->createFromJsonOrDefaultInternal(lightConfigName, msg, doRegister);

  if (nullptr != attrs) {
    LOG(INFO) << msg << " light layout attributes created"
              << (doRegister ? " and registered." : ".");
  }
  return attrs;
}  // PhysicsAttributesManager::createObject

void LightLayoutAttributesManager::setValsFromJSONDoc(
    attributes::LightLayoutAttributes::ptr lightAttribs,
    const io::JsonGenericValue& jsonConfig) {
  const std::string layoutNameAndPath = lightAttribs->getHandle();
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
  LightInstanceAttributes::ptr lightInstanceAttribs = nullptr;
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
      lightInstanceAttribs = LightInstanceAttributes::create(key);
      // set file directory here, based on layout's directory
      lightInstanceAttribs->setFileDirectory(lightAttribs->getFileDirectory());
      // set attributes values from JSON doc
      this->setLightInstanceValsFromJSONDoc(lightInstanceAttribs, obj);

      // add ref to object in appropriate layout
      lightAttribs->addLightInstance(lightInstanceAttribs);

      LOG(INFO) << "LightLayoutAttributesManager::setValsFromJSONDoc : "
                   "LightInstanceAttributes "
                << lightInstanceAttribs->getHandle()
                << " created successfully and added to LightLayoutAttributes "
                << layoutName << ".";
    }
    // register
    this->postCreateRegister(lightAttribs, true);

  } else {
    LOG(WARNING)
        << "LightLayoutAttributesManager::setValsFromJSONDoc : " << layoutName
        << " does not contain a \"lights\" object and so no parsing was "
           "done.";
  }
}  // LightLayoutAttributesManager::setValsFromJSONDoc

void LightLayoutAttributesManager::setLightInstanceValsFromJSONDoc(
    LightInstanceAttributes::ptr lightAttribs,
    const io::JsonGenericValue& jsonConfig) {
  // jsonConfig here holds the JSON description for a single light attributes.
  // set position
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "position",
      std::bind(&LightInstanceAttributes::setPosition, lightAttribs, _1));
  // set direction
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "direction",
      std::bind(&LightInstanceAttributes::setDirection, lightAttribs, _1));
  // set color
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "color",
      std::bind(&LightInstanceAttributes::setColor, lightAttribs, _1));
  // set intensity
  io::jsonIntoSetter<double>(
      jsonConfig, "intensity",
      std::bind(&LightInstanceAttributes::setIntensity, lightAttribs, _1));
  // type of light
  io::jsonIntoSetter<std::string>(
      jsonConfig, "type",
      std::bind(&LightInstanceAttributes::setType, lightAttribs, _1));
  // read spotlight params
  if (jsonConfig.HasMember("spot")) {
    if (!jsonConfig["spot"].IsObject()) {
      LOG(WARNING)
          << "LightLayoutAttributesManager::setValsFromJSONDoc : \"spot\" "
             "cell in JSON config unable to be parsed to set "
             "spotlight parameters so skipping.";
    } else {
      const auto& spotArea = jsonConfig["spot"];
      // set inner cone angle
      io::jsonIntoSetter<double>(
          spotArea, "innerConeAngle",
          std::bind(&LightInstanceAttributes::setInnerConeAngle, lightAttribs,
                    _1));

      // set outer cone angle
      io::jsonIntoSetter<double>(
          spotArea, "outerConeAngle",
          std::bind(&LightInstanceAttributes::setOuterConeAngle, lightAttribs,
                    _1));
    }
  }  // if member spot present
}  // LightLayoutAttributesManager::setValsFromJSONDoc

LightLayoutAttributes::ptr LightLayoutAttributesManager::initNewObjectInternal(
    const std::string& handleName,
    bool builtFromConfig) {
  attributes::LightLayoutAttributes::ptr newAttributes =
      this->constructFromDefault(handleName);
  // if no default then create new.
  if (nullptr == newAttributes) {
    newAttributes = attributes::LightLayoutAttributes::create(handleName);
  }
  return newAttributes;
}  // LightLayoutAttributesManager::initNewObjectInternal

int LightLayoutAttributesManager::registerObjectFinalize(
    LightLayoutAttributes::ptr lightAttribs,
    const std::string& lightAttribsHandle) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by LightLayoutAttributesHandle, or the next available
  // ID if not found.
  int LightLayoutAttributesID =
      this->addObjectToLibrary(lightAttribs, lightAttribsHandle);

  return LightLayoutAttributesID;
}  // LightLayoutAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
