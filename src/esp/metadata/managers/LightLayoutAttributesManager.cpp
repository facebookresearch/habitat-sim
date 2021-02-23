// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightLayoutAttributesManager.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

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
  std::string jsonAttrFileName = getFormattedJSONFileName(lightConfigName);
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
    size_t numLightConfigs = lightCell.Size();
    int count = 0;
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
      ++count;
    }
    LOG(INFO) << "LightLayoutAttributesManager::setValsFromJSONDoc : " << count
              << " of " << numLightConfigs
              << " LightInstanceAttributes created successfully and added to "
                 "LightLayoutAttributes "
              << layoutName << ".";

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
    const LightInstanceAttributes::ptr& lightAttribs,
    const io::JsonGenericValue& jsonConfig) {
  // jsonConfig here holds the JSON description for a single light attributes.
  // set position
  bool posIsSet = io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "position", [lightAttribs](const Magnum::Vector3& position) {
        lightAttribs->setPosition(position);
      });

  // set direction
  bool dirIsSet = io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "direction",
      [lightAttribs](const Magnum::Vector3& direction) {
        lightAttribs->setDirection(direction);
      });

  // set color
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "color", [lightAttribs](const Magnum::Vector3& color) {
        lightAttribs->setColor(color);
      });

  // set intensity
  io::jsonIntoSetter<double>(jsonConfig, "intensity",
                             [lightAttribs](double intensity) {
                               lightAttribs->setIntensity(intensity);
                             });

  // type of light - should map to enum values in esp::gfx::LightType
  int typeVal = -1;
  std::string tmpVal = "";
  if (io::readMember<std::string>(jsonConfig, "type", tmpVal)) {
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpVal);
    if (LightInstanceAttributes::LightTypeNamesMap.count(strToLookFor)) {
      typeVal = static_cast<int>(
          LightInstanceAttributes::LightTypeNamesMap.at(strToLookFor));
    } else {
      LOG(WARNING)
          << "LightLayoutAttributesManager::setLightInstanceValsFromJSONDoc : "
             "Type Value in json : `"
          << tmpVal
          << "` does not map to a valid "
             "LightInstanceAttributes::LightTypeNamesMap value, so "
             "defaulting LightInfo type to esp::gfx::LightType::Point.";
      typeVal = static_cast<int>(esp::gfx::LightType::Point);
    }
    lightAttribs->setType(typeVal);
  } else if (posIsSet) {
    // if no value found in attributes, attempt to infer desired type based on
    // whether position or direction were set from JSON.
    lightAttribs->setType(static_cast<int>(esp::gfx::LightType::Point));
  } else if (dirIsSet) {
    lightAttribs->setType(static_cast<int>(esp::gfx::LightType::Directional));
  }  // if nothing set by here, will default to constructor defaults

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
      io::jsonIntoSetter<Magnum::Rad>(
          spotArea, "innerConeAngle",
          [lightAttribs](Magnum::Rad innerConeAngle) {
            lightAttribs->setInnerConeAngle(innerConeAngle);
          });

      // set outer cone angle
      io::jsonIntoSetter<Magnum::Rad>(
          spotArea, "outerConeAngle",
          [lightAttribs](Magnum::Rad outerConeAngle) {
            lightAttribs->setOuterConeAngle(outerConeAngle);
          });
    }
  }  // if member spot present
}  // LightLayoutAttributesManager::setValsFromJSONDoc

LightLayoutAttributes::ptr LightLayoutAttributesManager::initNewObjectInternal(
    const std::string& handleName,
    bool) {
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
    const std::string& lightAttribsHandle,
    bool) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by LightLayoutAttributesHandle, or the next available
  // ID if not found.
  int LightLayoutAttributesID =
      this->addObjectToLibrary(lightAttribs, lightAttribsHandle);

  return LightLayoutAttributesID;
}  // LightLayoutAttributesManager::registerObjectFinalize

gfx::LightSetup LightLayoutAttributesManager::createLightSetupFromAttributes(
    const std::string& lightConfigName) {
  gfx::LightSetup res{};
  attributes::LightLayoutAttributes::ptr lightLayoutAttributes =
      this->getObjectByHandle(lightConfigName);
  if (lightLayoutAttributes != nullptr) {
    int numLightInstances = lightLayoutAttributes->getNumLightInstances();
    if (numLightInstances == 0) {
      // setup default LightInfo instances - lifted from LightSetup.cpp.
      // TODO create default attributes describing these lights?
      return gfx::LightSetup{{{1.0, 1.0, 0.0, 0.0}, {0.75, 0.75, 0.75}},
                             {{-0.5, 0.0, 1.0, 0.0}, {0.4, 0.4, 0.4}}};
    } else {
      const std::map<std::string, LightInstanceAttributes::ptr>&
          lightInstances = lightLayoutAttributes->getLightInstances();
      for (const std::pair<const std::string, LightInstanceAttributes::ptr>&
               elem : lightInstances) {
        const LightInstanceAttributes::ptr& lightAttr = elem.second;
        const int type = lightAttr->getType();
        const gfx::LightType typeEnum = static_cast<gfx::LightType>(type);
        const Magnum::Color3 color =
            lightAttr->getColor() * lightAttr->getIntensity();
        Magnum::Vector4 lightVector;
        switch (typeEnum) {
          case gfx::LightType::Point: {
            lightVector = {lightAttr->getPosition(), 1.0f};
            break;
          }
          case gfx::LightType::Directional: {
            lightVector = {lightAttr->getDirection(), 0.0f};
            break;
          }
          default: {
            LOG(INFO)
                << "LightLayoutAttributesManager::"
                   "createLightSetupFromAttributes : Enum gfx::LightType with "
                   "val "
                << type
                << " is not supported, so defaulting to gfx::LightType::Point";
            lightVector = {lightAttr->getPosition(), 1.0f};
          }
        }  // switch on type
        res.push_back({lightVector, color});
      }  // for each light instance described
    }    // if >0 light instances described
  }      // lightLayoutAttributes of requested name exists
  return res;

}  // LightLayoutAttributesManager::createLightSetupFromAttributes

}  // namespace managers
}  // namespace metadata
}  // namespace esp
