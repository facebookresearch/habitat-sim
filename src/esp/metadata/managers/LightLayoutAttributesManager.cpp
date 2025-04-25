// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightLayoutAttributesManager.h"

#include <utility>
#include "esp/io/Io.h"
#include "esp/io/Json.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
using attributes::LightInstanceAttributes;
using attributes::LightLayoutAttributes;
namespace managers {
using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

LightLayoutAttributes::ptr LightLayoutAttributesManager::createObject(
    const std::string& lightConfigName,
    bool registerTemplate) {
  std::string msg;
  bool doRegister = registerTemplate;
  // File based attributes are automatically registered.
  std::string jsonAttrFileName = getFormattedJSONFileName(lightConfigName);
  bool jsonFileExists = (Cr::Utility::Path::exists(jsonAttrFileName));
  if (jsonFileExists) {
    // if exists, force registration to be true.
    doRegister = true;
  }
  // build attributes
  LightLayoutAttributes::ptr attrs =
      this->createFromJsonOrDefaultInternal(lightConfigName, msg, doRegister);

  if (nullptr != attrs) {
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << msg << " light layout attributes created"
        << (doRegister ? " and registered." : ".");
  }
  return attrs;
}  // PhysicsAttributesManager::createObject

void LightLayoutAttributesManager::setValsFromJSONDocInternal(
    attributes::LightLayoutAttributes::ptr lightAttribs,
    const io::JsonGenericValue& jsonConfig) {
  const std::string layoutNameAndPath = lightAttribs->getHandle();
  // this will parse jsonConfig for the description of each light, and build an
  // attributes for each.
  // set file directory here, based on layout name
  std::string filenameExt =
      Cr::Utility::Path::split(layoutNameAndPath).second();
  // remove ".lighting_config.json" from name
  std::string layoutName =
      Cr::Utility::Path::splitExtension(
          Cr::Utility::Path::splitExtension(filenameExt).first())
          .first();

  // check if positive scaling is set
  bool hasPosScale = io::jsonIntoSetter<double>(
      jsonConfig, "positive_intensity_scale",
      [lightAttribs](double positive_intensity_scale) {
        lightAttribs->setPositiveIntensityScale(positive_intensity_scale);
      });
  // check if positive scaling is set
  bool hasNegScale = io::jsonIntoSetter<double>(
      jsonConfig, "negative_intensity_scale",
      [lightAttribs](double negative_intensity_scale) {
        lightAttribs->setNegativeIntensityScale(negative_intensity_scale);
      });

  // scaling values are required for light instance processing

  io::JsonGenericValue::ConstMemberIterator jsonIter =
      jsonConfig.FindMember("lights");

  bool hasLights =
      (jsonIter != jsonConfig.MemberEnd()) && (jsonIter->value.IsObject());

  if (hasLights) {
    const auto& lightCell = jsonIter->value;
    int count = 0;
    // iterate through objects
    for (rapidjson::Value::ConstMemberIterator it = lightCell.MemberBegin();
         it != lightCell.MemberEnd(); ++it) {
      // create attributes and set its name to be the tag in the JSON for the
      // individual light
      const std::string key = it->name.GetString();
      const auto& obj = it->value;
      // TODO construct name using file name prepended to key
      LightInstanceAttributes::ptr lightInstanceAttribs =
          LightInstanceAttributes::create(key);
      // set file directory here, based on layout's directory
      lightInstanceAttribs->setFileDirectory(lightAttribs->getFileDirectory());
      // set attributes values from JSON doc
      this->setLightInstanceValsFromJSONDoc(lightInstanceAttribs, obj);

      // check for user defined attributes
      this->parseUserDefinedJsonVals(lightInstanceAttribs, obj);
      // add ref to object in appropriate layout
      lightAttribs->addLightInstance(std::move(lightInstanceAttribs));
      ++count;
    }
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "" << count
        << " LightInstanceAttributes created successfully and added to "
           "LightLayoutAttributes `"
        << layoutName << "`.";
  }
  // check for user defined attributes at main attributes level
  bool hasUserConfig = this->parseUserDefinedJsonVals(lightAttribs, jsonConfig);

  if (hasLights || hasUserConfig || hasNegScale || hasPosScale) {
    // register if anything worth registering was found
    this->postCreateRegister(std::move(lightAttribs), true);
  } else {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "`" << layoutName
        << "` does not contain a \"lights\" object or a valid "
           "\"user_defined\" object and so no parsing was "
           "done and this attributes is not being saved.";
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

  // set frame of reference for light transformation
  std::string posMdleVal = "global";
  std::string tmpPosMdleVal = "";
  if (io::readMember<std::string>(jsonConfig, "position_model",
                                  tmpPosMdleVal)) {
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpPosMdleVal);
    if (attributes::LightPositionNamesMap.count(strToLookFor) != 0u) {
      posMdleVal = std::move(tmpPosMdleVal);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "'position_model' Value in JSON : `" << posMdleVal
          << "` does not map to a valid "
             "attributes::LightPositionNamesMap value, so "
             "defaulting LightInfo position model to "
             "esp::gfx::LightPositionModel::Global.";
    }
    lightAttribs->setPositionModel(posMdleVal);
  }  // position model

  // type of light - should map to enum values in esp::gfx::LightType
  std::string specifiedTypeVal = "point";
  std::string tmpTypeVal = "";
  if (io::readMember<std::string>(jsonConfig, "type", tmpTypeVal)) {
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpTypeVal);
    if (strToLookFor == "spot") {
      // TODO remove this if block to support spot lights
      ESP_WARNING()
          << "Type spotlight specified in JSON not currently supported, so "
             "defaulting LightInfo type to esp::gfx::LightType::Point.";
    } else if (attributes::LightTypeNamesMap.count(strToLookFor) != 0u) {
      specifiedTypeVal = std::move(tmpTypeVal);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Type Value in JSON : `" << tmpTypeVal
          << "` does not map to a valid "
             "attributes::LightTypeNamesMap value, so "
             "defaulting LightInfo type to esp::gfx::LightType::Point.";
    }
    lightAttribs->setType(specifiedTypeVal);
  } else if (posIsSet) {
    // if no value found in attributes, attempt to infer desired type based on
    // whether position or direction were set from JSON.
    lightAttribs->setType(
        attributes::getLightTypeName(esp::gfx::LightType::Point));
  } else if (dirIsSet) {
    lightAttribs->setType(
        attributes::getLightTypeName(esp::gfx::LightType::Directional));
  }  // if nothing set by here, will default to constructor defaults

  // if the user specifies a type, we will assume that type overrides any
  // inferred light type based on vector position/direction provided.  If the
  // vector provided does not match the type specified, we copy the vector into
  // the appropriate location.
  if ((specifiedTypeVal ==
       attributes::getLightTypeName(esp::gfx::LightType::Directional)) &&
      (posIsSet) && !(dirIsSet)) {
    // position set, direction absent, but directional type explicitly specified
    lightAttribs->setDirection(lightAttribs->getPosition());
  } else if ((specifiedTypeVal ==
              attributes::getLightTypeName(esp::gfx::LightType::Point)) &&
             (dirIsSet) && !(posIsSet)) {
    // direction set, position absent, but point type explicitly specified
    lightAttribs->setPosition(lightAttribs->getDirection());
  }

  // read spotlight params
  io::JsonGenericValue::ConstMemberIterator jsonIter =
      jsonConfig.FindMember("spot");
  if (jsonIter != jsonConfig.MemberEnd()) {
    if (!jsonIter->value.IsObject()) {
      // TODO prune NOTE: component when spotlights are supported
      ESP_WARNING()
          << "\"spot\" cell in JSON config unable to be "
             "parsed to set spotlight parameters so skipping.  NOTE : "
             "Spotlights not currently supported, so cone angle values are "
             "ignored and light will be created as a point light.";
    } else {
      // sets values in light instance subconfig "spot"
      const auto& spotArea = jsonIter->value;
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
  }  // if JSON object 'spot' present
}  // LightLayoutAttributesManager::setLightInstanceValsFromJSONDoc

LightLayoutAttributes::ptr LightLayoutAttributesManager::initNewObjectInternal(
    const std::string& handleName,
    bool) {
  attributes::LightLayoutAttributes::ptr newAttributes =
      this->constructFromDefault(handleName);
  // if no default then create new.
  if (nullptr == newAttributes) {
    newAttributes = attributes::LightLayoutAttributes::create(handleName);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);
  return newAttributes;
}  // LightLayoutAttributesManager::initNewObjectInternal

gfx::LightSetup LightLayoutAttributesManager::createLightSetupFromAttributes(
    const std::string& lightConfigName) {
  // if passing no-lighting key, return empty light setup
  if (lightConfigName == NO_LIGHT_KEY) {
    ESP_VERY_VERBOSE()
        << "`No-lights` lighting key specified so using empty light setup.";
    return esp::gfx::LightSetup{};
  }
  // if passing DEFAULT_LIGHTING_KEY key, use default lights for light setup
  if (lightConfigName == DEFAULT_LIGHTING_KEY) {
    // DEFAULT_LIGHTING_KEY is specified as the empty string. This would
    // generate an error if the attributes manager was queried with an empty
    // string
    ESP_VERY_VERBOSE() << "`Default-lighting` key specified so using "
                          "Habitat-Sim-specified default light setup.";
    return gfx::getDefaultLights();
  }

  gfx::LightSetup res{};
  attributes::LightLayoutAttributes::ptr lightLayoutAttributes =
      this->getObjectByHandle(lightConfigName);
  if (lightLayoutAttributes != nullptr) {
    double posIntensityScale =
        lightLayoutAttributes->getPositiveIntensityScale();
    double negIntensityScale =
        lightLayoutAttributes->getNegativeIntensityScale();
    int numLightInstances = lightLayoutAttributes->getNumLightInstances();
    if (numLightInstances == 0) {
      // setup default LightInfo instances - lifted from LightSetup.cpp.
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Lighting configuration specified by key :`" << lightConfigName
          << "` specifies no light instances so using Habitat-Sim-specified "
             "default light setup.";
      return gfx::getDefaultLights();
    } else {
      auto lightInstances = lightLayoutAttributes->getLightInstances();
      for (const LightInstanceAttributes::cptr& lightAttr : lightInstances) {
        const gfx::LightType typeEnum = lightAttr->getType();
        const gfx::LightPositionModel posModelEnum =
            lightAttr->getPositionModel();
        const Magnum::Color3 color =
            lightAttr->getColor() *
            (lightAttr->getIntensity() > 0 ? posIntensityScale
                                           : negIntensityScale) *
            lightAttr->getIntensity();
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
            ESP_DEBUG() << "Enum gfx::LightType with val"
                        << attributes::getLightTypeName(typeEnum)
                        << "is not supported, so defaulting to "
                           "gfx::LightType::Point (point light)";
            lightVector = {lightAttr->getPosition(), 1.0f};
          }
        }  // switch on type

        res.push_back({lightVector, color, posModelEnum});
        //{.vector = lightVector, .color = color, .model = posModelEnum});
      }  // for each light instance described
    }    // if >0 light instances described
  }      // lightLayoutAttributes of requested name exists
  return res;

}  // LightLayoutAttributesManager::createLightSetupFromAttributes

}  // namespace managers
}  // namespace metadata
}  // namespace esp
