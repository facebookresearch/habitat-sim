// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AttributesEnumMaps.h"
#include "esp/physics/PhysicsObjectBase.h"
namespace esp {

namespace metadata {
namespace attributes {

// All keys must be lowercase
const std::map<std::string, esp::gfx::LightType> LightTypeNamesMap = {
    {"point", esp::gfx::LightType::Point},
    {"directional", esp::gfx::LightType::Directional},
    {"spot", esp::gfx::LightType::Spot}};

std::string getLightTypeName(esp::gfx::LightType lightTypeEnum) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // LightTypeNamesMap
  if (lightTypeEnum < esp::gfx::LightType::Point ||
      lightTypeEnum > esp::gfx::LightType::Spot) {
    return "point";
  }
  // Must always be valid value
  for (const auto& it : LightTypeNamesMap) {
    if (it.second == lightTypeEnum) {
      return it.first;
    }
  }
  return "point";
}

const std::map<std::string, esp::gfx::LightPositionModel>
    LightPositionNamesMap = {{"global", esp::gfx::LightPositionModel::Global},
                             {"camera", esp::gfx::LightPositionModel::Camera},
                             {"object", esp::gfx::LightPositionModel::Object}};

std::string getLightPositionModelName(
    esp::gfx::LightPositionModel lightPositionEnum) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // LightPositionNamesMap
  if (lightPositionEnum < esp::gfx::LightPositionModel::Camera ||
      lightPositionEnum > esp::gfx::LightPositionModel::Object) {
    return "global";
  }
  // Must always be valid value
  for (const auto& it : LightPositionNamesMap) {
    if (it.second == lightPositionEnum) {
      return it.first;
    }
  }
  return "global";
}

const std::map<std::string, ArticulatedObjectBaseType> AOBaseTypeMap{
    {"unspecified", ArticulatedObjectBaseType::Unspecified},
    {"free", ArticulatedObjectBaseType::Free},
    {"fixed", ArticulatedObjectBaseType::Fixed},
};

std::string getAOBaseTypeName(ArticulatedObjectBaseType aoBaseType) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // ShaderTypeNamesMap
  if (aoBaseType <= ArticulatedObjectBaseType::Unspecified ||
      aoBaseType >= ArticulatedObjectBaseType::EndAOBaseType) {
    return "unspecified";
  }
  // Must always be valid value
  for (const auto& it : AOBaseTypeMap) {
    if (it.second == aoBaseType) {
      return it.first;
    }
  }
  return "unspecified";

}  // getAOBaseTypeName

const std::map<std::string, ArticulatedObjectInertiaSource> AOInertiaSourceMap{
    {"unspecified", ArticulatedObjectInertiaSource::Unspecified},
    {"computed", ArticulatedObjectInertiaSource::Computed},
    {"urdf", ArticulatedObjectInertiaSource::URDF},
};

std::string getAOInertiaSourceName(
    ArticulatedObjectInertiaSource aoInertiaSource) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // AOInertiaSourceMap
  if (aoInertiaSource <= ArticulatedObjectInertiaSource::Unspecified ||
      aoInertiaSource >= ArticulatedObjectInertiaSource::EndAOInertiaSource) {
    return "unspecified";
  }
  // Must always be valid value
  for (const auto& it : AOInertiaSourceMap) {
    if (it.second == aoInertiaSource) {
      return it.first;
    }
  }
  return "unspecified";

}  // getAOInertiaSourceName

const std::map<std::string, ArticulatedObjectLinkOrder> AOLinkOrderMap{
    {"unspecified", ArticulatedObjectLinkOrder::Unspecified},
    {"urdf_order", ArticulatedObjectLinkOrder::URDFOrder},
    {"tree_traversal", ArticulatedObjectLinkOrder::TreeTraversal},

};

std::string getAOLinkOrderName(ArticulatedObjectLinkOrder aoLinkOrder) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // AOLinkOrderMap
  if (aoLinkOrder <= ArticulatedObjectLinkOrder::Unspecified ||
      aoLinkOrder >= ArticulatedObjectLinkOrder::EndAOLinkOrder) {
    return "unspecified";
  }
  // Must always be valid value
  for (const auto& it : AOLinkOrderMap) {
    if (it.second == aoLinkOrder) {
      return it.first;
    }
  }
  return "unspecified";

}  // getAOLinkOrderName

const std::map<std::string, ArticulatedObjectRenderMode> AORenderModesMap = {
    {"unspecified", ArticulatedObjectRenderMode::Unspecified},
    {"default", ArticulatedObjectRenderMode::Default},
    {"skin", ArticulatedObjectRenderMode::Skin},
    {"link_visuals", ArticulatedObjectRenderMode::LinkVisuals},
    {"none", ArticulatedObjectRenderMode::None},
    {"both", ArticulatedObjectRenderMode::Both},
};

std::string getAORenderModeName(ArticulatedObjectRenderMode aoRenderMode) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // AORenderModesMap
  if (aoRenderMode <= ArticulatedObjectRenderMode::Unspecified ||
      aoRenderMode >= ArticulatedObjectRenderMode::EndAORenderMode) {
    return "unspecified";
  }
  // Must always be valid value
  for (const auto& it : AORenderModesMap) {
    if (it.second == aoRenderMode) {
      return it.first;
    }
  }
  return "unspecified";
}  // getAORenderModeName

const std::map<std::string, ObjectInstanceShaderType> ShaderTypeNamesMap = {
    {"unspecified", ObjectInstanceShaderType::Unspecified},
    {"material", ObjectInstanceShaderType::Material},
    {"flat", ObjectInstanceShaderType::Flat},
    {"phong", ObjectInstanceShaderType::Phong},
    {"pbr", ObjectInstanceShaderType::PBR},
};

std::string getShaderTypeName(ObjectInstanceShaderType shaderTypeVal) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // ShaderTypeNamesMap
  if (shaderTypeVal <= ObjectInstanceShaderType::Unspecified ||
      shaderTypeVal >= ObjectInstanceShaderType::EndShaderType) {
    return "unspecified";
  }
  // Must always be valid value
  for (const auto& it : ShaderTypeNamesMap) {
    if (it.second == shaderTypeVal) {
      return it.first;
    }
  }
  return "unspecified";
}  // getShaderTypeName

const std::map<std::string, SceneInstanceTranslationOrigin>
    InstanceTranslationOriginMap = {
        {"asset_local", SceneInstanceTranslationOrigin::AssetLocal},
        {"com", SceneInstanceTranslationOrigin::COM},
};

std::string getTranslationOriginName(
    SceneInstanceTranslationOrigin translationOrigin) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // InstanceTranslationOriginMap
  if ((translationOrigin <= SceneInstanceTranslationOrigin::Unknown) ||
      (translationOrigin >= SceneInstanceTranslationOrigin::EndTransOrigin)) {
    return "default";
  }
  // Must always be valid value
  for (const auto& it : InstanceTranslationOriginMap) {
    if (it.second == translationOrigin) {
      return it.first;
    }
  }
  return "default";
}  // getTranslationOriginName

// All keys must be lowercase
const std::map<std::string, esp::physics::MotionType> MotionTypeNamesMap = {
    {"static", esp::physics::MotionType::STATIC},
    {"kinematic", esp::physics::MotionType::KINEMATIC},
    {"dynamic", esp::physics::MotionType::DYNAMIC},
};

std::string getMotionTypeName(esp::physics::MotionType motionTypeEnum) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // MotionTypeNamesMap
  if ((motionTypeEnum <= esp::physics::MotionType::UNDEFINED) ||
      (motionTypeEnum > esp::physics::MotionType::DYNAMIC)) {
    return "undefined";
  }
  // Must always be valid value
  for (const auto& it : MotionTypeNamesMap) {
    if (it.second == motionTypeEnum) {
      return it.first;
    }
  }
  return "undefined";
}  // getMotionTypeName

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
