// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AttributesBase.h"
namespace esp {
namespace metadata {
namespace attributes {

const std::map<std::string, ObjectInstanceShaderType> ShaderTypeNamesMap = {
    {"material", ObjectInstanceShaderType::Material},
    {"flat", ObjectInstanceShaderType::Flat},
    {"phong", ObjectInstanceShaderType::Phong},
    {"pbr", ObjectInstanceShaderType::PBR},
};

const std::map<std::string, SceneInstanceTranslationOrigin>
    InstanceTranslationOriginMap = {
        {"asset_local", SceneInstanceTranslationOrigin::AssetLocal},
        {"com", SceneInstanceTranslationOrigin::COM},
};

std::string getShaderTypeName(int shaderTypeVal) {
  if (shaderTypeVal <= static_cast<int>(ObjectInstanceShaderType::Unknown) ||
      shaderTypeVal >=
          static_cast<int>(ObjectInstanceShaderType::EndShaderType)) {
    return "unspecified";
  }
  // Must always be valid value
  ObjectInstanceShaderType shaderType =
      static_cast<ObjectInstanceShaderType>(shaderTypeVal);
  for (const auto& it : ShaderTypeNamesMap) {
    if (it.second == shaderType) {
      return it.first;
    }
  }
  return "unspecified";
}

std::string getTranslationOriginName(int translationOrigin) {
  if (translationOrigin <=
          static_cast<int>(SceneInstanceTranslationOrigin::Unknown) ||
      translationOrigin >=
          static_cast<int>(SceneInstanceTranslationOrigin::EndTransOrigin)) {
    return "default";
  }
  // Must always be valid value
  SceneInstanceTranslationOrigin transOrigin =
      static_cast<SceneInstanceTranslationOrigin>(translationOrigin);
  for (const auto& it : InstanceTranslationOriginMap) {
    if (it.second == transOrigin) {
      return it.first;
    }
  }
  return "default";
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
