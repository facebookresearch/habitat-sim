// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(
    const std::shared_ptr<core::config::Configuration>& configPtr,
    JsonAllocator& allocator) {
  JsonGenericValue jsonObj(rapidjson::kObjectType);
  // iterate through all values
  // pair of begin/end const iterators to all values
  auto valIterPair = configPtr->getValuesIterator();
  auto valBegin = valIterPair.first;
  auto valEnd = valIterPair.second;
  for (auto& valIter = valIterPair.first; valIter != valIterPair.second;
       ++valIter) {
    if (valIter->second.isValid()) {
      // make sure value is legal
      rapidjson::GenericStringRef<char> name{valIter->first.c_str()};
      auto cfgValJson = toJsonValue(valIter->second, allocator);
      jsonObj.AddMember(name, cfgValJson, allocator);
      // io::addMember(jsonObj, name, valIter->second, allocator);
    } else {
      ESP_WARNING() << "Unitialized ConfigValue in Configuration @ key ["
                    << valIter->first << "]";
    }
  }  // iterate through all values

  // iterate through subconfigs
  // pair of begin/end const iterators to all subconfigurations
  auto cfgIterPair = configPtr->getSubconfigIterator();
  for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
       ++cfgIter) {
    rapidjson::GenericStringRef<char> name{cfgIter->first.c_str()};
    JsonGenericValue subObj = toJsonValue(cfgIter->second, allocator);
    jsonObj.AddMember(name, subObj, allocator);
    // io::addMember(jsonObj, name, subObj, allocator);
  }  // iterate through all configurations

  return jsonObj;
}  // toJsonValue<core::config::Configuration>

JsonGenericValue toJsonValue(const core::config::ConfigValue& cfgVal,
                             JsonAllocator& allocator) {
  // unknown is checked before this function is called, so does not need support
  switch (cfgVal.getType()) {
    case core::config::ConfigStoredType::Boolean: {
      return toJsonValue(cfgVal.get<bool>(), allocator);
    }
    case core::config::ConfigStoredType::Integer: {
      return toJsonValue(cfgVal.get<int>(), allocator);
    }
    case core::config::ConfigStoredType::Double: {
      return toJsonValue(cfgVal.get<double>(), allocator);
    }
    case core::config::ConfigStoredType::MagnumVec3: {
      return toJsonValue(cfgVal.get<Magnum::Vector3>(), allocator);
    }
    case core::config::ConfigStoredType::MagnumQuat: {
      return toJsonValue(cfgVal.get<Magnum::Quaternion>(), allocator);
    }
    case core::config::ConfigStoredType::MagnumRad: {
      auto r = cfgVal.get<Magnum::Rad>();
      return toJsonValue((r.operator float()), allocator);
    }
    case core::config::ConfigStoredType::String: {
      return toJsonValue(cfgVal.get<std::string>(), allocator);
    }
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in io::toJsonValue<ConfigValue>", {});
  }
}  // toJsonValue<core::config::ConfigValue>

JsonGenericValue toJsonValue(const gfx::replay::Keyframe& keyframe,
                             JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);

  io::addMember(obj, "loads", keyframe.loads, allocator);

  if (!keyframe.creations.empty()) {
    JsonGenericValue creationsArray(rapidjson::kArrayType);
    for (const auto& pair : keyframe.creations) {
      JsonGenericValue creationPairObj(rapidjson::kObjectType);
      io::addMember(creationPairObj, "instanceKey", pair.first, allocator);
      io::addMember(creationPairObj, "creation", pair.second, allocator);

      creationsArray.PushBack(creationPairObj, allocator);
    }
    io::addMember(obj, "creations", creationsArray, allocator);
  }

  io::addMember(obj, "deletions", keyframe.deletions, allocator);

  if (!keyframe.stateUpdates.empty()) {
    JsonGenericValue stateUpdatesArray(rapidjson::kArrayType);
    for (const auto& pair : keyframe.stateUpdates) {
      JsonGenericValue stateObj(rapidjson::kObjectType);
      io::addMember(stateObj, "instanceKey", pair.first, allocator);
      io::addMember(stateObj, "state", pair.second, allocator);
      stateUpdatesArray.PushBack(stateObj, allocator);
    }
    io::addMember(obj, "stateUpdates", stateUpdatesArray, allocator);
  }

  if (!keyframe.userTransforms.empty()) {
    JsonGenericValue userTransformsArray(rapidjson::kArrayType);
    for (const auto& pair : keyframe.userTransforms) {
      JsonGenericValue wrapperObj(rapidjson::kObjectType);
      io::addMember(wrapperObj, "name", pair.first, allocator);
      io::addMember(wrapperObj, "transform", pair.second, allocator);
      userTransformsArray.PushBack(wrapperObj, allocator);
    }
    io::addMember(obj, "userTransforms", userTransformsArray, allocator);
  }

  return obj;
}

bool fromJsonValue(const JsonGenericValue& obj,
                   gfx::replay::Keyframe& keyframe) {
  io::readMember(obj, "loads", keyframe.loads);

  auto itr = obj.FindMember("creations");
  if (itr != obj.MemberEnd()) {
    const JsonGenericValue& creationsArray = itr->value;
    keyframe.creations.reserve(creationsArray.Size());
    for (const auto& creationPairObj : creationsArray.GetArray()) {
      std::pair<gfx::replay::RenderAssetInstanceKey,
                assets::RenderAssetInstanceCreationInfo>
          pair;
      io::readMember(creationPairObj, "instanceKey", pair.first);
      io::readMember(creationPairObj, "creation", pair.second);
      keyframe.creations.emplace_back(std::move(pair));
    }
  }

  io::readMember(obj, "deletions", keyframe.deletions);

  itr = obj.FindMember("stateUpdates");
  if (itr != obj.MemberEnd()) {
    const JsonGenericValue& stateUpdatesArray = itr->value;
    keyframe.stateUpdates.reserve(stateUpdatesArray.Size());
    for (const auto& stateObj : stateUpdatesArray.GetArray()) {
      std::pair<gfx::replay::RenderAssetInstanceKey,
                gfx::replay::RenderAssetInstanceState>
          pair;
      io::readMember(stateObj, "instanceKey", pair.first);
      io::readMember(stateObj, "state", pair.second);
      keyframe.stateUpdates.emplace_back(std::move(pair));
    }
  }

  itr = obj.FindMember("userTransforms");
  if (itr != obj.MemberEnd()) {
    const JsonGenericValue& userTransformsArray = itr->value;
    for (const auto& userTransformObj : userTransformsArray.GetArray()) {
      std::string name;
      gfx::replay::Transform transform;
      io::readMember(userTransformObj, "name", name);
      io::readMember(userTransformObj, "transform", transform);
      keyframe.userTransforms[name] = transform;
    }
  }

  return true;
}

}  // namespace io
}  // namespace esp
