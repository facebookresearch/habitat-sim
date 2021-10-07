// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"

namespace esp {
namespace io {

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
