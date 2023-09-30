// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(const gfx::replay::Keyframe& keyframe,
                             JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);

  io::addMember(obj, "loads", keyframe.loads, allocator);

  if (!keyframe.rigCreations.empty()) {
    JsonGenericValue rigCreationsArray(rapidjson::kArrayType);
    for (const auto& rig : keyframe.rigCreations) {
      JsonGenericValue rigObj(rapidjson::kObjectType);
      io::addMember(rigObj, "id", rig.id, allocator);
      io::addMember(rigObj, "boneNames", rig.boneNames, allocator);
      rigCreationsArray.PushBack(rigObj, allocator);
    }
    io::addMember(obj, "rigCreations", rigCreationsArray, allocator);
  }

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

  if (!keyframe.rigUpdates.empty()) {
    JsonGenericValue rigUpdatesArray(rapidjson::kArrayType);
    for (const auto& rig : keyframe.rigUpdates) {
      JsonGenericValue rigObj(rapidjson::kObjectType);
      io::addMember(rigObj, "id", rig.id, allocator);
      JsonGenericValue poseArray(rapidjson::kArrayType);
      for (const auto& bone : rig.pose) {
        JsonGenericValue boneObj(rapidjson::kObjectType);
        io::addMember(boneObj, "t", bone.translation, allocator);
        io::addMember(boneObj, "r", bone.rotation, allocator);
        poseArray.PushBack(boneObj, allocator);
      }
      io::addMember(rigObj, "pose", poseArray, allocator);
      rigUpdatesArray.PushBack(rigObj, allocator);
    }
    io::addMember(obj, "rigUpdates", rigUpdatesArray, allocator);
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

  if (keyframe.lightsChanged) {
    io::addMember(obj, "lightsChanged", true, allocator);
    io::addMember(obj, "lights", keyframe.lights, allocator);
  }

  return obj;
}

bool fromJsonValue(const JsonGenericValue& obj,
                   gfx::replay::Keyframe& keyframe) {
  io::readMember(obj, "loads", keyframe.loads);

  auto itr = obj.FindMember("rigCreations");
  if (itr != obj.MemberEnd()) {
    const JsonGenericValue& ricCreationsArray = itr->value;
    keyframe.rigCreations.reserve(ricCreationsArray.Size());
    for (const auto& creationPairObj : ricCreationsArray.GetArray()) {
      gfx::replay::RigCreation rigCreation;
      io::readMember(creationPairObj, "id", rigCreation.id);
      io::readMember(creationPairObj, "boneNames", rigCreation.boneNames);
      keyframe.rigCreations.emplace_back(std::move(rigCreation));
    }
  }

  itr = obj.FindMember("creations");
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

  itr = obj.FindMember("rigUpdates");
  if (itr != obj.MemberEnd()) {
    const JsonGenericValue& rigUpdatesArray = itr->value;
    keyframe.rigUpdates.reserve(rigUpdatesArray.Size());
    for (const auto& rigObj : rigUpdatesArray.GetArray()) {
      gfx::replay::RigUpdate rigUpdate;
      io::readMember(rigObj, "id", rigUpdate.id);
      itr = rigObj.FindMember("pose");
      if (itr != rigObj.MemberEnd()) {
        const JsonGenericValue& poseArray = itr->value;
        rigUpdate.pose.reserve(poseArray.Size());
        for (const auto& boneObj : poseArray.GetArray()) {
          gfx::replay::Transform transform;
          io::readMember(boneObj, "t", transform.translation);
          io::readMember(boneObj, "r", transform.rotation);
          rigUpdate.pose.emplace_back(transform);
        }
      }
      keyframe.rigUpdates.emplace_back(std::move(rigUpdate));
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

  io::readMember(obj, "lightsChanged", keyframe.lightsChanged);
  if (keyframe.lightsChanged) {
    io::readMember(obj, "lights", keyframe.lights);
  }

  return true;
}

JsonGenericValue toJsonValue(const esp::assets::AssetInfo& x,
                             JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMemberAsUint32(obj, "type", x.type, allocator);
  addMember(obj, "filepath", x.filepath, allocator);
  addMember(obj, "frame", x.frame, allocator);
  addMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters, allocator);
  addMember(obj, "forceFlatShading", x.forceFlatShading, allocator);
  addMember(obj, "splitInstanceMesh", x.splitInstanceMesh, allocator);
  addMember(obj, "shaderTypeToUse", x.shaderTypeToUse, allocator);
  addMember(obj, "overridePhongMaterial", x.overridePhongMaterial, allocator);
  addMember(obj, "hasSemanticTextures", x.hasSemanticTextures, allocator);

  return obj;
}

bool fromJsonValue(const JsonGenericValue& obj, esp::assets::AssetInfo& x) {
  readMemberAsUint32(obj, "type", x.type);
  readMember(obj, "filepath", x.filepath);
  readMember(obj, "frame", x.frame);
  readMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters);
  readMember(obj, "forceFlatShading", x.forceFlatShading);
  readMember(obj, "splitInstanceMesh", x.splitInstanceMesh);
  readMember(obj, "shaderTypeToUse", x.shaderTypeToUse);
  readMember(obj, "overridePhongMaterial", x.overridePhongMaterial);
  readMember(obj, "hasSemanticTextures", x.hasSemanticTextures);
  return true;
}

JsonGenericValue toJsonValue(
    const metadata::attributes::ObjectInstanceShaderType& x,
    JsonAllocator& allocator) {
  return toJsonValue(metadata::attributes::getShaderTypeName(x), allocator);
}

bool fromJsonValue(const JsonGenericValue& obj,
                   metadata::attributes::ObjectInstanceShaderType& x) {
  std::string shaderTypeToUseString;
  // read as string
  bool shaderTypeSuccess = fromJsonValue(obj, shaderTypeToUseString);
  // convert to enum
  if (shaderTypeSuccess) {
    const std::string shaderTypeLC =
        Cr::Utility::String::lowercase(shaderTypeToUseString);
    auto mapIter = metadata::attributes::ShaderTypeNamesMap.find(shaderTypeLC);
    ESP_CHECK(mapIter != metadata::attributes::ShaderTypeNamesMap.end(),
              "Illegal shader_type value '"
                  << shaderTypeToUseString
                  << "' specified in JSON to be used to set "
                     "AssetInfo.shaderTypeToUse. "
                     "Aborting.");
    x = mapIter->second;
  }
  return shaderTypeSuccess;
}

JsonGenericValue toJsonValue(const esp::gfx::LightPositionModel& x,
                             JsonAllocator& allocator) {
  return toJsonValue(metadata::attributes::getLightPositionModelName(x),
                     allocator);
}

bool fromJsonValue(const JsonGenericValue& obj,
                   esp::gfx::LightPositionModel& x) {
  std::string lightPositionModelString;
  // read as string
  bool success = fromJsonValue(obj, lightPositionModelString);
  // convert to enum
  if (success) {
    const std::string lightPositionModelLC =
        Cr::Utility::String::lowercase(lightPositionModelString);
    auto mapIter =
        metadata::attributes::LightPositionNamesMap.find(lightPositionModelLC);
    ESP_CHECK(mapIter != metadata::attributes::LightPositionNamesMap.end(),
              "Illegal model value '"
                  << lightPositionModelString
                  << "' specified in JSON to be used to set LightInfo.model. "
                  << "Aborting.");
    x = mapIter->second;
  }
  return success;
}

JsonGenericValue toJsonValue(const esp::nav::NavMeshSettings& x,
                             JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "cellSize", x.cellSize, allocator);
  addMember(obj, "cellHeight", x.cellHeight, allocator);
  addMember(obj, "agentHeight", x.agentHeight, allocator);
  addMember(obj, "agentRadius", x.agentRadius, allocator);
  addMember(obj, "agentMaxClimb", x.agentMaxClimb, allocator);
  addMember(obj, "agentMaxSlope", x.agentMaxSlope, allocator);
  addMember(obj, "regionMinSize", x.regionMinSize, allocator);
  addMember(obj, "regionMergeSize", x.regionMergeSize, allocator);
  addMember(obj, "edgeMaxLen", x.edgeMaxLen, allocator);
  addMember(obj, "edgeMaxError", x.edgeMaxError, allocator);
  addMember(obj, "vertsPerPoly", x.vertsPerPoly, allocator);
  addMember(obj, "detailSampleDist", x.detailSampleDist, allocator);
  addMember(obj, "detailSampleMaxError", x.detailSampleMaxError, allocator);
  addMember(obj, "filterLowHangingObstacles", x.filterLowHangingObstacles,
            allocator);
  addMember(obj, "filterLedgeSpans", x.filterLedgeSpans, allocator);
  addMember(obj, "filterWalkableLowHeightSpans", x.filterWalkableLowHeightSpans,
            allocator);

  return obj;
}

bool fromJsonValue(const JsonGenericValue& obj, esp::nav::NavMeshSettings& x) {
  readMember(obj, "cellSize", x.cellSize);
  readMember(obj, "cellHeight", x.cellHeight);
  readMember(obj, "agentHeight", x.agentHeight);
  readMember(obj, "agentRadius", x.agentRadius);
  readMember(obj, "agentMaxClimb", x.agentMaxClimb);
  readMember(obj, "agentMaxSlope", x.agentMaxSlope);
  readMember(obj, "regionMinSize", x.regionMinSize);
  readMember(obj, "regionMergeSize", x.regionMergeSize);
  readMember(obj, "edgeMaxLen", x.edgeMaxLen);
  readMember(obj, "edgeMaxError", x.edgeMaxError);
  readMember(obj, "vertsPerPoly", x.vertsPerPoly);
  readMember(obj, "detailSampleDist", x.detailSampleDist);
  readMember(obj, "detailSampleMaxError", x.detailSampleMaxError);
  readMember(obj, "filterLowHangingObstacles", x.filterLowHangingObstacles);
  readMember(obj, "filterLedgeSpans", x.filterLedgeSpans);
  readMember(obj, "filterWalkableLowHeightSpans",
             x.filterWalkableLowHeightSpans);

  return true;
}

}  // namespace io
}  // namespace esp
