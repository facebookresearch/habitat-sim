// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BpsSceneMapping.h"

#include "esp/io/json.h"

namespace esp {
namespace batched_sim {

BpsSceneMapping BpsSceneMapping::loadFromFile(const std::string& filepath) {
  BpsSceneMapping mapping;
  auto newDoc = esp::io::parseJsonFile(filepath);
  esp::io::readMember(newDoc, "mapping", mapping);
  return mapping;
}

esp::io::JsonGenericValue toJsonValue(const BpsSceneMapping& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "materials", x.materials, allocator);
  esp::io::addMember(obj, "meshes", x.meshes, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, BpsSceneMapping& x) {
  esp::io::readMember(obj, "materials", x.materials);
  esp::io::readMember(obj, "meshes", x.meshes);
  return true;
}

std::pair<int, int> BpsSceneMapping::findMeshIndexMaterialIndex(
    const std::string& meshName,
    const std::string& mtrlName) {
  auto it = std::find(meshes.begin(), meshes.end(), meshName);
  CORRADE_INTERNAL_ASSERT(it != meshes.end());
  int meshIndex = it - meshes.begin();

  it = std::find(materials.begin(), materials.end(), mtrlName);
  CORRADE_INTERNAL_ASSERT(it != materials.end());
  int mtrlIndex = it - materials.begin();

  return {meshIndex, mtrlIndex};
}

std::tuple<int, int, float> BpsSceneMapping::findMeshIndexMaterialIndexScale(
    const std::string& nodeName) {
  std::vector<std::pair<std::string, std::string> > nodeMeshesMaterials = {
      {"base_link", "material0"},
      {"elbow_flex_link", "material0.001"},
      {"gripper_link", "material0.002"},
      {"forearm_roll_link", "material0.003"},
      {"head_pan_link", "material0.004"},
      {"head_tilt_link", "material0.005"},
      {"shoulder_lift_link", "material0.006"},
      {"shoulder_pan_link", "material0.007"},
      {"torso_fixed_link", "material0.008"},
      {"upperarm_roll_link", "material0.009"},
      {"wrist_flex_link", "material0.010"},
      {"wrist_roll_link", "material0.011"},
      {"r_wheel_link", "r_l_wheel_link"},
      {"l_wheel_link", "r_l_wheel_link"},
      {"r_gripper_finger_link", "r_l_gripper_finger_link"},
      {"l_gripper_finger_link", "r_l_gripper_finger_link"},
      {"bellows_link", "Material.005"},
      {"laser_link", "Material.007"},
      {"torso_lift_link", "material0.012"},
      {"estop_link", "Material_001"}};

  const std::string meshName = nodeName;
  bool found = false;
  std::string mtrlName;
  for (const auto& pair : nodeMeshesMaterials) {
    if (pair.first == meshName) {
      mtrlName = pair.second;
      found = true;
      break;
    }
  }
  CORRADE_INTERNAL_ASSERT(found);

  auto [meshIndex, mtrlIndex] = findMeshIndexMaterialIndex(meshName, mtrlName);
  constexpr float scale = 1.f;

  return std::make_tuple(meshIndex, mtrlIndex, scale);
}

}  // namespace batched_sim
}  // namespace esp
