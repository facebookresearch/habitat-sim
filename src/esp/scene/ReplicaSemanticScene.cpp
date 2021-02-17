// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplicaSemanticScene.h"
#include "SemanticScene.h"

#include <map>
#include <sophus/se3.hpp>
#include <string>

#include "esp/io/io.h"
#include "esp/io/json.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

namespace esp {
namespace scene {

constexpr int kMaxIds = 10000; /* We shouldn't every need more than this. */

bool SemanticScene::loadReplicaHouse(
    const std::string& houseFilename,
    SemanticScene& scene,
    const quatf& worldRotation /* = quatf::Identity() */) {
  if (!checkFileExists(houseFilename, "loadReplicaHouse")) {
    return false;
  }

  // top-level scene
  VLOG(1) << "loadReplicaHouse::Parsing " << houseFilename;
  const auto& json = io::parseJsonFile(houseFilename);
  VLOG(1) << "loadReplicaHouse::Parsed.";

  return buildReplicaHouse(json, scene, worldRotation);

}  // SemanticScene::loadReplicaHouse

bool SemanticScene::buildReplicaHouse(const io::JsonDocument& jsonDoc,
                                      SemanticScene& scene,
                                      const quatf& worldRotation) {
  scene.categories_.clear();
  scene.objects_.clear();

  // categories
  const auto& categories = jsonDoc["classes"].GetArray();
  scene.elementCounts_["categories"] = categories.Size();
  for (const auto& category : categories) {
    int id = category["id"].GetInt();

    /*
     * We store the category object at categories_[id] in order to making
     * indexing easy.
     */
    if (id > kMaxIds) {
      LOG(ERROR) << "Exceeded max number of ids";
      continue;
    }
    if (scene.categories_.size() < id + 1) {
      scene.categories_.resize(id + 1, nullptr);
    }
    scene.categories_[id] = std::make_shared<ReplicaObjectCategory>(
        id, category["name"].GetString());
  }

  // objects
  const auto& objects = jsonDoc["objects"].GetArray();
  scene.elementCounts_["objects"] = objects.Size();
  for (const auto& jsonObject : objects) {
    SemanticObject::ptr object = SemanticObject::create();
    int id = jsonObject["id"].GetInt();
    int categoryIndex = jsonObject["class_id"].GetInt();
    /*
     * We store the category object at categories_[id] in order to making
     * indexing easy.
     */
    if (id > kMaxIds) {
      LOG(ERROR) << "Exceeded max number of ids";
      continue;
    }
    if (scene.objects_.size() < id + 1) {
      scene.objects_.resize(id + 1, nullptr);
    }
    object->index_ = id;
    if (categoryIndex < scene.categories_.size()) {
      object->category_ = scene.categories_[categoryIndex];
    }

    auto& obb = jsonObject["oriented_bbox"];
    const vec3f aabbCenter = io::jsonToVec3f(obb["abb"]["center"]);
    const vec3f aabbSizes = io::jsonToVec3f(obb["abb"]["sizes"]);

    const vec3f translationBoxToWorld =
        io::jsonToVec3f(obb["orientation"]["translation"]);

    std::vector<float> rotationBoxToWorldCoeffs;
    io::toFloatVector(obb["orientation"]["rotation"],
                      &rotationBoxToWorldCoeffs);
    const Eigen::Map<quatf> rotationBoxToWorld(rotationBoxToWorldCoeffs.data());

    const auto transformBoxToWorld =
        Sophus::SE3f{worldRotation, vec3f::Zero()} *
        Sophus::SE3f{rotationBoxToWorld, translationBoxToWorld};

    object->obb_ = geo::OBB{transformBoxToWorld * aabbCenter, aabbSizes,
                            transformBoxToWorld.so3().unit_quaternion()};
    scene.objects_[id] = std::move(object);
  }

  return true;
}

}  // namespace scene
}  // namespace esp
