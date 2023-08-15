// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplicaSemanticScene.h"
#include "SemanticScene.h"

#include <map>
#include <string>

#include "esp/io/Json.h"

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
  ESP_VERY_VERBOSE() << "Parsing" << houseFilename;
  const auto& json = io::parseJsonFile(houseFilename);
  ESP_VERY_VERBOSE() << "Parsed.";

  // check if Replica or ReplicaCAD
  io::JsonGenericValue::ConstMemberIterator replicaObjIter =
      json.FindMember("objects");
  bool hasObjects =
      (replicaObjIter != json.MemberEnd()) && (replicaObjIter->value.IsArray());

  return buildReplicaHouse(json, scene, hasObjects, worldRotation);

}  // SemanticScene::loadReplicaHouse

bool SemanticScene::buildReplicaHouse(const io::JsonDocument& jsonDoc,
                                      SemanticScene& scene,
                                      bool objectsExist,
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
      ESP_ERROR() << "Exceeded max number of ids";
      continue;
    }
    if (scene.categories_.size() < id + 1) {
      scene.categories_.resize(id + 1, nullptr);
    }
    scene.categories_[id] = std::make_shared<ReplicaObjectCategory>(
        id, category["name"].GetString());
  }

  // if does not have objects, then this is ReplcaCAD semantic map which lacks
  // object semantic mappings
  if (!objectsExist) {
    return true;
  }

  // objects
  const auto& objects = jsonDoc["objects"].GetArray();
  scene.elementCounts_["objects"] = objects.Size();

  // construct rotation matrix to be used to construct transform
  const Eigen::Isometry3f worldRotationMat{worldRotation.normalized()};
  for (const auto& jsonObject : objects) {
    SemanticObject::ptr object = SemanticObject::create();
    int id = jsonObject["id"].GetInt();
    int categoryIndex = jsonObject["class_id"].GetInt();
    /*
     * We store the category object at categories_[id] in order to making
     * indexing easy.
     */
    if (id > kMaxIds) {
      ESP_ERROR() << "Exceeded max number of ids";
      continue;
    }
    if (scene.objects_.size() < id + 1) {
      scene.objects_.resize(id + 1, nullptr);
    }
    object->index_ = id;
    if (categoryIndex < scene.categories_.size()) {
      object->category_ = scene.categories_[categoryIndex];
    }

    const auto& obb = jsonObject["oriented_bbox"];
    const vec3f aabbCenter = io::jsonToVec3f(obb["abb"]["center"]);
    const vec3f aabbSizes = io::jsonToVec3f(obb["abb"]["sizes"]);

    const vec3f translationBoxToWorld =
        io::jsonToVec3f(obb["orientation"]["translation"]);

    std::vector<float> rotationBoxToWorldCoeffs;
    io::toFloatVector(obb["orientation"]["rotation"],
                      &rotationBoxToWorldCoeffs);
    const Eigen::Map<quatf> rotationBoxToWorld(rotationBoxToWorldCoeffs.data());

    Eigen::Isometry3f transformBox{rotationBoxToWorld.normalized()};
    transformBox *= Eigen::Translation3f(translationBoxToWorld);

    const Eigen::Isometry3f transformBoxToWorld{worldRotationMat *
                                                transformBox};

    object->obb_ = geo::OBB{transformBoxToWorld * aabbCenter, aabbSizes,
                            quatf{transformBoxToWorld.linear()}.normalized()};

    scene.objects_[id] = std::move(object);
  }
  scene.hasVertColors_ = true;
  return true;
}

}  // namespace scene
}  // namespace esp
