// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplicaSemanticScene.h"
#include "SemanticScene.h"

#include <Magnum/Math/Matrix4.h>

#include <map>
#include <string>

#include "esp/io/Json.h"

namespace esp {
namespace scene {

constexpr int kMaxIds = 10000; /* We shouldn't every need more than this. */

bool SemanticScene::loadReplicaHouse(const std::string& houseFilename,
                                     SemanticScene& scene,
                                     const Magnum::Quaternion& worldRotation) {
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
                                      const Magnum::Quaternion& worldRotation) {
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
  const auto worldRotationMat =
      Mn::Matrix4::from(worldRotation.normalized().toMatrix(), {});
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
    Mn::Vector3 aabbCenter;
    io::fromJsonValue(obb["abb"]["center"], aabbCenter);
    Mn::Vector3 aabbSizes;
    io::fromJsonValue(obb["abb"]["sizes"], aabbSizes);
    Mn::Vector3 translationBoxToWorld;
    io::fromJsonValue(obb["orientation"]["translation"], translationBoxToWorld);
    // 4th element is scalar in json
    Mn::Vector4 rotBoxToWorldCoeffs;
    io::fromJsonValue(obb["orientation"]["rotation"], rotBoxToWorldCoeffs);

    const Mn::Quaternion rotationBoxToWorld(rotBoxToWorldCoeffs.xyz(),
                                            rotBoxToWorldCoeffs.w());

    const Mn::Matrix4 transformBox = Mn::Matrix4::from(
        rotationBoxToWorld.normalized().toMatrix(), translationBoxToWorld);

    const auto transformBoxToWorld{worldRotationMat * transformBox};

    object->obb_ = geo::OBB{
        transformBoxToWorld.transformVector(aabbCenter), aabbSizes,
        Mn::Quaternion::fromMatrix(transformBoxToWorld.rotationNormalized())};

    scene.objects_[id] = std::move(object);
  }
  scene.hasVertColors_ = true;
  return true;
}

}  // namespace scene
}  // namespace esp
