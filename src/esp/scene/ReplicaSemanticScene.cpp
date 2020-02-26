// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplicaSemanticScene.h"
#include "SemanticScene.h"

#include <map>
#include <string>

#include <Corrade/Utility/String.h>

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
namespace scene {

constexpr int kMaxIds = 10000; /* We shouldn't every need more than this. */

bool SemanticScene::loadReplicaHouse(
    const std::string& houseFilename,
    SemanticScene& scene,
    const quatf& worldRotation /* = quatf::Identity() */) {
  if (!io::exists(houseFilename)) {
    LOG(ERROR) << "Could not load file " << houseFilename;
    return false;
  }

  scene.categories_.clear();
  scene.objects_.clear();

  // top-level scene
  VLOG(1) << "Parsing " << houseFilename;
  const auto& json = io::parseJsonFile(houseFilename);
  VLOG(1) << "Parsed.";

  // categories
  const auto& categories = json["classes"].GetArray();
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
  const auto& objects = json["objects"].GetArray();
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
    // TODO(msb) object->obb = ;
    scene.objects_[id] = std::move(object);
  }

  return true;
}

}  // namespace scene
}  // namespace esp
