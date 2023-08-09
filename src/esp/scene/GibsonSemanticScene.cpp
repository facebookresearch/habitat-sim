// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GibsonSemanticScene.h"
#include "SemanticScene.h"

#include <map>
#include <string>

#include "esp/io/Json.h"

namespace Cr = Corrade;

namespace esp {
namespace scene {

constexpr int kMaxIds = 10000; /* We shouldn't ever need more than this. */

bool SemanticScene::
    loadGibsonHouse(const std::string& houseFilename, SemanticScene& scene, const quatf& rotation /* = quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY) */) {
  if (!checkFileExists(houseFilename, "loadGibsonHouse")) {
    return false;
  }

  // top-level scene
  ESP_VERY_VERBOSE() << "Parsing" << houseFilename;
  const io::JsonDocument& json = io::parseJsonFile(houseFilename);
  ESP_VERY_VERBOSE() << "Parsed.";

  return buildGibsonHouse(json, scene, rotation);
}  // SemanticScene::loadGibsonHouse

bool SemanticScene::buildGibsonHouse(const io::JsonDocument& jsonDoc,
                                     SemanticScene& scene,
                                     const quatf& rotation) {
  scene.categories_.clear();
  scene.objects_.clear();

  std::unordered_map<std::string, int> categories;

  // objects
  const auto& objects = jsonDoc["objects"].GetArray();
  scene.elementCounts_["objects"] = objects.Size();
  for (const auto& jsonObject : objects) {
    SemanticObject::ptr object = SemanticObject::create();
    int id = jsonObject["id"].GetInt();
    if (id > kMaxIds) {
      ESP_ERROR() << "Exceeded max number of ids";
      continue;
    }
    if (scene.objects_.size() < id + 1) {
      scene.objects_.resize(id + 1, nullptr);
    }
    object->index_ = id;
    // Assuming class_ -always- exists
    const std::string categoryName = jsonObject["class_"].GetString();
    auto it = categories.find(categoryName);
    if (it != categories.end()) {
      object->category_ = scene.categories_[it->second];
    } else {
      int nextCategoryIndex = scene.categories_.size();
      categories[categoryName] = nextCategoryIndex;
      // NOTE(msb) vector is 0-indexed but categories index starts at 1
      ++nextCategoryIndex;
      auto category = std::make_shared<GibsonObjectCategory>(nextCategoryIndex,
                                                             categoryName);
      scene.categories_.push_back(category);
      object->category_ = std::move(category);
    }

    io::JsonGenericValue::ConstMemberIterator jsonLocIter =
        jsonObject.FindMember("location");
    if (jsonLocIter != jsonObject.MemberEnd()) {
      // if (jsonObject.HasMember("location")) {
      const auto& jsonCenter = jsonLocIter->value;
      vec3f center = rotation * io::jsonToVec3f(jsonCenter);
      vec3f size = vec3f::Zero();
      io::JsonGenericValue::ConstMemberIterator jsonSizeIter =
          jsonObject.FindMember("size");
      if (jsonSizeIter != jsonObject.MemberEnd()) {
        const auto& jsonSize = jsonSizeIter->value;
        // Rotating sizes
        size = (rotation * io::jsonToVec3f(jsonSize)).array().abs();
      } else {
        ESP_WARNING() << "Object size from" << categoryName
                      << "isn't provided.";
      }
      object->setObb(center, size, quatf::Identity());
    } else {
      ESP_WARNING() << "Object center coordinates from" << categoryName
                    << "aren't provided.";
    }
    scene.objects_[id] = std::move(object);
  }

  return true;
}  // SemanticScene::buildGibsonHouse

}  // namespace scene
}  // namespace esp
