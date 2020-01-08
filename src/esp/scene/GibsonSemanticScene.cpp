// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GibsonSemanticScene.h"
#include "SemanticScene.h"

#include <map>
#include <string>

#include <Corrade/Utility/Directory.h>

#include "esp/io/json.h"

namespace Cr = Corrade;

namespace esp {
namespace scene {

constexpr int kMaxIds = 10000; /* We shouldn't every need more than this. */

bool SemanticScene::loadGibsonHouse(
    const std::string& houseFilename,
    SemanticScene& scene,
    const quatf& worldRotation /* = quatf::Identity() */) {
  if (!Cr::Utility::Directory::exists(houseFilename)) {
    LOG(ERROR) << "Could not load file " << houseFilename;
    return false;
  }

  scene.categories_.clear();
  scene.objects_.clear();

  // top-level scene
  VLOG(1) << "Parsing " << houseFilename;
  const auto& json = io::parseJsonFile(houseFilename);
  VLOG(1) << "Parsed.";

  std::unordered_map<std::string, int> categories;

  // objects
  const auto& objects = json["objects"].GetArray();
  scene.elementCounts_["objects"] = objects.Size();
  for (const auto& jsonObject : objects) {
    SemanticObject::ptr object = SemanticObject::create();
    int id = jsonObject["id"].GetInt();
    if (id > kMaxIds) {
      LOG(ERROR) << "Exceeded max number of ids";
      continue;
    }
    if (scene.objects_.size() < id + 1) {
      scene.objects_.resize(id + 1, nullptr);
    }
    object->index_ = id;

    const std::string categoryName = jsonObject["class_"].GetString();
    auto it = categories.find(categoryName);
    if (it != categories.end()) {
      object->category_ = scene.categories_[it->second];
    } else {
      int nextCategoryIndex = scene.categories_.size();
      categories[categoryName] = nextCategoryIndex;
      // NOTE(msb) vector is 0-indexed but categories index starts at 1
      nextCategoryIndex++;
      auto category = std::make_shared<GibsonObjectCategory>(nextCategoryIndex,
                                                             categoryName);
      scene.categories_.push_back(category);
      object->category_ = std::move(category);
    }
    // TODO(msb) add support for aabb
    scene.objects_[id] = std::move(object);
  }

  return true;
}

}  // namespace scene
}  // namespace esp
