// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/EpisodeSet.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/GlmUtils.h"

#include "esp/core/Check.h"
#include "esp/io/json.h"

#include <Corrade/Utility/Directory.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

const std::string& getFreeObjectName(const FreeObject& freeObject,
                                     const EpisodeSet& set) {
  return safeVectorGet(set.renderAssets_, freeObject.renderAssetIndex_).name_;
}

void updateFromSerializeCollection(FreeObject& freeObject,
                                   const serialize::FreeObject& serFreeObject,
                                   const serialize::Collection& collection) {
  freeObject.aabb_ = Mn::Range3D(serFreeObject.collisionBox.min,
                                 serFreeObject.collisionBox.max);
  freeObject.heldRotationIndex_ = serFreeObject.heldRotationIndex;
  ESP_CHECK(
      freeObject.heldRotationIndex_ >= 0 &&
          freeObject.heldRotationIndex_ < freeObject.startRotations_.size(),
      "updateFromSerializeCollection: heldRotationIndex "
          << serFreeObject.heldRotationIndex
          << " is out-of-range for FreeObject with startRotations.size() == "
          << freeObject.startRotations_.size());

  freeObject.collisionSpheres_.clear();
  std::vector<serialize::Sphere> generatedSpheres;
  const std::vector<serialize::Sphere>* serializeCollisionSpheres = nullptr;

  if (!serFreeObject.generateCollisionSpheresTechnique.empty()) {
    auto& spheres = generatedSpheres;
    serializeCollisionSpheres = &generatedSpheres;

    float smallRadius = 0.015f;
    float mediumRadius = 0.05f;
    float largeRadius = 0.12f;

    const auto& aabb = freeObject.aabb_;
    Mn::Vector3 aabbCenter = aabb.center();

    if (serFreeObject.generateCollisionSpheresTechnique == "box") {
      // small and medium spheres at each corner
      // consolidate duplicates at the end

      // insert larger spheres first, so that de-duplication (later) leaves
      // larger spheres
      bool isFirstUsableSize = true;
      for (float r : {largeRadius, mediumRadius, smallRadius}) {
        if (Mn::Math::min(Mn::Math::min(aabb.sizeX(), aabb.sizeY()),
                          aabb.sizeZ()) < r * 2.f) {
          // object is too small for even one sphere of this radius
          continue;
        }

        if (isFirstUsableSize) {
          // 2 and 3 are heuristics for sphere density, with tradeoff between
          // collision accuracy and speed
          const float spacing = (r == largeRadius) ? r * 2.f : r * 3.f;
          int numX = Mn::Math::max(int(aabb.sizeX() / spacing) + 1, 2);
          int numY = Mn::Math::max(int(aabb.sizeY() / spacing) + 1, 2);
          int numZ = Mn::Math::max(int(aabb.sizeZ() / spacing) + 1, 2);
          Mn::Vector3 origin;
          for (int ix = 0; ix < numX; ix++) {
            origin.x() = Mn::Math::lerp(aabb.min().x() + r, aabb.max().x() - r,
                                        float(ix) / (numX - 1));
            for (int iy = 0; iy < numY; iy++) {
              origin.y() =
                  Mn::Math::lerp(aabb.min().y() + r, aabb.max().y() - r,
                                 float(iy) / (numY - 1));
              for (int iz = 0; iz < numZ; iz++) {
                origin.z() =
                    Mn::Math::lerp(aabb.min().z() + r, aabb.max().z() - r,
                                   float(iz) / (numZ - 1));
                spheres.push_back({origin, r});
              }
            }
          }
          isFirstUsableSize = false;
        } else {
          // just fill in corners
          spheres.push_back({aabb.backBottomLeft(), r});
          spheres.push_back({aabb.backBottomRight(), r});
          spheres.push_back({aabb.backTopLeft(), r});
          spheres.push_back({aabb.backTopRight(), r});
          spheres.push_back({aabb.frontBottomLeft(), r});
          spheres.push_back({aabb.frontBottomRight(), r});
          spheres.push_back({aabb.frontTopLeft(), r});
          spheres.push_back({aabb.frontTopRight(), r});
        }
      }

    } else if (serFreeObject.generateCollisionSpheresTechnique ==
               "uprightCylinder") {
      bool isFirstUsableSize = true;

      // insert larger spheres first, so that de-duplication (later) leaves
      // larger spheres
      for (float r : {largeRadius, mediumRadius, smallRadius}) {
        if (Mn::Math::min(Mn::Math::min(aabb.sizeX(), aabb.sizeY()),
                          aabb.sizeZ()) < r * 2.f) {
          // object is too small for even one sphere of this radius
          continue;
        }

        // 2 and 3 are heuristics for sphere density, with tradeoff between
        // collision accuracy and speed
        const float spacing = (r == largeRadius) ? r * 2.f : r * 3.f;
        int numX = Mn::Math::max(int(aabb.sizeX() / spacing) + 1, 2);
        int numY = Mn::Math::max(int(aabb.sizeY() / spacing) + 1, 2);
        int numZ = isFirstUsableSize
                       ? Mn::Math::max(int(aabb.sizeZ() / spacing) + 1, 2)
                       : 2;
        int numAngles = numX + numY;  // heuristic based on XY dimension
        Mn::Vector3 origin;
        Mn::Vector3 aabbCenter = aabb.center();
        // construct rings at various z heights
        for (int iz = 0; iz < numX; iz++) {
          origin.z() = Mn::Math::lerp(aabb.min().z() + r, aabb.max().z() - r,
                                      float(iz) / (numZ - 1));
          for (int angleIdx = 0; angleIdx < numAngles; angleIdx++) {
            auto angle = Mn::Math::Deg(
                Mn::Math::lerp(0.f, 360.f, float(angleIdx) / (numAngles)));
            origin =
                Mn::Vector3(Mn::Math::lerp(aabbCenter.x(), aabb.max().x() - r,
                                           Mn::Math::cos(angle)),
                            Mn::Math::lerp(aabbCenter.y(), aabb.max().y() - r,
                                           Mn::Math::sin(angle)),
                            origin.z());
            spheres.push_back({origin, r});
          }

          if (isFirstUsableSize && numAngles > 4) {
            // also insert sphere at XY center
            origin = Mn::Vector3(aabbCenter.x(), aabbCenter.y(), origin.z());
            spheres.push_back({origin, r});
          }
        }
        isFirstUsableSize = false;
      }

    } else {
      ESP_CHECK(false, "free object generateCollisionSpheresTechnique \""
                           << serFreeObject.generateCollisionSpheresTechnique
                           << "\" not recognized. "
                              "Valid values are empty-string, \"box\", and "
                              "\"uprightCylinder\"");
    }

    // clamp to fit inside box extents, but don't move sphere center past center
    // of box (to other side)
    for (auto& sphere : spheres) {
      Mn::Vector3 clampedOrigin;
      for (int dim = 0; dim < 3; dim++) {
        clampedOrigin[dim] =
            sphere.origin[dim] < aabbCenter[dim]
                ? Mn::Math::clamp(sphere.origin[dim],
                                  Mn::Math::min(aabb.min()[dim] + sphere.radius,
                                                aabbCenter[dim]),
                                  aabbCenter[dim])
                : Mn::Math::clamp(sphere.origin[dim], aabbCenter[dim],
                                  Mn::Math::max(aabb.max()[dim] - sphere.radius,
                                                aabbCenter[dim]));
      }
      sphere.origin = clampedOrigin;
    }

    // remove duplicates
    for (int i = spheres.size() - 1; i >= 0; i--) {
      bool foundDup = false;
      for (int j = 0; j < i; j++) {
        if (spheres[i].origin == spheres[j].origin) {
          auto it = spheres.begin() + i;
          spheres.erase(spheres.begin() + i);
          break;
        }
      }
    }

    BATCHED_SIM_ASSERT(!spheres.empty());
  } else {
    ESP_CHECK(!serFreeObject.collisionSpheres.empty(),
              "no collision spheres for free object "
                  << serFreeObject.name
                  << " and generateCollisionSpheresFromBox==false");
    serializeCollisionSpheres = &serFreeObject.collisionSpheres;
  }

  for (const auto& serSphere : *serializeCollisionSpheres) {
    int radiusIdx = getCollisionRadiusIndex(collection, serSphere.radius);
    freeObject.collisionSpheres_.push_back({serSphere.origin, radiusIdx});
  }
}

void updateFromSerializeCollection(EpisodeSet& set,
                                   const serialize::Collection& collection) {
  for (const auto& serFreeObject : collection.freeObjects) {
    auto it = std::find_if(set.freeObjects_.begin(), set.freeObjects_.end(),
                           [&](const auto& item) {
                             return getFreeObjectName(item, set) ==
                                    serFreeObject.name;
                           });
    if (it == set.freeObjects_.end()) {
      // collection may have info for free objects not used in the current
      // EpisodeSet
      continue;
    }

    auto& freeObject = *it;
    updateFromSerializeCollection(freeObject, serFreeObject, collection);
  }
}

void postLoadFixup(EpisodeSet& set,
                   const BpsSceneMapping& sceneMapping,
                   const serialize::Collection& collection) {
  for (auto& renderAsset : set.renderAssets_) {
    renderAsset.instanceBlueprint_ =
        sceneMapping.findInstanceBlueprint(renderAsset.name_);
    renderAsset.needsPostLoadFixup_ = false;
  }

  for (auto& freeObj : set.freeObjects_) {
    ESP_CHECK(freeObj.renderAssetIndex_ >= 0 &&
                  freeObj.renderAssetIndex_ < set.renderAssets_.size(),
              "postLoadFixup: FreeObject "
                  << getFreeObjectName(freeObj, set)
                  << " has out-of-range renderAssetIndex_="
                  << freeObj.renderAssetIndex_);
    for (const auto& rot : freeObj.startRotations_) {
      ESP_CHECK(rot.isNormalized(), "postLoadFixup: FreeObject "
                                        << getFreeObjectName(freeObj, set)
                                        << " rotation " << rot
                                        << " isn't normalized");
    }
  }

  set.allEpisodesAABB_ = Mn::Range3D(Mn::Math::ZeroInit);
  bool isFirstFixedObj = true;
  for (auto& staticScene : set.staticScenes_) {
    for (auto& instance : staticScene.renderAssetInstances_) {
      ESP_CHECK(instance.renderAssetIndex_ >= 0 &&
                    instance.renderAssetIndex_ < set.renderAssets_.size(),
                "postLoadFixup: StaticScene render asset instance has "
                "out-of-range renderAssetIndex_="
                    << instance.renderAssetIndex_);

      const auto& transform = instance.transform_;
      Mn::Matrix4 mat =
          Mn::Matrix4::from(transform.rotation_.toMatrix(), transform.origin_);
      mat = mat * Mn::Matrix4::scaling(instance.transform_.scale_);
      instance.glMat_ = toGlmMat4x3(mat);
    }

    // sloppy: copy-pasted from addStageFixedObject
    std::string columnGridFilepathBase =
        "../data/columngrids/" + staticScene.columnGridSetName_;
    staticScene.columnGridSet_.load(columnGridFilepathBase);

    ESP_CHECK(staticScene.columnGridSet_.getSphereRadii() ==
                  collection.collisionRadiusWorkingSet,
              "ColumnGridSet "
                  << staticScene.columnGridSetName_ << " with radii "
                  << staticScene.columnGridSet_.getSphereRadii()
                  << " doesn't match collection collision radius working set "
                  << collection.collisionRadiusWorkingSet);

    staticScene.needsPostLoadFixup_ = false;

    // update set.allEpisodesAABB_
    {
      const auto& columnGrid = staticScene.columnGridSet_.getColumnGrid(0);
      constexpr float dummyMinY = -1e7f;
      constexpr float dummyMaxY = 1e7f;
      Mn::Vector3 cgMin(columnGrid.minX, dummyMinY, columnGrid.minZ);
      Mn::Vector3 cgMax(columnGrid.getMaxX(), dummyMaxY, columnGrid.getMaxZ());

      if (isFirstFixedObj) {
        set.allEpisodesAABB_ = Mn::Range3D(cgMin, cgMax);
        isFirstFixedObj = false;
      } else {
        set.allEpisodesAABB_.min() =
            Mn::Math::min(set.allEpisodesAABB_.min(), cgMin);
        set.allEpisodesAABB_.max() =
            Mn::Math::max(set.allEpisodesAABB_.max(), cgMax);
      }
    }
  }

  set.maxFreeObjects_ = 0;
  for (int i = 0; i < set.episodes_.size(); i++) {
    const auto& episode = safeVectorGet(set.episodes_, i);
    set.maxFreeObjects_ = Mn::Math::max(set.maxFreeObjects_,
                                        (int32_t)episode.numFreeObjectSpawns_);

    ESP_CHECK(episode.targetObjGoalRotation_.isNormalized(),
              "postLoadFixup: episode " << i << " targetObjGoalRotation "
                                        << episode.targetObjGoalRotation_
                                        << " isn't normalized");
    ESP_CHECK(episode.numFreeObjectSpawns_ > 0,
              "postLoadFixup: episode " << i
                                        << " has invalid numFreeObjectSpawns_="
                                        << episode.numFreeObjectSpawns_);
    ESP_CHECK(episode.staticSceneIndex_ >= 0 &&
                  episode.staticSceneIndex_ < set.staticScenes_.size(),
              "postLoadFixup: episode " << i
                                        << " has invalid staticSceneIndex_="
                                        << episode.staticSceneIndex_);
    ESP_CHECK(episode.targetObjIndex_ >= 0 &&
                  episode.targetObjIndex_ < episode.numFreeObjectSpawns_,
              "postLoadFixup: episode " << i << " has invalid targetObjIndex_="
                                        << episode.targetObjIndex_);
  }

  updateFromSerializeCollection(set, collection);

  for (auto& freeObj : set.freeObjects_) {
    freeObj.needsPostLoadFixup_ = false;
  }
  set.needsPostLoadFixup_ = false;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, FreeObject& val) {
  // sloppy: some fields here are populated by updateFromSerializeCollection so
  // they shouldn't be saved/loaded
  esp::io::readMember(obj, "renderAssetIndex", val.renderAssetIndex_);
  // esp::io::readMember(obj, "aabb", val.aabb_);
  esp::io::readMember(obj, "startRotations", val.startRotations_);
  ESP_CHECK(!val.startRotations_.empty(),
            "FreeObject has empty startRotations");
  // esp::io::readMember(obj, "heldRotationIndex", val.heldRotationIndex_);
  // esp::io::readMember(obj, "collisionSpheres", val.collisionSpheres_);

  // for instanceBlueprint_, aabb_, heldRotationIndex_, collisionSpheres_
  val.needsPostLoadFixup_ = true;

  return true;
}

esp::io::JsonGenericValue toJsonValue(const FreeObject& x,
                                      esp::io::JsonAllocator& allocator) {
  // sloppy: some fields here are populated by updateFromSerializeCollection so
  // they shouldn't be saved/loaded
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "renderAssetIndex", x.renderAssetIndex_, allocator);
  // esp::io::addMember(obj, "aabb", x.aabb_, allocator);
  esp::io::addMember(obj, "startRotations", x.startRotations_, allocator);
  // esp::io::addMember(obj, "heldRotationIndex", x.heldRotationIndex_,
  // allocator); esp::io::addMember(obj, "collisionSpheres",
  // x.collisionSpheres_, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, CollisionSphere& val) {
  esp::io::readMember(obj, "origin", val.origin);
  esp::io::readMember(obj, "radiusIdx", val.radiusIdx);

  return true;
}

esp::io::JsonGenericValue toJsonValue(const CollisionSphere& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "origin", x.origin, allocator);
  esp::io::addMember(obj, "radiusIdx", x.radiusIdx, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Transform& val) {
  esp::io::readMember(obj, "origin", val.origin_);
  esp::io::readMember(obj, "rotation", val.rotation_);
  esp::io::readMember(obj, "scale", val.scale_);

  return true;
}

esp::io::JsonGenericValue toJsonValue(const Transform& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "origin", x.origin_, allocator);
  esp::io::addMember(obj, "rotation", x.rotation_, allocator);
  esp::io::addMember(obj, "scale", x.scale_, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   RenderAssetInstance& val) {
  esp::io::readMember(obj, "renderAssetIndex", val.renderAssetIndex_);
  esp::io::readMember(obj, "transform", val.transform_);

  return true;
}

esp::io::JsonGenericValue toJsonValue(const RenderAssetInstance& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "renderAssetIndex", x.renderAssetIndex_, allocator);
  esp::io::addMember(obj, "transform", x.transform_, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, RenderAsset& val) {
  esp::io::readMember(obj, "name", val.name_);

  // for instanceBlueprint_
  val.needsPostLoadFixup_ = true;

  return true;
}

esp::io::JsonGenericValue toJsonValue(const RenderAsset& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "name", x.name_, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, StaticScene& val) {
  esp::io::readMember(obj, "name", val.name_);
  esp::io::readMember(obj, "renderAssetInstances", val.renderAssetInstances_);
  esp::io::readMember(obj, "columnGridSetName", val.columnGridSetName_);

  // for columnGridSet_
  val.needsPostLoadFixup_ = true;

  return true;
}

esp::io::JsonGenericValue toJsonValue(const StaticScene& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "name", x.name_, allocator);
  esp::io::addMember(obj, "renderAssetInstances", x.renderAssetInstances_,
                     allocator);
  esp::io::addMember(obj, "columnGridSetName", x.columnGridSetName_, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, FreeObjectSpawn& val) {
  bool ok = true;
  ok &= esp::io::readMember(obj, "freeObjIndex", val.freeObjIndex_);
#ifdef EPISODESET_DISCRETE_SPAWN_ROTATIONS
  ok &= esp::io::readMember(obj, "startRotationIndex", val.startRotationIndex_);
#else
  ok &= esp::io::readMember(obj, "startRotation", val.startRotation_);
#endif
  ok &= esp::io::readMember(obj, "startPos", val.startPos_);
  ESP_CHECK(ok, "fromJsonValue: failed to parse FreeObjectSpawn");

  return true;
}

esp::io::JsonGenericValue toJsonValue(const FreeObjectSpawn& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "freeObjIndex", x.freeObjIndex_, allocator);
#ifdef EPISODESET_DISCRETE_SPAWN_ROTATIONS
  esp::io::addMember(obj, "startRotationIndex", x.startRotationIndex_,
                     allocator);
#else
  esp::io::addMember(obj, "startRotation", x.startRotation_, allocator);
#endif
  esp::io::addMember(obj, "startPos", x.startPos_, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Episode& val) {
  esp::io::readMember(obj, "staticSceneIndex", val.staticSceneIndex_);
  esp::io::readMember(obj, "numFreeObjectSpawns", val.numFreeObjectSpawns_);
  esp::io::readMember(obj, "targetObjIndex", val.targetObjIndex_);
  esp::io::readMember(obj, "firstFreeObjectSpawnIndex",
                      val.firstFreeObjectSpawnIndex_);
  esp::io::readMember(obj, "agentStartPos", val.agentStartPos_);
  esp::io::readMember(obj, "agentStartYaw", val.agentStartYaw_);
  esp::io::readMember(obj, "robotStartJointPositions",
                      val.robotStartJointPositions_);
  esp::io::readMember(obj, "targetObjGoalPos", val.targetObjGoalPos_);
  esp::io::readMember(obj, "targetObjGoalRotation", val.targetObjGoalRotation_);
  return true;
}

esp::io::JsonGenericValue toJsonValue(const Episode& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "staticSceneIndex", x.staticSceneIndex_, allocator);
  esp::io::addMember(obj, "numFreeObjectSpawns", x.numFreeObjectSpawns_,
                     allocator);
  esp::io::addMember(obj, "targetObjIndex", x.targetObjIndex_, allocator);
  esp::io::addMember(obj, "firstFreeObjectSpawnIndex",
                     x.firstFreeObjectSpawnIndex_, allocator);
  esp::io::addMember(obj, "agentStartPos", x.agentStartPos_, allocator);
  esp::io::addMember(obj, "agentStartYaw", x.agentStartYaw_, allocator);
  esp::io::addMember(obj, "robotStartJointPositions",
                     x.robotStartJointPositions_, allocator);
  esp::io::addMember(obj, "targetObjGoalPos", x.targetObjGoalPos_, allocator);
  esp::io::addMember(obj, "targetObjGoalRotation", x.targetObjGoalRotation_,
                     allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, EpisodeSet& val) {
  esp::io::readMember(obj, "renderAssets", val.renderAssets_);
  esp::io::readMember(obj, "staticScenes", val.staticScenes_);
  esp::io::readMember(obj, "freeObjects", val.freeObjects_);
  esp::io::readMember(obj, "freeObjectSpawns", val.freeObjectSpawns_);
  esp::io::readMember(obj, "episodes", val.episodes_);

  // for maxFreeObjects_ and allEpisodesAABB_
  val.needsPostLoadFixup_ = true;
  val.maxFreeObjects_ = -1;

  return true;
}

esp::io::JsonGenericValue toJsonValue(const EpisodeSet& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "renderAssets", x.renderAssets_, allocator);
  esp::io::addMember(obj, "staticScenes", x.staticScenes_, allocator);
  esp::io::addMember(obj, "freeObjects", x.freeObjects_, allocator);
  esp::io::addMember(obj, "freeObjectSpawns", x.freeObjectSpawns_, allocator);
  esp::io::addMember(obj, "episodes", x.episodes_, allocator);
  // don't write maxFreeObjects_

  return obj;
}

EpisodeSet EpisodeSet::loadFromFile(const std::string& filepath) {
  EpisodeSet episodeSet;
  ESP_CHECK(Cr::Utility::Directory::exists(filepath),
            "couldn't find EpisodeSet file " << filepath);
  auto newDoc = esp::io::parseJsonFile(filepath);
  esp::io::readMember(newDoc, "episodeSet", episodeSet);

  return episodeSet;
}

void EpisodeSet::saveToFile(const std::string& filepath) const {
  rapidjson::Document document(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
  esp::io::addMember(document, "episodeSet", *this, allocator);

  // EpisodeSet use floats (not doubles) so this is plenty of precision
  const float maxDecimalPlaces = 7;
  constexpr bool usePrettyWriter = false;
  bool success = esp::io::writeJsonToFile(document, filepath, usePrettyWriter,
                                          maxDecimalPlaces);
  ESP_CHECK(success, "failed to save EpisodeSet to " << filepath);
}

}  // namespace batched_sim
}  // namespace esp
