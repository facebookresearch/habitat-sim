// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/EpisodeSet.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/PlacementHelper.h"

#include "esp/core/random.h"
#include "esp/core/Check.h"
#include "esp/io/json.h"

#include <Corrade/Utility/Directory.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

namespace {

void addStageFixedObject(EpisodeSet& set, const std::string& name) {

  FixedObject fixedObj;
  fixedObj.name_ = name;
  fixedObj.needsPostLoadFixup_ = true;

  set.fixedObjects_.push_back(std::move(fixedObj));
}

// radius of sphere at origin that bounds this AABB
float getOriginBoundingSphereRadiusSquaredForAABB(const Magnum::Range3D& aabb) {
  auto absMin = Mn::Math::abs(aabb.min());
  Mn::Vector3 maxCorner = Mn::Math::max(absMin, aabb.max());
  return maxCorner.dot();
}

void addFreeObject(EpisodeSet& set, const std::string& name) {

  FreeObject freeObj;
  freeObj.name_ = name;
  freeObj.needsPostLoadFixup_ = true;

  // all YCB objects needs this to be upright
  const auto baseRot = Mn::Quaternion::rotation(Mn::Deg(-90), Mn::Vector3(1.f, 0.f, 0.f));

  constexpr int numRotationsAboutUpAxis = 32;
  for (int i = 0; i < numRotationsAboutUpAxis; i++) {
    const auto angle = Mn::Deg((float)i * 360.f / numRotationsAboutUpAxis);
    const auto rotAboutUpAxis = Mn::Quaternion::rotation(angle, Mn::Vector3(0.f, 1.f, 0.f));
    freeObj.startRotations_.push_back((rotAboutUpAxis * baseRot));
  }

  set.freeObjects_.emplace_back(std::move(freeObj));  
}

void addEpisode(EpisodeSet& set, const serialize::Collection& collection, int stageFixedObjectIndex, core::Random& random) {
  Episode episode;
  episode.stageFixedObjIndex = stageFixedObjectIndex;
  episode.firstFreeObjectSpawnIndex_ = set.freeObjectSpawns_.size();

  // place robot agent
  episode.agentStartPos_ = Mn::Vector2(2.59f, 0.f);
  episode.agentStartYaw_ = random.uniform_float(-float(Mn::Rad(Mn::Deg(135.f))), -float(Mn::Rad(Mn::Deg(45.f))));

  // keep object count close to 28 (from Hab 2.0 benchmark), but include variation
  int targetNumSpawns = random.uniform_int(28, 33);
  episode.numFreeObjectSpawns_ = 0;

  // good for area around staircase and living room
  Mn::Range3D spawnRange({-1.f, 0.15f, -0.5f}, {4.f, 2.f, 3.f});

  // good for white bookshelf for stage 5
  // Mn::Range3D spawnRange({0.33f, 0.15f, -0.4f}, {1.18f, 1.85f, -0.25f});

  const auto robotStartPos = Mn::Vector3(2.59, 0.f, 0.f);
  const auto pad = Mn::Vector3(0.9f, 2.f, 0.9);
  const auto exclusionRange = Mn::Range3D(robotStartPos - pad, robotStartPos + pad);

  const auto& stageFixedObject = safeVectorGet(set.fixedObjects_, episode.stageFixedObjIndex);
  const auto& columnGrid = stageFixedObject.columnGridSet_.getColumnGrid(0);      
  // perf todo: re-use this across entire set (have extents for set)
  // todo: find extents for entire EpisodeSet, not just this specific columnGrid
  constexpr int maxBytes = 1000 * 1024;
  // this is tuned assuming a building-scale simulation with household-object-scale obstacles
  constexpr float maxGridSpacing = 0.5f;
  CollisionBroadphaseGrid colGrid = CollisionBroadphaseGrid(getMaxCollisionRadius(collection), 
    columnGrid.minX, columnGrid.minZ,
    columnGrid.getMaxX(), columnGrid.getMaxZ(),
    maxBytes, maxGridSpacing);

  constexpr int maxFailedPlacements = 3;
  PlacementHelper placementHelper(stageFixedObject.columnGridSet_, 
    colGrid, collection, random, maxFailedPlacements);

  std::array<int, 6> selectedFreeObjectIndices = {1, 2, 3, 4, 5, 8};

  episode.targetObjIndex_ = 0; // arbitrary
  int numSpawnAttempts = 2000;
  bool success = false;
  for (int i = 0; i < numSpawnAttempts; i++) {

    // After finding all object spawns, do a search for the target object's goal 
    // position. This logic is mostly identical to finding an object spawn.
    bool isGoalPositionAttempt = (episode.numFreeObjectSpawns_ == targetNumSpawns);

    FreeObjectSpawn spawn;
    // for the goal position, use the free object correspnding to targetObjIdx
    spawn.freeObjIndex_ = isGoalPositionAttempt
      ? set.freeObjectSpawns_[episode.targetObjIndex_].freeObjIndex_
      : selectedFreeObjectIndices[random.uniform_int(0, selectedFreeObjectIndices.size())];
    const auto& freeObject = safeVectorGet(set.freeObjects_, spawn.freeObjIndex_);
    spawn.startRotationIndex_ = random.uniform_int(0, freeObject.startRotations_.size());

    Mn::Vector3 randomPos;
    int numAttempts = 0;
    while (true) {
      numAttempts++;
      randomPos = Mn::Vector3(
        random.uniform_float(spawnRange.min().x(), spawnRange.max().x()),
        random.uniform_float(spawnRange.min().y(), spawnRange.max().y()),
        random.uniform_float(spawnRange.min().z(), spawnRange.max().z()));

      if (!exclusionRange.contains(randomPos)) {
        break;
      }
      BATCHED_SIM_ASSERT(numAttempts < 1000);
    }

    const auto rotation = freeObject.startRotations_[spawn.startRotationIndex_];
    Mn::Matrix4 mat = Mn::Matrix4::from(
        rotation.toMatrix(), randomPos);

    if (placementHelper.place(mat, freeObject)) {
      if (!spawnRange.contains(mat.translation())) {
        continue;
      }
      spawn.startPos_ = mat.translation();

      if (isGoalPositionAttempt) {
        episode.targetObjGoalPos_ = spawn.startPos_;
        episode.targetObjGoalRotation_ = rotation;
        success = true;
        break;
      } else {
        set.freeObjectSpawns_.emplace_back(std::move(spawn));
        episode.numFreeObjectSpawns_++;

        // add to colGrid so future spawns don't intersect this one
        colGrid.insertObstacle(spawn.startPos_, rotation, &freeObject.aabb_);
      }
    }
  }
  constexpr int numGoalPositions = 1;
  ESP_CHECK(success, "episode-generation failed; couldn't find " 
    << (targetNumSpawns + numGoalPositions) << " collision-free spawn locations");

  set.maxFreeObjects_ = Mn::Math::max(set.maxFreeObjects_, (int32_t)episode.numFreeObjectSpawns_);

  set.episodes_.emplace_back(std::move(episode));
}

}

EpisodeSet generateBenchmarkEpisodeSet(int numEpisodes, 
  const BpsSceneMapping& sceneMapping, 
  const serialize::Collection& collection) {

  // 5 is hand-picked for a demo
  core::Random random(/*seed*/3);

  EpisodeSet set;

  std::vector<std::string> replicaCadBakedStages = {
    "Baked_sc0_staging_00",
    "Baked_sc0_staging_01",
    "Baked_sc0_staging_02",
    "Baked_sc0_staging_03",
    "Baked_sc0_staging_04",
    "Baked_sc0_staging_05",
    "Baked_sc0_staging_06",
    "Baked_sc0_staging_07",
    "Baked_sc0_staging_08",
    "Baked_sc0_staging_09",
    "Baked_sc0_staging_10",
    "Baked_sc0_staging_11",
    // "Baked_sc0_staging_12",
  };

  for (const auto& stageName : replicaCadBakedStages) {
    addStageFixedObject(set, stageName);
  }

  for (const auto& serFreeObject : collection.freeObjects) {
    addFreeObject(set, serFreeObject.name);
  }

  // sloppy: call postLoadFixup before adding episodes; this means that
  // set.maxFreeObjects_ gets computed incorrectly in here (but it will get computed
  // correctly, incrementally, in addEpisode).
  postLoadFixup(set, sceneMapping, collection);

  // distribute stages across episodes
  for (int i = 0; i < numEpisodes; i++) {
    int stageIndex = i * set.fixedObjects_.size() / numEpisodes;
    addEpisode(set, collection, stageIndex, random);
  }
  BATCHED_SIM_ASSERT(set.maxFreeObjects_ > 0);

  return set;
}


void updateFromSerializeCollection(EpisodeSet& set, const serialize::Collection& collection) {

  for (const auto& serFreeObject : collection.freeObjects) {

    auto it = std::find_if(set.freeObjects_.begin(), set.freeObjects_.end(),
      [&serFreeObject](const auto& item) { return item.name_ == serFreeObject.name; });
    ESP_CHECK(it != set.freeObjects_.end(), "collection free object with name " <<
      serFreeObject.name << " not found in EpisodeSet. If you hit this error during "
      "hot-reloading, try restarting the simulator.");

    auto& freeObject = *it;
    freeObject.aabb_ = Mn::Range3D(serFreeObject.collisionBox.min, serFreeObject.collisionBox.max);
    freeObject.heldRotationIndex_ = serFreeObject.heldRotationIndex;
    ESP_CHECK(freeObject.heldRotationIndex_ >= 0 
      && freeObject.heldRotationIndex_ < freeObject.startRotations_.size(),
      "updateFromSerializeCollection: heldRotationIndex " << serFreeObject.heldRotationIndex 
      << " is out-of-range for FreeObject "
      << freeObject.name_ << " with startRotations.size() == " << freeObject.startRotations_.size());

    freeObject.collisionSpheres_.clear();
    std::vector<serialize::Sphere> generatedSpheres;
    const std::vector<serialize::Sphere>* serializeCollisionSpheres = nullptr;

    if (!serFreeObject.generateCollisionSpheresTechnique.empty()) {

      auto& spheres = generatedSpheres;
      serializeCollisionSpheres = &generatedSpheres;

      float smallRadius = 0.015f;
      float mediumRadius = 0.05f;

      const auto& aabb = freeObject.aabb_;
      Mn::Vector3 aabbCenter = aabb.center();

      if (serFreeObject.generateCollisionSpheresTechnique == "box") {

        // small and medium spheres at each corner
        // consolidate duplicates at the end

        // insert larger spheres first, so that de-duplication (later) leaves larger spheres
        for (float r : {mediumRadius, smallRadius}) {
          if (aabb.size().length() < r * 2.f) {
            // object is too small for even one sphere of this radius
            continue;
          }
          if (aabb.sizeZ() < r * 2.f) {
            continue;
          }
          spheres.push_back({aabb.backBottomLeft(), r});
          spheres.push_back({aabb.backBottomRight(), r});
          spheres.push_back({aabb.backTopLeft(), r});
          spheres.push_back({aabb.backTopRight(), r});
          spheres.push_back({aabb.frontBottomLeft(), r});
          spheres.push_back({aabb.frontBottomRight(), r});
          spheres.push_back({aabb.frontTopLeft(), r});
          spheres.push_back({aabb.frontTopRight(), r});
        }

      } else if (serFreeObject.generateCollisionSpheresTechnique == "uprightCylinder") {

        // insert larger spheres first, so that de-duplication (later) leaves larger spheres
        for (float r : {mediumRadius, smallRadius}) {
          if (aabb.size().length() < r * 2.f) {
            // object is too small for even one sphere of this radius
            continue;
          }

          for (float z : {aabb.min().z(), aabb.max().z()}) {
            for (int xyDim = 0; xyDim < 2; xyDim++) {
              int otherXyDim = xyDim == 0 ? 1 : 0;
              Mn::Vector3 pMin;
              pMin[xyDim] = aabb.min()[xyDim];
              pMin[otherXyDim] = aabb.center()[otherXyDim];
              pMin.z() = z;

              Mn::Vector3 pMax;
              pMax[xyDim] = aabb.max()[xyDim];
              pMax[otherXyDim] = aabb.center()[otherXyDim];
              pMax.z() = z;

              spheres.push_back({pMin, r});
              spheres.push_back({pMax, r});
            }
          }
        }

      } else {
        ESP_CHECK(false, "free object generateCollisionSpheresTechnique \"" 
          << serFreeObject.generateCollisionSpheresTechnique << "\" not recognized. "
          "Valid values are empty-string, \"box\", and \"uprightCylinder\"");
      }

      // clamp to fit inside box extents, but don't move sphere center past center of box (to other side)
      for (auto& sphere : spheres) {
        Mn::Vector3 clampedOrigin;
        for (int dim = 0; dim < 3; dim++) {
          clampedOrigin[dim] = sphere.origin[dim] < aabbCenter[dim]
            ? Mn::Math::clamp(sphere.origin[dim], 
              Mn::Math::min(aabb.min()[dim] + sphere.radius, aabbCenter[dim]), aabbCenter[dim])
            : Mn::Math::clamp(sphere.origin[dim], 
              aabbCenter[dim], Mn::Math::max(aabb.max()[dim] - sphere.radius, aabbCenter[dim]));
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
      ESP_CHECK(!serFreeObject.collisionSpheres.empty(), "no collision spheres for free object "
        << serFreeObject.name << " and generateCollisionSpheresFromBox==false");
      serializeCollisionSpheres = &serFreeObject.collisionSpheres;
    }

    for (const auto& serSphere : *serializeCollisionSpheres) {
      int radiusIdx = getCollisionRadiusIndex(collection, serSphere.radius);
      freeObject.collisionSpheres_.push_back({serSphere.origin, radiusIdx});
    }
  }
}

void postLoadFixup(EpisodeSet& set, const BpsSceneMapping& sceneMapping, const serialize::Collection& collection) {

  for (auto& freeObj : set.freeObjects_) {
    freeObj.instanceBlueprint_ = sceneMapping.findInstanceBlueprint(freeObj.name_);
    freeObj.needsPostLoadFixup_ = false;    
    for (const auto& rot : freeObj.startRotations_) {
      ESP_CHECK(rot.isNormalized(), "postLoadFixup: FreeObject " << freeObj.name_ << " rotation " << rot << " isn't normalized");
    }
  }
  
  set.allEpisodesAABB_ = Mn::Range3D(Mn::Math::ZeroInit);
  bool isFirstFixedObj = true;
  for (auto& fixedObj : set.fixedObjects_) {
    fixedObj.instanceBlueprint_ = sceneMapping.findInstanceBlueprint(fixedObj.name_);

    // sloppy: copy-pasted from addStageFixedObject
    std::string columnGridFilepathBase = "../data/columngrids/" + fixedObj.name_ + "_stage_only";
    fixedObj.columnGridSet_.load(columnGridFilepathBase);

    ESP_CHECK(fixedObj.columnGridSet_.getSphereRadii() == collection.collisionRadiusWorkingSet,
      "ColumnGridSet " << fixedObj.name_ << " with radii " << fixedObj.columnGridSet_.getSphereRadii()
      << " doesn't match collection collision radius working set " 
      << collection.collisionRadiusWorkingSet);

    fixedObj.needsPostLoadFixup_ = false;

    // update set.allEpisodesAABB_
    {
      const auto& columnGrid = fixedObj.columnGridSet_.getColumnGrid(0);
      constexpr float dummyMinY = -1e7f;
      constexpr float dummyMaxY = 1e7f;
      Mn::Vector3 cgMin(
        columnGrid.minX,
        dummyMinY,
        columnGrid.minZ);
      Mn::Vector3 cgMax(
        columnGrid.getMaxX(),
        dummyMaxY,
        columnGrid.getMaxZ());

      if (isFirstFixedObj) {
        set.allEpisodesAABB_ = Mn::Range3D(cgMin, cgMax);
        isFirstFixedObj = false;
      } else {
        set.allEpisodesAABB_.min() = Mn::Math::min(set.allEpisodesAABB_.min(), cgMin);
        set.allEpisodesAABB_.max() = Mn::Math::max(set.allEpisodesAABB_.max(), cgMax);
      }
    }
  }

  set.maxFreeObjects_ = 0;
  for (int i = 0; i < set.episodes_.size(); i++) {
    const auto& episode = safeVectorGet(set.episodes_, i);
    set.maxFreeObjects_ = Mn::Math::max(set.maxFreeObjects_, (int32_t)episode.numFreeObjectSpawns_);

    ESP_CHECK(episode.targetObjGoalRotation_.isNormalized(), 
      "postLoadFixup: episode " << i << " targetObjGoalRotation " 
      << episode.targetObjGoalRotation_ << " isn't normalized");
    ESP_CHECK(episode.numFreeObjectSpawns_ > 0,
      "postLoadFixup: episode " << i << " has invalid numFreeObjectSpawns_=" << episode.numFreeObjectSpawns_);
    ESP_CHECK(episode.stageFixedObjIndex >= 0 && episode.stageFixedObjIndex < set.fixedObjects_.size(),
      "postLoadFixup: episode " << i << " has invalid stageFixedObjIndex=" << episode.stageFixedObjIndex);
    ESP_CHECK(episode.targetObjIndex_ >= 0 && episode.targetObjIndex_ < episode.numFreeObjectSpawns_,
      "postLoadFixup: episode " << i << " has invalid targetObjIndex_=" << episode.targetObjIndex_);
  }
  set.needsPostLoadFixup_ = false;

  updateFromSerializeCollection(set, collection);
}


bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   FreeObject& val) {
  // sloppy: some fields here are populated by updateFromSerializeCollection so they
  // shouldn't be saved/loaded
  esp::io::readMember(obj, "name", val.name_);
  // esp::io::readMember(obj, "aabb", val.aabb_);
  esp::io::readMember(obj, "startRotations", val.startRotations_);
  ESP_CHECK(!val.startRotations_.empty(), "FreeObject " << val.name_ << " has empty startRotations");
  // esp::io::readMember(obj, "heldRotationIndex", val.heldRotationIndex_);
  // esp::io::readMember(obj, "collisionSpheres", val.collisionSpheres_);

  // for instanceBlueprint_, aabb_, heldRotationIndex_, collisionSpheres_
  val.needsPostLoadFixup_ = true;

  return true;
}

esp::io::JsonGenericValue toJsonValue(const FreeObject& x,
                             esp::io::JsonAllocator& allocator) {
  // sloppy: some fields here are populated by updateFromSerializeCollection so they
  // shouldn't be saved/loaded
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "name", x.name_, allocator);
  //esp::io::addMember(obj, "aabb", x.aabb_, allocator);
  esp::io::addMember(obj, "startRotations", x.startRotations_, allocator);
  //esp::io::addMember(obj, "heldRotationIndex", x.heldRotationIndex_, allocator);
  //esp::io::addMember(obj, "collisionSpheres", x.collisionSpheres_, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   CollisionSphere& val) {
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

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   FixedObject& val) {
  esp::io::readMember(obj, "name", val.name_);

  // for instanceBlueprint_ and columnGridSet_
  val.needsPostLoadFixup_ = true;

  return true;
}

esp::io::JsonGenericValue toJsonValue(const FixedObject& x,
                             esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "name", x.name_, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   FreeObjectSpawn& val) {
  esp::io::readMember(obj, "freeObjIndex", val.freeObjIndex_);
  esp::io::readMember(obj, "startRotationIndex", val.startRotationIndex_);
  esp::io::readMember(obj, "startPos", val.startPos_);

  return true;
}

esp::io::JsonGenericValue toJsonValue(const FreeObjectSpawn& x,
                             esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "freeObjIndex", x.freeObjIndex_, allocator);
  esp::io::addMember(obj, "startRotationIndex", x.startRotationIndex_, allocator);
  esp::io::addMember(obj, "startPos", x.startPos_, allocator);

  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   Episode& val) {
  esp::io::readMember(obj, "stageFixedObjIndex", val.stageFixedObjIndex);
  esp::io::readMember(obj, "numFreeObjectSpawns", val.numFreeObjectSpawns_);
  esp::io::readMember(obj, "targetObjIndex", val.targetObjIndex_);
  esp::io::readMember(obj, "firstFreeObjectSpawnIndex", val.firstFreeObjectSpawnIndex_);
  esp::io::readMember(obj, "agentStartPos", val.agentStartPos_);
  esp::io::readMember(obj, "agentStartYaw", val.agentStartYaw_);
  esp::io::readMember(obj, "targetObjGoalPos", val.targetObjGoalPos_);
  esp::io::readMember(obj, "targetObjGoalRotation", val.targetObjGoalRotation_);

  return true;
}

esp::io::JsonGenericValue toJsonValue(const Episode& x,
                             esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "stageFixedObjIndex", x.stageFixedObjIndex, allocator);
  esp::io::addMember(obj, "numFreeObjectSpawns", x.numFreeObjectSpawns_, allocator);
  esp::io::addMember(obj, "targetObjIndex", x.targetObjIndex_, allocator);
  esp::io::addMember(obj, "firstFreeObjectSpawnIndex", x.firstFreeObjectSpawnIndex_, allocator);
  esp::io::addMember(obj, "agentStartPos", x.agentStartPos_, allocator);
  esp::io::addMember(obj, "agentStartYaw", x.agentStartYaw_, allocator);
  esp::io::addMember(obj, "targetObjGoalPos", x.targetObjGoalPos_, allocator);
  esp::io::addMember(obj, "targetObjGoalRotation", x.targetObjGoalRotation_, allocator);

  return obj;
}


bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   EpisodeSet& val) {
  esp::io::readMember(obj, "episodes", val.episodes_);
  esp::io::readMember(obj, "fixedObjects", val.fixedObjects_);
  esp::io::readMember(obj, "freeObjectSpawns", val.freeObjectSpawns_);
  esp::io::readMember(obj, "freeObjects", val.freeObjects_);

  // for maxFreeObjects_ and allEpisodesAABB_
  val.needsPostLoadFixup_ = true;
  val.maxFreeObjects_ = -1;

  return true;
}


esp::io::JsonGenericValue toJsonValue(const EpisodeSet& x,
                             esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "episodes", x.episodes_, allocator);
  esp::io::addMember(obj, "fixedObjects", x.fixedObjects_, allocator);
  esp::io::addMember(obj, "freeObjectSpawns", x.freeObjectSpawns_, allocator);
  esp::io::addMember(obj, "freeObjects", x.freeObjects_, allocator);
  // don't write maxFreeObjects_

  return obj;
}

EpisodeSet EpisodeSet::loadFromFile(const std::string& filepath) {
  EpisodeSet episodeSet;
  ESP_CHECK(Cr::Utility::Directory::exists(filepath), "couldn't find EpisodeSet file " << filepath);
  auto newDoc = esp::io::parseJsonFile(filepath);
  esp::io::readMember(newDoc, "episodeSet", episodeSet);

  return episodeSet;
}


void EpisodeSet::saveToFile(const std::string& filepath)const {

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
