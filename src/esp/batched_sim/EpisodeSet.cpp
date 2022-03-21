// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/EpisodeSet.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/PlacementHelper.h"

#include "esp/core/random.h"
#include "esp/core/Check.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

namespace {

void addStageFixedObject(EpisodeSet& set, const std::string& name, 
  const BpsSceneMapping& sceneMapping, const serialize::Collection& collection) {

  FixedObject fixedObj;
  fixedObj.name_ = name;
  fixedObj.instanceBlueprint_ = sceneMapping.findInstanceBlueprint(name);

  std::string columnGridFilepathBase = "../data/columngrids/" + name + "_stage_only";
  fixedObj.columnGridSet_.load(columnGridFilepathBase);

  ESP_CHECK(fixedObj.columnGridSet_.getSphereRadii() == collection.collisionRadiusWorkingSet,
    "ColumnGridSet " << name << " with radii " << fixedObj.columnGridSet_.getSphereRadii()
    << " doesn't match collection collision radius working set " 
    << collection.collisionRadiusWorkingSet);

  set.fixedObjects_.push_back(std::move(fixedObj));
}

// radius of sphere at origin that bounds this AABB
float getOriginBoundingSphereRadiusSquaredForAABB(const Magnum::Range3D& aabb) {
  auto absMin = Mn::Math::abs(aabb.min());
  Mn::Vector3 maxCorner = Mn::Math::max(absMin, aabb.max());
  return maxCorner.dot();
}

void addFreeObject(EpisodeSet& set, const std::string& name, const BpsSceneMapping& sceneMapping) {

  FreeObject freeObj;
  freeObj.name_ = name;
  freeObj.instanceBlueprint_ = sceneMapping.findInstanceBlueprint(name);

#if 0
  freeObj.aabb_ = aabb;
 //  freeObj.boundingSphereRadiusSq_ = getOriginBoundingSphereRadiusSquaredForAABB(aabb);

  // add one collision sphere at base of aabb
  constexpr float sphereRadius = 0.1f; // temp
  freeObj.collisionSphereLocalOrigins_.push_back(
    {aabb.center().x(), aabb.center().y(), aabb.min().z() + sphereRadius});
#endif

  // all YCB objects needs this to be upright
  const auto baseRot = Mn::Quaternion::rotation(Mn::Deg(-90), Mn::Vector3(1.f, 0.f, 0.f));

  constexpr int numRotationsAboutUpAxis = 32;
  for (int i = 0; i < numRotationsAboutUpAxis; i++) {
    const auto angle = Mn::Deg((float)i * 360.f / numRotationsAboutUpAxis);
    const auto rotAboutUpAxis = Mn::Quaternion::rotation(angle, Mn::Vector3(0.f, 1.f, 0.f));
    freeObj.startRotations_.push_back((rotAboutUpAxis * baseRot).toMatrix());
  }

  set.freeObjects_.emplace_back(std::move(freeObj));  
}

void addEpisode(EpisodeSet& set, const serialize::Collection& collection, int stageFixedObjectIndex, core::Random& random) {
  Episode episode;
  episode.stageFixedObjIndex = stageFixedObjectIndex;
  episode.firstFreeObjectSpawnIndex_ = set.freeObjectSpawns_.size();

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
        rotation, randomPos);

    if (placementHelper.place(mat, freeObject)) {
      if (!spawnRange.contains(mat.translation())) {
        continue;
      }
      spawn.startPos_ = mat.translation();

      if (isGoalPositionAttempt) {
        episode.targetObjGoalPos_ = spawn.startPos_;
        success = true;
        break;
      } else {
        set.freeObjectSpawns_.emplace_back(std::move(spawn));
        episode.numFreeObjectSpawns_++;

        // add to colGrid so future spawns don't intersect this one
        colGrid.insertObstacle(spawn.startPos_, Mn::Quaternion::fromMatrix(rotation), &freeObject.aabb_);
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

  set.maxFreeObjects_ = 0;
  
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
    addStageFixedObject(set, stageName, sceneMapping, collection);
  }

  for (const auto& serFreeObject : collection.freeObjects) {
    addFreeObject(set, serFreeObject.name, sceneMapping);
  }

  updateFromSerializeCollection(set, collection);

  // distribute stages across episodes
  for (int i = 0; i < numEpisodes; i++) {
    int stageIndex = i * set.fixedObjects_.size() / numEpisodes;
    addEpisode(set, collection, stageIndex, random);
  }

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

}  // namespace batched_sim
}  // namespace esp
