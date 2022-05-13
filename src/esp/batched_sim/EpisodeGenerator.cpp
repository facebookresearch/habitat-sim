// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/EpisodeGenerator.h"
#include "esp/batched_sim/GlmUtils.h"
#include "esp/batched_sim/PlacementHelper.h"

#include "esp/core/Check.h"
#include "esp/core/random.h"
#include "esp/gfx/replay/Keyframe.h"
#include "esp/io/JsonAllTypes.h"
#include "esp/io/json.h"

#include <Corrade/Utility/Directory.h>

#include <rapidjson/document.h>
#include <filesystem>
#include <unordered_map>

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::gfx::replay::Keyframe;
using esp::gfx::replay::RenderAssetInstanceKey;

namespace esp {
namespace batched_sim {

namespace {

Keyframe loadKeyframe(const std::string& filepath) {
  ESP_CHECK(Corrade::Utility::Directory::exists(filepath),
            "loadKeyframe: can't find replay file " << filepath);

  auto newDoc = esp::io::parseJsonFile(filepath);
  std::vector<Keyframe> keyframes;
  esp::io::readMember(newDoc, "keyframes", keyframes);
  ESP_CHECK(keyframes.size() == 1, "loadKeyframe: for replay file "
                                       << filepath
                                       << ", expected 1 keyframe but loaded "
                                       << keyframes.size());
  return std::move(keyframes.front());
}

int findOrInsertRenderAsset(std::vector<RenderAsset>& renderAssets,
                            const std::string& filepathOrName) {
  auto name = std::filesystem::path(filepathOrName).stem();
  auto it = std::find_if(renderAssets.begin(), renderAssets.end(),
                         [&](const auto& item) { return item.name_ == name; });
  if (it != renderAssets.end()) {
    return it - renderAssets.begin();
  } else {
    renderAssets.push_back(RenderAsset{.name_ = name});
    return renderAssets.size() - 1;
  }
}

void addStaticScene(EpisodeSet& set, const std::string& name) {
  StaticScene staticScene;
  staticScene.name_ = name;
  staticScene.columnGridSetName_ = name;

  std::string replayFilepath = "../data/replays/" + name + ".replay.json";
  auto keyframe = loadKeyframe(replayFilepath);

  ESP_CHECK(keyframe.stateUpdates.size() == keyframe.creations.size(),
            "addStaticScene: keyframe.stateUpdates.size() != "
            "keyframe.creations.size()");

  for (int i = 0; i < keyframe.creations.size(); i++) {
    const auto& creationPair = keyframe.creations[i];
    const auto& updatePair = keyframe.stateUpdates[i];
    ESP_CHECK(creationPair.first == i,
              "unexpected instanceKey in " << replayFilepath);
    ESP_CHECK(updatePair.first == i,
              "unexpected instanceKey in " << replayFilepath);
    auto key = creationPair.first;

    const auto& creationInfo = creationPair.second;
    const auto& update = updatePair.second;

    ESP_CHECK(creationInfo.isRGBD(),
              "addStaticScene: unexpected RenderAssetInstanceCreation isRGBD");
    int renderAssetIndex =
        findOrInsertRenderAsset(set.renderAssets_, creationInfo.filepath);

    auto scale =
        creationInfo.scale ? *creationInfo.scale : Mn::Vector3(1.f, 1.f, 1.f);

    Transform transform{.origin_ = update.absTransform.translation,
                        .rotation_ = update.absTransform.rotation,
                        .scale_ = scale};
    RenderAssetInstance instance{.renderAssetIndex_ = renderAssetIndex,
                                 .transform_ = transform};
    staticScene.renderAssetInstances_.push_back(instance);
  }

  staticScene.needsPostLoadFixup_ = true;

  set.staticScenes_.push_back(std::move(staticScene));
}

void addFreeObject(EpisodeSet& set, const std::string& name) {
  int renderAssetIndex = findOrInsertRenderAsset(set.renderAssets_, name);

  FreeObject freeObj;
  freeObj.renderAssetIndex_ = renderAssetIndex;
  freeObj.needsPostLoadFixup_ = true;

  // all YCB objects needs this to be upright
  const auto baseRot =
      Mn::Quaternion::rotation(Mn::Deg(-90), Mn::Vector3(1.f, 0.f, 0.f));

  constexpr int numRotationsAboutUpAxis = 32;
  for (int i = 0; i < numRotationsAboutUpAxis; i++) {
    const auto angle = Mn::Deg((float)i * 360.f / numRotationsAboutUpAxis);
    const auto rotAboutUpAxis =
        Mn::Quaternion::rotation(angle, Mn::Vector3(0.f, 1.f, 0.f));
    freeObj.startRotations_.push_back((rotAboutUpAxis * baseRot));
  }

  set.freeObjects_.emplace_back(std::move(freeObj));
}

FreeObject createFreeObjectProxyForRobot(
    const serialize::Collection& collection) {
#if 0
  const auto& serLink = safeVectorGet(safeVectorGet(collection.robots, 0).links, 0);
  ESP_CHECK(serLink.linkName == "base_link", "createFreeObjectProxyForRobot failed");

  FreeObject freeObject;

  freeObject.aabb_ = Mn::Range3D({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});

  for (const auto& serSphere : serLink.collisionSpheres) {
    int radiusIdx = getCollisionRadiusIndex(collection, serSphere.radius);
    freeObject.collisionSpheres_.push_back({serSphere.origin, radiusIdx});

    freeObject.aabb_.min() = min(freeObject.aabb_.min(),
      serSphere.origin - Mn::Vector3(serSphere.radius, serSphere.radius, serSphere.radius));
    freeObject.aabb_.max() = max(freeObject.aabb_.max(),
      serSphere.origin + Mn::Vector3(serSphere.radius, serSphere.radius, serSphere.radius));
  }
#endif

  FreeObject freeObject;

  freeObject.startRotations_.push_back(Mn::Quaternion(Mn::Math::IdentityInit));

  auto it =
      std::find_if(collection.freeObjects.begin(), collection.freeObjects.end(),
                   [](const auto& item) { return item.name == "robotProxy"; });
  ESP_CHECK(it != collection.freeObjects.end(),
            "createFreeObjectProxyForRobot failed");

  updateFromSerializeCollection(freeObject, *it, collection);
  BATCHED_SIM_ASSERT(!freeObject.collisionSpheres_.empty());

  return freeObject;
}

void setFetchJointStartPositions(const EpisodeGeneratorConfig& config,
                                 Episode& episode,
                                 const serialize::Collection& collection,
                                 core::Random& random) {
  const auto& serRobot = safeVectorGet(collection.robots, 0);
  ESP_CHECK(
      serRobot.actionMap.joints.size() <= Episode::MAX_JOINT_POSITIONS,
      "setJointStartPositions: collection.json robot actionMap.joints.size()="
          << serRobot.actionMap.joints.size()
          << " must be <= Episode::MAX_JOINT_POSITIONS="
          << Episode::MAX_JOINT_POSITIONS);
  if (config.useFixedRobotJointStartPositions) {
    for (int i = 0; i < serRobot.actionMap.joints.size(); i++) {
      const auto& pair = serRobot.actionMap.joints[i];
      int jointIdx = pair.first;
      const auto& setup = pair.second;
      episode.robotStartJointPositions_[i] =
          serRobot.startJointPositions[jointIdx];
    }

  } else {
    constexpr float refAngle = float(Mn::Rad(Mn::Deg(180.f)));
    // for joints 6-12
    constexpr std::array<float, 7> jointsRangeMin = {
        -1.6056,   -1.22099996, -refAngle, -2.25099993,
        -refAngle, -2.16000009, -refAngle};

    constexpr std::array<float, 7> jointsRangeMax = {
        1.6056,   1.51800001, refAngle, 2.25099993,
        refAngle, 2.16000009, refAngle};

    for (int i = 0; i < serRobot.actionMap.joints.size(); i++) {
      const auto& pair = serRobot.actionMap.joints[i];
      int jointIdx = pair.first;
      const auto& setup = pair.second;
      auto& pos = episode.robotStartJointPositions_[i];

      // hacky logic to ensure EE doesn't extend too far from base (where it
      // might be in collision with the environment).
      if (jointIdx == 9 || jointIdx == 11) {
        constexpr float refAngle = (float)Mn::Rad(Mn::Deg(90.f));
        constexpr float pad = (float)Mn::Rad(Mn::Deg(20.f));

        const float rangeMin = safeVectorGet(jointsRangeMin, i);
        const float rangeMax = safeVectorGet(jointsRangeMax, i);
        // near min or max
        pos = (random.uniform_uint() % 2)
                  ? random.uniform_float(rangeMin, rangeMin + pad)
                  : random.uniform_float(rangeMax - pad, rangeMax);
      } else {
        // random angle over entire range
        pos = random.uniform_float(safeVectorGet(jointsRangeMin, i),
                                   safeVectorGet(jointsRangeMax, i));
      }
    }
  }
}

void addEpisode(const EpisodeGeneratorConfig& config,
                EpisodeSet& set,
                const serialize::Collection& collection,
                int staticSceneIndex,
                core::Random& random,
                core::Random& random2,
                const FreeObject& robotProxy) {
  Episode episode;
  episode.staticSceneIndex_ = staticSceneIndex;
  episode.firstFreeObjectSpawnIndex_ = set.freeObjectSpawns_.size();

  // Use a separate rand generator for joint start positions. This is so that
  // toggling random vs fixed joint start positions doesn't affect the rest of
  // episode generation.
  setFetchJointStartPositions(config, episode, collection, random2);

  if (config.useFixedRobotStartPos) {
    episode.agentStartPos_ = Mn::Vector2(2.59f, 0.f);
    episode.agentStartYaw_ =
        config.useFixedRobotStartYaw
            ? -float(Mn::Rad(Mn::Deg(90.f)))
            : random.uniform_float(-float(Mn::Rad(Mn::Deg(135.f))),
                                   -float(Mn::Rad(Mn::Deg(45.f))));
  } else {
    // set to NAN for now and we'll find a robot start pos later in here
    episode.agentStartPos_ = Mn::Vector2(NAN, NAN);
  }

  // 1 spawn for target object, plus nontarget objects
  constexpr int numTargetObjectSpawns = 1;
  int numObjectSpawns = numTargetObjectSpawns +
                        random.uniform_int(config.minNontargetObjects,
                                           config.maxNontargetObjects + 1);
  episode.numFreeObjectSpawns_ = 0;

  // good for area around staircase and living room
  constexpr float allowedSnapDown = 0.05f;
  Mn::Range3D objectSpawnRange({-2.4f, 0.2f, -1.f},
                               {4.3f, 2.f, 4.f});  // above the floor
  Mn::Range3D robotSpawnRange({1.f, 0.05f, -0.5f},
                              {3.3f, 0.05f, 7.0f});  // just above the floor

  // exclusion range is for legacy useFixedRobotStartPos
  const auto robotStartPos = Mn::Vector3(2.59, 0.f, 0.f);
  const auto pad = Mn::Vector3(0.9f, 2.f, 0.9);
  const auto exclusionRange =
      Mn::Range3D(robotStartPos - pad, robotStartPos + pad);

  const auto& staticScene =
      safeVectorGet(set.staticScenes_, episode.staticSceneIndex_);
  const auto& columnGrid = staticScene.columnGridSet_.getColumnGrid(0);
  // perf todo: re-use this across entire set (have extents for set)
  // todo: find extents for entire EpisodeSet, not just this specific columnGrid
  constexpr int maxBytes = 1000 * 1024;
  // this is tuned assuming a building-scale simulation with
  // household-object-scale obstacles
  constexpr float maxGridSpacing = 0.5f;
  CollisionBroadphaseGrid colGrid = CollisionBroadphaseGrid(
      getMaxCollisionRadius(collection), columnGrid.minX, columnGrid.minZ,
      columnGrid.getMaxX(), columnGrid.getMaxZ(), maxBytes, maxGridSpacing);

  constexpr int maxFailedPlacements = 1;
  PlacementHelper placementHelper(staticScene.columnGridSet_, colGrid,
                                  collection, random, maxFailedPlacements);

  episode.targetObjIndex_ = 0;  // arbitrary
  int numSpawnAttempts = 4000;
  bool success = false;
  for (int i = 0; i < numSpawnAttempts; i++) {
    // find a robot spawn after finding all object spawns
    bool isRobotPosAttempt =
        (episode.numFreeObjectSpawns_ == numObjectSpawns) &&
        std::isnan(episode.agentStartPos_.x());

    // find a goal position spawn after finding robot spawn and all object
    // spawns
    bool isGoalPositionAttempt =
        (episode.numFreeObjectSpawns_ == numObjectSpawns) && !isRobotPosAttempt;

    const bool useExclusionRange =
        !isRobotPosAttempt && config.useFixedRobotStartPos;

    const auto& spawnRange =
        isRobotPosAttempt ? robotSpawnRange : objectSpawnRange;

    FreeObjectSpawn spawn;
    const FreeObject* freeObjectPtr = nullptr;
    Mn::Quaternion rotation;
    float robotYaw = 0.f;
    if (isRobotPosAttempt) {
      spawn.freeObjIndex_ = -1;
      freeObjectPtr = &robotProxy;

      robotYaw = config.useFixedRobotStartYaw
                     ? -float(Mn::Rad(Mn::Deg(90.f)))
                     : random.uniform_float(-float(Mn::Rad(Mn::Deg(180.f))),
                                            float(Mn::Rad(Mn::Deg(180.f))));

      rotation = yawToRotation(robotYaw);

    } else {
      // for the goal position, use the free object correspnding to targetObjIdx
      spawn.freeObjIndex_ =
          isGoalPositionAttempt
              ? set.freeObjectSpawns_[episode.targetObjIndex_].freeObjIndex_
              : random.uniform_int(0, set.freeObjects_.size());

      freeObjectPtr = &safeVectorGet(set.freeObjects_, spawn.freeObjIndex_);
      spawn.startRotationIndex_ =
          random.uniform_int(0, freeObjectPtr->startRotations_.size());
      rotation = freeObjectPtr->startRotations_[spawn.startRotationIndex_];
    }
    const auto& freeObject = *freeObjectPtr;

    Mn::Vector3 randomPos;
    int numAttempts = 0;
    while (true) {
      numAttempts++;
      randomPos = Mn::Vector3(
          random.uniform_float(spawnRange.min().x(), spawnRange.max().x()),
          random.uniform_float(spawnRange.min().y(), spawnRange.max().y()),
          random.uniform_float(spawnRange.min().z(), spawnRange.max().z()));

      if (!useExclusionRange || !exclusionRange.contains(randomPos)) {
        break;
      }
      BATCHED_SIM_ASSERT(numAttempts < 1000);
    }

    Mn::Matrix4 mat = Mn::Matrix4::from(rotation.toMatrix(), randomPos);

    if (placementHelper.place(mat, freeObject)) {
      auto adjustedSpawnRange = spawnRange;
      adjustedSpawnRange.min().y() -= allowedSnapDown;
      if (!adjustedSpawnRange.contains(mat.translation())) {
        continue;
      }
      spawn.startPos_ = mat.translation();

      if (isRobotPosAttempt) {
        episode.agentStartPos_ =
            Mn::Vector2(spawn.startPos_.x(), spawn.startPos_.z());
        episode.agentStartYaw_ = robotYaw;
      } else if (isGoalPositionAttempt) {
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
                         << (numObjectSpawns + numGoalPositions)
                         << " collision-free spawn locations with "
                         << "staticSceneIndex_=" << episode.staticSceneIndex_);

  set.maxFreeObjects_ =
      Mn::Math::max(set.maxFreeObjects_, (int32_t)episode.numFreeObjectSpawns_);

  set.episodes_.emplace_back(std::move(episode));
}

}  // namespace

EpisodeSet generateBenchmarkEpisodeSet(
    const EpisodeGeneratorConfig& config,
    const BpsSceneMapping& sceneMapping,
    const serialize::Collection& collection) {
  int numEpisodes = config.numEpisodes;

  core::Random random(config.seed);
  core::Random random2(config.seed);

  EpisodeSet set;

  std::vector<std::string> selectedReplicaCadBakedScenes = {
      "Baked_sc0_staging_00", "Baked_sc0_staging_01", "Baked_sc0_staging_02",
      "Baked_sc0_staging_03", "Baked_sc0_staging_04", "Baked_sc0_staging_05",
      "Baked_sc0_staging_06", "Baked_sc0_staging_07", "Baked_sc0_staging_08",
      "Baked_sc0_staging_09", "Baked_sc0_staging_10", "Baked_sc0_staging_11",
      "Baked_sc0_staging_12", "Baked_sc0_staging_13", "Baked_sc0_staging_14",
      "Baked_sc0_staging_15", "Baked_sc0_staging_16", "Baked_sc0_staging_17",
      "Baked_sc0_staging_18", "Baked_sc0_staging_19", "Baked_sc0_staging_20",
      "Baked_sc1_staging_00", "Baked_sc1_staging_01", "Baked_sc1_staging_02",
      "Baked_sc1_staging_03", "Baked_sc1_staging_04", "Baked_sc1_staging_05",
      "Baked_sc1_staging_06", "Baked_sc1_staging_07", "Baked_sc1_staging_08",
      "Baked_sc1_staging_09", "Baked_sc1_staging_10", "Baked_sc1_staging_11",
      "Baked_sc1_staging_12", "Baked_sc1_staging_13", "Baked_sc1_staging_14",
      "Baked_sc1_staging_15", "Baked_sc1_staging_16", "Baked_sc1_staging_17",
      "Baked_sc1_staging_18", "Baked_sc1_staging_19", "Baked_sc1_staging_20",
      "Baked_sc2_staging_00", "Baked_sc2_staging_01", "Baked_sc2_staging_02",
      "Baked_sc2_staging_03", "Baked_sc2_staging_04", "Baked_sc2_staging_05",
      "Baked_sc2_staging_06", "Baked_sc2_staging_07", "Baked_sc2_staging_08",
      "Baked_sc2_staging_09", "Baked_sc2_staging_10", "Baked_sc2_staging_11",
      "Baked_sc2_staging_12", "Baked_sc2_staging_13", "Baked_sc2_staging_14",
      "Baked_sc2_staging_15", "Baked_sc2_staging_16", "Baked_sc2_staging_17",
      "Baked_sc2_staging_18", "Baked_sc2_staging_19", "Baked_sc2_staging_20",
      "Baked_sc3_staging_00", "Baked_sc3_staging_01", "Baked_sc3_staging_02",
      "Baked_sc3_staging_03", "Baked_sc3_staging_04", "Baked_sc3_staging_05",
      "Baked_sc3_staging_06", "Baked_sc3_staging_07", "Baked_sc3_staging_08",
      "Baked_sc3_staging_09", "Baked_sc3_staging_10", "Baked_sc3_staging_11",
      "Baked_sc3_staging_12", "Baked_sc3_staging_13", "Baked_sc3_staging_14",
      "Baked_sc3_staging_15", "Baked_sc3_staging_16", "Baked_sc3_staging_17",
      "Baked_sc3_staging_18", "Baked_sc3_staging_19", "Baked_sc3_staging_20",

  };
  ESP_CHECK(config.numStageVariations <= selectedReplicaCadBakedScenes.size(),
            "generateBenchmarkEpisodeSet: config.numStageVariations="
                << config.numStageVariations
                << " must be <= selectedReplicaCadBakedScenes.size()="
                << selectedReplicaCadBakedScenes.size());

  for (int i = 0; i < config.numStageVariations; i++) {
    addStaticScene(set, selectedReplicaCadBakedScenes[i]);
  }

  std::vector<std::string> selectedYCBObjects = {
      "024_bowl",
      "003_cracker_box",
      "010_potted_meat_can",
      "002_master_chef_can",
      "004_sugar_box",
      "005_tomato_soup_can",
      "009_gelatin_box",
      "008_pudding_box",
      "007_tuna_fish_can",
  };
  ESP_CHECK(config.numObjectVariations <= selectedYCBObjects.size(),
            "generateBenchmarkEpisodeSet: config.numObjectVariations="
                << config.numObjectVariations
                << " must be <= selectedYCBObjects.size()="
                << selectedYCBObjects.size());

  for (int i = 0; i < config.numObjectVariations; i++) {
    addFreeObject(set, selectedYCBObjects[i]);
  }

  // sloppy: call postLoadFixup before adding episodes; this means that
  // set.maxFreeObjects_ gets computed incorrectly in here (but it will get
  // computed correctly, incrementally, in addEpisode).
  postLoadFixup(set, sceneMapping, collection);

  const auto robotProxy = createFreeObjectProxyForRobot(collection);

  // distribute scenes across episodes
  for (int i = 0; i < numEpisodes; i++) {
    int sceneIndex = i * config.numStageVariations / numEpisodes;
    addEpisode(config, set, collection, sceneIndex, random, random2,
               robotProxy);
  }
  BATCHED_SIM_ASSERT(set.maxFreeObjects_ > 0);

  return set;
}

}  // namespace batched_sim
}  // namespace esp
