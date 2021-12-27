// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/EpisodeSet.h"
#include "esp/batched_sim/BatchedSimAssert.h"

#include "esp/core/random.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

namespace {

void addStageFixedObject(EpisodeSet& set, const std::string& name, const BpsSceneMapping& sceneMapping) {

  FixedObject fixedObj;
  fixedObj.name_ = name;
  fixedObj.instanceBlueprint_ = sceneMapping.findInstanceBlueprint(name);
  // hacky naming convention for ReplicaCAD baked scenes which just contain a stage
  std::string columnGridFilepath = "../data/columngrids/" 
    + name + "_stage_only.columngrid";
  fixedObj.columnGrid_.load(columnGridFilepath);

  set.fixedObjects_.emplace_back(std::move(fixedObj));
}

// radius of sphere at origin that bounds this AABB
float getOriginBoundingSphereRadiusSquaredForAABB(const Magnum::Range3D& aabb) {
  auto absMin = Mn::Math::abs(aabb.min());
  Mn::Vector3 maxCorner = Mn::Math::max(absMin, aabb.max());
  return maxCorner.dot();
}

void addFreeObject(EpisodeSet& set, const std::string& name, const Magnum::Range3D& aabb, const BpsSceneMapping& sceneMapping) {

  FreeObject freeObj;
  freeObj.name_ = name;
  freeObj.instanceBlueprint_ = sceneMapping.findInstanceBlueprint(name);
  freeObj.aabb_ = aabb;
 //  freeObj.boundingSphereRadiusSq_ = getOriginBoundingSphereRadiusSquaredForAABB(aabb);

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

void addEpisode(EpisodeSet& set, int stageFixedObjectIndex, core::Random& random) {
  Episode episode;
  episode.stageFixedObjIndex = stageFixedObjectIndex;
  episode.firstFreeObjectSpawnIndex_ = set.freeObjectSpawns_.size();

  // keep object count close to 28 (from Hab 2.0 benchmark), but include variation
  episode.numFreeObjectSpawns_ = random.uniform_int(28, 33);
  set.maxFreeObjects_ = Mn::Math::max(set.maxFreeObjects_, episode.numFreeObjectSpawns_);

  //Mn::Range3D spawnRange({-1.f, 0.05f, 0.f}, {3.f, 0.05, 3.f});
  Mn::Range3D spawnRange({-1.f, 0.05f, -0.5f}, {4.f, 1.5f, 3.f});
  const auto robotStartPos = Mn::Vector3(1.61, 0.f, 0.98);
  const auto pad = Mn::Vector3(0.9f, 2.f, 0.9);
  const auto exclusionRange = Mn::Range3D(robotStartPos - pad, robotStartPos + pad);

  for (int i = 0; i < episode.numFreeObjectSpawns_; i++) {

    FreeObjectSpawn spawn;
    spawn.freeObjIndex_ = random.uniform_int(0, set.freeObjects_.size());
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

    spawn.startPos_ = randomPos;

    set.freeObjectSpawns_.emplace_back(std::move(spawn));
  }

  set.episodes_.emplace_back(std::move(episode));
}

}

EpisodeSet generateBenchmarkEpisodeSet(int numEpisodes, const BpsSceneMapping& sceneMapping) {

  core::Random random(/*seed*/0);

  EpisodeSet set;

  set.maxFreeObjects_ = 0;
  
  std::vector<std::string> replicaCadBakedStages = {
    "Baked_sc0_staging_00",
    "Baked_sc0_staging_01"
  };

  for (const auto& stageName : replicaCadBakedStages) {
    addStageFixedObject(set, stageName, sceneMapping);
  }

  addFreeObject(set, "024_bowl", Mn::Range3D({-0.09557099640369415, -0.12427099794149399, -0.0005300004268065095}, {0.06589200347661972, 0.03689299523830414, 0.05447899550199509 }), sceneMapping);
  addFreeObject(set, "003_cracker_box", Mn::Range3D({-0.048785001039505005, -0.09616000950336456, -0.0032430035062134266}, {0.02301499992609024, 0.06787599623203278, 0.21019400656223297 }), sceneMapping);
  addFreeObject(set, "010_potted_meat_can", Mn::Range3D({-0.08382699638605118, -0.05660400539636612, -0.0031880023889243603}, {0.018257999792695045, 0.0034989966079592705, 0.08035500347614288 }), sceneMapping);
  addFreeObject(set, "002_master_chef_can", Mn::Range3D({-0.06831300258636475, -0.06094900891184807, -0.00018700220971368253}, {0.03421600162982941, 0.04142799228429794, 0.13999000191688538 }), sceneMapping);
  addFreeObject(set, "004_sugar_box", Mn::Range3D({-0.032214999198913574, -0.06379300355911255, 3.0998555303085595e-05 }, {0.017280999571084976, 0.030368993058800697, 0.1760459989309311 }), sceneMapping);
  addFreeObject(set, "005_tomato_soup_can", Mn::Range3D({-0.0431240014731884, 0.05014599487185478, 7.90045305620879e-05}, {0.024786999449133873, 0.11788899451494217, 0.10193400084972382 }), sceneMapping);
  addFreeObject(set, "009_gelatin_box", Mn::Range3D({-0.06747700273990631, -0.05879899859428406, -0.0005450012977235019 }, {0.02192699909210205, 0.042309001088142395, 0.02952899970114231 }), sceneMapping);
  addFreeObject(set, "008_pudding_box", Mn::Range3D({-0.0684640035033226, -0.04525500163435936, -0.0004969995934516191}, {0.069473996758461, 0.08350100368261337, 0.038391999900341034}), sceneMapping);
  addFreeObject(set, "007_tuna_fish_can", Mn::Range3D({-0.06882800161838531, -0.06490200012922287, -0.003218000056222081}, {0.01673099957406521, 0.0206379983574152, 0.030319999903440475}), sceneMapping);

  // distribute stages across episodes
  for (int i = 0; i < numEpisodes; i++) {
    int stageIndex = i * set.fixedObjects_.size() / numEpisodes;
    addEpisode(set, stageIndex, random);
  }

  return set;
}


}  // namespace batched_sim
}  // namespace esp
