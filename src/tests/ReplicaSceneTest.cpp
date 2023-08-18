// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>

#include <Corrade/Utility/Path.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include "configure.h"
#include "esp/scene/ReplicaSemanticScene.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/Simulator.h"

#include "esp/assets/GenericSemanticMeshData.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::GenericSemanticMeshData;

namespace {

const std::string replicaRoom0 =
    Cr::Utility::Path::join(SCENE_DATASETS, "replica_dataset/room_0/habitat");
const std::string replicaCAD =
    Cr::Utility::Path::join(SCENE_DATASETS, "replicaCAD");

struct ReplicaSceneTest : Cr::TestSuite::Tester {
  explicit ReplicaSceneTest();

  void testSemanticSceneOBB();

  void testSemanticSceneLoading();

  void testSemanticSceneDescriptorReplicaCAD();

  esp::logging::LoggingContext loggingContext_;
};

ReplicaSceneTest::ReplicaSceneTest() {
  addTests({&ReplicaSceneTest::testSemanticSceneOBB,
            &ReplicaSceneTest::testSemanticSceneLoading,

#ifdef ESP_BUILD_WITH_BULLET
            &ReplicaSceneTest::testSemanticSceneDescriptorReplicaCAD
#endif
  });
}

void ReplicaSceneTest::testSemanticSceneOBB() {
  if (!Cr::Utility::Path::exists(replicaRoom0)) {
    CORRADE_SKIP("Replica dataset not found at '" + replicaRoom0 +
                 "'\nSkipping test");
  }

  esp::scene::SemanticScene scene;
  CORRADE_VERIFY(esp::scene::SemanticScene::loadReplicaHouse(
      Cr::Utility::Path::join(replicaRoom0, "info_semantic.json"), scene));

#ifndef MAGNUM_BUILD_STATIC
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
#else
  // avoid using plugins that might depend on different library versions
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager{
      "nonexistent"};
#endif

  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer;
  CORRADE_INTERNAL_ASSERT(importer =
                              manager.loadAndInstantiate("StanfordImporter"));

  // load ply but do not split
  // dummy colormap
  std::vector<Magnum::Vector3ub> dummyColormap;
  const std::string semanticFilename =
      Cr::Utility::Path::join(replicaRoom0, "mesh_semantic.ply");
  /* Open the file. On error the importer already prints a diagnostic message,
     so no need to do that here. The importer implicitly converts per-face
     attributes to per-vertex, so nothing extra needs to be done. */
  Cr::Containers::Optional<Mn::Trade::MeshData> meshData;
  ESP_CHECK(
      (importer->openFile(semanticFilename) && (meshData = importer->mesh(0))),
      Cr::Utility::formatString("Error loading instance mesh data from file {}",
                                semanticFilename));

  static std::unique_ptr<GenericSemanticMeshData> semanticMesh =
      GenericSemanticMeshData::buildSemanticMeshData(
          *meshData, semanticFilename, dummyColormap, false);
  // verify first entry exists
  CORRADE_VERIFY(semanticMesh);

  const auto& vbo = semanticMesh->getVertexBufferObjectCPU();
  const auto& objectIds = semanticMesh->getObjectIdsBufferObjectCPU();
  const auto& ibo = semanticMesh->getIndexBufferObjectCPU();

  for (const auto& obj : scene.objects()) {
    if (obj == nullptr)
      continue;

    const auto& stringId = obj->id();
    const int id = std::stoi(stringId.substr(stringId.find_last_of('_') + 1));
    CORRADE_ITERATION(stringId);

    for (uint64_t fid = 0; fid < ibo.size(); fid += 6) {
      CORRADE_ITERATION(fid);
      if (objectIds[ibo[fid]] == id) {
        Mn::Vector3 quadCenter{};
        // Mesh was converted from quads to tris
        for (int i = 0; i < 6; ++i) {
          quadCenter += vbo[ibo[fid + i]] / 6;
        }

        CORRADE_VERIFY(
            obj->obb().contains(esp::vec3f{quadCenter.data()}, 5e-2));
      }
    }
  }
}  // ReplicaSceneTest::testSemanticSceneOBB()

void ReplicaSceneTest::testSemanticSceneLoading() {
  if (!Cr::Utility::Path::exists(replicaRoom0)) {
    CORRADE_SKIP("Replica dataset not found at '" + replicaRoom0 +
                 "'\nSkipping test");
  }

  esp::sim::SimulatorConfiguration cfg;
  cfg.activeSceneName =
      Cr::Utility::Path::join(replicaRoom0, "mesh_semantic.ply");

  esp::sim::Simulator sim{cfg};

  const auto& scene = sim.getSemanticScene();
  CORRADE_VERIFY(scene);
  CORRADE_COMPARE(scene->objects().size(), 94);

  const auto obj12 = scene->objects()[12];

  CORRADE_VERIFY(obj12);
  CORRADE_COMPARE(obj12->id(), "_12");
  // obj12's obb
  // Eigen Calc
  // center:[3.52103,-1.00543,-1.02705]
  // halfextents:[0.169882,0.160166,0.01264]
  // rotational quat coefficients:[-0.70592,0.0131598,0.0157815,0.707994]
  // Old Sophus calc:
  // {c:[3.52103,-1.00543,-1.02705],h:[0.169882,0.160166,0.01264],r:[-0.70592,0.0131598,0.0157815,0.707994]}

  CORRADE_VERIFY(
      obj12->obb().center().isApprox(esp::vec3f{3.52103, -1.00543, -1.02705}));
  CORRADE_VERIFY(obj12->obb().halfExtents().isApprox(
      esp::vec3f{0.169882, 0.160166, 0.01264}));
  CORRADE_VERIFY(obj12->obb().rotation().coeffs().isApprox(
      esp::vec4f{-0.70592, 0.0131598, 0.0157815, 0.707994}));

  CORRADE_VERIFY(obj12->category());
  CORRADE_COMPARE(obj12->category()->index(), 13);
  CORRADE_COMPARE(obj12->category()->name(), "book");
}

void ReplicaSceneTest::testSemanticSceneDescriptorReplicaCAD() {
  if (!Cr::Utility::Path::exists(replicaCAD)) {
    CORRADE_SKIP("ReplicaCAD dataset not found at '" + replicaCAD +
                 "'\nSkipping test");
  }
  esp::sim::SimulatorConfiguration cfg;
  cfg.sceneDatasetConfigFile = Cr::Utility::Path::join(
      replicaCAD, "replicaCAD.scene_dataset_config.json");

  cfg.activeSceneName = "apt_0.scene_instance";
  cfg.enablePhysics = true;

  esp::sim::Simulator sim{cfg};
  // semantic scene descriptor is specified in scene dataset config
  const auto& scene = sim.getSemanticScene();
  CORRADE_VERIFY(scene);
  // ReplicaCAD only populates categories - 101 specified + cat 0
  CORRADE_COMPARE(scene->categories().size(), 102);

  // verify name id and name
  CORRADE_COMPARE(scene->categories()[13]->index(), 13);
  CORRADE_COMPARE(scene->categories()[13]->name(), "book");
}
}  // namespace

CORRADE_TEST_MAIN(ReplicaSceneTest)
