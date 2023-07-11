// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
//
#include <Corrade/Containers/Optional.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/GL/SampleQuery.h>
#include <Magnum/Math/Frustum.h>
#include <Magnum/Math/Intersection.h>
#include <Magnum/Math/Range.h>
#include <string>

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/gfx_batch/DepthUnprojection.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::scene::SceneManager;
using Magnum::Math::Literals::operator""_degf;

// on GCC and Clang, the following namespace causes useful warnings to be
// printed when you have accidentally unused variables or functions in the test
namespace {
struct CullingTest : Cr::TestSuite::Tester {
  explicit CullingTest();

  // init, returns ref to scene graph
  int setupTests();

  // tests
  void computeAbsoluteAABB();
  void frustumCulling();

 protected:
  esp::logging::LoggingContext loggingContext_;
  esp::gfx::WindowlessContext::uptr context_ = nullptr;
  std::unique_ptr<ResourceManager> resourceManager_ = nullptr;
  SceneManager::uptr sceneManager_ = nullptr;
};

CullingTest::CullingTest() {
  // clang-format off
  addTests({&CullingTest::computeAbsoluteAABB,
            &CullingTest::frustumCulling});
  // clang-format on
}

int CullingTest::setupTests() {
  // set up a default simulation config to initialize MM
  auto cfg = esp::sim::SimulatorConfiguration{};
  // setting values for stage load
  cfg.loadSemanticMesh = false;
  cfg.forceSeparateSemanticSceneGraph = false;
  auto MM = MetadataMediator::create(cfg);
  // must declare these in this order due to avoid deallocation errors
  if (!resourceManager_) {
    resourceManager_ = std::make_unique<ResourceManager>(MM);
  }
  if (!sceneManager_) {
    sceneManager_ = SceneManager::create_unique();
  }
  if (!context_) {
    context_ = esp::gfx::WindowlessContext::create_unique(0);
  }
  auto stageAttributesMgr = MM->getStageAttributesManager();
  std::string stageFile =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/5boxes.glb");
  // create scene attributes file
  auto stageAttributes = stageAttributesMgr->createObject(stageFile, true);
  int sceneID = sceneManager_->initSceneGraph();

  std::vector<int> tempIDs{sceneID, esp::ID_UNDEFINED};
  bool result = resourceManager_->loadStage(stageAttributes, nullptr, nullptr,
                                            sceneManager_.get(), tempIDs);
  CORRADE_VERIFY(result);
  return sceneID;
}

void CullingTest::computeAbsoluteAABB() {
  int sceneID = setupTests();
  auto& sceneGraph = sceneManager_->getSceneGraph(sceneID);
  auto& drawables = sceneGraph.getDrawables();

  std::vector<Mn::Range3D> aabbs;
  for (unsigned int iDrawable = 0; iDrawable < drawables.size(); ++iDrawable) {
    Cr::Containers::Optional<Mn::Range3D> aabb =
        dynamic_cast<esp::scene::SceneNode&>(drawables[iDrawable].object())
            .getAbsoluteAABB();
    if (aabb) {
      aabbs.emplace_back(*aabb);
    }
  }

  /* ground truth
   *
   * Objects: (TODO: add more objects to the test, e.g., sphere, cylinder)
   *  a) a cube, with edge length 2.0
   *
   */
  std::vector<Mn::Range3D> aabbsGroundTruth{
      // Box 0: root (parent: null), object "a", centered at origin
      {{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}},

      // Box 1: (parent, Box 0), object "a", relative translation (0.0, -4.0,
      // 0.0)
      {{-1.0f, -5.0f, -1.0f}, {1.0f, -3.0f, 1.0f}},

      // Box 2: (parent, Box 1), object "a", relative translation (0.0,
      // 0.0, 4.0)
      {{-1.0f, -5.0f, 3.0f}, {1.0f, -3.0f, 5.0f}},

      // Box 3: (parent, Box 0), object "a", relative translation (-4.0,
      // 0.0, 4.0),
      // relative rotation pi/4 (ccw) around local z-axis of Box 3
      {{-4.0f - Mn::Constants::sqrt2(), -Mn::Constants::sqrt2(), 3.0f},
       {-4.0f + Mn::Constants::sqrt2(), Mn::Constants::sqrt2(), 5.0f}},

      // Box 4: (parent, Box 3), object "a", relative translation (8.0, 0.0,
      // 0.0),
      // relative rotation pi/4 (ccw) around local z-axis of Box 4
      {{3.0f, -1.0f, 3.0f}, {5.0f, 1.0f, 5.0f}}};

  // compare against the ground truth
  CORRADE_COMPARE(aabbs.size(), aabbsGroundTruth.size());
  const float eps = 1e-6;
  for (unsigned int iBox = 0; iBox < aabbsGroundTruth.size(); ++iBox) {
    CORRADE_ITERATION(iBox);
    CORRADE_COMPARE_WITH(aabbs[iBox].min(), aabbsGroundTruth[iBox].min(),
                         Cr::TestSuite::Compare::around(Mn::Vector3{eps}));
    CORRADE_COMPARE_WITH(aabbs[iBox].max(), aabbsGroundTruth[iBox].max(),
                         Cr::TestSuite::Compare::around(Mn::Vector3{eps}));
  }
}

void CullingTest::frustumCulling() {
  int sceneID = setupTests();
  auto& sceneGraph = sceneManager_->getSceneGraph(sceneID);
  // esp::scene::SceneNode& sceneRootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();

  // set the camera
  esp::scene::SceneNode agentNode = sceneGraph.getRootNode().createChild();
  esp::scene::SceneNode cameraNode = agentNode.createChild();
  esp::gfx::RenderCamera& renderCamera =
      *(new esp::gfx::RenderCamera(cameraNode));

  // The camera to be set:
  // pos: {7.3589f, -6.9258f,4.9583f}
  // rotation: 77.4 deg, around {0.773, 0.334, 0.539}
  // fov = 39.6 deg
  // resolution: 800 x 600
  // clip planes (near: 0.1m, far: 100m)
  // with such a camera, the box 3 should be invisible, box 0, 1, 2, 4 should be
  // visible.

  // NOTE: the following test results have been visually verified in utility
  // viewer
  Mn::Vector2i frameBufferSize{800, 600};
  renderCamera.setProjectionMatrix(frameBufferSize.x(),  // width
                                   frameBufferSize.y(),  // height
                                   0.01f,                // znear
                                   100.0f,               // zfar
                                   39.6_degf);           // hfov

  cameraNode.translate({7.3589f, -6.9258f, 4.9583f});
  const Mn::Vector3 axis{0.773, 0.334, 0.539};
  cameraNode.rotate(Mn::Math::Deg<float>(77.4f), axis.normalized());
  renderCamera.node().setTransformation(cameraNode.absoluteTransformation());

  // collect all the drawables and their transformations
  std::vector<std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                        Mn::Matrix4>>
      drawableTransforms = renderCamera.drawableTransformations(drawables);

  // do the culling (to create the testing group)
  size_t numVisibles = renderCamera.cull(drawableTransforms);
  auto newEndIter = drawableTransforms.begin() + numVisibles;

  // create a render target
  Mn::Matrix4 projMtx = renderCamera.projectionMatrix();
  esp::gfx::RenderTarget::uptr target = esp::gfx::RenderTarget::create_unique(
      frameBufferSize, esp::gfx_batch::calculateDepthUnprojection(projMtx));

  // ============== Test 1 ==================
  // draw all the invisibles reported by cull()
  // check passed if 0 sample is returned (that means they are indeed
  // invisibles.)
  {
    // objects will contain all the invisible ones
    std::vector<std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                          Mn::Matrix4>>
        objects = renderCamera.drawableTransformations(drawables);

    // CAREFUL:
    // all the invisible ones are NOT stored at [newEndIter,
    // drawableTransforms.end()]. This is because std::remove_if will only move
    // elements, not swap elements
    objects.erase(
        std::remove_if(
            objects.begin(), objects.end(),
            [&](const std::pair<
                std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                Mn::Matrix4>& a) {
              for (std::vector<std::pair<
                       std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                       Mn::Matrix4>>::iterator iter =
                       drawableTransforms.begin();
                   iter != newEndIter; ++iter) {
                if (std::addressof(a.first.get()) ==
                    std::addressof(iter->first.get())) {
                  return true;  // it is visible, remove it
                }
              }
              return false;
            }),
        objects.end());

    target->renderEnter();
    Mn::GL::SampleQuery q{Mn::GL::SampleQuery::Target::AnySamplesPassed};
    q.begin();
    renderCamera.MagnumCamera::draw(objects);
    q.end();
    target->renderExit();

    CORRADE_VERIFY(!q.result<bool>());
  }

  // ============== Test 2 ==================
  // draw the visibles one by one.
  // check if each one is a genuine visible drawable
  unsigned int numVisibleObjectsGroundTruth = 0;
  auto renderOneDrawable =
      [&](const std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                          Mn::Matrix4>& a) {
        std::vector<std::pair<
            std::reference_wrapper<Mn::SceneGraph::Drawable3D>, Mn::Matrix4>>
            objects;
        objects.emplace_back(a);

        target->renderEnter();
        Mn::GL::SampleQuery q{Mn::GL::SampleQuery::Target::AnySamplesPassed};
        q.begin();
        renderCamera.MagnumCamera::draw(objects);
        q.end();
        target->renderExit();

        // check if it a genuine visible drawable
        CORRADE_VERIFY(q.result<bool>());

        if (q.result<bool>()) {
          ++numVisibleObjectsGroundTruth;
        }
      };
  for_each(drawableTransforms.begin(), newEndIter, renderOneDrawable);

  // ============== Test 3 ==================
  // draw using the RenderCamera overload draw()
  target->renderEnter();
  size_t numVisibleObjects = renderCamera.draw(
      drawables, {esp::gfx::RenderCamera::Flag::
                      FrustumCulling} /* enable frustum culling */);
  target->renderExit();
  CORRADE_COMPARE(numVisibleObjects, numVisibleObjectsGroundTruth);
}
}  // namespace

CORRADE_TEST_MAIN(CullingTest)
