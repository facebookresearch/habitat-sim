// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Optional.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Frustum.h>
#include <Magnum/Math/Intersection.h>
#include <Magnum/Math/Range.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::scene::SceneManager;

TEST(CullingTest, computeAbsoluteAABB) {
  // must create a GL context which will be used in the resource manager
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  // must declare these in this order due to avoid deallocation errors
  ResourceManager resourceManager;
  SceneManager sceneManager;

  std::string sceneFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/5boxes.glb");

  int sceneID = sceneManager.initSceneGraph();
  auto& sceneGraph = sceneManager.getSceneGraph(sceneID);
  esp::scene::SceneNode& sceneRootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();
  const esp::assets::AssetInfo info =
      esp::assets::AssetInfo::fromPath(sceneFile);
  bool loadSuccess =
      resourceManager.loadScene(info, &sceneRootNode, &drawables);
  CHECK_EQ(loadSuccess, true);

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
  std::vector<Mn::Range3D> aabbsGroundTruth;
  // Box 0: root (parent: null), object "a", centered at origin
  aabbsGroundTruth.emplace_back(Mn::Vector3{-1.0, -1.0, -1.0},
                                Mn::Vector3{1.0, 1.0, 1.0});
  // Box 1: (parent, Box 0), object "a", relative translation (0.0, -4.0, 0.0)
  aabbsGroundTruth.emplace_back(Mn::Vector3{-1.0, -5.0, -1.0},
                                Mn::Vector3{1.0, -3.0, 1.0});
  // Box 2: (parent, Box 1), object "a", relative translation (0.0, 0.0, 4.0)
  aabbsGroundTruth.emplace_back(Mn::Vector3{-1.0, -5.0, 3.0},
                                Mn::Vector3{1.0, -3.0, 5.0});
  // Box 3: (parent, Box 0), object "a", relative translation (-4.0, 0.0, 4.0),
  // relative rotation pi/4 (ccw) around local z-axis of Box 3
  aabbsGroundTruth.emplace_back(
      Mn::Vector3{-4.0f - sqrt(2.0f), -sqrt(2.0f), 3.0},
      Mn::Vector3{-4.0f + sqrt(2.0f), sqrt(2.0f), 5.0});
  // Box 4: (parent, Box 3), object "a", relative translation (8.0, 0.0, 0.0),
  // relative rotation pi/4 (ccw) around local z-axis of Box 4
  aabbsGroundTruth.emplace_back(Mn::Vector3{3.0, -1.0, 3.0},
                                Mn::Vector3{5.0, 1.0, 5.0});

  // compare against the ground truth
  CHECK_EQ(aabbs.size(), aabbsGroundTruth.size());
  const float epsilon = 1e-6;
  for (unsigned int iBox = 0; iBox < aabbsGroundTruth.size(); ++iBox) {
    CHECK_LE(std::abs((aabbs[iBox].min() - aabbsGroundTruth[iBox].min()).dot()),
             epsilon);
    CHECK_LE(std::abs((aabbs[iBox].max() - aabbsGroundTruth[iBox].max()).dot()),
             epsilon);
  }
}

TEST(CullingTest, frustumCulling) {
  // must create a GL context which will be used in the resource manager
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  // must declare these in this order due to avoid deallocation errors
  ResourceManager resourceManager;
  SceneManager sceneManager;

  std::string sceneFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/5boxes.glb");

  // load the scene
  int sceneID = sceneManager.initSceneGraph();
  auto& sceneGraph = sceneManager.getSceneGraph(sceneID);
  esp::scene::SceneNode& sceneRootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();
  const esp::assets::AssetInfo info =
      esp::assets::AssetInfo::fromPath(sceneFile);
  bool loadSuccess =
      resourceManager.loadScene(info, &sceneRootNode, &drawables);
  CHECK_EQ(loadSuccess, true);

  // set the camera
  esp::gfx::RenderCamera& renderCamera = sceneGraph.getDefaultRenderCamera();

  // The camera to be set:
  // pos: {7.3589f, -6.9258f,4.9583f}
  // rotation: 77.4 deg, around {0.773, 0.334, 0.539}
  // fov = 39.6 deg
  // resolution: 800 x 600
  // clip planes (near: 0.1m, far: 100m)
  // with such a camera, the box 3 should be invisible, box 0, 1, 2, 4 should be
  // visible.

  // NOTE: the following test reults have been visually verified in utility
  // viewer
  renderCamera.setProjectionMatrix(800,     // width
                                   600,     // height
                                   0.01f,   // znear
                                   100.0f,  // zfar
                                   39.6f);  // hfov

  esp::scene::SceneNode agentNode = sceneGraph.getRootNode().createChild();
  esp::scene::SceneNode cameraNode = agentNode.createChild();
  cameraNode.translate({7.3589f, -6.9258f, 4.9583f});
  const Mn::Vector3 axis{0.773, 0.334, 0.539};
  cameraNode.rotate(Mn::Math::Deg<float>(77.4f), axis.normalized());
  renderCamera.node().setTransformation(cameraNode.absoluteTransformation());

  // ground truth
  const unsigned int visibleGroundTruth = 4;
  const unsigned int invisibleBoxGroundTruth = 3;

  // frustum culling test
  unsigned int visible = 0;
  Mn::Matrix4 vp =
      renderCamera.projectionMatrix() * renderCamera.cameraMatrix();
  const Mn::Math::Frustum<float> frustum =
      Mn::Math::Frustum<float>::fromMatrix(vp);

  for (unsigned int iDrawable = 0; iDrawable < drawables.size(); ++iDrawable) {
    Cr::Containers::Optional<Mn::Range3D> aabb =
        dynamic_cast<esp::scene::SceneNode&>(drawables[iDrawable].object())
            .getAbsoluteAABB();
    if (aabb) {
      if (Mn::Math::Intersection::rangeFrustum(*aabb, frustum)) {
        visible++;
      } else {
        CHECK_EQ(iDrawable, invisibleBoxGroundTruth);
      }
    }
  }
  CHECK_EQ(visibleGroundTruth, visible);
}
