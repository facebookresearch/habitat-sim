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
  for (size_t iDrawable = 0; iDrawable < drawables.size(); ++iDrawable) {
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
  aabbsGroundTruth.emplace_back(Mn::Vector3{-4.0 - sqrt(2.0), -sqrt(2.0), 3.0},
                                Mn::Vector3{-4.0 + sqrt(2.0), sqrt(2.0), 5.0});
  // Box 4: (parent, Box 3), object "a", relative translation (8.0, 0.0, 0.0),
  // relative rotation pi/4 (ccw) around local z-axis of Box 4
  aabbsGroundTruth.emplace_back(Mn::Vector3{3.0, -1.0, 3.0},
                                Mn::Vector3{5.0, 1.0, 5.0});

  // compare against the ground truth
  CHECK_EQ(aabbs.size(), aabbsGroundTruth.size());
  const float epsilon = 1e-6;
  for (size_t iBox = 0; iBox < aabbsGroundTruth.size(); ++iBox) {
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
  // inside the 5boxes.glb, there is a default camera created in the scene
  // pos: {7.3589f, -6.9258f,4.9583f}
  // rotation: 77.4 deg, around {0.773, 0.334, 0.539}
  // focal length = 50mm (it means the hfov = 26.3 deg, vfov = 17.7 deg
  // resolution: 1920 x 1080
  // clip planes (near: 0.1m, far: 100m)
  // with such a camera, the box 4 should be invisible, box 0 to 3 should be
  // visible.
  renderCamera.setProjectionMatrix(1980,    // width
                                   1080,    // height
                                   0.01f,   // znear
                                   100.0f,  // zfar
                                   90.3);   // hfov

  esp::scene::SceneNode cameraNode = sceneGraph.getRootNode();
  cameraNode.translateLocal({7.3589f, -6.9258f, 4.9583f});
  // cameraNode.translateLocal({0.0f, -8.0f, 0.0f});
  auto pos = cameraNode.absoluteTranslation();
  printf("pos = %lf, %lf, %lf\n", pos.x(), pos.y(), pos.z());
  const Mn::Vector3 axis{0.773, 0.334, 0.539};
  const float pi = 3.1415926535;
  cameraNode.rotate(Mn::Math::Deg<float>(77.4f), axis.normalized());
  renderCamera.node().setTransformation(cameraNode.absoluteTransformation());

  // frustum culling test
  size_t visible = 0;
  const Mn::Math::Frustum<float> frustum = Mn::Math::Frustum<float>::fromMatrix(
      renderCamera.projectionMatrix() * renderCamera.cameraMatrix());
  for (size_t iDrawable = 0; iDrawable < drawables.size(); ++iDrawable) {
    Cr::Containers::Optional<Mn::Range3D> aabb =
        dynamic_cast<esp::scene::SceneNode&>(drawables[iDrawable].object())
            .getAbsoluteAABB();
    if (aabb) {
      if (Mn::Math::Intersection::rangeFrustum(*aabb, frustum)) {
        printf("Box %u is in.\n", iDrawable);
        visible++;
      } else {
        printf("Box %u is NOT in.\n", iDrawable);
      }
    }
  }
  printf("visible = %u\n", visible);
}
