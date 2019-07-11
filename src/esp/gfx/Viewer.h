// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/Camera.h>

#include "esp/agent/Agent.h"
#include "esp/assets/ResourceManager.h"
#include "esp/assets/PhysicsManager.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"
#include "esp/physics/BulletObject.h"

// forward declaration
namespace Corrade {
namespace Utility {
class Arguments;
}
}  // namespace Corrade

namespace esp {
namespace gfx {

class Viewer : public Magnum::Platform::Application {
 public:
  explicit Viewer(const Arguments& arguments);

 private:
  void drawEvent() override;
  void viewportEvent(ViewportEvent& event) override;
  void mousePressEvent(MouseEvent& event) override;
  void mouseReleaseEvent(MouseEvent& event) override;
  void mouseMoveEvent(MouseMoveEvent& event) override;
  void mouseScrollEvent(MouseScrollEvent& event) override;
  void keyPressEvent(KeyEvent& event) override;

  void addObject();
  void pokeLastObject();
  void pushLastObject();

  assets::ResourceManager resourceManager_;
  assets::PhysicsManager physicsManager_;
  scene::SceneManager sceneManager_;
  std::vector<int> sceneID_;
  scene::SceneNode* agentBodyNode_ = nullptr;
  scene::SceneNode* cameraNode_ = nullptr;
  
  scene::SceneNode* navSceneNode_ = nullptr;

  scene::SceneGraph* sceneGraph;
  scene::SceneNode* rootNode;

  RenderCamera* renderCamera_ = nullptr;
  nav::PathFinder::ptr pathfinder_;
  scene::ObjectControls controls_;
  Magnum::Vector3 previousPosition_;

  bool computeActionPath_;
  bool enablePhysics_;
  bool surreal_mesh = false;
  bool replica_mesh = false;
  bool vangoth_mesh = true;
  bool castle_mesh = false;

  bool do_profile_ = false;
  int  frame_limit_ = 1000;
  int  frame_curr_ = 0;

  int numObjects_ = 1;
  int lastObjectID = -1;
  vec3f goalPos_;
  quatf goalHeading_;
};

}  // namespace gfx
}  // namespace esp
