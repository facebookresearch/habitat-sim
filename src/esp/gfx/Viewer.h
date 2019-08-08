// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/configure.h>
#ifdef MAGNUM_TARGET_WEBGL
#include <Magnum/Platform/Sdl2Application.h>
#else
#include <Magnum/Platform/GlfwApplication.h>
#endif
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/Timeline.h>

#include "esp/agent/Agent.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/RigidObject.h"
//#include "esp/physics/bullet/BulletPhysicsManager.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"

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

  // Interactive functions
  void addObject(std::string configFile);
  void pokeLastObject();
  void pushLastObject();
  void removeLastObject();
  void invertGravity();
  Magnum::Vector3 randomDirection();
  void wiggleLastObject();

  assets::ResourceManager resourceManager_;
  std::shared_ptr<physics::PhysicsManager> physicsManager_;
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
  bool vangoth_mesh = false;
  bool castle_mesh = true;

  bool do_profile_ = false;
  int frame_limit_ = 1000;
  int frame_curr_ = 0;

  int numObjects_ = 1;
  std::vector<int> objectIDs;

  vec3f goalPos_;
  quatf goalHeading_;

  Magnum::Timeline timeline_;
};

}  // namespace gfx
}  // namespace esp
