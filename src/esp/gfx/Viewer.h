// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/Camera.h>

#include "esp/agent/Agent.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/nav/PathFinder.h"
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

  assets::ResourceManager resourceManager_;
  scene::SceneManager sceneManager_;
  std::vector<int> sceneID_;
  scene::SceneNode* agentBodyNode_ = nullptr;
  scene::SceneNode* cameraNode_ = nullptr;
  RenderCamera* renderCamera_ = nullptr;
  nav::PathFinder::ptr pathfinder_;
  scene::ObjectControls controls_;
  Magnum::Vector3 previousPosition_;

  bool computeActionPath_;
  vec3f goalPos_;
  quatf goalHeading_;
};

}  // namespace gfx
}  // namespace esp
