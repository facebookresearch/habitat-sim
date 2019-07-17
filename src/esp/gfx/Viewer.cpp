// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Viewer.h"

#include <Corrade/Utility/Arguments.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <sophus/so3.hpp>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include "Drawable.h"
#include "esp/io/io.h"
#include "esp/physics/ObjectType.h"


using namespace Magnum;
using namespace Math::Literals;
using namespace Corrade;

constexpr float moveSensitivity = 0.1f;
constexpr float lookSensitivity = 11.25f;
constexpr float cameraHeight = 1.5f;

namespace esp {
namespace gfx {

Viewer::Viewer(const Arguments& arguments)
    : Platform::Application{arguments,
                            Configuration{}.setTitle("Viewer").setWindowFlags(
                                Configuration::WindowFlag::Resizable),
                            GLConfiguration{}.setColorBufferSize(
                                Vector4i(8, 8, 8, 8))},
      pathfinder_(nav::PathFinder::create()),
      controls_(),
      previousPosition_(),
      physicsManager_(resourceManager_) {
  Utility::Arguments args;
  args.addArgument("file")
      .setHelp("file", "file to load")
      .addOption("obj", "./data/objects/chefcan.glb")
      .setHelp("obj", "obj file to load")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp("Displays a 3D scene file provided on command line")
      .addBooleanOption("enable-physics")
      .parse(arguments.argc, arguments.argv);

  const auto viewportSize = GL::defaultFramebuffer.viewport().size();
  enablePhysics_ = args.isSet("enable-physics");

  // Setup renderer and shader defaults
  GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
  GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

  int sceneID = sceneManager_.initSceneGraph();
  sceneID_.push_back(sceneID);
  sceneGraph = &sceneManager_.getSceneGraph(sceneID);
  rootNode = &sceneGraph->getRootNode();
  navSceneNode_ = &rootNode->createChild();

  if (enablePhysics_) {
    // ======= Init timestep, physics starts =======
    physicsManager_.initPhysics(navSceneNode_, do_profile_);
  }

  auto& drawables = sceneGraph->getDrawables();
  const std::string& file = args.value("file");
  const assets::AssetInfo info = assets::AssetInfo::fromPath(file);
  LOG(INFO) << "Nav scene node (before) " << navSceneNode_;
  if (!resourceManager_.loadScene(info, navSceneNode_, &drawables, 
      &physicsManager_, enablePhysics_)) {
    LOG(ERROR) << "cannot load " << file;
    std::exit(0);
  }

  // Set up physics
  LOG(INFO) << "Nav scene node (done) " << navSceneNode_;

  // Set up camera
  renderCamera_ = &sceneGraph->getDefaultRenderCamera();
  agentBodyNode_ = &rootNode->createChild();
  cameraNode_ = &agentBodyNode_->createChild();

  float hfov = 90.0f;
  int width = viewportSize[0];
  int height = viewportSize[1];
  const float aspectRatio = static_cast<float>(width) / height;
  float znear = 0.01f;
  float zfar = 1000.0f;
  renderCamera_->setProjectionMatrix(width, height, znear, zfar, hfov);

  // Load navmesh if available
  const std::string navmeshFilename = io::changeExtension(file, ".navmesh");
  if (io::exists(navmeshFilename)) {
    LOG(INFO) << "Loading navmesh from " << navmeshFilename;
    pathfinder_->loadNavMesh(navmeshFilename);
    LOG(INFO) << "Loaded.";
  }

  // Messing around with agent location and finding object initial position
  LOG(INFO) << "Agent position " << agentBodyNode_->getAbsolutePosition();
  LOG(INFO) << "Camera position " << cameraNode_->getAbsolutePosition();
  LOG(INFO) << "Scene position" << navSceneNode_->getAbsolutePosition();

  if (surreal_mesh) {
    Vector3 agent_pos = Vector3(-2.93701f, -3.53019f, 3.68798f);
    agentBodyNode_->setTranslation(Eigen::Map<vec3f>(agent_pos.data()));
    agentBodyNode_->rotate(3.14f, vec3f(0, 1, 0));
  } else if (replica_mesh) {
    Vector3 agent_pos = Vector3(0.0707106, 1.0f, -0.7898f);
    agentBodyNode_->setTranslation(Eigen::Map<vec3f>(agent_pos.data()));
    agentBodyNode_->rotate(3.14f, vec3f(0, 1, 0));
  } else if (vangoth_mesh) {
    //agentBodyNode_->rotate(3.14f * 1.1, vec3f(0, 1, 0));
    // Vector3 agent_pos = Vector3(0.0f, 0.0f, 0.0f);
    Vector3 agent_pos = Vector3(2.38, 1.4f, 1.74);
    agentBodyNode_->setTranslation(Eigen::Map<vec3f>(agent_pos.data()));
  } else if (castle_mesh) {
    agentBodyNode_->rotate(3.14f * 1.1, vec3f(0, 1, 0));
    // Vector3 agent_pos = Vector3(0.0f, 0.0f, 0.0f);
    agentBodyNode_->translate(vec3f(-0.8, 1.4f, 11.0f));
    // agentBodyNode_->setTranslation(Eigen::Map<vec3f>(agent_pos.data()));
  }

  //std::string object_file(args.value("obj"));

  renderCamera_->setTransformation(cameraNode_->getAbsoluteTransformation());

  //! Load cheezit object
  std::string object_file("./data/objects/cheezit.glb");
  std::string object_config("./data/objects/cheezit.config");
  //! Do not draw or instantiate
  cheezitID = resourceManager_.loadObject(object_config, nullptr, nullptr);
  for (int o = 0; o < numObjects_; o++) {
    addObject(cheezitID);
  }
  LOG(INFO) << "Viewer initialization is done. ";
}  // namespace gfx


void Viewer::addObject(int resourceObjectID) {
  if (resourceObjectID < 0) {
    std::exit(0);
  }
  if (enablePhysics_) {
    // TODO (JH) swap interval
    // setSwapInterval(1);   // Loop at 60 Hz max
  }

  Magnum::Matrix4 T = agentBodyNode_
      ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 new_pos = T.transformPoint({0.0f, 0.0f, 0.0f});
  if (castle_mesh) {
    // new_pos = T.transformPoint({0.1f, 1.0f, -3.0f});
    new_pos = T.transformPoint({0.1f, 1.0f, -3.0f});
  } else if (vangoth_mesh) {
    new_pos = T.transformPoint({0.1f, 0.0f, -1.0f});
  } else if (surreal_mesh) {
    new_pos = T.transformPoint({0.0f, 0.0f, -1.0f});
  }

  LOG(INFO) << "Camera position " << T.translation().x() << " "
            << T.translation().y() << " " << T.translation().z();
  LOG(INFO) << "Object new position " << new_pos.x() << " " << new_pos.y()
            << " " << new_pos.z();
  LOG(INFO) << "Camera transformation " << Eigen::Map<mat4f>(T.data());


  auto& drawables = sceneGraph->getDrawables();
  int physObjectID = physicsManager_.addObject(
      resourceObjectID, navSceneNode_, 
      physics::PhysicalObjectType::DYNAMIC, &drawables);
  physicsManager_.setTranslation(physObjectID, Eigen::Map<vec3f>(new_pos.data()));
  lastObjectID += 1;
}


void Viewer::pokeLastObject() {
  Magnum::Matrix4 T = agentBodyNode_
      ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 impulse = T.transformPoint({0.0f, 0.0f, -3.0f});
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  LOG(INFO) << "Poking object " << lastObjectID;
  physicsManager_.applyImpulse(lastObjectID, impulse, rel_pos);
}

void Viewer::pushLastObject() {
  Magnum::Matrix4 T = agentBodyNode_
      ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 force = T.transformPoint({0.0f, 0.0f, -40.0f});
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  LOG(INFO) << "Pushing object " << lastObjectID;
  physicsManager_.applyForce(lastObjectID, force, rel_pos);
}

Vector3 positionOnSphere(Magnum::SceneGraph::Camera3D& camera,
                         const Vector2i& position) {
  const Vector2 positionNormalized =
      Vector2{position} / Vector2{camera.viewport()} - Vector2{0.5f};
  const Float length = positionNormalized.length();
  const Vector3 result(length > 1.0f
                     ? Vector3(positionNormalized, 0.0f)
                     : Vector3(positionNormalized, 1.0f - length));
  return (result * Vector3::yScale(-1.0f)).normalized();
}

void Viewer::drawEvent() {
  GL::defaultFramebuffer.clear(GL::FramebufferClear::Color |
                               GL::FramebufferClear::Depth);
  if (sceneID_.size() <= 0)
    return;

  frame_curr_ += 1;
  physicsManager_.stepPhysics();

  if (do_profile_ && frame_curr_ > frame_limit_) {
    std::exit(0);
  }

  int DEFAULT_SCENE = 0;
  int sceneID = sceneID_[DEFAULT_SCENE];
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  renderCamera_->getMagnumCamera().draw(sceneGraph.getDrawables());
  // Draw debug forces
  //renderCamera_->getMagnumCamera().draw(physicsManager_.getDrawables());
  swapBuffers();
  physicsManager_.nextFrame();
  redraw();
}

void Viewer::viewportEvent(ViewportEvent& event) {
  GL::defaultFramebuffer.setViewport({{}, framebufferSize()});
  renderCamera_->getMagnumCamera().setViewport(event.windowSize());
}

void Viewer::mousePressEvent(MouseEvent& event) {
  if (event.button() == MouseEvent::Button::Left)
    previousPosition_ =
        positionOnSphere(renderCamera_->getMagnumCamera(), event.position());

  event.setAccepted();
}

void Viewer::mouseReleaseEvent(MouseEvent& event) {
  if (event.button() == MouseEvent::Button::Left)
    previousPosition_ = Vector3();

  event.setAccepted();
}

void Viewer::mouseScrollEvent(MouseScrollEvent& event) {
  if (!event.offset().y()) {
    return;
  }

  /* Distance to origin */
  const float distance = renderCamera_->getTransformation().col(3).z();

  /* Move 15% of the distance back or forward */
  renderCamera_->translateLocal(vec3f(
      0, 0, distance * (1.0f - (event.offset().y() > 0 ? 1 / 0.85f : 0.85f))));

  event.setAccepted();
}

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  if (!(event.buttons() & MouseMoveEvent::Button::Left)) {
    return;
  }

  const Vector3 currentPosition =
      positionOnSphere(renderCamera_->getMagnumCamera(), event.position());
  const Vector3 axis = Math::cross(previousPosition_, currentPosition);

  if (previousPosition_.length() < 0.001f || axis.length() < 0.001f) {
    return;
  }
  const auto angle = Math::angle(previousPosition_, currentPosition);
  renderCamera_->getSceneNode()->MagnumObject::rotate(-angle,
                                                      axis.normalized());
  previousPosition_ = currentPosition;

  event.setAccepted();
}

void Viewer::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  switch (key) {
    case KeyEvent::Key::Esc:
      std::exit(0);
      break;
    case KeyEvent::Key::Left:
      controls_(*agentBodyNode_, "lookLeft", lookSensitivity);
      break;
    case KeyEvent::Key::Right:
      controls_(*agentBodyNode_, "lookRight", lookSensitivity);
      break;
    case KeyEvent::Key::Up:
      controls_(*cameraNode_, "lookUp", lookSensitivity, false);
      break;
    case KeyEvent::Key::Down:
      controls_(*cameraNode_, "lookDown", lookSensitivity, false);
      break;
    case KeyEvent::Key::Nine: {
      const vec3f position = pathfinder_->getRandomNavigablePoint();
      agentBodyNode_->setTranslation(position);
    } break;
    case KeyEvent::Key::A:
      controls_(*agentBodyNode_, "moveLeft", moveSensitivity);
      LOG(INFO) << "Agent position " << agentBodyNode_->getAbsolutePosition();
      break;
    case KeyEvent::Key::D:
      controls_(*agentBodyNode_, "moveRight", moveSensitivity);
      LOG(INFO) << "Agent position " << agentBodyNode_->getAbsolutePosition();
      break;
    case KeyEvent::Key::S:
      controls_(*agentBodyNode_, "moveBackward", moveSensitivity);
      LOG(INFO) << "Agent position " << agentBodyNode_->getAbsolutePosition();
      break;
    case KeyEvent::Key::W:
      controls_(*agentBodyNode_, "moveForward", moveSensitivity);
      LOG(INFO) << "Agent position " << agentBodyNode_->getAbsolutePosition();
      break;
    case KeyEvent::Key::X:
      controls_(*agentBodyNode_, "moveDown", moveSensitivity, false);
      break;
    case KeyEvent::Key::Z:
      controls_(*agentBodyNode_, "moveUp", moveSensitivity, false);
      break;
    case KeyEvent::Key::O:
      addObject(cheezitID);
      break;
    case KeyEvent::Key::P:
      pokeLastObject();
      break;
    case KeyEvent::Key::F:
      pushLastObject();
      break;
    default:
      break;
  }
  renderCamera_->setTransformation(cameraNode_->getAbsoluteTransformation());
  event.setAccepted();
}

}  // namespace gfx
}  // namespace esp

