// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <stdlib.h>

#include <Magnum/configure.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#ifdef MAGNUM_TARGET_WEBGL
#include <Magnum/Platform/EmscriptenApplication.h>
#else
#include <Magnum/Platform/GlfwApplication.h>
#endif
#include <Magnum/PixelFormat.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/Timeline.h>

#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Shaders/Shaders.h>

#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneNode.h"

#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>
#include <Magnum/DebugTools/Screenshot.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <sophus/so3.hpp>
#include "esp/core/Utility.h"
#include "esp/core/esp.h"
#include "esp/gfx/Drawable.h"
#include "esp/io/io.h"

#include "esp/scene/SceneConfiguration.h"
#include "esp/sim/Simulator.h"

#include "ObjectPickingHelper.h"
#include "esp/physics/configure.h"

constexpr float moveSensitivity = 0.1f;
constexpr float lookSensitivity = 11.25f;
constexpr float rgbSensorHeight = 1.5f;
// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

using namespace Mn::Math::Literals;

class SimViewer : public Mn::Platform::Application {
 public:
  explicit SimViewer(const Arguments& arguments);

 private:
  void drawEvent() override;
  void viewportEvent(ViewportEvent& event) override;
  void mousePressEvent(MouseEvent& event) override;
  void mouseReleaseEvent(MouseEvent& event) override;
  void mouseMoveEvent(MouseMoveEvent& event) override;
  void mouseScrollEvent(MouseScrollEvent& event) override;
  void keyPressEvent(KeyEvent& event) override;

  // Interactive functions
  void addObject(const std::string& configHandle);
  void addObject(int objID);

  // add template-derived object
  void addTemplateObject();

  // add primiitive object
  void addPrimitiveObject();

  void pokeLastObject();
  void pushLastObject();
  void torqueLastObject();
  void removeLastObject();
  void wiggleLastObject();
  void invertGravity();
  Mn::Vector3 randomDirection();

  // single inline for logging agent state msgs, so can be easily modified
  inline void logAgentStateMsg(bool showPos, bool showOrient) {
    std::stringstream strDat("");
    if (showPos) {
      strDat << "Agent position "
             << Eigen::Map<esp::vec3f>(agentBodyNode_->translation().data())
             << " ";
    }
    if (showOrient) {
      strDat << "Agent orientation "
             << esp::quatf(agentBodyNode_->rotation()).coeffs().transpose();
    }

    auto str = strDat.str();
    if (str.size() > 0) {
      LOG(INFO) << str;
    }
  }

  // The simulator object backend for this viewer instance
  std::unique_ptr<esp::sim::Simulator> simulator_;

  // The managers belonging to the simulator
  std::shared_ptr<esp::assets::managers::ObjectAttributesManager>
      objectAttrManager_ = nullptr;
  std::shared_ptr<esp::assets::managers::AssetAttributesManager>
      assetAttrManager_ = nullptr;
  std::shared_ptr<esp::assets::managers::SceneAttributesManager>
      sceneAttrManager_ = nullptr;
  std::shared_ptr<esp::assets::managers::PhysicsAttributesManager>
      physAttrManager_ = nullptr;

  bool debugBullet_ = false;

  esp::scene::SceneNode* rootNode_ = nullptr;
  esp::scene::SceneNode* agentBodyNode_ = nullptr;

  std::string sceneFileName;
  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  bool drawObjectBBs = false;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};
  bool showFPS_ = true;

  // NOTE: Mouse + shift is to select object on the screen!!
  void createPickedObjectVisualizer(unsigned int objectId);
  std::unique_ptr<ObjectPickingHelper> objectPickingHelper_;
};

SimViewer::SimViewer(const Arguments& arguments)
    : Mn::Platform::Application{
          arguments,
          Configuration{}.setTitle("Viewer").setWindowFlags(
              Configuration::WindowFlag::Resizable),
          GLConfiguration{}
              .setColorBufferSize(Mn::Vector4i(8, 8, 8, 8))
              .setSampleCount(4)} {
  Cr::Utility::Arguments args;
#ifdef CORRADE_TARGET_EMSCRIPTEN
  args.addNamedArgument("scene")
#else
  args.addArgument("scene")
#endif
      .setHelp("scene", "scene file to load")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp("Displays a 3D scene file provided on command line")
      .addBooleanOption("enable-physics")
      .addBooleanOption("scene-requires-lighting")
      .setHelp("scene-requires-lighting", "scene requires lighting")
      .addBooleanOption("debug-bullet")
      .setHelp("debug-bullet", "render Bullet physics debug wireframes")
      .addOption("physics-config", ESP_DEFAULT_PHYS_SCENE_CONFIG_REL_PATH)
      .setHelp("physics-config", "physics scene config file")
      .addOption("navmesh-file")
      .setHelp("navmesh-file", "manual override path to scene navmesh file")
      .addBooleanOption("recompute-navmesh")
      .setHelp("recompute-navmesh", "programmatically generate scene navmesh")
      .parse(arguments.argc, arguments.argv);

  const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();

  imgui_ =
      Mn::ImGuiIntegration::Context(Mn::Vector2{windowSize()} / dpiScaling(),
                                    windowSize(), framebufferSize());

  /* Set up proper blending to be used by ImGui. There's a great chance
     you'll need this exact behavior for the rest of your scene. If not, set
     this only for the drawFrame() call. */
  Mn::GL::Renderer::setBlendEquation(Mn::GL::Renderer::BlendEquation::Add,
                                     Mn::GL::Renderer::BlendEquation::Add);
  Mn::GL::Renderer::setBlendFunction(
      Mn::GL::Renderer::BlendFunction::SourceAlpha,
      Mn::GL::Renderer::BlendFunction::OneMinusSourceAlpha);

  // Setup renderer and shader defaults
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

  sceneFileName = args.value("scene");
  bool useBullet = args.isSet("enable-physics");
  if (useBullet && (args.isSet("debug-bullet"))) {
    debugBullet_ = true;
  }

  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.scene.id = sceneFileName;
  simConfig.enablePhysics = useBullet;
  simConfig.frustumCulling = true;

  simulator_ = esp::sim::Simulator::create_unique(simConfig);

  objectAttrManager_ = simulator_->getObjectAttributesManager();
  assetAttrManager_ = simulator_->getAssetAttributesManager();
  sceneAttrManager_ = simulator_->getSceneAttributesManager();
  physAttrManager_ = simulator_->getPhysicsAttributesManager();

  // configure and initialize default Agent and Sensor
  auto agentConfig = esp::agent::AgentConfiguration();
  // TODO: add extra, non-default actions
  agentConfig.sensorSpecifications[0]->resolution =
      esp::vec2i(viewportSize[1], viewportSize[0]);
  // add selects a random initial state and sets up the default controls and
  // step filter
  simulator_->addAgent(agentConfig);

  // Set up camera
  renderCamera_ = &simulator_->getActiveSceneGraph().getDefaultRenderCamera();
  renderCamera_->setAspectRatioPolicy(
      Mn::SceneGraph::AspectRatioPolicy::Extend);
  rootNode_ = &simulator_->getActiveSceneGraph().getRootNode();
  agentBodyNode_ = &simulator_->getAgent(0)->node();

  objectPickingHelper_ = std::make_unique<ObjectPickingHelper>(viewportSize);
  timeline_.start();

}  // end SimViewer::SimViewer

void SimViewer::addObject(int ID) {
  const std::string& configHandle =
      simulator_->getObjectAttributesManager()->getTemplateHandleByID(ID);
  addObject(configHandle);
}  // addObject

void SimViewer::addObject(const std::string& configFile) {
  // Relative to agent bodynode
  Mn::Matrix4 T = agentBodyNode_->MagnumObject::transformationMatrix();
  Mn::Vector3 new_pos = T.transformPoint({0.1f, 1.5f, -2.0f});

  int physObjectID = simulator_->addObjectByHandle(configFile);
  simulator_->setTranslation(new_pos, physObjectID);
  simulator_->setRotation(esp::core::randomRotation(), physObjectID);
}  // addObject

// add file-based template derived object from keypress
void SimViewer::addTemplateObject() {
  int numObjTemplates = objectAttrManager_->getNumFileTemplateObjects();
  if (numObjTemplates > 0) {
    addObject(objectAttrManager_->getRandomFileTemplateHandle());
  } else
    LOG(WARNING) << "No objects loaded, can't add any";

}  // addTemplateObject

// add synthesized primiitive object from keypress
void SimViewer::addPrimitiveObject() {
  // TODO : use this to implement synthesizing rendered physical objects

  int numObjPrims = objectAttrManager_->getNumSynthTemplateObjects();
  if (numObjPrims > 0) {
    addObject(objectAttrManager_->getRandomSynthTemplateHandle());
  } else
    LOG(WARNING) << "No primitive templates available, can't add any objects";

}  // addPrimitiveObject

void SimViewer::removeLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0) {
    return;
  }
  simulator_->removeObject(existingObjectIDs.back());
}

void SimViewer::invertGravity() {
  const Mn::Vector3& gravity = simulator_->getGravity();
  const Mn::Vector3 invGravity = -1 * gravity;
  simulator_->setGravity(invGravity);
}

void SimViewer::pokeLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0)
    return;
  Mn::Matrix4 T =
      agentBodyNode_->MagnumObject::transformationMatrix();  // Relative to
                                                             // agent bodynode
  Mn::Vector3 impulse = T.transformVector({0.0f, 0.0f, -3.0f});
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);
  // TODO: not in Simulator yet
  Cr::Utility::Debug() << "SimViewer::pokeLastObject() failed. applyImpulse "
                          "not in Simulator class.";
  // simulator_->applyImpulse(impulse, rel_pos, existingObjectIDs.back());
}

void SimViewer::pushLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0)
    return;
  Mn::Matrix4 T =
      agentBodyNode_->MagnumObject::transformationMatrix();  // Relative to
                                                             // agent bodynode
  Mn::Vector3 force = T.transformVector({0.0f, 0.0f, -40.0f});
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);
  simulator_->applyForce(force, rel_pos, existingObjectIDs.back());
}

void SimViewer::torqueLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0)
    return;
  Mn::Vector3 torque = randomDirection() * 30;
  simulator_->applyTorque(torque, existingObjectIDs.back());
}

// generate random direction vectors:
Mn::Vector3 SimViewer::randomDirection() {
  Mn::Vector3 dir(1.0f, 1.0f, 1.0f);
  while (sqrt(dir.dot()) > 1.0) {
    dir = Mn::Vector3((float)((rand() % 2000 - 1000) / 1000.0),
                      (float)((rand() % 2000 - 1000) / 1000.0),
                      (float)((rand() % 2000 - 1000) / 1000.0));
  }
  dir = dir / sqrt(dir.dot());
  return dir;
}

void SimViewer::wiggleLastObject() {
  // demo of kinematic motion capability
  // randomly translate last added object
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0)
    return;

  Mn::Vector3 randDir = randomDirection();
  // Only allow +Y so dynamic objects don't push through the floor.
  randDir[1] = abs(randDir[1]);

  auto translation = simulator_->getTranslation(existingObjectIDs.back());

  simulator_->setTranslation(translation + randDir * 0.1,
                             existingObjectIDs.back());
}

float timeSinceLastSimulation = 0.0;
void SimViewer::drawEvent() {
  Mn::GL::defaultFramebuffer.clear(Mn::GL::FramebufferClear::Color |
                                   Mn::GL::FramebufferClear::Depth);

  // step physics at a fixed rate
  timeSinceLastSimulation += timeline_.previousFrameDuration();
  if (timeSinceLastSimulation >= 1.0 / 60.0) {
    simulator_->stepWorld(1.0 / 60.0);
    timeSinceLastSimulation = 0.0;
  }

  // TODO: enable other sensors to be displayed
  simulator_->displayObservation(0, "rgba_camera");

  // TODO: not hooked up to Simulator
  /*
  if (debugBullet_) {
    Mn::Matrix4 camM(renderCamera_->cameraMatrix());
    Mn::Matrix4 projM(renderCamera_->projectionMatrix());

    physicsManager_->debugDraw(projM * camM);
  }
  */

  // draw picked object
  if (objectPickingHelper_->isObjectPicked()) {
    // setup blending function
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);

    // rendering
    esp::gfx::RenderCamera::Flags flags;
    if (simulator_->isFrustumCullingEnabled()) {
      flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
    }
    renderCamera_->draw(objectPickingHelper_->getDrawables(), flags);

    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
  }

  imgui_.newFrame();

  if (showFPS_) {
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::Begin("main", NULL,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(2.0);
    ImGui::Text("%.1f FPS", Mn::Double(ImGui::GetIO().Framerate));
    uint32_t total = simulator_->getActiveSceneGraph().getDrawables().size();
    ImGui::Text("%u drawables", total);
    // TODO: how to get number culled from sensor?
    // ImGui::Text("%u culled", total - visibles);
    ImGui::End();
  }

  /* Set appropriate states. If you only draw ImGui, it is sufficient to
     just enable blending and scissor test in the constructor. */
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::ScissorTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::FaceCulling);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);

  imgui_.drawFrame();

  /* Reset state. Only needed if you want to draw something else with
     different state after. */

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::ScissorTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);

  swapBuffers();
  timeline_.nextFrame();
  redraw();
}

void SimViewer::viewportEvent(ViewportEvent& event) {
  auto& sensors = simulator_->getAgent(0)->getSensorSuite();
  for (auto entry : sensors.getSensors()) {
    auto visualSensor =
        dynamic_cast<esp::sensor::VisualSensor*>(entry.second.get());
    if (visualSensor != nullptr) {
      visualSensor->specification()->resolution = {event.windowSize()[1],
                                                   event.windowSize()[0]};
      simulator_->getRenderer()->bindRenderTarget(*visualSensor);
    }
  }
  Mn::GL::defaultFramebuffer.setViewport({{}, framebufferSize()});

  imgui_.relayout(Mn::Vector2{event.windowSize()} / event.dpiScaling(),
                  event.windowSize(), event.framebufferSize());

  objectPickingHelper_->handleViewportChange(event.framebufferSize());
}

void SimViewer::createPickedObjectVisualizer(unsigned int objectId) {
  for (auto& it : simulator_->getActiveSceneGraph().getDrawableGroups()) {
    if (it.second.hasDrawable(objectId)) {
      auto* pickedDrawable = it.second.getDrawable(objectId);
      objectPickingHelper_->createPickedObjectVisualizer(pickedDrawable);
      break;
    }
  }
}

void SimViewer::mousePressEvent(MouseEvent& event) {
  if (event.button() == MouseEvent::Button::Right &&
      (event.modifiers() & MouseEvent::Modifier::Shift)) {
    // cannot use the default framebuffer, so setup another framebuffer,
    // also, setup the color attachment for rendering, and remove the visualizer
    // for the previously picked object
    objectPickingHelper_->prepareToDraw();

    // redraw the scene on the object picking framebuffer
    esp::gfx::RenderCamera::Flags flags =
        esp::gfx::RenderCamera::Flag::ObjectPicking;
    if (simulator_->isFrustumCullingEnabled())
      flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
    for (auto& it : simulator_->getActiveSceneGraph().getDrawableGroups()) {
      renderCamera_->draw(it.second, flags);
    }

    // Read the object Id
    unsigned int pickedObject =
        objectPickingHelper_->getObjectId(event.position(), windowSize());

    // if an object is selected, create a visualizer
    createPickedObjectVisualizer(pickedObject);
    return;
  }  // drawable selection
  // add primitive w/ right click
  else if (event.button() == MouseEvent::Button::Right) {
    auto viewportPoint = event.position();
    auto ray = renderCamera_->unproject(viewportPoint);
    Corrade::Utility::Debug()
        << "Ray: (org=" << ray.origin << ", dir=" << ray.direction << ")";

    esp::physics::RaycastResults raycastResults = simulator_->castRay(ray);

    for (auto& hit : raycastResults.hits) {
      Corrade::Utility::Debug() << "Hit: ";
      Corrade::Utility::Debug() << "  distance: " << hit.rayDistance;
      Corrade::Utility::Debug() << "  object: " << hit.objectId;
      Corrade::Utility::Debug() << "  point: " << hit.point;
      Corrade::Utility::Debug() << "  normal: " << hit.normal;
    }

    addPrimitiveObject();
    auto existingObjectIDs = simulator_->getExistingObjectIDs();
    if (raycastResults.hasHits()) {
      // use the bounding box to create a safety margin for adding the object
      float boundingBuffer =
          simulator_->getObjectSceneNode(existingObjectIDs.back())
                  ->computeCumulativeBB()
                  .size()
                  .max() /
              2.0 +
          0.04;
      simulator_->setTranslation(
          raycastResults.hits[0].point +
              raycastResults.hits[0].normal * boundingBuffer,
          existingObjectIDs.back());
    } else {
      simulator_->setTranslation(ray.origin + ray.direction,
                                 existingObjectIDs.back());
    }
    simulator_->setRotation(esp::core::randomRotation(),
                            existingObjectIDs.back());
  }  // end add primitive w/ right click

  event.setAccepted();
  redraw();
}

void SimViewer::mouseReleaseEvent(MouseEvent& event) {
  event.setAccepted();
}

void SimViewer::mouseScrollEvent(MouseScrollEvent& event) {
  if (!event.offset().y()) {
    return;
  }

  /* Distance to origin */
  const float distance =
      renderCamera_->node().transformation().translation().z();

  /* Move 15% of the distance back or forward */
  // TODO: update this
  // controls_(*agentBodyNode_, "moveForward",
  //          distance * (1.0f - (event.offset().y() > 0 ? 1 / 0.85f : 0.85f)));

  logAgentStateMsg(true, true);
  redraw();

  event.setAccepted();
}

void SimViewer::mouseMoveEvent(MouseMoveEvent& event) {
  if (!(event.buttons() & MouseMoveEvent::Button::Left)) {
    return;
  }
  // TODO: update this
  /*
  const Mn::Vector2i delta = event.relativePosition();
  controls_(*agentBodyNode_, "turnRight", delta.x());
  controls_(*rgbSensorNode_, "lookDown", delta.y(), false);
   */

  // logAgentStateMsg(true, true);
  redraw();

  event.setAccepted();
}

// NOTE: Mouse + shift is to select object on the screen!!
void SimViewer::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  bool agentMoved = false;
  switch (key) {
    case KeyEvent::Key::Esc:
      std::exit(0);
      break;
    case KeyEvent::Key::Left:
      simulator_->getAgent(0)->act("turnLeft");
      break;
    case KeyEvent::Key::Right:
      simulator_->getAgent(0)->act("turnRight");
      break;
    case KeyEvent::Key::Up:
      simulator_->getAgent(0)->act("lookUp");
      break;
    case KeyEvent::Key::Down:
      simulator_->getAgent(0)->act("lookDown");
      break;
    case KeyEvent::Key::Eight:
      addPrimitiveObject();
      break;
    case KeyEvent::Key::Nine:
      if (simulator_->getPathFinder()->isLoaded()) {
        const esp::vec3f position =
            simulator_->getPathFinder()->getRandomNavigablePoint();
        agentBodyNode_->setTranslation(Mn::Vector3(position));
      }
      break;
    case KeyEvent::Key::A:
      simulator_->getAgent(0)->act("moveLeft");
      break;
    case KeyEvent::Key::D:
      simulator_->getAgent(0)->act("moveRight");
      break;
    case KeyEvent::Key::S:
      simulator_->getAgent(0)->act("moveBackward");
      break;
    case KeyEvent::Key::W:
      simulator_->getAgent(0)->act("moveForward");
      break;
    case KeyEvent::Key::X:
      simulator_->getAgent(0)->act("moveDown");
      break;
    case KeyEvent::Key::Z:
      simulator_->getAgent(0)->act("moveUp");
      break;
    case KeyEvent::Key::E:
      simulator_->setFrustumCullingEnabled(
          !simulator_->isFrustumCullingEnabled());
      break;
    case KeyEvent::Key::C:
      showFPS_ = !showFPS_;
      break;
    case KeyEvent::Key::O:
      addTemplateObject();
      break;
    case KeyEvent::Key::P:
      pokeLastObject();
      break;
    case KeyEvent::Key::F:
      pushLastObject();
      break;
    case KeyEvent::Key::K:
      wiggleLastObject();
      break;
    case KeyEvent::Key::U:
      removeLastObject();
      break;
    case KeyEvent::Key::V:
      invertGravity();
      break;
    case KeyEvent::Key::T:
      // Test key. Put what you want here...
      torqueLastObject();
      break;
    case KeyEvent::Key::N:
      // toggle navmesh visualization
      simulator_->setNavMeshVisualization(
          !simulator_->isNavMeshVisualizationActive());
      break;
    case KeyEvent::Key::I:
      Mn::DebugTools::screenshot(Mn::GL::defaultFramebuffer,
                                 "test_image_save.png");
      break;
    case KeyEvent::Key::B: {
      // toggle bounding box on objects
      drawObjectBBs = !drawObjectBBs;
      for (auto id : simulator_->getExistingObjectIDs()) {
        simulator_->setObjectBBDraw(drawObjectBBs, id);
      }
    } break;
    default:
      break;
  }
  if (agentMoved) {
    logAgentStateMsg(true, true);
  }
  redraw();
}

}  // namespace

MAGNUM_APPLICATION_MAIN(SimViewer)
