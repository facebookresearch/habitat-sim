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

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/RigidObject.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneManager.h"
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

class Viewer : public Mn::Platform::Application {
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
  void updateRenderCamera();

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

  void recomputeNavMesh(const std::string& sceneFilename,
                        esp::nav::NavMeshSettings& navMeshSettings);

  void invertGravity();
  Mn::Vector3 randomDirection();

  void toggleNavMeshVisualization();

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

  esp::assets::ResourceManager resourceManager_;
  // SceneManager must be before physicsManager_ as the physicsManager_
  // assumes that it "owns" things owned by the scene manager
  esp::scene::SceneManager sceneManager_;

  std::shared_ptr<esp::physics::PhysicsManager> physicsManager_;

  bool debugBullet_ = false;

  std::vector<int> sceneID_;
  esp::scene::SceneNode* agentBodyNode_ = nullptr;
  esp::scene::SceneNode* rgbSensorNode_ = nullptr;

  std::string stageFileName;
  esp::scene::SceneGraph* sceneGraph_;
  esp::scene::SceneNode* rootNode_;

  int navMeshVisPrimID_ = esp::ID_UNDEFINED;
  esp::scene::SceneNode* navMeshVisNode_ = nullptr;

  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::nav::PathFinder::ptr pathfinder_;
  esp::scene::ObjectControls controls_;
  std::vector<int> objectIDs_;

  bool drawObjectBBs = false;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};
  bool showFPS_ = true;
  bool frustumCullingEnabled_ = true;

  // NOTE: Mouse + shift is to select object on the screen!!
  void createPickedObjectVisualizer(unsigned int objectId);
  std::unique_ptr<ObjectPickingHelper> objectPickingHelper_;
};

Viewer::Viewer(const Arguments& arguments)
    : Mn::Platform::Application{arguments,
                                Configuration{}
                                    .setTitle("Viewer")
                                    .setWindowFlags(
                                        Configuration::WindowFlag::Resizable),
                                GLConfiguration{}
                                    .setColorBufferSize(
                                        Mn::Vector4i(8, 8, 8, 8))
                                    .setSampleCount(4)},
      pathfinder_(esp::nav::PathFinder::create()),
      controls_() {
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
      .addBooleanOption("disable-navmesh")
      .setHelp("disable-navmesh",
               "disable the navmesh, so no navigation constraints and "
               "collision response")
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

  int sceneID = sceneManager_.initSceneGraph();
  sceneID_.push_back(sceneID);
  sceneGraph_ = &sceneManager_.getSceneGraph(sceneID);
  rootNode_ = &sceneGraph_->getRootNode();

  stageFileName = args.value("scene");
  esp::assets::AssetInfo info = esp::assets::AssetInfo::fromPath(stageFileName);
  std::string sceneLightSetup = esp::assets::ResourceManager::NO_LIGHT_KEY;
  if (args.isSet("scene-requires-lighting")) {
    info.requiresLighting = true;
    sceneLightSetup = esp::assets::ResourceManager::DEFAULT_LIGHTING_KEY;
  }

  std::string physicsConfigFilename = args.value("physics-config");
  if (!Cr::Utility::Directory::exists(physicsConfigFilename)) {
    LOG(FATAL)
        << physicsConfigFilename
        << " was not found, specify an existing file in --physics-config";
  }
  // use physics world attributes manager to get physics manager attributes
  // described by config file
  auto physicsManagerAttributes =
      resourceManager_.getPhysicsAttributesManager()->createAttributesTemplate(
          physicsConfigFilename, true);
  CORRADE_ASSERT(physicsManagerAttributes != nullptr,
                 "Viewer::ctor : Error attempting to load world described by"
                     << physicsConfigFilename << ". Aborting", );

  auto stageAttributesMgr = resourceManager_.getStageAttributesManager();
  stageAttributesMgr->setCurrPhysicsManagerAttributesHandle(
      physicsManagerAttributes->getHandle());

  auto stageAttributes =
      stageAttributesMgr->createAttributesTemplate(stageFileName, true);

  stageAttributes->setLightSetup(sceneLightSetup);
  stageAttributes->setRequiresLighting(info.requiresLighting);

  bool useBullet = args.isSet("enable-physics");
  // construct physics manager based on specifications in attributes
  resourceManager_.initPhysicsManager(physicsManager_, useBullet, rootNode_,
                                      physicsManagerAttributes);

  std::vector<int> tempIDs{sceneID, esp::ID_UNDEFINED};
  bool stageLoadSuccess = resourceManager_.loadStage(
      stageAttributes, physicsManager_, &sceneManager_, tempIDs, false);
  if (!stageLoadSuccess) {
    LOG(FATAL) << "cannot load " << stageFileName;
  }
  if (useBullet && (args.isSet("debug-bullet"))) {
    debugBullet_ = true;
  }

  const Mn::Range3D& sceneBB = rootNode_->computeCumulativeBB();
  resourceManager_.setLightSetup(esp::gfx::getLightsAtBoxCorners(sceneBB));

  // Set up camera
  renderCamera_ = &sceneGraph_->getDefaultRenderCamera();
  agentBodyNode_ = &rootNode_->createChild();
  rgbSensorNode_ = &agentBodyNode_->createChild();

  rgbSensorNode_->translate({0.0f, rgbSensorHeight, 0.0f});
  agentBodyNode_->translate({0.0f, 0.0f, 5.0f});

  renderCamera_->setProjectionMatrix(viewportSize.x(),  // width
                                     viewportSize.y(),  // height
                                     0.01f,             // znear
                                     1000.0f,           // zfar
                                     90.0f);            // hfov
  renderCamera_->setAspectRatioPolicy(
      Mn::SceneGraph::AspectRatioPolicy::Extend);

  if (!args.isSet("disable-navmesh")) {
    // Load navmesh if available
    std::string navmeshFilename;
    if (!args.value("navmesh-file").empty()) {
      navmeshFilename = Corrade::Utility::Directory::join(
          Corrade::Utility::Directory::current(), args.value("navmesh-file"));
    } else if (stageFileName.compare(esp::assets::EMPTY_SCENE)) {
      navmeshFilename = esp::io::changeExtension(stageFileName, ".navmesh");

      // TODO: short term solution to mitigate issue #430
      // we load the pre-computed navmesh for the ptex mesh to avoid
      // online computation.
      // for long term solution, see issue #430
      if (Cr::Utility::String::endsWith(stageFileName, "mesh.ply")) {
        navmeshFilename = Corrade::Utility::Directory::join(
            Corrade::Utility::Directory::path(stageFileName) + "/habitat",
            "mesh_semantic.navmesh");
      }
    }

    if (esp::io::exists(navmeshFilename) && !args.isSet("recompute-navmesh")) {
      LOG(INFO) << "Loading navmesh from " << navmeshFilename;
      pathfinder_->loadNavMesh(navmeshFilename);
    } else if (stageFileName.compare(esp::assets::EMPTY_SCENE)) {
      esp::nav::NavMeshSettings navMeshSettings;
      navMeshSettings.setDefaults();
      recomputeNavMesh(stageFileName, navMeshSettings);
    }

    // connect controls to navmesh if loaded
    if (pathfinder_->isLoaded()) {
      // some scenes could have pathable roof polygons. We are not filtering
      // those starting points here.
      esp::vec3f position = pathfinder_->getRandomNavigablePoint();
      agentBodyNode_->setTranslation(Mn::Vector3(position));

      controls_.setMoveFilterFunction([&](const esp::vec3f& start,
                                          const esp::vec3f& end) {
        esp::vec3f currentPosition = pathfinder_->tryStep(start, end);
        LOG(INFO)
            << "position=" << currentPosition.transpose() << " rotation="
            << esp::quatf(agentBodyNode_->rotation()).coeffs().transpose();
        LOG(INFO) << "Distance to closest obstacle: "
                  << pathfinder_->distanceToClosestObstacle(currentPosition);

        return currentPosition;
      });
    }
  }

  renderCamera_->node().setTransformation(
      rgbSensorNode_->absoluteTransformation());

  objectPickingHelper_ = std::make_unique<ObjectPickingHelper>(viewportSize);
  timeline_.start();

}  // end Viewer::Viewer

void Viewer::addObject(int ID) {
  const std::string& configHandle =
      resourceManager_.getObjectAttributesManager()->getTemplateHandleByID(ID);
  addObject(configHandle);
}  // addObject

void Viewer::addObject(const std::string& configFile) {
  // Relative to agent bodynode
  Mn::Matrix4 T = agentBodyNode_->MagnumObject::transformationMatrix();
  Mn::Vector3 new_pos = T.transformPoint({0.1f, 1.5f, -2.0f});

  auto& drawables = sceneGraph_->getDrawables();

  int physObjectID = physicsManager_->addObject(configFile, &drawables);
  physicsManager_->setTranslation(physObjectID, new_pos);

  physicsManager_->setRotation(physObjectID, esp::core::randomRotation());

  objectIDs_.push_back(physObjectID);

}  // addObject

// add file-based template derived object from keypress
void Viewer::addTemplateObject() {
  int numObjTemplates = resourceManager_.getObjectAttributesManager()
                            ->getNumFileTemplateObjects();
  if (numObjTemplates > 0) {
    addObject(resourceManager_.getObjectAttributesManager()
                  ->getRandomFileTemplateHandle());
  } else
    LOG(WARNING) << "No objects loaded, can't add any";

}  // addTemplateObject

// add synthesized primiitive object from keypress
void Viewer::addPrimitiveObject() {
  // TODO : use this to implement synthesizing rendered physical objects

  int numObjPrims = resourceManager_.getObjectAttributesManager()
                        ->getNumSynthTemplateObjects();
  if (numObjPrims > 0) {
    addObject(resourceManager_.getObjectAttributesManager()
                  ->getRandomSynthTemplateHandle());
  } else
    LOG(WARNING) << "No primitive templates available, can't add any objects";

}  // addPrimitiveObject

void Viewer::removeLastObject() {
  if (objectIDs_.size() == 0) {
    return;
  }
  physicsManager_->removeObject(objectIDs_.back());
  objectIDs_.pop_back();
}

void Viewer::invertGravity() {
  const Mn::Vector3& gravity = physicsManager_->getGravity();
  const Mn::Vector3 invGravity = -1 * gravity;
  physicsManager_->setGravity(invGravity);
}

void Viewer::pokeLastObject() {
  if (objectIDs_.size() == 0)
    return;
  Mn::Matrix4 T =
      agentBodyNode_->MagnumObject::transformationMatrix();  // Relative to
                                                             // agent bodynode
  Mn::Vector3 impulse = T.transformVector({0.0f, 0.0f, -3.0f});
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyImpulse(objectIDs_.back(), impulse, rel_pos);
}

void Viewer::pushLastObject() {
  if (objectIDs_.size() == 0)
    return;
  Mn::Matrix4 T =
      agentBodyNode_->MagnumObject::transformationMatrix();  // Relative to
                                                             // agent bodynode
  Mn::Vector3 force = T.transformVector({0.0f, 0.0f, -40.0f});
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyForce(objectIDs_.back(), force, rel_pos);
}

void Viewer::recomputeNavMesh(const std::string& sceneFilename,
                              esp::nav::NavMeshSettings& navMeshSettings) {
  esp::nav::PathFinder::ptr pf = esp::nav::PathFinder::create();

  esp::assets::MeshData::uptr joinedMesh =
      resourceManager_.createJoinedCollisionMesh(sceneFilename);

  if (!pf->build(navMeshSettings, *joinedMesh)) {
    LOG(ERROR) << "Failed to build navmesh";
    return;
  }

  LOG(INFO) << "reconstruct navmesh successful";
  pathfinder_ = pf;

  // reset the visualization if necessary
  if (navMeshVisNode_ != nullptr) {
    toggleNavMeshVisualization();  // first clear old vis
    toggleNavMeshVisualization();
  }
}

void Viewer::torqueLastObject() {
  if (objectIDs_.size() == 0)
    return;
  Mn::Vector3 torque = randomDirection() * 30;
  physicsManager_->applyTorque(objectIDs_.back(), torque);
}

// generate random direction vectors:
Mn::Vector3 Viewer::randomDirection() {
  Mn::Vector3 dir(1.0f, 1.0f, 1.0f);
  while (sqrt(dir.dot()) > 1.0) {
    dir = Mn::Vector3((float)((rand() % 2000 - 1000) / 1000.0),
                      (float)((rand() % 2000 - 1000) / 1000.0),
                      (float)((rand() % 2000 - 1000) / 1000.0));
  }
  dir = dir / sqrt(dir.dot());
  return dir;
}

void Viewer::wiggleLastObject() {
  // demo of kinematic motion capability
  // randomly translate last added object
  if (objectIDs_.size() == 0)
    return;

  Mn::Vector3 randDir = randomDirection();
  // Only allow +Y so dynamic objects don't push through the floor.
  randDir[1] = abs(randDir[1]);

  physicsManager_->translate(objectIDs_.back(), randDir * 0.1);
}

void Viewer::toggleNavMeshVisualization() {
  if (navMeshVisNode_ == nullptr && pathfinder_->isLoaded()) {
    // test navmesh visualization
    navMeshVisNode_ = &rootNode_->createChild();
    navMeshVisPrimID_ = resourceManager_.loadNavMeshVisualization(
        *pathfinder_, navMeshVisNode_, &sceneGraph_->getDrawables());
    Corrade::Utility::Debug() << "navMeshVisPrimID_ = " << navMeshVisPrimID_;
    if (navMeshVisPrimID_ == esp::ID_UNDEFINED) {
      LOG(ERROR) << "Viewer::toggleNavMeshVisualization : Failed to load "
                    "navmesh visualization.";
      delete navMeshVisNode_;
    }
  } else if (navMeshVisNode_ != nullptr) {
    delete navMeshVisNode_;
    navMeshVisNode_ = nullptr;
    resourceManager_.removePrimitiveMesh(navMeshVisPrimID_);
    navMeshVisPrimID_ = esp::ID_UNDEFINED;
  }
}

float timeSinceLastSimulation = 0.0;
void Viewer::drawEvent() {
  Mn::GL::defaultFramebuffer.clear(Mn::GL::FramebufferClear::Color |
                                   Mn::GL::FramebufferClear::Depth);
  if (sceneID_.size() <= 0)
    return;

  // step physics at a fixed rate
  timeSinceLastSimulation += timeline_.previousFrameDuration();
  if (timeSinceLastSimulation >= 1.0 / 60.0) {
    physicsManager_->stepPhysics(1.0 / 60.0);
    timeSinceLastSimulation = 0.0;
  }

  uint32_t visibles = 0;
  for (auto& it : sceneGraph_->getDrawableGroups()) {
    // TODO: remove || true
    if (it.second.prepareForDraw(*renderCamera_) || true) {
      esp::gfx::RenderCamera::Flags flags;
      if (frustumCullingEnabled_)
        flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
      visibles += renderCamera_->draw(it.second, flags);
    }
  }

  if (debugBullet_) {
    Mn::Matrix4 camM(renderCamera_->cameraMatrix());
    Mn::Matrix4 projM(renderCamera_->projectionMatrix());

    physicsManager_->debugDraw(projM * camM);
  }

  // draw picked object
  if (objectPickingHelper_->isObjectPicked()) {
    // setup blending function
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);

    // rendering
    esp::gfx::RenderCamera::Flags flags;
    if (frustumCullingEnabled_) {
      flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
    }
    renderCamera_->draw(objectPickingHelper_->getDrawables(), flags);

    // Neither the blend equation, nor the blend function is changed,
    // so no need to restore the "blending" status before the imgui draw
    /*
    // The following is to make imgui work properly:
    Mn::GL::Renderer::setBlendEquation(Mn::GL::Renderer::BlendEquation::Add,
                                       Mn::GL::Renderer::BlendEquation::Add);
    Mn::GL::Renderer::setBlendFunction(
        Mn::GL::Renderer::BlendFunction::SourceAlpha,
        Mn::GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    */
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
    uint32_t total = sceneGraph_->getDrawables().size();
    ImGui::Text("%u drawables", total);
    ImGui::Text("%u culled", total - visibles);
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

void Viewer::viewportEvent(ViewportEvent& event) {
  Mn::GL::defaultFramebuffer.setViewport({{}, framebufferSize()});
  renderCamera_->setViewport(event.windowSize());
  imgui_.relayout(Mn::Vector2{event.windowSize()} / event.dpiScaling(),
                  event.windowSize(), event.framebufferSize());

  objectPickingHelper_->handleViewportChange(event.framebufferSize());
}

void Viewer::createPickedObjectVisualizer(unsigned int objectId) {
  for (auto& it : sceneGraph_->getDrawableGroups()) {
    if (it.second.hasDrawable(objectId)) {
      auto* pickedDrawable = it.second.getDrawable(objectId);
      objectPickingHelper_->createPickedObjectVisualizer(pickedDrawable);
      break;
    }
  }
}

void Viewer::mousePressEvent(MouseEvent& event) {
  if (event.button() == MouseEvent::Button::Right &&
      (event.modifiers() & MouseEvent::Modifier::Shift)) {
    // cannot use the default framebuffer, so setup another framebuffer,
    // also, setup the color attachment for rendering, and remove the visualizer
    // for the previously picked object
    objectPickingHelper_->prepareToDraw();

    // redraw the scene on the object picking framebuffer
    esp::gfx::RenderCamera::Flags flags =
        esp::gfx::RenderCamera::Flag::UseDrawableIdAsObjectId;
    if (frustumCullingEnabled_)
      flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
    for (auto& it : sceneGraph_->getDrawableGroups()) {
      renderCamera_->draw(it.second, flags);
    }

    // Read the object Id
    unsigned int pickedObject =
        objectPickingHelper_->getObjectId(event.position(), windowSize());

    // if an object is selected, create a visualizer
    createPickedObjectVisualizer(pickedObject);
    return;
  }  // drawable selection

  // DEBUGGING/DEMO code TODO: remove this
  auto viewportPoint = event.position();
  auto ray = renderCamera_->unproject(viewportPoint);
  Corrade::Utility::Debug()
      << "Ray: (org=" << ray.origin << ", dir=" << ray.direction << ")";

  esp::physics::RaycastResults raycastResults = physicsManager_->castRay(ray);

  for (auto& hit : raycastResults.hits) {
    Corrade::Utility::Debug() << "Hit: ";
    Corrade::Utility::Debug() << "  distance: " << hit.rayDistance;
    Corrade::Utility::Debug() << "  object: " << hit.objectId;
    Corrade::Utility::Debug() << "  point: " << hit.point;
    Corrade::Utility::Debug() << "  normal: " << hit.normal;
  }

  if (event.button() == MouseEvent::Button::Left) {
    if (raycastResults.hasHits()) {
      if (raycastResults.hits[0].objectId != -1) {
        Mn::Vector3 relativeContactPoint =
            raycastResults.hits[0].point -
            physicsManager_->getTranslation(raycastResults.hits[0].objectId);
        physicsManager_->applyImpulse(raycastResults.hits[0].objectId,
                                      ray.direction * 5.0,
                                      relativeContactPoint);
      }
    }
  } else if (event.button() == MouseEvent::Button::Right) {
    addPrimitiveObject();
    if (raycastResults.hasHits()) {
      // use the bounding box to create a safety margin for adding the object
      float boundingBuffer =
          physicsManager_->getObjectSceneNode(objectIDs_.back())
                  .computeCumulativeBB()
                  .size()
                  .max() /
              2.0 +
          0.04;
      physicsManager_->setTranslation(
          objectIDs_.back(),
          raycastResults.hits[0].point +
              raycastResults.hits[0].normal * boundingBuffer);
    } else {
      physicsManager_->setTranslation(objectIDs_.back(),
                                      ray.origin + ray.direction);
    }
    physicsManager_->setRotation(objectIDs_.back(),
                                 esp::core::randomRotation());
  }
  // DEBUGGING/DEMO code end TODO: remove above this

  event.setAccepted();
  redraw();
}

void Viewer::mouseReleaseEvent(MouseEvent& event) {
  event.setAccepted();
}

void Viewer::mouseScrollEvent(MouseScrollEvent& event) {
  if (!event.offset().y()) {
    return;
  }

  /* Distance to origin */
  const float distance =
      renderCamera_->node().transformation().translation().z();

  /* Move 15% of the distance back or forward */
  controls_(*agentBodyNode_, "moveForward",
            distance * (1.0f - (event.offset().y() > 0 ? 1 / 0.85f : 0.85f)));

  logAgentStateMsg(true, true);
  updateRenderCamera();
  redraw();

  event.setAccepted();
}

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  if (!(event.buttons() & MouseMoveEvent::Button::Left)) {
    return;
  }
  const Mn::Vector2i delta = event.relativePosition();
  controls_(*agentBodyNode_, "turnRight", delta.x());
  controls_(*rgbSensorNode_, "lookDown", delta.y(), false);

  logAgentStateMsg(true, true);
  updateRenderCamera();
  redraw();

  event.setAccepted();
}

// NOTE: Mouse + shift is to select object on the screen!!
void Viewer::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  bool agentMoved = false;
  switch (key) {
    case KeyEvent::Key::Esc:
      std::exit(0);
      break;
    case KeyEvent::Key::Left:
      controls_(*agentBodyNode_, "turnLeft", lookSensitivity);
      agentMoved = true;
      break;
    case KeyEvent::Key::Right:
      controls_(*agentBodyNode_, "turnRight", lookSensitivity);
      agentMoved = true;
      break;
    case KeyEvent::Key::Up:
      controls_(*rgbSensorNode_, "lookUp", lookSensitivity, false);
      agentMoved = true;
      break;
    case KeyEvent::Key::Down:
      controls_(*rgbSensorNode_, "lookDown", lookSensitivity, false);
      agentMoved = true;
      break;
    case KeyEvent::Key::Eight:
      addPrimitiveObject();
      break;
    case KeyEvent::Key::Nine:
      if (pathfinder_->isLoaded()) {
        const esp::vec3f position = pathfinder_->getRandomNavigablePoint();
        agentBodyNode_->setTranslation(Mn::Vector3(position));
      }
      break;
    case KeyEvent::Key::A:
      controls_(*agentBodyNode_, "moveLeft", moveSensitivity);
      agentMoved = true;
      break;
    case KeyEvent::Key::D:
      controls_(*agentBodyNode_, "moveRight", moveSensitivity);
      agentMoved = true;
      break;
    case KeyEvent::Key::S:
      controls_(*agentBodyNode_, "moveBackward", moveSensitivity);
      agentMoved = true;
      break;
    case KeyEvent::Key::W:
      controls_(*agentBodyNode_, "moveForward", moveSensitivity);
      agentMoved = true;
      break;
    case KeyEvent::Key::X:
      controls_(*agentBodyNode_, "moveDown", moveSensitivity, false);
      agentMoved = true;
      break;
    case KeyEvent::Key::Z:
      controls_(*agentBodyNode_, "moveUp", moveSensitivity, false);
      agentMoved = true;
      break;
    case KeyEvent::Key::E:
      frustumCullingEnabled_ ^= true;
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
      toggleNavMeshVisualization();
      break;
    case KeyEvent::Key::I:
      Mn::DebugTools::screenshot(Mn::GL::defaultFramebuffer,
                                 "test_image_save.png");
      break;
    case KeyEvent::Key::B: {
      // toggle bounding box on objects
      drawObjectBBs = !drawObjectBBs;
      for (auto id : physicsManager_->getExistingObjectIDs()) {
        physicsManager_->setObjectBBDraw(id, &sceneGraph_->getDrawables(),
                                         drawObjectBBs);
      }
    } break;
    default:
      break;
  }
  if (agentMoved) {
    logAgentStateMsg(true, true);
  }
  updateRenderCamera();
  redraw();
}

void Viewer::updateRenderCamera() {
  renderCamera_->node().setTransformation(
      rgbSensorNode_->absoluteTransformation());
}

}  // namespace

MAGNUM_APPLICATION_MAIN(Viewer)
