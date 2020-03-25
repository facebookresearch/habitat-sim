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
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Math.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/Timeline.h>

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

#include "esp/gfx/configure.h"

#include "esp/physics/bullet/BulletPhysicsManager.h"

using namespace Magnum;
using namespace Math::Literals;
using namespace Corrade;

using namespace esp;

constexpr float moveSensitivity = 0.1f;
constexpr float lookSensitivity = 11.25f;
constexpr float rgbSensorHeight = 1.5f;

namespace {

enum MouseInteractionMode {
  PUSH,
  PULL,
  GRAB,
  THROW,

  NUM_MODES
};

std::string getEnumName(MouseInteractionMode mode) {
  switch (mode) {
    case (PUSH):
      return "PUSH";
      break;
    case (PULL):
      return "PULL";
      break;
    case (GRAB):
      return "GRAB";
      break;
    case (THROW):
      return "THROW";
      break;
  }
  return "NONE";
}

class Viewer : public Magnum::Platform::Application {
 public:
  explicit Viewer(const Arguments& arguments);

 private:
  void drawEvent() override;
  void viewportEvent(ViewportEvent& event) override;
  void mousePressEvent(MouseEvent& event) override;
  void keyReleaseEvent(MouseEvent& event) override;
  void mouseReleaseEvent(MouseEvent& event) override;
  Magnum::Vector3 unproject(const Magnum::Vector2i& windowPosition,
                            float depth) const;
  float depthAt(const Magnum::Vector2i& windowPosition);

  scene::SceneNode* clickNode_ = nullptr;
  MouseInteractionMode mouseInteractionMode = PUSH;
  void mouseMoveEvent(MouseMoveEvent& event) override;
  void mouseScrollEvent(MouseScrollEvent& event) override;
  void keyPressEvent(KeyEvent& event) override;

  // Interactive functions
  void addObject(std::string configFile);

  void throwSphere(Magnum::Vector3 direction);

  int agentObjectId = -1;

  void syncAgentObject();

  void attachAgentObject();
  void grabReleaseObject(int id);

  void syncGrippedObjects();
  void pokeLastObject();
  void pushLastObject();

  void recomputeNavMesh(const std::string& sceneFilename,
                        esp::nav::NavMeshSettings& navMeshSettings);

  void torqueLastObject();
  void removeLastObject();
  void invertGravity();
  Magnum::Vector3 randomDirection();
  void wiggleLastObject();

  void toggleNavMeshVisualization();

  Magnum::Vector3 positionOnSphere(Magnum::SceneGraph::Camera3D& camera,
                                   const Magnum::Vector2i& position);

  assets::ResourceManager resourceManager_;
  // SceneManager must be before physicsManager_ as the physicsManager_
  // assumes that it "owns" things owned by the scene manager
  scene::SceneManager sceneManager_;

  std::shared_ptr<physics::PhysicsManager> physicsManager_;

  bool debugBullet_ = false;

  std::vector<int> sceneID_;
  scene::SceneNode* agentBodyNode_ = nullptr;
  scene::SceneNode* rgbSensorNode_ = nullptr;

  scene::SceneNode* navSceneNode_ = nullptr;

  scene::SceneGraph* sceneGraph_;
  scene::SceneNode* rootNode_;

  scene::SceneNode* navmeshVisNode_;

  gfx::RenderCamera* renderCamera_ = nullptr;
  nav::PathFinder::ptr pathfinder_;
  scene::ObjectControls controls_;
  Magnum::Vector3 previousPosition_;

  std::vector<int> objectIDs_;

  bool drawObjectBBs = false;

  Magnum::Timeline timeline_;

  ImGuiIntegration::Context imgui_{NoCreate};
  bool showFPS_ = false;
  bool frustumCullingEnabled_ = true;
};

Viewer::Viewer(const Arguments& arguments)
    : Platform::Application{arguments,
                            Configuration{}.setTitle("Viewer").setWindowFlags(
                                Configuration::WindowFlag::Resizable),
                            GLConfiguration{}
                                .setColorBufferSize(Vector4i(8, 8, 8, 8))
                                .setSampleCount(4)},
      pathfinder_(nav::PathFinder::create()),
      controls_(),
      previousPosition_() {
  Utility::Arguments args;
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
      .addOption("physics-config", ESP_DEFAULT_PHYS_SCENE_CONFIG)
      .setHelp("physics-config", "physics scene config file")
      .addOption("navmesh-file")
      .setHelp("navmesh-file", "manual override path to scene navmesh file")
      .addBooleanOption("recompute-navmesh")
      .setHelp("recompute-navmesh", "programmatically generate scene navmesh")
      .parse(arguments.argc, arguments.argv);

  const auto viewportSize = GL::defaultFramebuffer.viewport().size();

  imgui_ = ImGuiIntegration::Context(Vector2{windowSize()} / dpiScaling(),
                                     windowSize(), framebufferSize());

  /* Set up proper blending to be used by ImGui. There's a great chance
     you'll need this exact behavior for the rest of your scene. If not, set
     this only for the drawFrame() call. */
  GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
                                 GL::Renderer::BlendEquation::Add);
  GL::Renderer::setBlendFunction(
      GL::Renderer::BlendFunction::SourceAlpha,
      GL::Renderer::BlendFunction::OneMinusSourceAlpha);

  // Setup renderer and shader defaults
  GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
  GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

  int sceneID = sceneManager_.initSceneGraph();
  sceneID_.push_back(sceneID);
  sceneGraph_ = &sceneManager_.getSceneGraph(sceneID);
  rootNode_ = &sceneGraph_->getRootNode();
  navSceneNode_ = &rootNode_->createChild();

  auto& drawables = sceneGraph_->getDrawables();
  const std::string& file = args.value("scene");
  assets::AssetInfo info = assets::AssetInfo::fromPath(file);
  std::string sceneLightSetup = assets::ResourceManager::NO_LIGHT_KEY;
  if (args.isSet("scene-requires-lighting")) {
    info.requiresLighting = true;
    sceneLightSetup = assets::ResourceManager::DEFAULT_LIGHTING_KEY;
  }

  if (args.isSet("enable-physics")) {
    std::string physicsConfigFilename = args.value("physics-config");
    if (!Utility::Directory::exists(physicsConfigFilename)) {
      LOG(FATAL)
          << physicsConfigFilename
          << " was not found, specify an existing file in --physics-config";
    }
    if (!resourceManager_.loadScene(info, physicsManager_, navSceneNode_,
                                    &drawables, sceneLightSetup,
                                    physicsConfigFilename)) {
      LOG(FATAL) << "cannot load " << file;
    }
    if (args.isSet("debug-bullet")) {
      debugBullet_ = true;
    }
  } else {
    if (!resourceManager_.loadScene(info, navSceneNode_, &drawables,
                                    sceneLightSetup)) {
      LOG(FATAL) << "cannot load " << file;
    }
  }

  const Magnum::Range3D& sceneBB = rootNode_->computeCumulativeBB();
  resourceManager_.setLightSetup(gfx::getLightsAtBoxCorners(sceneBB));

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
      Magnum::SceneGraph::AspectRatioPolicy::Extend);

  // Load navmesh if available
  std::string navmeshFilename;
  if (!args.value("navmesh-file").empty()) {
    navmeshFilename = Corrade::Utility::Directory::join(
        Corrade::Utility::Directory::current(), args.value("navmesh-file"));
  } else if (file.compare(esp::assets::EMPTY_SCENE)) {
    navmeshFilename = io::changeExtension(file, ".navmesh");

    // TODO: short term solution to mitigate issue #430
    // we load the pre-computed navmesh for the ptex mesh to avoid
    // online computation.
    // for long term solution, see issue #430
    if (Utility::String::endsWith(file, "mesh.ply")) {
      navmeshFilename = Corrade::Utility::Directory::join(
          Corrade::Utility::Directory::path(file) + "/habitat",
          "mesh_semantic.navmesh");
    }
  }

  if (io::exists(navmeshFilename) && !args.isSet("recompute-navmesh")) {
    LOG(INFO) << "Loading navmesh from " << navmeshFilename;
    pathfinder_->loadNavMesh(navmeshFilename);
  } else {
    esp::nav::NavMeshSettings navMeshSettings;
    navMeshSettings.setDefaults();
    recomputeNavMesh(file, navMeshSettings);
  }

  // connect controls to navmesh if loaded
  if (pathfinder_->isLoaded()) {
    // some scenes could have pathable roof polygons. We are not filtering
    // those starting points here.
    vec3f position = pathfinder_->getRandomNavigablePoint();
    agentBodyNode_->setTranslation(Vector3(position));

    controls_.setMoveFilterFunction([&](const vec3f& start, const vec3f& end) {
      vec3f currentPosition = pathfinder_->tryStep(start, end);
      LOG(INFO) << "position=" << currentPosition.transpose() << " rotation="
                << quatf(agentBodyNode_->rotation()).coeffs().transpose();
      LOG(INFO) << "Distance to closest obstacle: "
                << pathfinder_->distanceToClosestObstacle(currentPosition);

      return currentPosition;
    });
  }

  renderCamera_->node().setTransformation(
      rgbSensorNode_->absoluteTransformation());

  timeline_.start();

}  // end Viewer::Viewer

void Viewer::addObject(std::string configFile) {
  if (physicsManager_ == nullptr)
    return;

  Magnum::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 new_pos = T.transformPoint({0.1f, 2.5f, -2.0f});

  auto& drawables = sceneGraph_->getDrawables();

  assets::PhysicsObjectAttributes& objTemplate =
      resourceManager_.getPhysicsObjectAttributes(configFile);
  objTemplate.setDouble("mass", 10.0);
  objTemplate.setDouble("frictionCoefficient", 0.8);

  int physObjectID = physicsManager_->addObject(configFile, &drawables);
  physicsManager_->setTranslation(physObjectID, new_pos);

  // physicsManager_->setRotation(physObjectID, esp::core::randomRotation());

  objectIDs_.push_back(physObjectID);
}

void Viewer::throwSphere(Magnum::Vector3 direction) {
  if (physicsManager_ == nullptr)
    return;

  Magnum::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 new_pos = T.transformPoint({0.0f, 1.5f, -0.0f});

  auto& drawables = sceneGraph_->getDrawables();

  std::string physConfig(ESP_DEFAULT_PHYS_SCENE_CONFIG);
  std::string dataFolder =
      physConfig.substr(0, physConfig.find("default.phys_scene_config.json"));
  std::string sphereAsset = Corrade::Utility::Directory::join(
      dataFolder, "test_assets/objects/sphere.glb");

  int objID = resourceManager_.getObjectID(sphereAsset);
  LOG(INFO) << "object ID: " << objID;
  if (objID < 0) {
    LOG(INFO) << "adding object template ";
    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", sphereAsset);
    physicsObjectAttributes.setDouble("margin", 0.0);
    physicsObjectAttributes.setMagnumVec3("scale",
                                          Magnum::Vector3{0.25, 0.25, 0.25});
    resourceManager_.loadObject(physicsObjectAttributes, sphereAsset);
  }

  int physObjectID = physicsManager_->addObject(sphereAsset, &drawables);
  physicsManager_->setTranslation(physObjectID, new_pos);

  objectIDs_.push_back(physObjectID);

  // throw the object
  // Vector3 impulse = T.transformVector({0.0f, 1.0f, -4.0f});
  Vector3 impulse = direction;
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyImpulse(objectIDs_.back(), impulse, rel_pos);
}

void Viewer::removeLastObject() {
  if (physicsManager_ == nullptr || objectIDs_.size() == 0)
    return;
  physicsManager_->removeObject(objectIDs_.back());
  objectIDs_.pop_back();
}

void Viewer::invertGravity() {
  if (physicsManager_ == nullptr)
    return;
  const Magnum::Vector3& gravity = physicsManager_->getGravity();
  const Magnum::Vector3 invGravity = -1 * gravity;
  physicsManager_->setGravity(invGravity);
}

void Viewer::pokeLastObject() {
  if (physicsManager_ == nullptr || objectIDs_.size() == 0)
    return;
  Magnum::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 impulse = T.transformVector({0.0f, 0.0f, -3.0f});
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyImpulse(objectIDs_.back(), impulse, rel_pos);
}

void Viewer::pushLastObject() {
  if (physicsManager_ == nullptr || objectIDs_.size() == 0)
    return;
  Magnum::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 force = T.transformVector({0.0f, 0.0f, -40.0f});
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyForce(objectIDs_.back(), force, rel_pos);
}

void Viewer::recomputeNavMesh(const std::string& sceneFilename,
                              nav::NavMeshSettings& navMeshSettings) {
  nav::PathFinder::ptr pf = nav::PathFinder::create();

  assets::MeshData::uptr joinedMesh =
      resourceManager_.createJoinedCollisionMesh(sceneFilename);

  if (!pf->build(navMeshSettings, *joinedMesh)) {
    LOG(ERROR) << "Failed to build navmesh";
    return;
  }

  LOG(INFO) << "reconstruct navmesh successful";
  pathfinder_ = pf;
}

void Viewer::torqueLastObject() {
  if (physicsManager_ == nullptr || objectIDs_.size() == 0)
    return;
  Vector3 torque = randomDirection() * 30;
  physicsManager_->applyTorque(objectIDs_.back(), torque);
}

// generate random direction vectors:
Magnum::Vector3 Viewer::randomDirection() {
  Magnum::Vector3 dir(1.0f, 1.0f, 1.0f);
  while (sqrt(dir.dot()) > 1.0) {
    dir = Magnum::Vector3((float)((rand() % 2000 - 1000) / 1000.0),
                          (float)((rand() % 2000 - 1000) / 1000.0),
                          (float)((rand() % 2000 - 1000) / 1000.0));
  }
  dir = dir / sqrt(dir.dot());
  return dir;
}

void Viewer::wiggleLastObject() {
  // demo of kinematic motion capability
  // randomly translate last added object
  if (physicsManager_ == nullptr || objectIDs_.size() == 0)
    return;

  Magnum::Vector3 randDir = randomDirection();
  // Only allow +Y so dynamic objects don't push through the floor.
  randDir[1] = abs(randDir[1]);

  physicsManager_->translate(objectIDs_.back(), randDir * 0.1);
}

void Viewer::syncAgentObject() {
  if (agentObjectId >= 0) {
    Magnum::Matrix4 agentT =
        agentBodyNode_->MagnumObject::transformationMatrix();
    physicsManager_->setTransformation(agentObjectId, agentT);
    physicsManager_->setTranslation(
        agentObjectId, agentT.transformPoint(Magnum::Vector3{0, 0.5, 0}));
    physicsManager_->setActive(agentObjectId, true);
  } else {
    attachAgentObject();
  }
}

void Viewer::attachAgentObject() {
  // load the new object
  assets::PhysicsObjectAttributes agentObjectTemplate;
  std::string physConfigFile = ESP_DEFAULT_PHYS_SCENE_CONFIG;
  std::string dataDir =
      physConfigFile.substr(0, physConfigFile.find("default"));
  std::string objectFile = Corrade::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");
  LOG(INFO) << "render mesh path: " << objectFile;
  agentObjectTemplate.setString("renderMeshHandle", objectFile);
  agentObjectTemplate.setBool("useBoundingBoxForCollision", true);
  agentObjectTemplate.setMagnumVec3("scale", Magnum::Vector3{0.2, 0.5, 0.2});
  int agentBodyObjectID =
      resourceManager_.loadObject(agentObjectTemplate, "agentObject");

  // addObject without a DrawableGroup to
  auto& drawables = sceneGraph_->getDrawables();
  agentObjectId = physicsManager_->addObject(agentBodyObjectID, &drawables);
  // agentObjectId = physicsManager_->addObject(agentBodyObjectID, nullptr);
  physicsManager_->setObjectMotionType(agentObjectId,
                                       physics::MotionType::KINEMATIC);
}

std::map<int, Magnum::Matrix4> gripOffsets;
void Viewer::grabReleaseObject(int id) {
  if (objectIDs_.size() > 0) {
    if (physicsManager_->getObjectMotionType(id) ==
        physics::MotionType::KINEMATIC) {
      // already gripped, so let it go
      physicsManager_->setObjectMotionType(id, physics::MotionType::DYNAMIC);
      gripOffsets.erase(id);
    } else {
      // not gripped, so grip it
      Magnum::Matrix4 agentT =
          agentBodyNode_->MagnumObject::transformationMatrix();
      // gripOffset =
      // agentT.inverted().transformPoint(physicsManager_->getTranslation(objectIDs_.back()));
      gripOffsets[id] =
          agentT.inverted() *
          physicsManager_->getObjectSceneNode(id).transformation();
      physicsManager_->setObjectMotionType(id, physics::MotionType::KINEMATIC);
    }
  }
}

void Viewer::syncGrippedObjects() {
  Magnum::Matrix4 agentT = agentBodyNode_->MagnumObject::transformationMatrix();
  for (auto& grip : gripOffsets) {
    physicsManager_->setTransformation(grip.first, agentT * grip.second);
  }
}

void Viewer::toggleNavMeshVisualization() {
  if (navmeshVisNode_ == nullptr && pathfinder_->isLoaded()) {
    // test navmesh visualization
    navmeshVisNode_ = &rootNode_->createChild();
    int nevMeshVisPrimID = resourceManager_.loadNavMeshVisualization(
        *pathfinder_, navmeshVisNode_, &sceneGraph_->getDrawables());
    navmeshVisNode_->translate({0, 0.1, 0});
  } else {
    delete navmeshVisNode_;
    navmeshVisNode_ = nullptr;
  }
}

Vector3 Viewer::positionOnSphere(Magnum::SceneGraph::Camera3D& camera,
                                 const Vector2i& position) {
  // Convert from window to frame coordinates.
  Vector2 framePosition =
      (Vector2{position} * Vector2{framebufferSize()}) / Vector2{windowSize()};
  const Vector2 positionNormalized =
      framePosition / Vector2{camera.viewport()} - Vector2{0.5f};
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

  syncAgentObject();
  syncGrippedObjects();

  if (physicsManager_ != nullptr)
    physicsManager_->stepPhysics(timeline_.previousFrameDuration());

  int DEFAULT_SCENE = 0;
  int sceneID = sceneID_[DEFAULT_SCENE];
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  uint32_t visibles = 0;

  for (auto& it : sceneGraph.getDrawableGroups()) {
    // TODO: remove || true
    if (it.second.prepareForDraw(*renderCamera_) || true) {
      visibles += renderCamera_->draw(it.second, frustumCullingEnabled_);
    }
  }

  if (debugBullet_) {
    Magnum::Matrix4 camM(renderCamera_->cameraMatrix());
    Magnum::Matrix4 projM(renderCamera_->projectionMatrix());

    physicsManager_->debugDraw(projM * camM);
  }

  imgui_.newFrame();

  if (showFPS_) {
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::Begin("main", NULL,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(2.0);
    ImGui::Text("%.1f FPS", Double(ImGui::GetIO().Framerate));
    uint32_t total = sceneGraph.getDrawables().size();
    ImGui::Text("%u drawables", total);
    ImGui::Text("%u culled", total - visibles);
    ImGui::End();
  }

  ImGui::SetNextWindowPos(ImVec2(10, 10));
  ImGui::Begin("main", NULL,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                   ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::SetWindowFontScale(2.0);
  std::string modeText =
      "Mouse Ineraction Mode: " + getEnumName(mouseInteractionMode);
  ImGui::Text(modeText.c_str());
  ImGui::End();

  /* Set appropriate states. If you only draw ImGui, it is sufficient to
     just enable blending and scissor test in the constructor. */
  GL::Renderer::enable(GL::Renderer::Feature::Blending);
  GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
  GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
  GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

  imgui_.drawFrame();

  /* Reset state. Only needed if you want to draw something else with
     different state after. */
  GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
  GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
  GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
  GL::Renderer::disable(GL::Renderer::Feature::Blending);

  swapBuffers();
  timeline_.nextFrame();
  redraw();
}

void Viewer::viewportEvent(ViewportEvent& event) {
  GL::defaultFramebuffer.setViewport({{}, framebufferSize()});
  renderCamera_->setViewport(event.windowSize());
  imgui_.relayout(Vector2{event.windowSize()} / event.dpiScaling(),
                  event.windowSize(), event.framebufferSize());
}

void Viewer::mousePressEvent(MouseEvent& event) {
  Corrade::Utility::Debug() << "pressed";
  if (event.button() == MouseEvent::Button::Left) {
    Corrade::Utility::Debug() << "RAYCAST::";
    float depth = depthAt(event.position());
    Corrade::Utility::Debug() << "depth: " << depth;
    Magnum::Vector3 point = unproject(event.position(), depth);
    Corrade::Utility::Debug() << "point: " << point;
    if (clickNode_ == nullptr) {
      clickNode_ = &rootNode_->createChild();
      resourceManager_.addPrimitiveToDrawables(0, *clickNode_,
                                               &sceneGraph_->getDrawables());
      clickNode_->setScaling({0.1, 0.1, 0.1});
    }
    Magnum::Vector3 cast =
        (point - renderCamera_->node().absoluteTranslation()).normalized();
    clickNode_->setTranslation(renderCamera_->node().absoluteTranslation() +
                               cast * 1.0);

    // try a ray test
    physics::BulletPhysicsManager* bpm =
        static_cast<physics::BulletPhysicsManager*>(physicsManager_.get());

    btCollisionWorld::AllHitsRayResultCallback hit =
        bpm->castRay(renderCamera_->node().absoluteTranslation(), cast);
    float best_fraction = 99999.0;
    int hitID = ID_UNDEFINED;
    Magnum::Vector3 hitPoint;

    for (int hitIx = 0; hitIx < hit.m_hitPointWorld.size(); hitIx++) {
      Corrade::Utility::Debug()
          << " hit fraction: " << hit.m_hitFractions.at(hitIx);
      if (hit.m_hitFractions.at(hitIx) < best_fraction) {
        best_fraction = hit.m_hitFractions.at(hitIx);
        clickNode_->setTranslation(
            Magnum::Vector3{hit.m_hitPointWorld.at(hitIx)});
        hitID = bpm->getObjectIDFromCollisionObject(
            hit.m_collisionObjects.at(hitIx));
        hitPoint = Magnum::Vector3{hit.m_hitPointWorld.at(hitIx)};
      }
    }
    Corrade::Utility::Debug() << " hitID = " << hitID;

    if (hitID != ID_UNDEFINED) {
      if (mouseInteractionMode == PUSH) {
        physicsManager_->applyForce(hitID, cast * 500,
                                    physicsManager_->getObjectSceneNode(hitID)
                                        .absoluteTransformationMatrix()
                                        .inverted()
                                        .transformPoint(hitPoint));
      } else if (mouseInteractionMode == PULL) {
        physicsManager_->applyForce(hitID, -cast * 500,
                                    physicsManager_->getObjectSceneNode(hitID)
                                        .absoluteTransformationMatrix()
                                        .inverted()
                                        .transformPoint(hitPoint));
      } else if (mouseInteractionMode == GRAB) {
        Corrade::Utility::Debug()
            << "object select distance = "
            << (hitPoint - agentBodyNode_->absoluteTranslation()).length();
        if ((hitPoint - agentBodyNode_->absoluteTranslation()).length() < 1.0) {
          grabReleaseObject(hitID);
        }
      }
    }
    if (mouseInteractionMode == THROW) {
      throwSphere(cast * 10);
    }
  }
  // previousPosition_ = positionOnSphere(*renderCamera_, event.position());

  event.setAccepted();
}

void Viewer::mouseReleaseEvent(MouseEvent& event) {
  Corrade::Utility::Debug() << "released";
  // if (event.button() == MouseEvent::Button::Left)
  // previousPosition_ = Vector3();
  // unproject(event.position(), -1.0);

  event.setAccepted();
}

float Viewer::depthAt(const Magnum::Vector2i& windowPosition) {
  /* First scale the position from being relative to window size to being
     relative to framebuffer size as those two can be different on HiDPI
     systems */
  const Vector2i position =
      windowPosition * Vector2{framebufferSize()} / Vector2{windowSize()};
  const Vector2i fbPosition{
      position.x(),
      GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1};

  GL::defaultFramebuffer.mapForRead(
      GL::DefaultFramebuffer::ReadAttachment::Front);
  Magnum::Image2D data = GL::defaultFramebuffer.read(
      Range2Di::fromSize(fbPosition, Vector2i{1}).padded(Vector2i{2}),
      {Magnum::GL::PixelFormat::DepthComponent, Magnum::GL::PixelType::Float});

  Corrade::Utility::Debug() << "data: " << data.data();

  // fix efficiency here:
  float min = data.data()[0];
  for (auto d : data.data()) {
    if (d < min) {
      min = d;
    }
  }

  return min;

  // return Magnum::Math::min<Float>(Containers::arrayCast<const
  // Float>(data.data()));
}

Magnum::Vector3 Viewer::unproject(const Magnum::Vector2i& windowPosition,
                                  float depth) const {
  /* We have to take window size, not framebuffer size, since the position is
     in window coordinates and the two can be different on HiDPI systems */
  const Magnum::Vector2i viewSize = windowSize();
  const Magnum::Vector2i viewPosition{windowPosition.x(),
                                      viewSize.y() - windowPosition.y() - 1};
  const Magnum::Vector3 in{
      2 * Vector2{viewPosition} / Magnum::Vector2{viewSize} -
          Magnum::Vector2{1.0f},
      depth * 2.0f - 1.0f};

  // global
  return (renderCamera_->node().absoluteTransformationMatrix() *
          renderCamera_->projectionMatrix().inverted())
      .transformPoint(in);
  // camera local
  // return renderCamera_->projectionMatrix().inverted().transformPoint(in);
}

void Viewer::mouseScrollEvent(MouseScrollEvent& event) {
  if (!event.offset().y()) {
    return;
  }

  /* Distance to origin */
  /*
  const float distance =
      renderCamera_->node().transformation().translation().z();

  // Move 15% of the distance back or forward
  renderCamera_->node().translateLocal(
      {0.0f, 0.0f,
       distance * (1.0f - (event.offset().y() > 0 ? 1 / 0.85f : 0.85f))});
  */

  mouseInteractionMode =
      MouseInteractionMode((int(mouseInteractionMode) + 1) % int(NUM_MODES));

  event.setAccepted();
}

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  /*
  if (!(event.buttons() & MouseMoveEvent::Button::Left)) {
    return;
  }

  const Vector3 currentPosition =
      positionOnSphere(*renderCamera_, event.position());
  const Vector3 axis = Math::cross(previousPosition_, currentPosition);

  if (previousPosition_.length() < 0.001f || axis.length() < 0.001f) {
    return;
  }
  const auto angle = Math::angle(previousPosition_, currentPosition);
  renderCamera_->node().rotate(-angle, axis.normalized());
  previousPosition_ = currentPosition;

  event.setAccepted();
   */
}

void Viewer::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  switch (key) {
    case KeyEvent::Key::Esc:
      std::exit(0);
      break;
    case KeyEvent::Key::Left:
      controls_(*agentBodyNode_, "turnLeft", lookSensitivity);
      break;
    case KeyEvent::Key::Right:
      controls_(*agentBodyNode_, "turnRight", lookSensitivity);
      break;
    case KeyEvent::Key::Up:
      controls_(*rgbSensorNode_, "lookUp", lookSensitivity, false);
      break;
    case KeyEvent::Key::Down:
      controls_(*rgbSensorNode_, "lookDown", lookSensitivity, false);
      break;
    case KeyEvent::Key::Nine:
      if (pathfinder_->isLoaded()) {
        const vec3f position = pathfinder_->getRandomNavigablePoint();
        agentBodyNode_->setTranslation(Vector3(position));
      }
      break;
    case KeyEvent::Key::A:
      controls_(*agentBodyNode_, "moveLeft", moveSensitivity);
      LOG(INFO) << "Agent position "
                << Eigen::Map<vec3f>(agentBodyNode_->translation().data());
      break;
    case KeyEvent::Key::D:
      controls_(*agentBodyNode_, "moveRight", moveSensitivity);
      LOG(INFO) << "Agent position "
                << Eigen::Map<vec3f>(agentBodyNode_->translation().data());
      break;
    case KeyEvent::Key::E:
      frustumCullingEnabled_ ^= true;
      break;
    case KeyEvent::Key::C:
      showFPS_ = !showFPS_;
      break;
    case KeyEvent::Key::S:
      controls_(*agentBodyNode_, "moveBackward", moveSensitivity);
      LOG(INFO) << "Agent position "
                << Eigen::Map<vec3f>(agentBodyNode_->translation().data());
      break;
    case KeyEvent::Key::W:
      controls_(*agentBodyNode_, "moveForward", moveSensitivity);
      LOG(INFO) << "Agent position "
                << Eigen::Map<vec3f>(agentBodyNode_->translation().data());
      break;
    case KeyEvent::Key::X:
      controls_(*agentBodyNode_, "moveDown", moveSensitivity, false);
      LOG(INFO) << "Agent position "
                << Eigen::Map<vec3f>(agentBodyNode_->translation().data());
      break;
    case KeyEvent::Key::Z:
      controls_(*agentBodyNode_, "moveUp", moveSensitivity, false);
      LOG(INFO) << "Agent position "
                << Eigen::Map<vec3f>(agentBodyNode_->translation().data());
      break;
    case KeyEvent::Key::O: {
      if (physicsManager_ != nullptr) {
        int numObjects = resourceManager_.getNumLibraryObjects();
        if (numObjects) {
          int randObjectID = rand() % (numObjects - 1);
          addObject(resourceManager_.getObjectConfig(randObjectID));

        } else
          LOG(WARNING) << "No objects loaded, can't add any";
      } else
        LOG(WARNING)
            << "Run the app with --enable-physics in order to add objects";
    } break;
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
    case KeyEvent::Key::G:
      if (objectIDs_.size())
        grabReleaseObject(objectIDs_.back());
      break;
    case KeyEvent::Key::N:
      toggleNavMeshVisualization();
      break;
    case KeyEvent::Key::I:
      Magnum::DebugTools::screenshot(GL::defaultFramebuffer,
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
  renderCamera_->node().setTransformation(
      rgbSensorNode_->absoluteTransformation());
  redraw();
}

void Viewer::keyReleaseEvent(MouseEvent& event) {}

}  // namespace

MAGNUM_APPLICATION_MAIN(Viewer)
