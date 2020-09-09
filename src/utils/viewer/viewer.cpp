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

#include "esp/physics/bullet/BulletArticulatedObject.h"
#include "esp/physics/bullet/BulletPhysicsManager.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/sim/Simulator.h"

#include "ObjectPickingHelper.h"
#include "esp/physics/configure.h"

// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

constexpr float moveSensitivity = 0.1f;
constexpr float lookSensitivity = 11.25f;
constexpr float rgbSensorHeight = 1.5f;

struct Openable {
  int articulatedObjectId, linkId, dofIx;
  float dofOpen, dofClosed;

  esp::physics::BulletPhysicsManager* bpm;

  Openable(esp::physics::BulletPhysicsManager* _bpm,
           int _articulatedObjectId,
           int _linkId,
           int _dofIx,
           float _dofOpen,
           float _dofClosed)
      : articulatedObjectId(_articulatedObjectId),
        linkId(_linkId),
        dofIx(_dofIx),
        dofOpen(_dofOpen),
        dofClosed(_dofClosed) {
    bpm = _bpm;
  }

  virtual void openClose() {
    std::vector<float> pose =
        bpm->getArticulatedObjectPositions(articulatedObjectId);
    Corrade::Utility::Debug() << "openClose: " << pose;

    if (abs(pose[dofIx] - dofClosed) > 0.1) {
      Corrade::Utility::Debug() << "close = " << dofClosed;
      // count this as open and close it
      pose[dofIx] = dofClosed;
    } else {
      Corrade::Utility::Debug() << "open = " << dofOpen;
      pose[dofIx] = dofOpen;
    }
    bpm->setArticulatedObjectPositions(articulatedObjectId, pose);
  }
};

// locobot demo
enum LocobotControlMode {
  NO_OP,
  FORWARD,
  BACK,
  RIGHT,
  LEFT,
  RANDOM,
};

std::string getEnumName(LocobotControlMode mode) {
  switch (mode) {
    case (NO_OP):
      return "NO_OP";
      break;
    case (FORWARD):
      return "FORWARD";
      break;
    case (BACK):
      return "BACK";
      break;
    case (RIGHT):
      return "RIGHT";
      break;
    case (RANDOM):
      return "RANDOM";
      break;
  }
  return "NONE";
}

// controls a locobot
struct LocobotController {
  std::map<int, int> dofsToMotorIds;
  LocobotControlMode mode = NO_OP;
  esp::physics::BulletPhysicsManager* bpm;
  int objectId;
  int lWheelDof;
  int rWheelDof;

  int lWheelMotorId, rWheelMotorId;

  LocobotController(esp::physics::BulletPhysicsManager* _bpm,
                    int _articulatedObjectId,
                    int _lWheelDof = 2,
                    int _rWheelDof = 3)
      : bpm(_bpm), objectId(_articulatedObjectId) {
    Corrade::Utility::Debug() << "LocobotController constructor.";
    dofsToMotorIds = bpm->createMotorsForAllDofs(objectId);
    Corrade::Utility::Debug() << "dofsToMotorIds = " << dofsToMotorIds;
    std::vector<float> pose = bpm->getArticulatedObjectPositions(objectId);
    for (auto id : dofsToMotorIds) {
      bpm->updateJointMotor(objectId, id.second,
                            {pose[id.first], 1.0, 0, 0, 1.0});
    }
    lWheelDof = _lWheelDof;
    rWheelDof = _rWheelDof;
    lWheelMotorId = dofsToMotorIds.at(lWheelDof);
    rWheelMotorId = dofsToMotorIds.at(rWheelDof);
    bpm->updateJointMotor(objectId, lWheelMotorId, {0, 0, 0, 0, 10.0});
    bpm->updateJointMotor(objectId, rWheelMotorId, {0, 0, 0, 0, 10.0});
  }

  ~LocobotController() {
    esp::physics::BulletArticulatedObject& bao =
        *(static_cast<esp::physics::BulletArticulatedObject*>(
            &bpm->getArticulatedObject(objectId)));
    for (auto id : dofsToMotorIds) {
      bao.removeJointMotor(id.second);
    }
  }

  void toggle() {
    // toggle the mode
    mode = LocobotControlMode(int(mode + 1) % 6);
    Corrade::Utility::Debug() << "Set Locobot mode: " << mode;

    float wheelVel = 10.0;
    float maxImpulse = 10.0;
    switch (mode) {
      case NO_OP: {
        bpm->updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, 0, 0, maxImpulse});
        bpm->updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, 0, 0, maxImpulse});
        return;
      } break;
      case FORWARD: {
        bpm->updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, wheelVel * 2, 1.0, maxImpulse});
        bpm->updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, wheelVel * 2, 1.0, maxImpulse});
      } break;
      case BACK: {
        bpm->updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, -wheelVel * 2, 1.0, maxImpulse});
        bpm->updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, -wheelVel * 2, 1.0, maxImpulse});
      } break;
      case LEFT: {
        bpm->updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, -wheelVel, 1.0, maxImpulse});
        bpm->updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, wheelVel, 1.0, maxImpulse});
      } break;
      case RIGHT: {
        bpm->updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, wheelVel, 1.0, maxImpulse});
        bpm->updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, -wheelVel, 1.0, maxImpulse});
      } break;
      case RANDOM: {
        float randL = (float)((rand() % 2000 - 1000) / 1000.0);
        float randR = (float)((rand() % 2000 - 1000) / 1000.0);
        bpm->updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, wheelVel * randL, 1.0, 1.0});
        bpm->updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, wheelVel * randR, 1.0, 1.0});
      } break;

      default:
        break;
    }
  }
};

struct AliengoController {
  std::map<int, int> dofsToMotorIds;
  esp::physics::BulletPhysicsManager* bpm;
  int objectId;

  std::vector<float> initialPose;
  std::vector<float> cyclePose;

  int mode = 0;
  float cycleTime = 0;
  float maxImpulse = 0.45;

  AliengoController(esp::physics::BulletPhysicsManager* _bpm,
                    int _articulatedObjectId)
      : bpm(_bpm), objectId(_articulatedObjectId) {
    dofsToMotorIds = bpm->createMotorsForAllDofs(objectId);
    initialPose = bpm->getArticulatedObjectPositions(objectId);
    cyclePose = bpm->getArticulatedObjectPositions(objectId);
  }

  ~AliengoController() {
    for (auto id : dofsToMotorIds) {
      bpm->removeJointMotor(objectId, id.second);
    }
  }

  void toggle() {
    mode = (mode + 1) % 3;
    Corrade::Utility::Debug() << "AliengoController toggle mode = " << mode;
    float positionGain = 0.0;
    if (mode != 0) {
      positionGain = 1.0;
    }
    for (auto id : dofsToMotorIds) {
      if (mode != 2) {
        bpm->updateJointMotor(objectId, id.second,
                              {initialPose[id.first], positionGain, 0,
                               maxImpulse / 2.0, maxImpulse});
      } else {
        bpm->updateJointMotor(objectId, id.second,
                              {cyclePose[id.first], positionGain, 0,
                               maxImpulse / 2.0, maxImpulse});
      }
    }
  }

  void cycleUpdate(float dt) {
    if (mode == 2) {
      cycleTime += dt / 2.0;
      float sinDof = sin(cycleTime);
      std::vector<int> shoulderDofs = {1, 4, 7, 10};
      for (auto dof : shoulderDofs) {
        cyclePose[dof] = sinDof;
      }
      for (auto id : dofsToMotorIds) {
        bpm->updateJointMotor(
            objectId, id.second,
            {cyclePose[id.first], 1.0, 0, maxImpulse / 2.0, maxImpulse});
      }
    }
  }
};

enum MouseInteractionMode {
  OPENCLOSE,
  GRAB,
  THROW,
  DOF,

  NUM_MODES
};

// dof controlled by the mouse
int mouseControlDof = 0;

std::string getEnumName(MouseInteractionMode mode) {
  switch (mode) {
    case (OPENCLOSE):
      return "OPEN|CLOSE";
      break;
    case (GRAB):
      return "GRAB";
      break;
    case (THROW):
      return "THROW";
      break;
    case (DOF):
      return "DOF (" + std::to_string(mouseControlDof) + ")";
      break;
  }
  return "NONE";
}

struct MouseGrabber {
  Magnum::Vector3 target;
  int p2pId;
  esp::physics::BulletPhysicsManager* bpm;

  float gripDepth;

  MouseGrabber(const Magnum::Vector3& clickPos,
               float _gripDepth,
               esp::physics::BulletPhysicsManager* _bpm) {
    bpm = _bpm;
    target = clickPos;
    gripDepth = _gripDepth;
  }

  virtual ~MouseGrabber() { bpm->removeP2PConstraint(p2pId); }

  virtual void updatePivotB(Magnum::Vector3 pos) {
    bpm->updateP2PConstraintPivot(p2pId, pos);
  }
};

struct MouseLinkGrabber : public MouseGrabber {
  int articulatedObjectId, linkId;

  MouseLinkGrabber(const Magnum::Vector3& clickPos,
                   float _gripDepth,
                   int _articulatedObjectId,
                   int _linkId,
                   esp::physics::BulletPhysicsManager* _bpm)
      : MouseGrabber(clickPos, _gripDepth, _bpm) {
    articulatedObjectId = _articulatedObjectId;
    linkId = _linkId;
    Corrade::Utility::Debug()
        << "MouseLinkGrabber init: articulatedObjectId=" << articulatedObjectId
        << ", linkId=" << linkId;
    p2pId = _bpm->createArticulatedP2PConstraint(articulatedObjectId, linkId,
                                                 clickPos);
  }
};

//! kinematically transform the root of the selected articulated object
struct MouseArticulatedBaseGrabber : public MouseGrabber {
  int articulatedObjectId;
  Magnum::Vector3 rootClickOffset;
  MouseArticulatedBaseGrabber(const Magnum::Vector3& clickPos,
                              float _gripDepth,
                              int _articulatedObjectId,
                              esp::physics::BulletPhysicsManager* _bpm)
      : MouseGrabber(clickPos, _gripDepth, _bpm) {
    articulatedObjectId = _articulatedObjectId;
    Magnum::Vector3 root =
        bpm->getArticulatedObjectRootState(articulatedObjectId).translation();
    rootClickOffset = root - clickPos;
  }

  virtual ~MouseArticulatedBaseGrabber() override {
    Corrade::Utility::Debug()
        << "~MouseArticulatedBaseGrabber final root pos: "
        << bpm->getArticulatedObjectRootState(articulatedObjectId)
               .translation();
  }

  virtual void updatePivotB(Magnum::Vector3 pos) override {
    Magnum::Matrix4 rootState =
        bpm->getArticulatedObjectRootState(articulatedObjectId);
    rootState.translation() = pos + rootClickOffset;
    Corrade::Utility::Debug() << "newRootState = " << rootState;
    bpm->setArticulatedObjectSleep(articulatedObjectId, false);
    bpm->setArticulatedObjectRootState(articulatedObjectId, rootState);
  }
};

struct MouseObjectGrabber : public MouseGrabber {
  int objectId;

  MouseObjectGrabber(const Magnum::Vector3& clickPos,
                     float _gripDepth,
                     int _objectId,
                     esp::physics::BulletPhysicsManager* _bpm)
      : MouseGrabber(clickPos, _gripDepth, _bpm) {
    objectId = _objectId;
    bpm->setObjectMotionType(objectId, esp::physics::MotionType::DYNAMIC);
    Corrade::Utility::Debug()
        << "MouseObjectGrabber init: objectId=" << objectId;
    p2pId = _bpm->createRigidP2PConstraintFromPickPoint(objectId, clickPos);
  }
};

struct MouseObjectKinematicGrabber : public MouseGrabber {
  int objectId;
  Magnum::Vector3 clickOffset;
  MouseObjectKinematicGrabber(const Magnum::Vector3& clickPos,
                              float _gripDepth,
                              int _objectId,
                              esp::physics::BulletPhysicsManager* _bpm)
      : MouseGrabber(clickPos, _gripDepth, _bpm) {
    objectId = _objectId;
    Magnum::Vector3 origin = bpm->getTranslation(objectId);
    clickOffset = origin - clickPos;
    bpm->setObjectMotionType(objectId, esp::physics::MotionType::KINEMATIC);
  }

  virtual ~MouseObjectKinematicGrabber() override {
    Corrade::Utility::Debug()
        << "~MouseObjectKinematicGrabber final origin pos: "
        << bpm->getTranslation(objectId);
  }

  virtual void updatePivotB(Magnum::Vector3 pos) override {
    Magnum::Vector3 objectOrigin = bpm->getTranslation(objectId);
    bpm->setTranslation(objectId, clickOffset + pos);
  }
};

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

  esp::scene::SceneNode* clickNode_ = nullptr;

  std::unique_ptr<MouseGrabber> mouseGrabber_ = nullptr;

  MouseInteractionMode mouseInteractionMode = GRAB;

  std::vector<Openable> openableObjects;
  std::vector<std::unique_ptr<LocobotController>> locobotControllers;
  std::vector<std::unique_ptr<AliengoController>> aliengoControllers;

  // Interactive functions
  void addObject(const std::string& configHandle);
  void addObject(int objID);

  void throwSphere(Magnum::Vector3 direction);

  // add template-derived object
  void addTemplateObject();

  // add primiitive object
  void addPrimitiveObject();

  int addArticulatedObject(std::string urdfFilename, bool fixedBase = false);

  void clearAllObjects();

  void placeArticulatedObjectAgentFront(int objectId);

  void toggleArticulatedMotionType(int objectId);

  // DEMO setup functions
  void setupDemoFurniture();

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

  std::vector<int> articulatedObjectIDs_;

  bool drawObjectBBs = false;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};
  bool showFPS_ = false;
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

  // Setting manual agent position
  // middle of room
  agentBodyNode_->setTranslation({2.8141, -1.44123, 2.61645});
  agentBodyNode_->setRotation({{0, -0.7071, 0}, -0.707107});

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

void Viewer::throwSphere(Mn::Vector3 direction) {
  if (physicsManager_ == nullptr)
    return;

  Mn::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Mn::Vector3 new_pos = T.transformPoint({0.0f, 1.5f, -0.5f});

  esp::assets::PrimObjTypes icoSphere =
      esp::assets::PrimObjTypes::ICOSPHERE_SOLID;
  esp::assets::attributes::AbstractPrimitiveAttributes::ptr sphereAttributes =
      resourceManager_.getAssetAttributesManager()->createAttributesTemplate(
          icoSphere);
  int sphereObjectTemplateId = resourceManager_.getObjectAttributesManager()
                                   ->createPrimBasedAttributesTemplate(
                                       sphereAttributes->getHandle(), true)
                                   ->getID();

  int physObjectID = physicsManager_->addObject(sphereObjectTemplateId,
                                                &sceneGraph_->getDrawables());
  physicsManager_->setTranslation(physObjectID, new_pos);

  objectIDs_.push_back(physObjectID);

  // throw the object
  Mn::Vector3 impulse = direction;
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyImpulse(objectIDs_.back(), impulse, rel_pos);
}

int Viewer::addArticulatedObject(std::string urdfFilename, bool fixedBase) {
  int articulatedObjectId = physicsManager_->addArticulatedObjectFromURDF(
      urdfFilename, &sceneGraph_->getDrawables(), fixedBase);
  articulatedObjectIDs_.push_back(articulatedObjectId);
  placeArticulatedObjectAgentFront(articulatedObjectId);
  return articulatedObjectId;
}

void Viewer::placeArticulatedObjectAgentFront(int objectId) {
  Mn::Vector3 localBasePos{0.0f, 1.0f, -2.0f};
  Mn::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  // rotate the object
  Mn::Matrix4 initialArticulatedObjectTransform;
  // = Mn::Matrix4::rotationX(Mn::Rad(-3.14 / 2.0));
  initialArticulatedObjectTransform.translation() =
      T.transformPoint(localBasePos);
  esp::physics::BulletPhysicsManager* bpm =
      static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());
  bpm->setArticulatedObjectRootState(objectId,
                                     initialArticulatedObjectTransform);
}

void Viewer::toggleArticulatedMotionType(int objectId) {
  esp::physics::BulletPhysicsManager* bpm =
      static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());
  if (bpm->getArticulatedObjectMotionType(objectId) ==
      esp::physics::MotionType::DYNAMIC) {
    bpm->setArticulatedObjectMotionType(objectId,
                                        esp::physics::MotionType::KINEMATIC);
    Corrade::Utility::Debug() << "setting MotionType::KINEMATIC";
  } else {
    bpm->setArticulatedObjectMotionType(objectId,
                                        esp::physics::MotionType::DYNAMIC);
    Corrade::Utility::Debug() << "setting MotionType::DYNAMIC";
  }
}

void Viewer::removeLastObject() {
  if (objectIDs_.size() == 0) {
    return;
  }
  physicsManager_->removeObject(objectIDs_.back());
  objectIDs_.pop_back();
}

void Viewer::clearAllObjects() {
  locobotControllers.clear();
  aliengoControllers.clear();
  for (auto id : objectIDs_) {
    physicsManager_->removeObject(id);
  }
  for (auto id : articulatedObjectIDs_) {
    physicsManager_->removeArticulatedObject(id);
  }
  objectIDs_.clear();
  articulatedObjectIDs_.clear();
  openableObjects.clear();
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
    for (auto& aliengoController : aliengoControllers) {
      aliengoController->cycleUpdate(1.0 / 60.0);
    }
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

  ImGui::SetNextWindowPos(ImVec2(10, 10));
  ImGui::Begin("main", NULL,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                   ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::SetWindowFontScale(2.0);
  std::string modeText =
      "Mouse Ineraction Mode: " + getEnumName(mouseInteractionMode);
  ImGui::Text("%s", modeText.c_str());
  ImGui::End();

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

  if (event.button() == MouseEvent::Button::Left ||
      event.button() == MouseEvent::Button::Right) {
    auto viewportPoint = event.position();
    auto ray = renderCamera_->unproject(viewportPoint);
    Corrade::Utility::Debug()
        << "Ray: (org=" << ray.origin << ", dir=" << ray.direction << ")";

    esp::physics::RaycastResults raycastResults = physicsManager_->castRay(ray);

    // create the clickNode primitive if not present
    if (clickNode_ == nullptr) {
      clickNode_ = &rootNode_->createChild();
      resourceManager_.addPrimitiveToDrawables(0, *clickNode_,
                                               &sceneGraph_->getDrawables());
      clickNode_->setScaling({0.1, 0.1, 0.1});
    }

    clickNode_->setTranslation(renderCamera_->node().absoluteTranslation() +
                               ray.direction);

    // ray hit a collision object, visualize this and determine the hit object
    if (raycastResults.hasHits()) {
      auto hitInfo = raycastResults.hits[0];  // first hit
      clickNode_->setTranslation(hitInfo.point);

      if (hitInfo.objectId != esp::ID_UNDEFINED) {
        // we hit an non-stage collision object

        esp::physics::BulletPhysicsManager* bpm =
            static_cast<esp::physics::BulletPhysicsManager*>(
                physicsManager_.get());

        bool hitArticulatedObject = false;
        // TODO: determine if this is true (link id?)
        int hitArticulatedObjectId = esp::ID_UNDEFINED;
        int hitArticulatedLinkIndex = esp::ID_UNDEFINED;
        // TODO: get this info from link?
        for (auto aoId : physicsManager_->getExistingArticulatedObjectIDs()) {
          if (aoId == hitInfo.objectId) {
            // TODO: grabbed the base link, do something with this
          } else if (physicsManager_->getArticulatedObject(aoId)
                         .objectIdToLinkId_.count(hitInfo.objectId) > 0) {
            hitArticulatedObject = true;
            hitArticulatedObjectId = aoId;
            hitArticulatedLinkIndex =
                physicsManager_->getArticulatedObject(aoId)
                    .objectIdToLinkId_.at(hitInfo.objectId);
          }
        }

        if (mouseInteractionMode == GRAB) {
          if (hitArticulatedObject) {
            if (event.button() == MouseEvent::Button::Right) {
              mouseGrabber_ = std::make_unique<MouseArticulatedBaseGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().translation())
                      .length(),
                  hitArticulatedObjectId, bpm);
            } else if (event.button() == MouseEvent::Button::Left) {
              mouseGrabber_ = std::make_unique<MouseLinkGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().translation())
                      .length(),
                  hitArticulatedObjectId, hitArticulatedLinkIndex, bpm);
            }
          } else {
            if (event.button() == MouseEvent::Button::Right) {
              mouseGrabber_ = std::make_unique<MouseObjectKinematicGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().translation())
                      .length(),
                  hitInfo.objectId, bpm);
            } else if (event.button() == MouseEvent::Button::Left) {
              mouseGrabber_ = std::make_unique<MouseObjectGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().translation())
                      .length(),
                  hitInfo.objectId, bpm);
            }
          }
        } else if (mouseInteractionMode == OPENCLOSE) {
          if (hitArticulatedObject) {
            Corrade::Utility::Debug()
                << "OPENCLOSE: aoid = " << hitArticulatedObjectId
                << ", linkId = " << hitInfo.objectId;
            for (auto openable : openableObjects) {
              if (openable.articulatedObjectId == hitArticulatedObjectId) {
                if (openable.linkId == hitArticulatedLinkIndex) {
                  openable.openClose();
                }
              }
            }
            for (auto& locobotController : locobotControllers) {
              if (locobotController->objectId == hitArticulatedObjectId) {
                locobotController->toggle();
              }
            }
            for (auto& aliengoController : aliengoControllers) {
              if (aliengoController->objectId == hitArticulatedObjectId) {
                aliengoController->toggle();
              }
            }
          }
        }
      }  // end if not hit stage
    }    // end raycastResults.hasHits()
    if (mouseInteractionMode == THROW) {
      throwSphere(ray.direction * 10);
    }
  }  // end MouseEvent::Button::Left || Right

  event.setAccepted();
  redraw();
}

int mouseDofDelta = 0;
void Viewer::mouseReleaseEvent(MouseEvent& event) {
  // reset the mouse delta for dof selection
  mouseDofDelta = 0;
  if (event.button() == MouseEvent::Button::Left ||
      event.button() == MouseEvent::Button::Right) {
    mouseGrabber_ = nullptr;

    // print the DOF
    if (mouseInteractionMode == DOF) {
      if (articulatedObjectIDs_.size()) {
        esp::physics::BulletPhysicsManager* bpm =
            static_cast<esp::physics::BulletPhysicsManager*>(
                physicsManager_.get());
        std::vector<float> pose =
            bpm->getArticulatedObjectPositions(articulatedObjectIDs_.back());
        Corrade::Utility::Debug()
            << "DOF(" << mouseControlDof << ") = " << pose[mouseControlDof];
      }
    }
  }

  event.setAccepted();
}

void Viewer::mouseScrollEvent(MouseScrollEvent& event) {
  if (!event.offset().y()) {
    return;
  }

  if (mouseGrabber_ != nullptr) {
    // adjust the depth
    auto ray = renderCamera_->unproject(event.position());
    mouseGrabber_->gripDepth += event.offset().y() * 0.01;
    mouseGrabber_->target = renderCamera_->node().absoluteTranslation() +
                            ray.direction * mouseGrabber_->gripDepth;
    mouseGrabber_->updatePivotB(mouseGrabber_->target);
    clickNode_->setTranslation(mouseGrabber_->target);
  } else {
    // change the mouse interaction mode
    int delta = 1;
    if (event.offset().y() < 0)
      delta = -1;
    mouseInteractionMode = MouseInteractionMode(
        (int(mouseInteractionMode) + delta) % int(NUM_MODES));
    if (mouseInteractionMode < 0)
      mouseInteractionMode = MouseInteractionMode(int(NUM_MODES) - 1);
  }

  event.setAccepted();
}

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  if (mouseGrabber_ != nullptr) {
    auto ray = renderCamera_->unproject(event.position());
    mouseGrabber_->target = renderCamera_->node().absoluteTranslation() +
                            ray.direction * mouseGrabber_->gripDepth;
    mouseGrabber_->updatePivotB(mouseGrabber_->target);
    clickNode_->setTranslation(mouseGrabber_->target);
  } else if (mouseInteractionMode == DOF) {
    if (articulatedObjectIDs_.size()) {
      esp::physics::BulletPhysicsManager* bpm =
          static_cast<esp::physics::BulletPhysicsManager*>(
              physicsManager_.get());
      if (event.buttons() & MouseMoveEvent::Button::Left) {
        std::vector<float> pose =
            bpm->getArticulatedObjectPositions(articulatedObjectIDs_.back());
        mouseControlDof = mouseControlDof % pose.size();
        if (mouseControlDof < 0)
          mouseControlDof = pose.size() - 1;
        pose[mouseControlDof] += event.relativePosition()[0] * 0.02;
        bpm->setArticulatedObjectPositions(articulatedObjectIDs_.back(), pose);
      } else if (event.buttons() & MouseMoveEvent::Button::Right) {
        mouseDofDelta += event.relativePosition()[0];
        if (abs(mouseDofDelta) > 20) {
          mouseControlDof += mouseDofDelta / abs(mouseDofDelta);
          mouseDofDelta = 0;
        }
        if (mouseControlDof < 0)
          mouseControlDof =
              bpm->getArticulatedObjectPositions(articulatedObjectIDs_.back())
                  .size() -
              1;
        mouseControlDof = mouseControlDof % bpm->getArticulatedObjectPositions(
                                                   articulatedObjectIDs_.back())
                                                .size();
      }
    }
  }

  event.setAccepted();
}

// NOTE: Mouse + shift is to select object on the screen!!
void Viewer::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  bool agentMoved = false;
  // TODO: placeholder until full API is established.
  esp::physics::BulletPhysicsManager* bpm =
      static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());
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
    case KeyEvent::Key::M:
      toggleArticulatedMotionType(articulatedObjectIDs_.back());
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
    case KeyEvent::Key::Zero: {
      std::string urdfFilePath =
          "data/URDF_demo_assets/aliengo/urdf/aliengo.urdf";
      int objectId = addArticulatedObject(urdfFilePath);
      auto R = Magnum::Matrix4::rotationX(Magnum::Rad(-1.56));
      R.translation() =
          bpm->getArticulatedObjectRootState(objectId).translation();
      bpm->setArticulatedObjectRootState(objectId, R);
      // manually set joint damping
      for (auto motor : physicsManager_->getExistingJointMotors(objectId)) {
        bpm->updateJointMotor(objectId, motor.first, {0, 0, 0, 1.0, 0.1});
      }
      // modify the pose to account for joint limits
      std::vector<float> pose =
          physicsManager_->getArticulatedObjectPositions(objectId);
      std::vector<int> calfDofs = {2, 5, 8, 11};
      for (auto dof : calfDofs) {
        pose[dof] = -1.0;
        pose[dof - 1] = 0.45;  // also set a thigh
      }
      physicsManager_->setArticulatedObjectPositions(objectId, pose);
      auto aliengoController =
          std::make_unique<AliengoController>(bpm, objectId);
      aliengoControllers.push_back(std::move(aliengoController));
    } break;
    case KeyEvent::Key::One: {
      std::string urdfFilePath =
          "data/test_assets/URDF/kuka_iiwa/model_free_base.urdf";
      int objectId = addArticulatedObject(urdfFilePath, true);
      // manually adjust joint damping (half impulse)
      for (auto motor : physicsManager_->getExistingJointMotors(objectId)) {
        auto settings =
            physicsManager_->getJointMotorSettings(objectId, motor.first);
        Mn::Debug{} << "motor: " << motor;
        physicsManager_->updateJointMotor(
            objectId, motor.first, {0, 0, 0, 1.0, settings.maxImpulse / 2.0});
      }
    } break;
    case KeyEvent::Key::Two: {
      // TODO: open
    } break;
    case KeyEvent::Key::Three: {
      std::string urdfFilePath =
          "data/URDF_demo_assets/locobot/urdf/"
          "locobot_description_lite2.urdf";
      int objectId = addArticulatedObject(urdfFilePath);
      auto R = Magnum::Matrix4::rotationX(Magnum::Rad(-1.56));
      R.translation() =
          bpm->getArticulatedObjectRootState(articulatedObjectIDs_.back())
              .translation();
      bpm->setArticulatedObjectRootState(articulatedObjectIDs_.back(), R);
      auto locobotController = std::make_unique<LocobotController>(
          bpm, articulatedObjectIDs_.back());
      locobotControllers.push_back(std::move(locobotController));
    } break;
    case KeyEvent::Key::Four: {
      // Locobot with arm: dynamics don't work very well
      std::string urdfFilePath =
          "data/URDF_demo_assets/locobot/urdf/"
          "locobot_description2.urdf";
      int objectId = addArticulatedObject(urdfFilePath);
      auto R = Magnum::Matrix4::rotationX(Magnum::Rad(-1.56));
      R.translation() =
          bpm->getArticulatedObjectRootState(articulatedObjectIDs_.back())
              .translation();
      bpm->setArticulatedObjectRootState(articulatedObjectIDs_.back(), R);
      // manually set joint damping
      for (auto motor : physicsManager_->getExistingJointMotors(objectId)) {
        bpm->updateJointMotor(objectId, motor.first, {0, 0, 0, 1.0, 0.1});
      }
      std::vector<float> pose =
          physicsManager_->getArticulatedObjectPositions(objectId);
      pose[4] = -1.7;
      physicsManager_->setArticulatedObjectPositions(objectId, pose);
      auto locobotController = std::make_unique<LocobotController>(
          bpm, articulatedObjectIDs_.back(), 9, 10);
      locobotControllers.push_back(std::move(locobotController));
    } break;
    case KeyEvent::Key::Five: {
      // reset the scene
      clearAllObjects();
      Corrade::Utility::Debug() << "done clearing, now generating";
      setupDemoFurniture();
    } break;
    case KeyEvent::Key::Space: {
      if (articulatedObjectIDs_.size() > 0) {
        Corrade::Utility::Debug()
            << "setArticulatedObjectSleep: "
            << !bpm->getArticulatedObjectSleep(articulatedObjectIDs_.back());
        bpm->setArticulatedObjectSleep(
            articulatedObjectIDs_.back(),
            !bpm->getArticulatedObjectSleep(articulatedObjectIDs_.back()));
      }
    } break;
    case KeyEvent::Key::R: {
      bpm->setArticulatedObjectSleep(articulatedObjectIDs_.back(), false);
      placeArticulatedObjectAgentFront(articulatedObjectIDs_.back());
      bpm->resetArticulatedObject(articulatedObjectIDs_.back());
      bpm->setArticulatedObjectSleep(articulatedObjectIDs_.back(), true);
    } break;
    case KeyEvent::Key::Minus: {
      if (articulatedObjectIDs_.size()) {
        for (auto& controller : locobotControllers) {
          if (controller->objectId == articulatedObjectIDs_.back()) {
            locobotControllers.pop_back();
          }
        }
        for (auto& controller : aliengoControllers) {
          if (controller->objectId == articulatedObjectIDs_.back()) {
            aliengoControllers.pop_back();
          }
        }
        physicsManager_->removeArticulatedObject(articulatedObjectIDs_.back());
        articulatedObjectIDs_.pop_back();
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

void Viewer::setupDemoFurniture() {
  esp::physics::BulletPhysicsManager* bpm =
      static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());
  Magnum::Matrix4 T;
  // add the articualted objects
  {
    std::vector<Mn::Vector3> objectPositions = {
        {1.68198, -1.5831, 5.50846},     // door1 (kitchen)
        {1.08896, 0.856144, -1.20688},   // door2 (1st hall closet)
        {1.08632, 0.527348, -1.87166},   // door3 (hall stairway closet)
        {-0.550399, 0.478112, -2.6035},  // doubledoor
        {-0.533, -0.5, 4.7},             // fridge
        {2.93748, -1.52348, 3.11267},    // cabinet
        {-0.4, -1.53703, 2.7}            // kitchen_counter
    };
    std::vector<std::string> objectFilenames = {
        "data/test_assets/URDF/doors/door1.urdf",
        "data/test_assets/URDF/doors/door2.urdf",
        "data/test_assets/URDF/doors/door3.urdf",
        "data/test_assets/URDF/doors/doubledoor.urdf",
        "data/test_assets/URDF/fridge/fridge.urdf",
        "data/test_assets/URDF/cabinet/cabinet.urdf",
        "data/test_assets/URDF/kitchen_counter/kitchen_counter.urdf",
    };

    int initialArticulatedObjectCount = articulatedObjectIDs_.size();
    for (size_t i = 0; i < objectFilenames.size(); ++i) {
      addArticulatedObject(objectFilenames[i], true);
      T = bpm->getArticulatedObjectRootState(articulatedObjectIDs_.back());
      T.translation() = objectPositions[i];
      bpm->setArticulatedObjectRootState(articulatedObjectIDs_.back(), T);
      bpm->resetArticulatedObject(articulatedObjectIDs_.back());
      bpm->setArticulatedObjectSleep(articulatedObjectIDs_.back(), true);
    }

    for (auto objectId : articulatedObjectIDs_) {
      bpm->resetArticulatedObject(articulatedObjectIDs_.back());
      bpm->setArticulatedObjectSleep(articulatedObjectIDs_.back(), true);
    }

    // setup the openable entries
    openableObjects = {
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 0,
                 0, 2.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 0,
                 0, -1.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 0,
                 0, 2.4, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 1,
                 0, -1.2, 0),  // doubledoor
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 2,
                 1, 1.0, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 1,
                 0, 2.3, 0),  // fridge
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 2,
                 1, 2.3, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 1,
                 0, 0.8, 0),  // cabinet
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 2,
                 1, -0.76, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 1,
                 0, 0.5, 0),  // kitchen counter
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 2,
                 1, 0.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 3,
                 2, 0.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 4,
                 3, 0.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 5,
                 4, 0.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount], 6,
                 5, 0.5, 0),
        Openable(bpm, articulatedObjectIDs_[initialArticulatedObjectCount++], 7,
                 6, 0.5, 0),
    };
  }

  {
    // TODO: change these paths for your local directories or download the
    // assets these folders.
    std::string zatunObjectsPath = "data/objects/zatun_objects/";
    std::string ycbObjectsPath = "data/objects/ycb_google_16k/";
    std::vector<std::string> objectFilenames = {
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_lamp.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_small_appliance_01.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_picture_03.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_basket.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_knifeblock.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_kitchen_utensil_08.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_kitchen_utensil_09.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_kitchen_utensil_06.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_choppingboard_02.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_bowl_06.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_cup_02.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_kitchen_utensil_02.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_cup_03.phys_properties.json",
        zatunObjectsPath +
            "configs_convex/"
            "frl_apartment_cup_01.phys_properties.json",
        ycbObjectsPath +
            "configs/"
            "006_mustard_bottle.phys_properties.json",
    };
    std::vector<int> parsedObjectIds;
    for (auto filename : objectFilenames) {
      parsedObjectIds.push_back(
          resourceManager_.getObjectAttributesManager()
              ->createFileBasedAttributesTemplate(filename)
              ->getID());
    }
    std::vector<std::pair<int, Mn::Vector3>> objectPositions = {
        {0, {2.2311, -0.425956, 3.58849}},      // lamp 1
        {0, {2.76, -0.425956, 3.85}},           // lamp 1
        {1, {-0.468548, -0.507495, 2.59997}},   // mixer
        {2, {-0.660318, -0.523204, 1.89543}},   // picture
        {3, {-0.379807, -0.476218, 1.5544}},    // basket
        {4, {-0.776413, -0.551627, 1.86241}},   // knife block
        {5, {-0.622869, -0.568486, 1.77434}},   // untensil 8 (canister)
        {6, {-0.661958, -0.56693, 1.68653}},    // untensil 9 (canister)
        {7, {-0.795027, -0.590883, 2.00284}},   // untensil 6 (canister)
        {8, {-0.944428, -0.505018, 1.75193}},   // chopping board (canister)
        {9, {-0.384167, -0.565578, 3.38958}},   // bowl 6
        {10, {-0.604808, -0.557711, 3.5138}},   // cup 2
        {11, {-0.785562, -0.539605, 3.38024}},  // utensil 2 (canister)
        {12, {-0.379722, -0.557321, 4.05498}},  // cup 3
        {13, {-0.612517, -0.553776, 4.00046}},  // cup 1
        {14, {-0.504227, -0.250771, 4.67401}},  // mustard
    };

    auto& drawables = sceneGraph_->getDrawables();
    for (auto& objPlacement : objectPositions) {
      objectIDs_.push_back(physicsManager_->addObject(
          parsedObjectIds[objPlacement.first], &drawables));
      physicsManager_->setTranslation(objectIDs_.back(), objPlacement.second);
    }
  }
}

}  // namespace

MAGNUM_APPLICATION_MAIN(Viewer)
