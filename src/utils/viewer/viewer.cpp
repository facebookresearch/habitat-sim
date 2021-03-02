// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <fstream>

#include <Magnum/configure.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#ifdef MAGNUM_TARGET_WEBGL
#include <Magnum/Platform/EmscriptenApplication.h>
#else
#include <Magnum/Platform/GlfwApplication.h>
#endif
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Shaders/Shaders.h>
#include <Magnum/Timeline.h>
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneNode.h"

#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/String.h>
#include <Magnum/DebugTools/FrameProfiler.h>
#include <Magnum/DebugTools/Screenshot.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <sophus/so3.hpp>
#include "esp/core/Utility.h"
#include "esp/core/esp.h"
#include "esp/gfx/Drawable.h"
#include "esp/io/io.h"

#include "esp/sensor/CameraSensor.h"
#include "esp/sim/Simulator.h"

#include "ObjectPickingHelper.h"
#include "esp/physics/configure.h"

// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

constexpr float moveSensitivity = 0.07f;
constexpr float lookSensitivity = 0.9f;
constexpr float rgbSensorHeight = 1.5f;
constexpr float agentActionsPerSecond = 60.0f;

struct Openable {
  int articulatedObjectId, linkId, dofIx;
  float dofOpen, dofClosed;

  esp::sim::Simulator* sim_;

  Openable(esp::sim::Simulator* sim,
           int _articulatedObjectId,
           int _linkId,
           int _dofIx,
           float _dofOpen,
           float _dofClosed)
      : articulatedObjectId(_articulatedObjectId),
        linkId(_linkId),
        dofIx(_dofIx),
        dofOpen(_dofOpen),
        dofClosed(_dofClosed),
        sim_(sim) {}

  // copy constructor
  Openable(const Openable&) = default;

  virtual void openClose() {
    std::vector<float> pose =
        sim_->getArticulatedObjectPositions(articulatedObjectId);
    Corrade::Utility::Debug() << "openClose: " << pose;

    if (abs(pose[dofIx] - dofClosed) > 0.1) {
      Corrade::Utility::Debug() << "close = " << dofClosed;
      // count this as open and close it
      pose[dofIx] = dofClosed;
    } else {
      Corrade::Utility::Debug() << "open = " << dofOpen;
      pose[dofIx] = dofOpen;
    }
    sim_->setArticulatedObjectPositions(articulatedObjectId, pose);
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
  esp::sim::Simulator& sim_;
  int objectId;
  int lWheelDof;
  int rWheelDof;

  int lWheelMotorId, rWheelMotorId;

  LocobotController(esp::sim::Simulator& sim,
                    int _articulatedObjectId,
                    int _lWheelDof = 2,
                    int _rWheelDof = 3)
      : sim_(sim), objectId(_articulatedObjectId) {
    Corrade::Utility::Debug() << "LocobotController constructor.";
    dofsToMotorIds = sim_.createMotorsForAllDofs(objectId);
    Corrade::Utility::Debug() << "dofsToMotorIds = " << dofsToMotorIds;
    std::vector<float> pose = sim_.getArticulatedObjectPositions(objectId);
    for (auto id : dofsToMotorIds) {
      sim_.updateJointMotor(objectId, id.second,
                            {pose[id.first], 1.0, 0, 0, 1.0});
    }
    lWheelDof = _lWheelDof;
    rWheelDof = _rWheelDof;
    lWheelMotorId = dofsToMotorIds.at(lWheelDof);
    rWheelMotorId = dofsToMotorIds.at(rWheelDof);
    sim_.updateJointMotor(objectId, lWheelMotorId, {0, 0, 0, 0, 10.0});
    sim_.updateJointMotor(objectId, rWheelMotorId, {0, 0, 0, 0, 10.0});
  }

  ~LocobotController() {
    for (auto id : dofsToMotorIds) {
      sim_.removeJointMotor(objectId, id.second);
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
        sim_.updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, 0, 0, maxImpulse});
        sim_.updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, 0, 0, maxImpulse});
        return;
      } break;
      case FORWARD: {
        sim_.updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, wheelVel * 2, 1.0, maxImpulse});
        sim_.updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, wheelVel * 2, 1.0, maxImpulse});
      } break;
      case BACK: {
        sim_.updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, -wheelVel * 2, 1.0, maxImpulse});
        sim_.updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, -wheelVel * 2, 1.0, maxImpulse});
      } break;
      case LEFT: {
        sim_.updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, -wheelVel, 1.0, maxImpulse});
        sim_.updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, wheelVel, 1.0, maxImpulse});
      } break;
      case RIGHT: {
        sim_.updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, wheelVel, 1.0, maxImpulse});
        sim_.updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, -wheelVel, 1.0, maxImpulse});
      } break;
      case RANDOM: {
        float randL = (float)((rand() % 2000 - 1000) / 1000.0);
        float randR = (float)((rand() % 2000 - 1000) / 1000.0);
        sim_.updateJointMotor(objectId, lWheelMotorId,
                              {0, 0, wheelVel * randL, 1.0, 1.0});
        sim_.updateJointMotor(objectId, rWheelMotorId,
                              {0, 0, wheelVel * randR, 1.0, 1.0});
      } break;

      default:
        break;
    }
  }
};

struct AliengoController {
  std::map<int, int> dofsToMotorIds;
  esp::sim::Simulator& sim_;
  int objectId;

  std::vector<float> initialPose;
  std::vector<float> cyclePose;

  int mode = 0;
  float cycleTime = 0;
  float maxImpulse = 0.45;

  AliengoController(esp::sim::Simulator& sim, int _articulatedObjectId)
      : sim_(sim), objectId(_articulatedObjectId) {
    dofsToMotorIds = sim_.createMotorsForAllDofs(objectId);
    initialPose = sim_.getArticulatedObjectPositions(objectId);
    cyclePose = sim_.getArticulatedObjectPositions(objectId);
  }

  ~AliengoController() {
    for (auto id : dofsToMotorIds) {
      sim_.removeJointMotor(objectId, id.second);
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
        sim_.updateJointMotor(objectId, id.second,
                              {initialPose[id.first], positionGain, 0,
                               maxImpulse / 2.0, maxImpulse});
      } else {
        sim_.updateJointMotor(objectId, id.second,
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
        sim_.updateJointMotor(
            objectId, id.second,
            {cyclePose[id.first], 1.0, 0, maxImpulse / 2.0, maxImpulse});
      }
    }
  }
};

enum MouseInteractionMode {
  LOOK,
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
    case (LOOK):
      return "LOOK";
      break;
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
  esp::sim::Simulator* sim_;

  float gripDepth;

  MouseGrabber(const Magnum::Vector3& clickPos,
               float _gripDepth,
               esp::sim::Simulator* sim) {
    sim_ = sim;
    target = clickPos;
    gripDepth = _gripDepth;
  }

  virtual ~MouseGrabber() { sim_->removeConstraint(p2pId); }

  virtual void updatePivotB(Magnum::Vector3 pos) {
    sim_->updateP2PConstraintPivot(p2pId, pos);
  }
};

struct MouseLinkGrabber : public MouseGrabber {
  int articulatedObjectId, linkId;

  MouseLinkGrabber(const Magnum::Vector3& clickPos,
                   float _gripDepth,
                   int _articulatedObjectId,
                   int _linkId,
                   esp::sim::Simulator* sim)
      : MouseGrabber(clickPos, _gripDepth, sim) {
    articulatedObjectId = _articulatedObjectId;
    linkId = _linkId;
    Corrade::Utility::Debug()
        << "MouseLinkGrabber init: articulatedObjectId=" << articulatedObjectId
        << ", linkId=" << linkId;
    p2pId = sim_->createArticulatedP2PConstraint(articulatedObjectId, linkId,
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
                              esp::sim::Simulator* sim)
      : MouseGrabber(clickPos, _gripDepth, sim) {
    articulatedObjectId = _articulatedObjectId;
    Magnum::Vector3 root =
        sim_->getArticulatedObjectRootState(articulatedObjectId).translation();
    rootClickOffset = root - clickPos;
  }

  virtual ~MouseArticulatedBaseGrabber() override {
    Corrade::Utility::Debug()
        << "~MouseArticulatedBaseGrabber final root pos: "
        << sim_->getArticulatedObjectRootState(articulatedObjectId)
               .translation();
  }

  virtual void updatePivotB(Magnum::Vector3 pos) override {
    Magnum::Matrix4 rootState =
        sim_->getArticulatedObjectRootState(articulatedObjectId);
    rootState.translation() = pos + rootClickOffset;
    Corrade::Utility::Debug() << "newRootState = " << rootState;
    sim_->setArticulatedObjectSleep(articulatedObjectId, false);
    sim_->setArticulatedObjectRootState(articulatedObjectId, rootState);
  }
};

struct MouseObjectGrabber : public MouseGrabber {
  int objectId;

  MouseObjectGrabber(const Magnum::Vector3& clickPos,
                     float _gripDepth,
                     int _objectId,
                     esp::sim::Simulator* sim)
      : MouseGrabber(clickPos, _gripDepth, sim) {
    objectId = _objectId;
    sim_->setObjectMotionType(esp::physics::MotionType::DYNAMIC, objectId);
    Corrade::Utility::Debug()
        << "MouseObjectGrabber init: objectId=" << objectId;
    p2pId = sim_->createRigidP2PConstraint(objectId, clickPos, false);
  }
};

struct MouseObjectKinematicGrabber : public MouseGrabber {
  int objectId;
  Magnum::Vector3 clickOffset;
  MouseObjectKinematicGrabber(const Magnum::Vector3& clickPos,
                              float _gripDepth,
                              int _objectId,
                              esp::sim::Simulator* sim)
      : MouseGrabber(clickPos, _gripDepth, sim) {
    objectId = _objectId;
    Magnum::Vector3 origin = sim_->getTranslation(objectId);
    clickOffset = origin - clickPos;
    sim_->setObjectMotionType(esp::physics::MotionType::KINEMATIC, objectId);
  }

  virtual ~MouseObjectKinematicGrabber() override {
    Corrade::Utility::Debug()
        << "~MouseObjectKinematicGrabber final origin pos: "
        << sim_->getTranslation(objectId);
  }

  virtual void updatePivotB(Magnum::Vector3 pos) override {
    Magnum::Vector3 objectOrigin = sim_->getTranslation(objectId);
    sim_->setTranslation(clickOffset + pos, objectId);
  }
};

namespace {

//! return current time as string in format
//! "year_month_day_hour-minutes-seconds"
std::string getCurrentTimeString() {
  time_t now = time(0);
  tm* ltm = localtime(&now);
  return std::to_string(1900 + ltm->tm_year) + "_" +
         std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) +
         "_" + std::to_string(ltm->tm_hour) + "-" +
         std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec);
}

using namespace Mn::Math::Literals;
using Magnum::Math::Literals::operator""_degf;

class Viewer : public Mn::Platform::Application {
 public:
  explicit Viewer(const Arguments& arguments);

 private:
  // Keys for moving/looking are recorded according to whether they are
  // currently being pressed
  std::map<KeyEvent::Key, bool> keysPressed = {
      {KeyEvent::Key::Left, false}, {KeyEvent::Key::Right, false},
      {KeyEvent::Key::Up, false},   {KeyEvent::Key::Down, false},
      {KeyEvent::Key::A, false},    {KeyEvent::Key::D, false},
      {KeyEvent::Key::S, false},    {KeyEvent::Key::W, false},
      {KeyEvent::Key::X, false},    {KeyEvent::Key::Z, false}};

  void drawEvent() override;
  void viewportEvent(ViewportEvent& event) override;
  void mousePressEvent(MouseEvent& event) override;
  void mouseReleaseEvent(MouseEvent& event) override;
  void mouseMoveEvent(MouseMoveEvent& event) override;
  void mouseScrollEvent(MouseScrollEvent& event) override;
  void keyPressEvent(KeyEvent& event) override;
  void keyReleaseEvent(KeyEvent& event) override;
  void moveAndLook(int repetitions);

  //! id of render-only object used to visualize the click location
  int clickVisObjectID_ = esp::ID_UNDEFINED;

  std::unique_ptr<MouseGrabber> mouseGrabber_ = nullptr;

  MouseInteractionMode mouseInteractionMode = LOOK;

  std::vector<Openable> openableObjects;
  std::vector<std::unique_ptr<LocobotController>> locobotControllers;
  std::vector<std::unique_ptr<AliengoController>> aliengoControllers;

  int throwSphere(Magnum::Vector3 direction);

  /**
   * @brief Instance an object from an ObjectAttributes.
   * @param configHandle The handle referencing the object's template in the
   * ObjectAttributesManager.
   * @return The newly allocated object's ID for referencing it through the
   * Simulator API.
   */
  int addObject(const std::string& configHandle);

  /**
   * @brief Instance an object from an ObjectAttributes.
   * @param objID The unique ID referencing the object's template in the
   * ObjectAttributesManager.
   * @return The newly allocated object's ID for referencing it through the
   * Simulator API.
   */
  int addObject(int objID);

  /**
   * @brief Instance a random object based on an imported asset if one exists.
   * @return The newly allocated object's ID for referencing it through the
   * Simulator API.
   */
  int addTemplateObject();

  /**
   * @brief Instance a random object based on a primitive shape.
   * @return The newly allocated object's ID for referencing it through the
   * Simulator API.
   */
  int addPrimitiveObject();

  int addArticulatedObject(std::string urdfFilename,
                           bool fixedBase = false,
                           float globalScale = 1.0);

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
  void invertGravity();
  /**
   * @brief Toggle between ortho and perspective camera
   */
  void switchCameraType();
  Mn::Vector3 randomDirection();

  void saveCameraTransformToFile();
  void saveNodeTransformToFile(esp::scene::SceneNode& node,
                               const std::string& filename);
  void loadCameraTransformFromFile();
  void loadNodeTransformFromFile(esp::scene::SceneNode& node,
                                 const std::string& filename);

  //! string rep of time when viewer application was started
  std::string viewerStartTimeString = getCurrentTimeString();
  void screenshot();

  esp::sensor::CameraSensor& getAgentCamera() {
    esp::sensor::Sensor& cameraSensor =
        *defaultAgent_->getSensorSuite().getSensors()["rgba_camera"];
    return static_cast<esp::sensor::CameraSensor&>(cameraSensor);
  }

  std::string helpText = R"(
==================================================
Welcome to the Habitat-sim C++ Viewer application!
==================================================
Mouse Functions:
----------------
  LEFT:
    Click and drag to rotate the agent and look up/down.
  RIGHT:
    (With 'enable-physics') Click a surface to instance a random primitive object at that location.
  SHIFT-RIGHT:
    Click a mesh to highlight it.
  WHEEL:
    Modify orthographic camera zoom/perspective camera FOV (+SHIFT for fine grained control)

Key Commands:
-------------
  esc: Exit the application.
  'H': Display this help message.

  Agent Controls:
  'wasd': Move the agent's body forward/backward, left/right.
  'zx': Move the agent's body up/down.
  arrow keys: Turn the agent's body left/right and camera look up/down.
  '9': Randomly place agent on NavMesh (if loaded).
  'q': Query the agent's state and print to terminal.

  Utilities:
  '2' switch ortho/perspective camera.
  'e' enable/disable frustum culling.
  'c' show/hide FPS overlay.
  'n' show/hide NavMesh wireframe.
  'i' Save a screenshot to "./screenshots/year_month_day_hour-minute-second/#.png"
  'r' Write a replay of the recent simulated frames to a file specified by --gfx-replay-record-filepath.
  '[' save camera position/orientation to "./saved_transformations/camera.year_month_day_hour-minute-second.txt"
  ']' load camera position/orientation from file system (useful when flying camera mode is enabled), or else from last save in current instance

  Object Interactions:
  SPACE: Toggle physics simulation on/off
  '.': Take a single simulation step if not simulating continuously.
  '8': Instance a random primitive object in front of the agent.
  'o': Instance a random file-based object in front of the agent.
  'u': Remove most recently instanced object.
  'b': Toggle display of object bounding boxes.
  'k': Kinematically wiggle the most recently added object.
  'p': (physics) Poke the most recently added object.
  'f': (physics) Push the most recently added object.
  't': (physics) Torque the most recently added object.
  'v': (physics) Invert gravity.
  ==================================================
  )";

  //! Print viewer help text to terminal output.
  void printHelpText() { Mn::Debug{} << helpText; };

  // single inline for logging agent state msgs, so can be easily modified
  inline void showAgentStateMsg(bool showPos, bool showOrient) {
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

  /**
   * @brief vector holding past agent locations to build trajectory
   * visualization
   */
  std::vector<Magnum::Vector3> agentLocs_;
  float agentTrajRad_ = .01f;
  bool agentLocRecordOn_ = false;

  /**
   * @brief Set whether agent locations should be recorded or not. If toggling
   * on then clear old locations
   */
  inline void setAgentLocationRecord(bool enable) {
    if (enable == !agentLocRecordOn_) {
      agentLocRecordOn_ = enable;
      if (enable) {  // if turning on, clear old data
        agentLocs_.clear();
        recAgentLocation();
      }
    }
    LOG(INFO) << "Agent location recording "
              << (agentLocRecordOn_ ? "on" : "off");
  }  // setAgentLocationRecord

  /**
   * @brief Record agent location if enabled.  Called after any movement.
   */
  inline void recAgentLocation() {
    if (agentLocRecordOn_) {
      auto pt = agentBodyNode_->translation() +
                Magnum::Vector3{0, (2.0f * agentTrajRad_), 0};
      agentLocs_.push_back(pt);
      LOG(INFO) << "Recording agent location : {" << pt.x() << "," << pt.y()
                << "," << pt.z() << "}";
    }
  }

  /**
   * @brief Build trajectory visualization
   */
  void buildTrajectoryVis();
  void modTrajRad(bool bigger) {
    std::string mod = "";
    if (bigger) {
      if (agentTrajRad_ < 1.0) {
        agentTrajRad_ += 0.001f;
        mod = "increased to ";
      }
    } else {
      if (agentTrajRad_ > 0.001f) {
        agentTrajRad_ -= 0.001f;
        mod = "decreased to ";
      }
    }
    esp::geo::clamp(agentTrajRad_, 0.001f, 1.0f);
    LOG(INFO) << "Agent Trajectory Radius " << mod << ": " << agentTrajRad_;
  }

  // The simulator object backend for this viewer instance
  std::unique_ptr<esp::sim::Simulator> simulator_;

  // Toggle physics simulation on/off
  bool simulating_ = true;

  // Toggle a single simulation step at the next opportunity if not simulating
  // continuously.
  bool simulateSingleStep_ = false;

  // The managers belonging to the simulator
  std::shared_ptr<esp::metadata::managers::ObjectAttributesManager>
      objectAttrManager_ = nullptr;
  std::shared_ptr<esp::metadata::managers::AssetAttributesManager>
      assetAttrManager_ = nullptr;
  std::shared_ptr<esp::metadata::managers::StageAttributesManager>
      stageAttrManager_ = nullptr;
  std::shared_ptr<esp::metadata::managers::PhysicsAttributesManager>
      physAttrManager_ = nullptr;

  bool debugBullet_ = false;

  esp::scene::SceneNode* agentBodyNode_ = nullptr;

  const int defaultAgentId_ = 0;
  esp::agent::Agent::ptr defaultAgent_ = nullptr;

  // Scene or stage file to load
  std::string sceneFileName;
  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::scene::SceneGraph* activeSceneGraph_ = nullptr;
  bool drawObjectBBs = false;
  std::string gfxReplayRecordFilepath_;

  std::string cameraLoadPath_;
  // bool cameraSaved_ = false;
  // Mn::Matrix4 lastCameraSave_;
  Cr::Containers::Optional<Mn::Matrix4> lastCameraSave_;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};
  bool showFPS_ = true;
  bool flyingCameraMode_ = false;

  // NOTE: Mouse + shift is to select object on the screen!!
  void createPickedObjectVisualizer(unsigned int objectId);
  std::unique_ptr<ObjectPickingHelper> objectPickingHelper_;
  // returns the number of visible drawables (meshVisualizer drawables are not
  // included)

  Mn::DebugTools::GLFrameProfiler profiler_{};
};

Viewer::Viewer(const Arguments& arguments)
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
      .setHelp("scene", "scene/stage file to load")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp("Displays a 3D scene file provided on command line")
      .addOption("dataset", "default")
      .setHelp("dataset", "dataset configuration file to use")
      .addBooleanOption("enable-physics")
      .addBooleanOption("stage-requires-lighting")
      .setHelp("stage-requires-lighting",
               "Stage asset should be lit with Phong shading.")
      .addBooleanOption("debug-bullet")
      .setHelp("debug-bullet", "Render Bullet physics debug wireframes.")
      .addOption("gfx-replay-record-filepath")
      .setHelp("gfx-replay-record-filepath",
               "Enable replay recording with R key.")
      .addOption("physics-config", ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH)
      .setHelp("physics-config",
               "Provide a non-default PhysicsManager config file.")
      .addOption("object-dir", "./data/objects")
      .setHelp("object-dir",
               "Provide a directory to search for object config files "
               "(relative to habitat-sim directory).")
      .addBooleanOption("orthographic")
      .setHelp("orthographic",
               "If specified, use orthographic camera to view scene.")
      .addBooleanOption("disable-navmesh")
      .setHelp("disable-navmesh",
               "Disable the navmesh, disabling agent navigation constraints.")
      .addOption("navmesh-file")
      .setHelp("navmesh-file", "Manual override path to scene navmesh file.")
      .addBooleanOption("recompute-navmesh")
      .setHelp("recompute-navmesh",
               "Programmatically re-generate the scene navmesh.")
      .addOption("camera-transform-filepath")
      .setHelp("camera-transform-filepath",
               "Specify path to load camera transform from.")
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

  cameraLoadPath_ = args.value("camera-transform-filepath");
  gfxReplayRecordFilepath_ = args.value("gfx-replay-record-filepath");

  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.activeSceneName = sceneFileName;
  simConfig.sceneDatasetConfigFile = args.value("dataset");
  LOG(INFO) << "Dataset : " << simConfig.sceneDatasetConfigFile;
  simConfig.enablePhysics = useBullet;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;
  simConfig.enableGfxReplaySave = !gfxReplayRecordFilepath_.empty();
  if (args.isSet("stage-requires-lighting")) {
    Mn::Debug{} << "Stage using DEFAULT_LIGHTING_KEY";
    simConfig.sceneLightSetup = esp::DEFAULT_LIGHTING_KEY;
  }

  // setup the PhysicsManager config file
  std::string physicsConfig = Cr::Utility::Directory::join(
      Corrade::Utility::Directory::current(), args.value("physics-config"));
  if (Cr::Utility::Directory::exists(physicsConfig)) {
    Mn::Debug{} << "Using PhysicsManager config: " << physicsConfig;
    simConfig.physicsConfigFile = physicsConfig;
  }

  simulator_ = esp::sim::Simulator::create_unique(simConfig);

  objectAttrManager_ = simulator_->getObjectAttributesManager();
  objectAttrManager_->loadAllConfigsFromPath(Cr::Utility::Directory::join(
      Corrade::Utility::Directory::current(), args.value("object-dir")));
  assetAttrManager_ = simulator_->getAssetAttributesManager();
  stageAttrManager_ = simulator_->getStageAttributesManager();
  physAttrManager_ = simulator_->getPhysicsAttributesManager();

  // NavMesh customization options
  if (args.isSet("disable-navmesh")) {
    if (simulator_->getPathFinder()->isLoaded()) {
      simulator_->setPathFinder(esp::nav::PathFinder::create());
    }
  } else if (args.isSet("recompute-navmesh")) {
    esp::nav::NavMeshSettings navMeshSettings;
    simulator_->recomputeNavMesh(*simulator_->getPathFinder().get(),
                                 navMeshSettings, true);
  } else if (!args.value("navmesh-file").empty()) {
    std::string navmeshFile = Cr::Utility::Directory::join(
        Corrade::Utility::Directory::current(), args.value("navmesh-file"));
    if (Cr::Utility::Directory::exists(navmeshFile)) {
      simulator_->getPathFinder()->loadNavMesh(navmeshFile);
    }
  }

  // configure and initialize default Agent and Sensor
  auto agentConfig = esp::agent::AgentConfiguration();
  agentConfig.height = rgbSensorHeight;
  agentConfig.actionSpace = {
      // setup viewer action space
      {"moveForward",
       esp::agent::ActionSpec::create(
           "moveForward",
           esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveBackward",
       esp::agent::ActionSpec::create(
           "moveBackward",
           esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveLeft",
       esp::agent::ActionSpec::create(
           "moveLeft", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveRight",
       esp::agent::ActionSpec::create(
           "moveRight", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveDown",
       esp::agent::ActionSpec::create(
           "moveDown", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveUp",
       esp::agent::ActionSpec::create(
           "moveUp", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"turnLeft",
       esp::agent::ActionSpec::create(
           "turnLeft", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
      {"turnRight",
       esp::agent::ActionSpec::create(
           "turnRight", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
      {"lookUp",
       esp::agent::ActionSpec::create(
           "lookUp", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
      {"lookDown",
       esp::agent::ActionSpec::create(
           "lookDown", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
  };
  auto cameraSensorSpec = esp::sensor::CameraSensorSpec::create();
  cameraSensorSpec->sensorSubType =
      args.isSet("orthographic") ? esp::sensor::SensorSubType::Orthographic
                                 : esp::sensor::SensorSubType::Pinhole;
  cameraSensorSpec->sensorType = esp::sensor::SensorType::Color;
  cameraSensorSpec->position = {0.0f, 1.5f, 0.0f};
  cameraSensorSpec->orientation = {0, 0, 0};
  cameraSensorSpec->resolution = esp::vec2i(viewportSize[1], viewportSize[0]);
  agentConfig.sensorSpecifications = {cameraSensorSpec};

  simulator_->addAgent(agentConfig);

  // Set up camera
  activeSceneGraph_ = &simulator_->getActiveSceneGraph();
  defaultAgent_ = simulator_->getAgent(defaultAgentId_);
  agentBodyNode_ = &defaultAgent_->node();
  renderCamera_ = getAgentCamera().getRenderCamera();

  objectPickingHelper_ = std::make_unique<ObjectPickingHelper>(viewportSize);
  timeline_.start();

  /**
   * Set up per frame profiler to be aware of bottlenecking in processing data
   * Interpretation: CpuDuration should be less than GpuDuration to avoid GPU
   * idling, and CpuDuration and GpuDuration should be roughly equal for faster
   * rendering times
   *
   * FrameTime: (Units::Nanoseconds) Time to render per frame, 1/FPS
   *
   * CpuDuration: (Units::Nanoseconds) CPU time spent processing events,
   * physics, traversing SceneGraph, and submitting data to GPU/drivers per
   * frame
   *
   * GpuDuration: (Units::Nanoseconds) Measures how much time it takes for the
   * GPU to process all work submitted by CPU Uses asynchronous querying to
   * measure the amount of time to fully complete a set of GL commands without
   * stalling rendering
   */
  Mn::DebugTools::GLFrameProfiler::Values profilerValues =
      Mn::DebugTools::GLFrameProfiler::Value::FrameTime |
      Mn::DebugTools::GLFrameProfiler::Value::CpuDuration |
      Mn::DebugTools::GLFrameProfiler::Value::GpuDuration;

// VertexFetchRatio and PrimitiveClipRatio only supported for GL 4.6
#ifndef MAGNUM_TARGET_GLES
  if (Mn::GL::Context::current()
          .isExtensionSupported<
              Mn::GL::Extensions::ARB::pipeline_statistics_query>()) {
    profilerValues |=
        Mn::DebugTools::GLFrameProfiler::Value::
            VertexFetchRatio |  // Ratio of vertex shader invocations to count
                                // of vertices submitted
        Mn::DebugTools::GLFrameProfiler::Value::
            PrimitiveClipRatio;  // Ratio of primitives discarded by the
                                 // clipping stage to count of primitives
                                 // submitted
  }
#endif

  // Per frame profiler will average measurements taken over previous 50 frames
  profiler_.setup(profilerValues, 50);

  printHelpText();
}  // end Viewer::Viewer

void Viewer::switchCameraType() {
  auto& cam = getAgentCamera();

  auto oldCameraType = cam.getCameraType();
  switch (oldCameraType) {
    case esp::sensor::SensorSubType::Pinhole: {
      cam.setCameraType(esp::sensor::SensorSubType::Orthographic);
      return;
    }
    case esp::sensor::SensorSubType::Orthographic: {
      cam.setCameraType(esp::sensor::SensorSubType::Pinhole);
      return;
    }
    case esp::sensor::SensorSubType::None: {
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      return;
    }
  }
}

void Viewer::saveCameraTransformToFile() {
  const char* saved_transformations_directory = "saved_transformations/";
  if (!Cr::Utility::Directory::exists(saved_transformations_directory)) {
    Cr::Utility::Directory::mkpath(saved_transformations_directory);
  }

  // update temporary save
  lastCameraSave_ = renderCamera_->node().absoluteTransformation();

  // update save in file system
  saveNodeTransformToFile(
      renderCamera_->node(),
      Cr::Utility::formatString("{}camera.{}.txt",
                                saved_transformations_directory,
                                getCurrentTimeString()));
}

void Viewer::loadCameraTransformFromFile() {
  if (!cameraLoadPath_.empty()) {
    // loading from file system
    loadNodeTransformFromFile(renderCamera_->node(), cameraLoadPath_);
  } else {
    // attempting to load from last temporary save
    LOG(WARNING)
        << "Camera transform file not specified, attempting to load from "
           "current instance. Use --camera-transform-filepath to specify file "
           "to load from.";
    if (!lastCameraSave_) {
      LOG(ERROR) << "No transformation saved in current instance.";
      return;
    }

    renderCamera_->node().setTransformation(*lastCameraSave_);
    LOG(INFO) << "Transformation matrix loaded from current instance : "
              << Eigen::Map<esp::mat4f>(lastCameraSave_->data());
  }

  if (!flyingCameraMode_) {
    LOG(INFO) << "Note: The flying camera mode is currently OFF. You may not "
                 "see any view change.";
  }
}

void Viewer::saveNodeTransformToFile(esp::scene::SceneNode& node,
                                     const std::string& filename) {
  std::ofstream file(filename);
  if (!file.good()) {
    LOG(ERROR) << "Cannot open " << filename << " to output data.";
    return;
  }

  // saving transformation into file system
  Mn::Matrix4 transform = node.absoluteTransformation();
  float* t = transform.data();
  for (int i = 0; i < 16; ++i) {
    file << t[i] << " ";
  }
  file.close();

  LOG(INFO) << "Transformation matrix saved to " << filename << " : "
            << Eigen::Map<esp::mat4f>(transform.data());
}

void Viewer::loadNodeTransformFromFile(esp::scene::SceneNode& node,
                                       const std::string& filename) {
  std::ifstream file(filename);
  if (!file.good()) {
    LOG(ERROR) << "Cannot open " << filename << " to load data.";
    return;
  }

  // reading file system data into matrix as transformation
  Mn::Vector4 cols[4];
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      file >> cols[col][row];
    }
  }
  Mn::Matrix4 transform{cols[0], cols[1], cols[2], cols[3]};
  file.close();

  // checking for corruption
  if (!transform.isRigidTransformation()) {
    LOG(WARNING) << "Data loaded from " << filename
                 << " is not a valid transformation.";
    return;
  }

  node.setTransformation(transform);
  LOG(INFO) << "Transformation matrix loaded from " << filename << " : "
            << Eigen::Map<esp::mat4f>(transform.data());
}

int Viewer::addObject(int ID) {
  const std::string& configHandle =
      simulator_->getObjectAttributesManager()->getObjectHandleByID(ID);
  return addObject(configHandle);
}  // addObject

int Viewer::addObject(const std::string& objectAttrHandle) {
  // Relative to agent bodynode
  Mn::Matrix4 T = agentBodyNode_->MagnumObject::transformationMatrix();
  Mn::Vector3 new_pos = T.transformPoint({0.1f, 1.5f, -2.0f});

  int physObjectID = simulator_->addObjectByHandle(objectAttrHandle);
  simulator_->setTranslation(new_pos, physObjectID);
  simulator_->setRotation(Mn::Quaternion::fromMatrix(T.rotationNormalized()),
                          physObjectID);
  return physObjectID;
}  // addObject

// add file-based template derived object from keypress
int Viewer::addTemplateObject() {
  int numObjTemplates = objectAttrManager_->getNumFileTemplateObjects();
  if (numObjTemplates > 0) {
    return addObject(objectAttrManager_->getRandomFileTemplateHandle());
  } else {
    LOG(WARNING) << "No objects loaded, can't add any";
    return esp::ID_UNDEFINED;
  }
}  // addTemplateObject

// add synthesized primiitive object from keypress
int Viewer::addPrimitiveObject() {
  // TODO : use this to implement synthesizing rendered physical objects

  int numObjPrims = objectAttrManager_->getNumSynthTemplateObjects();
  if (numObjPrims > 0) {
    return addObject(objectAttrManager_->getRandomSynthTemplateHandle());
  } else {
    LOG(WARNING) << "No primitive templates available, can't add any objects";
    return esp::ID_UNDEFINED;
  }
}  // addPrimitiveObject

int Viewer::throwSphere(Mn::Vector3 direction) {
  if (simulator_->getPhysicsSimulationLibrary() ==
      esp::physics::PhysicsManager::NONE) {
    return esp::ID_UNDEFINED;
  }

  Mn::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Mn::Vector3 new_pos = T.transformPoint({0.0f, 1.5f, -0.5f});

  auto icoSphere = esp::metadata::PrimObjTypes::ICOSPHERE_SOLID;
  auto sphereAttributes = assetAttrManager_->createObject(icoSphere);
  int sphereObjectTemplateId = objectAttrManager_
                                   ->createPrimBasedAttributesTemplate(
                                       sphereAttributes->getHandle(), true)
                                   ->getID();

  int physObjectID = simulator_->addObject(sphereObjectTemplateId);
  simulator_->setTranslation(new_pos, physObjectID);

  // throw the object
  Mn::Vector3 impulse = direction;
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);
  simulator_->applyImpulse(impulse, rel_pos, physObjectID);
  return physObjectID;
}

int Viewer::addArticulatedObject(std::string urdfFilename,
                                 bool fixedBase,
                                 float globalScale) {
  int articulatedObjectId = simulator_->addArticulatedObjectFromURDF(
      urdfFilename, fixedBase, globalScale);
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
  simulator_->setArticulatedObjectRootState(objectId,
                                            initialArticulatedObjectTransform);
}

void Viewer::toggleArticulatedMotionType(int objectId) {
  if (simulator_->getArticulatedObjectMotionType(objectId) ==
      esp::physics::MotionType::DYNAMIC) {
    simulator_->setArticulatedObjectMotionType(
        objectId, esp::physics::MotionType::KINEMATIC);
    Corrade::Utility::Debug() << "setting MotionType::KINEMATIC";
  } else {
    simulator_->setArticulatedObjectMotionType(
        objectId, esp::physics::MotionType::DYNAMIC);
    Corrade::Utility::Debug() << "setting MotionType::DYNAMIC";
  }
}
void Viewer::buildTrajectoryVis() {
  if (agentLocs_.size() < 2) {
    LOG(WARNING) << "Viewer::buildTrajectoryVis : No recorded trajectory "
                    "points, so nothing to build. Aborting.";
    return;
  }
  Mn::Color4 color{randomDirection(), 1.0f};
  // synthesize a name for asset based on color, radius, point count
  std::ostringstream tmpName;
  tmpName << "viewerTrajVis_R" << color.r() << "_G" << color.g() << "_B"
          << color.b() << "_rad" << agentLocs_.size() << "_"
          << agentLocs_.size() << "_pts";
  std::string trajObjName(tmpName.str());

  LOG(INFO) << "Viewer::buildTrajectoryVis : Attempting to build trajectory "
               "tube for :"
            << agentLocs_.size() << " points.";
  int trajObjID = simulator_->addTrajectoryObject(
      trajObjName, agentLocs_, 6, agentTrajRad_, color, true, 10);
  if (trajObjID != esp::ID_UNDEFINED) {
    LOG(INFO) << "Viewer::buildTrajectoryVis : Success!  Traj Obj Name : "
              << trajObjName << " has object ID : " << trajObjID;
  } else {
    LOG(WARNING) << "Viewer::buildTrajectoryVis : Attempt to build trajectory "
                    "visualization "
                 << trajObjName << " failed; Returned ID_UNDEFINED.";
  }
}  // buildTrajectoryVis

void Viewer::removeLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0) {
    return;
  }
  if (clickVisObjectID_ == existingObjectIDs.back()) {
    clickVisObjectID_ = esp::ID_UNDEFINED;
  }
  simulator_->removeObject(existingObjectIDs.back());
}

void Viewer::clearAllObjects() {
  locobotControllers.clear();
  aliengoControllers.clear();
  for (auto id : simulator_->getExistingObjectIDs()) {
    simulator_->removeObject(id);
  }
  for (auto id : simulator_->getExistingArticulatedObjectIDs()) {
    simulator_->removeArticulatedObject(id);
  }
  openableObjects.clear();
  clickVisObjectID_ = esp::ID_UNDEFINED;
}

void Viewer::invertGravity() {
  const Mn::Vector3& gravity = simulator_->getGravity();
  const Mn::Vector3 invGravity = -1 * gravity;
  simulator_->setGravity(invGravity);
}

void Viewer::pokeLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0)
    return;
  Mn::Matrix4 T =
      agentBodyNode_->MagnumObject::transformationMatrix();  // Relative to
                                                             // agent bodynode
  Mn::Vector3 impulse = T.transformVector({0.0f, 0.0f, -3.0f});
  Mn::Vector3 rel_pos = Mn::Vector3(0.0f, 0.0f, 0.0f);

  simulator_->applyImpulse(impulse, rel_pos, existingObjectIDs.back());
}

void Viewer::pushLastObject() {
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

void Viewer::torqueLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0)
    return;
  Mn::Vector3 torque = randomDirection() * 30;
  simulator_->applyTorque(torque, existingObjectIDs.back());
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
void Viewer::drawEvent() {
  // Wrap profiler measurements around all methods to render images from
  // RenderCamera
  profiler_.beginFrame();

  Mn::GL::defaultFramebuffer.clear(Mn::GL::FramebufferClear::Color |
                                   Mn::GL::FramebufferClear::Depth);

  // Agent actions should occur at a fixed rate per second
  timeSinceLastSimulation += timeline_.previousFrameDuration();
  int numAgentActions = timeSinceLastSimulation * agentActionsPerSecond;
  moveAndLook(numAgentActions);

  // occasionally a frame will pass quicker than 1/60 seconds
  if (timeSinceLastSimulation >= 1.0 / 60.0) {
    if (simulating_ || simulateSingleStep_) {
      // step physics at a fixed rate
      // In the interest of frame rate, only a single step is taken,
      // even if timeSinceLastSimulation is quite large
      for (auto& aliengoController : aliengoControllers) {
        aliengoController->cycleUpdate(1.0 / 60.0);
      }
      simulator_->stepWorld(1.0 / 60.0);
      simulateSingleStep_ = false;
      const auto recorder = simulator_->getGfxReplayManager()->getRecorder();
      if (recorder) {
        recorder->saveKeyframe();
      }
    }
    // reset timeSinceLastSimulation, accounting for potential overflow
    timeSinceLastSimulation = fmod(timeSinceLastSimulation, 1.0 / 60.0);
  }
  uint32_t visibles = 0;
  if (flyingCameraMode_) {
    Mn::GL::defaultFramebuffer.bind();
    for (auto& it : activeSceneGraph_->getDrawableGroups()) {
      esp::gfx::RenderCamera::Flags flags =
          esp::gfx::RenderCamera::Flag::FrustumCulling;
      visibles += renderCamera_->draw(it.second, flags);
    }
  } else {
    // using polygon offset to increase mesh depth to a avoid z-fighting with
    // debug draw (since lines will not respond to offset).
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::PolygonOffsetFill);
    Mn::GL::Renderer::setPolygonOffset(1.0f, 0.1f);

    // ONLY draw the content to the frame buffer but not immediately blit the
    // result to the default main buffer
    // (this is the reason we do not call displayObservation)
    simulator_->drawObservation(defaultAgentId_, "rgba_camera");
    // TODO: enable other sensors to be displayed

    Mn::GL::Renderer::setDepthFunction(
        Mn::GL::Renderer::DepthFunction::LessOrEqual);
    if (debugBullet_) {
      Mn::Matrix4 camM(renderCamera_->cameraMatrix());
      Mn::Matrix4 projM(renderCamera_->projectionMatrix());

      simulator_->physicsDebugDraw(projM * camM);
    }
    Mn::GL::Renderer::setDepthFunction(Mn::GL::Renderer::DepthFunction::Less);
    Mn::GL::Renderer::setPolygonOffset(0.0f, 0.0f);
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::PolygonOffsetFill);

    visibles = renderCamera_->getPreviousNumVisibleDrawables();
    esp::gfx::RenderTarget* sensorRenderTarget =
        simulator_->getRenderTarget(defaultAgentId_, "rgba_camera");
    CORRADE_ASSERT(sensorRenderTarget,
                   "Error in Viewer::drawEvent: sensor's rendering target "
                   "cannot be nullptr.", );
    if (objectPickingHelper_->isObjectPicked()) {
      // we need to immediately draw picked object to the SAME frame buffer
      // so bind it first
      // bind the framebuffer
      sensorRenderTarget->renderReEnter();

      // setup blending function
      Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);

      // render the picked object on top of the existing contents
      esp::gfx::RenderCamera::Flags flags;
      if (simulator_->isFrustumCullingEnabled()) {
        flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
      }
      renderCamera_->draw(objectPickingHelper_->getDrawables(), flags);
      Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
    }
    sensorRenderTarget->blitRgbaToDefault();
    // Immediately bind the main buffer back so that the "imgui" below can work
    // properly
    Mn::GL::defaultFramebuffer.bind();
  }

  // Do not include ImGui content drawing in per frame profiler measurements
  profiler_.endFrame();

  // Immediately bind the main buffer back so that the "imgui" below can work
  // properly
  Mn::GL::defaultFramebuffer.bind();

  imgui_.newFrame();
  if (showFPS_) {
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::Begin("main", NULL,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(1.5);
    ImGui::Text("%.1f FPS", Mn::Double(ImGui::GetIO().Framerate));
    uint32_t total = activeSceneGraph_->getDrawables().size();
    ImGui::Text("%u drawables", total);
    ImGui::Text("%u culled", total - visibles);
    auto& cam = getAgentCamera();
    ImGui::Text("%s camera",
                (cam.getCameraType() == esp::sensor::SensorSubType::Orthographic
                     ? "Orthographic"
                     : "Pinhole"));
    ImGui::Text("%s", profiler_.statistics().c_str());
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

void Viewer::moveAndLook(int repetitions) {
  for (int i = 0; i < repetitions; i++) {
    if (keysPressed[KeyEvent::Key::Left]) {
      defaultAgent_->act("turnLeft");
    }
    if (keysPressed[KeyEvent::Key::Right]) {
      defaultAgent_->act("turnRight");
    }
    if (keysPressed[KeyEvent::Key::Up]) {
      defaultAgent_->act("lookUp");
    }
    if (keysPressed[KeyEvent::Key::Down]) {
      defaultAgent_->act("lookDown");
    }

    bool moved = false;
    if (keysPressed[KeyEvent::Key::A]) {
      defaultAgent_->act("moveLeft");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::D]) {
      defaultAgent_->act("moveRight");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::S]) {
      defaultAgent_->act("moveBackward");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::W]) {
      defaultAgent_->act("moveForward");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::X]) {
      defaultAgent_->act("moveDown");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::Z]) {
      defaultAgent_->act("moveUp");
      moved = true;
    }

    if (moved) {
      recAgentLocation();
    }
  }
}

void Viewer::viewportEvent(ViewportEvent& event) {
  auto& sensors = defaultAgent_->getSensorSuite();
  for (auto entry : sensors.getSensors()) {
    auto visualSensor =
        dynamic_cast<esp::sensor::VisualSensor*>(entry.second.get());
    if (visualSensor != nullptr) {
      visualSensor->setResolution(event.framebufferSize()[1],
                                  event.framebufferSize()[0]);
      renderCamera_->setViewport(visualSensor->framebufferSize());
      simulator_->getRenderer()->bindRenderTarget(*visualSensor);
    }
  }
  Mn::GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

  if (flyingCameraMode_) {
    renderCamera_->setViewport(event.framebufferSize());
  }

  imgui_.relayout(Mn::Vector2{event.windowSize()} / event.dpiScaling(),
                  event.windowSize(), event.framebufferSize());

  objectPickingHelper_->handleViewportChange(event.framebufferSize());
}

void Viewer::createPickedObjectVisualizer(unsigned int objectId) {
  for (auto& it : activeSceneGraph_->getDrawableGroups()) {
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
    // also, setup the color attachment for rendering, and remove the
    // visualizer for the previously picked object
    objectPickingHelper_->prepareToDraw();

    // redraw the scene on the object picking framebuffer
    esp::gfx::RenderCamera::Flags flags =
        esp::gfx::RenderCamera::Flag::UseDrawableIdAsObjectId;
    if (simulator_->isFrustumCullingEnabled())
      flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
    for (auto& it : activeSceneGraph_->getDrawableGroups()) {
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

    esp::physics::RaycastResults raycastResults = simulator_->castRay(ray);

    // create the clickNode primitive if not present
    if (clickVisObjectID_ == esp::ID_UNDEFINED) {
      auto boxTemplateHandle =
          objectAttrManager_->getSynthTemplateHandlesBySubstring(
              "cubeWireframe")[0];
      auto boxTemplate =
          objectAttrManager_->getObjectByHandle(boxTemplateHandle);
      boxTemplate->setScale({0.1, 0.1, 0.1});
      boxTemplate->setIsCollidable(false);
      objectAttrManager_->registerObject(boxTemplate, "click_vis");
      clickVisObjectID_ = simulator_->addObjectByHandle("click_vis");
      simulator_->setObjectMotionType(esp::physics::MotionType::KINEMATIC,
                                      clickVisObjectID_);
    }

    // update click visualization
    simulator_->setTranslation(
        renderCamera_->node().absoluteTranslation() + ray.direction,
        clickVisObjectID_);

    // ray hit a collision object, visualize this and determine the hit object
    if (raycastResults.hasHits()) {
      auto hitInfo = raycastResults.hits[0];  // first hit

      // update click visualization
      simulator_->setTranslation(hitInfo.point, clickVisObjectID_);

      if (hitInfo.objectId != esp::ID_UNDEFINED) {
        // we hit an non-stage collision object

        bool hitArticulatedObject = false;
        // TODO: determine if this is true (link id?)
        int hitArticulatedObjectId = esp::ID_UNDEFINED;
        int hitArticulatedLinkIndex = esp::ID_UNDEFINED;
        // TODO: get this info from link?
        for (auto aoId : simulator_->getExistingArticulatedObjectIDs()) {
          if (aoId == hitInfo.objectId) {
            // grabbed the base link
            hitArticulatedObject = true;
            hitArticulatedObjectId = aoId;
          } else if (simulator_->getObjectIdsToLinkIds(aoId).count(
                         hitInfo.objectId) > 0) {
            hitArticulatedObject = true;
            hitArticulatedObjectId = aoId;
            hitArticulatedLinkIndex =
                simulator_->getObjectIdsToLinkIds(aoId).at(hitInfo.objectId);
          }
        }

        if (mouseInteractionMode == GRAB) {
          if (hitArticulatedObject) {
            if (event.button() == MouseEvent::Button::Right) {
              mouseGrabber_ = std::make_unique<MouseArticulatedBaseGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().absoluteTranslation())
                      .length(),
                  hitArticulatedObjectId, simulator_.get());
            } else if (event.button() == MouseEvent::Button::Left) {
              if (hitArticulatedLinkIndex != esp::ID_UNDEFINED) {
                // TODO: handle constraint to base link
                mouseGrabber_ = std::make_unique<MouseLinkGrabber>(
                    hitInfo.point,
                    (hitInfo.point -
                     renderCamera_->node().absoluteTranslation())
                        .length(),
                    hitArticulatedObjectId, hitArticulatedLinkIndex,
                    simulator_.get());
              }
            }
          } else {
            if (event.button() == MouseEvent::Button::Right) {
              mouseGrabber_ = std::make_unique<MouseObjectKinematicGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().absoluteTranslation())
                      .length(),
                  hitInfo.objectId, simulator_.get());
            } else if (event.button() == MouseEvent::Button::Left) {
              mouseGrabber_ = std::make_unique<MouseObjectGrabber>(
                  hitInfo.point,
                  (hitInfo.point - renderCamera_->node().absoluteTranslation())
                      .length(),
                  hitInfo.objectId, simulator_.get());
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
      if (simulator_->getExistingArticulatedObjectIDs().size()) {
        std::vector<float> pose = simulator_->getArticulatedObjectPositions(
            simulator_->getExistingArticulatedObjectIDs().back());
        Corrade::Utility::Debug()
            << "DOF(" << mouseControlDof << ") = " << pose[mouseControlDof];
      }
    }
  }

  event.setAccepted();
}

void Viewer::mouseScrollEvent(MouseScrollEvent& event) {
  // shift+scroll is forced into x direction on mac, seemingly at OS level, so
  // use both x and y offsets.
  float scrollModVal = abs(event.offset().y()) > abs(event.offset().x())
                           ? event.offset().y()
                           : event.offset().x();
  if (!(scrollModVal)) {
    return;
  }

  if (mouseGrabber_ != nullptr) {
    // adjust the depth
    auto ray = renderCamera_->unproject(event.position());
    mouseGrabber_->gripDepth += event.offset().y() * 0.01;
    mouseGrabber_->target = renderCamera_->node().absoluteTranslation() +
                            ray.direction * mouseGrabber_->gripDepth;
    mouseGrabber_->updatePivotB(mouseGrabber_->target);
    // update click visualization
    simulator_->setTranslation(mouseGrabber_->target, clickVisObjectID_);
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
}  // Viewer::mouseScrollEvent

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  if (mouseInteractionMode == LOOK &&
      (event.buttons() & MouseMoveEvent::Button::Left)) {
    const Mn::Vector2i delta = event.relativePosition();
    auto& controls = *defaultAgent_->getControls().get();
    controls(*agentBodyNode_, "turnRight", delta.x());
    // apply the transformation to all sensors
    for (auto p : defaultAgent_->getSensorSuite().getSensors()) {
      controls(p.second->object(),  // SceneNode
               "lookDown",          // action name
               delta.y(),           // amount
               false);              // applyFilter
      redraw();
    }
  } else if (mouseGrabber_ != nullptr) {
    auto ray = renderCamera_->unproject(event.position());
    mouseGrabber_->target = renderCamera_->node().absoluteTranslation() +
                            ray.direction * mouseGrabber_->gripDepth;
    mouseGrabber_->updatePivotB(mouseGrabber_->target);
    // update click visualization
    simulator_->setTranslation(mouseGrabber_->target, clickVisObjectID_);

  } else if (mouseInteractionMode == DOF) {
    if (simulator_->getExistingArticulatedObjectIDs().size()) {
      if (event.buttons() & MouseMoveEvent::Button::Left) {
        std::vector<float> pose = simulator_->getArticulatedObjectPositions(
            simulator_->getExistingArticulatedObjectIDs().back());
        mouseControlDof = mouseControlDof % pose.size();
        if (mouseControlDof < 0)
          mouseControlDof = pose.size() - 1;
        pose[mouseControlDof] += event.relativePosition()[0] * 0.02;
        simulator_->setArticulatedObjectPositions(
            simulator_->getExistingArticulatedObjectIDs().back(), pose);
      } else if (event.buttons() & MouseMoveEvent::Button::Right) {
        mouseDofDelta += event.relativePosition()[0];
        if (abs(mouseDofDelta) > 20) {
          mouseControlDof += mouseDofDelta / abs(mouseDofDelta);
          mouseDofDelta = 0;
        }
        if (mouseControlDof < 0)
          mouseControlDof =
              simulator_
                  ->getArticulatedObjectPositions(
                      simulator_->getExistingArticulatedObjectIDs().back())
                  .size() -
              1;
        mouseControlDof =
            mouseControlDof %
            simulator_
                ->getArticulatedObjectPositions(
                    simulator_->getExistingArticulatedObjectIDs().back())
                .size();
      }
    }
  }

  event.setAccepted();
}

// NOTE: Mouse + shift is to select object on the screen!!
void Viewer::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  switch (key) {
    case KeyEvent::Key::R: {
      const auto recorder = simulator_->getGfxReplayManager()->getRecorder();
      if (recorder) {
        recorder->writeSavedKeyframesToFile(gfxReplayRecordFilepath_);
      }
    } break;
    case KeyEvent::Key::Esc:
      /* Using Application::exit(), which exits at the next iteration of the
         event loop (same as the window close button would do). Using
         std::exit() would exit immediately, but without calling any scoped
         destructors, which could hide potential destruction order issues or
         crashes at exit. We don't want that. */
      exit(0);
      break;
    case KeyEvent::Key::Space:
      simulating_ = !simulating_;
      Mn::Debug{} << " Physics Simulation: " << simulating_;
      break;
    case KeyEvent::Key::Period:
      // also `>` key
      simulateSingleStep_ = true;
      break;
      // ==== Look direction and Movement ====
    case KeyEvent::Key::Left:
      defaultAgent_->act("turnLeft");
      break;
    case KeyEvent::Key::Right:
      defaultAgent_->act("turnRight");
      break;
    case KeyEvent::Key::Up:
      defaultAgent_->act("lookUp");
      break;
    case KeyEvent::Key::Down:
      defaultAgent_->act("lookDown");
      break;
    case KeyEvent::Key::A:
      defaultAgent_->act("moveLeft");
      recAgentLocation();
      break;
    case KeyEvent::Key::D:
      defaultAgent_->act("moveRight");
      recAgentLocation();
      break;
    case KeyEvent::Key::S:
      defaultAgent_->act("moveBackward");
      recAgentLocation();
      break;
    case KeyEvent::Key::W:
      defaultAgent_->act("moveForward");
      recAgentLocation();
      break;
    case KeyEvent::Key::X:
      defaultAgent_->act("moveDown");
      recAgentLocation();
      break;
    case KeyEvent::Key::Z:
      defaultAgent_->act("moveUp");
      recAgentLocation();
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
    case KeyEvent::Key::C:
      showFPS_ = !showFPS_;
      showFPS_ ? profiler_.enable() : profiler_.disable();
      break;
    case KeyEvent::Key::E:
      simulator_->setFrustumCullingEnabled(
          !simulator_->isFrustumCullingEnabled());
      break;
    case KeyEvent::Key::F:
      pushLastObject();
      break;
    case KeyEvent::Key::K:
      wiggleLastObject();
      break;
    case KeyEvent::Key::N:
      // toggle navmesh visualization
      simulator_->setNavMeshVisualization(
          !simulator_->isNavMeshVisualizationActive());
      break;
    case KeyEvent::Key::M:
      toggleArticulatedMotionType(
          simulator_->getExistingArticulatedObjectIDs().back());
      break;
    case KeyEvent::Key::I:
      screenshot();
      break;
    case KeyEvent::Key::O:
      addTemplateObject();
      break;
    case KeyEvent::Key::P:
      pokeLastObject();
      break;
    case KeyEvent::Key::Q:
      // query the agent state
      showAgentStateMsg(true, true);
      break;
    case KeyEvent::Key::B: {
      // toggle bounding box on objects
      drawObjectBBs = !drawObjectBBs;
      for (auto id : simulator_->getExistingObjectIDs()) {
        simulator_->setObjectBBDraw(drawObjectBBs, id);
      }
    } break;
    case KeyEvent::Key::Zero: {
      std::string urdfFilePath =
          "data/URDF_demo_assets/aliengo/urdf/aliengo.urdf";
      float URDFScaling = esp::core::Random().uniform_float_01() + 0.5;
      int objectId = addArticulatedObject(urdfFilePath, false, URDFScaling);
      Mn::Debug{} << "URDF Randomly scaled to " << URDFScaling;
      auto R = Magnum::Matrix4::rotationX(Magnum::Rad(-1.56));
      R.translation() =
          simulator_->getArticulatedObjectRootState(objectId).translation();
      simulator_->setArticulatedObjectRootState(objectId, R);
      // manually set joint damping
      for (auto motor : simulator_->getExistingJointMotors(objectId)) {
        simulator_->updateJointMotor(objectId, motor.first,
                                     {0, 0, 0, 1.0, 0.1});
      }
      // modify the pose to account for joint limits
      std::vector<float> pose =
          simulator_->getArticulatedObjectPositions(objectId);
      std::vector<int> calfDofs = {2, 5, 8, 11};
      for (auto dof : calfDofs) {
        pose[dof] = -1.0;
        pose[dof - 1] = 0.45;  // also set a thigh
      }
      simulator_->setArticulatedObjectPositions(objectId, pose);
      auto aliengoController =
          std::make_unique<AliengoController>(*simulator_.get(), objectId);
      aliengoControllers.push_back(std::move(aliengoController));

      // edit the friction
      /*
      for(int linkId=0; linkId<simulator_->getNumArticulatedLinks(objectId);
      ++linkId){ simulator_->setArticulatedLinkFriction(objectId, linkId, 1);
      }
       */
    } break;
    case KeyEvent::Key::One: {
      std::string urdfFilePath =
          "data/test_assets/URDF/kuka_iiwa/model_free_base.urdf";
      int objectId = addArticulatedObject(urdfFilePath, true);
      // manually adjust joint damping (half impulse)
      for (auto motor : simulator_->getExistingJointMotors(objectId)) {
        auto settings =
            simulator_->getJointMotorSettings(objectId, motor.first);
        Mn::Debug{} << "motor: " << motor;
        simulator_->updateJointMotor(objectId, motor.first,
                                     {0, 0, 0, 1.0, settings.maxImpulse / 2.0});
      }
    } break;
    case KeyEvent::Key::Two: {
      // switch camera between ortho and perspective
      switchCameraType();
    } break;
    case KeyEvent::Key::Three: {
      std::string urdfFilePath =
          "data/URDF_demo_assets/locobot/urdf/"
          "locobot_description_lite2.urdf";
      int objectId = addArticulatedObject(urdfFilePath);
      auto R = Magnum::Matrix4::rotationX(Magnum::Rad(-1.56));
      R.translation() =
          simulator_
              ->getArticulatedObjectRootState(
                  simulator_->getExistingArticulatedObjectIDs().back())
              .translation();
      simulator_->setArticulatedObjectRootState(
          simulator_->getExistingArticulatedObjectIDs().back(), R);
      auto locobotController = std::make_unique<LocobotController>(
          *simulator_.get(),
          simulator_->getExistingArticulatedObjectIDs().back());
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
          simulator_
              ->getArticulatedObjectRootState(
                  simulator_->getExistingArticulatedObjectIDs().back())
              .translation();
      simulator_->setArticulatedObjectRootState(
          simulator_->getExistingArticulatedObjectIDs().back(), R);
      // manually set joint damping
      for (auto motor : simulator_->getExistingJointMotors(objectId)) {
        simulator_->updateJointMotor(objectId, motor.first,
                                     {0, 0, 0, 1.0, 0.1});
      }
      std::vector<float> pose =
          simulator_->getArticulatedObjectPositions(objectId);
      pose[4] = -1.7;
      simulator_->setArticulatedObjectPositions(objectId, pose);
      auto locobotController = std::make_unique<LocobotController>(
          *simulator_.get(),
          simulator_->getExistingArticulatedObjectIDs().back(), 9, 10);
      locobotControllers.push_back(std::move(locobotController));
    } break;
    case KeyEvent::Key::Five: {
      // reset the scene
      clearAllObjects();
      Corrade::Utility::Debug() << "done clearing, now generating";
      setupDemoFurniture();
    } break;
    case KeyEvent::Key::Minus: {
      if (simulator_->getExistingArticulatedObjectIDs().size()) {
        for (auto& controller : locobotControllers) {
          if (controller->objectId ==
              simulator_->getExistingArticulatedObjectIDs().back()) {
            locobotControllers.pop_back();
          }
        }
        for (auto& controller : aliengoControllers) {
          if (controller->objectId ==
              simulator_->getExistingArticulatedObjectIDs().back()) {
            aliengoControllers.pop_back();
          }
        }
        simulator_->removeArticulatedObject(
            simulator_->getExistingArticulatedObjectIDs().back());
      }
    } break;
    case KeyEvent::Key::Equal: {
      if (simulator_->getPhysicsSimulationLibrary() ==
          esp::physics::PhysicsManager::BULLET) {
        debugBullet_ = !debugBullet_;
        Mn::Debug{} << "debugBullet_ = " << debugBullet_;
      } else {
        Mn::Debug{} << "Bullet not enabled, cannot toggle Bullet debug view.";
      }
    } break;
    case KeyEvent::Key::H:
      printHelpText();
    case KeyEvent::Key::T:
      torqueLastObject();
      break;
    case KeyEvent::Key::U:
      removeLastObject();
      break;
    case KeyEvent::Key::V:
      invertGravity();
      break;
    default:
      break;
  }

  // Update map of moving/looking keys which are currently pressed
  if (keysPressed.count(key) > 0) {
    keysPressed[key] = true;
  }
  redraw();
}

void Viewer::keyReleaseEvent(KeyEvent& event) {
  // Update map of moving/looking keys which are currently pressed
  const auto key = event.key();
  if (keysPressed.count(key) > 0) {
    keysPressed[key] = false;
  }
  redraw();
}

int savedFrames = 0;
//! Save a screenshot to "screenshots/year_month_day_hour-minute-second/#.png"
void Viewer::screenshot() {
  std::string screenshot_directory =
      "screenshots/" + viewerStartTimeString + "/";
  if (!Cr::Utility::Directory::exists(screenshot_directory)) {
    Cr::Utility::Directory::mkpath(screenshot_directory);
  }
  Mn::DebugTools::screenshot(
      Mn::GL::defaultFramebuffer,
      screenshot_directory + std::to_string(savedFrames++) + ".png");
}  // Viewer::screenshot

void Viewer::setupDemoFurniture() {
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

    int initialArticulatedObjectCount =
        simulator_->getExistingArticulatedObjectIDs().size();
    for (size_t i = 0; i < objectFilenames.size(); ++i) {
      addArticulatedObject(objectFilenames[i], true);
      T = simulator_->getArticulatedObjectRootState(
          simulator_->getExistingArticulatedObjectIDs().back());
      T.translation() = objectPositions[i];
      simulator_->setArticulatedObjectRootState(
          simulator_->getExistingArticulatedObjectIDs().back(), T);
      simulator_->resetArticulatedObject(
          simulator_->getExistingArticulatedObjectIDs().back());
      simulator_->setArticulatedObjectSleep(
          simulator_->getExistingArticulatedObjectIDs().back(), true);
    }

    for (auto objectId : simulator_->getExistingArticulatedObjectIDs()) {
      simulator_->resetArticulatedObject(
          simulator_->getExistingArticulatedObjectIDs().back());
      simulator_->setArticulatedObjectSleep(
          simulator_->getExistingArticulatedObjectIDs().back(), true);
    }

    // setup the openable entries
    openableObjects = {
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 0, 0, 2.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 0, 0, -1.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 0, 0, 2.4, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 1, 0, -1.2, 0),  // doubledoor
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 2, 1, 1.0, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 1, 0, 2.3, 0),  // fridge
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 2, 1, 2.3, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 1, 0, 0.8, 0),  // cabinet
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 2, 1, -0.76, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 1, 0, 0.5, 0),  // kitchen counter
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 2, 1, 0.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 3, 2, 0.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 4, 3, 0.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 5, 4, 0.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount],
                 6, 5, 0.5, 0),
        Openable(simulator_.get(),
                 simulator_->getExistingArticulatedObjectIDs()
                     [initialArticulatedObjectCount++],
                 7, 6, 0.5, 0),
    };
  }
}

}  // namespace

MAGNUM_APPLICATION_MAIN(Viewer)
