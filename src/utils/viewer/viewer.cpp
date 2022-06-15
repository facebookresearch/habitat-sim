// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <fstream>
#include <iostream>

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
#include <Magnum/Shaders/Shaders.h>
#include <Magnum/Timeline.h>
#include "esp/core/configure.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneNode.h"
#include "esp/sensor/configure.h"

#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Path.h>
#include <Corrade/Utility/String.h>
#include <Magnum/DebugTools/FrameProfiler.h>
#include <Magnum/DebugTools/Screenshot.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include "esp/core/Esp.h"
#include "esp/core/Utility.h"
#include "esp/gfx/Drawable.h"
#include "esp/scene/SemanticScene.h"

#ifdef ESP_BUILD_WITH_VHACD
#include "esp/geo/VoxelUtils.h"
#endif

#ifdef ESP_BUILD_WITH_AUDIO
#include "esp/sensor/AudioSensor.h"
#endif  // ESP_BUILD_WITH_AUDIO

#include "esp/physics/configure.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/EquirectangularSensor.h"
#include "esp/sensor/FisheyeSensor.h"
#include "esp/sim/Simulator.h"

#include "ObjectPickingHelper.h"

constexpr float moveSensitivity = 0.07f;
constexpr float lookSensitivity = 0.9f;
constexpr float rgbSensorHeight = 1.5f;
constexpr float agentActionsPerSecond = 60.0f;

// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

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
using Mn::Math::Literals::operator""_degf;

//! Define different UI roles for the mouse
enum MouseInteractionMode {
  LOOK,
  GRAB,

  NUM_MODES
};

std::map<MouseInteractionMode, std::string> mouseModeNames = {
    {MouseInteractionMode::LOOK, "LOOK"},
    {MouseInteractionMode::GRAB, "GRAB"}};

//! Create a MouseGrabber from RigidConstraintSettings to manipulate objects
struct MouseGrabber {
  esp::physics::RigidConstraintSettings settings_;
  esp::sim::Simulator& sim_;

  // defines the distance of the grip point from the camera/eye for pivot
  // updates
  float gripDepth = 0;
  int constraintId = esp::ID_UNDEFINED;

  MouseGrabber(const esp::physics::RigidConstraintSettings& settings,
               float _gripDepth,
               esp::sim::Simulator& sim)
      : sim_(sim),
        settings_(settings),
        gripDepth(_gripDepth),
        constraintId(sim_.createRigidConstraint(settings_)) {}

  virtual ~MouseGrabber() { sim_.removeRigidConstraint(constraintId); }

  //! update global pivot position for the constraint
  virtual void updatePivot(const Mn::Vector3& pos) {
    settings_.pivotB = pos;
    sim_.updateRigidConstraint(constraintId, settings_);
  }

  //! update global rotation frame for the constraint
  virtual void updateFrame(const Mn::Matrix3x3& frame) {
    settings_.frameB = frame;
    sim_.updateRigidConstraint(constraintId, settings_);
  }

  //! update global rotation frame and pivot position for the constraint
  virtual void updateTransform(const Mn::Matrix4& transform) {
    settings_.frameB = transform.rotation();
    settings_.pivotB = transform.translation();
    sim_.updateRigidConstraint(constraintId, settings_);
  }

  //! rotate the object's local constraint frame with a global angle axis input
  virtual void rotateLocalFrameByGlobalAngleAxis(const Mn::Vector3 axis,
                                                 const Mn::Rad angle) {
    Mn::Matrix4 objectTransform;
    auto rom = sim_.getRigidObjectManager();
    auto aom = sim_.getArticulatedObjectManager();
    if (rom->getObjectLibHasID(settings_.objectIdA)) {
      objectTransform =
          rom->getObjectByID(settings_.objectIdA)->getTransformation();
    } else {
      objectTransform = aom->getObjectByID(settings_.objectIdA)
                            ->getLinkSceneNode(settings_.linkIdA)
                            ->transformation();
    }
    // convert the axis into the object local space
    Mn::Vector3 localAxis = objectTransform.inverted().transformVector(axis);
    // create a rotation matrix
    Mn::Matrix4 R = Mn::Matrix4::rotation(angle, localAxis.normalized());
    // apply to the frame
    settings_.frameA = R.rotation() * settings_.frameA;
    sim_.updateRigidConstraint(constraintId, settings_);
  }

  //! Render a downward projection of the grasped object's COM
  virtual void renderDebugLines() {
    // cast a ray downward from COM to determine approximate landing point
    Mn::Matrix4 objectTransform;
    auto rom = sim_.getRigidObjectManager();
    auto aom = sim_.getArticulatedObjectManager();
    std::vector<int> objectRelatedIds(1, settings_.objectIdA);
    if (rom->getObjectLibHasID(settings_.objectIdA)) {
      objectTransform =
          rom->getObjectByID(settings_.objectIdA)->getTransformation();
    } else {
      auto ao = aom->getObjectByID(settings_.objectIdA);
      objectTransform =
          ao->getLinkSceneNode(settings_.linkIdA)->transformation();
      for (const auto& entry : ao->getLinkObjectIds()) {
        objectRelatedIds.push_back(entry.first);
      }
    }
    esp::physics::RaycastResults raycastResults = sim_.castRay(
        esp::geo::Ray(objectTransform.translation(), Mn::Vector3(0, -1, 0)));
    float lineLength = 9999.9;
    for (auto& hit : raycastResults.hits) {
      if (!std::count(objectRelatedIds.begin(), objectRelatedIds.end(),
                      hit.objectId)) {
        lineLength = hit.rayDistance;
        break;
      }
    }
    sim_.getDebugLineRender()->drawLine(
        objectTransform.translation(),
        objectTransform.translation() + Mn::Vector3(0, -lineLength, 0),
        Mn::Color4::green());
  }
};

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

  MouseInteractionMode mouseInteractionMode = LOOK;

  Mn::Vector2i previousMousePoint{};

  void drawEvent() override;
  void viewportEvent(ViewportEvent& event) override;
  void mousePressEvent(MouseEvent& event) override;
  void mouseReleaseEvent(MouseEvent& event) override;
  void mouseMoveEvent(MouseMoveEvent& event) override;
  void mouseScrollEvent(MouseScrollEvent& event) override;
  void keyPressEvent(KeyEvent& event) override;
  void keyReleaseEvent(KeyEvent& event) override;
  void moveAndLook(int repetitions);

  /**
   * @brief This function will get a screen-space mouse position appropriately
   * scaled based on framebuffer size and window size.  Generally these would be
   * the same value, but on certain HiDPI displays (Retina displays) they may be
   * different.
   * @param event The mouse event we wish to extract a location of
   * @return The screen-space location of the mouse event.
   */

  Mn::Vector2i getMousePosition(Mn::Vector2i mouseEventPosition) {
    // aquire the mouse position, and scale it based on ratio of framebuffer and
    // window size.
    // on retina displays this scaling calc is necessary to account for HiDPI
    // monitors.
    Mn::Vector2 scaling = Mn::Vector2{framebufferSize()} * dpiScaling() /
                          Mn::Vector2{windowSize()};

    return Mn::Vector2i(mouseEventPosition * scaling);
  }
  // exists if a mouse grabbing constraint is active, destroyed on release
  std::unique_ptr<MouseGrabber> mouseGrabber_ = nullptr;

  //! Most recently custom loaded URDF ('t' key)
  std::string cachedURDF_ = "";

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
  void removeLastObject();
  void invertGravity();

#ifdef ESP_BUILD_WITH_VHACD
  void displayStageDistanceGradientField();

  void iterateAndDisplaySignedDistanceField();

  void displayVoxelField(int objectID);

  int objectDisplayed = -1;

  //! The slice of the grid's SDF to visualize.
  int voxelDistance = 0;
#endif

  /**
   * @brief Toggle between ortho and perspective camera
   */
  void switchCameraType();
  Mn::Vector3 randomDirection();

  /**
   * @brief Display information about the currently loaded scene.
   */
  void dispMetadataInfo();

  esp::agent::AgentConfiguration agentConfig_;

  void saveAgentAndSensorTransformToFile();
  void loadAgentAndSensorTransformFromFile();

  //! string rep of time when viewer application was started
  std::string viewerStartTimeString = getCurrentTimeString();
  void screenshot();

  esp::sensor::CameraSensor& getAgentCamera() {
    esp::sensor::Sensor& cameraSensor =
        agentBodyNode_->getNodeSensorSuite().get("rgba_camera");
    return static_cast<esp::sensor::CameraSensor&>(cameraSensor);
  }

#ifdef ESP_BUILD_WITH_AUDIO
  esp::sensor::AudioSensor& getAgentAudioSensor() {
    esp::sensor::Sensor& audioSensor =
        agentBodyNode_->getNodeSensorSuite().get("audio");
    return static_cast<esp::sensor::AudioSensor&>(audioSensor);
  }
#endif  // ESP_BUILD_WITH_AUDIO

  std::string helpText = R"(
==================================================
Welcome to the Habitat-sim C++ Viewer application!
==================================================
Mouse Functions ('m' to toggle mode):
----------------
In LOOK mode (default):
  LEFT:
    Click and drag to rotate the agent and look up/down.
  RIGHT:
    (physics) Click a surface to instance a random primitive object at that location.
  SHIFT-LEFT:
    Read Semantic ID and tag of clicked object (Currently only HM3D);
  SHIFT-RIGHT:
    Click a mesh to highlight it.
  CTRL-RIGHT:
    (physics) Click on an object to voxelize it and display the voxelization.
  WHEEL:
    Modify orthographic camera zoom/perspective camera FOV (+SHIFT for fine grained control)
In GRAB mode (with 'enable-physics'):
  LEFT:
    Click and drag to pickup and move an object with a point-to-point constraint (e.g. ball joint).
  RIGHT:
    Click and drag to pickup and move an object with a fixed frame constraint.
  WHEEL (with picked object):
    Pull gripped object closer or push it away.
    + ALT: rotate object fixed constraint frame (yaw)
    + CTRL: rotate object fixed constraint frame (pitch)
    + ALT+CTRL: rotate object fixed constraint frame (roll)

Key Commands:
-------------
  esc: Exit the application.
  'H': Display this help message.
  'm': Toggle mouse mode (LOOK | GRAB).
  TAB/Shift-TAB : Cycle to next/previous scene in scene dataset.
  ALT+TAB: Reload the current scene.

  Agent Controls:
  'wasd': Move the agent's body forward/backward, left/right.
  'zx': Move the agent's body up/down.
  arrow keys: Turn the agent's body left/right and camera look up/down.
  '9': Randomly place agent on NavMesh (if loaded).
  'q': Query the agent's state and print to terminal.
  '[': Save agent position/orientation to "./saved_transformations/camera.year_month_day_hour-minute-second.txt".
  ']': Load agent position/orientation from file system, or else from last save in current instance.

  Camera Settings
  '4': Cycle through camera modes (Camera, Fisheye, Equirectangular)
  '5': Switch ortho/perspective camera.
  '6': Reset ortho camera zoom/perspective camera FOV.
  '7': Cycle through rendering modes (RGB, depth, semantic)

  Visualization Utilities:
  'l': Override the default lighting setup with configured settings in `default_light_override.lighting_config.json`.
  'e': Enable/disable frustum culling.
  'c': Show/hide UI overlay.
  'n': Show/hide NavMesh wireframe.
  'i': Save a screenshot to "./screenshots/year_month_day_hour-minute-second/#.png".
  ',': Render a Bullet collision shape debug wireframe overlay (white=active, green=sleeping, blue=wants sleeping, red=can't sleep)

  Object Interactions:
  SPACE: Toggle physics simulation on/off
  '.': Take a single simulation step if not simulating continuously.
  '8': Instance a random primitive object in front of the agent.
  'o': Instance a random file-based object in front of the agent.
  'u': Remove most recently instanced rigid object.
  't': Instance an ArticulatedObject in front of the camera from a URDF file by entering the filepath when prompted.
    +ALT: Import the object with a fixed base.
    +SHIFT Quick-reload the previously specified URDF.
  'b': Toggle display of object bounding boxes.
  'p': Save current simulation state to SceneInstanceAttributes JSON file (with non-colliding filename).
  'v': (physics) Invert gravity.
  'g': (physics) Display a stage's signed distance gradient vector field.
  'k': (physics) Iterate through different ranges of the stage's voxelized signed distance field.

  Additional Utilities:
  'r': Write a replay of the recent simulated frames to a file specified by --gfx-replay-record-filepath.
  '/': Write the current scene's metadata information to console.

  Nav Trajectory Visualization:
  '1': Toggle recording locations for trajectory visualization.
  '2': Build and display trajectory visualization.
  '3': Toggle single color/multi-color trajectory.
  '+': Increase trajectory diameter.
  '-': Decrease trajectory diameter.

  'F': (audio) Add audio source in front of the agent
  '0': (audio) Run audio simulation
  ==================================================
  )";

  //! Print viewer help text to terminal output.
  void printHelpText() { ESP_DEBUG() << helpText; };

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
      ESP_DEBUG() << str;
    }
  }

  /**
   * @brief vector holding past agent locations to build trajectory
   * visualization
   */
  std::vector<Mn::Vector3> agentLocs_;
  float agentTrajRad_ = .01f;
  bool agentLocRecordOn_ = false;
  bool singleColorTrajectory_ = true;

  std::string semanticTag_ = "";

  // Wireframe bounding box around semantic region
  int semanticBBID_ = -1;

  /**
   * @brief Generate and save semantic CC reports for all scenes in dataset
   */
  void generateAndSaveAllSemanticCCReports();

  /**
   * @brief Generate and save semantic CC report
   * @return Whether successful or not.
   */
  bool generateAndSaveSemanticCCReport();
  /**
   * @brief Generate and save vertex-color-semantic object mapping reports for
   * all scenes.
   */
  void generateAndSaveAllVertColorMapReports();

  /**
   * @brief Generate and save vertex-color-semantic object mapping reports
   * @return Whether successful or not.
   */
  bool generateAndSaveVertColorMapReports();

  /**
   * @brief Build semantic region prims.
   */
  void buildSemanticPrims(int semanticID,
                          const std::string& semanticTag,
                          const Mn::Vector3& semanticCtr,
                          const Mn::Vector3& semanticSize,
                          const Mn::Quaternion& rotation);

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
    ESP_DEBUG() << "Agent location recording"
                << (agentLocRecordOn_ ? "on" : "off");
  }  // setAgentLocationRecord

  /**
   * @brief Record agent location if enabled.  Called after any movement.
   */
  inline void recAgentLocation() {
    if (agentLocRecordOn_) {
      auto pt = agentBodyNode_->translation() +
                Mn::Vector3{0, (2.0f * agentTrajRad_), 0};
      agentLocs_.push_back(pt);
      ESP_DEBUG() << "Recording agent location : {" << pt.x() << "," << pt.y()
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
    ESP_DEBUG() << "Agent Trajectory Radius" << mod << ":" << agentTrajRad_;
  }

  esp::logging::LoggingContext loggingContext_;
  // Configuration to use to set up simulator_
  esp::sim::SimulatorConfiguration simConfig_;

  // NavMesh customization options, from args
  bool disableNavmesh_ = false;
  bool recomputeNavmesh_ = false;
  std::string navmeshFilename_{};

  // if currently cycling through available SceneInstances, this is the list of
  // SceneInstances available
  std::vector<std::string> curSceneInstances_;
  // current index in SceneInstance array, if cycling through scenes
  int curSceneInstanceIDX_ = 0;

  // increment/decrement currently displayed scene instance
  int getNextSceneInstanceIDX(int incr) {
    if (curSceneInstances_.size() == 0) {
      return 0;
    }
    return (curSceneInstanceIDX_ + curSceneInstances_.size() + incr) %
           curSceneInstances_.size();
  }

  /**
   * @brief Set the scene instance to use to build the scene.
   */
  void setSceneInstanceFromListAndShow(int nextSceneInstanceIDX);

  // The MetadataMediator can exist independent of simulator
  // and provides access to all managers for currently active scene dataset
  std::shared_ptr<esp::metadata::MetadataMediator> MM_;

  // The managers belonging to MetadataMediator
  std::shared_ptr<esp::metadata::managers::ObjectAttributesManager>
      objectAttrManager_ = nullptr;

  // The simulator object backend for this viewer instance
  std::unique_ptr<esp::sim::Simulator> simulator_;

  // Initialize simulator after a scene has been loaded - handle navmesh, agent
  // and sensor configs
  void initSimPostReconfigure();

  // Toggle physics simulation on/off
  bool simulating_ = true;

  // Toggle a single simulation step at the next opportunity if not simulating
  // continuously.
  bool simulateSingleStep_ = false;

  bool debugBullet_ = false;

  esp::scene::SceneNode* agentBodyNode_ = nullptr;

  const int defaultAgentId_ = 0;
  esp::agent::Agent::ptr defaultAgent_ = nullptr;

  // if currently orthographic
  bool isOrtho_ = false;

  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::scene::SceneGraph* activeSceneGraph_ = nullptr;
  bool drawObjectBBs = false;
  std::string gfxReplayRecordFilepath_;

  std::string agentTransformLoadPath_;
  Cr::Containers::Optional<Mn::Matrix4> savedAgentTransform_ =
      Cr::Containers::NullOpt;
  // there could be a couple of sensors, just save the rgb CameraSensor's
  // transformation
  Cr::Containers::Optional<Mn::Matrix4> savedSensorTransform_ =
      Cr::Containers::NullOpt;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};
  bool showFPS_ = true;

  // NOTE: Mouse + shift is to select object on the screen!!
  void createPickedObjectVisualizer(unsigned int objectId);
  std::unique_ptr<ObjectPickingHelper> objectPickingHelper_;

  enum class VisualizeMode : uint8_t {
    RGBA = 0,
    Depth,
    Semantic,
    VisualizeModeCount,
  };
  VisualizeMode visualizeMode_ = VisualizeMode::RGBA;

  Mn::DebugTools::FrameProfilerGL profiler_{};

  enum class VisualSensorMode : uint8_t {
    Camera = 0,
    Fisheye,
    Equirectangular,
    VisualSensorModeCount,
  };
  VisualSensorMode sensorMode_ = VisualSensorMode::Camera;

  /**
   * @brief Set the sensorVisID_ string used to determine which sensor to pull
   * an observation from.  Should be set whenever visualizeMode_ or sensorMode_
   * change.
   */
  void setSensorVisID();

  /**
   * @brief String key used to determine which sensor from a suite of sensors to
   * pull an observation from.
   */
  std::string sensorVisID_ = "rgba_camera";
  void bindRenderTarget();

#ifdef ESP_BUILD_WITH_AUDIO
  /**
   * @brief Add an audio source to the scene
   *  The source is added in front of the agent
   */
  void addAudioSource();
  /**
   * @brief Run the audio simulation and get the observations
   */
  void runAudioSimulation();
#endif  // ESP_BUILD_WITH_AUDIO
};      // class viewer declaration

void addSensors(esp::agent::AgentConfiguration& agentConfig, bool isOrtho) {
  const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();

  auto addCameraSensor = [&](const std::string& uuid,
                             esp::sensor::SensorType sensorType) {
    agentConfig.sensorSpecifications.emplace_back(
        esp::sensor::CameraSensorSpec::create());
    auto spec = static_cast<esp::sensor::CameraSensorSpec*>(
        agentConfig.sensorSpecifications.back().get());

    spec->uuid = uuid;
    spec->sensorSubType = isOrtho ? esp::sensor::SensorSubType::Orthographic
                                  : esp::sensor::SensorSubType::Pinhole;
    spec->sensorType = sensorType;
    if (sensorType == esp::sensor::SensorType::Depth ||
        sensorType == esp::sensor::SensorType::Semantic) {
      spec->channels = 1;
    }
    spec->position = {0.0f, rgbSensorHeight, 0.0f};
    spec->orientation = {0, 0, 0};
    spec->resolution = esp::vec2i(viewportSize[1], viewportSize[0]);
  };
  // add the camera color sensor
  // for historical reasons, we call it "rgba_camera"
  addCameraSensor("rgba_camera", esp::sensor::SensorType::Color);
  // add the camera depth sensor
  addCameraSensor("depth_camera", esp::sensor::SensorType::Depth);
  // add the camera semantic sensor
  addCameraSensor("semantic_camera", esp::sensor::SensorType::Semantic);

  auto addFisheyeSensor = [&](const std::string& uuid,
                              esp::sensor::SensorType sensorType,
                              esp::sensor::FisheyeSensorModelType modelType) {
    // TODO: support the other model types in the future.
    CORRADE_INTERNAL_ASSERT(modelType ==
                            esp::sensor::FisheyeSensorModelType::DoubleSphere);
    agentConfig.sensorSpecifications.emplace_back(
        esp::sensor::FisheyeSensorDoubleSphereSpec::create());
    auto spec = static_cast<esp::sensor::FisheyeSensorDoubleSphereSpec*>(
        agentConfig.sensorSpecifications.back().get());

    spec->uuid = uuid;
    spec->sensorType = sensorType;
    if (sensorType == esp::sensor::SensorType::Depth ||
        sensorType == esp::sensor::SensorType::Semantic) {
      spec->channels = 1;
    }
    spec->sensorSubType = esp::sensor::SensorSubType::Fisheye;
    spec->fisheyeModelType = modelType;
    spec->resolution = esp::vec2i(viewportSize[1], viewportSize[0]);
    // default viewport size: 1600 x 1200
    spec->principalPointOffset =
        Mn::Vector2(viewportSize[0] / 2, viewportSize[1] / 2);
    if (modelType == esp::sensor::FisheyeSensorModelType::DoubleSphere) {
      // in this demo, we choose "GoPro":
      spec->focalLength = {364.84f, 364.86f};
      spec->xi = -0.27;
      spec->alpha = 0.57;
      // Certainly you can try your own lenses.
      // For your convenience, there are some other lenses, e.g., BF2M2020S23,
      // BM2820, BF5M13720, BM4018S118, whose parameters can be found at:
      //   Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double
      //   Sphere Camera Model, The International Conference on 3D Vision(3DV),
      //   2018

      // BF2M2020S23
      // spec->focalLength = Mn::Vector2(313.21, 313.21);
      // spec->xi = -0.18;
      // spec->alpha = 0.59;
    }
  };
  // add the fisheye sensor
  addFisheyeSensor("rgba_fisheye", esp::sensor::SensorType::Color,
                   esp::sensor::FisheyeSensorModelType::DoubleSphere);
  // add the fisheye depth sensor
  addFisheyeSensor("depth_fisheye", esp::sensor::SensorType::Depth,
                   esp::sensor::FisheyeSensorModelType::DoubleSphere);
  // add the fisheye semantic sensor
  addFisheyeSensor("semantic_fisheye", esp::sensor::SensorType::Semantic,
                   esp::sensor::FisheyeSensorModelType::DoubleSphere);

  auto addEquirectangularSensor = [&](const std::string& uuid,
                                      esp::sensor::SensorType sensorType) {
    agentConfig.sensorSpecifications.emplace_back(
        esp::sensor::EquirectangularSensorSpec::create());
    auto spec = static_cast<esp::sensor::EquirectangularSensorSpec*>(
        agentConfig.sensorSpecifications.back().get());
    spec->uuid = uuid;
    spec->sensorType = sensorType;
    if (sensorType == esp::sensor::SensorType::Depth ||
        sensorType == esp::sensor::SensorType::Semantic) {
      spec->channels = 1;
    }
    spec->sensorSubType = esp::sensor::SensorSubType::Equirectangular;
    spec->resolution = esp::vec2i(viewportSize[1], viewportSize[0]);
  };
  // add the equirectangular sensor
  addEquirectangularSensor("rgba_equirectangular",
                           esp::sensor::SensorType::Color);
  // add the equirectangular depth sensor
  addEquirectangularSensor("depth_equirectangular",
                           esp::sensor::SensorType::Depth);
  // add the equirectangular semantic sensor
  addEquirectangularSensor("semantic_equirectangular",
                           esp::sensor::SensorType::Semantic);

// add audio sensor
#ifdef ESP_BUILD_WITH_AUDIO
  ESP_DEBUG() << "Adding audio sendor";

  auto addAudioSensor = [&](const std::string& uuid,
                            esp::sensor::SensorType sensorType,
                            esp::sensor::SensorSubType sensorSubType) {
    agentConfig.sensorSpecifications.emplace_back(
        esp::sensor::AudioSensorSpec::create());
    auto spec = static_cast<esp::sensor::AudioSensorSpec*>(
        agentConfig.sensorSpecifications.back().get());
    spec->uuid = uuid;
    spec->sensorType = sensorType;
    spec->sensorSubType = sensorSubType;

    // Set the audio sensor configs
    spec->acousticsConfig_.dumpWaveFiles = true;
    spec->acousticsConfig_.enableMaterials = true;
    spec->acousticsConfig_.writeIrToFile = true;
    // Set the output directory
    spec->outputDirectory_ = "/tmp/AudioSimulation";
    // Set the output channel layout
    spec->channelLayout_.channelCount = 2;
    spec->channelLayout_.channelType =
        RLRAudioPropagation::ChannelLayoutType::Binaural;
  };
  addAudioSensor("audio", esp::sensor::SensorType::Audio,
                 esp::sensor::SensorSubType::ImpulseResponse);
#endif  // ESP_BUILD_WITH_AUDIO
}  // addSensors

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
      loggingContext_{},
      simConfig_(),
      MM_(std::make_shared<esp::metadata::MetadataMediator>(simConfig_)),
      curSceneInstances_{} {
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
      .addOption("object-dir", "data/objects/example_objects")
      .setHelp("object-dir",
               "Provide a directory to search for object config files "
               "(relative to habitat-sim directory).")
      .addBooleanOption("no-semantic-textures")
      .setHelp("no-semantic-textures",
               "If specified, force vertex semantic annotations even if the "
               "scene/dataset support texture-based.")
      .addBooleanOption("disable-navmesh")
      .setHelp("disable-navmesh",
               "Disable the navmesh, disabling agent navigation constraints.")
      .addOption("navmesh-file")
      .setHelp("navmesh-file", "Manual override path to scene navmesh file.")
      .addBooleanOption("recompute-navmesh")
      .setHelp("recompute-navmesh",
               "Programmatically re-generate the scene navmesh.")
      .addOption("agent-transform-filepath")
      .setHelp("agent-transform-filepath",
               "Specify path to load camera transform from.")
      .addBooleanOption("shadows")
      .setHelp("shadows", "Rendering shadows. (only works with PBR rendering.")
      .addBooleanOption("ibl")
      .setHelp("ibl",
               "Image Based Lighting (it works only when PBR models exist in "
               "the scene.")
      .parse(arguments.argc, arguments.argv);

  const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();

  imgui_ =
      Mn::ImGuiIntegration::Context(Mn::Vector2{windowSize()} / dpiScaling(),
                                    windowSize(), framebufferSize());
  ImGui::GetIO().IniFilename = nullptr;

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

  if (args.isSet("enable-physics") && (args.isSet("debug-bullet"))) {
    debugBullet_ = true;
  }

  agentTransformLoadPath_ = args.value("agent-transform-filepath");
  gfxReplayRecordFilepath_ = args.value("gfx-replay-record-filepath");

  // NavMesh customization options

  disableNavmesh_ = args.isSet("disable-navmesh");
  recomputeNavmesh_ = args.isSet("recompute-navmesh");
  navmeshFilename_ = args.value("navmesh-file").empty();

  // configure default Agent Config for actions and sensors
  agentConfig_ = esp::agent::AgentConfiguration();
  agentConfig_.height = rgbSensorHeight;
  agentConfig_.actionSpace = {
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
  // add sensor specifications to agent config
  addSensors(agentConfig_, isOrtho_);

  // setup SimulatorConfig from args.
  simConfig_.activeSceneName = args.value("scene");
  simConfig_.sceneDatasetConfigFile = args.value("dataset");
  simConfig_.enablePhysics = args.isSet("enable-physics");
  simConfig_.frustumCulling = true;
  simConfig_.requiresTextures = true;
  simConfig_.enableGfxReplaySave = !gfxReplayRecordFilepath_.empty();
  simConfig_.useSemanticTexturesIfFound = !args.isSet("no-semantic-textures");
  if (args.isSet("stage-requires-lighting")) {
    ESP_DEBUG() << "Stage using DEFAULT_LIGHTING_KEY";
    simConfig_.overrideSceneLightDefaults = true;
    simConfig_.sceneLightSetupKey = esp::DEFAULT_LIGHTING_KEY;
  }

  // setup the PhysicsManager config file
  std::string physicsConfig =
      Cr::Utility::Path::join(*Corrade::Utility::Path::currentDirectory(),
                              args.value("physics-config"));
  if (Cr::Utility::Path::exists(physicsConfig)) {
    ESP_DEBUG() << "Using PhysicsManager config:" << physicsConfig;
    simConfig_.physicsConfigFile = physicsConfig;
  }

  // will set simulator configuration in MM - sets ActiveDataset as well
  MM_->setSimulatorConfiguration(simConfig_);
  objectAttrManager_ = MM_->getObjectAttributesManager();
  objectAttrManager_->loadAllJSONConfigsFromPath(args.value("object-dir"));

  ESP_DEBUG() << "Scene Dataset Configuration file location :"
              << simConfig_.sceneDatasetConfigFile
              << "| Loading Scene :" << simConfig_.activeSceneName;

  // image based lighting (PBR)
  simConfig_.pbrImageBasedLighting = args.isSet("ibl");

  // create simulator instance
  simulator_ = esp::sim::Simulator::create_unique(simConfig_, MM_);

  ////////////////////////////
  // Build list of scenes/stages in specified scene dataset to cycle through
  // get list of all scenes
  curSceneInstances_ = MM_->getAllSceneInstanceHandles();
  // check if any scene instances exist - some datasets might only have stages
  // defined and not scene instances
  std::size_t numInstances = curSceneInstances_.size();
  // if only 1 scene instance, then get all available stages in dataset to cycle
  // through, if more than 1
  if (numInstances == 1) {
    const std::size_t numStages =
        MM_->getStageAttributesManager()->getNumObjects();
    if (numStages > 1) {
      numInstances = numStages;
      curSceneInstances_ = MM_->getAllStageAttributesHandles();
    }
  }

  // To handle cycling through scene instances, set first scene to be first in
  // list of current scene dataset's scene instances
  // Set this to index in curSceneInstances where args.value("scene") can be
  // found.
  curSceneInstanceIDX_ = 0;
  for (int i = 0; i < numInstances; ++i) {
    if (curSceneInstances_[i].find(simConfig_.activeSceneName) !=
        std::string::npos) {
      curSceneInstanceIDX_ = i;
      break;
    }
  }

  // initialize sim navmesh, agent, sensors after creation/reconfigure
  initSimPostReconfigure();

  objectPickingHelper_ = std::make_unique<ObjectPickingHelper>(viewportSize);

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
  Mn::DebugTools::FrameProfilerGL::Values profilerValues =
      Mn::DebugTools::FrameProfilerGL::Value::FrameTime |
      Mn::DebugTools::FrameProfilerGL::Value::CpuDuration |
      Mn::DebugTools::FrameProfilerGL::Value::GpuDuration;

// VertexFetchRatio and PrimitiveClipRatio only supported for GL 4.6
#ifndef MAGNUM_TARGET_GLES
  if (Mn::GL::Context::current()
          .isExtensionSupported<
              Mn::GL::Extensions::ARB::pipeline_statistics_query>()) {
    profilerValues |=
        Mn::DebugTools::FrameProfilerGL::Value::
            VertexFetchRatio |  // Ratio of vertex shader invocations to count
                                // of vertices submitted
        Mn::DebugTools::FrameProfilerGL::Value::
            PrimitiveClipRatio;  // Ratio of primitives discarded by the
                                 // clipping stage to count of primitives
                                 // submitted
  }
#endif

  // Per frame profiler will average measurements taken over previous 50 frames
  profiler_.setup(profilerValues, 50);

  // shadows
  if (args.isSet("shadows")) {
    simulator_->updateShadowMapDrawableGroup();
    simulator_->computeShadowMaps(0.01f,   // lightNearPlane
                                  20.0f);  // lightFarPlane
    simulator_->setShadowMapsToDrawables();
  }

  printHelpText();
}  // end Viewer::Viewer

void Viewer::initSimPostReconfigure() {
  const auto sceneInstName = simulator_->getCurSceneInstanceName();
  if (sceneInstName == "NONE") {
    setWindowTitle("Viewer");
  } else {
    setWindowTitle("Viewer @ Scene : " + sceneInstName);
  }
  // clear any semantic tags from previoius scene
  semanticTag_ = "";
  semanticBBID_ = -1;
  // NavMesh customization options
  if (disableNavmesh_) {
    if (simulator_->getPathFinder()->isLoaded()) {
      simulator_->setPathFinder(esp::nav::PathFinder::create());
    }
  } else if (recomputeNavmesh_) {
    esp::nav::NavMeshSettings navMeshSettings;
    navMeshSettings.agentHeight = agentConfig_.height;
    navMeshSettings.agentRadius = agentConfig_.radius;
    simulator_->recomputeNavMesh(*simulator_->getPathFinder().get(),
                                 navMeshSettings, true);
  } else if (!navmeshFilename_.empty()) {
    std::string navmeshFile = Cr::Utility::Path::join(
        *Corrade::Utility::Path::currentDirectory(), navmeshFilename_);
    if (Cr::Utility::Path::exists(navmeshFile)) {
      simulator_->getPathFinder()->loadNavMesh(navmeshFile);
    }
  }
  // add selects a random initial state and sets up the default controls and
  // step filter
  simulator_->addAgent(agentConfig_);

  // Set up camera
  activeSceneGraph_ = &simulator_->getActiveSceneGraph();
  defaultAgent_ = simulator_->getAgent(defaultAgentId_);
  agentBodyNode_ = &defaultAgent_->node();
  renderCamera_ = getAgentCamera().getRenderCamera();
  timeline_.start();
}  // initSimPostReconfigure

void Viewer::switchCameraType() {
  auto& cam = getAgentCamera();
  auto oldCameraType = cam.getCameraType();
  switch (oldCameraType) {
    case esp::sensor::SensorSubType::Pinhole: {
      isOrtho_ = true;
      cam.setCameraType(esp::sensor::SensorSubType::Orthographic);
      return;
    }
    case esp::sensor::SensorSubType::Orthographic: {
      isOrtho_ = false;
      cam.setCameraType(esp::sensor::SensorSubType::Pinhole);
      return;
    }
    case esp::sensor::SensorSubType::Fisheye: {
      return;
    }
    case esp::sensor::SensorSubType::Equirectangular: {
      return;
    }
    case esp::sensor::SensorSubType::None: {
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      return;
    }
  }
}

void saveTransformToFile(const std::string& filename,
                         const Mn::Matrix4 agentTransform,
                         const Mn::Matrix4 sensorTransform) {
  std::ofstream file(filename);
  if (!file.good()) {
    ESP_ERROR() << "Cannot open" << filename << "to output data.";
    return;
  }

  // saving transformation into file system
  auto save = [&](const Mn::Matrix4 transform) {
    const float* t = transform.data();
    for (int i = 0; i < 16; ++i) {
      file << t[i] << " ";
    }
    ESP_DEBUG() << "Transformation matrix saved to" << filename << ":"
                << Eigen::Map<const esp::mat4f>(transform.data());
  };
  save(agentTransform);
  save(sensorTransform);

  file.close();
}
void Viewer::saveAgentAndSensorTransformToFile() {
  const char* saved_transformations_directory = "saved_transformations/";
  if (!Cr::Utility::Path::exists(saved_transformations_directory)) {
    Cr::Utility::Path::make(saved_transformations_directory);
  }

  // update temporary save
  savedAgentTransform_ = defaultAgent_->node().transformation();
  savedSensorTransform_ = getAgentCamera().node().transformation();

  // update save in file system
  saveTransformToFile(Cr::Utility::formatString("{}camera.{}.txt",
                                                saved_transformations_directory,
                                                getCurrentTimeString()),
                      *savedAgentTransform_, *savedSensorTransform_);
}

bool loadTransformFromFile(const std::string& filename,
                           Mn::Matrix4& agentTransform,
                           Mn::Matrix4& sensorTransform) {
  std::ifstream file(filename);
  if (!file.good()) {
    ESP_ERROR() << "Cannot open" << filename << "to load data.";
    return false;
  }

  // reading file system data into matrix as transformation
  auto load = [&](Mn::Matrix4& transform) {
    Mn::Vector4 cols[4];
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        file >> cols[col][row];
      }
    }
    Mn::Matrix4 temp{cols[0], cols[1], cols[2], cols[3]};
    if (!temp.isRigidTransformation()) {
      ESP_WARNING() << "Data loaded from" << filename
                    << "is not a valid rigid transformation.";
      return false;
    }
    transform = temp;
    ESP_DEBUG() << "Transformation matrix loaded from" << filename << ":"
                << Eigen::Map<esp::mat4f>(transform.data());
    return true;
  };
  // NOTE: load Agent first!!
  bool status = load(agentTransform) && load(sensorTransform);
  file.close();
  return status;
}

void Viewer::generateAndSaveAllVertColorMapReports() {
  for (int idx = 0; idx < curSceneInstances_.size(); ++idx) {
    if (std::string::npos != curSceneInstances_[idx].find("NONE")) {
      continue;
    }
    setSceneInstanceFromListAndShow(idx);
    bool success = generateAndSaveVertColorMapReports();
    if (!success) {
      ESP_DEBUG() << "Report failed. Aborting!";
    }
  }
  ESP_DEBUG() << "All reports done!";
}  // Viewer::generateAndSaveAllVertColorMapReports

/**
 * @brief Generate and save vertex-color-semantic object mapping reports
 */
bool Viewer::generateAndSaveVertColorMapReports() {
  const auto results = simulator_->buildVertexColorMapReport();
  if (results.empty()) {
    return false;
  }
  const std::string fileDir = Cr::Utility::Path::join(
      Cr::Utility::Path::split(MM_->getActiveSceneDatasetName()).first(),
      "Vertex_Color_Reports");

  Cr::Utility::Path::make(fileDir);

  const std::string filename = Cr::Utility::Path::join(
      fileDir,
      Cr::Utility::formatString(
          "{}_vert_color_report.txt",
          Cr::Utility::Path::splitExtension(
              Cr::Utility::Path::split(simConfig_.activeSceneName).second())
              .first()));
  ESP_DEBUG() << "Fully qualified destination file name :" << filename;
  std::ofstream file(filename);
  if (!file.good()) {
    ESP_ERROR() << "Cannot open" << filename
                << "to output vert-color-semantic object mapping report data.";
    return false;
  }
  for (const std::string& line : results) {
    ESP_DEBUG() << line;
    file << line << '\n';
  }
  file.close();
  return true;
}  // Viewer::generateAndSaveVertColorMapReports

void Viewer::generateAndSaveAllSemanticCCReports() {
  for (int idx = 0; idx < curSceneInstances_.size(); ++idx) {
    if (std::string::npos != curSceneInstances_[idx].find("NONE")) {
      continue;
    }
    setSceneInstanceFromListAndShow(idx);
    bool success = generateAndSaveSemanticCCReport();
    if (!success) {
      ESP_DEBUG() << "Report failed. Aborting!";
    }
  }
  ESP_DEBUG() << "All reports done!";
}  // Viewer::generateAndSaveAllSemanticCCReports

bool Viewer::generateAndSaveSemanticCCReport() {
  const auto results = simulator_->buildSemanticCCObjects();
  if (results.empty()) {
    return false;
  }
  const std::string fileDir = Cr::Utility::Path::join(
      Cr::Utility::Path::split(MM_->getActiveSceneDatasetName()).first(),
      "Semantic_CC_Reports");

  Cr::Utility::Path::make(fileDir);

  const std::string filename = Cr::Utility::Path::join(
      fileDir,
      Cr::Utility::formatString(
          "{}_CC_report.csv",
          Cr::Utility::Path::splitExtension(
              Cr::Utility::Path::split(simConfig_.activeSceneName).second())
              .first()));
  ESP_DEBUG() << "Fully qualified destination file name :" << filename;
  auto semanticScene = simulator_->getSemanticScene();

  const auto& semanticObjs = semanticScene->objects();

  std::ofstream file(filename);
  if (!file.good()) {
    ESP_ERROR() << "Cannot open" << filename << "to output CC report data.";
    return false;
  }

  file << "Obj IDX, Object ID, Color RGB, # Verts in partition, BBOX "
          "Center XYZ, BBOX Dims XYZ, BBOX Vol\n";

  for (const auto& elem : results) {
    const uint32_t objIDX = elem.first;
    const std::vector<std::shared_ptr<esp::scene::CCSemanticObject>>&
        listOfObbs = elem.second;
    const auto baseObj = semanticObjs[objIDX];
    for (const std::shared_ptr<esp::scene::CCSemanticObject>& ccObj :
         listOfObbs) {
      const auto obb = ccObj->obb();
      const auto clr = ccObj->getColor();
      const auto ctr = obb.center();
      const auto sizes = obb.sizes();
      const std::string dataString = Cr::Utility::formatString(
          "{},{},{} {} {},{},{} {} {}, {} {} {},{}", objIDX, baseObj->id(),
          clr.r(), clr.g(), clr.b(), ccObj->getNumSrcVerts(), ctr.x(), ctr.y(),
          ctr.z(), sizes.x(), sizes.y(), sizes.z(), obb.volume());
      ESP_VERY_VERBOSE() << dataString;
      file << dataString << '\n';
    }
  }

  file.close();
  return true;
}  // Viewer::generateAndSaveSemanticCCReport

void Viewer::loadAgentAndSensorTransformFromFile() {
  if (!agentTransformLoadPath_.empty()) {
    // loading from file system
    Mn::Matrix4 agentMtx;
    Mn::Matrix4 sensorMtx;
    if (!loadTransformFromFile(agentTransformLoadPath_, agentMtx, sensorMtx))
      return;
    savedAgentTransform_ = agentMtx;
    savedSensorTransform_ = sensorMtx;
  } else {
    // attempting to load from last temporary save
    ESP_DEBUG()
        << "Camera transform file not specified, attempting to load from "
           "current instance. Use --agent-transform-filepath to specify file "
           "to load from.";
    if (!savedAgentTransform_ || !savedSensorTransform_) {
      ESP_DEBUG()
          << "Well, no transformation saved in current instance. nothing "
             "is changed.";
      return;
    }
  }

  defaultAgent_->node().setTransformation(*savedAgentTransform_);
  for (const auto& p : defaultAgent_->node().getNodeSensors()) {
    p.second.get().object().setTransformation(*savedSensorTransform_);
  }
  ESP_DEBUG()
      << "Transformation matrices are loaded to the agent and the sensors.";
}

int Viewer::addObject(int ID) {
  const std::string& configHandle =
      simulator_->getObjectAttributesManager()->getObjectHandleByID(ID);
  return addObject(configHandle);
}  // addObject

int Viewer::addObject(const std::string& objectAttrHandle) {
  // Relative to agent bodynode
  Mn::Matrix4 T = agentBodyNode_->transformationMatrix();
  Mn::Vector3 new_pos = T.transformPoint({0.1f, 1.5f, -2.0f});
  auto rigidObjMgr = simulator_->getRigidObjectManager();
  auto obj = rigidObjMgr->addObjectByHandle(objectAttrHandle);
  obj->setTranslation(new_pos);
  obj->setRotation(Mn::Quaternion::fromMatrix(T.rotationNormalized()));
  return obj->getID();
}  // addObject

// add file-based template derived object from keypress
int Viewer::addTemplateObject() {
  int numObjTemplates = objectAttrManager_->getNumFileTemplateObjects();
  if (numObjTemplates > 0) {
    return addObject(objectAttrManager_->getRandomFileTemplateHandle());
  } else {
    ESP_WARNING() << "No objects loaded, can't add any";
    return esp::ID_UNDEFINED;
  }
}  // addTemplateObject

// add synthesized primiitive object from keypress
int Viewer::addPrimitiveObject() {
  int numObjPrims = objectAttrManager_->getNumSynthTemplateObjects();
  if (numObjPrims > 0) {
    return addObject(objectAttrManager_->getRandomSynthTemplateHandle());
  } else {
    ESP_WARNING() << "No primitive templates available, can't add any objects";
    return esp::ID_UNDEFINED;
  }
}  // addPrimitiveObject

void Viewer::buildTrajectoryVis() {
  if (agentLocs_.size() < 2) {
    ESP_WARNING() << "No recorded trajectory "
                     "points, so nothing to build. Aborting.";
    return;
  }
  std::vector<Mn::Color3> clrs;
  int numClrs = (singleColorTrajectory_ ? 1 : rand() % 4 + 2);
  clrs.reserve(numClrs);
  for (int i = 0; i < numClrs; ++i) {
    clrs.emplace_back(Mn::Color3{randomDirection()});
  }
  // synthesize a name for asset based on color, radius, point count
  std::string trajObjName = Cr::Utility::formatString(
      "viewerTrajVis_R{}_G{}_B{}_clrs_{}_rad_{}_{}_pts", clrs[0].r(),
      clrs[0].g(), clrs[0].b(), numClrs, agentTrajRad_, agentLocs_.size());

  ESP_DEBUG() << "Attempting to build trajectory tube for :"
              << agentLocs_.size() << "points with " << numClrs << "colors.";
  int trajObjID = simulator_->addTrajectoryObject(trajObjName, agentLocs_, clrs,
                                                  6, agentTrajRad_, true, 10);
  if (trajObjID != esp::ID_UNDEFINED) {
    ESP_DEBUG() << "Success!  Traj Obj Name :" << trajObjName
                << "has object ID :" << trajObjID;
  } else {
    ESP_WARNING() << "Attempt to build trajectory visualization" << trajObjName
                  << "failed; Returned ID_UNDEFINED.";
  }
}  // buildTrajectoryVis

void Viewer::removeLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0) {
    return;
  }
  auto rigidObjMgr = simulator_->getRigidObjectManager();
  rigidObjMgr->removeObjectByID(existingObjectIDs.back());
}

void Viewer::invertGravity() {
  const Mn::Vector3& gravity = simulator_->getGravity();
  const Mn::Vector3 invGravity = -1 * gravity;
  simulator_->setGravity(invGravity);
}
#ifdef ESP_BUILD_WITH_VHACD

void Viewer::displayStageDistanceGradientField() {
  // Temporary key event used for testing & visualizing Voxel Grid framework
  std::shared_ptr<esp::geo::VoxelWrapper> stageVoxelization;
  stageVoxelization = simulator_->getStageVoxelization();

  // if the object hasn't been voxelized, do that and generate an SDF as
  // well
  !ESP_DEBUG();
  if (stageVoxelization == nullptr) {
    simulator_->createStageVoxelization(2000000);
    stageVoxelization = simulator_->getStageVoxelization();
    esp::geo::generateEuclideanDistanceSDF(stageVoxelization,
                                           "ESignedDistanceField");
  }
  !ESP_DEBUG();

  // generate a vector field for the SDF gradient
  esp::geo::generateScalarGradientField(
      stageVoxelization, "ESignedDistanceField", "GradientField");
  // generate a mesh of the vector field with boolean isVectorField set to
  // true
  !ESP_DEBUG();

  stageVoxelization->generateMesh("GradientField");

  // draw the vector field
  simulator_->setStageVoxelizationDraw(true, "GradientField");
}

void Viewer::iterateAndDisplaySignedDistanceField() {
  // Temporary key event used for testing & visualizing Voxel Grid framework
  std::shared_ptr<esp::geo::VoxelWrapper> stageVoxelization;
  stageVoxelization = simulator_->getStageVoxelization();

  // if the object hasn't been voxelized, do that and generate an SDF as
  // well
  if (stageVoxelization == nullptr) {
    simulator_->createStageVoxelization(2000000);
    stageVoxelization = simulator_->getStageVoxelization();
    esp::geo::generateEuclideanDistanceSDF(stageVoxelization,
                                           "ESignedDistanceField");
  }

  // Set the range of distances to render, and generate a mesh for this (18
  // is set to be the max distance)
  Mn::Vector3i dims = stageVoxelization->getVoxelGridDimensions();
  int curDistanceVisualization = (voxelDistance % dims[0]);
  /*sceneVoxelization->generateBoolGridFromFloatGrid("ESignedDistanceField",
     "SDFSubset", curDistanceVisualization, curDistanceVisualization + 1);*/
  stageVoxelization->generateSliceMesh("ESignedDistanceField",
                                       curDistanceVisualization, -15.0f, 0.0f);
  // Draw the voxel grid's slice
  simulator_->setStageVoxelizationDraw(true, "ESignedDistanceField");
}

void Viewer::displayVoxelField(int objectID) {
  // create a voxelization and get a pointer to the underlying VoxelWrapper
  // class
  unsigned int resolution = 2000000;
  std::shared_ptr<esp::geo::VoxelWrapper> voxelWrapper;
  if (objectID == -1) {
    simulator_->createStageVoxelization(resolution);
    voxelWrapper = simulator_->getStageVoxelization();
  } else {
    simulator_->createObjectVoxelization(objectID, resolution);
    voxelWrapper = simulator_->getObjectVoxelization(objectID);
  }

  // turn off the voxel grid visualization for the last voxelized object
  if (objectDisplayed == -1) {
    simulator_->setStageVoxelizationDraw(false, "Boundary");
  } else if (objectDisplayed >= 0) {
    simulator_->setObjectVoxelizationDraw(false, objectDisplayed, "Boundary");
  }

  // Generate the mesh for the boundary voxel grid
  voxelWrapper->generateMesh("Boundary");

  esp::geo::generateEuclideanDistanceSDF(voxelWrapper, "ESignedDistanceField");

  // visualizes the Boundary voxel grid
  if (objectID == -1) {
    simulator_->setStageVoxelizationDraw(true, "Boundary");
  } else {
    simulator_->setObjectVoxelizationDraw(true, objectID, "Boundary");
  }

  objectDisplayed = objectID;
}
#endif

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

void Viewer::setSceneInstanceFromListAndShow(int nextSceneInstanceIDX) {
  // set current to be passed idx, making sure it is in bounds of scene list
  curSceneInstanceIDX_ = (nextSceneInstanceIDX % curSceneInstances_.size());
  // Set scene instance in SimConfig
  simConfig_.activeSceneName = curSceneInstances_[curSceneInstanceIDX_];

  // update MM's config with new active scene name
  MM_->setSimulatorConfiguration(simConfig_);
  // close and reconfigure simulator - is this really necessary?
  ESP_DEBUG() << "Active Scene Dataset :" << MM_->getActiveSceneDatasetName()
              << "| Loading Scene :" << simConfig_.activeSceneName;

  renderCamera_ = nullptr;
  agentBodyNode_ = nullptr;
  defaultAgent_ = nullptr;
  activeSceneGraph_ = nullptr;

  // close and reconfigure
  simulator_->close(false);
  // simulator_ = esp::sim::Simulator::create_unique(simConfig_, MM_);
  simulator_->reconfigure(simConfig_);

  // initialize sim navmesh, agent, sensors after creation/reconfigure
  initSimPostReconfigure();
  // run this in case we are currently displaying something other than visual
  // sensor
  bindRenderTarget();
}  // Viewer::setSceneInstanceFromListAndShow

/**
 * @brief Set the sensorVisID_ string used to determine which sensor to pull an
 * observation from.  Should be set whenever visualizeMode_ or sensorMode_
 * change.
 */
void Viewer::setSensorVisID() {
  std::string prefix = "rgba";
  switch (visualizeMode_) {
    case VisualizeMode::RGBA: {
      prefix = "rgba";
      break;
    }
    case VisualizeMode::Depth: {
      prefix = "depth";
      break;
    }
    case VisualizeMode::Semantic: {
      prefix = "semantic";
      break;
    }
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }  // switch on visualize mode
  std::string suffix = "camera";
  switch (sensorMode_) {
    case VisualSensorMode::Camera: {
      suffix = "camera";
      break;
    }
    case VisualSensorMode::Fisheye: {
      suffix = "fisheye";
      break;
    }
    case VisualSensorMode::Equirectangular: {
      suffix = "equirectangular";
      break;
    }
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }  // switch on sensor mode
  sensorVisID_ = Cr::Utility::formatString("{}_{}", prefix, suffix);
  ESP_DEBUG() << Cr::Utility::formatString(
      "Visualizing {} {} sensor. Sensor Suite Key : {}", prefix, suffix,
      sensorVisID_);
}  // Viewer::setSensorVisID

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

  uint32_t visibles = renderCamera_->getPreviousNumVisibleDrawables();

  if ((visualizeMode_ == VisualizeMode::RGBA) &&
      (sensorMode_ == VisualSensorMode::Camera)) {
    // Visualizing RGBA pinhole camera
    if (mouseGrabber_ != nullptr) {
      mouseGrabber_->renderDebugLines();
    }

    // ============= regular RGB with object picking =================
    // using polygon offset to increase mesh depth to avoid z-fighting with
    // debug draw (since lines will not respond to offset).
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::PolygonOffsetFill);
    Mn::GL::Renderer::setPolygonOffset(1.0f, 0.1f);

    // ONLY draw the content to the frame buffer but not immediately blit the
    // result to the default main buffer
    // (this is the reason we do not call displayObservation)
    simulator_->drawObservation(defaultAgentId_, sensorVisID_);

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
        simulator_->getRenderTarget(defaultAgentId_, sensorVisID_);
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
  } else {
    // Depth Or Semantic, or Non-pinhole RGBA
    simulator_->drawObservation(defaultAgentId_, sensorVisID_);

    esp::gfx::RenderTarget* sensorRenderTarget =
        simulator_->getRenderTarget(defaultAgentId_, sensorVisID_);
    CORRADE_ASSERT(sensorRenderTarget,
                   "Error in Viewer::drawEvent: sensor's rendering target "
                   "cannot be nullptr.", );

    if (visualizeMode_ == VisualizeMode::Depth) {
      simulator_->visualizeObservation(defaultAgentId_, sensorVisID_,
                                       1.0f / 512.0f,  // colorMapOffset
                                       1.0f / 12.0f);  // colorMapScale
    } else if (visualizeMode_ == VisualizeMode::Semantic) {
      Mn::GL::defaultFramebuffer.clear(Mn::GL::FramebufferClear::Color |
                                       Mn::GL::FramebufferClear::Depth);
      simulator_->visualizeObservation(defaultAgentId_, sensorVisID_);
    }
    sensorRenderTarget->blitRgbaToDefault();
  }

  // Immediately bind the main buffer back so that the "imgui" below can work
  // properly
  Mn::GL::defaultFramebuffer.bind();

  // Do not include ImGui content drawing in per frame profiler measurements
  profiler_.endFrame();

  imgui_.newFrame();
  ImGui::SetNextWindowPos(ImVec2(10, 10));
  if (showFPS_) {
    ImGui::Begin("main", NULL,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(1.5);
    ImGui::Text("%.1f FPS", Mn::Double(ImGui::GetIO().Framerate));
    uint32_t total = activeSceneGraph_->getDrawables().size();
    ImGui::Text("%u drawables", total);
    if (sensorMode_ == VisualSensorMode::Camera) {
      ImGui::Text("%u culled", total - visibles);
    }
    auto& cam = getAgentCamera();

    switch (sensorMode_) {
      case VisualSensorMode::Camera:
        if (cam.getCameraType() == esp::sensor::SensorSubType::Orthographic) {
          ImGui::Text("Orthographic camera sensor");
        } else if (cam.getCameraType() == esp::sensor::SensorSubType::Pinhole) {
          ImGui::Text("Pinhole camera sensor");
        };
        break;
      case VisualSensorMode::Fisheye:
        ImGui::Text("Fisheye sensor");
        break;
      case VisualSensorMode::Equirectangular:
        ImGui::Text("Equirectangular sensor");
        break;

      default:
        break;
    }
    ImGui::Text("%s", profiler_.statistics().c_str());
    std::string modeText =
        "Mouse Interaction Mode: " + mouseModeNames.at(mouseInteractionMode);
    ImGui::Text("%s", modeText.c_str());
    if (!semanticTag_.empty()) {
      ImGui::Text("Semantic %s", semanticTag_.c_str());
    }
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
}  // Viewer::drawEvent()

void Viewer::dispMetadataInfo() {  // display info report
  std::string dsInfoReport = MM_->createDatasetReport();
  ESP_DEBUG() << "\nActive Dataset Details : \n"
              << dsInfoReport << "\nActive Dataset Report Details Done";
}

void Viewer::moveAndLook(int repetitions) {
  for (int i = 0; i < repetitions; ++i) {
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

  // update the grabber transform when agent is moved
  if (mouseGrabber_ != nullptr) {
    // GRAB mode, move the constraint
    auto ray = renderCamera_->unproject(previousMousePoint);
    mouseGrabber_->updateTransform(
        Mn::Matrix4::from(defaultAgent_->node().rotation().toMatrix(),
                          renderCamera_->node().absoluteTranslation() +
                              ray.direction * mouseGrabber_->gripDepth));
  }
}

void Viewer::bindRenderTarget() {
  for (auto& it : agentBodyNode_->getSubtreeSensors()) {
    if (it.second.get().isVisualSensor()) {
      esp::sensor::VisualSensor& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(it.second.get());
      if (visualizeMode_ == VisualizeMode::Depth ||
          visualizeMode_ == VisualizeMode::Semantic) {
        simulator_->getRenderer()->bindRenderTarget(
            visualSensor, {esp::gfx::Renderer::Flag::VisualizeTexture});
      } else {
        simulator_->getRenderer()->bindRenderTarget(visualSensor);
      }
    }  // if
  }    // for
}

void Viewer::viewportEvent(ViewportEvent& event) {
  for (auto& it : agentBodyNode_->getSubtreeSensors()) {
    if (it.second.get().isVisualSensor()) {
      esp::sensor::VisualSensor& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(it.second.get());
      visualSensor.setResolution(event.framebufferSize()[1],
                                 event.framebufferSize()[0]);
      renderCamera_->setViewport(visualSensor.framebufferSize());
      // before, here we will bind the render target, but now we defer it
      if (visualSensor.specification()->sensorSubType ==
          esp::sensor::SensorSubType::Fisheye) {
        auto spec = static_cast<esp::sensor::FisheyeSensorDoubleSphereSpec*>(
            visualSensor.specification().get());

        // const auto viewportSize =
        // Mn::GL::defaultFramebuffer.viewport().size();
        const auto viewportSize = event.framebufferSize();
        int size = viewportSize[0] < viewportSize[1] ? viewportSize[0]
                                                     : viewportSize[1];
        // since the sensor is determined, sensor's focal length is fixed.
        spec->principalPointOffset =
            Mn::Vector2(viewportSize[0] / 2, viewportSize[1] / 2);
      }
    }
  }
  bindRenderTarget();
  Mn::GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

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

void Viewer::buildSemanticPrims(int semanticID,
                                const std::string& objTag,
                                const Mn::Vector3& semanticCtr,
                                const Mn::Vector3& semanticSize,
                                const Mn::Quaternion& rotation) {
  auto rigidObjMgr = simulator_->getRigidObjectManager();
  if (semanticBBID_ != -1) {
    // delete semantic wireframe bounding box if it exists
    // returns nullptr if dne
    auto obj = rigidObjMgr->getObjectOrCopyByID(semanticBBID_);
    // delete object template used to create bounding box
    objectAttrManager_->removeObjectByHandle(
        obj->getInitializationAttributes()->getHandle());

    rigidObjMgr->removeObjectByID(semanticBBID_);
    semanticBBID_ = -1;
  }
  ESP_WARNING() << "Object ID : " << semanticID << " Tag : " << objTag
                << " : Center : " << semanticCtr
                << " | Size : " << semanticSize;
  // // build semantic wireframe bounding box

  auto bbWfObjTemplate = objectAttrManager_->getObjectCopyByHandle(
      MM_->getAssetAttributesManager()
          ->getDefaultCubeTemplate(true)
          ->getHandle());
  // modify template to have appropriate scale and new handle, and then register
  bbWfObjTemplate->setScale(semanticSize * .5f);
  bbWfObjTemplate->setHandle(Cr::Utility::formatString(
      "{}_semanticBB_{}", bbWfObjTemplate->getHandle(), semanticID));
  bbWfObjTemplate->setIsCollidable(false);
  objectAttrManager_->registerObject(bbWfObjTemplate);
  // create bounding box wireframe
  auto bbWfObj = rigidObjMgr->addObjectByHandle(bbWfObjTemplate->getHandle());
  bbWfObj->setTranslation(semanticCtr);
  bbWfObj->setRotation(rotation);
  bbWfObj->setMotionType(esp::physics::MotionType::STATIC);
  semanticBBID_ = bbWfObj->getID();
}  // Viewer::buildSemanticPrims

void Viewer::mousePressEvent(MouseEvent& event) {
  // get mouse position, appropriately scaled for Retina Displays
  auto viewportPoint = getMousePosition(event.position());
  if (mouseInteractionMode == MouseInteractionMode::LOOK) {
    if (event.button() == MouseEvent::Button::Right) {
      // if shift pressed w/right click in look mode, get object ID and
      // create visualization
      if (event.modifiers() & MouseEvent::Modifier::Shift) {
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

        // Read the object Id - takes unscaled mouse position, and scales it in
        // objectPicker
        unsigned int pickedObject =
            objectPickingHelper_->getObjectId(event.position(), windowSize());

        // if an object is selected, create a visualizer
        createPickedObjectVisualizer(pickedObject);
        return;
      }  // drawable selection
      // add primitive w/ right click if a collision object is hit by a raycast
      if (simulator_->getPhysicsSimulationLibrary() !=
          esp::physics::PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
        auto ray = renderCamera_->unproject(viewportPoint);
        esp::physics::RaycastResults raycastResults = simulator_->castRay(ray);

        if (raycastResults.hasHits()) {
          // If VHACD is enabled, and Ctrl + Right Click is used, voxelized
          // the object clicked on.
#ifdef ESP_BUILD_WITH_VHACD
          if (event.modifiers() & MouseEvent::Modifier::Ctrl) {
            auto objID = raycastResults.hits[0].objectId;
            displayVoxelField(objID);
            return;
          }
#endif
          addPrimitiveObject();

          auto existingObjectIDs = simulator_->getExistingObjectIDs();
          // use the bounding box to create a safety margin for adding the
          // object
          auto rigidObjMgr = simulator_->getRigidObjectManager();
          auto obj = rigidObjMgr->getObjectByID(existingObjectIDs.back());
          float boundingBuffer =
              obj->getSceneNode()->computeCumulativeBB().size().max() / 2.0 +
              0.04;
          obj->setTranslation(raycastResults.hits[0].point +
                              raycastResults.hits[0].normal * boundingBuffer);

          obj->setRotation(esp::core::randomRotation());
        }
      }
      // end add primitive w/ right click
    } else if (event.button() == MouseEvent::Button::Left) {
      // if shift-click is pressed, display semantic ID and name if exists
      if (event.modifiers() & MouseEvent::Modifier::Shift) {
        semanticTag_ = "";

        // get semantic scene
        auto semanticScene = simulator_->getSemanticScene();
        // only enable for HM3D, MP3D and Replica for now
        if ((semanticScene) && (semanticScene->hasVertColorsDefined())) {
          auto semanticObjects = semanticScene->objects();
          std::string sensorId = "semantic_camera";
          esp::sensor::Observation observation;
          simulator_->getAgentObservation(defaultAgentId_, sensorId,
                                          observation);

          uint32_t desiredIdx =
              (viewportPoint[0] +
               (observation.buffer->shape[1] *
                (observation.buffer->shape[0] - viewportPoint[1])));
          uint32_t objIdx = Corrade::Containers::arrayCast<int>(
              observation.buffer->data)[desiredIdx];
          // TODO : Change core::buffer to magnum image, then we can simplify
          // access uint32_t objIdx =
          // observation.buffer->pixels<uint32_t>().flipped<0>()[viewportPoint[1]][viewportPoint[0]];

          // subtract 1 to align with semanticObject array
          //--objIdx;
          std::string tmpStr = "Unknown";

          if ((objIdx >= 0) && (objIdx < semanticObjects.size())) {
            const auto& semanticObj = semanticObjects[objIdx];
            tmpStr = semanticObj->id();
            const auto obb = semanticObj->obb();
            // get center and scale of bb and use to build visualization reps
            buildSemanticPrims(objIdx, tmpStr, Mn::Vector3{obb.center()},
                               Mn::Vector3{obb.sizes()},
                               Mn::Quaternion{obb.rotation()});
          }
          semanticTag_ = Cr::Utility::formatString("id:{}:{}", objIdx, tmpStr);
        }
      }
    }
  } else if (mouseInteractionMode == MouseInteractionMode::GRAB) {
    // GRAB mode
    if (simulator_->getPhysicsSimulationLibrary() !=
        esp::physics::PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
      auto ray = renderCamera_->unproject(viewportPoint);
      esp::physics::RaycastResults raycastResults = simulator_->castRay(ray);

      if (raycastResults.hasHits()) {
        int hitObject = esp::ID_UNDEFINED;
        Mn::Quaternion objectFrame;
        Mn::Vector3 objectPivot;
        int aoLink = esp::ID_UNDEFINED;
        auto hitInfo = raycastResults.hits[0];  // first hit
        // check if ao
        if (hitInfo.objectId != esp::ID_UNDEFINED) {
          // we hit an non-stage collision object
          auto roMngr = simulator_->getRigidObjectManager();
          auto aoMngr = simulator_->getArticulatedObjectManager();
          auto ro = roMngr->getObjectByID(hitInfo.objectId);
          auto ao = aoMngr->getObjectByID(hitInfo.objectId);
          if (ro != nullptr) {
            // grabbed an object
            hitObject = hitInfo.objectId;
            objectPivot = ro->getTransformation().inverted().transformPoint(
                hitInfo.point);
            objectFrame = ro->getRotation().inverted();
          } else if (ao != nullptr) {
            // grabbed the base link
            hitObject = hitInfo.objectId;
            objectPivot = ao->getTransformation().inverted().transformPoint(
                hitInfo.point);
            objectFrame = ao->getRotation().inverted();
          } else {
            for (auto aoHandle : aoMngr->getObjectHandlesBySubstring()) {
              auto ao = aoMngr->getObjectByHandle(aoHandle);
              auto linkToObjIds = ao->getLinkObjectIds();
              const auto aoLinkIter = linkToObjIds.find(hitInfo.objectId);
              if (aoLinkIter != linkToObjIds.end()) {
                // got a link
                aoLink = aoLinkIter->second;
                objectPivot = ao->getLinkSceneNode(aoLink)
                                  ->transformation()
                                  .inverted()
                                  .transformPoint(hitInfo.point);
                objectFrame =
                    ao->getLinkSceneNode(aoLink)->rotation().inverted();
                hitObject = ao->getID();
                break;
              }
            }
          }  // done checking for AO

          if (hitObject >= 0) {
            esp::physics::RigidConstraintSettings constraintSettings;
            constraintSettings.objectIdA = hitObject;
            constraintSettings.linkIdA = aoLink;
            constraintSettings.pivotA = objectPivot;
            constraintSettings.frameA =
                objectFrame.toMatrix() *
                defaultAgent_->node().rotation().toMatrix();
            constraintSettings.frameB =
                defaultAgent_->node().rotation().toMatrix();
            constraintSettings.pivotB = hitInfo.point;
            // by default use a point 2 point constraint
            if (event.button() == MouseEvent::Button::Right) {
              constraintSettings.constraintType =
                  esp::physics::RigidConstraintType::Fixed;
            }
            mouseGrabber_ = std::make_unique<MouseGrabber>(
                constraintSettings,
                (hitInfo.point - renderCamera_->node().absoluteTranslation())
                    .length(),
                *simulator_);
          } else {
            ESP_DEBUG() << "Oops, couldn't find the hit object. That's odd.";
          }
        }  // end didn't hit the scene
      }    // end has raycast hit
    }      // end has physics enabled
  }        // end GRAB

  previousMousePoint = viewportPoint;
  event.setAccepted();
  redraw();
}

void Viewer::mouseReleaseEvent(MouseEvent& event) {
  // release any existing mouse constraint
  mouseGrabber_ = nullptr;
  event.setAccepted();
}

void Viewer::mouseScrollEvent(MouseScrollEvent& event) {
  // shift+scroll is forced into x direction on mac, seemingly at OS level,
  // so use both x and y offsets.
  float scrollModVal = abs(event.offset().y()) > abs(event.offset().x())
                           ? event.offset().y()
                           : event.offset().x();
  if (!(scrollModVal)) {
    return;
  }
  // Use shift to scale action response
  auto shiftPressed = event.modifiers() & MouseEvent::Modifier::Shift;
  auto altPressed = event.modifiers() & MouseEvent::Modifier::Alt;
  auto ctrlPressed = event.modifiers() & MouseEvent::Modifier::Ctrl;
  if (mouseInteractionMode == MouseInteractionMode::LOOK) {
    // Use shift for fine-grained zooming
    float modVal = shiftPressed ? 1.01 : 1.1;
    float mod = scrollModVal > 0 ? modVal : 1.0 / modVal;
    auto& cam = getAgentCamera();
    cam.modifyZoom(mod);
    redraw();
  } else if (mouseInteractionMode == MouseInteractionMode::GRAB &&
             mouseGrabber_ != nullptr) {
    auto viewportPoint = getMousePosition(event.position());
    // adjust the depth
    float modVal = shiftPressed ? 0.1 : 0.01;
    float scrollDelta = scrollModVal * modVal;
    if (altPressed || ctrlPressed) {
      // rotate the object's local constraint frame
      auto agentT = agentBodyNode_->transformationMatrix();

      // ALT - yaw
      Mn::Vector3 rotationAxis = agentT.transformVector(Mn::Vector3(0, 1, 0));
      if (altPressed && ctrlPressed) {
        // ALT+CTRL - roll
        rotationAxis = agentT.transformVector(Mn::Vector3(0, 0, -1));
      } else if (ctrlPressed) {
        // CTRL - pitch
        rotationAxis = agentT.transformVector(Mn::Vector3(1, 0, 0));
      }
      mouseGrabber_->rotateLocalFrameByGlobalAngleAxis(rotationAxis,
                                                       Mn::Rad(scrollDelta));
    } else {
      // translate the object forward/backward
      auto ray = renderCamera_->unproject(viewportPoint);
      mouseGrabber_->gripDepth += scrollDelta;
      mouseGrabber_->updateTransform(
          Mn::Matrix4::from(defaultAgent_->node().rotation().toMatrix(),
                            renderCamera_->node().absoluteTranslation() +
                                ray.direction * mouseGrabber_->gripDepth));
    }
  }

  event.setAccepted();
}  // Viewer::mouseScrollEvent

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  if ((mouseInteractionMode == MouseInteractionMode::LOOK) &&
      (!(event.buttons() & MouseMoveEvent::Button::Left))) {
    return;
  }
  auto viewportPoint = getMousePosition(event.position());
  if (mouseInteractionMode == MouseInteractionMode::LOOK) {
    const Mn::Vector2i delta = event.relativePosition();
    auto& controls = *defaultAgent_->getControls().get();
    controls(*agentBodyNode_, "turnRight", delta.x());
    // apply the transformation to all sensors
    for (auto& p : agentBodyNode_->getSubtreeSensors()) {
      controls(p.second.get().object(),  // SceneNode
               "lookDown",               // action name
               delta.y(),                // amount
               false);                   // applyFilter
    }
  } else if (mouseInteractionMode == MouseInteractionMode::GRAB &&
             mouseGrabber_ != nullptr) {
    // GRAB mode, move the constraint
    auto ray = renderCamera_->unproject(viewportPoint);
    mouseGrabber_->updateTransform(
        Mn::Matrix4::from(defaultAgent_->node().rotation().toMatrix(),
                          renderCamera_->node().absoluteTranslation() +
                              ray.direction * mouseGrabber_->gripDepth));
  }

  redraw();
  previousMousePoint = viewportPoint;
  event.setAccepted();
}  // Viewer::mouseMoveEvent

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
    case KeyEvent::Key::Tab:
      if (event.modifiers() & MouseEvent::Modifier::Alt) {
        setSceneInstanceFromListAndShow(curSceneInstanceIDX_);
      } else {
        ESP_DEBUG() << "Cycling to"
                    << ((event.modifiers() & MouseEvent::Modifier::Shift)
                            ? "previous"
                            : "next")
                    << "SceneInstance";
        setSceneInstanceFromListAndShow(getNextSceneInstanceIDX(
            (event.modifiers() & MouseEvent::Modifier::Shift) ? -1 : 1));
      }
      break;
    case KeyEvent::Key::Space:
      simulating_ = !simulating_;
      ESP_DEBUG() << "Physics Simulation cycling from" << !simulating_ << "to"
                  << simulating_;
      break;
    case KeyEvent::Key::Period:
      // also `>` key
      simulateSingleStep_ = true;
      break;
    case KeyEvent::Key::Comma:
      debugBullet_ = !debugBullet_;
      break;
      // ==== Miscellaneous ====
    case KeyEvent::Key::One:
      // toggle agent location recording for trajectory
      setAgentLocationRecord(!agentLocRecordOn_);
      break;
    case KeyEvent::Key::Two:
      // agent motion trajectory mesh synthesis with random color
      buildTrajectoryVis();
      break;
    case KeyEvent::Key::Three:
      // toggle between single color and multi-color trajectories
      singleColorTrajectory_ = !singleColorTrajectory_;
      ESP_DEBUG() << (singleColorTrajectory_
                          ? "Building trajectory with multiple random colors "
                            "changed to single random color."
                          : "Building trajectory with single random color "
                            "changed to multiple random colors.");
      break;
    case KeyEvent::Key::Four:
      // switch between camera types (Camera, Fisheye, Equirectangular)
      sensorMode_ = static_cast<VisualSensorMode>(
          (uint8_t(sensorMode_) + 1) %
          uint8_t(VisualSensorMode::VisualSensorModeCount));
      setSensorVisID();
      break;
    case KeyEvent::Key::Five:
      // switch camera between ortho and perspective
      switchCameraType();
      break;
    case KeyEvent::Key::Six:
      // reset camera zoom
      getAgentCamera().resetZoom();
      break;
    case KeyEvent::Key::Seven:
      visualizeMode_ = static_cast<VisualizeMode>(
          (uint8_t(visualizeMode_) + 1) %
          uint8_t(VisualizeMode::VisualizeModeCount));
      setSensorVisID();
      bindRenderTarget();
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
    case KeyEvent::Key::LeftBracket:
      saveAgentAndSensorTransformToFile();
      break;
    case KeyEvent::Key::RightBracket:
      loadAgentAndSensorTransformFromFile();
      break;
    case KeyEvent::Key::Equal: {
      // increase trajectory tube diameter
      ESP_DEBUG() << "Bigger";
      modTrajRad(true);
      break;
    }
    case KeyEvent::Key::Minus: {
      // decrease trajectory tube diameter
      ESP_DEBUG() << "Smaller";
      modTrajRad(false);
      break;
    }
    case KeyEvent::Key::B: {
      // toggle bounding box on objects
      drawObjectBBs = !drawObjectBBs;
      for (auto id : simulator_->getExistingObjectIDs()) {
        simulator_->setObjectBBDraw(drawObjectBBs, id);
      }
    } break;
    case KeyEvent::Key::C:
      showFPS_ = !showFPS_;
      showFPS_ ? profiler_.enable() : profiler_.disable();
      break;
    case KeyEvent::Key::E:
      simulator_->setFrustumCullingEnabled(
          !simulator_->isFrustumCullingEnabled());
      break;
    case KeyEvent::Key::H:
      printHelpText();
      break;
    case KeyEvent::Key::I:
      screenshot();
      break;
    case KeyEvent::Key::M: {
      // toggle the mouse interaction mode
      mouseInteractionMode = MouseInteractionMode(
          (int(mouseInteractionMode) + 1) % int(NUM_MODES));
    } break;
    case KeyEvent::Key::N:
      // toggle navmesh visualization
      simulator_->setNavMeshVisualization(
          !simulator_->isNavMeshVisualizationActive());
      break;
    case KeyEvent::Key::O:
      addTemplateObject();
      break;
    case KeyEvent::Key::P:
      // save current sim state
      simulator_->saveCurrentSceneInstance();
      break;
    case KeyEvent::Key::Slash:
      // display current scene's metadata information
      dispMetadataInfo();
      break;
    case KeyEvent::Key::Q:
      // query the agent state
      showAgentStateMsg(true, true);
      break;
    case KeyEvent::Key::J: {
      if (event.modifiers() & MouseEvent::Modifier::Alt) {
        generateAndSaveAllSemanticCCReports();
      } else {
        // generate and save semantic CC report
        generateAndSaveSemanticCCReport();
      }
      break;
    }
    case KeyEvent::Key::Y: {
      if (event.modifiers() & MouseEvent::Modifier::Alt) {
        generateAndSaveAllVertColorMapReports();
      } else {
        // generate and save semantic CC report
        generateAndSaveVertColorMapReports();
      }
      break;
    }
    case KeyEvent::Key::T: {
      //+ALT for fixedBase
      bool fixedBase = bool(event.modifiers() & MouseEvent::Modifier::Alt);

      // add an ArticulatedObject from provided filepath
      std::string urdfFilepath;
      if (event.modifiers() & MouseEvent::Modifier::Shift &&
          !cachedURDF_.empty()) {
        // quick-reload the most recently loaded URDF
        ESP_DEBUG() << "URDF quick-reload: " << cachedURDF_;
        urdfFilepath = cachedURDF_;
      } else {
        ESP_DEBUG() << "Load URDF: provide a URDF filepath.";
        std::cin >> urdfFilepath;
      }

      if (urdfFilepath.empty()) {
        ESP_DEBUG() << "... no input provided. Aborting.";
      } else if (!Cr::Utility::String::endsWith(urdfFilepath, ".urdf") &&
                 !Cr::Utility::String::endsWith(urdfFilepath, ".URDF")) {
        ESP_DEBUG() << "... input is not a URDF. Aborting.";
      } else if (Cr::Utility::Path::exists(urdfFilepath)) {
        // cache the file for quick-reload with SHIFT-T
        cachedURDF_ = urdfFilepath;
        auto aom = simulator_->getArticulatedObjectManager();
        auto ao = aom->addArticulatedObjectFromURDF(urdfFilepath, fixedBase,
                                                    1.0, 1.0, true);
        ao->setTranslation(
            defaultAgent_->node().transformation().transformPoint(
                {0, 1.0, -1.5}));
      } else {
        ESP_DEBUG() << "... input file not found. Aborting.";
      }
    } break;
    case KeyEvent::Key::L: {
      // override the default light setup with the config in this directory
      auto& lightLayoutMgr = simulator_->getLightLayoutAttributesManager();
      auto loadedLayout = lightLayoutMgr->createObjectFromJSONFile(
          "src/utils/viewer/default_light_override.lighting_config.json");
      lightLayoutMgr->registerObject(loadedLayout, "default_override");
      simulator_->setLightSetup(
          lightLayoutMgr->createLightSetupFromAttributes("default_override"));
    } break;
    case KeyEvent::Key::U:
      removeLastObject();
      break;
    case KeyEvent::Key::V:
      invertGravity();
      break;
    case KeyEvent::Key::K: {
#ifdef ESP_BUILD_WITH_VHACD
      iterateAndDisplaySignedDistanceField();
      // Increase the distance visualized for next time (Pressing L
      // repeatedly will visualize different distances)
      voxelDistance++;
#endif
      break;
    }
    case KeyEvent::Key::G: {
#ifdef ESP_BUILD_WITH_VHACD
      displayStageDistanceGradientField();
#endif
      break;
    }
    case KeyEvent::Key::F: {
#ifdef ESP_BUILD_WITH_AUDIO
      // Add an audio source
      addAudioSource();
#else
      ESP_DEBUG() << "[Audio] ESP_BUILD_WITH_AUDIO is not set, skipping adding "
                     "audio source";
#endif  // ESP_BUILD_WITH_AUDIO
      break;
    }
    case KeyEvent::Key::Zero: {
#ifdef ESP_BUILD_WITH_AUDIO
      // Run audio simulation
      runAudioSimulation();
#else
      ESP_DEBUG() << "[Audio] ESP_BUILD_WITH_AUDIO is not set, skipping "
                     "running audio simulation";
#endif  // ESP_BUILD_WITH_AUDIO
      break;
    }
  }

  // Update map of moving/looking keys which are currently pressed
  auto keyPressedIter = keysPressed.find(key);
  if (keyPressedIter != keysPressed.end()) {
    keyPressedIter->second = true;
  }
  redraw();
}

void Viewer::keyReleaseEvent(KeyEvent& event) {
  // Update map of moving/looking keys which are currently pressed
  const auto key = event.key();
  auto keyPressedIter = keysPressed.find(key);
  if (keyPressedIter != keysPressed.end()) {
    keyPressedIter->second = false;
  }
  redraw();
}

int savedFrames = 0;
//! Save a screenshot to
//! "screenshots/year_month_day_hour-minute-second/#.png"
void Viewer::screenshot() {
  std::string screenshot_directory =
      "screenshots/" + viewerStartTimeString + "/";
  if (!Cr::Utility::Path::exists(screenshot_directory)) {
    Cr::Utility::Path::make(screenshot_directory);
  }
  Mn::DebugTools::screenshot(
      Mn::GL::defaultFramebuffer,
      screenshot_directory + std::to_string(savedFrames++) + ".png");
}  // Viewer::screenshot

#ifdef ESP_BUILD_WITH_AUDIO
void Viewer::addAudioSource() {
  ESP_DEBUG() << "[Audio] Adding audio source";
  // Add an audio source in front of the agent
  addPrimitiveObject();
  Mn::Matrix4 T = agentBodyNode_->MagnumObject::transformationMatrix();
  Mn::Vector3 new_pos = T.transformPoint({0.1f, 1.5f, -2.0f});

  esp::sensor::AudioSensor& audioSensor = getAgentAudioSensor();

  audioSensor.setAudioSourceTransform({new_pos[0], new_pos[1], new_pos[2]});
}

void Viewer::runAudioSimulation() {
  ESP_DEBUG() << "[Audio] Running audio simulation";
  // Run the audio simulation code to generate the impulse response
  Mn::Matrix4 T = agentBodyNode_->MagnumObject::transformationMatrix();
  Mn::Vector3 pos = T.transformPoint({0.0f, 0.0f, 0.0f});
  auto rotScalar = agentBodyNode_->rotation().scalar();
  auto rotVec = agentBodyNode_->rotation().vector();

  esp::sensor::AudioSensor& audioSensor = getAgentAudioSensor();
  audioSensor.setAudioListenerTransform(
      {pos[0], pos[1], pos[2]}, {rotScalar, rotVec[0], rotVec[1], rotVec[2]});
  audioSensor.runSimulation(*simulator_);
  esp::sensor::Observation obs;
  const bool success = audioSensor.getObservation(*simulator_, obs);

  if (success) {
    // obs should be populated, log the sizes to sanity check everything works
    ESP_DEBUG() << "[Audio] RESULTS : Observation Space : "
                << obs.buffer->shape[0] << ", " << obs.buffer->shape[1];
  } else {
    ESP_ERROR() << "[Audio] Audio simulation was unsuccessful";
  }
}
#endif  // ESP_BUILD_WITH_AUDIO

}  // namespace

MAGNUM_APPLICATION_MAIN(Viewer)
