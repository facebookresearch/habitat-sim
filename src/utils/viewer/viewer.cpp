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

#ifdef ESP_BUILD_WITH_VHACD
#include "esp/geo/VoxelUtils.h"
#endif

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
  void pokeLastObject();
  void pushLastObject();
  void torqueLastObject();
  void removeLastObject();
  void wiggleLastObject();
  void invertGravity();

#ifdef ESP_BUILD_WITH_VHACD
  void displayStageDistanceGradientField();

  void iterateAndDisplaySignedDistanceField();

  void displayVoxelField(int objectID);

  int objectDisplayed = -1;
#endif

  bool isInRange(float val);
  /**
   * @brief Toggle between ortho and perspective camera
   */
  void switchCameraType();
  Mn::Vector3 randomDirection();

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
  CTRL-RIGHT:
    (With 'enable-physics') Click on an object to voxelize it and display the voxelization.
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
  '1' toggle recording locations for trajectory visualization.
  '2' build and display trajectory visualization.
  '+' increase trajectory diameter
  '-' decrease trajectory diameter
  '3' toggle flying camera mode (user can apply camera transformation loaded from disk)
  '5' switch ortho/perspective camera.
  '6' reset ortho camera zoom/perspective camera FOV.
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
  'g': (physics) Display a stage's signed distance gradient vector field.
  'l': (physics) Iterate through different ranges of the stage's voxelized signed distance field.
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

#ifdef ESP_BUILD_WITH_VHACD
  //! The slice of the grid's SDF to visualize.
  int voxelDistance = 0;
#endif
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

  void bindRenderTarget();
};

void addSensors(esp::agent::AgentConfiguration& agentConfig,
                const Cr::Utility::Arguments& args) {
  const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();

  auto addCameraSensor = [&](const std::string& uuid,
                             esp::sensor::SensorType sensorType) {
    agentConfig.sensorSpecifications.emplace_back(
        esp::sensor::CameraSensorSpec::create());
    auto spec = static_cast<esp::sensor::CameraSensorSpec*>(
        agentConfig.sensorSpecifications.back().get());

    spec->uuid = uuid;
    spec->sensorSubType = args.isSet("orthographic")
                              ? esp::sensor::SensorSubType::Orthographic
                              : esp::sensor::SensorSubType::Pinhole;
    spec->sensorType = sensorType;
    if (sensorType == esp::sensor::SensorType::Depth ||
        sensorType == esp::sensor::SensorType::Semantic) {
      spec->channels = 1;
    }
    spec->position = {0.0f, 1.5f, 0.0f};
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
}

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
      .addOption("object-dir", "data/objects/example_objects")
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
      .addOption("agent-transform-filepath")
      .setHelp("agent-transform-filepath",
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

  agentTransformLoadPath_ = args.value("agent-transform-filepath");
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
  objectAttrManager_->loadAllJSONConfigsFromPath(args.value("object-dir"));
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

  addSensors(agentConfig, args);
  // add selects a random initial state and sets up the default controls and
  // step filter
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
    LOG(ERROR) << "Cannot open " << filename << " to output data.";
    return;
  }

  // saving transformation into file system
  auto save = [&](const Mn::Matrix4 transform) {
    const float* t = transform.data();
    for (int i = 0; i < 16; ++i) {
      file << t[i] << " ";
    }
    LOG(INFO) << "Transformation matrix saved to " << filename << " : "
              << Eigen::Map<const esp::mat4f>(transform.data());
  };
  save(agentTransform);
  save(sensorTransform);

  file.close();
}
void Viewer::saveAgentAndSensorTransformToFile() {
  const char* saved_transformations_directory = "saved_transformations/";
  if (!Cr::Utility::Directory::exists(saved_transformations_directory)) {
    Cr::Utility::Directory::mkpath(saved_transformations_directory);
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
    LOG(ERROR) << "Cannot open " << filename << " to load data.";
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
      LOG(WARNING) << "Data loaded from " << filename
                   << " is not a valid rigid transformation.";
      return false;
    }
    transform = temp;
    LOG(INFO) << "Transformation matrix loaded from " << filename << " : "
              << Eigen::Map<esp::mat4f>(transform.data());
    return true;
  };
  // NOTE: load Agent first!!
  bool status = load(agentTransform) && load(sensorTransform);
  file.close();
  return status;
}

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
    LOG(INFO)
        << "Camera transform file not specified, attempting to load from "
           "current instance. Use --agent-transform-filepath to specify file "
           "to load from.";
    if (!savedAgentTransform_ || !savedSensorTransform_) {
      LOG(INFO) << "Well, no transformation saved in current instance. nothing "
                   "is changed.";
      return;
    }
  }

  defaultAgent_->node().setTransformation(*savedAgentTransform_);
  for (const auto& p : defaultAgent_->node().getNodeSensors()) {
    p.second.get().object().setTransformation(*savedSensorTransform_);
  }
  LOG(INFO)
      << "Transformation matrices are loaded to the agent and the sensors.";
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

void Viewer::buildTrajectoryVis() {
  if (agentLocs_.size() < 2) {
    LOG(WARNING) << "::buildTrajectoryVis : No recorded trajectory "
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

  LOG(INFO) << "::buildTrajectoryVis : Attempting to build trajectory "
               "tube for :"
            << agentLocs_.size() << " points.";
  int trajObjID = simulator_->addTrajectoryObject(
      trajObjName, agentLocs_, 6, agentTrajRad_, color, true, 10);
  if (trajObjID != esp::ID_UNDEFINED) {
    LOG(INFO) << "::buildTrajectoryVis : Success!  Traj Obj Name : "
              << trajObjName << " has object ID : " << trajObjID;
  } else {
    LOG(WARNING) << "::buildTrajectoryVis : Attempt to build trajectory "
                    "visualization "
                 << trajObjName << " failed; Returned ID_UNDEFINED.";
  }
}  // buildTrajectoryVis

void Viewer::removeLastObject() {
  auto existingObjectIDs = simulator_->getExistingObjectIDs();
  if (existingObjectIDs.size() == 0) {
    return;
  }
  simulator_->removeObject(existingObjectIDs.back());
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
  !Mn::Debug();
  if (stageVoxelization == nullptr) {
    simulator_->createStageVoxelization(2000000);
    stageVoxelization = simulator_->getStageVoxelization();
    esp::geo::generateEuclideanDistanceSDF(stageVoxelization,
                                           "ESignedDistanceField");
  }
  !Mn::Debug();

  // generate a vector field for the SDF gradient
  esp::geo::generateScalarGradientField(
      stageVoxelization, "ESignedDistanceField", "GradientField");
  // generate a mesh of the vector field with boolean isVectorField set to
  // true
  !Mn::Debug();

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

bool isTrue(bool val) {
  return val;
}

bool isHorizontal(Mn::Vector3 val) {
  return abs(val.normalized()[0]) * abs(val.normalized()[0]) +
             abs(val.normalized()[2]) * abs(val.normalized()[2]) <=
         0;
  // return val[0] == 1;
}

bool Viewer::isInRange(float val) {
  int curDistanceVisualization = 1 * (voxelDistance % 18);
  return val >= curDistanceVisualization - 1 && val < curDistanceVisualization;
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

  if (visualizeMode_ == VisualizeMode::Depth ||
      visualizeMode_ == VisualizeMode::Semantic) {
    // ================ Depth Visualization ==================================
    std::string sensorId = "depth_camera";
    if (visualizeMode_ == VisualizeMode::Depth) {
      if (sensorMode_ == VisualSensorMode::Fisheye) {
        sensorId = "depth_fisheye";
      } else if (sensorMode_ == VisualSensorMode::Equirectangular) {
        sensorId = "depth_equirectangular";
      }
    } else if (visualizeMode_ == VisualizeMode::Semantic) {
      sensorId = "semantic_camera";
      if (sensorMode_ == VisualSensorMode::Fisheye) {
        sensorId = "semantic_fisheye";
      } else if (sensorMode_ == VisualSensorMode::Equirectangular) {
        sensorId = "semantic_equirectangular";
      }
    }

    simulator_->drawObservation(defaultAgentId_, sensorId);
    esp::gfx::RenderTarget* sensorRenderTarget =
        simulator_->getRenderTarget(defaultAgentId_, sensorId);
    if (visualizeMode_ == VisualizeMode::Depth) {
      simulator_->visualizeObservation(defaultAgentId_, sensorId,
                                       1.0f / 512.0f,  // colorMapOffset
                                       1.0f / 12.0f);  // colorMapScale
    } else if (visualizeMode_ == VisualizeMode::Semantic) {
      simulator_->visualizeObservation(defaultAgentId_, sensorId,
                                       1.0f / 512.0f,  // colorMapOffset
                                       1.0f / 50.0f);  // colorMapScale
    }
    sensorRenderTarget->blitRgbaToDefault();
  } else {
    if (sensorMode_ == VisualSensorMode::Camera) {
      // ============= regular RGB with object picking =================
      // using polygon offset to increase mesh depth to avoid z-fighting with
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
    } else {
      // ================ NON-regular RGB sensor ==================
      std::string sensorId = "";
      if (sensorMode_ == VisualSensorMode::Fisheye) {
        sensorId = "rgba_fisheye";
      } else if (sensorMode_ == VisualSensorMode::Equirectangular) {
        sensorId = "rgba_equirectangular";
      } else {
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      }
      CORRADE_INTERNAL_ASSERT(!sensorId.empty());

      simulator_->drawObservation(defaultAgentId_, sensorId);
      esp::gfx::RenderTarget* sensorRenderTarget =
          simulator_->getRenderTarget(defaultAgentId_, sensorId);
      CORRADE_ASSERT(sensorRenderTarget,
                     "Error in Viewer::drawEvent: sensor's rendering target "
                     "cannot be nullptr.", );
      sensorRenderTarget->blitRgbaToDefault();
    }
  }

  // Immediately bind the main buffer back so that the "imgui" below can work
  // properly
  Mn::GL::defaultFramebuffer.bind();

  // Do not include ImGui content drawing in per frame profiler measurements
  profiler_.endFrame();

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
  // add primitive w/ right click if a collision object is hit by a raycast
  else if (event.button() == MouseEvent::Button::Right) {
    if (simulator_->getPhysicsSimulationLibrary() !=
        esp::physics::PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
      auto viewportPoint = event.position();
      auto ray = renderCamera_->unproject(viewportPoint);
      esp::physics::RaycastResults raycastResults = simulator_->castRay(ray);

      if (raycastResults.hasHits()) {
        // If VHACD is enabled, and Ctrl + Right Click is used, voxelized the
        // object clicked on.
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

        simulator_->setRotation(esp::core::randomRotation(),
                                existingObjectIDs.back());
      }
    }
  }  // end add primitive w/ right click

  event.setAccepted();
  redraw();
}

void Viewer::mouseReleaseEvent(MouseEvent& event) {
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
  // Use shift for fine-grained zooming
  float modVal = (event.modifiers() & MouseEvent::Modifier::Shift) ? 1.01 : 1.1;
  float mod = scrollModVal > 0 ? modVal : 1.0 / modVal;
  auto& cam = getAgentCamera();
  cam.modifyZoom(mod);
  redraw();

  event.setAccepted();
}  // Viewer::mouseScrollEvent

void Viewer::mouseMoveEvent(MouseMoveEvent& event) {
  if (!(event.buttons() & MouseMoveEvent::Button::Left)) {
    return;
  }

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

  redraw();

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
      // ==== Miscellaneous ====
    case KeyEvent::Key::One:
      // toggle agent location recording for trajectory
      setAgentLocationRecord(!agentLocRecordOn_);
      break;
    case KeyEvent::Key::Two:
      // agent motion trajectory mesh synthesis with random color
      buildTrajectoryVis();
      break;
    case KeyEvent::Key::Four:
      sensorMode_ = static_cast<VisualSensorMode>(
          (uint8_t(sensorMode_) + 1) %
          uint8_t(VisualSensorMode::VisualSensorModeCount));
      LOG(INFO) << "Sensor mode is set to " << int(sensorMode_);
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
      bindRenderTarget();
      switch (visualizeMode_) {
        case VisualizeMode::RGBA:
          LOG(INFO) << "Visualizing COLOR sensor observation.";
          break;
        case VisualizeMode::Depth:
          LOG(INFO) << "Visualizing DEPTH sensor observation.";
          break;
        case VisualizeMode::Semantic:
          LOG(INFO) << "Visualizing SEMANTIC sensor observation.";
          break;
        default:
          CORRADE_INTERNAL_ASSERT_UNREACHABLE();
          break;
      }
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
      LOG(INFO) << "Bigger";
      modTrajRad(true);
      break;
    }
    case KeyEvent::Key::Minus: {
      // decrease trajectory tube diameter
      LOG(INFO) << "Smaller";
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
    case KeyEvent::Key::F:
      pushLastObject();
      break;
    case KeyEvent::Key::H:
      printHelpText();
      break;
    case KeyEvent::Key::I:
      screenshot();
      break;
    case KeyEvent::Key::K:
      wiggleLastObject();
      break;
    case KeyEvent::Key::N:
      // toggle navmesh visualization
      simulator_->setNavMeshVisualization(
          !simulator_->isNavMeshVisualizationActive());
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
    case KeyEvent::Key::T:
      torqueLastObject();
      break;
    case KeyEvent::Key::U:
      removeLastObject();
      break;
    case KeyEvent::Key::V:
      invertGravity();
      break;
#ifdef ESP_BUILD_WITH_VHACD
    case KeyEvent::Key::L: {
      iterateAndDisplaySignedDistanceField();
      // Increase the distance visualized for next time (Pressing L repeatedly
      // will visualize different distances)
      voxelDistance++;
      break;
    }
    case KeyEvent::Key::G: {
      displayStageDistanceGradientField();
      break;
    }
#endif
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

}  // namespace

MAGNUM_APPLICATION_MAIN(Viewer)
