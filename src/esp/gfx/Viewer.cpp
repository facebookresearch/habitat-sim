// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <stdlib.h>
#include <chrono>

#include "Viewer.h"

#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/DebugTools/Screenshot.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <sophus/so3.hpp>
#include "Drawable.h"
#include "esp/core/esp.h"
#include "esp/io/io.h"

#include "esp/gfx/Simulator.h"
#include "esp/scene/SceneConfiguration.h"

#include "esp/gfx/configure.h"

#include "esp/nav/PathFinder.h"
#include "utils/datatool/SceneLoader.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

using namespace Magnum;
using namespace Math::Literals;
using namespace Corrade;

constexpr float moveSensitivity = 0.1f;
constexpr float lookSensitivity = 11.25f;
constexpr float rgbSensorHeight = 1.5f;

namespace esp {
namespace gfx {

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
      .addOption("physics-config", ESP_DEFAULT_PHYS_SCENE_CONFIG)
      .setHelp("physics-config", "physics scene config file")
      .parse(arguments.argc, arguments.argv);

  const auto viewportSize = GL::defaultFramebuffer.viewport().size();
  enablePhysics_ = args.isSet("enable-physics");
  std::string physicsConfigFilename = args.value("physics-config");
  if (!Utility::Directory::exists(physicsConfigFilename)) {
    LOG(ERROR)
        << physicsConfigFilename
        << " was not found, specify an existing file in --physics-config";
    std::exit(1);
  }

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
  const assets::AssetInfo info = assets::AssetInfo::fromPath(file);
  sceneInfo_ = info;

  if (enablePhysics_) {
    if (!resourceManager_.loadScene(info, physicsManager_, navSceneNode_,
                                    &drawables, physicsConfigFilename)) {
      LOG(ERROR) << "cannot load " << file;
      std::exit(1);
    }
  } else {
    if (!resourceManager_.loadScene(info, navSceneNode_, &drawables)) {
      LOG(ERROR) << "cannot load " << file;
      std::exit(1);
    }
  }

  // Set up camera
  renderCamera_ = &sceneGraph_->getDefaultRenderCamera();
  agentBodyNode_ = &rootNode_->createChild();
  rgbSensorNode_ = &agentBodyNode_->createChild();

  rgbSensorNode_->translate({0.0f, rgbSensorHeight, 0.0f});
  agentBodyNode_->translate({0.0f, 0.0f, 5.0f});

  float hfov = 90.0f;
  int width = viewportSize[0];
  int height = viewportSize[1];
  float znear = 0.01f;
  float zfar = 1000.0f;
  renderCamera_->setProjectionMatrix(width, height, znear, zfar, hfov);

  // Load navmesh if available
  const std::string navmeshFilename = io::changeExtension(file, ".navmesh");
  if (io::exists(navmeshFilename)) {
    LOG(INFO) << "Loading navmesh from " << navmeshFilename;
    pathfinder_->loadNavMesh(navmeshFilename);

    const vec3f position = pathfinder_->getRandomNavigablePoint();
    agentBodyNode_->setTranslation(Vector3(position));
  } else {
    LOG(INFO) << "trying to build navmesh from " << file;
    LOG(INFO) << "available collision meshes: ";
    for (int i = 0; i < resourceManager_.getNumLibraryObjects(); i++)
      LOG(INFO) << resourceManager_.getObjectConfig(i);

    // pathfinder_->build(nav::NavMeshSettings(),
    // resourceManager_.getCollisionMesh(0));
  }

  // connect controls to navmesh if loaded
  if (pathfinder_->isLoaded()) {
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
  assets::PhysicsObjectAttributes& poa =
      resourceManager_.getPhysicsObjectAttributes(configFile);
  // poa.setBool("useBoundingBoxForCollision", true);
  int physObjectID = physicsManager_->addObject(configFile, &drawables);
  physicsManager_->setTranslation(physObjectID, new_pos);

  // draw random quaternion via the method:
  // http://planning.cs.uiuc.edu/node198.html
  double u1 = (rand() % 1000) / 1000.0;
  double u2 = (rand() % 1000) / 1000.0;
  double u3 = (rand() % 1000) / 1000.0;

  Magnum::Vector3 qAxis(sqrt(1 - u1) * cos(2 * M_PI * u2),
                        sqrt(u1) * sin(2 * M_PI * u3),
                        sqrt(u1) * cos(2 * M_PI * u3));
  physicsManager_->setRotation(
      physObjectID,
      Magnum::Quaternion(qAxis, sqrt(1 - u1) * sin(2 * M_PI * u2)));
  objectIDs_.push_back(physObjectID);
  physicsManager_->toggleBBDraw(physObjectID, &drawables);

  // const Magnum::Vector3& trans =
  // physicsManager_->getTranslation(physObjectID); LOG(INFO) << "translation: "
  // << trans[0] << ", " << trans[1] << ", " << trans[2];
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
  Vector3 impulse = T.transformPoint({0.0f, 0.0f, -3.0f});
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyImpulse(objectIDs_.back(), impulse, rel_pos);
}

void Viewer::pushLastObject() {
  if (physicsManager_ == nullptr || objectIDs_.size() == 0)
    return;
  Magnum::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 force = T.transformPoint({0.0f, 0.0f, -40.0f});
  Vector3 rel_pos = Vector3(0.0f, 0.0f, 0.0f);
  physicsManager_->applyForce(objectIDs_.back(), force, rel_pos);
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

  physicsManager_->translate(objectIDs_.back(), randomDirection() * 0.1);
}

void Viewer::addPrimitiveDrawable(int primitiveID) {
  Magnum::Matrix4 T =
      agentBodyNode_
          ->MagnumObject::transformationMatrix();  // Relative to agent bodynode
  Vector3 new_pos = T.transformPoint({0.1f, 2.5f, -2.0f});

  Magnum::Color4 rand_color((float)((rand() % 1000) / 1000.0),
                            (float)((rand() % 1000) / 1000.0),
                            (float)((rand() % 1000) / 1000.0), 1.0);

  primitiveNodes_.push_back(&rootNode_->createChild());
  primitiveNodes_.back()->setTranslation(new_pos);
  primitiveNodes_.back()->setScaling(Magnum::Vector3{0.2});
  resourceManager_.addPrimitiveToDrawables(primitiveID, *primitiveNodes_.back(),
                                           &sceneGraph_->getDrawables(),
                                           rand_color);
}

void Viewer::removePrimitiveDrawable(int index) {
  if (index >= 0 && index < primitiveNodes_.size()) {
    delete primitiveNodes_[index];
    primitiveNodes_.erase(primitiveNodes_.begin() + index);
  }
}

void Viewer::highlightNavmeshIsland() {
  const vec3f position = pathfinder_->getRandomNavigablePoint();
  // vec3f position(-5.7589, 0.120596, 9.3611);
  // vec3f position(1.44672,0,-3.26787);
  // float rad = pathfinder_->islandRadius(position);
  float rad = pathfinder_->distanceToClosestObstacle(position);
  // LOG(INFO) << "Location: " << position << ", radius: " << rad;
  // addPrimitiveDrawable(esp::assets::ResourceManager::AvailablePrimitives::WIRE_CYLINDER);
  addPrimitiveDrawable(
      esp::assets::ResourceManager::AvailablePrimitives::SOLID_SPHERE);
  primitiveNodes_.back()->setTranslation(Magnum::Vector3(position));
  primitiveNodes_.back()->setScaling(Magnum::Vector3{rad, 0.2, rad});
  /*
  int numObjects = resourceManager_.getNumLibraryObjects();
  if (!numObjects) {
    return;
  }
  std::string configFile =
  resourceManager_.getObjectConfig(sequentialItemSpawnID_); auto& drawables =
  sceneGraph_->getDrawables(); assets::PhysicsObjectAttributes poa =
      resourceManager_.getPhysicsObjectAttributes(configFile);
  int physObjectID = physicsManager_->addObject(configFile, &drawables);
  Magnum::Range3D BB = physicsManager_->getObjectLocalBoundingBox(physObjectID);
  Magnum::Vector3 offset(0.0, BB.sizeY(), 0.0);
  physicsManager_->setTranslation(physObjectID, Magnum::Vector3(position));
  objectIDs_.push_back(physObjectID);
  sequentialItemSpawnID_ = (sequentialItemSpawnID_+1)%numObjects;
  */
}

assets::MeshData Viewer::load(const assets::AssetInfo& info) {
  assets::MeshData mesh;
  if (!esp::io::exists(info.filepath)) {
    LOG(ERROR) << "Could not find file " << info.filepath;
    return mesh;
  }

  if (info.type == assets::AssetType::INSTANCE_MESH) {
    assets::GenericInstanceMeshData instanceMeshData;
    instanceMeshData.loadPLY(info.filepath);

    const auto& vbo = instanceMeshData.getVertexBufferObjectCPU();
    const auto& cbo = instanceMeshData.getColorBufferObjectCPU();
    const auto& ibo = instanceMeshData.getIndexBufferObjectCPU();
    mesh.vbo = vbo;
    mesh.ibo = ibo;
    for (const auto& c : cbo) {
      mesh.cbo.emplace_back(c.cast<float>() / 255.0f);
    }
  } else {
    const aiScene* scene;
    Assimp::Importer Importer;

    // Flags for loading the mesh
    static const int assimpFlags =
        aiProcess_Triangulate | aiProcess_PreTransformVertices;

    scene = Importer.ReadFile(info.filepath.c_str(), assimpFlags);

    const quatf alignSceneToEspGravity =
        quatf::FromTwoVectors(info.frame.gravity(), esp::geo::ESP_GRAVITY);

    // Iterate through all meshes in the file and extract the vertex components
    for (uint32_t m = 0, indexBase = 0; m < scene->mNumMeshes; ++m) {
      const aiMesh& assimpMesh = *scene->mMeshes[m];
      for (uint32_t v = 0; v < assimpMesh.mNumVertices; ++v) {
        // Use Eigen::Map to convert ASSIMP vectors to eigen vectors
        const Eigen::Map<const vec3f> xyz_scene(&assimpMesh.mVertices[v].x);
        const vec3f xyz_esp = alignSceneToEspGravity * xyz_scene;
        mesh.vbo.push_back(xyz_esp);

        if (assimpMesh.mNormals) {
          const Eigen::Map<const vec3f> normal_scene(&assimpMesh.mNormals[v].x);
          const vec3f normal_esp = alignSceneToEspGravity * normal_scene;
          mesh.nbo.push_back(normal_esp);
        }

        if (assimpMesh.HasTextureCoords(0)) {
          const Eigen::Map<const vec2f> texCoord(
              &assimpMesh.mTextureCoords[0][v].x);
          mesh.tbo.push_back(texCoord);
        }

        if (assimpMesh.HasVertexColors(0)) {
          const Eigen::Map<const vec3f> color(&assimpMesh.mColors[0][v].r);
          mesh.cbo.push_back(color);
        }
      }  // vertices

      // Generate and append index buffer for mesh
      for (uint32_t f = 0; f < assimpMesh.mNumFaces; ++f) {
        const aiFace& face = assimpMesh.mFaces[f];
        for (uint32_t i = 0; i < face.mNumIndices; ++i) {
          mesh.ibo.push_back(face.mIndices[i] + indexBase);
        }
      }  // faces
      indexBase += assimpMesh.mNumVertices;
    }  // meshes
  }

  LOG(INFO) << "Loaded " << mesh.vbo.size() << " vertices, " << mesh.ibo.size()
            << " indices";

  return mesh;
}

vec3f toEig(Magnum::Vector3 v) {
  return vec3f(v[0], v[1], v[2]);
}

void Viewer::reconstructNavMesh() {
  pathfinder_.reset(new esp::nav::PathFinder);

  assets::MeshData mesh = load(sceneInfo_);

  // now add the bounding boxes to the mesh
  for (auto id : objectIDs_) {
    Magnum::Range3D BB = physicsManager_->getObjectLocalBoundingBox(id);
    Magnum::Matrix4 T = physicsManager_->getTransformation(id);

    // index base for newly added BB corners
    uint32_t ixb = mesh.vbo.size();
    mesh.vbo.push_back(toEig(T.transformPoint(BB.frontTopLeft())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.frontTopRight())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.frontBottomLeft())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.frontBottomRight())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.backTopLeft())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.backTopRight())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.backBottomLeft())));
    mesh.vbo.push_back(toEig(T.transformPoint(BB.backBottomRight())));

    // now add the faces
    std::vector<uint32_t> indices{0, 1, 2, 1, 3, 2, 1, 5, 7, 1, 7, 3,
                                  0, 4, 5, 0, 5, 1, 0, 2, 6, 0, 6, 4,
                                  4, 6, 7, 4, 7, 5, 2, 6, 3, 6, 7, 3};

    for (auto i : indices) {
      mesh.ibo.push_back(i + ixb);
    }
  }

  nav::NavMeshSettings bs;
  bs.setDefaults();

  if (!pathfinder_->build(bs, mesh)) {
    LOG(ERROR) << "Failed to build navmesh";
    return;
  }

  LOG(INFO) << "reconstruct navmesh successful";
}

void Viewer::incrementallyAddObjects() {
  // vec3f position(-5.7589, 0.120596, 9.3611); //skokloster
  // vec3f position(1.44672, 0.03, -3.26787); //CODA room

  int numObjects = resourceManager_.getNumLibraryObjects();
  if (!numObjects) {
    return;
  }
  addAndPlaceObjectFromNavmesh(sequentialItemSpawnID_);
  sequentialItemSpawnID_ = (sequentialItemSpawnID_ + 1) % numObjects;
}

int Viewer::addAndPlaceObjectFromNavmesh(int objLibraryID) {
  std::string configFile = resourceManager_.getObjectConfig(objLibraryID);
  auto& drawables = sceneGraph_->getDrawables();
  assets::PhysicsObjectAttributes poa =
      resourceManager_.getPhysicsObjectAttributes(configFile);
  int physObjectID = physicsManager_->addObject(configFile, &drawables);
  physicsManager_->setObjectMotionType(physObjectID,
                                       physics::MotionType::KINEMATIC);
  Magnum::Range3D BB = physicsManager_->getObjectLocalBoundingBox(physObjectID);

  // find a place to put the object
  int tries = 0;
  bool placedWell = false;
  while (!placedWell && tries < 99) {
    float rad = 0;
    vec3f position;
    while ((rad < BB.sizeX() / 2.0 || rad < BB.sizeZ() / 2.0) && tries < 99) {
      position = pathfinder_->getRandomNavigablePoint();
      rad = pathfinder_->distanceToClosestObstacle(position);
      tries++;
    }

    Magnum::Vector3 offset(0.0, BB.sizeY() / 2.0, 0.0);
    physicsManager_->setTranslation(physObjectID,
                                    Magnum::Vector3(position) + offset);

    // now pick an orientation
    Magnum::Quaternion R = Magnum::Quaternion::rotation(
        Rad((rand() % 1000 / 1000.0) * M_PI * 2.0), Magnum::Vector3(0, 1.0, 0));
    physicsManager_->setRotation(physObjectID, R);

    // LOG(INFO) << "num overlapping pairs: " <<
    // physicsManager_->getNumOverlappingObjectPairs(true);
    placedWell = !physicsManager_->contactTest(physObjectID);
    // LOG(INFO) << "new object overlapping?: " <<
    // physicsManager_->contactTest(physObjectID);
  }

  if (!placedWell) {
    // remove the object!
    physicsManager_->removeObject(physObjectID);
    return ID_UNDEFINED;
  }
  objectIDs_.push_back(physObjectID);
  return physObjectID;
}

void Viewer::generateRandomScene(int numObjects) {
  auto start = std::chrono::steady_clock::now();
  // empty the scene
  for (auto id : physicsManager_->getExistingObjectIDs()) {
    physicsManager_->removeObject(id);
  }
  objectIDs_.clear();

  // add new objects to the scene
  for (int i = 0; i < numObjects; i++) {
    int randObjLibID = rand() % resourceManager_.getNumLibraryObjects();
    addAndPlaceObjectFromNavmesh(randObjLibID);
  }

  // rebuild the navmesh with the new item bounding boxes and draw an agent
  // position
  reconstructNavMesh();
  const vec3f position = pathfinder_->getRandomNavigablePoint();
  agentBodyNode_->setTranslation(Vector3(position));
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start);
  LOG(INFO) << "We reset the scene in " << duration.count() << " milliseconds!";
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

  if (physicsManager_ != nullptr)
    physicsManager_->stepPhysics(timeline_.previousFrameDuration());

  int DEFAULT_SCENE = 0;
  int sceneID = sceneID_[DEFAULT_SCENE];
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  renderCamera_->getMagnumCamera().draw(sceneGraph.getDrawables());

  swapBuffers();
  timeline_.nextFrame();
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
  const float distance =
      renderCamera_->node().transformation().translation().z();

  /* Move 15% of the distance back or forward */
  renderCamera_->node().translateLocal(
      {0.0f, 0.0f,
       distance * (1.0f - (event.offset().y() > 0 ? 1 / 0.85f : 0.85f))});

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
  renderCamera_->node().rotate(-angle, axis.normalized());
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
          int randObjectID = rand() % numObjects;
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
    case KeyEvent::Key::B: {
      // toggle a random object's bounding box draw
      int numObjects = physicsManager_->getNumRigidObjects();
      int randObjectID = rand() % numObjects;
      physicsManager_->toggleBBDraw(randObjectID, &sceneGraph_->getDrawables());
    } break;
    case KeyEvent::Key::T: {
      // Test key. Put what you want here...
      torqueLastObject();
    } break;
    case KeyEvent::Key::I:
      Magnum::DebugTools::screenshot(GL::defaultFramebuffer,
                                     "test_image_save.png");
      break;
    case KeyEvent::Key::Equal: {
      int randPrimitiveID = rand() %
                            esp::assets::ResourceManager::AvailablePrimitives::
                                FINAL_NUM_PRIMITIVES_COUNTER;
      addPrimitiveDrawable(randPrimitiveID);
    } break;
    case KeyEvent::Key::Minus: {
      // remove the first primitive
      removePrimitiveDrawable(0);
    } break;
    case KeyEvent::Key::N: {
      highlightNavmeshIsland();
    } break;
    case KeyEvent::Key::M: {
      incrementallyAddObjects();
    } break;
    case KeyEvent::Key::R: {
      // reset the agent position
      const vec3f position = pathfinder_->getRandomNavigablePoint();
      agentBodyNode_->setTranslation(Vector3(position));
    } break;
    case KeyEvent::Key::Period: {
      // regenerate the roomw ith obstacles
      generateRandomScene(10);
    } break;

    default:
      break;
  }
  renderCamera_->node().setTransformation(
      rgbSensorNode_->absoluteTransformation());
  redraw();
}

}  // namespace gfx
}  // namespace esp
