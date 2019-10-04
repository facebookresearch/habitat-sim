// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Simulator.h"

#include <string>

#include <Corrade/Containers/Pointer.h>
#include <Corrade/Utility/String.h>

#include <Magnum/ImageView.h>
#include <Magnum/Math/Range.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/AbstractImageConverter.h>

#include "Drawable.h"

#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/io/io.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/PinholeCamera.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

using namespace Magnum;
using namespace Math::Literals;
using namespace Corrade;

namespace esp {
namespace gfx {

Simulator::Simulator(const SimulatorConfiguration& cfg) {
  // initalize members according to cfg
  // NOTE: NOT SO GREAT NOW THAT WE HAVE virtual functions
  //       Maybe better not to do this reconfigure
  reconfigure(cfg);
}

Simulator::~Simulator() {
  LOG(INFO) << "Deconstructing Simulator";
}

void Simulator::reconfigure(const SimulatorConfiguration& cfg) {
  // if configuration is unchanged, just reset and return
  if (cfg == config_) {
    reset();
    return;
  }
  // otherwise set current configuration and initialize
  // TODO can optimize to do partial re-initialization instead of from-scratch
  config_ = cfg;

  // load scene
  std::string sceneFilename = cfg.scene.id;
  if (cfg.scene.filepaths.count("mesh")) {
    sceneFilename = cfg.scene.filepaths.at("mesh");
  }

  std::string houseFilename = io::changeExtension(sceneFilename, ".house");
  if (cfg.scene.filepaths.count("house")) {
    houseFilename = cfg.scene.filepaths.at("house");
  }

  const assets::AssetInfo sceneInfo =
      assets::AssetInfo::fromPath(sceneFilename);

  // initalize scene graph
  // CAREFUL!
  // previous scene graph is not deleted!
  // TODO:
  // We need to make a design decision here:
  // when doing reconfigure, shall we delete all of the previous scene graphs
  activeSceneID_ = sceneManager_.initSceneGraph();

  // LOG(INFO) << "Active scene graph ID = " << activeSceneID_;
  sceneID_.push_back(activeSceneID_);

  if (cfg.createRenderer) {
    if (!context_) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    // reinitalize members
    if (!renderer_) {
      renderer_ = Renderer::create();
    }

    auto& sceneGraph = sceneManager_.getSceneGraph(activeSceneID_);

    auto& rootNode = sceneGraph.getRootNode();
    auto& drawables = sceneGraph.getDrawables();
    resourceManager_.compressTextures(cfg.compressTextures);

    bool loadSuccess = false;
    if (config_.enablePhysics) {
      loadSuccess =
          resourceManager_.loadScene(sceneInfo, physicsManager_, &rootNode,
                                     &drawables, cfg.physicsConfigFile);
    } else {
      loadSuccess =
          resourceManager_.loadScene(sceneInfo, &rootNode, &drawables);
    }
    if (!loadSuccess) {
      LOG(ERROR) << "cannot load " << sceneFilename;
      // Pass the error to the python through pybind11 allowing graceful exit
      throw std::invalid_argument("Cannot load: " + sceneFilename);
    }

    if (io::exists(houseFilename)) {
      LOG(INFO) << "Loading house from " << houseFilename;
      // if semantic mesh exists, load it as well
      // TODO: remove hardcoded filename change and use SceneConfiguration
      const std::string semanticMeshFilename =
          io::removeExtension(houseFilename) + "_semantic.ply";
      if (io::exists(semanticMeshFilename)) {
        LOG(INFO) << "Loading semantic mesh " << semanticMeshFilename;
        activeSemanticSceneID_ = sceneManager_.initSceneGraph();
        sceneID_.push_back(activeSemanticSceneID_);
        auto& semanticSceneGraph =
            sceneManager_.getSceneGraph(activeSemanticSceneID_);
        auto& semanticRootNode = semanticSceneGraph.getRootNode();
        auto& semanticDrawables = semanticSceneGraph.getDrawables();
        const assets::AssetInfo semanticSceneInfo =
            assets::AssetInfo::fromPath(semanticMeshFilename);
        resourceManager_.loadScene(semanticSceneInfo, &semanticRootNode,
                                   &semanticDrawables);
      }
      LOG(INFO) << "Loaded.";
    }

    // instance meshes and suncg houses contain their semantic annotations
    if (sceneInfo.type == assets::AssetType::SUNCG_SCENE ||
        sceneInfo.type == assets::AssetType::INSTANCE_MESH) {
      activeSemanticSceneID_ = activeSceneID_;
    }
  }

  semanticScene_ = nullptr;
  semanticScene_ = scene::SemanticScene::create();
  if (io::exists(houseFilename)) {
    scene::SemanticScene::loadMp3dHouse(houseFilename, *semanticScene_);
  }

  // also load SemanticScene for SUNCG house file
  if (sceneInfo.type == assets::AssetType::SUNCG_SCENE) {
    scene::SemanticScene::loadSuncgHouse(sceneFilename, *semanticScene_);
  }

  // now reset to sample agent state
  reset();
}

void Simulator::reset() {
  if (physicsManager_ != nullptr)
    physicsManager_
        ->reset();  // TODO: this does nothing yet... desired reset behavior?
}

void Simulator::seed(uint32_t newSeed) {
  random_.seed(newSeed);
}

std::shared_ptr<Renderer> Simulator::getRenderer() {
  return renderer_;
}

std::shared_ptr<physics::PhysicsManager> Simulator::getPhysicsManager() {
  return physicsManager_;
}

std::shared_ptr<scene::SemanticScene> Simulator::getSemanticScene() {
  return semanticScene_;
}

scene::SceneGraph& Simulator::getActiveSceneGraph() {
  CHECK_GE(activeSceneID_, 0);
  CHECK_LT(activeSceneID_, sceneID_.size());
  return sceneManager_.getSceneGraph(activeSceneID_);
}

//! return the semantic scene's SceneGraph for rendering
scene::SceneGraph& Simulator::getActiveSemanticSceneGraph() {
  CHECK_GE(activeSemanticSceneID_, 0);
  CHECK_LT(activeSemanticSceneID_, sceneID_.size());
  return sceneManager_.getSceneGraph(activeSemanticSceneID_);
}

bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return a.scene == b.scene && a.defaultAgentId == b.defaultAgentId &&
         a.defaultCameraUuid == b.defaultCameraUuid &&
         a.compressTextures == b.compressTextures &&
         a.createRenderer == b.createRenderer &&
         a.enablePhysics == b.enablePhysics &&
         a.physicsConfigFile.compare(b.physicsConfigFile) == 0;
}

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return !(a == b);
}

// === Physics Simulator Functions ===

int Simulator::addObject(const int objectLibIndex, const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    // TODO: change implementation to support multi-world and physics worlds to
    // own reference to a sceneGraph to avoid this.
    auto& sceneGraph_ = sceneManager_.getSceneGraph(sceneID);
    auto& drawables = sceneGraph_.getDrawables();
    return physicsManager_->addObject(objectLibIndex, &drawables);
  }
  return ID_UNDEFINED;
}

// return the current size of the physics object library (objects [0,size) can
// be instanced)
int Simulator::getPhysicsObjectLibrarySize() {
  return resourceManager_.getNumLibraryObjects();
}

// return a list of existing objected IDs in a physical scene
std::vector<int> Simulator::getExistingObjectIDs(const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->getExistingObjectIDs();
  }
  return std::vector<int>();  // empty if no simulator exists
}

// remove object objectID instance in sceneID
int Simulator::removeObject(const int objectID, const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->removeObject(objectID);
  }
  return ID_UNDEFINED;
}

// apply forces and torques to objects
void Simulator::applyTorque(const Magnum::Vector3& tau,
                            const int objectID,
                            const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    physicsManager_->applyTorque(objectID, tau);
  }
}

void Simulator::applyForce(const Magnum::Vector3& force,
                           const Magnum::Vector3& relPos,
                           const int objectID,
                           const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    physicsManager_->applyForce(objectID, force, relPos);
  }
}

// set object transform (kinemmatic control)
void Simulator::setTransformation(const Magnum::Matrix4& transform,
                                  const int objectID,
                                  const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    physicsManager_->setTransformation(objectID, transform);
  }
}

Magnum::Matrix4 Simulator::getTransformation(const int objectID,
                                             const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->getTransformation(objectID);
  }
  return Magnum::Matrix4::fromDiagonal(Magnum::Vector4(1));
}

// set object translation directly
void Simulator::setTranslation(const Magnum::Vector3& translation,
                               const int objectID,
                               const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    physicsManager_->setTranslation(objectID, translation);
  }
}

Magnum::Vector3 Simulator::getTranslation(const int objectID,
                                          const int sceneID) {
  // can throw if physicsManager is not initialized or either objectID/sceneID
  // is invalid
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->getTranslation(objectID);
  }
  return Magnum::Vector3();
}

// set object orientation directly
void Simulator::setRotation(const Magnum::Quaternion& rotation,
                            const int objectID,
                            const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    physicsManager_->setRotation(objectID, rotation);
  }
}

Magnum::Quaternion Simulator::getRotation(const int objectID,
                                          const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->getRotation(objectID);
  }
  return Magnum::Quaternion();
}

double Simulator::stepWorld(const double dt) {
  if (physicsManager_ != nullptr) {
    physicsManager_->stepPhysics(dt);
  }
  return getWorldTime();
}

// get the simulated world time (0 if no physics enabled)
double Simulator::getWorldTime() {
  if (physicsManager_ != nullptr) {
    return physicsManager_->getWorldTime();
  }
  return NO_TIME;
}

assets::MeshData load(const assets::AssetInfo& info) {
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

std::shared_ptr<nav::PathFinder> Simulator::recomputeNavMesh(
    bool includeObjectBBs) {
  // sceneManager_.
  std::shared_ptr<nav::PathFinder> pf = std::make_shared<nav::PathFinder>();

  const assets::AssetInfo info = assets::AssetInfo::fromPath(config_.scene.id);

  assets::MeshData mesh = load(info);

  // now add the bounding boxes to the mesh
  for (auto id : physicsManager_->getExistingObjectIDs()) {
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
  // bs.agentRadius *= 5.0;

  if (!pf->build(bs, mesh)) {
    LOG(ERROR) << "Failed to build navmesh";
  }

  LOG(INFO) << "reconstruct navmesh successful";
  return pf;
}

Magnum::Range3D Simulator::getObjLocalBB(const int objectID,
                                         const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->getObjectLocalBoundingBox(objectID);
  }
  return Magnum::Range3D();
}

bool Simulator::contactTest(const int objectID, const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->contactTest(objectID);
  }
  return false;
}

esp::physics::MotionType Simulator::getObjectMotionType(const int objectID,
                                                        const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    return physicsManager_->getObjectMotionType(objectID);
  }
  return esp::physics::MotionType::ERROR_MOTIONTYPE;
}

void Simulator::setObjectMotionType(esp::physics::MotionType mt,
                                    const int objectID,
                                    const int sceneID) {
  if (physicsManager_ != nullptr && sceneID >= 0 && sceneID < sceneID_.size()) {
    physicsManager_->setObjectMotionType(objectID, mt);
  }
}

}  // namespace gfx
}  // namespace esp
