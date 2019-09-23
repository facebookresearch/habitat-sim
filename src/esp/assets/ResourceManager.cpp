// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/Tags.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>

#include "esp/geo/geo.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/PrimitiveIDDrawable.h"
#include "esp/gfx/PrimitiveIDShader.h"
#include "esp/io/io.h"
#include "esp/io/json.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneGraph.h"

#include "CollisionMeshData.h"
#include "GenericInstanceMeshData.h"
#include "GltfMeshData.h"
#include "MeshData.h"
#include "Mp3dInstanceMeshData.h"
#include "ResourceManager.h"
#include "esp/physics/PhysicsManager.h"

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#endif

#ifdef ESP_BUILD_PTEX_SUPPORT
#include "PTexMeshData.h"
#include "esp/gfx/PTexMeshDrawable.h"
#include "esp/gfx/PTexMeshShader.h"
#endif

namespace Cr = Corrade;

namespace esp {
namespace assets {

bool ResourceManager::loadScene(const AssetInfo& info,
                                scene::SceneNode* parent, /* = nullptr */
                                DrawableGroup* drawables /* = nullptr */) {
  // scene mesh loading
  bool meshSuccess = true;
  if (info.filepath.compare(EMPTY_SCENE) != 0) {
    if (!io::exists(info.filepath)) {
      LOG(ERROR) << "Cannot load from file " << info.filepath;
      meshSuccess = false;
    } else {
      if (info.type == AssetType::INSTANCE_MESH) {
        meshSuccess = loadInstanceMeshData(info, parent, drawables);
      } else if (info.type == AssetType::FRL_PTEX_MESH) {
        meshSuccess = loadPTexMeshData(info, parent, drawables);
      } else if (info.type == AssetType::SUNCG_SCENE) {
        meshSuccess = loadSUNCGHouseFile(info, parent, drawables);
      } else if (info.type == AssetType::MP3D_MESH) {
        meshSuccess = loadGeneralMeshData(info, parent, drawables);
      } else {
        // Unknown type, just load general mesh data
        meshSuccess = loadGeneralMeshData(info, parent, drawables);
      }
      // add a scene attributes for this filename or modify the existing one
      if (meshSuccess) {
        physicsSceneLibrary_[info.filepath].setString("renderMeshHandle",
                                                      info.filepath);
      }
    }
  } else {
    LOG(INFO) << "Loading empty scene";
    // EMPTY_SCENE (ie. "NONE") string indicates desire for an empty scene (no
    // scene mesh): welcome
    // to the void
  }

  return meshSuccess;
}

//! (1) Read config and set physics timestep
//! (2) loadScene() with PhysicsSceneMetaData
// TODO (JH): this function seems to entangle certain physicsManager functions
bool ResourceManager::loadScene(
    const AssetInfo& info,
    std::shared_ptr<physics::PhysicsManager>& _physicsManager,
    scene::SceneNode* parent, /* = nullptr */
    DrawableGroup* drawables, /* = nullptr */
    std::string physicsFilename /* data/default.phys_scene_config.json */) {
  // In-memory representation of scene meta data
  PhysicsManagerAttributes physicsManagerAttributes =
      loadPhysicsConfig(physicsFilename);
  physicsManagerLibrary_[physicsFilename] = physicsManagerAttributes;

  return loadScene(info, _physicsManager, physicsManagerAttributes, parent,
                   drawables);
}

// TODO: kill existing scene mesh drawables, nodes, etc... (all but meshes in
// memory?)
//! (1) load scene mesh
//! (2) add drawable (if parent and drawables != nullptr)
//! (3) consume PhysicsSceneMetaData to initialize physics simulator
//! (4) create scene collision mesh if possible
bool ResourceManager::loadScene(
    const AssetInfo& info,
    std::shared_ptr<physics::PhysicsManager>& _physicsManager,
    PhysicsManagerAttributes physicsManagerAttributes,
    scene::SceneNode* parent, /* = nullptr */
    DrawableGroup* drawables /* = nullptr */) {
  // default scene mesh loading
  bool meshSuccess = loadScene(info, parent, drawables);

  //! PHYSICS INIT: Use the above config to initialize physics engine
  bool defaultToNoneSimulator = true;
  if (physicsManagerAttributes.getString("simulator").compare("bullet") == 0) {
#ifdef ESP_BUILD_WITH_BULLET
    _physicsManager.reset(new physics::BulletPhysicsManager(this));
    defaultToNoneSimulator = false;
#else
    LOG(WARNING)
        << ":\n---\nPhysics was enabled and Bullet physics engine was "
           "specified, but the project is built without Bullet support. "
           "Objects added to the scene will be restricted to kinematic updates "
           "only. Reinstall with --bullet to enable Bullet dynamics.\n---";
#endif
  }

  // reseat to base PhysicsManager to override previous as default behavior
  // if the desired simulator is not supported reset to "none" in metaData
  if (defaultToNoneSimulator) {
    _physicsManager.reset(new physics::PhysicsManager(this));
    physicsManagerAttributes.setString("simulator", "none");
  }

  // load objects from sceneMetaData list...
  for (auto objPhysPropertiesFilename :
       physicsManagerAttributes.getVecStrings("objectLibraryPaths")) {
    LOG(INFO) << "loading object: " << objPhysPropertiesFilename;
    loadObject(objPhysPropertiesFilename);
  }
  LOG(INFO) << "loaded objects: "
            << std::to_string(physicsObjectLibrary_.size());

  // initialize the physics simulator
  _physicsManager->initPhysics(parent, physicsManagerAttributes);

  if (!meshSuccess) {
    LOG(ERROR) << "Physics manager loaded. Scene mesh load failed, aborting "
                  "scene initialization.";
    return meshSuccess;
  }

  // TODO: enable loading of multiple scenes from file and storing individual
  // parameters instead of scene properties in manager global config
  physicsSceneLibrary_[info.filepath].setDouble(
      "frictionCoefficient",
      physicsManagerAttributes.getDouble("frictionCoefficient"));
  physicsSceneLibrary_[info.filepath].setDouble(
      "restitutionCoefficient",
      physicsManagerAttributes.getDouble("restitutionCoefficient"));

  //! CONSTRUCT SCENE
  const std::string& filename = info.filepath;
  // if we have a scene mesh, add it as a collision object
  if (filename.compare(EMPTY_SCENE) != 0) {
    MeshMetaData& metaData = resourceDict_.at(filename);
    auto indexPair = metaData.meshIndex;
    int start = indexPair.first;
    int end = indexPair.second;

    //! Collect collision mesh group
    std::vector<CollisionMeshData> meshGroup;
    for (int mesh_i = start; mesh_i <= end; mesh_i++) {
      // PLY Instance mesh
      if (info.type == AssetType::INSTANCE_MESH) {
        GenericInstanceMeshData* insMeshData =
            dynamic_cast<GenericInstanceMeshData*>(meshes_[mesh_i].get());
        CollisionMeshData& meshData = insMeshData->getCollisionMeshData();
        meshGroup.push_back(meshData);
      }

      // GLB Mesh
      else if (info.type == AssetType::MP3D_MESH) {
        quatf quatFront =
            quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
        Magnum::Quaternion quat = Magnum::Quaternion(quatFront);
        GltfMeshData* gltfMeshData =
            dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
        CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
        Magnum::Matrix4 transform =
            Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());
        Magnum::MeshTools::transformPointsInPlace(transform,
                                                  meshData.positions);
        meshGroup.push_back(meshData);
      }
    }

    //! Initialize collision mesh
    bool sceneSuccess = _physicsManager->addScene(
        physicsSceneLibrary_.at(info.filepath), meshGroup);
    if (!sceneSuccess) {
      return false;
    }
  }

  return meshSuccess;
}

PhysicsManagerAttributes ResourceManager::loadPhysicsConfig(
    std::string physicsFilename) {
  // Load the global scene config JSON here
  io::JsonDocument scenePhysicsConfig = io::parseJsonFile(physicsFilename);
  // In-memory representation of scene meta data
  PhysicsManagerAttributes physicsManagerAttributes;

  // load the simulator preference
  // default is "none" simulator
  if (scenePhysicsConfig.HasMember("physics simulator")) {
    if (scenePhysicsConfig["physics simulator"].IsString()) {
      physicsManagerAttributes.setString(
          "simulator", scenePhysicsConfig["physics simulator"].GetString());
    }
  }

  // load the physics timestep
  if (scenePhysicsConfig.HasMember("timestep")) {
    if (scenePhysicsConfig["timestep"].IsNumber()) {
      physicsManagerAttributes.setDouble(
          "timestep", scenePhysicsConfig["timestep"].GetDouble());
    }
  }

  if (scenePhysicsConfig.HasMember("friction coefficient") &&
      scenePhysicsConfig["friction coefficient"].IsNumber()) {
    physicsManagerAttributes.setDouble(
        "frictionCoefficient",
        scenePhysicsConfig["friction coefficient"].GetDouble());
  } else {
    LOG(ERROR) << " Invalid value in scene config - friction coefficient";
  }

  if (scenePhysicsConfig.HasMember("restitution coefficient") &&
      scenePhysicsConfig["restitution coefficient"].IsNumber()) {
    physicsManagerAttributes.setDouble(
        "restitutionCoefficient",
        scenePhysicsConfig["restitution coefficient"].GetDouble());
  } else {
    LOG(ERROR) << " Invalid value in scene config - restitution coefficient";
  }

  // load gravity
  if (scenePhysicsConfig.HasMember("gravity")) {
    if (scenePhysicsConfig["gravity"].IsArray()) {
      Magnum::Vector3 grav;
      for (rapidjson::SizeType i = 0; i < scenePhysicsConfig["gravity"].Size();
           i++) {
        if (!scenePhysicsConfig["gravity"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << "Invalid value in physics gravity array";
          break;
        } else {
          grav[i] = scenePhysicsConfig["gravity"][i].GetDouble();
        }
      }
      physicsManagerAttributes.setMagnumVec3("gravity", grav);
    }
  }

  //! Load object paths
  std::string configDirectory =
      physicsFilename.substr(0, physicsFilename.find_last_of("/"));
  // load the rigid object library metadata (no physics init yet...)
  // NOTE: expect relative paths to the global config
  if (scenePhysicsConfig.HasMember("rigid object paths")) {
    if (scenePhysicsConfig["rigid object paths"].IsArray()) {
      physicsManagerAttributes.setVecStrings("objectLibraryPaths",
                                             std::vector<std::string>());
      for (rapidjson::SizeType i = 0;
           i < scenePhysicsConfig["rigid object paths"].Size(); i++) {
        if (scenePhysicsConfig["rigid object paths"][i].IsString()) {
          // 1: read the filename (relative path)
          std::string objPhysPropertiesFilename =
              configDirectory + "/" +
              scenePhysicsConfig["rigid object paths"][i].GetString() +
              ".phys_properties.json";
          physicsManagerAttributes.appendVecStrings("objectLibraryPaths",
                                                    objPhysPropertiesFilename);
        } else {
          LOG(ERROR) << "Invalid value in physics scene config -rigid object "
                        "library- array "
                     << i;
        }
      }
    }
  }

  return physicsManagerAttributes;
}

//! Only load and does not instantiate object
//! For load-only: set parent = nullptr, drawables = nullptr
int ResourceManager::loadObject(const std::string& objPhysConfigFilename,
                                scene::SceneNode* parent,
                                DrawableGroup* drawables) {
  // Load Object from config
  const bool objectIsLoaded =
      physicsObjectLibrary_.count(objPhysConfigFilename) > 0;

  // Find objectID in resourceManager
  int objectID = -1;
  if (!objectIsLoaded) {
    // Main loading function
    objectID = loadObject(objPhysConfigFilename);

  } else {
    std::vector<std::string>::iterator itr =
        std::find(physicsObjectConfigList_.begin(),
                  physicsObjectConfigList_.end(), objPhysConfigFilename);
    objectID = std::distance(physicsObjectConfigList_.begin(), itr);
  }

  if (parent != nullptr and drawables != nullptr) {
    //! Add mesh to rendering stack

    // Meta data and collision mesh
    PhysicsObjectAttributes physicsObjectAttributes =
        physicsObjectLibrary_[objPhysConfigFilename];
    std::vector<CollisionMeshData> meshGroup =
        collisionMeshGroups_[objPhysConfigFilename];

    const std::string& filename =
        physicsObjectAttributes.getString("renderMeshHandle");

    MeshMetaData meshMetaData = resourceDict_[filename];
    scene::SceneNode& newNode = parent->createChild();
    AssetInfo renderMeshinfo = AssetInfo::fromPath(filename);
    Magnum::PluginManager::Manager<Importer> manager;
    std::unique_ptr<Importer> importer =
        manager.loadAndInstantiate("AnySceneImporter");
    manager.setPreferredPlugins("GltfImporter", {"TinyGltfImporter"});
    manager.setPreferredPlugins("ObjImporter", {"AssimpImporter"});
    importer->openFile(renderMeshinfo.filepath);
    for (auto componentID : magnumMeshDict_[filename]) {
      addComponent(*importer, meshMetaData, newNode, drawables, componentID);
    }
  }

  return objectID;
}

PhysicsObjectAttributes& ResourceManager::getPhysicsObjectAttributes(
    const std::string& objectName) {
  return physicsObjectLibrary_[objectName];
}

// load object from config filename
int ResourceManager::loadObject(const std::string& objPhysConfigFilename) {
  // check for duplicate load
  const bool objExists = physicsObjectLibrary_.count(objPhysConfigFilename) > 0;
  if (objExists) {
    // TODO: this will skip the duplicate. Is there a good reason to allow
    // duplicates?
    std::vector<std::string>::iterator itr =
        std::find(physicsObjectConfigList_.begin(),
                  physicsObjectConfigList_.end(), objPhysConfigFilename);
    int objectID = std::distance(physicsObjectConfigList_.begin(), itr);
    return objectID;
  }

  // 1. parse the config file
  io::JsonDocument objPhysicsConfig;
  if (io::exists(objPhysConfigFilename)) {
    try {
      objPhysicsConfig = io::parseJsonFile(objPhysConfigFilename);
    } catch (...) {
      LOG(ERROR) << "Failed to parse JSON: " << objPhysConfigFilename
                 << ". Aborting loadObject.";
      return ID_UNDEFINED;
    }
  } else {
    LOG(ERROR) << "File " << objPhysConfigFilename
               << " does not exist. Aborting loadObject.";
    return ID_UNDEFINED;
  }

  // 2. construct a physicsObjectMetaData
  PhysicsObjectAttributes physicsObjectAttributes;

  // NOTE: these paths should be relative to the properties file
  std::string propertiesFileDirectory =
      objPhysConfigFilename.substr(0, objPhysConfigFilename.find_last_of("/"));

  // 3. load physical properties to override defaults (set in
  // PhysicsObjectMetaData.h) load the mass
  if (objPhysicsConfig.HasMember("mass")) {
    if (objPhysicsConfig["mass"].IsNumber()) {
      physicsObjectAttributes.setDouble("mass",
                                        objPhysicsConfig["mass"].GetDouble());
    }
  }

  // load the center of mass (in the local frame of the object)
  bool shouldComputeMeshBBCenter = true;
  // if COM is provided, use it for mesh shift
  if (objPhysicsConfig.HasMember("COM")) {
    if (objPhysicsConfig["COM"].IsArray()) {
      Magnum::Vector3 COM;
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["COM"].Size(); i++) {
        if (!objPhysicsConfig["COM"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << " Invalid value in object physics config - COM array";
          break;
        } else {
          COM[i] = objPhysicsConfig["COM"][i].GetDouble();
          shouldComputeMeshBBCenter = false;
        }
      }
      physicsObjectAttributes.setMagnumVec3("COM", COM);
    }
  }

  // load the inertia diagonal
  if (objPhysicsConfig.HasMember("inertia")) {
    if (objPhysicsConfig["inertia"].IsArray()) {
      Magnum::Vector3 inertia;
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["inertia"].Size();
           i++) {
        if (!objPhysicsConfig["inertia"][i].IsNumber()) {
          // invalid config
          LOG(ERROR)
              << " Invalid value in object physics config - inertia array";
          break;
        } else {
          inertia[i] = objPhysicsConfig["inertia"][i].GetDouble();
        }
      }
      physicsObjectAttributes.setMagnumVec3("inertia", inertia);
    }
  }

  // load the friction coefficient
  if (objPhysicsConfig.HasMember("friction coefficient")) {
    if (objPhysicsConfig["friction coefficient"].IsNumber()) {
      physicsObjectAttributes.setDouble(
          "frictionCoefficient",
          objPhysicsConfig["friction coefficient"].GetDouble());
    } else {
      LOG(ERROR)
          << " Invalid value in object physics config - friction coefficient";
    }
  }

  // load the restitution coefficient
  if (objPhysicsConfig.HasMember("restitution coefficient")) {
    if (objPhysicsConfig["restitution coefficient"].IsNumber()) {
      physicsObjectAttributes.setDouble(
          "restitutionCoefficient",
          objPhysicsConfig["restitution coefficient"].GetDouble());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - restitution "
                    "coefficient";
    }
  }

  // 3. load/check_for render and collision mesh metadata
  //! Get render mesh names
  std::string renderMeshFilename = "";
  std::string collisionMeshFilename = "";

  if (objPhysicsConfig.HasMember("render mesh")) {
    if (objPhysicsConfig["render mesh"].IsString()) {
      renderMeshFilename = Cr::Utility::Directory::join(
          propertiesFileDirectory, objPhysicsConfig["render mesh"].GetString());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - render mesh";
    }
  }
  if (objPhysicsConfig.HasMember("collision mesh")) {
    if (objPhysicsConfig["collision mesh"].IsString()) {
      collisionMeshFilename = Cr::Utility::Directory::join(
          propertiesFileDirectory,
          objPhysicsConfig["collision mesh"].GetString());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - collision mesh";
    }
  }

  bool renderMeshSuccess = false;
  bool collisionMeshSuccess = false;
  AssetInfo renderMeshinfo;
  AssetInfo collisionMeshinfo;

  Magnum::Vector3 COM = physicsObjectAttributes.getMagnumVec3("COM");
  bool shiftMeshOrigin = !(COM[0] == 0 && COM[1] == 0 && COM[2] == 0);

  //! Load rendering mesh
  // TODO: unify the COM move: don't want
  // simplified meshes to result in different rendering and collision
  // COMs/origins
  if (!renderMeshFilename.empty()) {
    renderMeshinfo = assets::AssetInfo::fromPath(renderMeshFilename);

    if (shouldComputeMeshBBCenter) {  // compute the COM from BB center
      renderMeshSuccess =
          loadGeneralMeshData(renderMeshinfo, nullptr, nullptr, true);
    } else if (shiftMeshOrigin) {  // use the provided COM
      renderMeshSuccess =
          loadGeneralMeshData(renderMeshinfo, nullptr, nullptr, true,
                              -physicsObjectAttributes.getMagnumVec3("COM"));
    } else {  // mesh origin already at COM
      renderMeshSuccess = loadGeneralMeshData(renderMeshinfo);
    }
    if (!renderMeshSuccess) {
      LOG(ERROR) << "Failed to load a physical object's render mesh: "
                 << objPhysConfigFilename << ", " << renderMeshFilename;
    }
  }
  //! Load collision mesh
  if (!collisionMeshFilename.empty()) {
    collisionMeshinfo = assets::AssetInfo::fromPath(collisionMeshFilename);
    if (shouldComputeMeshBBCenter) {  // compute the COM from BB center
      collisionMeshSuccess =
          loadGeneralMeshData(collisionMeshinfo, nullptr, nullptr, true);
    } else if (shiftMeshOrigin) {  // use the provided COM
      collisionMeshSuccess =
          loadGeneralMeshData(collisionMeshinfo, nullptr, nullptr, true,
                              -physicsObjectAttributes.getMagnumVec3("COM"));
    } else {  // mesh origin already at COM
      collisionMeshSuccess = loadGeneralMeshData(collisionMeshinfo);
    }
    if (!collisionMeshSuccess) {
      LOG(ERROR) << "Failed to load a physical object's collision mesh: "
                 << objPhysConfigFilename << ", " << collisionMeshFilename;
    }
  }

  // NOTE: if we want to save these after edit we need to save the moved
  // mesh or save the original COM as a member of RigidBody...
  physicsObjectAttributes.setMagnumVec3("COM", Magnum::Vector3(0));
  // once we move the meshes, the COM is aligned with the origin...

  if (!renderMeshSuccess && !collisionMeshSuccess) {
    // we only allow objects with SOME mesh file. Failing
    // both loads or having no mesh will cancel the load.
    LOG(ERROR) << "Failed to load a physical object: no meshes...: "
               << objPhysConfigFilename;
    return ID_UNDEFINED;
  }

  physicsObjectAttributes.setString("renderMeshHandle", renderMeshFilename);
  physicsObjectAttributes.setString("collisionMeshHandle",
                                    collisionMeshFilename);

  // handle one missing mesh
  if (!renderMeshSuccess)
    physicsObjectAttributes.setString("renderMeshHandle",
                                      collisionMeshFilename);
  if (!collisionMeshSuccess)
    physicsObjectAttributes.setString("collisionMeshHandle",
                                      renderMeshFilename);

  // 5. cache metaData, collision mesh Group
  physicsObjectLibrary_.emplace(objPhysConfigFilename, physicsObjectAttributes);
  MeshMetaData& meshMetaData = resourceDict_.at(
      physicsObjectAttributes.getString("collisionMeshHandle"));

  int start = meshMetaData.meshIndex.first;
  int end = meshMetaData.meshIndex.second;
  //! Gather mesh components for meshGroup data
  std::vector<CollisionMeshData> meshGroup;
  for (int mesh_i = start; mesh_i <= end; mesh_i++) {
    GltfMeshData* gltfMeshData =
        dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
    CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
    meshGroup.push_back(meshData);
  }
  //! Properly align axis direction
  // NOTE: this breaks the collision properties of some files
  collisionMeshGroups_.emplace(objPhysConfigFilename, meshGroup);
  physicsObjectConfigList_.push_back(objPhysConfigFilename);

  int objectID = physicsObjectConfigList_.size() - 1;
  return objectID;
}

const std::vector<assets::CollisionMeshData>& ResourceManager::getCollisionMesh(
    const int objectID) {
  std::string configFile = getObjectConfig(objectID);
  return collisionMeshGroups_[configFile];
}

const std::vector<assets::CollisionMeshData>& ResourceManager::getCollisionMesh(
    const std::string configFile) {
  return collisionMeshGroups_[configFile];
}

int ResourceManager::getObjectID(const std::string& configFile) {
  std::vector<std::string>::iterator itr =
      std::find(physicsObjectConfigList_.begin(),
                physicsObjectConfigList_.end(), configFile);
  if (itr == physicsObjectConfigList_.cend()) {
    return -1;
  } else {
    int objectID = std::distance(physicsObjectConfigList_.begin(), itr);
    return objectID;
  }
}

std::string ResourceManager::getObjectConfig(const int objectID) {
  return physicsObjectConfigList_[objectID];
}

Magnum::Vector3 ResourceManager::computeMeshBBCenter(GltfMeshData* meshDataGL) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  return Magnum::Range3D{
      Magnum::Math::minmax<Magnum::Vector3>(meshData.positions)}
      .center();
}

void ResourceManager::translateMesh(GltfMeshData* meshDataGL,
                                    Magnum::Vector3 translation) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();

  Magnum::Matrix4 transform = Magnum::Matrix4::translation(translation);
  Magnum::MeshTools::transformPointsInPlace(transform, meshData.positions);
  // save the mesh transformation for future query
  meshDataGL->meshTransform_ = transform * meshDataGL->meshTransform_;
}

Magnum::GL::AbstractShaderProgram* ResourceManager::getShaderProgram(
    ShaderType type) {
  if (shaderPrograms_.count(type) == 0) {
    switch (type) {
      case INSTANCE_MESH_SHADER: {
        shaderPrograms_[INSTANCE_MESH_SHADER] =
            std::make_shared<gfx::PrimitiveIDShader>();
      } break;

#ifdef ESP_BUILD_PTEX_SUPPORT
      case PTEX_MESH_SHADER: {
        shaderPrograms_[PTEX_MESH_SHADER] =
            std::make_shared<gfx::PTexMeshShader>();
      } break;
#endif

      case COLORED_SHADER: {
        shaderPrograms_[COLORED_SHADER] =
            std::make_shared<Magnum::Shaders::Flat3D>(
                Magnum::Shaders::Flat3D::Flag::ObjectId);
      } break;

      case VERTEX_COLORED_SHADER: {
        shaderPrograms_[VERTEX_COLORED_SHADER] =
            std::make_shared<Magnum::Shaders::Flat3D>(
                Magnum::Shaders::Flat3D::Flag::ObjectId |
                Magnum::Shaders::Flat3D::Flag::VertexColor);
      } break;

      case TEXTURED_SHADER: {
        shaderPrograms_[TEXTURED_SHADER] =
            std::make_shared<Magnum::Shaders::Flat3D>(
                Magnum::Shaders::Flat3D::Flag::ObjectId |
                Magnum::Shaders::Flat3D::Flag::Textured);
      } break;

      default:
        return nullptr;
        break;
    }
  }
  return shaderPrograms_[type].get();
}

bool ResourceManager::loadPTexMeshData(const AssetInfo& info,
                                       scene::SceneNode* parent,
                                       DrawableGroup* drawables) {
#ifdef ESP_BUILD_PTEX_SUPPORT
  // if this is a new file, load it and add it to the dictionary
  const std::string& filename = info.filepath;
  if (resourceDict_.count(filename) == 0) {
    const auto atlasDir = Corrade::Utility::Directory::join(
        Corrade::Utility::Directory::path(filename), "textures");

    meshes_.emplace_back(std::make_unique<PTexMeshData>());
    int index = meshes_.size() - 1;
    auto* pTexMeshData = dynamic_cast<PTexMeshData*>(meshes_[index].get());
    pTexMeshData->load(filename, atlasDir);

    // update the dictionary
    resourceDict_.emplace(filename, MeshMetaData(index, index));
  }

  // create the scene graph by request
  if (parent) {
    auto* ptexShader =
        dynamic_cast<gfx::PTexMeshShader*>(getShaderProgram(PTEX_MESH_SHADER));

    auto indexPair = resourceDict_.at(filename).meshIndex;
    int start = indexPair.first;
    int end = indexPair.second;

    for (int iMesh = start; iMesh <= end; ++iMesh) {
      auto* pTexMeshData = dynamic_cast<PTexMeshData*>(meshes_[iMesh].get());

      pTexMeshData->uploadBuffersToGPU(false);

      for (int jSubmesh = 0; jSubmesh < pTexMeshData->getSize(); ++jSubmesh) {
        scene::SceneNode& node = parent->createChild();
        const quatf transform = info.frame.rotationFrameToWorld();
        node.setRotation(Magnum::Quaternion(transform));
        new gfx::PTexMeshDrawable{node, *ptexShader, *pTexMeshData, jSubmesh,
                                  drawables};
      }
    }
  }

  return true;
#else
  LOG(ERROR) << "PTex support not enabled. Enable the BUILD_PTEX_SUPPORT CMake "
                "option when building.";
  return false;
#endif
}

// semantic instance mesh import
bool ResourceManager::loadInstanceMeshData(const AssetInfo& info,
                                           scene::SceneNode* parent,
                                           DrawableGroup* drawables) {
  // if this is a new file, load it and add it to the dictionary, create shaders
  // and add it to the shaderPrograms_
  const std::string& filename = info.filepath;
  if (resourceDict_.count(filename) == 0) {
    if (info.type == AssetType::INSTANCE_MESH) {
      meshes_.emplace_back(std::make_unique<GenericInstanceMeshData>());
    }
    int index = meshes_.size() - 1;
    auto* instanceMeshData =
        dynamic_cast<GenericInstanceMeshData*>(meshes_[index].get());

    instanceMeshData->loadPLY(filename);
    instanceMeshData->uploadBuffersToGPU(false);

    instance_mesh_ = &(instanceMeshData->getRenderingBuffer()->mesh);
    // update the dictionary
    resourceDict_.emplace(filename, MeshMetaData(index, index));
  }

  // create the scene graph by request
  if (parent) {
    auto indexPair = resourceDict_.at(filename).meshIndex;
    int start = indexPair.first;
    int end = indexPair.second;

    for (int iMesh = start; iMesh <= end; ++iMesh) {
      auto* instanceMeshData =
          dynamic_cast<GenericInstanceMeshData*>(meshes_[iMesh].get());
      scene::SceneNode& node = parent->createChild();
      createDrawable(INSTANCE_MESH_SHADER, *instanceMeshData->getMagnumGLMesh(),
                     node, drawables);
    }
  }

  return true;
}

bool ResourceManager::loadGeneralMeshData(
    const AssetInfo& info,
    scene::SceneNode* parent /* = nullptr */,
    DrawableGroup* drawables /* = nullptr */,
    bool shiftOrigin /* = false */,
    Magnum::Vector3 translation /* [0,0,0] */) {
  const std::string& filename = info.filepath;
  const bool fileIsLoaded = resourceDict_.count(filename) > 0;
  const bool drawData = parent != nullptr && drawables != nullptr;

  // Mesh & metaData container
  MeshMetaData metaData;
  std::vector<Magnum::UnsignedInt> magnumData;

  Magnum::PluginManager::Manager<Importer> manager;
  std::unique_ptr<Importer> importer =
      manager.loadAndInstantiate("AnySceneImporter");
  manager.setPreferredPlugins("GltfImporter", {"TinyGltfImporter"});
#ifdef ESP_BUILD_ASSIMP_SUPPORT
  manager.setPreferredPlugins("ObjImporter", {"AssimpImporter"});
#endif

  // Optional File loading
  if (!fileIsLoaded) {
    if (!importer->openFile(filename)) {
      LOG(ERROR) << "Cannot open file " << filename;
      return false;
    }
    // if this is a new file, load it and add it to the dictionary
    loadTextures(*importer, &metaData);
    loadMaterials(*importer, &metaData);
    loadMeshes(*importer, &metaData, shiftOrigin, translation);
    resourceDict_.emplace(filename, metaData);

    // Register magnum mesh
    if (importer->defaultScene() != -1) {
      Corrade::Containers::Optional<Magnum::Trade::SceneData> sceneData =
          importer->scene(importer->defaultScene());
      if (!sceneData) {
        LOG(ERROR) << "Cannot load scene, exiting";
        return false;
      }
      for (unsigned int sceneDataID : sceneData->children3D()) {
        magnumData.emplace_back(sceneDataID);
      }
    } else if (importer->mesh3DCount() && meshes_[metaData.meshIndex.first]) {
      // no default scene --- standalone OBJ/PLY files, for example
      // take a wild guess and load the first mesh with the first material
      // addMeshToDrawables(metaData, *parent, drawables, ID_UNDEFINED, 0, 0);
      magnumData.emplace_back(0);
    } else {
      LOG(ERROR) << "No default scene available and no meshes found, exiting";
      return false;
    }
    magnumMeshDict_.emplace(filename, magnumData);
  } else {
    metaData = resourceDict_[filename];
  }

  // Optional Instantiation
  if (!drawData) {
    //! Do not instantiate object
    return true;
  } else {
    // intercept nullptr scene graph nodes (default) to add mesh to
    // metadata list without adding it to scene graph
    scene::SceneNode& newNode = parent->createChild();

    //! Do instantiate object
    MeshMetaData& metaData = resourceDict_[filename];
    const bool forceReload = false;
    // re-bind position, normals, uv, colors etc. to the corresponding buffers
    // under *current* gl context
    if (forceReload) {
      int start = metaData.meshIndex.first;
      int end = metaData.meshIndex.second;
      if (0 <= start && start <= end) {
        for (int iMesh = start; iMesh <= end; ++iMesh) {
          meshes_[iMesh]->uploadBuffersToGPU(forceReload);
        }
      }
    }  // forceReload

    if (fileIsLoaded) {
      // if the file was loaded, the importer didn't open the file, so open it
      // before adding components
      if (!importer->openFile(filename)) {
        LOG(ERROR) << "Cannot open file " << filename;
        return false;
      }
    }

    const quatf transform = info.frame.rotationFrameToWorld();
    newNode.setRotation(Magnum::Quaternion(transform));
    // Recursively add all children
    for (auto sceneDataID : magnumMeshDict_[filename]) {
      addComponent(*importer, metaData, newNode, drawables, sceneDataID);
    }
    return true;
  }
}

void ResourceManager::loadMaterials(Importer& importer,
                                    MeshMetaData* metaData) {
  int materialStart = materials_.size();
  int materialEnd = materialStart + importer.materialCount() - 1;
  metaData->setMaterialIndices(materialStart, materialEnd);

  for (int iMaterial = 0; iMaterial < importer.materialCount(); ++iMaterial) {
    // default null material
    materials_.emplace_back(nullptr);
    auto& currentMaterial = materials_.back();

    // TODO:
    // it seems we have a way to just load the material once in this case,
    // as long as the materialName includes the full path to the material
    std::unique_ptr<Magnum::Trade::AbstractMaterialData> materialData =
        importer.material(iMaterial);
    if (!materialData ||
        materialData->type() != Magnum::Trade::MaterialType::Phong) {
      LOG(ERROR) << "Cannot load material, skipping";
      continue;
    }

    // using make_unique will not work here
    std::unique_ptr<Magnum::Trade::PhongMaterialData> phongMaterialData(
        static_cast<Magnum::Trade::PhongMaterialData*>(materialData.release()));

    currentMaterial = std::move(phongMaterialData);
  }
}

void ResourceManager::loadMeshes(Importer& importer,
                                 MeshMetaData* metaData,
                                 bool shiftOrigin /*=false*/,
                                 Magnum::Vector3 offset /* [0,0,0] */
) {
  int meshStart = meshes_.size();
  int meshEnd = meshStart + importer.mesh3DCount() - 1;
  metaData->setMeshIndices(meshStart, meshEnd);

  for (int iMesh = 0; iMesh < importer.mesh3DCount(); ++iMesh) {
    meshes_.emplace_back(std::make_unique<GltfMeshData>());
    auto& currentMesh = meshes_.back();
    auto* gltfMeshData = static_cast<GltfMeshData*>(currentMesh.get());
    gltfMeshData->setMeshData(importer, iMesh);

    // see if the mesh needs to be shifted
    if (shiftOrigin) {
      // compute BB center if necessary ([0,0,0])
      if (offset[0] == 0 && offset[1] == 0 && offset[2] == 0)
        offset = -computeMeshBBCenter(gltfMeshData);
      // translate the mesh if necessary
      if (!(offset[0] == 0 && offset[1] == 0 && offset[2] == 0))
        translateMesh(gltfMeshData, offset);
    }

    gltfMeshData->uploadBuffersToGPU(false);
  }
}

void ResourceManager::loadTextures(Importer& importer, MeshMetaData* metaData) {
  int textureStart = textures_.size();
  int textureEnd = textureStart + importer.textureCount() - 1;
  metaData->setTextureIndices(textureStart, textureEnd);

  for (int iTexture = 0; iTexture < importer.textureCount(); ++iTexture) {
    textures_.emplace_back(std::make_shared<Magnum::GL::Texture2D>());
    auto& currentTexture = textures_.back();

    auto textureData = importer.texture(iTexture);
    if (!textureData ||
        textureData->type() != Magnum::Trade::TextureData::Type::Texture2D) {
      LOG(ERROR) << "Cannot load texture " << iTexture << " skipping";
      currentTexture = nullptr;
      continue;
    }

    // TODO:
    // it seems we have a way to just load the image once in this case,
    // as long as the image2DName include the full path to the image
    Corrade::Containers::Optional<Magnum::Trade::ImageData2D> imageData =
        importer.image2D(textureData->image());
    Magnum::GL::TextureFormat format;
    if (imageData && imageData->format() == Magnum::PixelFormat::RGB8Unorm)
      format = compressTextures_
                   ? Magnum::GL::TextureFormat::CompressedRGBS3tcDxt1
                   : Magnum::GL::TextureFormat::RGB8;
    else if (imageData &&
             imageData->format() == Magnum::PixelFormat::RGBA8Unorm)
      format = compressTextures_
                   ? Magnum::GL::TextureFormat::CompressedRGBAS3tcDxt1
                   : Magnum::GL::TextureFormat::RGBA8;
    else {
      LOG(ERROR) << "Cannot load texture image, skipping";
      currentTexture = nullptr;
      continue;
    }

    // Configure the texture
    Magnum::GL::Texture2D& texture =
        *(textures_[textureStart + iTexture].get());
    texture.setMagnificationFilter(textureData->magnificationFilter())
        .setMinificationFilter(textureData->minificationFilter(),
                               textureData->mipmapFilter())
        .setWrapping(textureData->wrapping().xy())
        .setStorage(Magnum::Math::log2(imageData->size().max()) + 1, format,
                    imageData->size())
        .setSubImage(0, {}, *imageData)
        .generateMipmap();
  }
}

//! Add component to rendering stack, based on importer loading
//! TODO (JH): decouple importer part, so that objects can be
//! instantiated any time after initial loading
void ResourceManager::addComponent(Importer& importer,
                                   const MeshMetaData& metaData,
                                   scene::SceneNode& parent,
                                   DrawableGroup* drawables,
                                   int componentID) {
  std::unique_ptr<Magnum::Trade::ObjectData3D> objectData =
      importer.object3D(componentID);
  if (!objectData) {
    LOG(ERROR) << "Cannot import object " << importer.object3DName(componentID)
               << ", skipping";
    return;
  }

  // Add the object to the scene and set its transformation
  scene::SceneNode& node = parent.createChild();
  node.MagnumObject::setTransformation(objectData->transformation());

  const int meshIDLocal = objectData->instance();
  const int meshID = metaData.meshIndex.first + meshIDLocal;

  // Add a drawable if the object has a mesh and the mesh is loaded
  if (objectData->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh &&
      meshIDLocal != ID_UNDEFINED && meshes_[meshID]) {
    const int materialIDLocal =
        static_cast<Magnum::Trade::MeshObjectData3D*>(objectData.get())
            ->material();
    addMeshToDrawables(metaData, node, drawables, componentID, meshIDLocal,
                       materialIDLocal);
  }

  // Recursively add children
  for (auto childObjectID : objectData->children()) {
    addComponent(importer, metaData, node, drawables, childObjectID);
  }
}

void ResourceManager::addMeshToDrawables(const MeshMetaData& metaData,
                                         scene::SceneNode& node,
                                         DrawableGroup* drawables,
                                         int componentID,
                                         int meshIDLocal,
                                         int materialIDLocal) {
  const int meshStart = metaData.meshIndex.first;
  const int meshID = meshStart + meshIDLocal;
  Magnum::GL::Mesh& mesh = *meshes_[meshID]->getMagnumGLMesh();

  const int materialStart = metaData.materialIndex.first;
  const int materialID = materialStart + materialIDLocal;

  Magnum::GL::Texture2D* texture = nullptr;
  // Material not set / not available / not loaded, use a default material
  if (materialIDLocal == ID_UNDEFINED ||
      metaData.materialIndex.second == ID_UNDEFINED ||
      !materials_[materialID]) {
    createDrawable(COLORED_SHADER, mesh, node, drawables, texture, componentID);
  } else {
    if (materials_[materialID]->flags() &
        Magnum::Trade::PhongMaterialData::Flag::DiffuseTexture) {
      // Textured material. If the texture failed to load, again just use
      // a default colored material.
      const int textureStart = metaData.textureIndex.first;
      const int textureIndex = materials_[materialID]->diffuseTexture();
      texture = textures_[textureStart + textureIndex].get();
      if (texture) {
        createDrawable(TEXTURED_SHADER, mesh, node, drawables, texture,
                       componentID);
      } else {
        // Color-only material
        createDrawable(COLORED_SHADER, mesh, node, drawables, texture,
                       componentID, materials_[materialID]->diffuseColor());
      }
    } else {
      // Color-only material
      createDrawable(COLORED_SHADER, mesh, node, drawables, texture,
                     componentID, materials_[materialID]->diffuseColor());
    }
  }  // else
}

gfx::Drawable& ResourceManager::createDrawable(
    const ShaderType shaderType,
    Magnum::GL::Mesh& mesh,
    scene::SceneNode& node,
    Magnum::SceneGraph::DrawableGroup3D* group /* = nullptr */,
    Magnum::GL::Texture2D* texture /* = nullptr */,
    int objectId /* = ID_UNDEFINED */,
    const Magnum::Color4& color /* = Magnum::Color4{1} */) {
  gfx::Drawable* drawable = nullptr;
  if (shaderType == PTEX_MESH_SHADER) {
    LOG(ERROR)
        << "ResourceManager::createDrawable does not support PTEX_MESH_SHADER";
    ASSERT(shaderType != PTEX_MESH_SHADER);
    // NOTE: this is a runtime error and will never return
    return *drawable;
  } else if (shaderType == INSTANCE_MESH_SHADER) {
    auto* shader =
        static_cast<gfx::PrimitiveIDShader*>(getShaderProgram(shaderType));
    drawable = new gfx::PrimitiveIDDrawable{node, *shader, mesh, group};
  } else {  // all other shaders use GenericShader
    auto* shader =
        static_cast<Magnum::Shaders::Flat3D*>(getShaderProgram(shaderType));
    drawable = new gfx::GenericDrawable{node,    *shader,  mesh, group,
                                        texture, objectId, color};
  }
  return *drawable;
}

bool ResourceManager::loadSUNCGHouseFile(const AssetInfo& houseInfo,
                                         scene::SceneNode* parent,
                                         DrawableGroup* drawables) {
  ASSERT(parent != nullptr);
  std::string houseFile = Cr::Utility::Directory::join(
      Cr::Utility::Directory::current(), houseInfo.filepath);
  const auto& json = io::parseJsonFile(houseFile);
  const auto& levels = json["levels"].GetArray();
  std::vector<std::string> pathTokens = io::tokenize(houseFile, "/", 0, true);
  ASSERT(pathTokens.size() >= 3);
  pathTokens.pop_back();  // house.json
  const std::string houseId = pathTokens.back();
  pathTokens.pop_back();  // <houseId>
  pathTokens.pop_back();  // house
  const std::string basePath = Corrade::Utility::String::join(pathTokens, '/');

  // store nodeIds to obtain linearized index for semantic masks
  std::vector<std::string> nodeIds;

  for (const auto& level : levels) {
    const auto& nodes = level["nodes"].GetArray();
    for (const auto& node : nodes) {
      const std::string nodeId = node["id"].GetString();
      const std::string nodeType = node["type"].GetString();
      const int valid = node["valid"].GetInt();
      if (valid == 0) {
        continue;
      }

      // helper for creating object nodes
      auto createObjectFunc = [&](const AssetInfo& info,
                                  const std::string& id) -> scene::SceneNode& {
        scene::SceneNode& objectNode = parent->createChild();
        const int nodeIndex = nodeIds.size();
        nodeIds.push_back(id);
        objectNode.setId(nodeIndex);
        if (info.type == AssetType::SUNCG_OBJECT) {
          loadGeneralMeshData(info, &objectNode, drawables);
        }
        return objectNode;
      };

      const std::string roomPath = basePath + "/room/" + houseId + "/";
      if (nodeType == "Room") {
        const std::string roomBase = roomPath + node["modelId"].GetString();
        const int hideCeiling = node["hideCeiling"].GetInt();
        const int hideFloor = node["hideFloor"].GetInt();
        const int hideWalls = node["hideWalls"].GetInt();
        if (hideCeiling != 1) {
          createObjectFunc({AssetType::SUNCG_OBJECT, roomBase + "c.glb"},
                           nodeId + "c");
        }
        if (hideWalls != 1) {
          createObjectFunc({AssetType::SUNCG_OBJECT, roomBase + "w.glb"},
                           nodeId + "w");
        }
        if (hideFloor != 1) {
          createObjectFunc({AssetType::SUNCG_OBJECT, roomBase + "f.glb"},
                           nodeId + "f");
        }
      } else if (nodeType == "Object") {
        const std::string modelId = node["modelId"].GetString();
        // Parse model-to-scene transformation matrix
        // NOTE: only "Object" nodes have transform, other nodes are directly
        // specified in scene coordinates
        std::vector<float> transformVec;
        io::toFloatVector(node["transform"], &transformVec);
        mat4f transform(transformVec.data());
        const AssetInfo info{
            AssetType::SUNCG_OBJECT,
            basePath + "/object/" + modelId + "/" + modelId + ".glb"};
        createObjectFunc(info, nodeId)
            .setTransformation(Magnum::Matrix4{transform});
      } else if (nodeType == "Box") {
        // TODO(MS): create Box geometry
        createObjectFunc({}, nodeId);
      } else if (nodeType == "Ground") {
        const std::string roomBase = roomPath + node["modelId"].GetString();
        const AssetInfo info{AssetType::SUNCG_OBJECT, roomBase + "f.glb"};
        createObjectFunc(info, nodeId);
      } else {
        LOG(ERROR) << "Unrecognized SUNCG house node type " << nodeType;
      }
    }
  }
  return true;
}

}  // namespace assets
}  // namespace esp
