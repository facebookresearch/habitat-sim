// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/PluginManager/PluginMetadata.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/Tags.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/SceneGraph/Object.h>
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
namespace Mn = Magnum;

namespace esp {
namespace assets {

bool ResourceManager::loadScene(const AssetInfo& info,
                                scene::SceneNode* parent, /* = nullptr */
                                DrawableGroup* drawables /* = nullptr */) {
  // we only compute absolute AABB for every mesh component when loading ptex
  // mesh, or general mesh (e.g., MP3D)
  staticDrawableInfo_.clear();
  if (info.type == AssetType::FRL_PTEX_MESH ||
      info.type == AssetType::MP3D_MESH) {
    computeAbsoluteAABBs_ = true;
  }

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
        // TODO: need this anymore?
        physicsSceneLibrary_[info.filepath].setString("renderMeshHandle",
                                                      info.filepath);
      }
    }
  } else {
    LOG(INFO) << "Loading empty scene";
    // EMPTY_SCENE (ie. "NONE") string indicates desire for an empty scene (no
    // scene mesh): welcome to the void
  }

  // once a scene is loaded, we should have a GL::Context so load the primitives
  Magnum::Trade::MeshData3D cube = Magnum::Primitives::cubeWireframe();
  primitive_meshes_.push_back(Magnum::MeshTools::compile(cube));

  // compute the absolute transformation for each static drawables
  if (meshSuccess && parent && computeAbsoluteAABBs_) {
    if (info.type == AssetType::FRL_PTEX_MESH) {
#ifdef ESP_BUILD_PTEX_SUPPORT
      // retrieve the ptex mesh data
      const std::string& filename = info.filepath;
      CORRADE_ASSERT(resourceDict_.count(filename) != 0,
                     "ResourceManager::loadScene: ptex mesh is not loaded.",
                     false);
      MeshMetaData& metaData = resourceDict_.at(filename);
      CORRADE_ASSERT(
          metaData.meshIndex.first == metaData.meshIndex.second,
          "ResourceManager::loadScene: ptex mesh is not loaded correctly.",
          false);

      computePTexMeshAbsoluteAABBs(*(meshes_[metaData.meshIndex.first].get()));
#endif
    } else if (info.type == AssetType::MP3D_MESH) {
      computeGeneralMeshAbsoluteAABBs();
    }
  }

  if (computeAbsoluteAABBs_) {
    computeAbsoluteAABBs_ = false;
    // this is to prevent it from being misused in the future
    staticDrawableInfo_.clear();
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

  physicsSceneLibrary_[info.filepath].setString("renderMeshHandle",
                                                info.filepath);
  physicsSceneLibrary_[info.filepath].setString("collisionMeshHandle",
                                                info.filepath);

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
        GltfMeshData* gltfMeshData =
            dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
        CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
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
  CHECK(Cr::Utility::Directory::exists(physicsFilename));

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
  if (scenePhysicsConfig.HasMember("rigid object paths")) {
    if (scenePhysicsConfig["rigid object paths"].IsArray()) {
      physicsManagerAttributes.setVecStrings("objectLibraryPaths",
                                             std::vector<std::string>());
      for (rapidjson::SizeType i = 0;
           i < scenePhysicsConfig["rigid object paths"].Size(); i++) {
        if (scenePhysicsConfig["rigid object paths"][i].IsString()) {
          std::string filename =
              scenePhysicsConfig["rigid object paths"][i].GetString();
          std::string absolutePath =
              Cr::Utility::Directory::join(configDirectory, filename);
          if (Cr::Utility::Directory::isDirectory(absolutePath)) {
            LOG(INFO) << "Parsing object library directory: " + absolutePath;
            if (Cr::Utility::Directory::exists(absolutePath)) {
              for (auto& file : Cr::Utility::Directory::list(absolutePath)) {
                std::string absoluteSubfilePath =
                    Cr::Utility::Directory::join(absolutePath, file);
                if (Cr::Utility::String::endsWith(absoluteSubfilePath,
                                                  ".phys_properties.json")) {
                  physicsManagerAttributes.appendVecStrings(
                      "objectLibraryPaths", absoluteSubfilePath);
                }
              }
            } else {
              LOG(WARNING)
                  << "The specified directory does not exist. Aborting parse.";
            }
          } else {
            // 1: parse the filename (relative or global path)
            std::string objPhysPropertiesFilename =
                absolutePath + ".phys_properties.json";
            physicsManagerAttributes.appendVecStrings(
                "objectLibraryPaths", objPhysPropertiesFilename);
          }
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

    MeshMetaData& meshMetaData = resourceDict_[filename];

    // need a new node for scaling because motion state will override scale set
    // at the physical node
    scene::SceneNode& scalingNode = parent->createChild();
    Magnum::Vector3 objectScaling =
        physicsObjectAttributes.getMagnumVec3("scale");
    scalingNode.setScaling(objectScaling);

    addComponent(meshMetaData, scalingNode, drawables, meshMetaData.root);
    // compute the full BB hierarchy for the new tree.
    parent->computeCumulativeBB();
  }

  return objectID;
}

PhysicsObjectAttributes& ResourceManager::getPhysicsObjectAttributes(
    const std::string& objectName) {
  return physicsObjectLibrary_[objectName];
}

int ResourceManager::loadObject(PhysicsObjectAttributes& objectTemplate,
                                const std::string objectTemplateHandle) {
  CHECK(physicsObjectLibrary_.count(objectTemplateHandle) == 0);
  CHECK(objectTemplate.existsAs(STRING, "renderMeshHandle"));

  // load/check_for render and collision mesh metadata
  //! Get render mesh names
  std::string renderMeshFilename = objectTemplate.getString("renderMeshHandle");
  std::string collisionMeshFilename = "";

  if (objectTemplate.existsAs(STRING, "collisionMeshHandle")) {
    collisionMeshFilename = objectTemplate.getString("collisionMeshHandle");
  }

  bool renderMeshSuccess = false;
  bool collisionMeshSuccess = false;
  AssetInfo renderMeshinfo;
  AssetInfo collisionMeshinfo;

  //! Load rendering mesh
  if (!renderMeshFilename.empty()) {
    renderMeshinfo = assets::AssetInfo::fromPath(renderMeshFilename);
    renderMeshSuccess = loadGeneralMeshData(renderMeshinfo);
    if (!renderMeshSuccess) {
      LOG(ERROR) << "Failed to load a physical object's render mesh: "
                 << objectTemplateHandle << ", " << renderMeshFilename;
    }
  }
  //! Load collision mesh
  if (!collisionMeshFilename.empty()) {
    collisionMeshinfo = assets::AssetInfo::fromPath(collisionMeshFilename);
    collisionMeshSuccess = loadGeneralMeshData(collisionMeshinfo);
    if (!collisionMeshSuccess) {
      LOG(ERROR) << "Failed to load a physical object's collision mesh: "
                 << objectTemplateHandle << ", " << collisionMeshFilename;
    }
  }

  if (!renderMeshSuccess && !collisionMeshSuccess) {
    // we only allow objects with SOME mesh file. Failing
    // both loads or having no mesh will cancel the load.
    LOG(ERROR) << "Failed to load a physical object: no meshes...: "
               << objectTemplateHandle;
    return ID_UNDEFINED;
  }

  // handle one missing mesh
  if (!renderMeshSuccess)
    objectTemplate.setString("renderMeshHandle", collisionMeshFilename);
  if (!collisionMeshSuccess)
    objectTemplate.setString("collisionMeshHandle", renderMeshFilename);

  // cache metaData, collision mesh Group
  physicsObjectLibrary_.emplace(objectTemplateHandle, objectTemplate);
  MeshMetaData& meshMetaData =
      resourceDict_.at(objectTemplate.getString("collisionMeshHandle"));

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
  collisionMeshGroups_.emplace(objectTemplateHandle, meshGroup);
  physicsObjectConfigList_.push_back(objectTemplateHandle);

  int objectID = physicsObjectConfigList_.size() - 1;
  return objectID;
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

  // optional set bounding box as collision object
  if (objPhysicsConfig.HasMember("use bounding box for collision")) {
    if (objPhysicsConfig["use bounding box for collision"].IsBool()) {
      physicsObjectAttributes.setBool(
          "useBoundingBoxForCollision",
          objPhysicsConfig["use bounding box for collision"].GetBool());
    }
  }

  // load the center of mass (in the local frame of the object)
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
        }
      }
      physicsObjectAttributes.setMagnumVec3("COM", COM);
      // set a flag which we can find later so we don't override the desired COM
      // with BB center.
      physicsObjectAttributes.setBool("COM_provided", true);
    }
  }

  // scaling
  if (objPhysicsConfig.HasMember("scale")) {
    if (objPhysicsConfig["scale"].IsArray()) {
      Magnum::Vector3 scale;
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["scale"].Size();
           i++) {
        if (!objPhysicsConfig["scale"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << " Invalid value in object physics config - scale array";
          break;
        } else {
          scale[i] = objPhysicsConfig["scale"][i].GetDouble();
        }
      }
      physicsObjectAttributes.setMagnumVec3("scale", scale);
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

  //! Get collision configuration options if specified
  if (objPhysicsConfig.HasMember("join collision meshes")) {
    if (objPhysicsConfig["join collision meshes"].IsBool()) {
      physicsObjectAttributes.setBool(
          "joinCollisionMeshes",
          objPhysicsConfig["join collision meshes"].GetBool());
    } else {
      LOG(ERROR)
          << " Invalid value in object physics config - join collision meshes";
    }
  }

  // 4. parse render and collision mesh filepaths
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

  physicsObjectAttributes.setString("renderMeshHandle", renderMeshFilename);
  physicsObjectAttributes.setString("collisionMeshHandle",
                                    collisionMeshFilename);

  // 5. load the parsed file into the library
  return loadObject(physicsObjectAttributes, objPhysConfigFilename);
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

Magnum::Range3D ResourceManager::computeMeshBB(BaseMesh* meshDataGL) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  return Magnum::Range3D{
      Magnum::Math::minmax<Magnum::Vector3>(meshData.positions)};
}

#ifdef ESP_BUILD_PTEX_SUPPORT
void ResourceManager::computePTexMeshAbsoluteAABBs(BaseMesh& baseMesh) {
  std::vector<Mn::Matrix4> absTransforms = computeAbsoluteTransformations();

  CORRADE_ASSERT(absTransforms.size() == staticDrawableInfo_.size(),
                 "ResourceManager::computePTexMeshAbsoluteAABBs: number of "
                 "transformations does not match number of drawables.", );

  // obtain the sub-meshes within the ptex mesh
  PTexMeshData& ptexMeshData = dynamic_cast<PTexMeshData&>(baseMesh);
  const std::vector<PTexMeshData::MeshData>& submeshes = ptexMeshData.meshes();

  for (uint32_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    // convert std::vector<vec3f> to std::vector<Mn::Vector3>
    std::vector<Mn::Vector3> pos;
    uint32_t meshID = staticDrawableInfo_[iEntry].meshID;
    for (auto& p : submeshes[meshID].vbo) {
      pos.emplace_back(p);
    }

    // transform the vertex positions to the world space
    Mn::MeshTools::transformPointsInPlace(absTransforms[iEntry], pos);

    // locate the scene node which contains the current drawable
    scene::SceneNode& node = staticDrawableInfo_[iEntry].node;

    // set the absolute axis aligned bounding box
    node.setAbsoluteAABB(Mn::Range3D{Mn::Math::minmax<Mn::Vector3>(pos)});
  }
}
#endif

void ResourceManager::computeGeneralMeshAbsoluteAABBs() {
  std::vector<Mn::Matrix4> absTransforms = computeAbsoluteTransformations();

  CORRADE_ASSERT(absTransforms.size() == staticDrawableInfo_.size(),
                 "ResourceManager::computeGeneralMeshAbsoluteAABBs: number of "
                 "transforms does not match number of drawables.", );

  for (uint32_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    uint32_t meshID = staticDrawableInfo_[iEntry].meshID;

    Corrade::Containers::Optional<Magnum::Trade::MeshData3D>& meshData =
        meshes_[meshID]->getMeshData();
    CORRADE_ASSERT(meshData,
                   "ResourceManager::computeGeneralMeshAbsoluteAABBs: the "
                   "empty mesh data", );

    // a vector to store the min, max pos for the aabb of every position array
    std::vector<Mn::Vector3> bbPos;

    // transform the vertex positions to the world space, compute the aabb for
    // each position array
    for (uint32_t jArray = 0; jArray < (*meshData).positionArrayCount();
         ++jArray) {
      std::vector<Mn::Vector3>& pos = (*meshData).positions(jArray);
      std::vector<Mn::Vector3> absPos =
          Mn::MeshTools::transformPoints(absTransforms[iEntry], pos);

      std::pair<Mn::Vector3, Mn::Vector3> bb =
          Mn::Math::minmax<Mn::Vector3>(absPos);
      bbPos.push_back(bb.first);
      bbPos.push_back(bb.second);
    }

    // locate the scene node which contains the current drawable
    scene::SceneNode& node = staticDrawableInfo_[iEntry].node;

    // set the absolute axis aligned bounding box
    node.setAbsoluteAABB(Mn::Range3D{Mn::Math::minmax<Mn::Vector3>(bbPos)});

  }  // iEntry
}

std::vector<Mn::Matrix4> ResourceManager::computeAbsoluteTransformations() {
  // sanity check
  if (staticDrawableInfo_.size() == 0) {
    return std::vector<Mn::Matrix4>{};
  }

  // basic assumption is that all the drawables are in the same scene;
  // so use the 1st element in the vector to obtain this scene
  auto* scene = dynamic_cast<Mn::SceneGraph::Scene<
      Mn::SceneGraph::BasicTranslationRotationScalingTransformation3D<float>>*>(
      staticDrawableInfo_[0].node.scene());

  CORRADE_ASSERT(scene != nullptr,
                 "ResourceManager::computeAbsoluteTransformations: the node is "
                 "not attached to any scene graph.",
                 std::vector<Mn::Matrix4>{});

  // collect all drawable objects
  std::vector<std::reference_wrapper<Mn::SceneGraph::Object<
      Mn::SceneGraph::BasicTranslationRotationScalingTransformation3D<float>>>>
      objects;
  objects.reserve(staticDrawableInfo_.size());

  for (std::size_t iDrawable = 0; iDrawable < staticDrawableInfo_.size();
       ++iDrawable) {
    objects.emplace_back(
        dynamic_cast<Mn::SceneGraph::Object<
            Mn::SceneGraph::BasicTranslationRotationScalingTransformation3D<
                float>>&>(staticDrawableInfo_[iDrawable].node));
  }

  // compute transformations of all objects in the group relative to the root,
  // which are the absolute transformations
  std::vector<Mn::Matrix4> absTransforms =
      scene->transformationMatrices(objects);

  return absTransforms;
}

void ResourceManager::translateMesh(BaseMesh* meshDataGL,
                                    Magnum::Vector3 translation) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();

  Magnum::Matrix4 transform = Magnum::Matrix4::translation(translation);
  Magnum::MeshTools::transformPointsInPlace(transform, meshData.positions);
  // save the mesh transformation for future query
  meshDataGL->meshTransform_ = transform * meshDataGL->meshTransform_;

  meshDataGL->BB = meshDataGL->BB.translated(translation);
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
    resourceDict_[filename].root.meshIDLocal = 0;
    resourceDict_[filename].root.componentID = 0;
    // store the rotation to world frame upon load
    const quatf transform = info.frame.rotationFrameToWorld();
    Magnum::Matrix4 R = Magnum::Matrix4::from(
        Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
    resourceDict_[filename].root.transformFromLocalToParent =
        R * resourceDict_[filename].root.transformFromLocalToParent;
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

        // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
        new gfx::PTexMeshDrawable{node, *ptexShader, *pTexMeshData, jSubmesh,
                                  drawables};

        if (computeAbsoluteAABBs_) {
          staticDrawableInfo_.emplace_back(
              StaticDrawableInfo{node, static_cast<uint32_t>(jSubmesh)});
        }
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
  // if this is a new file, load it and add it to the dictionary, create
  // shaders and add it to the shaderPrograms_
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
    resourceDict_[filename].root.meshIDLocal = 0;
    resourceDict_[filename].root.componentID = 0;
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
    DrawableGroup* drawables /* = nullptr */) {
  const std::string& filename = info.filepath;
  const bool fileIsLoaded = resourceDict_.count(filename) > 0;
  const bool drawData = parent != nullptr && drawables != nullptr;

  // Mesh & metaData container
  MeshMetaData metaData;

#ifndef MAGNUM_BUILD_STATIC
  Magnum::PluginManager::Manager<Importer> manager;
#else
  // avoid using plugins that might depend on different library versions
  Magnum::PluginManager::Manager<Importer> manager{"nonexistent"};
#endif

  std::unique_ptr<Importer> importer =
      manager.loadAndInstantiate("AnySceneImporter");

  // Preferred plugins, Basis target GPU format
  manager.setPreferredPlugins("GltfImporter", {"TinyGltfImporter"});
#ifdef ESP_BUILD_ASSIMP_SUPPORT
  manager.setPreferredPlugins("ObjImporter", {"AssimpImporter"});
#endif
  {
    Cr::PluginManager::PluginMetadata* const metadata =
        manager.metadata("BasisImporter");
    Mn::GL::Context& context = Mn::GL::Context::current();
#ifdef MAGNUM_TARGET_WEBGL
    if (context.isExtensionSupported<
            Mn::GL::Extensions::WEBGL::compressed_texture_astc>())
#else
    if (context.isExtensionSupported<
            Mn::GL::Extensions::KHR::texture_compression_astc_ldr>())
#endif
    {
      LOG(INFO) << "Importing Basis files as ASTC 4x4";
      metadata->configuration().setValue("format", "Astc4x4RGBA");
    }
#ifdef MAGNUM_TARGET_GLES
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::EXT::texture_compression_bptc>())
#else
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::ARB::texture_compression_bptc>())
#endif
    {
      LOG(INFO) << "Importing Basis files as BC7";
      metadata->configuration().setValue("format", "Bc7RGBA");
    }
#ifdef MAGNUM_TARGET_WEBGL
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::WEBGL::compressed_texture_s3tc>())
#elif defined(MAGNUM_TARGET_GLES)
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::EXT::texture_compression_s3tc>() ||
             context.isExtensionSupported<
                 Mn::GL::Extensions::ANGLE::texture_compression_dxt5>())
#else
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::EXT::texture_compression_s3tc>())
#endif
    {
      LOG(INFO) << "Importing Basis files as BC3";
      metadata->configuration().setValue("format", "Bc3RGBA");
    }
#ifndef MAGNUM_TARGET_GLES2
    else
#ifndef MAGNUM_TARGET_GLES
        if (context.isExtensionSupported<
                Mn::GL::Extensions::ARB::ES3_compatibility>())
#endif
    {
      LOG(INFO) << "Importing Basis files as ETC2";
      metadata->configuration().setValue("format", "Etc2RGBA");
    }
#else /* For ES2, fall back to PVRTC as ETC2 is not available */
    else
#ifdef MAGNUM_TARGET_WEBGL
        if (context.isExtensionSupported<Mn::WEBGL::compressed_texture_pvrtc>())
#else
        if (context.isExtensionSupported<Mn::IMG::texture_compression_pvrtc>())
#endif
    {
      LOG(INFO) << "Importing Basis files as PVRTC 4bpp";
      metadata->configuration().setValue("format", "PvrtcRGBA4bpp");
    }
#endif
#if defined(MAGNUM_TARGET_GLES2) || !defined(MAGNUM_TARGET_GLES)
    else /* ES3 has ETC2 always */
    {
      LOG(WARNING) << "No supported GPU compressed texture format detected, "
                      "Basis images will get imported as RGBA8";
      metadata->configuration().setValue("format", "RGBA8");
    }
#endif
  }

  // Optional File loading
  if (!fileIsLoaded) {
    if (!importer->openFile(filename)) {
      LOG(ERROR) << "Cannot open file " << filename;
      return false;
    }
    // if this is a new file, load it and add it to the dictionary
    loadTextures(*importer, &metaData);
    loadMaterials(*importer, &metaData);
    loadMeshes(*importer, &metaData);
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
        loadMeshHierarchy(*importer, resourceDict_[filename].root, sceneDataID);
      }
    } else if (importer->mesh3DCount() && meshes_[metaData.meshIndex.first]) {
      // no default scene --- standalone OBJ/PLY files, for example
      // take a wild guess and load the first mesh with the first material
      // addMeshToDrawables(metaData, *parent, drawables, ID_UNDEFINED, 0, 0);
      loadMeshHierarchy(*importer, resourceDict_[filename].root, 0);
    } else {
      LOG(ERROR) << "No default scene available and no meshes found, exiting";
      return false;
    }

    const quatf transform = info.frame.rotationFrameToWorld();
    Magnum::Matrix4 R = Magnum::Matrix4::from(
        Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
    resourceDict_[filename].root.transformFromLocalToParent =
        R * resourceDict_[filename].root.transformFromLocalToParent;
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

    addComponent(metaData, newNode, drawables, metaData.root);
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

void ResourceManager::loadMeshes(Importer& importer, MeshMetaData* metaData) {
  int meshStart = meshes_.size();
  int meshEnd = meshStart + importer.mesh3DCount() - 1;
  metaData->setMeshIndices(meshStart, meshEnd);

  for (int iMesh = 0; iMesh < importer.mesh3DCount(); ++iMesh) {
    meshes_.emplace_back(std::make_unique<GltfMeshData>());
    auto& currentMesh = meshes_.back();
    auto* gltfMeshData = static_cast<GltfMeshData*>(currentMesh.get());
    gltfMeshData->setMeshData(importer, iMesh);

    // compute the mesh bounding box
    gltfMeshData->BB = computeMeshBB(gltfMeshData);

    gltfMeshData->uploadBuffersToGPU(false);
  }
}

//! Recursively load the transformation chain specified by the mesh file
void ResourceManager::loadMeshHierarchy(Importer& importer,
                                        MeshTransformNode& parent,
                                        int componentID) {
  std::unique_ptr<Magnum::Trade::ObjectData3D> objectData =
      importer.object3D(componentID);
  if (!objectData) {
    LOG(ERROR) << "Cannot import object " << importer.object3DName(componentID)
               << ", skipping";
    return;
  }

  // Add the new node to the hierarchy and set its transformation
  parent.children.push_back(MeshTransformNode());
  parent.children.back().transformFromLocalToParent =
      objectData->transformation();
  parent.children.back().componentID = componentID;

  const int meshIDLocal = objectData->instance();

  // Add a mesh index
  if (objectData->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh &&
      meshIDLocal != ID_UNDEFINED) {
    parent.children.back().meshIDLocal = meshIDLocal;
    parent.children.back().materialIDLocal =
        static_cast<Magnum::Trade::MeshObjectData3D*>(objectData.get())
            ->material();
  }

  // Recursively add children
  for (auto childObjectID : objectData->children()) {
    loadMeshHierarchy(importer, parent.children.back(), childObjectID);
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
    if (imageData && imageData->isCompressed())
      format = Mn::GL::textureFormat(imageData->compressedFormat());
    else if (imageData && imageData->format() == Magnum::PixelFormat::RGB8Unorm)
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
        .setWrapping(textureData->wrapping().xy());

    if (!imageData->isCompressed()) {
      texture
          .setStorage(Magnum::Math::log2(imageData->size().max()) + 1, format,
                      imageData->size())
          .setSubImage(0, {}, *imageData)
          .generateMipmap();
    } else {
      texture.setStorage(1, format, imageData->size())
          .setCompressedSubImage(0, {}, *imageData);
      // TODO: load mips from the Basis file once Magnum supports that
    }
  }
}

//! Add component to rendering stack, based on importer loading
//! TODO (JH): decouple importer part, so that objects can be
//! instantiated any time after initial loading
void ResourceManager::addComponent(const MeshMetaData& metaData,
                                   scene::SceneNode& parent,
                                   DrawableGroup* drawables,
                                   const MeshTransformNode& meshTransformNode) {
  // Add the object to the scene and set its transformation
  scene::SceneNode& node = parent.createChild();
  node.MagnumObject::setTransformation(
      meshTransformNode.transformFromLocalToParent);

  const int meshIDLocal = meshTransformNode.meshIDLocal;

  // Add a drawable if the object has a mesh and the mesh is loaded
  if (meshIDLocal != ID_UNDEFINED) {
    const int materialIDLocal = meshTransformNode.materialIDLocal;
    addMeshToDrawables(metaData, node, drawables, meshTransformNode.componentID,
                       meshIDLocal, materialIDLocal);

    // compute the bounding box for the mesh we are adding
    const int meshID = metaData.meshIndex.first + meshIDLocal;
    BaseMesh* mesh = meshes_[meshID].get();
    node.setMeshBB(computeMeshBB(mesh));
  }

  // Recursively add children
  for (auto& child : meshTransformNode.children) {
    addComponent(metaData, node, drawables, child);
  }
}

void ResourceManager::addMeshToDrawables(const MeshMetaData& metaData,
                                         scene::SceneNode& node,
                                         DrawableGroup* drawables,
                                         int objectID,
                                         int meshIDLocal,
                                         int materialIDLocal) {
  const int meshStart = metaData.meshIndex.first;
  const uint32_t meshID = meshStart + meshIDLocal;
  Magnum::GL::Mesh& mesh = *meshes_[meshID]->getMagnumGLMesh();

  const int materialStart = metaData.materialIndex.first;
  const int materialID = materialStart + materialIDLocal;

  Magnum::GL::Texture2D* texture = nullptr;
  // Material not set / not available / not loaded, use a default material
  if (materialIDLocal == ID_UNDEFINED ||
      metaData.materialIndex.second == ID_UNDEFINED ||
      !materials_[materialID]) {
    createDrawable(COLORED_SHADER, mesh, node, drawables, texture, objectID);
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
                       objectID);
      } else {
        // Color-only material
        createDrawable(COLORED_SHADER, mesh, node, drawables, texture, objectID,
                       materials_[materialID]->diffuseColor());
      }
    } else {
      // TODO: some types (such as .ply with vertex color) get binned here
      // incorrectly.
      // Color-only material
      createDrawable(COLORED_SHADER, mesh, node, drawables, texture, objectID,
                     materials_[materialID]->diffuseColor());
    }
  }  // else

  if (computeAbsoluteAABBs_) {
    staticDrawableInfo_.emplace_back(StaticDrawableInfo{node, meshID});
  }
}

void ResourceManager::addPrimitiveToDrawables(int primitiveID,
                                              scene::SceneNode& node,
                                              DrawableGroup* drawables) {
  CHECK(primitiveID >= 0 && primitiveID < primitive_meshes_.size());
  createDrawable(ShaderType::COLORED_SHADER, primitive_meshes_[primitiveID],
                 node, drawables);
}

void ResourceManager::createDrawable(
    const ShaderType shaderType,
    Magnum::GL::Mesh& mesh,
    scene::SceneNode& node,
    DrawableGroup* group /* = nullptr */,
    Magnum::GL::Texture2D* texture /* = nullptr */,
    int objectId /* = ID_UNDEFINED */,
    const Magnum::Color4& color /* = Magnum::Color4{1} */) {
  if (shaderType == PTEX_MESH_SHADER) {
    LOG(FATAL) << "ResourceManager::createDrawable does not support "
                  "PTEX_MESH_SHADER";
  } else if (shaderType == INSTANCE_MESH_SHADER) {
    auto* shader =
        static_cast<gfx::PrimitiveIDShader*>(getShaderProgram(shaderType));
    node.addFeature<gfx::PrimitiveIDDrawable>(*shader, mesh, group);
  } else {  // all other shaders use GenericShader
    auto* shader =
        static_cast<Magnum::Shaders::Flat3D*>(getShaderProgram(shaderType));
    node.addFeature<gfx::GenericDrawable>(*shader, mesh, group, texture,
                                          objectId, color);
  }
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

//! recursively join all sub-components of a mesh into a single unified
//! MeshData.
void ResourceManager::joinHeirarchy(
    MeshData& mesh,
    const MeshMetaData& metaData,
    const MeshTransformNode& node,
    const Magnum::Matrix4& transformFromParentToWorld) {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  if (node.meshIDLocal != ID_UNDEFINED) {
    CollisionMeshData& meshData =
        meshes_[node.meshIDLocal + metaData.meshIndex.first]
            ->getCollisionMeshData();
    int lastIndex = mesh.vbo.size();
    for (auto& pos : meshData.positions) {
      mesh.vbo.push_back(Magnum::EigenIntegration::cast<vec3f>(
          transformFromLocalToWorld.transformPoint(pos)));
    }
    for (auto& index : meshData.indices) {
      mesh.ibo.push_back(index + lastIndex);
    }
  }

  for (auto& child : node.children) {
    joinHeirarchy(mesh, metaData, child, transformFromLocalToWorld);
  }
}

std::unique_ptr<MeshData> ResourceManager::createJoinedCollisionMesh(
    const std::string& filename) {
  std::unique_ptr<MeshData> mesh = std::make_unique<MeshData>();

  CHECK(resourceDict_.count(filename) > 0);

  MeshMetaData& metaData = resourceDict_.at(filename);

  Magnum::Matrix4 identity;
  joinHeirarchy(*mesh, metaData, metaData.root, identity);

  return mesh;
}

}  // namespace assets
}  // namespace esp
