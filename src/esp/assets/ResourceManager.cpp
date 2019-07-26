// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/Math/Tags.h>

#include "esp/geo/geo.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/GenericShader.h"
#include "esp/gfx/PTexMeshDrawable.h"
#include "esp/gfx/PTexMeshShader.h"
#include "esp/io/io.h"
#include "esp/io/json.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneGraph.h"

#include "FRLInstanceMeshData.h"
#include "GenericInstanceMeshData.h"
#include "Mp3dInstanceMeshData.h"
#include "PTexMeshData.h"
#include "CollisionMeshData.h"
#include "MeshData.h"
#include "ResourceManager.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/BulletPhysicsManager.h" //Alex TODO: will this need to change for conditional build?

namespace esp {
namespace assets {

bool ResourceManager::loadScene(
    const AssetInfo& info,
    scene::SceneNode* parent,                 /* = nullptr */
    DrawableGroup* drawables                 /* = nullptr */) {

  //scene mesh loading
  bool meshSuccess = false;
  if (!io::exists(info.filepath)) {
    LOG(ERROR) << "Cannot load from file " << info.filepath;
    meshSuccess = false;
  }else{
    scene::SceneNode* sceneNode = nullptr;
    if (info.type == AssetType::FRL_INSTANCE_MESH ||
        info.type == AssetType::INSTANCE_MESH) {
      LOG(INFO) << "Loading FRL/Instance mesh data";
      meshSuccess = loadInstanceMeshData(info, parent, drawables);
    } else if (info.type == AssetType::FRL_PTEX_MESH) {
      LOG(INFO) << "Loading PTEX mesh data";
      meshSuccess = loadPTexMeshData(info, parent, drawables);
    } else if (info.type == AssetType::SUNCG_SCENE) {
      meshSuccess = loadSUNCGHouseFile(info, parent, drawables);
    } else if (info.type == AssetType::MP3D_MESH) {
      LOG(INFO) << "Loading MP3D mesh data";
      LOG(INFO) << "Parent " << parent << " drawables " << drawables;
      meshSuccess = loadGeneralMeshData(info, parent, drawables);
    } else {
      // Unknown type, just load general mesh data
      LOG(INFO) << "Loading General mesh data";
      meshSuccess = loadGeneralMeshData(info, parent, drawables);
    }
  }

  LOG(INFO) << "Loaded mesh scene, success " << meshSuccess << " parent " << parent << " drawables " << drawables;
  return meshSuccess;
}


//! (1) load and instantiate scene
//! (2) Read config and set physics timestep
//! (3) Initialize physics engine
// TODO (JH): this function seems to entangle certain physicsManager functions
bool ResourceManager::loadScene(
    const AssetInfo& info,
    std::shared_ptr<physics::PhysicsManager>& _physicsManager,
    scene::SceneNode* parent,                 /* = nullptr */
    DrawableGroup* drawables,                 /* = nullptr */
    std::string physicsFilename /* data/default.phys_scene_config.json */
    ) {

  //scene mesh loading
  bool meshSuccess = false;
  if (!io::exists(info.filepath)) {
    LOG(ERROR) << "Cannot load from file " << info.filepath;
    meshSuccess = false;
  }else{
    scene::SceneNode* sceneNode = nullptr;
    if (info.type == AssetType::FRL_INSTANCE_MESH ||
        info.type == AssetType::INSTANCE_MESH) {
      LOG(INFO) << "Loading FRL/Instance mesh data";
      meshSuccess = loadInstanceMeshData(info, parent, drawables);
    } else if (info.type == AssetType::FRL_PTEX_MESH) {
      LOG(INFO) << "Loading PTEX mesh data";
      meshSuccess = loadPTexMeshData(info, parent, drawables);
    } else if (info.type == AssetType::SUNCG_SCENE) {
      meshSuccess = loadSUNCGHouseFile(info, parent, drawables);
    } else if (info.type == AssetType::MP3D_MESH) {
      LOG(INFO) << "Loading MP3D mesh data";
      LOG(INFO) << "Parent " << parent << " drawables " << drawables;
      meshSuccess = loadGeneralMeshData(info, parent, drawables);
    } else {
      // Unknown type, just load general mesh data
      LOG(INFO) << "Loading General mesh data";
      meshSuccess = loadGeneralMeshData(info, parent, drawables);
    }
  }

  LOG(INFO) << "Loaded mesh scene, success " << meshSuccess << " parent " << parent << " drawables " << drawables;

  //if physics is enabled, initialize the physical scene
  LOG(INFO) << "Loading physics config... " << physicsFilename;

  //Load the global scene config JSON here
  io::JsonDocument scenePhysicsConfig = io::parseJsonFile(physicsFilename);
  LOG(INFO) << "...parsed config ";

  //load the simulator preference
  //default is no simulator
  std::string simulator = "none";
  if(scenePhysicsConfig.HasMember("physics simulator")){
    if(scenePhysicsConfig["physics simulator"].IsString()){
      std::string selectedSimulator = scenePhysicsConfig["physics simulator"].GetString();
      if(selectedSimulator.compare("bullet") == 0){
        simulator = "bullet";
      }else{
        //default to kinematic simulator
        simulator = "none";
      }
    }
  }

  //load the physics timestep
  double dt = 0.01;
  if(scenePhysicsConfig.HasMember("timestep")){
    if(scenePhysicsConfig["timestep"].IsNumber()){
      dt = scenePhysicsConfig["timestep"].GetDouble();
    }
  }

  //load gravity
  Magnum::Vector3d gravity(0,-9.81,0);       // default gravity
  if(scenePhysicsConfig.HasMember("gravity")){
    if(scenePhysicsConfig["gravity"].IsArray()){
      for (rapidjson::SizeType i = 0; i < scenePhysicsConfig["gravity"].Size(); i++){
        if(!scenePhysicsConfig["gravity"][i].IsNumber()){
          //invalid config
          LOG(ERROR) << "Invalid value in physics gravity array"; break;
        }else{
          gravity[i] = scenePhysicsConfig["gravity"][i].GetDouble();
        }
      }
    }
  }

  //! PHYSICS INIT: Use the above config to initialize physics engine
  
  if (simulator.compare("bullet") == 0)
    _physicsManager.reset(new physics::BulletPhysicsManager(this));
  _physicsManager->initPhysics(parent, gravity);
  _physicsManager->setTimestep(dt);

  //! LOAD OBJECTS
  LOG(INFO) << "...loading rigid body library metadata from individual paths";
  std::string configDirectory = physicsFilename.substr(0,
      physicsFilename.find_last_of("/"));
  LOG(INFO) << "...dir = " << configDirectory;
  //load the rigid object library metadata (no physics init yet...)
  //ALEX NOTE: expect relative paths to the global config
  if(scenePhysicsConfig.HasMember("rigid object paths")){
    if(scenePhysicsConfig["rigid object paths"].IsArray()){
      for (rapidjson::SizeType i = 0; 
          i < scenePhysicsConfig["rigid object paths"].Size(); 
          i++)
      {
        if(scenePhysicsConfig["rigid object paths"][i].IsString()){
          //1: read the filename (relative path)
          std::string objPhysPropertiesFilename = configDirectory;
          objPhysPropertiesFilename.append("/").append(
              scenePhysicsConfig["rigid object paths"][i].GetString()).append(
              ".phys_properties.json");
          //get the absolute path
          LOG(INFO) << "...   obj properties path = " << objPhysPropertiesFilename;

          //2. loadObject()
          int sceneID = loadObject(objPhysPropertiesFilename);
        }else{
          LOG(ERROR) << "Invalid value in physics scene config -rigid object library- array " << i;
        }
      }
    }
  }
  //ALEX TODO: load objects property files from an entire directory (a property dataset/library...)

  //! CONSTRUCT SCENE
  const std::string& filename = info.filepath;
  MeshMetaData& metaData = resourceDict_.at(filename);
  auto indexPair = metaData.meshIndex;
  int start = indexPair.first;
  int end = indexPair.second;

  //! Collect collision mesh group
  std::vector<CollisionMeshData> meshGroup;
  LOG(INFO) << "Accessing scene mesh start " << start << " end " << end;
  for (int mesh_i = start; mesh_i <= end; mesh_i++) {

    // FRL Quad Mesh
    if (info.type == AssetType::FRL_INSTANCE_MESH) {
      LOG(INFO) << "Loading FRL scene";
      FRLInstanceMeshData* frlMeshData = 
          dynamic_cast<FRLInstanceMeshData*>(meshes_[mesh_i].get());
      CollisionMeshData& meshData = frlMeshData->getCollisionMeshData();
      meshGroup.push_back(meshData);
    } 

    // PLY Instance mesh
    else if (info.type == AssetType::INSTANCE_MESH) {
      LOG(INFO) << "Loading PLY scene";
      GenericInstanceMeshData* insMeshData = 
          dynamic_cast<GenericInstanceMeshData*>(meshes_[mesh_i].get());
      quatf quatf = quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
      Magnum::Quaternion quat = Magnum::Quaternion(quatf);
      CollisionMeshData& meshData = insMeshData->getCollisionMeshData();
      meshGroup.push_back(meshData);
    }

    // GLB Mesh
    else if (info.type == AssetType::MP3D_MESH) {
      LOG(INFO) << "Loading GLB scene";
      quatf quatf = quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
      Magnum::Quaternion quat = Magnum::Quaternion(quatf);
      GltfMeshData* gltfMeshData = dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
      CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
      Magnum::Matrix4 transform =
          Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());
      Magnum::MeshTools::transformPointsInPlace(transform,
                                                meshData.positions);
      meshGroup.push_back(meshData);
    }
  }
  //! Initialize collision mesh
  bool sceneSuccess = _physicsManager->addScene(info, meshGroup);
  LOG(INFO) << "Initialized mesh scene, success " << sceneSuccess;
  if (!sceneSuccess) {
    LOG(INFO) << "Physics manager failed to initialize object";
    return false;
  }

  return meshSuccess;
}

// Add object by object key
int ResourceManager::addObject(
    const std::string configFile,
    scene::SceneNode* parent,
    DrawableGroup* drawables) 
{
  LOG(INFO) << "Resource add object " << configFile;
  int objectID = getObjectID(configFile);
  if (objectID < 0) {
    return -1;
  }
  LOG(INFO) << "Resource add object " << configFile << " " << objectID;
  LOG(INFO) << "Resource total object " << physicsObjectConfigList_.size();
  LOG(INFO) << "Parent " << parent << " drawables " << drawables;
  return addObject(objectID, parent, drawables);
}

// Add object by ID
int ResourceManager::addObject(
    const int objectID,
    scene::SceneNode* parent,
    DrawableGroup* drawables) 
{
  std::string objPhysConfigFilename = getObjectConfig(objectID);
  LOG(INFO) << "Resource add object " << objPhysConfigFilename << " " << objectID;
  int physObjectID = loadObject(objPhysConfigFilename, parent, drawables);
  return physObjectID;
}

//! Only load and does not instantiate object
//! For load-only: set parent = nullptr, drawables = nullptr
int ResourceManager::loadObject(
    const std::string objPhysConfigFilename,
    scene::SceneNode* parent,
    DrawableGroup* drawables) 
{
  // Load Object from config
  const bool objectIsLoaded = 
      physicsObjectLibrary_.count(objPhysConfigFilename) > 0;

  LOG(INFO) << "Loaded mesh object, drawables " << drawables;

  // Find objectID in resourceManager
  int objectID = -1;
  if (!objectIsLoaded) {
    // Main loading function
    objectID = loadObject(objPhysConfigFilename);

  } else {
    std::vector<std::string>::iterator itr = std::find(
      physicsObjectConfigList_.begin(), 
      physicsObjectConfigList_.end(), 
      objPhysConfigFilename);
    objectID = std::distance(physicsObjectConfigList_.begin(), itr);
    LOG(INFO) << "Object ID " << objectID;
  }

  if (parent != nullptr and drawables != nullptr) {
    //! Add mesh to rendering stack

    // Meta data and collision mesh
    PhysicsObjectMetaData physMetaData = 
        physicsObjectLibrary_[objPhysConfigFilename];
    std::vector<CollisionMeshData> meshGroup = 
        collisionMeshGroups_[objPhysConfigFilename];

    const std::string& filename = physMetaData.renderMeshHandle;
    
    MeshMetaData meshMetaData = resourceDict_[filename];
    scene::SceneNode& newNode = parent->createChild();
    //scene::SceneNode& newNode = *parent;
    AssetInfo renderMeshinfo  = AssetInfo::fromPath(filename);

    for (auto componentID : magnumMeshDict_[filename]) {
      LOG(INFO) << "Scene data ID " << componentID << " " << filename; 
      addComponent(*importer, renderMeshinfo, meshMetaData, newNode, 
          drawables, componentID);
    }
  }

  return objectID;
}

PhysicsObjectMetaData& ResourceManager::getPhysicsMetaData(
    const std::string objectName) 
{
  return physicsObjectLibrary_[objectName];
}

//load object from config filename
int ResourceManager::loadObject(const std::string objPhysConfigFilename)
{
  LOG(INFO) << "Load object " << objPhysConfigFilename;
  //check for duplicate load
  const bool objExists = physicsObjectLibrary_.count(objPhysConfigFilename) > 0;
  if(objExists){
    //ALEX TODO: for now this will skip the duplicate. Is there a good reason to allow duplicates?
    std::vector<std::string>::iterator itr = std::find(
      physicsObjectConfigList_.begin(), 
      physicsObjectConfigList_.end(), 
      objPhysConfigFilename);
    int objectID = std::distance(physicsObjectConfigList_.begin(), itr);
    return objectID;
  }

  //1. parse the config file
  //ALEX NOTE: we could create a datastructure of parsed JSON property files before creating individual entries...
  io::JsonDocument objPhysicsConfig = io::parseJsonFile(objPhysConfigFilename);

  //2. construct a physicsObjectMetaData
  PhysicsObjectMetaData physMetaData;

  //ALEX NOTE: these paths should be relative to the properties file
  std::string propertiesFileDirectory = objPhysConfigFilename.substr(0,
      objPhysConfigFilename.find_last_of("/"));

  //3. load physical properties to override defaults (set in PhysicsObjectMetaData.h)
  //load the mass
  if(objPhysicsConfig.HasMember("mass")){
    if(objPhysicsConfig["mass"].IsNumber()){
      physMetaData.mass = objPhysicsConfig["mass"].GetDouble();
    }
  }

  //load the center of mass (in the local frame of the object)
  bool shouldComputeMeshBBCenter = true; //if COM is provided, use it for mesh shift
  if(objPhysicsConfig.HasMember("COM")){
    if(objPhysicsConfig["COM"].IsArray()){
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["COM"].Size(); i++){
        if(!objPhysicsConfig["COM"][i].IsNumber()){
          //invalid config
          LOG(ERROR) << "Invalid value in object physics config COM array"; break;
        }else{
          physMetaData.COM[i] = objPhysicsConfig["COM"][i].GetDouble();
          shouldComputeMeshBBCenter = false;
        }
      }
    }
  }

  //load the inertia diagonal
  if(objPhysicsConfig.HasMember("inertia")){
    if(objPhysicsConfig["inertia"].IsArray()){
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["inertia"].Size(); i++){
        if(!objPhysicsConfig["inertia"][i].IsNumber()){
          //invalid config
          LOG(ERROR) << "Invalid value in object physics config inertia array"; break;
        }else{
          physMetaData.inertia[i] = objPhysicsConfig["inertia"][i].GetDouble();
        }
      }
    }
  }

  //load the friction coefficient
  if(objPhysicsConfig.HasMember("friction coefficient")){
    if(objPhysicsConfig["friction coefficient"].IsNumber()){
      physMetaData.frictionCoefficient = objPhysicsConfig["friction coefficient"].GetDouble();
    }
  }

  //load the restitution coefficient
  if(objPhysicsConfig.HasMember("restitution coefficient")){
    if(objPhysicsConfig["restitution coefficient"].IsNumber()){
      physMetaData.restitutionCoefficient = objPhysicsConfig["restitution coefficient"].GetDouble();
    }
  }

  //3. load/check_for render and collision mesh metadata
  //! Get render mesh names
  std::string renderMeshFilename = "";
  std::string collisionMeshFilename = "";

  if(objPhysicsConfig.HasMember("render mesh")){
    if(objPhysicsConfig["render mesh"].IsString()){
      renderMeshFilename = propertiesFileDirectory;
      renderMeshFilename.append("/").append(
          objPhysicsConfig["render mesh"].GetString());
    }
  }
  if(objPhysicsConfig.HasMember("collision mesh")){
    if(objPhysicsConfig["collision mesh"].IsString()){
      collisionMeshFilename = propertiesFileDirectory;
      collisionMeshFilename.append("/").append(
          objPhysicsConfig["collision mesh"].GetString());
    }
  }

  bool renderMeshSuccess = false;
  bool collisionMeshSuccess = false;
  AssetInfo renderMeshinfo;
  AssetInfo collisionMeshinfo;

  bool shiftMeshOrigin = !(physMetaData.COM[0]==0 && physMetaData.COM[1]==0 && physMetaData.COM[2]==0);

  //! Load rendering mesh
  //Alex TODO: add other mesh types and unify the COM move: don't want simplified meshes to result in different rendering and collision COMs/origins
  if (!renderMeshFilename.empty()){
    renderMeshinfo = assets::AssetInfo::fromPath(renderMeshFilename);
    //if (renderMeshinfo.type != AssetType::MP3D_MESH) {
    //  LOG(INFO) << "Cannot load non-GLB objects";
    //  return -1;
    //}
    if (shouldComputeMeshBBCenter){ //compute the COM from BB center
      LOG(INFO) << "LOADING BB CENTER COM>>>>>>>>>>>>>>>>>>>>>>>>>>>";
      renderMeshSuccess = loadGeneralMeshData(renderMeshinfo, nullptr, nullptr, true);
    }else if (shiftMeshOrigin){ //use the provided COM
      renderMeshSuccess = loadGeneralMeshData(renderMeshinfo, nullptr, nullptr, true, -Magnum::Vector3(physMetaData.COM[0], physMetaData.COM[1], physMetaData.COM[2]));
    }else{ //mesh origin already at COM
      renderMeshSuccess = loadGeneralMeshData(renderMeshinfo);
    }
    if(!renderMeshSuccess) {
      LOG(ERROR) << "Failed to load a physical object's render mesh: " << 
          objPhysConfigFilename << ", " << renderMeshFilename;
    }
  }
  //! Load collision mesh
  if (!collisionMeshFilename.empty()){
    collisionMeshinfo = assets::AssetInfo::fromPath(collisionMeshFilename);
    //if (collisionMeshinfo.type != AssetType::MP3D_MESH) {
    //  LOG(INFO) << "Cannot load non-GLB objects";
    //  return -1;
    //}
    if (shouldComputeMeshBBCenter){ //compute the COM from BB center
      collisionMeshSuccess = loadGeneralMeshData(collisionMeshinfo, nullptr, nullptr, true);
    }else if (shiftMeshOrigin){ //use the provided COM
      collisionMeshSuccess = loadGeneralMeshData(collisionMeshinfo, nullptr, nullptr, true, -Magnum::Vector3(physMetaData.COM[0], physMetaData.COM[1], physMetaData.COM[2]));
    }else{ //mesh origin already at COM
      collisionMeshSuccess = loadGeneralMeshData(collisionMeshinfo);
    }
    if(!collisionMeshSuccess) {
      LOG(ERROR) << "Failed to load a physical object's collision mesh: " << 
          objPhysConfigFilename << ", " << collisionMeshFilename;
    }
  }

  //Alex NOTE: if we want to save these after edit we need to save the moved mesh or save the original COM as a member of RigidBody...
  physMetaData.COM = Magnum::Vector3d(0,0,0); //once we move the meshes, the COM is aligned with the origin...

  if(!renderMeshSuccess && !collisionMeshSuccess){
    //ALEX TODO: for now we only allow objects with SOME mesh file. Failing both loads or having no mesh will cancel the load.
    LOG(ERROR) << "Failed to load a physical object: no meshes...: " << objPhysConfigFilename;
    return -1;
  }

  physMetaData.renderMeshHandle = renderMeshFilename;
  physMetaData.collisionMeshHandle = collisionMeshFilename;

  //handle one missing mesh
  if(!renderMeshSuccess)
    physMetaData.renderMeshHandle = collisionMeshFilename;
  if(!collisionMeshSuccess)
    physMetaData.collisionMeshHandle = renderMeshFilename;

  //5. cache metaData, collision mesh Group
  physicsObjectLibrary_.emplace(objPhysConfigFilename, physMetaData);
  LOG(INFO) << "Config " << objPhysConfigFilename;
  LOG(INFO) << "physMetaData.collisionMeshHandle " << physMetaData.collisionMeshHandle;
  LOG(INFO) << "Inertia " << physMetaData.inertia.x() << " " << physMetaData.inertia.y();
  MeshMetaData& meshMetaData = resourceDict_.at(physMetaData.collisionMeshHandle);

  int start = meshMetaData.meshIndex.first;
  int end   = meshMetaData.meshIndex.second;
  LOG(INFO) << "Accessing object mesh start " << start << " end " << end;
  //! Gather mesh components for meshGroup data
  std::vector<CollisionMeshData> meshGroup;
  for (int mesh_i = start; mesh_i <= end; mesh_i++) {
    GltfMeshData* gltfMeshData = dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
    CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
    meshGroup.push_back(meshData);
  }
  //! Properly align axis direction
  //ALEX NOTE: this breaks the collision properties of some files
  //LOG(INFO) << "Skipping  transformAxis for object meshes to avoid observed issues...";
  //transformAxis(renderMeshinfo, meshGroup);

  collisionMeshGroups_.emplace(objPhysConfigFilename, meshGroup);
  LOG(INFO) << "Config filename " << objPhysConfigFilename;
  physicsObjectConfigList_.push_back(objPhysConfigFilename);

  int objectID = physicsObjectConfigList_.size() - 1;
  return objectID;
}

// TODO (JH) there lack a keyname to store this object, e.g.objPhysConfigFilename
int ResourceManager::loadDefaultObject(
    const std::string renderMeshFilename,
    const std::string collisionMeshFilename /* "" */   ){
  
  const bool objectIsLoaded = 
      physicsObjectLibrary_.count(renderMeshFilename) > 0;

  int objectID;
  if (objectIsLoaded) {
    std::vector<std::string>::iterator itr = std::find(
      physicsObjectConfigList_.begin(), 
      physicsObjectConfigList_.end(), 
      renderMeshFilename);
    objectID = std::distance(physicsObjectConfigList_.begin(), itr);

  } else {
    //construct a physicsObjectMetaData
    PhysicsObjectMetaData physMetaData;

    //load desired meshes
    bool renderMeshSuccess = false;
    bool collisionMeshSuccess = false;
    if (!renderMeshFilename.empty()){
      const AssetInfo& renderMeshinfo = assets::AssetInfo::fromPath(renderMeshFilename);
      renderMeshSuccess = loadGeneralMeshData(renderMeshinfo);
      if(!renderMeshSuccess)
        LOG(ERROR) << "Failed to load a physical object's mesh: " << renderMeshFilename;
    }
    if (!collisionMeshFilename.empty()){
      const AssetInfo& collisionMeshinfo = assets::AssetInfo::fromPath(collisionMeshFilename);
      collisionMeshSuccess = loadGeneralMeshData(collisionMeshinfo);
      if(!collisionMeshSuccess)
        LOG(ERROR) << "Failed to load a physical object's mesh: " << collisionMeshFilename;
    }

    if(!renderMeshSuccess && !collisionMeshSuccess){
      //ALEX TODO: for now we only allow objects with SOME mesh file. Failing both loads or having no mesh will cancel the load.
      LOG(ERROR) << "Failed to load a physical object: no meshes...: ";
      return false;
    }

    physMetaData.renderMeshHandle = renderMeshFilename;
    physMetaData.collisionMeshHandle = collisionMeshFilename;

    //cache it
    physicsObjectLibrary_.emplace(renderMeshFilename, physMetaData);
    physicsObjectConfigList_.push_back(renderMeshFilename);

    objectID = physicsObjectConfigList_.size() - 1;

  }

  return objectID;
}


void ResourceManager::transformAxis(
    const AssetInfo& info,
    std::vector<CollisionMeshData> meshGroup) {

  quatf quatf = quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
  Magnum::Quaternion quat = Magnum::Quaternion(quatf);
  //LOG(INFO) << "quat: " << (float)(quat.angle()) << " [" << quat.axis().x() << " " << quat.axis().y() << " " << quat.axis().z() << "]";
  Magnum::Matrix4 transform;
  if(!(std::isnan(quat.axis().x()) || std::isnan(quat.axis().y()) || std::isnan(quat.axis().z()))){
    transform = Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());
  }

  for (CollisionMeshData& meshData: meshGroup) {
    Magnum::MeshTools::transformPointsInPlace(transform, meshData.positions);
  }
}

std::vector<assets::CollisionMeshData> ResourceManager::getCollisionMesh(
    const int objectID) {
  if (objectID < 0 || objectID > collisionMeshGroups_.size()) {
    return std::vector<assets::CollisionMeshData>();
  }
  std::string configFile = getObjectConfig(objectID);
  return collisionMeshGroups_[configFile];
}

std::vector<assets::CollisionMeshData> ResourceManager::getCollisionMesh(
    const std::string configFile) {
  if (collisionMeshGroups_.count(configFile) > 0) {
    return collisionMeshGroups_[configFile];
  } else {
    return std::vector<assets::CollisionMeshData>();
  }

}

int ResourceManager::getObjectID(std::string configFile) {
  std::vector<std::string>::iterator itr = std::find(
      physicsObjectConfigList_.begin(), physicsObjectConfigList_.end(), configFile);
  if (itr == physicsObjectConfigList_.cend()) {
    return -1;
  } else {
    int objectID = std::distance(physicsObjectConfigList_.begin(), itr);
    return objectID;
  }
}

std::string ResourceManager::getObjectConfig(int objectID) {
  return physicsObjectConfigList_[objectID];
}


Magnum::Vector3 ResourceManager::computeMeshBBCenter(GltfMeshData* meshDataGL){
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  Magnum::Vector3 maxCorner(-999999.9), minCorner(999999.9);
  for (int vi = 0; vi < meshData.positions.size(); vi++) {
    Magnum::Vector3 pos = meshData.positions[vi];
    maxCorner[0] = fmax(maxCorner.x(), pos.x());
    maxCorner[1] = fmax(maxCorner.y(), pos.y());
    maxCorner[2] = fmax(maxCorner.z(), pos.z());

    minCorner[0] = fmin(minCorner.x(), pos.x());
    minCorner[1] = fmin(minCorner.y(), pos.y());
    minCorner[2] = fmin(minCorner.z(), pos.z());
  }

  return (maxCorner+minCorner)/2.0;
}

void ResourceManager::translateMesh(GltfMeshData* meshDataGL, Magnum::Vector3 translation) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  
  Magnum::Matrix4 transform = Magnum::Matrix4::translation(translation);

  LOG(INFO) << "Shifting data origin: " << translation[0] << ", " << translation[1] << ", " << translation[2];
  Magnum::MeshTools::transformPointsInPlace(transform, meshData.positions);
  LOG(INFO) << "Shifting data origin done";
}

Magnum::GL::AbstractShaderProgram* ResourceManager::getShaderProgram(
    ShaderType type) {
  if (shaderPrograms_.count(type) == 0) {
    switch (type) {
      case INSTANCE_MESH_SHADER: {
        shaderPrograms_[INSTANCE_MESH_SHADER] =
            std::make_shared<gfx::GenericShader>(
                gfx::GenericShader::Flag::VertexColored |
                gfx::GenericShader::Flag::PrimitiveIDTextured);
      } break;

      case PTEX_MESH_SHADER: {
        shaderPrograms_[PTEX_MESH_SHADER] =
            std::make_shared<gfx::PTexMeshShader>();
      } break;

      case COLORED_SHADER: {
        shaderPrograms_[COLORED_SHADER] =
            std::make_shared<gfx::GenericShader>();
      } break;

      case VERTEX_COLORED_SHADER: {
        shaderPrograms_[VERTEX_COLORED_SHADER] =
            std::make_shared<gfx::GenericShader>(
                gfx::GenericShader::Flag::VertexColored);
      } break;

      case TEXTURED_SHADER: {
        shaderPrograms_[TEXTURED_SHADER] = std::make_shared<gfx::GenericShader>(
            gfx::GenericShader::Flag::Textured);
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
  // if this is a new file, load it and add it to the dictionary
  const std::string& filename = info.filepath;
  if (resourceDict_.count(filename) == 0) {
    const std::string atlasDir =
        Corrade::Utility::String::stripSuffix(filename, "ptex_quad_mesh.ply") +
        "ptex_textures";

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
        new gfx::PTexMeshDrawable{node, *ptexShader, *pTexMeshData, jSubmesh,
                                  drawables};
      }
    }
  }

  return true;
}

// semantic instance mesh import
bool ResourceManager::loadInstanceMeshData(const AssetInfo& info,
                                           scene::SceneNode* parent,
                                           DrawableGroup* drawables) {
  // if this is a new file, load it and add it to the dictionary, create shaders
  // and add it to the shaderPrograms_
  const std::string& filename = info.filepath;
  if (resourceDict_.count(filename) == 0) {
    if (info.type == AssetType::FRL_INSTANCE_MESH) {
      meshes_.emplace_back(std::make_unique<FRLInstanceMeshData>());
    } else if (info.type == AssetType::INSTANCE_MESH) {
      meshes_.emplace_back(std::make_unique<GenericInstanceMeshData>());
    }
    int index = meshes_.size() - 1;
    auto* instanceMeshData =
        dynamic_cast<GenericInstanceMeshData*>(meshes_[index].get());

    LOG(INFO) << "loading instance mesh data: " << filename;
    instanceMeshData->loadPLY(filename);
    instanceMeshData->uploadBuffersToGPU(false);

    instance_mesh = &(instanceMeshData->getRenderingBuffer()->mesh);
    // update the dictionary
    resourceDict_.emplace(filename, MeshMetaData(index, index));
  }

  // create the scene graph by request
  if (parent) {
    LOG(INFO) << "Building SceneGraph";
    auto indexPair = resourceDict_.at(filename).meshIndex;
    int start = indexPair.first;
    int end = indexPair.second;

    for (int iMesh = start; iMesh <= end; ++iMesh) {
      auto* instanceMeshData =
          dynamic_cast<GenericInstanceMeshData*>(meshes_[iMesh].get());
      scene::SceneNode& node = parent->createChild();
      createDrawable(INSTANCE_MESH_SHADER, *instanceMeshData->getMagnumGLMesh(),
                     node, drawables, instanceMeshData->getSemanticTexture());
    }
  }

  return true;
}

bool ResourceManager::loadGeneralMeshData(
    const AssetInfo& info,
    scene::SceneNode* parent  /* = nullptr */,
    DrawableGroup* drawables  /* = nullptr */,
    bool shiftOrigin          /* = false */,
    Magnum::Vector3 translation /* [0,0,0] */) 
{
  const std::string& filename = info.filepath;
  const bool fileIsLoaded = resourceDict_.count(filename) > 0;
  const bool drawData = parent != nullptr && drawables != nullptr;

  // Mesh & metaData container
  MeshMetaData metaData;
  std::vector<Magnum::UnsignedInt> magnumData;

  LOG(INFO) << "LoadingGeneralMeshData: " << filename;
  LOG(INFO) << "  ...already loaded? " << fileIsLoaded;
  
  // Optional File loading
  if (!fileIsLoaded) {

    if(importer->isOpened())
      LOG(ERROR) << " importer preload mesh3DCount = " << importer->mesh3DCount();
    else
    {
        LOG(ERROR) << " importer no file loaded... ";
    }
    

    // if file is not loaded
    if (!importer) {
      LOG(ERROR) << "Cannot load the importer. ";
      return false;
    }
    if (!importer->openFile(filename)) {
      LOG(ERROR) << "Cannot open file " << filename;
      return false;
    }

    LOG(ERROR) << " importer postload mesh3DCount = " << importer->mesh3DCount();

    // if this is a new file, load it and add it to the dictionary
    loadTextures(*importer, &metaData);
    LOG(INFO) << "Loaded total textures " << textures_.size();
    loadMaterials(*importer, &metaData);
    LOG(INFO) << "Loaded total materials " << materials_.size();
    
    // TODO (JH): shift origin merge with COM
    loadMeshes(*importer, &metaData, shiftOrigin, translation);
    LOG(INFO) << "Loaded total meshes " << meshes_.size();

    resourceDict_.emplace(filename, metaData);
    LOG(INFO) << "Added mesh metaData: meshes=" << metaData.meshIndex <<" materials=" << metaData.materialIndex << " textures=" << metaData.textureIndex;

    // Register magnum mesh
    LOG(INFO) << "Registering " << filename;
    if (importer->defaultScene() != -1) {
      Corrade::Containers::Optional<Magnum::Trade::SceneData> sceneData =
          importer->scene(importer->defaultScene());
      if (!sceneData) {
        LOG(ERROR) << "Cannot load scene, exiting";
        return false;
      }
      for (uint sceneDataID: sceneData->children3D()) {
        LOG(INFO) << "Loading child scene data ID " << sceneDataID;
        magnumData.emplace_back(sceneDataID);
      }
    } else if (importer->mesh3DCount() && meshes_[metaData.meshIndex.first]) {
      // no default scene --- standalone OBJ/PLY files, for example
      // take a wild guess and load the first mesh with the first material
      //addMeshToDrawables(metaData, *parent, drawables, ID_UNDEFINED, 0, 0);
      // TODO (JH): check this part, may not be working
      LOG(INFO) << "Load non-default";
      magnumData.emplace_back(0);
      LOG(INFO) << "Load 0";
    } else {
      LOG(ERROR) << "No default scene available and no meshes found, exiting";
      return false;
    }
    magnumMeshDict_.emplace(filename, magnumData);
    LOG(INFO) << "Load mesh/material/texture done";
  } else {
    LOG(INFO) << "  Loading metaData from resourceDict_";
    metaData = resourceDict_[filename];
  }

  // Optional Instantiation
  if (!drawData) {
    //! Do not instantiate object
    return true;
  } else {

    //Alex: intercept nullptr scene graph nodes (default) to add mesh to metadata list without adding it to scene graph
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

    const quatf transform = info.frame.rotationFrameToWorld();
    newNode.setRotation(Magnum::Math::Quaternion<float>(transform));
    // Recursively add all children
    for (auto sceneDataID : magnumMeshDict_[filename]) {
      LOG(INFO) << "Scene data ID " << sceneDataID << " " << filename; 
      addComponent(*importer, info, metaData, newNode, drawables, sceneDataID);
      //addComponent(*importer, info, metaData, *parent, drawables, sceneDataID);
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
    // LOG(INFO) << "Importing material" << i << importer->materialName(i);
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
    LOG(INFO) << "Loading material " << iMaterial << "/" << importer.materialCount() << " done";
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
    
    // Keep track of names
    object_names_.push_back(importer.mesh3DName(iMesh));

    //see if the mesh needs to be shifted
    if (shiftOrigin){
      //compute BB center if necessary ([0,0,0])
      if(offset[0]==0 && offset[1]==0 && offset[2]==0)
        offset = -computeMeshBBCenter(gltfMeshData);
      //translate the mesh if necessary
      if(!(offset[0]==0 && offset[1]==0 && offset[2]==0))
        translateMesh(gltfMeshData, offset);
    }

    CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
    gltfMeshData->uploadBuffersToGPU(false);
    LOG(INFO) << "Loading mesh " << iMesh << "/" << importer.mesh3DCount() << " " << importer.mesh3DName(iMesh) << " done";
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
    // LOG(INFO) << "Importing image" << textureData->image()
    //          << importer.image2DName(textureData->image());
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
    LOG(INFO) << "Loading texture " << iTexture << "/" << importer.textureCount() << " done";
  }
}

//! Add component to rendering stack, based on importer loading
//! TODO (JH): decouple importer part, so that objects can be
//! instantiated any time after initial loading
void ResourceManager::addComponent(Importer& importer,
                                   const AssetInfo& info,
                                   const MeshMetaData& metaData,
                                   scene::SceneNode& parent,
                                   DrawableGroup* drawables,
                                   int componentID) {
  //importer.openFile(info.filename);
  //TODO: check what is in the importer...
  LOG(INFO) << "   ...importer object name" << importer.object3DName(componentID);
  importer.openFile(info.filepath);
  LOG(INFO) << "   ...importer object name" << importer.object3DName(componentID);
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
  if (objectData->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh 
      && meshIDLocal != ID_UNDEFINED 
      && meshes_[meshID]) {
    const int materialIDLocal =
        static_cast<Magnum::Trade::MeshObjectData3D*>(objectData.get())
            ->material();
    LOG(INFO) << "Adding drawables materialIDLocal " << materialIDLocal;
    addMeshToDrawables(metaData, node, drawables, componentID, meshIDLocal,
        materialIDLocal);
  }

  // Recursively add children
  for (auto childObjectID : objectData->children()) {
    LOG(INFO) << "Child ID " << childObjectID << " (total " << objectData->children().size() << ")";
    addComponent(importer, info, metaData, node, drawables, childObjectID);
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
  LOG(INFO) << "Adding mesh to drawable meshIDLocal " << meshIDLocal << " meshID " << meshID << " materialID " << materialID << " componentID " << componentID;

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
        LOG(INFO) << "Create textured drawable";
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
    Magnum::SceneGraph::DrawableGroup3D* group  /* = nullptr */,
    Magnum::GL::Texture2D* texture              /* = nullptr */,
    int objectId                                /* = ID_UNDEFINED */,
    const Magnum::Color4& color                 /* = Magnum::Color4{1} */) {
  gfx::Drawable* drawable = nullptr;
  if (shaderType == PTEX_MESH_SHADER) {
    LOG(ERROR)
        << "ResourceManager::createDrawable does not support PTEX_MESH_SHADER";
    ASSERT(shaderType != PTEX_MESH_SHADER);
    // NOTE: this is a runtime error and will never return
    return *drawable;
  } else {  // all other shaders use GenericShader
    auto* shader =
        static_cast<gfx::GenericShader*>(getShaderProgram(shaderType));
    drawable = new gfx::GenericDrawable{node,    *shader,  mesh, group,
                                        texture, objectId, color};
  }
  return *drawable;
}


bool ResourceManager::loadSUNCGHouseFile(const AssetInfo& houseInfo,
                                         scene::SceneNode* parent,
                                         DrawableGroup* drawables) {
  ASSERT(parent != nullptr);
  const std::string& houseFile = houseInfo.filepath;
  const auto& json = io::parseJsonFile(houseFile);
  const auto& levels = json["levels"].GetArray();
  std::vector<std::string> pathTokens = io::tokenize(houseFile, "/", 0, true);
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
        // LOG(INFO) << modelId << " " << transform;
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
