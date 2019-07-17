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

namespace esp {
namespace assets {

bool ResourceManager::loadSceneData(
    const AssetInfo& info,
    scene::SceneNode* parent /* = nullptr */,
    DrawableGroup* drawables /* = nullptr */) {
  // check if the file exists
  if (!io::exists(info.filepath)) {
    LOG(ERROR) << "Cannot load from file " << info.filepath;
    return false;
  }

  scene::SceneNode* sceneNode = nullptr;
  if (info.type == AssetType::FRL_INSTANCE_MESH ||
      info.type == AssetType::INSTANCE_MESH) {
    LOG(INFO) << "Loading FRL/Instance mesh data";
    return loadInstanceMeshData(info, parent, drawables);
  } else if (info.type == AssetType::FRL_PTEX_MESH) {
    LOG(INFO) << "Loading PTEX mesh data";
    return loadPTexMeshData(info, parent, drawables);
  } else if (info.type == AssetType::SUNCG_SCENE) {
    return loadSUNCGHouseFile(info, parent, drawables);
  } else if (info.type == AssetType::MP3D_MESH) {
    LOG(INFO) << "Loading MP3D mesh data";
    bool meshSuccess = loadGeneralMeshData(info, parent, drawables);
    return meshSuccess;
  } else {
    // Unknown type, just load general mesh data
    LOG(INFO) << "Loading General mesh data";
    bool meshSuccess = loadGeneralMeshData(info, parent, drawables);
    return meshSuccess;
  }
}

//! Both load and instantiate scene
bool ResourceManager::loadScene(
    const AssetInfo& info,
    scene::SceneNode* parent,                 /* = nullptr */
    DrawableGroup* drawables,                 /* = nullptr */
    physics::PhysicsManager* _physicsManager, /* = nullptr */
    bool attach_physics                       /* = false */) {
  bool meshSuccess = loadSceneData(info, parent, drawables);
  LOG(INFO) << "Loaded mesh object, success " << meshSuccess;

  if (attach_physics) {
    const std::string& filename = info.filepath;
    MeshMetaData& metaData = resourceDict_.at(filename);
    auto indexPair = metaData.meshIndex;
    int start = indexPair.first;
    int end = indexPair.second;
    bool objectSuccess;

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
        /*Magnum::Matrix4 transform =
            Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());*/
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
    bool sceneSuccess = _physicsManager->addScene(info, parent, meshGroup);

    LOG(INFO) << "Initialized mesh scene, success " << sceneSuccess;
    if (!sceneSuccess) {
      LOG(INFO) << "Physics manager failed to initialize object";
      return false;
    }
  }
  return meshSuccess;
}

// Add object by object key
int ResourceManager::addObject(
    const std::string objectKey,
    scene::SceneNode* parent,
    DrawableGroup* drawables) 
{
  int objectID = getObjectID(objectKey);
  if (objectID < 0) {
    return -1;
  }
  LOG(INFO) << "Resource add object " << objectKey << " " << objectID;
  return addObject(objectID, parent, drawables);
}

// Add object by ID
int ResourceManager::addObject(
    const int objectID,
    scene::SceneNode* parent,
    DrawableGroup* drawables) 
{
  std::string object_config = getObjectConfig(objectID);
  LOG(INFO) << "Resource add object " << object_config << " " << objectID;
  return loadObject(object_config, parent, drawables);
}

//! Only load and does not instantiate object
//! For load-only: set parent = nullptr, drawables = nullptr
int ResourceManager::loadObject(
    const std::string object_config,
    scene::SceneNode* parent       /* nullptr */,
    DrawableGroup* drawables       /* nullptr */) 
{
  std::string object_file("./data/objects/cheezit.glb");
  std::string key_name("cheezit");
  assets::AssetInfo info = assets::AssetInfo::fromPath(object_file);
  physicsObjectLibrary_[key_name] = PhysicsObjectMetaData();

  if (info.type == AssetType::MP3D_MESH) {
    //! nullptr for both parent and drawables to disable immediate
    //! instantiation
    bool loadSuccess = loadGeneralMeshData(info, parent, drawables, true);
    //! Load convex hull
    std::string fname;
    fname.assign(info.filepath);
    std::string c_fname =
        fname.replace(fname.end() - 4, fname.end(), "_convex.glb");
    AssetInfo c_info = AssetInfo::fromPath(c_fname);

    //! Load collision mesh, need to shift to origin
    bool shift_Origin = true;
    bool c_loadSuccess = loadGeneralMeshData(c_info, nullptr, 
      nullptr, shift_Origin);
    // const std::string& filename = info.filepath;
    if (!c_loadSuccess) {
      return -1;
    }
    //! Register mesh data to dictionary
    PhysicsObjectMetaData& physMetaData = physicsObjectLibrary_[key_name];
    std::vector<std::string>::iterator itr = std::find(
        physicsObjectKeyList_.begin(), physicsObjectKeyList_.end(), key_name);
    if (itr == physicsObjectKeyList_.cend()) { 

      MeshMetaData& metaData = resourceDict_.at(c_fname);
      auto indexPair = metaData.meshIndex;
      int start = indexPair.first;
      int end = indexPair.second;
      LOG(INFO) << "Accessing object mesh start " << start << " end " << end;

      std::vector<CollisionMeshData> meshGroup;
      for (int mesh_i = start; mesh_i <= end; mesh_i++) {
        GltfMeshData* gltfMeshData = dynamic_cast<GltfMeshData*>(
            meshes_[mesh_i].get());
        CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
        meshGroup.push_back(meshData);
      }
      
      //! Properly align axis direction
      transformAxis(info, meshGroup);

      collisionMeshGroups_[key_name] = meshGroup;
      physicsObjectKeyList_.push_back(key_name);
      physicsObjectConfigList_.push_back(object_config);
      return physicsObjectKeyList_.size() - 1;
    } else {
      return std::distance(physicsObjectKeyList_.begin(), itr);
    }

    // TODO (JH) move this logic to somewhere else
    //int objectID = _physicsManager->addObject(info, metaData, meshGroup);
    //LOG(INFO) << "Object ID loaded " << objectID;
    //objectSuccess = (objectID != -1);
    //(*physNode)->setTransformation(parent->getTransformation());

  } else {
    //! Objects in other formats not supported yet
    //! For instance
    //! (1) assets::AssetType::INSTANCE_MESH: _semantic.ply
    //! (2) assets::AssetType::FRL_INSTANCE_MESH:  FRL mesh
    LOG(INFO) << "Cannot load non-GLB objects";
    return -1;
  }
}

PhysicsObjectMetaData& ResourceManager::getPhysicsMetaData(
    const std::string objectName) 
{
  return physicsObjectLibrary_[objectName];
}

void ResourceManager::transformAxis(
    const AssetInfo& info,
    std::vector<CollisionMeshData> meshGroup) {

  quatf quatf = quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
  Magnum::Quaternion quat = Magnum::Quaternion(quatf);
  for (CollisionMeshData& meshData: meshGroup) {
    Magnum::Matrix4 transform =
        Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());
    Magnum::MeshTools::transformPointsInPlace(transform,
                                              meshData.positions);
  }
}

std::vector<assets::CollisionMeshData> ResourceManager::getCollisionMesh(
    const int objectID) {
  if (objectID < 0 || objectID > collisionMeshGroups_.size()) {
    return std::vector<assets::CollisionMeshData>();
  }
  std::string keyName = getObjectKeyName(objectID);
  return collisionMeshGroups_[keyName];
}

std::vector<assets::CollisionMeshData> ResourceManager::getCollisionMesh(
    const std::string keyName) {
  return collisionMeshGroups_[keyName];
}

int ResourceManager::getObjectID(std::string keyName) {
  std::vector<std::string>::iterator itr = std::find(
      physicsObjectKeyList_.begin(), physicsObjectKeyList_.end(), keyName);
  if (itr == physicsObjectKeyList_.cend()) {
    return -1;
  } else {
    int objectID = std::distance(physicsObjectKeyList_.begin(), itr);
    return objectID;
  }
}

std::string ResourceManager::getObjectKeyName(int objectID) {
  return physicsObjectKeyList_[objectID];
}

std::string ResourceManager::getObjectConfig(int objectID) {
  return physicsObjectConfigList_[objectID];
}

void ResourceManager::shiftMeshDataToOrigin(GltfMeshData* meshDataGL) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  float minX = 999999.9f;
  float maxX = -999999.9f;
  float minY = 999999.9f;
  float maxY = -999999.9f;
  float minZ = 999999.9f;
  float maxZ = -999999.9f;
  for (int vi = 0; vi < meshData.positions.size(); vi++) {
    Magnum::Vector3 pos = meshData.positions[vi];
    if (pos.x() < minX) {
      minX = pos.x();
    }
    if (pos.x() > maxX) {
      maxX = pos.x();
    }
    if (pos.y() < minY) {
      minY = pos.y();
    }
    if (pos.y() > maxY) {
      maxY = pos.y();
    }
    if (pos.z() < minZ) {
      minZ = pos.z();
    }
    if (pos.z() > maxZ) {
      maxZ = pos.z();
    }
  }
  LOG(INFO) << "Shifting data origin";
  Magnum::Matrix4 transform = Magnum::Matrix4::translation(Magnum::Vector3(
      -(maxX + minX) / 2, -(maxY + minY) / 2, -(maxZ + minZ) / 2));
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
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    bool shiftOrigin /* = false */) 
{
  const std::string& filename = info.filepath;
  const bool fileIsLoaded = resourceDict_.count(filename) > 0;
  const bool drawData = parent != nullptr && drawables != nullptr;

  // Mesh & metaData container
  MeshMetaData metaData;
  std::vector<Magnum::UnsignedInt> magnumData;

  // Optional File loading
  if (!fileIsLoaded) {
    // if file is not loaded
    if (!importer) {
      LOG(ERROR) << "Cannot load the importer. ";
      return false;
    }
    if (!importer->openFile(filename)) {
      LOG(ERROR) << "Cannot open file " << filename;
      return false;
    }

    // if this is a new file, load it and add it to the dictionary
    loadTextures(*importer, &metaData);
    loadMaterials(*importer, &metaData);
    LOG(INFO) << "Total materials " << materials_.size();
    // TODO (JH): shift origin merge with COM
    loadMeshes(*importer, &metaData, shiftOrigin);
    resourceDict_.emplace(filename, metaData);

    // Register magnum mesh
    LOG(INFO) << "Loading " << filename;
    if (importer->defaultScene() != -1) {
      Corrade::Containers::Optional<Magnum::Trade::SceneData> sceneData =
          importer->scene(importer->defaultScene());
      if (!sceneData) {
        LOG(ERROR) << "Cannot load scene, exiting";
        return false;
      }
      LOG(INFO) << "Load default " << importer->mesh3DCount();
      for (uint sceneDataID: sceneData->children3D()) {
        LOG(INFO) << "Loading child scene dataID " << sceneDataID;
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
    metaData = resourceDict_[filename];
  }

  // Optional Instantiation
  if (!drawData) {
    //! Do not instantiate object
    return true;
  } else {
    //! Do instantiate object
    MeshMetaData& metaData = resourceDict_[filename];
    const bool forceReload = false;
    //scene::SceneNode newNode = parent->createChild();
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

    // create scene parent node with transformation aligning to global frame
    const quatf transform = info.frame.rotationFrameToWorld();
    parent->setRotation(transform);
    // Recursively add all children
    for (auto sceneDataID : magnumMeshDict_[filename]) {
      LOG(INFO) << "Scene data ID " << sceneDataID << " " << filename; 
      addComponent(*importer, info, metaData, *parent, drawables, sceneDataID);
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
  }
}

void ResourceManager::loadMeshes(Importer& importer,
                                 MeshMetaData* metaData,
                                 bool shiftOrigin /*=false*/) {
  int meshStart = meshes_.size();
  int meshEnd = meshStart + importer.mesh3DCount() - 1;
  metaData->setMeshIndices(meshStart, meshEnd);

  for (int iMesh = 0; iMesh < importer.mesh3DCount(); ++iMesh) {
    LOG(INFO) << "Importing mesh " << iMesh << ": "
              << importer.mesh3DName(iMesh);

    meshes_.emplace_back(std::make_unique<GltfMeshData>());
    auto& currentMesh = meshes_.back();
    auto* gltfMeshData = static_cast<GltfMeshData*>(currentMesh.get());
    gltfMeshData->setMeshData(importer, iMesh);
    
    // Keep track of names
    object_names_.push_back(importer.mesh3DName(iMesh));

    if (shiftOrigin) {
      // Need this function in order to set center of mass
      shiftMeshDataToOrigin(gltfMeshData);
    }
    CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();

    LOG(INFO) << "Upload buffer to gpu";
    gltfMeshData->uploadBuffersToGPU(false);
    LOG(INFO) << "Upload buffer to gpu done";
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
                                   int sceneDataID) {
  std::unique_ptr<Magnum::Trade::ObjectData3D> objectData =
      importer.object3D(sceneDataID);
  if (!objectData) {
    LOG(ERROR) << "Cannot import object " << importer.object3DName(sceneDataID)
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
    LOG(INFO) << "Adding drawables " << materialIDLocal;
    addMeshToDrawables(metaData, node, drawables, sceneDataID, meshIDLocal,
        materialIDLocal);
  }

  // Recursively add children
  for (auto childObjectID : objectData->children()) {
    LOG(INFO) << "Child ID " << childObjectID;
    addComponent(importer, info, metaData, node, drawables, childObjectID);
  }
}

void ResourceManager::addMeshToDrawables(const MeshMetaData& metaData,
                                         scene::SceneNode& node,
                                         DrawableGroup* drawables,
                                         int sceneDataID,
                                         int meshIDLocal,
                                         int materialIDLocal) {
  const int meshStart = metaData.meshIndex.first;
  const int meshID = meshStart + meshIDLocal;
  Magnum::GL::Mesh& mesh = *meshes_[meshID]->getMagnumGLMesh();

  const int materialStart = metaData.materialIndex.first;
  const int materialID = materialStart + materialIDLocal;
  LOG(INFO) << "Adding mesh to drawable meshIDLocal " << meshIDLocal << " material id local " << materialIDLocal << " materialID " << materialID;

  Magnum::GL::Texture2D* texture = nullptr;
  // Material not set / not available / not loaded, use a default material
  if (materialIDLocal == ID_UNDEFINED ||
      metaData.materialIndex.second == ID_UNDEFINED ||
      !materials_[materialID]) {
    createDrawable(COLORED_SHADER, mesh, node, drawables, texture, sceneDataID);
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
                       sceneDataID);
      } else {
        // Color-only material
        createDrawable(COLORED_SHADER, mesh, node, drawables, texture, 
            sceneDataID, materials_[materialID]->diffuseColor());
      }
    } else {
      // Color-only material
      createDrawable(COLORED_SHADER, mesh, node, drawables, texture, 
          sceneDataID, materials_[materialID]->diffuseColor());
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
        Eigen::Map<mat4f> transform(transformVec.data());
        // LOG(INFO) << modelId << " " << transform;
        const AssetInfo info{
            AssetType::SUNCG_OBJECT,
            basePath + "/object/" + modelId + "/" + modelId + ".glb"};
        createObjectFunc(info, nodeId).setTransformation(transform);
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
