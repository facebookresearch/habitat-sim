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
#include "MeshData.h"
#include "ResourceManager.h"

namespace esp {
namespace assets {

bool ResourceManager::loadScene(const AssetInfo& info,
                                scene::SceneNode* parent /* = nullptr */,
                                DrawableGroup* drawables /* = nullptr */) {
  // Deprecated
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
    return loadGeneralMeshData(info, parent, drawables);
  } else {
    // Unknown type, just load general mesh data
    LOG(INFO) << "Loading General mesh data";
    return loadGeneralMeshData(info, parent, drawables);
  }
}

bool ResourceManager::loadPhysicalScene(
    const AssetInfo& info,
    PhysicsManager& _physicsManager,
    scene::SceneNode* parent /* = nullptr */,
    bool attach_physics, /* = false */
    DrawableGroup* drawables /* = nullptr */) {
  bool meshSuccess = loadScene(info, parent, drawables);
  LOG(INFO) << "Loaded mesh object, success " << meshSuccess;

  if (attach_physics) {
    physics::BulletRigidObject* physNode = new physics::BulletRigidObject(parent);

    LOG(INFO) << "Physics node " << physNode;
    if (physNode == nullptr) {
      LOG(ERROR) << "Cannot instantiate physics node.";
      return false;
    }

    const std::string& filename = info.filepath;
    MeshMetaData& metaData = resourceDict_.at(filename);
    auto indexPair = metaData.meshIndex;
    int start = indexPair.first;
    int end = indexPair.second;
    bool objectSuccess;

    std::vector<CollisionMeshData> meshGroup;

    LOG(INFO) << "Accessing scene mesh start " << start << " end " << end;
    for (int mesh_i = start; mesh_i < end; mesh_i++) {

      // FRL Quad Mesh
      if (info.type == AssetType::FRL_INSTANCE_MESH) {
        FRLInstanceMeshData* frlMeshData = 
            dynamic_cast<FRLInstanceMeshData*>(meshes_[mesh_i].get());
        CollisionMeshData& meshData = frlMeshData->getCollisionMeshData();
        meshGroup.push_back(meshData);
      } 

      // PLY Instance mesh
      else if (info.type == AssetType::INSTANCE_MESH) {
        GenericInstanceMeshData* insMeshData = 
            dynamic_cast<GenericInstanceMeshData*>(meshes_[mesh_i].get());
        quatf quatf = quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
        Magnum::Quaternion quat = Magnum::Quaternion(quatf);
        Magnum::Matrix4 transform =
            Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());
        CollisionMeshData& meshData = insMeshData->getCollisionMeshData();
        meshGroup.push_back(meshData);
        
        /*Magnum::Trade::MeshData3D meshData(
            reinterpret_cast<std::vector>GMesh->getRenderingBuffer()->mesh)   
        Magnum::GL::Mesh* mesh  = &meshData->getRenderingBuffer()->mesh;
        Magnum::GL::Buffer* vbo = &meshData->getRenderingBuffer()->vbo;
        Magnum::GL::Buffer* cbo = &meshData->getRenderingBuffer()->cbo;
        Magnum::GL::Buffer* ibo = &meshData->getRenderingBuffer()->ibo;*/
        //Corrade::Containers::Array<char> v_data = vbo->data();
        //const std::vector<vec3f> v_data = meshData->get_vbo();
        //const std::vector<int> i_data   = meshData->get_ibo();
        // TODO (JH): need this in order to get Replica mesh working
        //Magnum::MeshTools::transformPointsInPlace(transform,
        //                                          meshData->positions(0));
      }

      // GLB Mesh
      else if (info.type == AssetType::MP3D_MESH) {
      
        //std::vector<std::shared_ptr<BaseMesh>> sceneMesh(&meshes_[start], &meshes[end]);
        // Apply in-place transformation to collision mesh, to be consistent with
        //  display mesh
        //Magnum::Trade::MeshData3D* meshData;
        quatf quatf = quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
        Magnum::Quaternion quat = Magnum::Quaternion(quatf);
        GltfMeshData* gltfMeshData = dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
        CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
        Magnum::Matrix4 transform =
            Magnum::Matrix4::rotation(quat.angle(), quat.axis().normalized());
        Magnum::MeshTools::transformPointsInPlace(transform,
                                                  meshData.positions);
        meshGroup.push_back(meshData);
        /*LOG(INFO) << "Initializing scene";
        LOG(INFO) << "Scene built in angle " << float(quat.angle());
        LOG(INFO) << "Scene built in axis " << Eigen::Map<vec3f>(quat.axis().data());*/
      }

    }

    bool sceneSuccess = _physicsManager.initScene(
        info, metaData, meshGroup, physNode);

    LOG(INFO) << "Initialized mesh scene, success " << sceneSuccess;
    if (!sceneSuccess) {
      LOG(INFO) << "Physics manager failed to initialize object";
      return false;
    }

    else {
      return false;
    }
  }

  return meshSuccess;
}

bool ResourceManager::loadObject(const AssetInfo& info,
                                 PhysicsManager& _physicsManager,
                                 scene::SceneNode* parent,
                                 bool attach_physics, /* = false */
                                 DrawableGroup* drawables /* = nullptr */,
                                 physics::BulletRigidObject** physNode) {
  // if this is a new file, load it and add it to the dictionary
  if (attach_physics) {

    if (info.type == AssetType::MP3D_MESH) {
      *physNode = new physics::BulletRigidObject(parent);
      bool meshSuccess =
          loadGeneralMeshData(info, parent, drawables, true, *physNode);
      bool objectSuccess;
      // Load convex hull
      std::string fname;
      fname.assign(info.filepath);
      std::string c_fname =
          //fname.replace(fname.end() - 4, fname.end(), "VHACD.glb");
          fname.replace(fname.end() - 4, fname.end(), "_convex.glb");
      AssetInfo c_info = AssetInfo::fromPath(c_fname);

      // Load collision mesh, need to shift to origin
      bool shift_Origin = true;
      bool c_MeshSuccess = loadGeneralMeshData(c_info, parent, nullptr, shift_Origin);

      // const std::string& filename = info.filepath;
      MeshMetaData& metaData = resourceDict_.at(c_fname);
      auto indexPair = metaData.meshIndex;
      int start = indexPair.first;
      int end = indexPair.second;
      LOG(INFO) << "Accessing object mesh start " << start << " end " << end;

      //std::vector<std::shared_ptr<BaseMesh>> objectMesh(&meshes_[start], &meshes[end]);
      std::vector<CollisionMeshData> meshGroup;
      for (int mesh_i = start; mesh_i < end; mesh_i++) {
        GltfMeshData* gltfMeshData = dynamic_cast<GltfMeshData*>(meshes_[mesh_i].get());
        CollisionMeshData& meshData = gltfMeshData->getCollisionMeshData();
        meshGroup.push_back(meshData);
      }
      
      transformAxis(info, meshGroup);
      objectSuccess = _physicsManager.initObject(info, metaData, meshGroup, *physNode);

      (*physNode)->MagnumObject::setTransformation(parent->transformation());
      (*physNode)->syncPose();
      //LOG(INFO) << "Parent trans " << Eigen::Map<mat4f>(parent->transformation().data());
      // PhysicsManager::initObject(*importer, info, mMetaData);
      // for (int index = 0; index < meshes_.size(); index++) {
      //
      return c_MeshSuccess && objectSuccess;

    } else {
      // Objects in other formats not supported yet
      return false;
    }
  } else {
    bool meshSuccess = loadGeneralMeshData(info, parent, drawables, true);
    return meshSuccess;
  }
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

bool ResourceManager::loadGeneralMeshData(const AssetInfo& info,
                                          scene::SceneNode* parent,
                                          DrawableGroup* drawables,
                                          bool shiftOrigin /* = false */,
                                          scene::SceneNode* node) {
  const std::string& filename = info.filepath;
  const bool fileIsLoaded = resourceDict_.count(filename) > 0;

  // if file is loaded, and no need to build the scene graph
  if (fileIsLoaded && parent == nullptr) {
    return true;
  }

  // Load a scene importer plugin. In case Magnum is built statically, arg is
  // pluginDirectory to silence warnings, otherwise we *do* want it to search
  // in the filesystem.
  Magnum::PluginManager::Manager<Importer> manager{
#ifdef MAGNUM_BUILD_STATIC
      "./"
#endif
  };
  std::unique_ptr<Importer> importer =
      manager.loadAndInstantiate("AnySceneImporter");

  // Prefer tiny_gltf for loading glTF files (Assimp is worse),
  // prefer Assimp for OBJ files (ObjImporter is worse)
  manager.setPreferredPlugins("GltfImporter", {"TinyGltfImporter"});
  manager.setPreferredPlugins("ObjImporter", {"AssimpImporter"});

  if (!importer) {
    LOG(ERROR) << "Cannot load the importer. ";
    return false;
  }

  if (!importer->openFile(filename)) {
    LOG(ERROR) << "Cannot open file " << filename;
    return false;
  }

  // if this is a new file, load it and add it to the dictionary
  if (!fileIsLoaded) {
    MeshMetaData metaData;
    loadTextures(*importer, &metaData);
    loadMaterials(*importer, &metaData);
    loadMeshes(*importer, &metaData, shiftOrigin);
    // update the dictionary
    resourceDict_.emplace(filename, metaData);
  }

  auto& metaData = resourceDict_.at(filename);
  const bool forceReload = false;

  scene::SceneNode* newNode;
  if (node) {
    newNode = node;
  } else {
    newNode = &parent->createChild();
  }
  bool success_ = true;
  if (drawables != nullptr) {
    success_ = createScene(*importer, info, metaData, *newNode, drawables,
                           forceReload);
  }

  return success_;
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

    // TODO (JH) apparently only gltfMesh allows accesing non-GL mesh data,
    // which can be attached with physics, others (PTex, FRLMesh, etc) do not
    // have this implemented
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

void ResourceManager::createObject(Importer& importer,
                                   const AssetInfo& info,
                                   const MeshMetaData& metaData,
                                   scene::SceneNode& parent,
                                   DrawableGroup* drawables,
                                   int objectID) {
  std::unique_ptr<Magnum::Trade::ObjectData3D> objectData =
      importer.object3D(objectID);
  if (!objectData) {
    LOG(ERROR) << "Cannot import object " << importer.object3DName(objectID)
               << ", skipping";
    return;
  }

  // Add the object to the scene and set its transformation
  scene::SceneNode& node = parent.createChild();
  node.MagnumObject::setTransformation(objectData->transformation());
  // LOG(INFO) << "Transformation " <<
  // Eigen::Map<mat4f>(objectData->transformation().data());

  const int meshIDLocal = objectData->instance();
  const int meshID = metaData.meshIndex.first + meshIDLocal;

  // Add a drawable if the object has a mesh and the mesh is loaded
  if (objectData->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh &&
      meshIDLocal != ID_UNDEFINED && meshes_[meshID]) {
    const int materialIDLocal =
        static_cast<Magnum::Trade::MeshObjectData3D*>(objectData.get())
            ->material();
    createMeshObject(metaData, node, drawables, objectID, meshIDLocal,
                     materialIDLocal);
  }

  // Recursively add children
  for (auto childObjectID : objectData->children()) {
    createObject(importer, info, metaData, node, drawables, childObjectID);
  }
}

void ResourceManager::createMeshObject(const MeshMetaData& metaData,
                                       scene::SceneNode& node,
                                       DrawableGroup* drawables,
                                       int objectID,
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
      // Color-only material
      createDrawable(COLORED_SHADER, mesh, node, drawables, texture, objectID,
                     materials_[materialID]->diffuseColor());
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

bool ResourceManager::createScene(Importer& importer,
                                  const AssetInfo& info,
                                  const MeshMetaData& metaData,
                                  scene::SceneNode& sceneNode,
                                  DrawableGroup* drawables,
                                  bool forceReload /* = false */) {
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

  if (importer.defaultScene() != -1) {
    Corrade::Containers::Optional<Magnum::Trade::SceneData> sceneData =
        importer.scene(importer.defaultScene());
    if (!sceneData) {
      LOG(ERROR) << "Cannot load scene, exiting";
      return false;
    }

    //const quatf transform =
    //    quatf::FromTwoVectors(info.frame.front(), geo::ESP_FRONT);
    const quatf transform = info.frame.rotationFrameToWorld();
    
    // create scene parent node with transformation aligning to global frame
    sceneNode.setRotation(transform);

    // Recursively add all children
    for (auto objectID : sceneData->children3D()) {
      createObject(importer, info, metaData, sceneNode, drawables, objectID);
    }
  } else if (importer.mesh3DCount() && meshes_[metaData.meshIndex.first]) {
    // no default scene --- standalone OBJ/PLY files, for example

    // create scene parent node with transformation aligning to global frame
    const quatf transform = info.frame.rotationFrameToWorld();
    sceneNode.setRotation(transform);

    // take a wild guess and load the first mesh with the first material
    createMeshObject(metaData, sceneNode, drawables, ID_UNDEFINED, 0, 0);

  } else {
    LOG(ERROR) << "No default scene available and no meshes found, exiting";
    return false;
  }

  return true;
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
