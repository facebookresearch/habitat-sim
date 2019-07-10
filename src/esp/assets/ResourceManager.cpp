// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
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
#include "GltfMeshData.h"
#include "Mp3dInstanceMeshData.h"
#include "PTexMeshData.h"
#include "ResourceManager.h"

namespace esp {
namespace assets {

bool ResourceManager::loadScene(const AssetInfo& info,
                                scene::SceneNode* parent /* = nullptr */,
                                DrawableGroup* drawables /* = nullptr */) {
  // check if the file exists
  if (!io::exists(info.filepath)) {
    LOG(ERROR) << "Cannot load from file " << info.filepath;
    return false;
  }

  if (info.type == AssetType::FRL_INSTANCE_MESH ||
      info.type == AssetType::INSTANCE_MESH) {
    return loadInstanceMeshData(info, parent, drawables);
  } else if (info.type == AssetType::FRL_PTEX_MESH) {
    return loadPTexMeshData(info, parent, drawables);
  } else if (info.type == AssetType::SUNCG_SCENE) {
    return loadSUNCGHouseFile(info, parent, drawables);
  } else if (info.type == AssetType::MP3D_MESH) {
    return loadGeneralMeshData(info, parent, drawables);
  } else {
    // Unknown type, just load general mesh data
    return loadGeneralMeshData(info, parent, drawables);
  }
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
                                          DrawableGroup* drawables) {
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
    loadMeshes(*importer, &metaData);
    // update the dictionary
    resourceDict_.emplace(filename, metaData);
  }

  auto& metaData = resourceDict_.at(filename);
  const bool forceReload = false;
  return createScene(*importer, info, metaData, *parent, drawables,
                     forceReload);
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

void ResourceManager::loadMeshes(Importer& importer, MeshMetaData* metaData) {
  int meshStart = meshes_.size();
  int meshEnd = meshStart + importer.mesh3DCount() - 1;
  metaData->setMeshIndices(meshStart, meshEnd);

  for (int iMesh = 0; iMesh < importer.mesh3DCount(); ++iMesh) {
    // LOG(INFO) << "Importing mesh" << iMesh << importer.mesh3DName(iMesh);
    meshes_.emplace_back(std::make_unique<GltfMeshData>());
    auto& currentMesh = meshes_.back();
    auto* gltfMeshData = static_cast<GltfMeshData*>(currentMesh.get());

    gltfMeshData->setMeshData(importer, iMesh);
    auto& meshData = gltfMeshData->getMeshData();
    if (!meshData ||
        meshData->primitive() != Magnum::MeshPrimitive::Triangles) {
      LOG(ERROR) << "Cannot load the mesh, skipping";
      currentMesh = nullptr;
      continue;
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
                                  scene::SceneNode& parent,
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

    // create scene parent node with transformation aligning to global frame
    auto& sceneNode = parent.createChild();
    const quatf transform = info.frame.rotationFrameToWorld();
    sceneNode.setRotation(Magnum::Quaternion(transform));

    // Recursively add all children
    for (auto objectID : sceneData->children3D()) {
      createObject(importer, info, metaData, sceneNode, drawables, objectID);
    }
  } else if (importer.mesh3DCount() && meshes_[metaData.meshIndex.first]) {
    // no default scene --- standalone OBJ/PLY files, for example

    // create scene parent node with transformation aligning to global frame
    auto& sceneNode = parent.createChild();
    const quatf transform = info.frame.rotationFrameToWorld();
    sceneNode.setRotation(Magnum::Quaternion(transform));

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
