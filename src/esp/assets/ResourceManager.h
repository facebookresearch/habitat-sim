// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

#include "Asset.h"
#include "BaseMesh.h"
#include "CollisionMeshData.h"
#include "GltfMeshData.h"
#include "MeshData.h"
#include "MeshMetaData.h"
#include "PhysicsObjectMetaData.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneNode.h"

// Debug draw
#include <Magnum/DebugTools/ForceRenderer.h>
#include <Magnum/DebugTools/ResourceManager.h>

// forward declarations
namespace Magnum {
namespace Trade {
class AbstractImporter;
class AbstractShaderProgram;
class PhongMaterialData;
}  // namespace Trade
}  // namespace Magnum

namespace esp {
namespace gfx {
class Drawable;
}
namespace scene {
class SceneConfiguration;
}
namespace physics {
class PhysicsManager;
class RigidObject;
}  // namespace physics
namespace assets {

class ResourceManager {
 public:
  // TODO:
  // Singleton
  // a common design pattern for implementing
  // subsystems such as "resource manager", thats make up an engine is
  // to define a singleton class;
  explicit ResourceManager() {
    LOG(INFO) << "Constructing ResourceManager";
    importer = manager.loadAndInstantiate("AnySceneImporter");
    // Prefer tiny_gltf for loading glTF files (Assimp is worse),
    // prefer Assimp for OBJ files (ObjImporter is worse)
    manager.setPreferredPlugins("GltfImporter", {"TinyGltfImporter"});
    manager.setPreferredPlugins("ObjImporter", {"AssimpImporter"});
  };
  ~ResourceManager() { LOG(INFO) << "Deconstructing ResourceManager"; }

  // Stores references to a set of drawable elements
  using DrawableGroup = Magnum::SceneGraph::DrawableGroup3D;
  // Convenience typedef for Importer class
  using Importer = Magnum::Trade::AbstractImporter;

  inline void compressTextures(bool newVal) { compressTextures_ = newVal; };

  //! Load Scene data + instantiate scene
  //! Both load + instantiate scene
  bool loadScene(const AssetInfo& info,
                 scene::SceneNode* parent = nullptr,
                 DrawableGroup* drawables = nullptr);

  //! Load Scene data + instantiate scene
  //! Both load + instantiate scene
  //! Load a physical scene from the sceneMetaData provided
  //! reseats _physicsManager to implementation supplied in sceneMetaData
  bool loadScene(const AssetInfo& info,
                 std::shared_ptr<physics::PhysicsManager>& _physicsManager,
                 PhysicsSceneMetaData sceneMetaData,
                 scene::SceneNode* parent = nullptr,
                 DrawableGroup* drawables = nullptr,
                 bool resetObjectLibrary = false);

  //! Load Scene data + instantiate scene
  //! Both load + instantiate scene
  //! Load a physical scene from a physics config file physicsFilename
  //! reseats _physicsManager to implementation supplied in sceneMetaData
  //! if resetObjectLibrary, clear the physicsObjectMetaData_ struct and re-load
  //! from file.
  //!   otherwise, add any new objects to the library if their filepaths are not
  //!   already registered
  bool loadScene(
      const AssetInfo& info,
      std::shared_ptr<physics::PhysicsManager>& _physicsManager,
      scene::SceneNode* parent = nullptr,
      DrawableGroup* drawables = nullptr,
      std::string physicsFilename = "data/default.phys_scene_config.json",
      bool resetObjectLibrary = false);

  // load a PhysicsSceneMetaData object from a config file
  PhysicsSceneMetaData loadPhysicsConfig(
      std::string physicsFilename = "data/default.phys_scene_config.json");

  //! Load Object data and store internally
  //! Does not instantiate (physics & drawable)
  //! Return index in physicsObjectList_
  int loadObject(const std::string objPhysConfigFilename,
                 scene::SceneNode* parent,
                 DrawableGroup* drawables);

  // load an object into the physicsObjectLibrary_ from a physics properties
  // filename
  int loadObject(const std::string objPhysConfigFilename);

  // load an object into the physicsObjectLibrary_ with default physical
  // parameters from absolute path to mesh files
  int loadDefaultObject(const std::string renderMeshFilename,
                        const std::string collisionMeshFilename = "");

  //======== Accessor functions ========
  std::vector<assets::CollisionMeshData> getCollisionMesh(
      const std::string configFile);

  std::vector<assets::CollisionMeshData> getCollisionMesh(const int objectID);

  int getObjectID(const std::string configFile);
  std::string getObjectConfig(const int objectID);

  PhysicsObjectMetaData& getPhysicsMetaData(const std::string configFile);

  int getNumLibraryObjects() { return physicsObjectConfigList_.size(); };

 protected:
  //======== Scene Functions ========
  //! Instantiate Scene:
  //! (1) create scene node
  //! (2) upload mesh to gpu and drawables
  //! (optional reload of GPU-side assets)
  void addComponent(Importer& importer,
                    const AssetInfo& info,
                    const MeshMetaData& metaData,
                    scene::SceneNode& parent,
                    DrawableGroup* drawables,
                    int objectID);

  // ======== Loading functions for mesh and texture ========
  // Load a scene importer plugin. In case Magnum is built statically, arg is
  // pluginDirectory to silence warnings, otherwise we *do* want it to search
  // in the filesystem.
  // Use importer
  Magnum::PluginManager::Manager<Importer> manager{
#ifdef MAGNUM_BUILD_STATIC
      "./"
#endif
  };
  std::unique_ptr<Importer> importer;

  //! Load textures from importer into assets, and update metaData
  void loadTextures(Importer& importer, MeshMetaData* metaData);

  //! Load meshes from importer into assets, and update metaData
  void loadMeshes(Importer& importer,
                  MeshMetaData* metaData,
                  bool shiftOrigin = false,
                  Magnum::Vector3 offset = Magnum::Vector3(0, 0, 0));

  //! Load materials from importer into assets, and update metaData
  void loadMaterials(Importer& importer, MeshMetaData* metaData);

  bool loadPTexMeshData(const AssetInfo& info,
                        scene::SceneNode* parent,
                        DrawableGroup* drawables);

  bool loadInstanceMeshData(const AssetInfo& info,
                            scene::SceneNode* parent,
                            DrawableGroup* drawables);

  // load the mesh data
  // If parent, also do scene graph
  // if shiftOrigin: translate the mesh by "translation"
  //  (default) if translation == [0,0,0]: compute center of mesh bounding box
  //  and then translate
  bool loadGeneralMeshData(const AssetInfo& info,
                           scene::SceneNode* parent = nullptr,
                           DrawableGroup* drawables = nullptr,
                           bool shiftOrigin = false,
                           Magnum::Vector3 translation = Magnum::Vector3(0,
                                                                         0,
                                                                         0));

  bool loadSUNCGHouseFile(const AssetInfo& info,
                          scene::SceneNode* parent,
                          DrawableGroup* drawables);

  // ======== Geometry helper functions ========
  // void shiftMeshDataToOrigin(GltfMeshData* meshDataGL);

  void translateMesh(GltfMeshData* meshDataGL, Magnum::Vector3 translation);

  // compute center of axis aligned mesh bounding box
  Magnum::Vector3 computeMeshBBCenter(GltfMeshData* meshDataGL);

  void transformAxis(const AssetInfo& info,
                     std::vector<CollisionMeshData> meshGroup);

  // ======== General geometry data ========
  // shared_ptr is used here, instead of Corrade::Containers::Optional, or
  // std::optional because shared_ptr is reference type, not value type, and
  // thus we can avoiding duplicated loading
  std::vector<std::shared_ptr<BaseMesh>> meshes_;
  std::vector<std::shared_ptr<Magnum::GL::Texture2D>> textures_;
  std::vector<std::shared_ptr<Magnum::Trade::PhongMaterialData>> materials_;
  std::vector<std::string> object_names_;

  Magnum::GL::Mesh* instance_mesh;

  // a dictionary to check if a mesh has been loaded
  // maps: absolutePath -> meshMetaData
  std::map<std::string, MeshMetaData> resourceDict_;  // meshes
  std::map<std::string, std::vector<Magnum::UnsignedInt>>
      magnumMeshDict_;  // IDs for object mesh hierarchies NOTE: needed? to
                        // bypass "importer" reload

  // ======== Physical geometry data ========
  // library of physics object parameters mapped from config filename (used by
  // physicsManager to instantiate physical objects) maps:
  // "data/objects/cheezit.phys_properties.json" -> physicalMetaData
  std::map<std::string, PhysicsObjectMetaData> physicsObjectLibrary_;
  // maps: "data/objects/cheezit.phys_properties.json" -> collesionMesh group
  std::map<std::string, std::vector<CollisionMeshData>>
      collisionMeshGroups_;  // meshes for the object hierarchies
  // vector of "data/objects/cheezit.phys_properties.json"
  std::vector<std::string>
      physicsObjectConfigList_;  // NOTE: can't get keys from the map (easily),
                                 // so store them for iteration

  // ======== Clone Object Node ========

  // ======== Rendering Utility Functions ========
  //! Adds mesh and material to given object
  //! Note (JH): Formerly createMeshObject
  void addMeshToDrawables(const MeshMetaData& metaData,
                          scene::SceneNode& node,
                          DrawableGroup* drawables,
                          int objectID,
                          int meshIDLocal,
                          int materialIDLocal);

  //! Types of supported Shader programs
  enum ShaderType {
    INSTANCE_MESH_SHADER = 0,
    PTEX_MESH_SHADER = 1,
    COLORED_SHADER = 2,
    VERTEX_COLORED_SHADER = 3,
    TEXTURED_SHADER = 4,
  };

  // maps a name to the shader program
  std::map<ShaderType, std::shared_ptr<Magnum::GL::AbstractShaderProgram>>
      shaderPrograms_;

  //! Return Shader of given type, creating if necessary
  Magnum::GL::AbstractShaderProgram* getShaderProgram(ShaderType type);

  //! Create a Drawable with given ShaderType for the given Mesh and SceneNode
  //! If DrawableGroup3D group is given add created Drawable to the group
  //! Optional Texture2D, objectId and color arguments set relevant shader
  //! parameters
  gfx::Drawable& createDrawable(
      const ShaderType shaderType,
      Magnum::GL::Mesh& mesh,
      scene::SceneNode& node,
      Magnum::SceneGraph::DrawableGroup3D* group = nullptr,
      Magnum::GL::Texture2D* texture = nullptr,
      int objectId = ID_UNDEFINED,
      const Magnum::Color4& color = Magnum::Color4{1});

  bool compressTextures_ = false;
};

}  // namespace assets
}  // namespace esp
