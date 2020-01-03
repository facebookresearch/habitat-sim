// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file */

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

#include "Asset.h"
#include "Attributes.h"
#include "BaseMesh.h"
#include "CollisionMeshData.h"
#include "GltfMeshData.h"
#include "MeshData.h"
#include "MeshMetaData.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneNode.h"

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
struct SceneConfiguration;
}
namespace physics {
class PhysicsManager;
class RigidObject;
}  // namespace physics
namespace assets {

/**
@brief Loaded asset and resource manager.
*/
class ResourceManager {
 public:
  // Singleton
  // a common design pattern for implementing
  // subsystems such as "resource manager", thats make up an engine is
  // to define a singleton class;
  explicit ResourceManager(){};
  ~ResourceManager() {}

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
                 PhysicsManagerAttributes physicsManagerAttributes,
                 scene::SceneNode* parent = nullptr,
                 DrawableGroup* drawables = nullptr);

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
      std::string physicsFilename = "data/default.phys_scene_config.json");

  /**
   * @brief Load and parse a physics world config file and generates a @ref
   * PhysicsManagerAttributes object.
   * @param physicsFilename The config file.
   * @return The resulting @ref PhysicsManagerAttributes object.
   */
  PhysicsManagerAttributes loadPhysicsConfig(
      std::string physicsFilename = "data/default.phys_scene_config.json");

  /**
   * @brief Load the drawable for an object from the @ref physicsObjectLibrary_
   * into the scene graph. Attempts to load and parse the referenced object
   * template file into the @ref physicsObjectLibrary_ if not already present.
   * @param objPhysConfigFilename The string key for the object template in the
   * @ref physicsObjectLibrary_.
   * @param parent The parent @ref scene::SceneNode for the object.
   * @param drawables The @ref DrawableGroup for the object drawable.
   * @return the object's index in @ref physicsObjectList_
   */
  int loadObject(const std::string& objPhysConfigFilename,
                 scene::SceneNode* parent,
                 DrawableGroup* drawables);

  /**
   * @brief Load and parse a physics object template config file and generates a
   * @ref PhysicsObjectAttributes object, adding it to the @ref
   * physicsObjectLibrary_.
   * @param objPhysConfigFilename The config file.
   * @return The index in the @ref physicsObjectLibrary_ of the resulting @ref
   * PhysicsObjectAttributes object.
   */
  int loadObject(const std::string& objPhysConfigFilename);

  /**
   * @brief Add a @ref PhysicsObjectAttributes object to the @ref
   * physicsObjectLibrary_. Can modify template values based on results of load.
   * @param objectTemplateHandle The key for referencing the template in the
   * @ref physicsObjectLibrary_.
   * @param objectTemplate The object template.
   * @return The index in the @ref physicsObjectLibrary_ of object template.
   */
  int loadObject(PhysicsObjectAttributes& objectTemplate,
                 const std::string objectTemplateHandle);

  //======== Accessor functions ========
  const std::vector<assets::CollisionMeshData>& getCollisionMesh(
      const std::string configFile);

  const std::vector<assets::CollisionMeshData>& getCollisionMesh(
      const int objectID);

  int getObjectID(const std::string& configFile);
  std::string getObjectConfig(const int objectID);

  PhysicsObjectAttributes& getPhysicsObjectAttributes(
      const std::string& configFile);

  int getNumLibraryObjects() { return physicsObjectConfigList_.size(); };

  const Magnum::Matrix4& getMeshTransformation(const size_t meshIndex) {
    return meshes_[meshIndex]->meshTransform_;
  };

  /**
   * @brief Public accessor for @ref MeshMetaData. See @ref resourceDict_.
   * @param filename The identifying string key for the asset.
   * @return The @ref MeshMetaData object for the asset.
   */
  const MeshMetaData& getMeshMetaData(const std::string& filename) {
    CHECK(resourceDict_.count(filename) > 0);
    return resourceDict_.at(filename);
  };

  /**
   * @brief Construct a unified @ref MeshData from a loaded asset's collision
   * meshes. See @ref joinHeirarchy.
   * @param filename The identifying string key for the asset. See @ref
   * resourceDict_ and @ref meshes_.
   * @return The unified @ref MeshData object for the asset.
   */
  std::unique_ptr<MeshData> createJoinedCollisionMesh(
      const std::string& filename);

  /**
   * @brief Create a new drawable primitive attached to the desired @ref
   * SceneNode. See @ref primitive_meshes_.
   * @param primitiveID The index of the primitive in @ref primitive_meshes_.
   * @param node The @ref SceneNode to which the primitive drawable will be
   * attached.
   * @param drawables The @ref DrawableGroup with which the primitive will be
   * rendered.
   */
  void addPrimitiveToDrawables(int primitiveID,
                               scene::SceneNode& node,
                               DrawableGroup* drawables);

 protected:
  //======== Scene Functions ========
  //! Instantiate Scene:
  //! (1) create scene node
  //! (2) upload mesh to gpu and drawables
  //! (optional reload of GPU-side assets)
  void addComponent(const MeshMetaData& metaData,
                    scene::SceneNode& parent,
                    DrawableGroup* drawables,
                    const MeshTransformNode& meshTransformNode);

  //! Load textures from importer into assets, and update metaData
  void loadTextures(Importer& importer, MeshMetaData* metaData);

  //! Load meshes from importer into assets, compute bounding boxes, and update
  //! metaData
  void loadMeshes(Importer& importer, MeshMetaData* metaData);

  //! Load the mesh transformation hierarchy for the imported file
  void loadMeshHierarchy(Importer& importer,
                         MeshTransformNode& parent,
                         int componentID);

  /**
   * @brief Recurses through the @ref MeshTransformNode heirarchy starting at
   * root and sets all @ref MeshTransformNode:objectID for nodes referencing
   * meshes.
   * @param objectID The new semantic ID to set.
   * @param root The root of the @ref MeshTransformNode tree for which all mesh
   * semantic objectIDs should be set.
   */
  void setAllObjectIDs(int objectID, MeshTransformNode& root);

  /**
   * @brief Recursively build a unified @ref MeshData from loaded assets via a
   * tree of @ref MeshTransformNode.
   * @param mesh The @ref MeshData being constructed.
   * @param metaData The @ref MeshMetaData for the object heirarchy being
   * joined.
   * @param node The current @ref MeshTransformNode in the recursion.
   * @param transformFromParentToWorld The cumulative transformation up to but
   * not including the current @ref MeshTransformNode.
   */
  void joinHeirarchy(MeshData& mesh,
                     const MeshMetaData& metaData,
                     const MeshTransformNode& node,
                     const Magnum::Matrix4& transformFromParentToWorld);

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
  bool loadGeneralMeshData(const AssetInfo& info,
                           scene::SceneNode* parent = nullptr,
                           DrawableGroup* drawables = nullptr);

  bool loadSUNCGHouseFile(const AssetInfo& info,
                          scene::SceneNode* parent,
                          DrawableGroup* drawables);

  // ======== Geometry helper functions ========

  void translateMesh(BaseMesh* meshDataGL, Magnum::Vector3 translation);

  /**
   * @brief Compute and return the axis aligned bounding box of a mesh.
   * @param meshDataGL The mesh data.
   * @return The mesh bounding box.
   */
  Magnum::Range3D computeMeshBB(BaseMesh* meshDataGL);

  // ======== General geometry data ========
  // shared_ptr is used here, instead of Corrade::Containers::Optional, or
  // std::optional because shared_ptr is reference type, not value type, and
  // thus we can avoiding duplicated loading
  std::vector<std::shared_ptr<BaseMesh>> meshes_;
  std::vector<std::shared_ptr<Magnum::GL::Texture2D>> textures_;
  std::vector<std::shared_ptr<Magnum::Trade::PhongMaterialData>> materials_;

  Magnum::GL::Mesh* instance_mesh_;

  // a dictionary to check if a mesh has been loaded
  // maps: absolutePath -> meshMetaData
  std::map<std::string, MeshMetaData> resourceDict_;  // meshes

  // ======== Physical geometry data ========
  // library of physics object parameters mapped from config filename (used by
  // physicsManager to instantiate physical objects) maps:
  // "data/objects/cheezit.phys_properties.json" -> physicalMetaData
  /** @brief Maps property filenames to physical object templates. */
  std::map<std::string, PhysicsObjectAttributes> physicsObjectLibrary_;
  // library of physics scene attributes for resetting/switching contexts
  std::map<std::string, PhysicsSceneAttributes> physicsSceneLibrary_;
  // library of physics manager attributes for resetting/swapping simulators or
  // simulation parameters
  std::map<std::string, PhysicsManagerAttributes> physicsManagerLibrary_;

  /**
   * @brief Primitive meshes available for instancing via @ref
   * addPrimitiveToDrawables for debugging or visualization purposes.
   */
  std::vector<Magnum::GL::Mesh> primitive_meshes_;

  // maps: "data/objects/cheezit.phys_properties.json" -> collesionMesh group
  std::map<std::string, std::vector<CollisionMeshData>>
      collisionMeshGroups_;  // meshes for the object hierarchies
  // vector of "data/objects/cheezit.phys_properties.json"
  std::vector<std::string>
      physicsObjectConfigList_;  // NOTE: can't get keys from the map (easily),
                                 // so store them for iteration //TODO: remove
                                 // this, unnecessary: use an iterator to get
                                 // the keys

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
      const Magnum::Color4& color = Magnum::Color4{1});

  bool compressTextures_ = false;
};

}  // namespace assets
}  // namespace esp
