// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::assets::ResourceManager, enum @ref
 * esp::assets::ResourceManager::ShaderType
 */

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
#include "esp/gfx/DrawableGroup.h"
#include "esp/gfx/MaterialData.h"
#include "esp/gfx/ShaderManager.h"
#include "esp/gfx/configure.h"
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
namespace nav {
class PathFinder;
}
namespace assets {

/**
 * @brief Singleton class responsible for
 * loading and managing common simulator assets such as meshes, textures, and
 * materials.
 */
class ResourceManager {
 public:
  /** @brief Constructor */
  explicit ResourceManager();

  /** @brief Destructor */
  ~ResourceManager() {}

  /** @brief Stores references to a set of drawable elements */
  using DrawableGroup = gfx::DrawableGroup;
  /** @brief Convenience typedef for Importer class */
  using Importer = Magnum::Trade::AbstractImporter;

  //! @brief The @ref ShaderManager key for @ref LightInfo which has no lights
  static constexpr char NO_LIGHT_KEY[] = "no_lights";

  //! @brief The @ref ShaderManager key for the default @ref LightInfo
  static constexpr char DEFAULT_LIGHTING_KEY[] = "";

  //! @brief The @ref ShaderManager key for the default @ref MaterialInfo
  static constexpr char DEFAULT_MATERIAL_KEY[] = "";

  /**
   * @brief Set whether textures should be compressed.
   * @param newVal New texture compression setting.
   */
  inline void compressTextures(bool newVal) { compressTextures_ = newVal; };

  /**
   * @brief Load a scene mesh and add it to the specified @ref DrawableGroup as
   * a child of the specified @ref scene::SceneNode.
   *
   * If parent and drawables are not specified, the assets are loaded, but no
   * new @ref gfx::Drawable is added for the scene (i.e. it will not be
   * rendered).
   * @param info The loaded @ref AssetInfo for the scene mesh.
   * @param parent The @ref scene::SceneNode of which the scene mesh will be
   * added as a child. Typically near the root of the scene. Expected to be
   * static.
   * @param drawables The @ref DrawableGroup with which the scene mesh will be
   * rendered.
   * @param lightSetup The @ref LightSetup used for scene lighting
   * @param splitSemanticMesh Split the semantic mesh by objectID, used for A/B
   * testing
   * @return Whether or not the scene load succeeded.
   */
  bool loadScene(
      const AssetInfo& info,
      scene::SceneNode* parent = nullptr,
      DrawableGroup* drawables = nullptr,
      const Magnum::ResourceKey& lightSetup = Magnum::ResourceKey{NO_LIGHT_KEY},
      bool splitSemanticMesh = true);

  /**
   * @brief Load and instantiate a scene including physics simulation.
   *
   * Loads a physics simulator for the world from the parameters defined in the
   * @ref PhysicsManagerAttributes and reseats the @ref physics::PhysicsManager
   * based on the configured simulator implementation. Loads the scene mesh and
   * adds it to the specified @ref DrawableGroup as a child of the specified
   * @ref scene::SceneNode. If these are not specified, the assets are loaded,
   * but no new @ref gfx::Drawable is added for the scene (i.e. it will not be
   * rendered).
   * @param info The loaded @ref AssetInfo for the scene mesh.
   * @param _physicsManager The currently defined @ref physics::PhysicsManager.
   * Will be reseated to the configured physics implementation.
   * @param physicsManagerAttributes The meta data structure storing configured
   * physics simulation parameters.
   * @param parent The @ref scene::SceneNode of which the scene mesh will be
   * added as a child. Typically near the root of the scene. Expected to be
   * static.
   * @param drawables The @ref DrawableGroup with which the scene mesh will be
   * rendered.
   * @return Whether or not the scene load succeeded.
   */
  bool loadScene(const AssetInfo& info,
                 std::shared_ptr<physics::PhysicsManager>& _physicsManager,
                 PhysicsManagerAttributes physicsManagerAttributes,
                 scene::SceneNode* parent = nullptr,
                 DrawableGroup* drawables = nullptr,
                 const Magnum::ResourceKey& lightSetup = Magnum::ResourceKey{
                     NO_LIGHT_KEY});

  /**
   * @brief Load and instantiate a scene including physics simulation.
   *
   * Loads a physics simulator for the world from the parameters defined in the
   * referenced configuration file. Also attempts to parse physical objects
   * listed in this configuration file into the @ref physicsObjTemplateLibrary_.
   * Reseats the @ref physics::PhysicsManager based on the configured simulator
   * implementation. Loads the scene mesh and adds it to the specified @ref
   * DrawableGroup as a child of the specified @ref scene::SceneNode. If these
   * are not specified, the assets are loaded, but no new @ref gfx::Drawable is
   * added for the scene (i.e. it will not be rendered).
   * @param info The loaded @ref AssetInfo for the scene mesh.
   * @param _physicsManager The currently defined @ref physics::PhysicsManager.
   * Will be reseated to the configured physics implementation.
   * @param parent The @ref scene::SceneNode of which the scene mesh will be
   * added as a child. Typically near the root of the scene. Expected to be
   * static.
   * @param drawables The @ref DrawableGroup with which the scene mesh will be
   * rendered.
   * @param physicsFilename The physics configuration file from which to
   * re-instatiate the @ref physics::PhysicsManager and parse object templates
   * for the
   * @ref physicsObjTemplateLibrary_. Defaults to the file location @ref
   * ESP_DEFAULT_PHYS_SCENE_CONFIG set by cmake.
   * @return Whether or not the scene load succeeded.
   */
  bool loadScene(
      const AssetInfo& info,
      std::shared_ptr<physics::PhysicsManager>& _physicsManager,
      scene::SceneNode* parent = nullptr,
      DrawableGroup* drawables = nullptr,
      const Magnum::ResourceKey& lightSetup = Magnum::ResourceKey{NO_LIGHT_KEY},
      std::string physicsFilename = ESP_DEFAULT_PHYS_SCENE_CONFIG);

  /**
   * @brief Parses global physics simulation parameters (such as timestep,
   * gravity, simulator implementation) from the specified configuration file.
   *
   * @param physicsFilename The configuration file to parse. Defaults to the
   * file location @ref ESP_DEFAULT_PHYS_SCENE_CONFIG set by cmake.
   * @return The physics simulation meta data object parsed from the specified
   * configuration file.
   */
  PhysicsManagerAttributes loadPhysicsConfig(
      std::string physicsFilename = ESP_DEFAULT_PHYS_SCENE_CONFIG);

  /**
   * @brief Get all "*.phys_properties.json" files from the provided file or
   * directory path.
   *
   * @param path A global path to a physics property file or directory
   * @return A list of valid global paths to "*.phys_properties.json" files.
   */
  std::vector<std::string> getObjectConfigPaths(std::string path);

  /**
   * @brief Add an object from a spcified configuration file to the specified
   * @ref DrawableGroup as a child of the specified @ref scene::SceneNode if
   * provided.
   *
   * If the attributes specified by objTemplateID exists in @ref
   * physicsObjTemplateLibrary_, and both parent and drawables are
   * specified, than an object referenced by that key is added to the scene.
   * @param objTemplateLibID The ID of the configuration file to parse and
   * load.  This is expected to exist.
   * @param parent The @ref scene::SceneNode of which the object will be a
   * child.
   * @param drawables The @ref DrawableGroup with which the object @ref
   * gfx::Drawable will be rendered.
   */
  void addObjectToDrawables(int objTemplateLibID,
                            scene::SceneNode* parent,
                            DrawableGroup* drawables,
                            const Magnum::ResourceKey& lightSetup =
                                Magnum::ResourceKey{DEFAULT_LIGHTING_KEY});

  /**
   * @brief Load and parse a physics object template config file and generates a
   * @ref PhysicsObjectAttributes object, adding it to the @ref
   * physicsObjTemplateLibrary_.
   *
   * @param objPhysConfigFilename The configuration file to parse and load.
   * @return The index in the @ref physicsObjTemplateLibrary_ to which the key,
   * objPhysConfigFilename, referes. Can be used to reference the object
   * template, but can change if the @ref physicsObjTemplateLibrary_ is
   * modified.
   */
  int parseAndLoadPhysObjTemplate(const std::string& objPhysConfigFilename);

  /**
   * @brief Add a @ref PhysicsObjectAttributes object to the @ref
   * physicsObjTemplateLibrary_.
   *
   * Can modify template values based on results of load.
   * @param objectTemplateHandle The key for referencing the template in the
   * @ref physicsObjTemplateLibrary_.
   * @param objectTemplate The object template.
   * @return The index in the @ref physicsObjTemplateLibrary_ of object
   * template.
   */
  int loadObjectTemplate(PhysicsObjectAttributes& objectTemplate,
                         const std::string objectTemplateHandle);

  //======== Accessor functions ========
  /**
   * @brief Getter for all @ref assets::CollisionMeshData associated with the
   * particular asset referenced by the key, configFile.
   *
   * @param configFile The key by which the asset is referenced in @ref
   * collisionMeshGroups_ and the @ref physicsObjTemplateLibrary_.
   * @return A vector reference to @ref assets::CollisionMeshData instances for
   * individual components of the asset.
   */
  const std::vector<assets::CollisionMeshData>& getCollisionMesh(
      const std::string configFile);

  /**
   * @brief Getter for all @ref assets::CollisionMeshData associated with the
   * particular asset referenced by the index, objectTemplateID, in @ref
   * physicsObjTemplateLibrary_.
   *
   * @param objectTemplateID The index of the object template in @ref
   * physicsObjTemplateLibrary_.
   * @return A vector reference to @ref assets::CollisionMeshData instances for
   * individual components of the asset.
   */
  const std::vector<assets::CollisionMeshData>& getCollisionMesh(
      const int objectTemplateID);

  /**
   * @brief Get the index in @ref physicsObjTemplateLibrary_ for the object
   * template asset identified by the key, configFile.
   *
   * @param configFile The key referencing the asset in @ref
   * physicsObjTemplateLibrary_.
   * @return The index of the object template in @ref
   * physicsObjTemplateLibrary_.
   */
  int getObjectTemplateID(const std::string& configFile);

  /**
   * @brief Get the key in @ref physicsObjTemplateLibrary_ for the object
   * template asset index.
   *
   * @param objectTemplateID The index of the object template in @ref
   * physicsObjTemplateLibrary_.
   * @return The key referencing the asset in @ref physicsObjTemplateLibrary_.
   */
  std::string getObjectConfig(const int objectTemplateID);

  /**
   * @brief Get a reference to the physics object template for the asset
   * identified by the key, configFile.  physicsObjTemplateLibrary_
   *
   * Can be used to manipulate an object
   * template before instancing new objects.
   * @param configFile The key referencing the asset in @ref
   * physicsObjTemplateLibrary_.
   * @return A mutable reference to the object template for the asset.
   */
  PhysicsObjectAttributes& getPhysicsObjectAttributes(
      const std::string& configFile);

  /**
   * @brief Get a reference to the physics object template for the asset
   * identified by the objectTemplateID.
   *
   * Can be used to manipulate an object
   * template before instancing new objects.
   * @param ObjTmplID The key referencing the asset in @ref
   * physicsObjTemplateLibrary_.
   * @return A mutable reference to the object template for the asset.
   */
  PhysicsObjectAttributes& getPhysicsObjectAttributes(
      const int objectTemplateID);

  /**
   * @brief Gets the number of object templates stored in the @ref
   * physicsObjTemplateLibrary_.
   *
   * @return The size of the @ref physicsObjTemplateLibrary_.
   */
  int getNumLibraryObjects() { return physicsObjTemplateLibrary_.size(); };

  /**
   * @brief Retrieve the composition of all transforms applied to a mesh since
   * it was loaded.
   *
   * See @ref translateMesh.
   * @param meshIndex Index of the mesh in @ref meshes_.
   * @return The transformation matrix mapping from the original state to its
   * current state.
   */
  const Magnum::Matrix4& getMeshTransformation(const size_t meshIndex) {
    return meshes_[meshIndex]->meshTransform_;
  }

  /**
   * @brief Retrieve the meta data for a particular asset.
   *
   * This includes identifiers for meshes, textures, materials, and a component
   * heirarchy.
   * @param filename The key identifying the asset in @ref resourceDict_.
   * Typically the filepath of the asset.
   * @return The asset's @ref MeshMetaData object.
   */
  const MeshMetaData& getMeshMetaData(const std::string& filename) const {
    CHECK(resourceDict_.count(filename) > 0);
    return resourceDict_.at(filename).meshMetaData;
  }

  /**
   * @brief Construct a unified @ref MeshData from a loaded asset's collision
   * meshes.
   *
   * See @ref joinHeirarchy.
   * @param filename The identifying string key for the asset. See @ref
   * resourceDict_ and @ref meshes_.
   * @return The unified @ref MeshData object for the asset.
   */
  std::unique_ptr<MeshData> createJoinedCollisionMesh(
      const std::string& filename);

  /**
   * @brief Create a new drawable primitive attached to the desired @ref
   * scene::SceneNode.
   *
   * See @ref primitive_meshes_.
   * @param primitiveID The index of the primitive in @ref primitive_meshes_.
   * @param node The @ref scene::SceneNode to which the primitive drawable will
   * be attached.
   * @param drawables The @ref DrawableGroup with which the primitive will be
   * rendered.
   */
  void addPrimitiveToDrawables(int primitiveID,
                               scene::SceneNode& node,
                               DrawableGroup* drawables);

  /**
   * @brief Set a named @ref LightSetup
   *
   * If this name already exists, the @ref LightSetup is updated and all @ref
   * Drawables using this setup are updated.
   *
   * @param setup Light setup this key will now reference
   * @param key Key to identify this @ref LightSetup
   */
  void setLightSetup(gfx::LightSetup setup,
                     const Magnum::ResourceKey& key = Magnum::ResourceKey{
                         DEFAULT_LIGHTING_KEY});

  /**
   * @brief Get a named @ref LightSetup
   */
  Magnum::Resource<gfx::LightSetup> getLightSetup(
      const Magnum::ResourceKey& key = Magnum::ResourceKey{
          DEFAULT_LIGHTING_KEY});

  /**
   * @brief generate a new primitive mesh asset for the NavMesh loaded in the
   * provided PathFinder object.
   *
   * If parent and drawables are provided, create the Drawable and render the
   * NavMesh.
   * @param pathFinder Holds the NavMesh information.
   * @param parent The new Drawable is attached to this node.
   * @param drawables The group with which the new Drawable will be rendered.
   * @return The primitive ID of the new object or @ref ID_UNDEFINED if
   * construction failed.
   */
  int loadNavMeshVisualization(esp::nav::PathFinder& pathFinder,
                               scene::SceneNode* parent,
                               DrawableGroup* drawables);

 protected:
  /**
   * @brief Data for a loaded asset
   *
   * Contains mesh, texture, material, and asset info
   */
  struct LoadedAssetData {
    AssetInfo assetInfo;
    MeshMetaData meshMetaData;
  };

  //======== Scene Functions ========

  /**
   * @brief Recursive contruction of scene nodes for an asset.
   *
   * Creates a drawable for the component of an asset referenced by the @ref
   * MeshTransformNode and adds it to the @ref DrawableGroup as child of parent.
   * @param metaData The @ref MeshMetaData object containing information about
   * the meshes, textures, materials, and component heirarchy of the asset.
   * @param parent The @ref scene::SceneNode of which the component will be a
   * child.
   * @param drawables The @ref DrawableGroup with which the component will be
   * rendered.
   * @param meshTransformNode The @ref MeshTransformNode for component
   * identifying its mesh, material, transformation, and children.
   * @param lightSetup The @ref LightSetup key that will be used
   * for the added component.
   */
  void addComponent(const MeshMetaData& metaData,
                    scene::SceneNode& parent,
                    const Magnum::ResourceKey& lightSetup,
                    DrawableGroup* drawables,
                    const MeshTransformNode& meshTransformNode);

  /**
   * @brief Load textures from importer into assets, and update metaData for an
   * asset to link textures to that asset.
   *
   * @param importer The importer already loaded with information for the asset.
   * @param loadedAssetData The asset's @ref LoadedAssetData object.
   */
  void loadTextures(Importer& importer, LoadedAssetData& loadedAssetData);

  /**
   * @brief Load meshes from importer into assets.
   *
   * Compute bounding boxes, upload mesh data to GPU, and update metaData for an
   * asset to link meshes to that asset.
   * @param importer The importer already loaded with information for the asset.
   * @param loadedAssetData The asset's @ref LoadedAssetData object.
   */
  void loadMeshes(Importer& importer, LoadedAssetData& loadedAssetData);

  /**
   * @brief Recursively parse the mesh component transformation heirarchy for
   * the imported asset.
   *
   * @param importer The importer already loaded with information for the asset.
   * @param parent The root of the mesh transform heirarchy for the remaining
   * sub-tree. The generated @ref MeshTransformNode will be added as a child.
   * Typically the @ref MeshMetaData::root to begin recursion.
   * @param componentID The next component to add to the heirarchy. Identifies
   * the component in the @ref Importer.
   */
  void loadMeshHierarchy(Importer& importer,
                         MeshTransformNode& parent,
                         int componentID);

  /**
   * @brief Recursively build a unified @ref MeshData from loaded assets via a
   * tree of @ref MeshTransformNode.
   *
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

  /**
   * @brief Load materials from importer into assets, and update metaData for an
   * asset to link materials to that asset.
   *
   * @param importer The importer already loaded with information for the asset.
   * @param loadedAssetData The asset's @ref LoadedAssetData object.
   */
  void loadMaterials(Importer& importer, LoadedAssetData& loadedAssetData);

  /**
   * @brief Get a @ref PhongMaterialData for use with flat shading
   *
   * Textures must already be loaded for the asset this material belongs to
   *
   * @param material Material data with texture IDs
   * @param textureBaseIndex Base index of the assets textures in textures_
   */
  gfx::PhongMaterialData::uptr getFlatShadedMaterialData(
      const Magnum::Trade::PhongMaterialData& material,
      int textureBaseIndex);

  /**
   * @brief Get a @ref PhongMaterialData for use with phong shading
   *
   * Textures must already be loaded for the asset this material belongs to
   *
   * @param material Material data with texture IDs
   * @param textureBaseIndex Base index of the assets textures in textures_

   */
  gfx::PhongMaterialData::uptr getPhongShadedMaterialData(
      const Magnum::Trade::PhongMaterialData& material,
      int textureBaseIndex);

  /**
   * @brief Load a PTex mesh into assets from a file and add it to the scene
   * graph for rendering.
   *
   * @param info The @ref AssetInfo for the mesh, already parsed from a file.
   * @param parent The @ref scene::SceneNode to which the mesh will be added as
   * a child.
   * @param drawables The @ref DrawableGroup with which the mesh will be
   * rendered.
   */
  bool loadPTexMeshData(const AssetInfo& info,
                        scene::SceneNode* parent,
                        DrawableGroup* drawables);

  /**
   * @brief Load an instance mesh (e.g. Matterport reconstruction) into assets
   * from a file and add it to the scene graph for rendering.
   *
   * @param info The @ref AssetInfo for the mesh, already parsed from a file.
   * @param parent The @ref scene::SceneNode to which the mesh will be added as
   * a child.
   * @param drawables The @ref DrawableGroup with which the mesh will be
   * rendered.
   */
  bool loadInstanceMeshData(const AssetInfo& info,
                            scene::SceneNode* parent,
                            DrawableGroup* drawables,
                            bool splitSemanticMesh = true);

  /**
   * @brief Load a mesh (e.g. gltf) into assets from a file.
   *
   * If both parent and drawables are provided, add the mesh to the
   * scene graph for rendering.
   * @param info The @ref AssetInfo for the mesh, already parsed from a file.
   * @param parent The @ref scene::SceneNode to which the mesh will be added as
   * a child.
   * @param drawables The @ref DrawableGroup with which the mesh will be
   * rendered.
   * @param isScene Whether this asset is being loaded as a scene. If it is then
   * it will be flat shaded for performance reasons
   */
  bool loadGeneralMeshData(const AssetInfo& info,
                           scene::SceneNode* parent = nullptr,
                           DrawableGroup* drawables = nullptr,
                           const Magnum::ResourceKey& lightSetup =
                               Magnum::ResourceKey{NO_LIGHT_KEY});

  /**
   * @brief Load a SUNCG mesh into assets from a file. !Deprecated! TODO:
   * remove?
   *
   * @param info The @ref AssetInfo for the mesh, already parsed from a file.
   * @param parent The @ref scene::SceneNode to which the mesh will be added as
   * a child.
   * @param drawables The @ref DrawableGroup with which the mesh will be
   * rendered.
   */
  bool loadSUNCGHouseFile(const AssetInfo& info,
                          scene::SceneNode* parent,
                          DrawableGroup* drawables);

  /**
   * @brief initialize default lighting setups in the current ShaderManager
   */
  void initDefaultLightSetups();

  /**
   * @brief initialize default material setups in the current ShaderManager
   */
  void initDefaultMaterials();

  /**
   * @brief Checks if light setup is compatible with loaded asset
   */
  bool isLightSetupCompatible(const LoadedAssetData& loadedAssetData,
                              const Magnum::ResourceKey& lightSetup) const;

  // ======== Geometry helper functions, data structures ========

  /**
   * @brief Apply a translation to the vertices of a mesh asset and store that
   * transformation in @ref BaseMesh::meshTransform_.
   *
   * @param meshDataGL The mesh data.
   * @param translation The translation transform to apply.
   */
  void translateMesh(BaseMesh* meshDataGL, Magnum::Vector3 translation);

  /**
   * @brief Compute and return the axis aligned bounding box of a mesh in mesh
   * local space
   * @param meshDataGL The mesh data.
   * @return The mesh bounding box.
   */
  Magnum::Range3D computeMeshBB(BaseMesh* meshDataGL);

  /**
   * @brief Compute the absolute AABBs for drawables in PTex mesh in world space
   * @param baseMesh: ptex mesh
   */
#ifdef ESP_BUILD_PTEX_SUPPORT
  void computePTexMeshAbsoluteAABBs(BaseMesh& baseMesh);
#endif

  /**
   * @brief Compute the absolute AABBs for drawables in general mesh (e.g.,
   * MP3D) world space
   */
  void computeGeneralMeshAbsoluteAABBs();

  /**
   * @brief Compute the absolute AABBs for drawables in semantic mesh in world
   * space
   */
  void computeInstanceMeshAbsoluteAABBs();

  /**
   * @brief Compute absolute transformations of all drwables stored in
   * staticDrawableInfo_
   */
  std::vector<Magnum::Matrix4> computeAbsoluteTransformations();

  /**
   * node: drawable's scene node
   *
   * meshID:
   * -) for non-ptex mesh:
   * meshID is the global index into meshes_.
   * meshes_[meshID] is the BaseMesh corresponding to the drawable;
   *
   * -) for ptex mesh:
   * meshID is the index of the submesh corresponding to the drawable;
   */
  struct StaticDrawableInfo {
    esp::scene::SceneNode& node;
    uint32_t meshID;
  };
  /**
   * @brief this helper vector contains information of the drawables on which
   * we will compute the absolute AABB pair
   *
   */
  std::vector<StaticDrawableInfo> staticDrawableInfo_;
  bool computeAbsoluteAABBs_ = false;

  // ======== General geometry data ========
  // shared_ptr is used here, instead of Corrade::Containers::Optional, or
  // std::optional because shared_ptr is reference type, not value type, and
  // thus we can avoiding duplicated loading

  /**
   * @brief The mesh data for loaded assets.
   */
  std::vector<std::shared_ptr<BaseMesh>> meshes_;

  /**
   * @brief The texture data for loaded assets.
   */
  std::vector<std::shared_ptr<Magnum::GL::Texture2D>> textures_;

  /**
   * @brief The next available unique ID for loaded materials
   */
  int nextMaterialID_ = 0;

  /**
   * @brief A pointer to render mesh data for the most recently loaded instance
   * mesh. //TODO: remove? doesn't seem to be used anywhere.
   */
  Magnum::GL::Mesh* instance_mesh_;

  /**
   * @brief Asset metadata linking meshes, textures, materials, and the
   * component transformation heirarchy for loaded assets.
   *
   * Maps absolute path keys to metadata.
   */
  std::map<std::string, LoadedAssetData> resourceDict_;

  /**
   * @brief The @ref ShaderManager used to store shader information for
   * drawables created by this ResourceManager
   */
  gfx::ShaderManager shaderManager_;

  // ======== Physical parameter data ========

  /**
   * @brief Maps string keys (typically property filenames) to physical object
   * templates.
   *
   * Templates are used by @ref physics::PhysicsManager to instance
   * new objects with common parameters. For example:
   * "data/objects/cheezit.phys_properties.json" -> physicalMetaData
   */
  std::map<std::string, PhysicsObjectAttributes> physicsObjTemplateLibrary_;

  /**
   * @brief Maps string keys (typically property filenames) to physical scene
   * templates.
   *
   * Templates are used by @ref physics::PhysicsManager to
   * initialize, reset scenes or switch contexts.
   */
  std::map<std::string, PhysicsSceneAttributes> physicsSceneLibrary_;

  /**
   * @brief Library of physics scene attributes for
   * initializing/resetting/switching physics world contexts.
   */
  std::map<std::string, PhysicsManagerAttributes> physicsManagerLibrary_;

  /**
   * @brief Primitive meshes available for instancing via @ref
   * addPrimitiveToDrawables for debugging or visualization purposes.
   */
  std::vector<std::unique_ptr<Magnum::GL::Mesh>> primitive_meshes_;

  /**
   * @brief Maps string keys (typically property filenames) to @ref
   * CollisionMeshData for all components of a loaded asset.
   */
  std::map<std::string, std::vector<CollisionMeshData>> collisionMeshGroups_;

  /**
   * @brief Maps object template ID to object template file names
   *
   * See @ref physicsObjTemplateLibrary_, @ref collisionMeshGroups_. NOTE: can't
   * get keys from the map (easily), so store them for iteration.
   * TODO: remove this. Unnecessary: use an iterator to get the keys.
   */
  std::map<int, std::string> physicsObjTmpltLibByID_;

  // ======== Rendering Utility Functions ========

  /**
   * @brief Creates a new @ref gfx::Drawable for a mesh and adds it to the scene
   * graph @ref scene::SceneNode.
   *
   * @param metaData Object meta data for the asset this mesh is linked to.
   * @param node The @ref scene::SceneNode which the new @ref gfx::Drawable will
   * be attached to.
   * @param lightSetup The @ref LightSetup key that will be used
   * for the added mesh.
   * @param drawables The @ref DrawableGroup with which the new @ref
   * gfx::Drawable will be rendered.
   * @param objectID The object type identifier or semantic group (e.g.
   * 1->chair, 2->table, etc..) for semantic rendering of the mesh.
   * @param meshIDLocal The index of the mesh within the mesh group linked to
   * the asset via the @ref MeshMetaData.
   * @param materialIDLocal The index of the material within the material group
   * linked to the asset via the @ref MeshMetaData.
   */
  void addMeshToDrawables(const MeshMetaData& metaData,
                          scene::SceneNode& node,
                          const Magnum::ResourceKey& lightSetup,
                          DrawableGroup* drawables,
                          int objectID,
                          int meshIDLocal,
                          int materialIDLocal);

  /**
   * @brief Create a @ref gfx::Drawable for the specified mesh, node,
   * and @ref ShaderType.
   *
   * Add this drawable to the @ref DrawableGroup if provided.
   * @param shaderType Indentifies the desired shader program for rendering the
   * @ref gfx::Drawable.
   * @param mesh The render mesh.
   * @param node The @ref scene::SceneNode to which the drawable will be
   * attached.
   * @param lightSetup The @ref LightSetup key that will be used
   * for the drawable.
   * @param material The @ref MaterialData key that will be used
   * for the drawable.
   * @param meshID Optional, the index of this mesh component stored in meshes_
   * @param group Optional @ref DrawableGroup with which the render the @ref
   * gfx::Drawable.
   * @param texture Optional texture for the mesh.
   * @param objectId Optional object type indentifier or semantic type for the
   * mesh (e.g. 1->table, 2->chair, etc...).
   * @param color Optional color parameter for the shader program. Defaults to
   * white.
   */
  void createGenericDrawable(Magnum::GL::Mesh& mesh,
                             scene::SceneNode& node,
                             const Magnum::ResourceKey& lightSetup,
                             const Magnum::ResourceKey& material,
                             DrawableGroup* group = nullptr,
                             int objectId = ID_UNDEFINED);

  /**
   * @brief Flag to denote the desire to compress textures. TODO: unused?
   */
  bool compressTextures_ = false;
};

}  // namespace assets
}  // namespace esp
