// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_RESOURCEMANAGER_H_
#define ESP_ASSETS_RESOURCEMANAGER_H_

/** @file
 * @brief Class @ref esp::assets::ResourceManager
 */

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Containers/Optional.h>
#include <Magnum/DebugTools/ColorMap.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/ResourceManager.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/Trade/Trade.h>

#include "Asset.h"
#include "BaseMesh.h"
#include "CollisionMeshData.h"
#include "GenericMeshData.h"
#include "GenericSemanticMeshData.h"
#include "MeshData.h"
#include "MeshMetaData.h"
#include "RenderAssetInstanceCreationInfo.h"
#include "esp/geo/VoxelGrid.h"
#include "esp/gfx/CubeMap.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/DrawableGroup.h"
#include "esp/gfx/MaterialData.h"
#include "esp/gfx/PbrImageBasedLighting.h"
#include "esp/gfx/ShaderManager.h"
#include "esp/gfx/ShadowMapManager.h"
#include "esp/physics/configure.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"

#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/attributes/AttributesBase.h"

#ifdef ESP_BUILD_WITH_VHACD
#include <VHACD.h>
#endif

namespace Mn = Magnum;

// forward declarations
namespace Magnum {
namespace Trade {
class AbstractImporter;
class PhongMaterialData;
}  // namespace Trade
}  // namespace Magnum

namespace esp {
namespace gfx {
class Drawable;
namespace replay {
class Recorder;
}
}  // namespace gfx
namespace scene {
class SemanticScene;
struct SceneConfiguration;
}  // namespace scene
namespace physics {
class PhysicsManager;
class RigidObject;
}  // namespace physics
namespace nav {
class PathFinder;
}
namespace io {
namespace URDF {
class Model;
}
}  // namespace io
namespace assets {
// used for shadertype specification
using metadata::attributes::ObjectInstanceShaderType;

/**
 * @brief Singleton class responsible for
 * loading and managing common simulator assets such as meshes, textures, and
 * materials.
 */
class ResourceManager {
 public:
  bool getCreateRenderer() const;

  /** @brief Stores references to a set of drawable elements */
  using DrawableGroup = gfx::DrawableGroup;
  /** @brief Convenience typedef for Importer class */
  using Importer = Mn::Trade::AbstractImporter;

#ifdef ESP_BUILD_WITH_VHACD
  /**
   * @brief Simple struct interface for creating and managing VHACD parameters.
   * These parameters are passed into VHACD and specify how convex hull
   * decomposition is ran.
   */
  struct VHACDParameters : VHACD::IVHACD::Parameters {
    VHACDParameters() {
      m_oclAcceleration = 0u;  // OCL Acceleration does not work on VHACD
    }
    ESP_SMART_POINTERS(VHACDParameters)
  };

  VHACD::IVHACD* interfaceVHACD;
#endif

  /**
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedShort {
    /**
     * use pbr image based lighting
     */
    PbrImageBasedLighting = 1 << 0,
  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /** @brief Constructor */
  explicit ResourceManager(metadata::MetadataMediator::ptr& _metadataMediator,
                           Flags flags = {});

  /** @brief Destructor */
  ~ResourceManager();

  /**
   * @brief This function will build the various @ref Importers used by the
   * system.
   */
  void buildImporters();

  /**
   * @brief Build default primitive attribute files and synthesize an object of
   * each type.
   */
  void initDefaultPrimAttributes();

  /**
   * @brief Instantiate, or reinstantiate, PhysicsManager defined by passed
   * attributes
   * @param physicsManager The currently defined @ref physics::PhysicsManager.
   * Will be reseated to the specified physics implementation.
   * @param parent The @ref scene::SceneNode of which the scene mesh will be
   * added as a child. Typically near the root of the scene. Expected to be
   * static.
   * @param physicsManagerAttributes A smart pointer to meta data structure
   * storing configured physics simulation parameters.
   */
  void initPhysicsManager(
      std::shared_ptr<physics::PhysicsManager>& physicsManager,
      scene::SceneNode* parent,
      const metadata::attributes::PhysicsManagerAttributes::ptr&
          physicsManagerAttributes);

  /**
   * @brief Return the currently loaded @ref esp::scene::SemanticScene .
   */
  std::shared_ptr<scene::SemanticScene> getSemanticScene() {
    return semanticScene_;
  }

  /**
   * @brief Return a view of the currently set Semantic scene colormap.
   */
  const std::vector<Mn::Vector3ub>& getSemanticSceneColormap() const {
    return semanticColorMapBeingUsed_;
  }

  /**
   * @brief Build @ref semanticColorMapBeingUsed_ holding the semantic colors
   * defined from a semantic scene descriptor, by iterating through the objects
   * and mapping their color values to their semantic ids.
   */
  void buildSemanticColorMap();

  /**
   * @brief Build @ref semanticColorAsInt_ (array of colors as integers) from
   * the current @ref semanticColorMapBeingUsed_ map. The @ref
   * semanticColorAsInt_ is used when building the color matrix for conversion
   * of colors found in semantic textures to their semantic IDs. When semantic
   * textures are preprocessed, this will not need to be performed.
   */
  void buildSemanticColorAsIntMap();

  /**
   * @brief Remap a semantic annotation texture to have the semantic IDs per
   * pxl.
   * @param srcImage The source texture with the semantic colors.
   * @param clrToSemanticId Large table of all possible colors to their semantic
   * IDs. initialized to 0xffff
   * @return An image of the semantic IDs, with the ID mapped
   */
  Mn::Image2D convertRGBToSemanticId(
      const Mn::ImageView2D& srcImage,
      Cr::Containers::Array<Mn::UnsignedShort>& clrToSemanticId);

  /** @brief check if the @ref esp::scene::SemanticScene exists.*/
  bool semanticSceneExists() const { return (semanticScene_ != nullptr); }

  /**
   * @brief Load semantic scene descriptor file specified by @p ssdFilename ,
   * for the passed @p activeSceneName .
   * @param ssdFilename The fully qualified filename candidate for the ssd file.
   * @param activeSceneName Name of the currently active scene that we will be
   * loading the SSD for.
   * @return whether loaded successfully or not.
   */
  bool loadSemanticSceneDescriptor(const std::string& ssdFilename,
                                   const std::string& activeSceneName);

  /**
   * @brief Load a scene mesh and add it to the specified @ref DrawableGroup as
   * a child of the specified @ref scene::SceneNode.
   *
   * If parent and drawables are not specified, the assets are loaded, but no
   * new @ref gfx::Drawable is added for the scene (i.e. it will not be
   * rendered).
   * @param stageAttributes The @ref StageAttributes that describes the
   * stage
   * @param stageInstanceAttributes The @ref SceneObjectInstanceAttributes that
   * describes this particular instance of the stage.  If nullptr then not
   * created by SceneInstanceAttributes.
   * @param _physicsManager The currently defined @ref physics::PhysicsManager.
   * @param sceneManagerPtr Pointer to scene manager, to fetch drawables and
   * parent node.
   * @param [out] activeSceneIDs active scene ID is in idx 0, if semantic scene
   * is made, its activeID should be pushed onto vector
   * @return Whether or not the scene load succeeded.
   */
  bool loadStage(
      const metadata::attributes::StageAttributes::ptr& stageAttributes,
      const metadata::attributes::SceneObjectInstanceAttributes::cptr&
          stageInstanceAttributes,
      const std::shared_ptr<physics::PhysicsManager>& _physicsManager,
      esp::scene::SceneManager* sceneManagerPtr,
      std::vector<int>& activeSceneIDs);

  /**
   * @brief Construct scene collision mesh group based on name and type of
   * scene.
   * @tparam T type of meshdata desired based on scene type.
   * @param filename The name of the file holding the mesh data
   * @param meshGroup The meshgroup to build
   * @return whether built successfully or not
   */
  template <class T>
  bool buildStageCollisionMeshGroup(const std::string& filename,
                                    std::vector<CollisionMeshData>& meshGroup);

  /**
   * @brief Load/instantiate any required render and collision assets for an
   * object, if they do not already exist in @ref resourceDict_ or @ref
   * collisionMeshGroups_, respectively. Assumes valid render and collisions
   * asset handles have been specified (This is checked/verified during
   * registration.)
   * @param ObjectAttributes The object template describing the object we wish
   * to instantiate, copied from an entry in @ref
   * esp::metadata::managers::ObjectAttributesManager::objectLibrary_.
   * @return whether process succeeded or not - only currently fails if
   * registration call fails.
   */
  bool instantiateAssetsOnDemand(
      const metadata::attributes::ObjectAttributes::ptr& ObjectAttributes);

  //======== Accessor functions ========
  /**
   * @brief Getter for all @ref assets::CollisionMeshData associated with the
   * particular asset.
   *
   * @param collisionAssetHandle The key by which the asset is referenced in
   * @ref collisionMeshGroups_, from the @ref
   * esp::metadata::managers::ObjectAttributesManager::objectLibrary_.
   * @return A vector reference to @ref assets::CollisionMeshData instances for
   * individual components of the asset.
   */
  const std::vector<assets::CollisionMeshData>& getCollisionMesh(
      const std::string& collisionAssetHandle) const {
    auto colMeshGroupIter = collisionMeshGroups_.find(collisionAssetHandle);
    CORRADE_INTERNAL_ASSERT(colMeshGroupIter != collisionMeshGroups_.end());
    return colMeshGroupIter->second;
  }

  /**
   * @brief Return manager for construction and access to asset attributes.
   */
  metadata::managers::AssetAttributesManager::ptr getAssetAttributesManager()
      const {
    return metadataMediator_->getAssetAttributesManager();
  }
  /**
   * @brief Return manager for construction and access to light and lighting
   * layout attributes.
   */
  metadata::managers::LightLayoutAttributesManager::ptr
  getLightLayoutAttributesManager() const {
    return metadataMediator_->getLightLayoutAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes.
   */
  metadata::managers::ObjectAttributesManager::ptr getObjectAttributesManager()
      const {
    return metadataMediator_->getObjectAttributesManager();
  }
  /**
   * @brief Return manager for construction and access to physics world
   * attributes.
   */
  metadata::managers::PhysicsAttributesManager::ptr
  getPhysicsAttributesManager() const {
    return metadataMediator_->getPhysicsAttributesManager();
  }
  /**
   * @brief Return manager for construction and access to scene attributes.
   */
  metadata::managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    return metadataMediator_->getStageAttributesManager();
  }

  /**
   * @brief Set a reference to the current @ref metadataMediator_.  Perform any
   * initialization that may be required when @ref metadataMediator_ is changed.
   * @param MM a reference to the new @ref metadataMediator_.
   */
  void setMetadataMediator(metadata::MetadataMediator::ptr MM) {
    metadataMediator_ = std::move(MM);
  }

  /**
   * @brief Retrieve the meta data for a particular asset.
   *
   * This includes identifiers for meshes, textures, materials, and a
   * component hierarchy.
   * @param metaDataName The key identifying the asset in @ref resourceDict_.
   * Typically the filepath of file-based assets.
   * @return The asset's @ref MeshMetaData object.
   */
  const MeshMetaData& getMeshMetaData(const std::string& metaDataName) const {
    auto resDictMDIter = resourceDict_.find(metaDataName);
    CORRADE_INTERNAL_ASSERT(resDictMDIter != resourceDict_.end());
    return resDictMDIter->second.meshMetaData;
  }

  /**
   * @brief check to see if a particular voxel grid has been created &
   * registered or not.
   * @param voxelGridName The key identifying the asset in @ref resourceDict_.
   * Typically the filepath of file-based assets.
   * @return Whether or not the specified grid exists.
   */
  bool voxelGridExists(const std::string& voxelGridName) const {
    return voxelGridDict_.count(voxelGridName) > 0;
  }

  /**
   * @brief Retrieve a VoxelGrid given a particular voxel grid handle.
   * @param voxelGridName The key identifying the asset in @ref resourceDict_.
   * Typically the filepath of file-based assets.
   * @return The specified VoxelGrid.
   */
  std::shared_ptr<esp::geo::VoxelGrid> getVoxelGrid(
      const std::string& voxelGridName) const {
    auto voxGridIter = voxelGridDict_.find(voxelGridName);
    CORRADE_INTERNAL_ASSERT(voxGridIter != voxelGridDict_.end());
    return voxGridIter->second;
  }

  /**
   * @brief Registers a given VoxelGrid pointer under the given handle in the
   * voxelGridDict_ if no such VoxelGrid has been registered.
   * @param voxelGridHandle The key to register the VoxelGrid under.
   * @param VoxelGridPtr The pointer to the VoxelGrid
   * @return Whether or not the registration succeeded.
   */
  bool registerVoxelGrid(
      const std::string& voxelGridHandle,
      const std::shared_ptr<esp::geo::VoxelGrid>& VoxelGridPtr) {
    auto voxGridEmplaceIter =
        voxelGridDict_.emplace(voxelGridHandle, VoxelGridPtr);
    // return whether placed or not;
    return voxGridEmplaceIter.second;
  }

  /**
   * @brief Get a named @ref LightSetup
   *
   * @param key The key identifying the light setup in shaderManager_.
   * @return The LightSetup object.
   */
  Mn::Resource<gfx::LightSetup> getLightSetup(
      const Mn::ResourceKey& key = Mn::ResourceKey{DEFAULT_LIGHTING_KEY}) {
    return shaderManager_.get<gfx::LightSetup>(key);
  }

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
                     const Mn::ResourceKey& key = Mn::ResourceKey{
                         DEFAULT_LIGHTING_KEY}) {
    shaderManager_.set(key, std::move(setup), Mn::ResourceDataState::Mutable,
                       Mn::ResourcePolicy::Manual);
  }

  /**
   * @brief Construct a unified @ref MeshData from a loaded asset's collision
   * meshes.
   *
   * See @ref joinHierarchy.
   * @param filename The identifying string key for the asset. See @ref
   * resourceDict_ and @ref meshes_.
   * @return The unified @ref MeshData object for the asset.
   */
  std::unique_ptr<MeshData> createJoinedCollisionMesh(
      const std::string& filename) const;

  /**
   * @brief Construct a unified @ref MeshData from a loaded asset's semantic
   * meshes.
   *
   * See @ref joinHierarchy.
   * @param[out] objectIds vector of uint16_t, will be populated with the object
   * ids of the semantic mesh
   * @param filename The identifying string key for the asset. See @ref
   * resourceDict_ and @ref meshes_.
   * @return The unified @ref MeshData object for the asset.
   */
  std::unique_ptr<MeshData> createJoinedSemanticCollisionMesh(
      std::vector<std::uint16_t>& objectIds,
      const std::string& filename) const;

#ifdef ESP_BUILD_WITH_VHACD
  /**
   * @brief Converts a MeshMetaData into a obj file.
   *
   * @param filename The MeshMetaData filename to be converted to obj.
   * @param new_filename The name of the file that will be created.
   * @param filepath The file path, including new file name, for the obj file.
   */
  bool outputMeshMetaDataToObj(const std::string& filename,
                               const std::string& new_filename,
                               const std::string& filepath) const;

  /**
   * @brief Returns the number of resources registered under a given resource
   * name.
   *
   * @param resourceName The name of the resource.
   */
  bool isAssetDataRegistered(const std::string& resourceName) const;

  /**
   * @brief Runs convex hull decomposition on a specified file.
   *
   * @param filename The MeshMetaData filename to be converted.
   * @param chdFilename The new filename for the chd collision mesh.
   * @param params VHACD params that specify resolution, vertices per convex
   * hull, etc.
   * @param saveChdToObj Specifies whether or not to save the newly created
   * convex hull asset to an obj file.
   */
  void createConvexHullDecomposition(
      const std::string& filename,
      const std::string& chdFilename,
      const VHACDParameters& params = VHACDParameters(),
      bool saveChdToObj = false);
#endif
  /**
   * @brief Add an object from a specified object template handle to the
   * specified @ref DrawableGroup as a child of the specified @ref
   * scene::SceneNode if provided.
   *
   * If the attributes specified by objTemplateHandle exists in @ref
   * esp::metadata::managers::ObjectAttributesManager::objectLibrary_, and both
   * parent and drawables are specified, than an object referenced by that key
   * is added to the scene.
   * @param ObjectAttributes The attributes used to create the object being
   * added.
   * @param parent The @ref scene::SceneNode of which the object will be a
   * child.
   * @param drawables The @ref DrawableGroup with which the object @ref
   * gfx::Drawable will be rendered.
   * @param lightSetupKey The @ref LightSetup key that will be used
   * for the added component.
   * @param[out] visNodeCache Cache for pointers to all nodes created as the
   * result of this process.
   */
  void addObjectToDrawables(
      const metadata::attributes::ObjectAttributes::ptr& ObjectAttributes,
      scene::SceneNode* parent,
      DrawableGroup* drawables,
      std::vector<scene::SceneNode*>& visNodeCache,
      const std::string& lightSetupKey = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Create a new drawable primitive attached to the desired @ref
   * scene::SceneNode.
   *
   * See @ref primitive_meshes_.
   * @param primitiveID The key of the primitive in @ref primitive_meshes_.
   * @param node The @ref scene::SceneNode to which the primitive drawable
   * will be attached.
   * @param drawables The @ref DrawableGroup with which the primitive will be
   * rendered.
   */
  void addPrimitiveToDrawables(int primitiveID,
                               scene::SceneNode& node,
                               DrawableGroup* drawables);

  /**
   * @brief Create a @ref gfx::Drawable for the specified mesh, and node.
   *
   * Add this drawable to the @ref DrawableGroup if provided.
   * @param mesh The render mesh.
   * @param meshAttributeFlags flags for the attributes of the render mesh
   * @param node The @ref scene::SceneNode to which the drawable will be
   * attached.
   * @param lightSetupKey The @ref LightSetup key that will be used
   * for the drawable.
   * @param materialKey The @ref MaterialData key that will be used
   * for the drawable.
   * @param group Optional @ref DrawableGroup with which the render the @ref
   * gfx::Drawable.
   */

  void createDrawable(Mn::GL::Mesh* mesh,
                      gfx::Drawable::Flags& meshAttributeFlags,
                      scene::SceneNode& node,
                      const Mn::ResourceKey& lightSetupKey,
                      const Mn::ResourceKey& materialKey,
                      DrawableGroup* group = nullptr);

  /**
   * @brief Remove the specified primitive mesh.
   *
   * @param primitiveID The key of the primitive in @ref primitive_meshes_.
   */
  void removePrimitiveMesh(int primitiveID);

  /**
   * @brief Generate a new primitive mesh asset for the NavMesh loaded in the
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

  /**
   * @brief Generate a tube following the passed trajectory of points.
   * @param trajVisName The name to use for the trajectory visualization mesh.
   * @param pts The points of a trajectory, in order
   * @param colorVec Array of Colors for trajectory tube.
   * @param numSegments The number of the segments around the circumference of
   * the tube. Must be greater than or equal to 3.
   * @param radius The radius of the tube.
   * @param smooth Whether to smooth the points in the trajectory or not
   * @param numInterp The number of interpolations between each trajectory
   * point, if smoothing.
   * @return Whether the process was a success or not
   */
  bool buildTrajectoryVisualization(const std::string& trajVisName,
                                    const std::vector<Mn::Vector3>& pts,
                                    const std::vector<Mn::Color3>& colorVec,
                                    int numSegments = 3,
                                    float radius = .001,
                                    bool smooth = false,
                                    int numInterp = 10);

  /**
   * @brief Build a configuration frame from specified up and front vectors
   * and return it.  If up is not orthogonal to front, will return default
   * frame.
   *
   * @param attribName the handle to the attributes the frame is being built
   * for, for debug purposes.
   * @param up The up vector to build the frame from
   * @param front The front vector to build the frame from.
   * @param origin Either the origin of the stageAttributes or the COM value of
   * the objectAttributes.
   * @return the coordinate frame of the assets the passed attributes describes.
   */
  esp::geo::CoordinateFrame buildFrameFromAttributes(
      const std::string& attribName,
      const Magnum::Vector3& up,
      const Magnum::Vector3& front,
      const Magnum::Vector3& origin);

  /**
   * @brief Sets whether or not the current agent sensor suite requires textures
   * for rendering. Textures will not be loaded if this is false.
   */
  inline void setRequiresTextures(bool newVal) { requiresTextures_ = newVal; }

  /**
   * @brief Set a replay recorder so that ResourceManager can notify it about
   * render assets.
   */
  void setRecorder(
      const std::shared_ptr<gfx::replay::Recorder>& gfxReplayRecorder) {
    gfxReplayRecorder_ = gfxReplayRecorder;
  }

  /**
   * @brief Construct and return a unique string key for the color material and
   * create an entry in the shaderManager_ if new.
   *
   * @param materialColor The color parameters.
   * @return The unique key string identifying the material in shaderManager_.
   */
  std::string createColorMaterial(
      const esp::assets::PhongMaterialColor& materialColor);

  /**
   * @brief Creates an asset name appropriately modified based certain
   * conditions present in passed @p assetInfo.  This function will derive
   * encodings based on the state of the assetInfo so that different material
   * configurations can be specified on the same asset.
   * @param info The AssetInfo that describes the asset being named.
   * @param materialId [in/out] A string key representing the material to use.
   * If empty, this will be generated and populated.
   * @return the modified asset name to be used to save this asset to @p
   * resourceDict_.
   */
  std::string createModifiedAssetName(const AssetInfo& info,
                                      std::string& materialId);

  /**
   * @brief Load a render asset (if not already loaded) and create a render
   * asset instance.
   *
   * @param assetInfo the render asset to load
   * @param creation How to create the instance
   * @param sceneManagerPtr Info about the scene graph(s). See loadStage.
   * @param activeSceneIDs Info about the scene graph(s). See loadStage.
   * @return the root node of the instance, or nullptr (if the load failed)
   */
  scene::SceneNode* loadAndCreateRenderAssetInstance(
      const AssetInfo& assetInfo,
      const RenderAssetInstanceCreationInfo& creation,
      esp::scene::SceneManager* sceneManagerPtr,
      const std::vector<int>& activeSceneIDs);

  /**
   * @brief Load a render asset (if not already loaded) and create a render
   * asset instance at a known SceneNode and Drawables.
   *
   * @param assetInfo the render asset to load
   * @param creation How to create the instance
   * @param parent The parent node under which the visual node hierarchy will be
   * generated.
   * @param drawables The DrawableGroup to which new Drawables will be added.
   * @param visNodeCache A reference to a SceneNode* vector which caches all new
   * SceneNodes created by the attachment process.
   * @return the root node of the instance, or nullptr (if the load failed)
   */
  scene::SceneNode* loadAndCreateRenderAssetInstance(
      const AssetInfo& assetInfo,
      const RenderAssetInstanceCreationInfo& creation,
      scene::SceneNode* parent = nullptr,
      DrawableGroup* drawables = nullptr,
      std::vector<scene::SceneNode*>* visNodeCache = nullptr);

  /**
   * @brief Load a render asset so it can be instanced. See also
   * createRenderAssetInstance.
   */
  bool loadRenderAsset(const AssetInfo& info);

  /**
   * @brief get the shader manager
   */
  gfx::ShaderManager& getShaderManager() { return shaderManager_; }

  /**
   * @brief get the shadow map manager
   */
  gfx::ShadowMapManager& getShadowMapManger() { return shadowManager_; }

  /**
   * @brief get the shadow map keys
   */
  std::map<int, std::vector<Magnum::ResourceKey>>& getShadowMapKeys() {
    return shadowMapKeys_;
  }

  static constexpr const char* SHADOW_MAP_KEY_TEMPLATE =
      "scene_id={}-light_id={}";

  /**
   * @brief Build data for a report for semantic mesh connected components based
   * on color/id.  Returns map of data keyed by SemanticObject index in
   * SemanticObjs array.
   */
  std::unordered_map<uint32_t,
                     std::vector<std::shared_ptr<scene::CCSemanticObject>>>
  buildSemanticCCObjects(
      const metadata::attributes::StageAttributes::ptr& stageAttributes);

  /**
   * @brief Build data for a report for vertex color mapping to semantic scene
   * objects - this list of strings will disclose which colors are found in
   * vertices but not in semantic scene descirptors, and which semantic objects
   * do not have their colors mapped in mesh verts.
   */
  std::vector<std::string> buildVertexColorMapReport(
      const metadata::attributes::StageAttributes::ptr& stageAttributes);

 private:
  /**
   * @brief Load the requested mesh info into @ref meshInfo corresponding to
   * specified @p assetType used by object described by @p objectAttributes
   *
   * @param filename the name of the file describing this mesh
   * @param objectAttributes the object attributes owning
   * this mesh.
   * @param assetType either "render" or "collision" (for error log output)
   * @param forceFlatShading whether to force this asset to be rendered via
   * flat shading.
   * @return whether or not the mesh was loaded successfully
   */
  bool loadObjectMeshDataFromFile(
      const std::string& filename,
      const metadata::attributes::ObjectAttributes::ptr& objectAttributes,
      const std::string& meshType,
      bool forceFlatShading);

  /**
   * @brief Build a primitive asset based on passed template parameters.  If
   * exists already, does nothing.  Will use primitiveImporter_ to call
   * appropriate method to construct asset.
   * @param primTemplateHandle the handle referring to the attributes describing
   * primitive to instantiate
   */
  void buildPrimitiveAssetData(const std::string& primTemplateHandle);

  /**
   * @brief Configure the importerManager_ GL Extensions appropriately based on
   * compilation flags, before any general assets are imported.  This should
   * only occur if a gl context exists.
   */
  void ConfigureImporterManagerGLExtensions();

 protected:
  // ======== Structs and Types only used locally ========
  /**
   * @brief Data for a loaded asset
   *
   * Contains mesh, texture, material, and asset info
   */
  struct LoadedAssetData {
    AssetInfo assetInfo;
    MeshMetaData meshMetaData;
  };

  /**
   * node: drawable's scene node
   *
   * meshID:
   * -) for non-ptex mesh:
   * meshID is the global key into meshes_.
   * meshes_[meshID] is the BaseMesh corresponding to the drawable;
   *
   * -) for ptex mesh:
   * meshID is the index of the submesh corresponding to the drawable;
   */
  struct StaticDrawableInfo {
    esp::scene::SceneNode& node;
    int meshID;
  };

  //======== Scene Functions ========

  /**
   * @brief Determines if passed type is a general mesh data.
   * @param type The type to verify.
   * @return Whether it is a General
   */
  inline bool isRenderAssetGeneral(AssetType type) {
    return type == AssetType::MP3D_MESH || type == AssetType::UNKNOWN;
  }
  /**
   * @brief Recursive construction of scene nodes for an asset.
   *
   * Creates a drawable for the component of an asset referenced by the @ref
   * MeshTransformNode and adds it to the @ref DrawableGroup as child of
   * parent.
   * @param metaData The @ref MeshMetaData object containing information about
   * the meshes, textures, materials, and component hierarchy of the asset.
   * @param parent The @ref scene::SceneNode of which the component will be a
   * child.
   * @param lightSetupKey The @ref LightSetup key that will be used
   * for the added component.
   * @param drawables The @ref DrawableGroup with which the component will be
   * rendered.
   * @param meshTransformNode The @ref MeshTransformNode for component
   * identifying its mesh, material, transformation, and children.
   * @param[out] visNodeCache Cache for pointers to all nodes created as the
   * result of this recursive process.
   * @param computeAbsoluteAABBs whether absolute bounding boxes should be
   * computed
   * @param staticDrawableInfo structure holding the drawable infos for aabbs
   */
  void addComponent(const MeshMetaData& metaData,
                    scene::SceneNode& parent,
                    const Mn::ResourceKey& lightSetupKey,
                    DrawableGroup* drawables,
                    const MeshTransformNode& meshTransformNode,
                    std::vector<scene::SceneNode*>& visNodeCache,
                    bool computeAbsoluteAABBs,
                    std::vector<StaticDrawableInfo>& staticDrawableInfo);

  /**
   * @brief Load textures from importer into assets, and update metaData for
   * an asset to link textures to that asset.
   *
   * @param importer The importer already loaded with information for the
   * asset.
   * @param loadedAssetData The asset's @ref LoadedAssetData object.
   */
  void loadTextures(Importer& importer, LoadedAssetData& loadedAssetData);

  /**
   * @brief Load meshes from importer into assets.
   *
   * Compute bounding boxes, upload mesh data to GPU, and update metaData for
   * an asset to link meshes to that asset.
   * @param importer The importer already loaded with information for the
   * asset.
   * @param loadedAssetData The asset's @ref LoadedAssetData object.
   */
  void loadMeshes(Importer& importer, LoadedAssetData& loadedAssetData);

  /**
   * @brief Recursively build a unified @ref MeshData from loaded assets via a
   * tree of @ref MeshTransformNode.
   *
   * @param[in,out] mesh The @ref MeshData being constructed.
   * @param metaData The @ref MeshMetaData for the object hierarchy being
   * joined.
   * @param node The current @ref MeshTransformNode in the recursion.
   * @param transformFromParentToWorld The cumulative transformation up to but
   * not including the current @ref MeshTransformNode.
   */
  void joinHierarchy(MeshData& mesh,
                     const MeshMetaData& metaData,
                     const MeshTransformNode& node,
                     const Mn::Matrix4& transformFromParentToWorld) const;

  /**
   * @brief Recursively build a unified @ref MeshData from loaded semantic
   * assets via a tree of @ref MeshTransformNode.
   *
   * @param[in,out] mesh The @ref MeshData being constructed.
   * @param[out] meshObjectIds The object ids
   * @param metaData The @ref MeshMetaData for the object hierarchy being
   * joined.
   * @param node The current @ref MeshTransformNode in the recursion.
   * @param transformFromParentToWorld The cumulative transformation up to but
   * not including the current @ref MeshTransformNode.
   */
  void joinSemanticHierarchy(
      MeshData& mesh,
      std::vector<uint16_t>& meshObjectIds,
      const MeshMetaData& metaData,
      const MeshTransformNode& node,
      const Mn::Matrix4& transformFromParentToWorld) const;

  /**
   * @brief Load materials from importer into assets, and update metaData for
   * an asset to link materials to that asset.
   *
   * @param importer The importer already loaded with information for the
   * asset.
   * @param loadedAssetData The asset's @ref LoadedAssetData object.
   */
  void loadMaterials(Importer& importer, LoadedAssetData& loadedAssetData);

  /**
   * @brief Get the appropriate the @ref
   * esp::metadata::attributes::ObjectInstanceShaderType to use to render the
   * passed @p material based on specification in passed @p info or the material
   * itself.
   * @param info The asset info describing the asset whose material is being
   * rendered.
   * @return the @ref esp::metadata::attributes::ObjectInstanceShaderType to use
   * to render the material.
   */
  ObjectInstanceShaderType getMaterialShaderType(const AssetInfo& info) const;

  /**
   * @brief Boolean check if @p typeToCheck aligns with passed types explicitly
   * specified, or type in material
   * @param typeToCheck The ObjectInstanceShaderType value beign queried for.
   * @param materialData The material whose type we are verifying against
   * @param verificationType The ObjectInstanceShaderType we are verifying
   * against
   * @param mnVerificationType The @ref Mn::Trade::MaterialType bitflag that the
   * passed material's specified type is being verified against.
   * @return Whether or not the passed @p typeToCheck matches the passed
   * criteria.
   */
  bool checkForPassedShaderType(
      const ObjectInstanceShaderType typeToCheck,
      const Mn::Trade::MaterialData& materialData,
      const ObjectInstanceShaderType verificationType,
      const Mn::Trade::MaterialType mnVerificationType) const {
    return (
        (typeToCheck == verificationType) ||
        ((typeToCheck == ObjectInstanceShaderType::Material) &&
         ((materialData.types() & mnVerificationType) == mnVerificationType)));
  }

  /**
   * @brief Build a @ref PhongMaterialData for use with flat shading
   *
   * Textures must already be loaded for the asset this material belongs to
   *
   * @param material Material data with texture IDs
   * @param textureBaseIndex Base index of the assets textures in textures_
   */
  gfx::PhongMaterialData::uptr buildFlatShadedMaterialData(
      const Mn::Trade::MaterialData& materialData,
      int textureBaseIndex);

  /**
   * @brief Build a @ref PhongMaterialData for use with phong shading
   *
   * Textures must already be loaded for the asset this material belongs to
   *
   * @param material Material data with texture IDs
   * @param textureBaseIndex Base index of the assets textures in textures_

   */
  gfx::PhongMaterialData::uptr buildPhongShadedMaterialData(
      const Mn::Trade::MaterialData& material,
      int textureBaseIndex) const;

  /**
   * @brief Build a @ref PbrMaterialData for use with PBR shading
   *
   * Textures must already be loaded for the asset this material belongs to
   *
   * @param material Material data with texture IDs
   * @param textureBaseIndex Base index of the assets textures in textures_
   */
  gfx::PbrMaterialData::uptr buildPbrShadedMaterialData(
      const Mn::Trade::MaterialData& material,
      int textureBaseIndex) const;

  /**
   * @brief Load a mesh describing some scene asset based on the passed
   * assetInfo.
   *
   * If both parent and drawables are provided, add the mesh to the
   * scene graph for rendering.
   * @param info The @ref AssetInfo for the mesh, already parsed from a file.
   * @param creation How to instance the render asset, or nullptr if not
   * instancing.
   * @param parent The @ref scene::SceneNode to which the mesh will be added
   * as a child. See also creation->isRGBD and creation->isSemantic. nullptr if
   * not instancing.
   * @param drawables The @ref DrawableGroup with which the mesh will be
   * rendered. See also creation->isRGBD and creation->isSemantic. nullptr if
   * not instancing.
   * @return Whether or not the load was successful.
   */
  bool loadStageInternal(const AssetInfo& info,
                         const RenderAssetInstanceCreationInfo* creation,
                         scene::SceneNode* parent,
                         DrawableGroup* drawables);

  /**
   * @brief Builds the appropriate collision mesh groups for the passed
   * assetInfo, and adds it to the @ref collisionMeshGroup map.
   * @param info The @ref AssetInfo for the mesh, already parsed from a file.
   * @param meshGroup The constructed @ref meshGroup
   * @return Whether the meshgroup was successfully built or not
   */
  bool buildMeshGroups(const AssetInfo& info,
                       std::vector<CollisionMeshData>& meshGroup);

  /**
   * @brief Creates a map of appropriate asset infos for sceneries.  Will always
   * create render asset info.  Will create collision asset info and semantic
   * stage asset info if requested.
   *
   * @param stageAttributes The stage attributes file holding the stage's
   * information.
   * @param createCollisionInfo Whether collision-based asset info should be
   * created (only if physicsManager type is not none)
   * @param createSemanticInfo Whether semantic mesh-based asset info should be
   * created
   */
  std::map<std::string, AssetInfo> createStageAssetInfosFromAttributes(
      const metadata::attributes::StageAttributes::ptr& stageAttributes,
      bool createCollisionInfo,
      bool createSemanticInfo);

  /**
   * @brief PTex Mesh backend for loadRenderAsset
   */
  bool loadRenderAssetPTex(const AssetInfo& info);

  /**
   * @brief Build @ref GenericSemanticMeshData from a single, flattened Magnum
   * Meshdata, built from the meshes provided by the importer, preserving all
   * transformations.  This building process will also synthesize bounding boxes
   * if requested from the @ref semanticScene_ .
   * @param fileImporter Importer used to load the scene.
   * @param info AssetInfo describing asset.
   * @return The GenericSemanticMeshData being built.
   */
  GenericSemanticMeshData::uptr flattenImportedMeshAndBuildSemantic(
      Importer& fileImporter,
      const AssetInfo& info);

  /**
   * @brief Semantic Mesh backend for loadRenderAsset.  Either use
   * loadRenderAssetSemantic if semantic mesh has vertex annotations only, or
   * loadRenderAssetGeneral if semantic mesh has texture-based annotations. This
   * choice is governed by info.hasSemanticTextures.
   */
  bool loadSemanticRenderAsset(const AssetInfo& info);

  /**
   * @brief Semantic (vertex-annotated) Mesh backend for loadRenderAsset
   */
  bool loadRenderAssetSemantic(const AssetInfo& info);

  /**
   * @brief General Mesh backend for loadRenderAsset
   */
  bool loadRenderAssetGeneral(const AssetInfo& info);

  /**
   * @brief Create a render asset instance.
   *
   * @param creation Controls how the instance is created.
   * @param parent The @ref scene::SceneNode to which the instance will be added
   * as a child. See also creation.isRGBD and isSemantic.
   * @param drawables The @ref DrawableGroup with which the instance will be
   * rendered. See also creation.isRGBD and isSemantic.
   * @param[out] visNodeCache Optional; cache for pointers to all nodes created
   * as the result of this process.
   */
  scene::SceneNode* createRenderAssetInstance(
      const RenderAssetInstanceCreationInfo& creation,
      scene::SceneNode* parent,
      DrawableGroup* drawables,
      std::vector<scene::SceneNode*>* visNodeCache = nullptr);

  /**
   * @brief PTex Mesh backend for createRenderAssetInstance
   */
  scene::SceneNode* createRenderAssetInstancePTex(
      const RenderAssetInstanceCreationInfo& creation,
      scene::SceneNode* parent,
      DrawableGroup* drawables);

  /**
   * @brief Semantic Mesh backend creation. Either use
   * createRenderAssetInstanceVertSemantic if semantic mesh has vertex
   * annotations only, or createRenderAssetInstanceGeneralPrimitive if semantic
   * mesh has texture-based annotations. This choice is governed by
   * creation.isTextureBasedSemantic().
   */
  scene::SceneNode* createSemanticRenderAssetInstance(
      const RenderAssetInstanceCreationInfo& creation,
      scene::SceneNode* parent,
      DrawableGroup* drawables);

  /**
   * @brief Semantic Mesh (vertex-annotated) backend for
   * createRenderAssetInstance
   */
  scene::SceneNode* createRenderAssetInstanceVertSemantic(
      const RenderAssetInstanceCreationInfo& creation,
      scene::SceneNode* parent,
      DrawableGroup* drawables);

  /**
   * @brief backend for both General Mesh and Primitive Mesh, for
   * createRenderAssetInstance
   */
  scene::SceneNode* createRenderAssetInstanceGeneralPrimitive(
      const RenderAssetInstanceCreationInfo& creation,
      scene::SceneNode* parent,
      DrawableGroup* drawables,
      std::vector<scene::SceneNode*>* userVisNodeCache);

  /**
   * @brief initialize default lighting setups in the current ShaderManager
   */
  void initDefaultLightSetups();

  /**
   * @brief initialize pbr image based lighting
   * @param[in] hdriImageFilename, the name of the
   * HDRi image (an equirectangular image), that will be converted to a
   * environment cube map
   * NOTE!!! Such an image MUST be SPECIFIED in the
   * ~/habitat-sim/data/pbr/PbrImages.conf
   * and be put in that folder.
   * example image:
   * ~/habitat-sim/data/pbr/lythwood_room_4k.png
   */
  void initPbrImageBasedLighting(const std::string& hdriImageFilename);

  /**
   * @brief initialize default material setups in the current ShaderManager
   */
  void initDefaultMaterials();

  /**
   * @brief Checks if light setup is compatible with loaded asset
   */
  bool isLightSetupCompatible(const LoadedAssetData& loadedAssetData,
                              const Mn::ResourceKey& lightSetupKey) const;

  // ======== Geometry helper functions, data structures ========

  /**
   * @brief Compute and return the axis aligned bounding box of a mesh in mesh
   * local space
   * @param meshDataGL The mesh data.
   * @return The mesh bounding box.
   */
  Mn::Range3D computeMeshBB(BaseMesh* meshDataGL);

#ifdef ESP_BUILD_PTEX_SUPPORT
  /**
   * @brief Compute the absolute AABBs for drawables in PTex mesh in world
   * space
   * @param baseMesh ptex mesh
   */
  void computePTexMeshAbsoluteAABBs(
      BaseMesh& baseMesh,
      const std::vector<StaticDrawableInfo>& staticDrawableInfo);
#endif

  /**
   * @brief Compute the absolute AABBs for drawables in general mesh (e.g.,
   * MP3D) world space
   */
  void computeGeneralMeshAbsoluteAABBs(
      const std::vector<StaticDrawableInfo>& staticDrawableInfo);

  /**
   * @brief Compute the absolute AABBs for drawables in semantic mesh in world
   * space
   */
  void computeInstanceMeshAbsoluteAABBs(
      const std::vector<StaticDrawableInfo>& staticDrawableInfo);

  /**
   * @brief Compute absolute transformations of all drwables stored in
   * staticDrawableInfo_
   */
  std::vector<Mn::Matrix4> computeAbsoluteTransformations(
      const std::vector<StaticDrawableInfo>& staticDrawableInfo);

  // ======== Rendering Utility Functions ========

  Flags flags_;

  // ======== General geometry data ========
  // shared_ptr is used here, instead of Corrade::Containers::Optional, or
  // std::optional because shared_ptr is reference type, not value type, and
  // thus we can avoiding duplicated loading

  /**
   * @brief The next available unique ID for loaded meshes
   */
  int nextMeshID_ = 0;

  /**
   * @brief The mesh data for loaded assets.
   */
  std::map<int, std::shared_ptr<BaseMesh>> meshes_;

  /**
   * @brief The next available unique ID for loaded textures
   */
  int nextTextureID_ = 0;

  /**
   * @brief The texture data for loaded assets.
   */
  std::map<int, std::shared_ptr<Mn::GL::Texture2D>> textures_;

  /**
   * @brief The next available unique ID for loaded materials
   */
  int nextMaterialID_ = 0;

  /**
   * @brief Storage for precomuted voxel grids. Useful for when multiple objects
   * in a scene are using the same VoxelGrid.
   *
   * Maps absolute path keys to VoxelGrid.
   */
  std::map<std::string, std::shared_ptr<esp::geo::VoxelGrid>> voxelGridDict_;

  /**
   * @brief Asset metadata linking meshes, textures, materials, and the
   * component transformation hierarchy for loaded assets.
   *
   * Maps absolute path keys to metadata.
   */
  std::map<std::string, LoadedAssetData> resourceDict_;

  /**
   * @brief The @ref ShaderManager used to store shader information for
   * drawables created by this ResourceManager
   */
  gfx::ShaderManager shaderManager_;

  // ======== Metadata, File and primitive importers ========
  /**
   * @brief A reference to the MetadataMediator managing all the metadata
   * currently in use.
   */
  metadata::MetadataMediator::ptr metadataMediator_ = nullptr;
  /**
   * @brief Plugin Manager used to instantiate importers which in turn are used
   * to load asset data
   */
  Corrade::PluginManager::Manager<Importer> importerManager_;

  /**
   * @brief This @ref GenericSemanticMeshData unique pointer is not used to
   * actually create a mesh, but rather for reporting and color annotations (for
   * texture-based semantics processing). We retain this so we do not need to
   * re-load from scratch for future reporting functionality.
   * NOTE : We must not use this to retain a semantic mesh that is actually
   * being rendered, since that mesh will have its components moved into actual
   * render mesh constructs.
   */
  GenericSemanticMeshData::uptr infoSemanticMeshData_{};

  /**
   * @brief Importer used to synthesize Magnum Primitives (PrimitiveImporter).
   * This object allows for similar usage to File-based importers, but requires
   * no file to be available/read.
   */
  Corrade::Containers::Pointer<Importer> primitiveImporter_;

  /**
   * @brief Importer used to load generic mesh files (AnySceneImporter)
   */
  Corrade::Containers::Pointer<Importer> fileImporter_;

  /**
   * @brief Reference to the currently loaded semanticScene Descriptor
   */
  std::shared_ptr<scene::SemanticScene> semanticScene_ = nullptr;

  /**
   * @brief Colormap to use for visualizing currently loaded semantic scene.
   */
  std::vector<Mn::Vector3ub> semanticColorMapBeingUsed_{};
  std::vector<uint32_t> semanticColorAsInt_{};

  // ======== Physical parameter data ========

  //! tracks primitive mesh ids
  int nextPrimitiveMeshId = 0;
  /**
   * @brief Primitive meshes available for instancing via @ref
   * addPrimitiveToDrawables for debugging or visualization purposes.
   */
  std::map<int, std::unique_ptr<Mn::GL::Mesh>> primitive_meshes_;

  /**
   * @brief Maps string keys (typically property filenames) to @ref
   * CollisionMeshData for all components of a loaded asset.
   */
  std::map<std::string, std::vector<CollisionMeshData>> collisionMeshGroups_;

  /**
   * @brief Flag to load textures of meshes
   */
  bool requiresTextures_ = true;

  /**
   * @brief See @ref setRecorder.
   */
  std::shared_ptr<esp::gfx::replay::Recorder> gfxReplayRecorder_;

  /**
   * @brief The imaged based lighting for PBR, each is a collection of
   * an environment map, an irradiance map, a BRDF lookup table (2D texture),
   * and a pre-fitered map
   */
  std::vector<std::unique_ptr<esp::gfx::PbrImageBasedLighting>>
      pbrImageBasedLightings_;

  int activePbrIbl_ = ID_UNDEFINED;

  /**
   * @brief shadow map for point lights
   */
  // TODO: directional light shadow maps
  gfx::ShadowMapManager shadowManager_;
  // scene graph id -> keys for the shadow maps
  std::map<int, std::vector<Magnum::ResourceKey>> shadowMapKeys_;
};  // namespace assets

CORRADE_ENUMSET_OPERATORS(ResourceManager::Flags)

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_RESOURCEMANAGER_H_
