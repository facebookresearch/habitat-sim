#ifndef ESP_GEO_VOXEL_WRAPPER_H_
#define ESP_GEO_VOXEL_WRAPPER_H_

#include "VoxelGrid.h"
#include "esp/assets/ResourceManager.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace geo {

class VoxelWrapper {
 private:
  esp::scene::SceneNode* SceneNode;
  std::shared_ptr<VoxelGrid> voxelGrid;

 public:
#ifdef ESP_BUILD_WITH_VHACD
  /**
   * @brief Generates or retrieves a voxelization of a render asset mesh
   * depending on whether it exists or not.
   * @param renderAssetHandle The handle for the render asset to which the voxel
   * grid corresponds.
   * @param sceneNode The scene node the voxel wrapper will be pointing to.
   * @param resourceManager_ Used for retrieving or registering the voxel grid.
   */
  VoxelWrapper(std::string renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager_,
               int resolution = 1000000);
#endif

  /**
   * @brief Generates a voxelization with a specified size and dimensions.
   * @param renderAssetHandle The handle for the render asset to which the voxel
   * grid corresponds.
   * @param sceneNode The scene node the voxel wrapper will be pointing to.
   * @param resourceManager_ Used for registering the voxel grid.
   * @param voxelSize The size of an individual voxel cell.
   * @param voxelDimensions The dimensions of the voxel grid.
   */
  VoxelWrapper(std::string renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager_,
               Mn::Vector3& voxelSize,
               Mn::Vector3i& voxelDimensions);

  /**
   * @brief Converts a global coordinate into voxel coordinates.
   * @param coords The global coordinate.
   * @return The voxel index corresponding to the global coordinate.
   */
  Mn::Vector3i getVoxelIndexFromGlobalCoords(Mn::Vector3 coords);

  /**
   * @brief Converts a voxel coordinate into global coordinates based on voxel
   * size, offset, and the transformation of the rigid body this wrapper is
   * attached to.
   * @param index The voxel index.
   * @return The global coordinate of the middle of the specified voxel cell.
   */
  Mn::Vector3 getGlobalCoordsFromVoxelIndex(Mn::Vector3i index);

  /**
   * @brief Returns the underlying voxel grid for direct manipulation.
   * @return The underlying voxel grid.
   */
  std::shared_ptr<VoxelGrid> getVoxelGrid();
};

}  // namespace geo
}  // namespace esp
#endif
