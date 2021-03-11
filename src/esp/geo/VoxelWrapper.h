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
  VoxelWrapper(std::string renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager_,
               int resolution = 1000000);
#endif
  VoxelWrapper(std::string renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager_,
               Mn::Vector3& voxelSize,
               Mn::Vector3i& voxelDimensions,
               int resolution = 1000000);
  Mn::Vector3i getVoxelIndexFromGlobalCoords(Mn::Vector3 coords);

  Mn::Vector3 getGlobalCoordsFromVoxelIndex(Mn::Vector3i index);
  std::shared_ptr<VoxelGrid> getVoxelGrid();
};

}  // namespace geo
}  // namespace esp
#endif
