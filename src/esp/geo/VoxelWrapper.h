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
  VoxelWrapper(std::string renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager_,
               int resolution = 1000000);

  std::shared_ptr<VoxelGrid> getVoxelGrid();
};

}  // namespace geo
}  // namespace esp
#endif
