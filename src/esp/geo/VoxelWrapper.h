#ifndef ESP_GEO_VOXEL_WRAPPER_H_
#define ESP_GEO_VOXEL_WRAPPER_H_

#include "VoxelGrid.h"

namespace esp {
namespace geo {

class VoxelWrapper {
 private:
  esp::scene::SceneNode* voxelVisualSceneNode;
  VoxelGrid* voxelGrid;

 public:
  VoxelWrapper(VoxelGrid* vGrid, esp::scene::SceneNode* vVisualSceneNode);
}

}  // namespace geo
}  // namespace esp
#endif
