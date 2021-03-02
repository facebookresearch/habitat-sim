
#include "VoxelWrapper.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

VoxelWrapper::VoxelWrapper(std::string renderAssetHandle,
                           esp::scene::SceneNode* sceneNode,
                           esp::assets::ResourceManager& resourceManager_,
                           int resolution)
    : SceneNode(sceneNode) {
  std::string voxelGridHandle =
      renderAssetHandle + "_" + std::to_string(resolution);
  // check for existence of specified VoxelGrid
  if (resourceManager_.voxelGridExists(
          voxelGridHandle)) {  // if it exists, simply point the wrapper to it.
    voxelGrid = resourceManager_.getVoxelGrid(voxelGridHandle);
  } else {  // if not, create a new voxel
    std::unique_ptr<esp::assets::MeshData> objMesh =
        esp::assets::MeshData::create_unique();
    objMesh = resourceManager_.createJoinedCollisionMesh(renderAssetHandle);
    voxelGrid = std::make_shared<VoxelGrid>(objMesh, resolution);
    resourceManager_.registerVoxelGrid(voxelGridHandle, voxelGrid);
  }
}

}  // namespace geo
}  // namespace esp
