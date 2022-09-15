
#include <cmath>

#include "VoxelWrapper.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

#ifdef ESP_BUILD_WITH_VHACD
VoxelWrapper::VoxelWrapper(const std::string& renderAssetHandle,
                           esp::scene::SceneNode* sceneNode,
                           esp::assets::ResourceManager& resourceManager_,
                           int resolution = 1000000)
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
    voxelGrid =
        std::make_shared<VoxelGrid>(*objMesh, renderAssetHandle, resolution);
    assert(resourceManager_.registerVoxelGrid(voxelGridHandle, voxelGrid));
  }
}
#endif

VoxelWrapper::VoxelWrapper(const std::string& handle,
                           esp::scene::SceneNode* sceneNode,
                           esp::assets::ResourceManager& resourceManager_,
                           Mn::Vector3& voxelSize,
                           Mn::Vector3i& voxelDimensions)
    : SceneNode(sceneNode) {
  int resolution = voxelDimensions[0] * voxelDimensions[1] * voxelDimensions[2];
  std::string voxelGridHandle = handle + "_" + std::to_string(resolution);
  // check for existence of specified VoxelGrid
  if (resourceManager_.voxelGridExists(
          voxelGridHandle)) {  // if it exists, simply point the wrapper to it.
    voxelGrid = resourceManager_.getVoxelGrid(voxelGridHandle);
  } else {  // if not, create a new voxel
    voxelGrid = std::make_shared<VoxelGrid>(voxelSize, voxelDimensions);
    assert(resourceManager_.registerVoxelGrid(voxelGridHandle, voxelGrid));
  }
}

Mn::Vector3i VoxelWrapper::getVoxelIndexFromGlobalCoords(
    const Mn::Vector3& coords) {
  // get absolute transform of Rigid Body.
  Mn::Matrix4 absTransform = SceneNode->Magnum::SceneGraph::AbstractObject<
      3, float>::absoluteTransformationMatrix();

  Mn::Vector3 transformedCoords =
      (absTransform.inverted().transformPoint(coords) -
       voxelGrid->getOffset()) /
      voxelGrid->getVoxelSize();
  // return the coords as Vector3i
  return Mn::Vector3i(transformedCoords + Mn::Vector3(0.5, 0.5, 0.5));
}

Mn::Vector3 VoxelWrapper::getGlobalCoordsFromVoxelIndex(
    const Mn::Vector3i& index) {
  Mn::Vector3 globalCoords = voxelGrid->getGlobalCoords(index);
  Mn::Matrix4 absTransform = SceneNode->Magnum::SceneGraph::AbstractObject<
      3, float>::absoluteTransformationMatrix();
  return absTransform.transformPoint(globalCoords);
}

}  // namespace geo
}  // namespace esp
