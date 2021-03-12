
#include <cmath>

#include "VoxelWrapper.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

#ifdef ESP_BUILD_WITH_VHACD
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
#endif

VoxelWrapper::VoxelWrapper(std::string renderAssetHandle,
                           esp::scene::SceneNode* sceneNode,
                           esp::assets::ResourceManager& resourceManager_,
                           Mn::Vector3& voxelSize,
                           Mn::Vector3i& voxelDimensions) {
  int resolution = voxelDimensions[0] * voxelDimensions[1] * voxelDimensions[2];
  std::string voxelGridHandle =
      renderAssetHandle + "_" + std::to_string(resolution);
  // check for existence of specified VoxelGrid
  if (resourceManager_.voxelGridExists(
          voxelGridHandle)) {  // if it exists, simply point the wrapper to it.
    voxelGrid = resourceManager_.getVoxelGrid(voxelGridHandle);
  } else {  // if not, create a new voxel
    voxelGrid = std::make_shared<VoxelGrid>(voxelSize, voxelDimensions);
    resourceManager_.registerVoxelGrid(voxelGridHandle, voxelGrid);
  }
}

Mn::Vector3i VoxelWrapper::getVoxelIndexFromGlobalCoords(Mn::Vector3 coords) {
  // get absolute transform of Rigid Body.
  Mn::Matrix4 absTransform = SceneNode->Magnum::SceneGraph::AbstractObject<
      3, float>::absoluteTransformationMatrix();

  Mn::Vector4 paddedCoords = Mn::Vector4(coords[0], coords[1], coords[2], 1);
  // Apply inverse of the transformation to coords
  Mn::Vector4 paddedTransformedCoords = absTransform.inverted() * paddedCoords;
  // subtract VoxelGrid's m_offset
  Mn::Vector3 transformedCoords =
      Mn::Vector3(paddedTransformedCoords[0], paddedTransformedCoords[1],
                  paddedTransformedCoords[2]);
  transformedCoords -= voxelGrid->getOffset();
  // Divide by voxel size
  transformedCoords /= voxelGrid->getVoxelSize();
  // return the coords as Vector3i
  return Mn::Vector3i(transformedCoords + Mn::Vector3(0.5, 0.5, 0.5));
}

Mn::Vector3 VoxelWrapper::getGlobalCoordsFromVoxelIndex(Mn::Vector3i index) {
  Mn::Vector3 globalCoords = voxelGrid->getGlobalCoords(index);
  Mn::Vector4 paddedCoords =
      Mn::Vector4(globalCoords[0], globalCoords[1], globalCoords[2], 1);
  Mn::Matrix4 absTransform = SceneNode->Magnum::SceneGraph::AbstractObject<
      3, float>::absoluteTransformationMatrix();
  Mn::Vector4 paddedTransformedCoords = absTransform * paddedCoords;
  return Mn::Vector3(paddedTransformedCoords[0], paddedTransformedCoords[1],
                     paddedTransformedCoords[2]);
}

std::shared_ptr<VoxelGrid> VoxelWrapper::getVoxelGrid() {
  return voxelGrid;
}

}  // namespace geo
}  // namespace esp
