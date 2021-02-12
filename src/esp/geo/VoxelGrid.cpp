

#include "VoxelGrid.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

VoxelGrid::VoxelGrid(const assets::MeshMetaData& meshMetaData, int resolution) {

}

VoxelGrid::VoxelGrid(const assets::MeshMetaData& meshMetaData,
                     Mn::Vector3 voxelSize) {}

VoxelGrid::VoxelGrid(const Mn::Vector3& voxelSize,
                     const Mn::Vector3i& voxelGridDimensions) {
  m_voxelSize = voxelSize;
  m_voxelGridDimensions = voxelGridDimensions;
  m_globalOffset = Mn::Vector3(0.0, 0.0, 0.0);
  const int gridSize =
      voxelGridDimensions[0] * voxelGridDimensions[1] * voxelGridDimensions[2];
  m_grid = new Voxel*[gridSize];
}

VoxelGrid::VoxelGrid(const std::string filepath) {}

// (coords.y * x.size * z.size + coords.z * x.size + coords.x)
int VoxelGrid::hashVoxelIndex(const Mn::Vector3i& coords) {
  return coords[0] + coords[2] * m_voxelGridDimensions[0] +
         coords[1] * m_voxelGridDimensions[0] * m_voxelGridDimensions[2];
}

// Gets a voxel pointer based on local coords (coords.y * x.size * z.size +
// coords.z * x.size + coords.x) O(1) access. Returns nullptr if invalid
// coordinates.
Voxel* VoxelGrid::getVoxelByLocalCoords(const Mn::Vector3i& coords) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return m_grid[hashedVoxelIndex];
}

void VoxelGrid::setVoxelByIndex(const Mn::Vector3i& coords, Voxel* voxel) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  m_grid[hashedVoxelIndex] = voxel;
}

}  // namespace geo
}  // namespace esp
