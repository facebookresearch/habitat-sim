
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Shaders/Phong.h>

#include "VoxelGrid.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

VoxelGrid::VoxelGrid() {}
VoxelGrid::VoxelGrid(const assets::MeshData& meshData, int resolution) {
  VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
  VHACD::IVHACD::Parameters params;
  params.m_resolution = resolution;
  interfaceVHACD->computeVoxelField(&meshData.vbo[0][0], meshData.vbo.size(),
                                    &meshData.ibo[0], meshData.ibo.size() / 3,
                                    params);
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
  int hashed_voxel =
      coords[0] + coords[2] * m_voxelGridDimensions[0] +
      coords[1] * m_voxelGridDimensions[0] * m_voxelGridDimensions[2];
  return hashed_voxel;
}

// Gets a voxel pointer based on local coords (coords.y * x.size * z.size +
// coords.z * x.size + coords.x) O(1) access. Returns nullptr if invalid
// coordinates.
Voxel* VoxelGrid::getVoxelByIndex(const Mn::Vector3i& coords) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return m_grid[hashedVoxelIndex];
}

Mn::Vector3 VoxelGrid::getGlobalCoords(const Mn::Vector3i& coords) {
  Mn::Vector3 global_coords(coords[0] * m_voxelSize[0],
                            coords[1] * m_voxelSize[1],
                            coords[2] * m_voxelSize[2]);
  global_coords += m_voxelSize;
  return global_coords;
}

void VoxelGrid::setVoxelByIndex(const Mn::Vector3i& coords, Voxel* voxel) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  m_grid[hashedVoxelIndex] = voxel;
}

void VoxelGrid::addVoxelToMeshPrimitives(
    std::vector<Mn::Vector3>& positions,
    std::vector<Mn::UnsignedShort>& indices,
    Mn::Vector3i local_coords) {
  // add cube to mesh
  Mn::Vector3 mid = getGlobalCoords(local_coords);
  // add vertices
  for (int i = -1; i <= 1; i += 2) {
    for (int j = -1; j <= 1; j += 2) {
      for (int k = -1; k <= 1; k += 2) {
        positions.push_back(mid + Mn::Vector3(i, j, k) * m_voxelSize / 2);
      }
    }
  }
  // add triangles (12)
  unsigned int sz = positions.size() - 8;
  indices.push_back(sz);
  indices.push_back(sz + 3);
  indices.push_back(sz + 2);
  indices.push_back(sz);
  indices.push_back(sz + 1);
  indices.push_back(sz + 3);

  indices.push_back(sz + 4);
  indices.push_back(sz + 6);
  indices.push_back(sz + 7);
  indices.push_back(sz + 4);
  indices.push_back(sz + 7);
  indices.push_back(sz + 5);

  indices.push_back(sz + 2);
  indices.push_back(sz + 7);
  indices.push_back(sz + 6);
  indices.push_back(sz + 2);
  indices.push_back(sz + 3);
  indices.push_back(sz + 7);

  indices.push_back(sz + 4);
  indices.push_back(sz + 0);
  indices.push_back(sz + 2);
  indices.push_back(sz + 4);
  indices.push_back(sz + 2);
  indices.push_back(sz + 6);

  indices.push_back(sz + 5);
  indices.push_back(sz + 3);
  indices.push_back(sz + 1);
  indices.push_back(sz + 5);
  indices.push_back(sz + 7);
  indices.push_back(sz + 3);

  indices.push_back(sz + 1);
  indices.push_back(sz + 4);
  indices.push_back(sz + 5);
  indices.push_back(sz + 1);
  indices.push_back(sz + 0);
  indices.push_back(sz + 4);
}

void VoxelGrid::fillVoxelMeshData(
    Cr::Containers::Optional<Mn::Trade::MeshData>& mesh) {
  std::vector<Mn::UnsignedShort> indices;
  std::vector<Mn::Vector3> positions;
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i local_coords(i, j, k);
        Voxel* vox = getVoxelByIndex(local_coords);
        if (vox == nullptr)
          continue;
        if (vox->is_filled) {
          addVoxelToMeshPrimitives(positions, indices, local_coords);
        }
      }
    }
  }

  mesh = Mn::MeshTools::owned(Mn::Trade::MeshData{
      Mn::MeshPrimitive::Triangles,
      {},
      indices,
      Mn::Trade::MeshIndexData{indices},
      {},
      positions,
      {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                    Cr::Containers::arrayView(positions)}}});
}

}  // namespace geo
}  // namespace esp
