
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/MeshTools/Reference.h>

#include "VoxelGrid.h"
#include "esp/assets/ResourceManager.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

VoxelGrid::VoxelGrid(const std::unique_ptr<assets::MeshData>& meshData,
                     int resolution) {
  VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
  /*VHACD::IVHACD::Parameters params;
  params.m_resolution = resolution;
  params.m_oclAcceleration = false;*/
  Mn::Debug() << "Voxelizing mesh..";

  // run VHACD
  interfaceVHACD->computeVoxelField(&meshData->vbo[0][0], meshData->vbo.size(),
                                    &meshData->ibo[0], meshData->ibo.size() / 3,
                                    resolution);

  // get VHACD volume, set scale and dimensions
  VHACD::Volume* vhacdVolume = interfaceVHACD->getVoxelField();
  double scale = vhacdVolume->getScale();
  m_voxelSize = Mn::Vector3(scale, scale, scale);
  const size_t* dims = vhacdVolume->getDimensions();
  m_voxelGridDimensions = Mn::Vector3i(dims[0], dims[1], dims[2]);

  VHACD::Vec3<double> center = vhacdVolume->getCenter();
  Mn::Debug() << "Barycenter" << center[0] << center[1] << center[2];

  // VHACD computes a axis-aligned bounding box; we need to offset the voxelgrid
  // by the minimum corner of the AABB
  VHACD::Vec3<double> minBB = vhacdVolume->getMinBB();
  m_offset = Mn::Vector3(minBB[0], minBB[1], minBB[2]);

  // create empty VoxelGrid
  const int gridSize = dims[0] * dims[1] * dims[2];
  bool* b_grid = new bool[gridSize];
  std::shared_ptr<bool> boundary_grid(b_grid);
  boolGrids_.insert(std::make_pair("boundary", boundary_grid));
  int num_filled = 0;
  // Transfer data from Volume to VoxelGrid
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        if (vhacdVolume->GetVoxel(i, j, k) >= 2) {
          num_filled++;
          setBoolVoxelByIndex(Mn::Vector3i(i, j, k), true);
        } else {
          setBoolVoxelByIndex(Mn::Vector3i(i, j, k), false);
        }
      }
    }
  }
  Mn::Debug() << "Resolution: " << dims[0] << dims[1] << dims[2];
  Mn::Debug() << "Number of filled voxels: " << num_filled;
}

VoxelGrid::VoxelGrid(const Mn::Vector3& voxelSize,
                     const Mn::Vector3i& voxelGridDimensions) {
  m_voxelSize = voxelSize;
  m_voxelGridDimensions = voxelGridDimensions;
  m_offset = Mn::Vector3(0.0, 0.0, 0.0);
  const int gridSize =
      voxelGridDimensions[0] * voxelGridDimensions[1] * voxelGridDimensions[2];
  bool* b_grid = new bool[gridSize];
  std::shared_ptr<bool> boundary_grid(b_grid);
  std::memset(boundary_grid.get(), false, gridSize * sizeof(bool));
  boolGrids_.insert(std::make_pair("boundary", boundary_grid));
}

VoxelGrid::VoxelGrid(const std::string filepath) {}

// Creators for extra voxel grids

void VoxelGrid::addBoolGrid(const std::string gridName) {
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  bool* n_grid = new bool[gridSize];
  std::shared_ptr<bool> new_grid(n_grid);
  std::memset(new_grid.get(), false, gridSize * sizeof(bool));
  boolGrids_.insert(std::make_pair(gridName, new_grid));
}

void VoxelGrid::addIntGrid(const std::string gridName) {
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  int* n_grid = new int[gridSize];
  std::shared_ptr<int> new_grid(n_grid);
  std::memset(new_grid.get(), 0, gridSize * sizeof(int));
  intGrids_.insert(std::make_pair(gridName, new_grid));
}

void VoxelGrid::addFloatGrid(const std::string gridName) {
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  float* n_grid = new float[gridSize];
  std::shared_ptr<float> new_grid(n_grid);
  std::memset(new_grid.get(), 0, gridSize * sizeof(float));
  floatGrids_.insert(std::make_pair(gridName, new_grid));
}

void VoxelGrid::addVector3Grid(const std::string gridName) {
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  Mn::Vector3* n_grid = new Mn::Vector3[gridSize];
  std::shared_ptr<Mn::Vector3> new_grid(n_grid);
  std::memset(new_grid.get(), 0, gridSize * sizeof(Mn::Vector3));
  vector3Grids_.insert(std::make_pair(gridName, new_grid));
}

// Currently naive hashing. TODO: Look into hibert curves and Z-curve
// alternatives for mappings that maximize memory locality of adjacent cells.
int VoxelGrid::hashVoxelIndex(const Mn::Vector3i& coords) {
  int hashed_voxel =
      coords[0] + coords[1] * m_voxelGridDimensions[0] +
      coords[2] * m_voxelGridDimensions[0] * m_voxelGridDimensions[1];
  return hashed_voxel;
}

//  --== GETTERS AND SETTERS FOR VOXELS ==--

// Getter and setter for bool value voxel grids
bool VoxelGrid::getBoolVoxelByIndex(const Mn::Vector3i& coords,
                                    std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return boolGrids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setBoolVoxelByIndex(const Mn::Vector3i& coords,
                                    bool val,
                                    std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  boolGrids_[gridName].get()[hashedVoxelIndex] = val;
}
// Getter and setter for int value voxel grids
int VoxelGrid::getIntVoxelByIndex(const Mn::Vector3i& coords,
                                  std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return intGrids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setIntVoxelByIndex(const Mn::Vector3i& coords,
                                   int val,
                                   std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  intGrids_[gridName].get()[hashedVoxelIndex] = val;
}

// Getter and setter for Float value voxel grids
float VoxelGrid::getFloatVoxelByIndex(const Mn::Vector3i& coords,
                                      std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return floatGrids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setFloatVoxelByIndex(const Mn::Vector3i& coords,
                                     float val,
                                     std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  floatGrids_[gridName].get()[hashedVoxelIndex] = val;
}

// Getter and setter for Vector3 value voxel grids
Mn::Vector3 VoxelGrid::getVector3VoxelByIndex(const Mn::Vector3i& coords,
                                              std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return vector3Grids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setVector3VoxelByIndex(const Mn::Vector3i& coords,
                                       Mn::Vector3 val,
                                       std::string gridName) {
  int hashedVoxelIndex = hashVoxelIndex(coords);
  vector3Grids_[gridName].get()[hashedVoxelIndex] = val;
}

Mn::Vector3 VoxelGrid::getGlobalCoords(const Mn::Vector3i& coords) {
  Mn::Vector3 global_coords((coords[0]) * m_voxelSize[0],
                            (coords[1]) * m_voxelSize[1],
                            (coords[2]) * m_voxelSize[2]);
  global_coords += m_offset;
  return global_coords;
}

void VoxelGrid::addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                         std::vector<Mn::UnsignedInt>& indices,
                                         Mn::Vector3i local_coords) {
  // add cube to mesh
  Mn::Vector3 mid = getGlobalCoords(local_coords);
  // add vertices
  for (int i = -1; i <= 1; i += 2) {
    for (int j = -1; j <= 1; j += 2) {
      for (int k = -1; k <= 1; k += 2) {
        positions.push_back(mid + Mn::Vector3(i, j, k) * m_voxelSize /
                                      2);  // TODO: Divide m_voxelSize by 2
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

void VoxelGrid::generateMesh() {
  std::vector<Mn::UnsignedInt> indices;
  std::vector<Mn::Vector3> positions;
  int num_filled = 0;
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i local_coords(i, j, k);
        int val = getBoolVoxelByIndex(local_coords);
        if (val == 1) {
          num_filled++;
          addVoxelToMeshPrimitives(positions, indices, local_coords);
        }
      }
    }
  }

  Mn::Debug() << "Number of filled voxels for the visual mesh: " << num_filled;
  meshData_ = std::make_shared<Mn::Trade::MeshData>(Mn::MeshTools::owned(
      Mn::Trade::MeshData{Mn::MeshPrimitive::Triangles,
                          {},
                          indices,
                          Mn::Trade::MeshIndexData{indices},
                          {},
                          positions,
                          {Mn::Trade::MeshAttributeData{
                              Mn::Trade::MeshAttribute::Position,
                              Cr::Containers::arrayView(positions)}}}));
  meshGL_ =
      std::make_unique<Magnum::GL::Mesh>(Mn::MeshTools::compile(*meshData_));
}

}  // namespace geo
}  // namespace esp
