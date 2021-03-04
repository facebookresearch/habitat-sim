
#include <limits.h>
#include <cmath>

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
  if (boolGrids_.find(gridName) != boolGrids_.end()) {
    // grid exists, simply overwrite
    Mn::Debug() << "Grid" << gridName << "exists, overwriting.";
    std::memset(boolGrids_[gridName].get(), 0,
                m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                    m_voxelGridDimensions[2] * sizeof(bool));
    return;
  }
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  bool* n_grid = new bool[gridSize];
  std::shared_ptr<bool> new_grid(n_grid);
  std::memset(new_grid.get(), false, gridSize * sizeof(bool));
  boolGrids_.insert(std::make_pair(gridName, new_grid));
}

void VoxelGrid::addIntGrid(const std::string gridName) {
  if (intGrids_.find(gridName) != intGrids_.end()) {
    // grid exists, simply overwrite
    Mn::Debug() << "Grid" << gridName << "exists, overwriting.";
    std::memset(intGrids_[gridName].get(), 0,
                m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                    m_voxelGridDimensions[2] * sizeof(int));
    return;
  }
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  int* n_grid = new int[gridSize];
  std::shared_ptr<int> new_grid(n_grid);
  std::memset(new_grid.get(), 0, gridSize * sizeof(int));
  intGrids_.insert(std::make_pair(gridName, new_grid));
}

void VoxelGrid::addFloatGrid(const std::string gridName) {
  if (floatGrids_.find(gridName) != floatGrids_.end()) {
    // grid exists, simply overwrite
    Mn::Debug() << "Grid" << gridName << "exists, overwriting.";
    std::memset(floatGrids_[gridName].get(), 0,
                m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                    m_voxelGridDimensions[2] * sizeof(float));
    return;
  }
  const int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                       m_voxelGridDimensions[2];
  float* n_grid = new float[gridSize];
  std::shared_ptr<float> new_grid(n_grid);
  std::memset(new_grid.get(), 0, gridSize * sizeof(float));
  floatGrids_.insert(std::make_pair(gridName, new_grid));
}

void VoxelGrid::addVector3Grid(const std::string gridName) {
  if (vector3Grids_.find(gridName) != vector3Grids_.end()) {
    // grid exists, simply overwrite
    Mn::Debug() << "Grid" << gridName << "exists, overwriting.";
    std::memset(vector3Grids_[gridName].get(), 0,
                m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                    m_voxelGridDimensions[2] * sizeof(Mn::Vector3));
    return;
  }
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

int VoxelGrid::generateBoolGridFromIntGrid(std::string intGridName,
                                           int startRange,
                                           int endRange,
                                           std::string boolGridName) {
  int num_filled = 0;
  if (boolGridName == "")
    boolGridName = intGridName;
  addBoolGrid(boolGridName);
  for (int i = 0; i < m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                          m_voxelGridDimensions[2];
       i++) {
    if (intGrids_[intGridName].get()[i] >= startRange &&
        intGrids_[intGridName].get()[i] <= endRange) {
      boolGrids_[boolGridName].get()[i] = true;
      num_filled++;
    }
  }
  return num_filled;
}

// 6D SDF - labels each cell as interior, exterior, or boundary
void VoxelGrid::generateInteriorExteriorVoxelGrid() {
  // create 6 bool grids
  addBoolGrid("negXShadow");
  addBoolGrid("posXShadow");
  addBoolGrid("negYShadow");
  addBoolGrid("posYShadow");
  addBoolGrid("negZShadow");
  addBoolGrid("posZShadow");
  // fill each grids with ray cast
  bool hit = false;
  int ind = 0;
  std::string gridName = "InteriorExterior";
  // X axis ray casts
  for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
    for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
      // fill negX
      hit = false;
      ind = m_voxelGridDimensions[0] - 1;
      while (ind--) {
        if (hit) {
          setBoolVoxelByIndex(Mn::Vector3i(ind, j, k), true, "negXShadow");
        } else if (getBoolVoxelByIndex(Mn::Vector3i(ind, j, k))) {
          hit = true;
          setBoolVoxelByIndex(Mn::Vector3i(ind, j, k), true, "negXShadow");
        }
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[0]) {
        if (hit) {
          setBoolVoxelByIndex(Mn::Vector3i(ind, j, k), true, "posXShadow");
        } else if (getBoolVoxelByIndex(Mn::Vector3i(ind, j, k))) {
          hit = true;
          setBoolVoxelByIndex(Mn::Vector3i(ind, j, k), true, "posXShadow");
        }
      }
    }
  }
  // Y axis ray casts
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
      // fill negX
      hit = false;
      ind = m_voxelGridDimensions[1] - 1;
      while (ind--) {
        if (hit) {
          setBoolVoxelByIndex(Mn::Vector3i(i, ind, k), true, "negYShadow");
        } else if (getBoolVoxelByIndex(Mn::Vector3i(i, ind, k))) {
          hit = true;
          setBoolVoxelByIndex(Mn::Vector3i(i, ind, k), true, "negYShadow");
        }
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[1]) {
        if (hit) {
          setBoolVoxelByIndex(Mn::Vector3i(i, ind, k), true, "posYShadow");
        } else if (getBoolVoxelByIndex(Mn::Vector3i(i, ind, k))) {
          hit = true;
          setBoolVoxelByIndex(Mn::Vector3i(i, ind, k), true, "posYShadow");
        }
      }
    }
  }
  // Z axis ray casts
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      // fill negX
      hit = false;
      ind = m_voxelGridDimensions[2] - 1;
      while (ind--) {
        if (hit) {
          setBoolVoxelByIndex(Mn::Vector3i(i, j, ind), true, "negZShadow");
        } else if (getBoolVoxelByIndex(Mn::Vector3i(i, j, ind))) {
          hit = true;
          setBoolVoxelByIndex(Mn::Vector3i(i, j, ind), true, "negZShadow");
        }
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[2]) {
        if (hit) {
          setBoolVoxelByIndex(Mn::Vector3i(i, j, ind), true, "posZShadow");
        } else if (getBoolVoxelByIndex(Mn::Vector3i(i, j, ind))) {
          hit = true;
          setBoolVoxelByIndex(Mn::Vector3i(i, j, ind), true, "posZShadow");
        }
      }
    }
  }

  // create int grid
  addIntGrid(gridName);
  bool nX, pX, nY, pY, nZ, pZ;
  int hash;
  // fill in int grid with voting approach
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (getBoolVoxelByIndex(Mn::Vector3i(i, j, k))) {
          setIntVoxelByIndex(Mn::Vector3i(i, j, k), 0, gridName);
          continue;
        }
        nX = !getBoolVoxelByIndex(Mn::Vector3i(i, j, k), "negXShadow");
        pX = !getBoolVoxelByIndex(Mn::Vector3i(i, j, k), "posXShadow");
        nY = !getBoolVoxelByIndex(Mn::Vector3i(i, j, k), "negYShadow");
        pY = !getBoolVoxelByIndex(Mn::Vector3i(i, j, k), "posYShadow");
        nZ = !getBoolVoxelByIndex(Mn::Vector3i(i, j, k), "negZShadow");
        pZ = !getBoolVoxelByIndex(Mn::Vector3i(i, j, k), "posZShadow");
        // || ((nX || pX) && (nY || pY) && (nZ || pZ))
        if (((nX && pX) || (nY && pY) || (nZ && pZ)) ||
            ((nX || pX) && (nY || pY) && (nZ || pZ))) {
          // Exterior (+inf)
          setIntVoxelByIndex(Mn::Vector3i(i, j, k), INT_MAX, gridName);
        } else {
          // Interior (-inf)
          setIntVoxelByIndex(Mn::Vector3i(i, j, k), INT_MIN, gridName);
        }
      }
    }
  }
}

// Manhattan distance SDF - starting from the interior exterior voxel grid,
// computes SDF with double sweep approach
void VoxelGrid::generateSDF(std::string gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  if (intGrids_.find("InteriorExterior") == intGrids_.end()) {
    generateInteriorExteriorVoxelGrid();
  }
  // create new intGrid and copy data from interior/exterior grid
  addIntGrid(gridName);
  for (int i = 0; i < m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                          m_voxelGridDimensions[2];
       i++) {
    intGrids_[gridName].get()[i] = intGrids_["InteriorExterior"].get()[i];
  }

  // 1st sweep
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        int i_behind, j_behind, k_behind;
        if (isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = abs(
              std::max(getIntVoxelByIndex(Mn::Vector3i(i - 1, j, k), gridName),
                       -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = abs(
              std::max(getIntVoxelByIndex(Mn::Vector3i(i, j - 1, k), gridName),
                       -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j, k - 1))) {
          k_behind = abs(
              std::max(getIntVoxelByIndex(Mn::Vector3i(i, j, k - 1), gridName),
                       -2147483646));
        }
        int curVal = getIntVoxelByIndex(Mn::Vector3i(i, j, k), gridName);
        int closest;
        if (i_behind <= j_behind && i_behind <= k_behind) {
          // i_behind is closest to nearest obstacle.
          closest = i_behind;
        } else if (j_behind <= i_behind && j_behind <= k_behind) {
          // j_behind is closest to nearest obstacle.
          closest = j_behind;
        } else {
          // k_behind is closest or tied for closest to nearest obstacle.
          closest = k_behind;
        }
        // Get the minimum of the cell that's closest to an obstacle and the
        // current distance to an obstacle, and multiply by the true sign of
        // curVal
        if (closest == INT_MAX)
          closest--;
        curVal = ((curVal > 0) - (curVal < 0)) *
                 std::min(abs(std::max(curVal, -2147483647)), closest + 1);
        setIntVoxelByIndex(Mn::Vector3i(i, j, k), curVal, gridName);
      }
    }
  }
  // second sweep
  for (int i = m_voxelGridDimensions[0]; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1]; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2]; k >= 0; k--) {
        int curVal = getIntVoxelByIndex(Mn::Vector3i(i, j, k), gridName);
        if (curVal == 0)
          continue;
        int i_ahead, j_ahead, k_ahead;
        if (isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = abs(
              std::max(getIntVoxelByIndex(Mn::Vector3i(i + 1, j, k), gridName),
                       -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = abs(
              std::max(getIntVoxelByIndex(Mn::Vector3i(i, j + 1, k), gridName),
                       -2147483646));
        }
        if (isValidIndex(Mn::Vector3i(i, j, k + 1))) {
          k_ahead = abs(
              std::max(getIntVoxelByIndex(Mn::Vector3i(i, j, k + 1), gridName),
                       -2147483646));
        }

        int closest;
        if (i_ahead <= j_ahead && i_ahead <= k_ahead) {
          // i_ahead is closest to nearest obstacle.
          closest = i_ahead;
        } else if (j_ahead <= i_ahead && j_ahead <= k_ahead) {
          // j_ahead is closest to nearest obstacle.
          closest = j_ahead;
        } else {
          // k_ahead is closest or tied for closest to nearest obstacle.
          closest = k_ahead;
        }
        // Get the minimum of the cell that's closest to an obstacle and the
        // current distance to an obstacle, and multiply by the true sign of
        // curVal
        if (closest == INT_MAX)
          closest--;
        curVal = ((curVal > 0) - (curVal < 0)) *
                 std::min(abs(std::max(curVal, -2147483647)), closest + 1);
        setIntVoxelByIndex(Mn::Vector3i(i, j, k), curVal, gridName);
      }
    }
  }
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

void VoxelGrid::generateMesh(std::string gridName) {
  std::vector<Mn::UnsignedInt> indices;
  std::vector<Mn::Vector3> positions;
  int num_filled = 0;
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i local_coords(i, j, k);
        bool val = getBoolVoxelByIndex(local_coords, gridName);
        if (val) {
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
