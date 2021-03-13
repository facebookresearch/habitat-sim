
#include <assert.h>
#include <limits.h>
#include <cmath>

#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/Primitives/Cube.h>

#include "VoxelGrid.h"
#include "esp/assets/ResourceManager.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

#ifdef ESP_BUILD_WITH_VHACD
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

  // VHACD computes a axis-aligned bounding box; we need to offset the voxelgrid
  // by the minimum corner of the AABB
  VHACD::Vec3<double> minBB = vhacdVolume->getMinBB();
  m_offset = Mn::Vector3(minBB[0], minBB[1], minBB[2]);

  // create empty VoxelGrid
  const int gridSize = dims[0] * dims[1] * dims[2];
  bool* b_grid = new bool[gridSize];
  std::shared_ptr<bool> boundary_grid(b_grid);
  boolGrids_.insert(std::make_pair("Boundary", boundary_grid));
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
}
#endif

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
  boolGrids_.insert(std::make_pair("Boundary", boundary_grid));
}

// Creators for extra voxel grids

void VoxelGrid::addBoolGrid(const std::string& gridName) {
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

void VoxelGrid::addIntGrid(const std::string& gridName) {
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

void VoxelGrid::addFloatGrid(const std::string& gridName) {
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

void VoxelGrid::addVector3Grid(const std::string& gridName) {
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

Mn::Vector3i VoxelGrid::reverseHash(const int hash) {
  return Mn::Vector3i(
      hash % m_voxelGridDimensions[0],
      hash / m_voxelGridDimensions[0] % m_voxelGridDimensions[1],
      hash / m_voxelGridDimensions[0] / m_voxelGridDimensions[1]);
}

//  --== GETTERS AND SETTERS FOR VOXELS ==--

// Getter and setter for bool value voxel grids
bool VoxelGrid::getBoolVoxelByIndex(const Mn::Vector3i& coords,
                                    const std::string& gridName) {
  assert(boolGrids_.find(gridName) != boolGrids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return boolGrids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setBoolVoxelByIndex(const Mn::Vector3i& coords,
                                    bool val,
                                    const std::string& gridName) {
  assert(boolGrids_.find(gridName) != boolGrids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  boolGrids_[gridName].get()[hashedVoxelIndex] = val;
}
// Getter and setter for int value voxel grids
int VoxelGrid::getIntVoxelByIndex(const Mn::Vector3i& coords,
                                  const std::string& gridName) {
  assert(intGrids_.find(gridName) != intGrids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return intGrids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setIntVoxelByIndex(const Mn::Vector3i& coords,
                                   int val,
                                   const std::string& gridName) {
  assert(intGrids_.find(gridName) != intGrids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  intGrids_[gridName].get()[hashedVoxelIndex] = val;
}

// Getter and setter for Float value voxel grids
float VoxelGrid::getFloatVoxelByIndex(const Mn::Vector3i& coords,
                                      const std::string& gridName) {
  assert(floatGrids_.find(gridName) != floatGrids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return floatGrids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setFloatVoxelByIndex(const Mn::Vector3i& coords,
                                     float val,
                                     const std::string& gridName) {
  assert(floatGrids_.find(gridName) != floatGrids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  floatGrids_[gridName].get()[hashedVoxelIndex] = val;
}

// Getter and setter for Vector3 value voxel grids
Mn::Vector3 VoxelGrid::getVector3VoxelByIndex(const Mn::Vector3i& coords,
                                              const std::string& gridName) {
  assert(vector3Grids_.find(gridName) != vector3Grids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  return vector3Grids_[gridName].get()[hashedVoxelIndex];
}

void VoxelGrid::setVector3VoxelByIndex(const Mn::Vector3i& coords,
                                       const Mn::Vector3& val,
                                       const std::string& gridName) {
  assert(vector3Grids_.find(gridName) != vector3Grids_.end());
  int hashedVoxelIndex = hashVoxelIndex(coords);
  vector3Grids_[gridName].get()[hashedVoxelIndex] = val;
}

std::shared_ptr<Mn::Trade::MeshData> VoxelGrid::getMeshData(
    const std::string& gridName) {
  if (meshDataDict_[gridName] == nullptr)
    generateMesh(gridName);
  return meshDataDict_[gridName];
}

Mn::GL::Mesh& VoxelGrid::getMeshGL(const std::string& gridName) {
  if (meshDataDict_[gridName] == nullptr)
    generateMesh(gridName);
  return meshGLDict_[gridName];
}

Mn::Vector3 VoxelGrid::getGlobalCoords(const Mn::Vector3i& coords) {
  Mn::Vector3 global_coords((coords[0]) * m_voxelSize[0],
                            (coords[1]) * m_voxelSize[1],
                            (coords[2]) * m_voxelSize[2]);
  global_coords += m_offset;
  return global_coords;
}

int VoxelGrid::generateBoolGridFromIntGrid(const std::string& intGridName,
                                           std::string boolGridName,
                                           int startRange,
                                           int endRange) {
  assert(intGrids_.find(intGridName) != intGrids_.end());
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

int VoxelGrid::generateBoolGridFromVector3Grid(
    const std::string& vector3GridName,
    std::string boolGridName,
    bool func(Mn::Vector3)) {
  assert(vector3Grids_.find(vector3GridName) != vector3Grids_.end());
  int num_filled = 0;
  if (boolGridName == "")
    boolGridName = vector3GridName;
  addBoolGrid(boolGridName);
  for (int i = 0; i < m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                          m_voxelGridDimensions[2];
       i++) {
    if (func(vector3Grids_[vector3GridName].get()[i])) {
      boolGrids_[boolGridName].get()[i] = true;
      num_filled++;
    }
  }
  return num_filled;
}

int VoxelGrid::generateBoolGridFromFloatGrid(const std::string& floatGridName,
                                             std::string boolGridName,
                                             float startRange,
                                             float endRange) {
  assert(floatGrids_.find(floatGridName) != floatGrids_.end());
  int num_filled = 0;
  if (boolGridName == "")
    boolGridName = floatGridName;
  addBoolGrid(boolGridName);
  for (int i = 0; i < m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                          m_voxelGridDimensions[2];
       i++) {
    if (floatGrids_[floatGridName].get()[i] >= startRange &&
        floatGrids_[floatGridName].get()[i] <= endRange) {
      boolGrids_[boolGridName].get()[i] = true;
      num_filled++;
    }
  }
  return num_filled;
}

void VoxelGrid::fillVoxelSetFromBoolGrid(std::vector<Mn::Vector3i>& voxelSet,
                                         const std::string& boolGridName,
                                         bool (*func)(bool)) {
  assert(boolGrids_.find(boolGridName) != boolGrids_.end());
  int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                 m_voxelGridDimensions[2];
  for (int i = 0; i < gridSize; i++) {
    if (func(boolGrids_[boolGridName].get()[i])) {
      voxelSet.push_back(reverseHash(i));
    }
  }
}

void VoxelGrid::fillVoxelSetFromIntGrid(std::vector<Mn::Vector3i>& voxelSet,
                                        const std::string& intGridName,
                                        bool (*func)(int)) {
  assert(intGrids_.find(intGridName) != intGrids_.end());
  int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                 m_voxelGridDimensions[2];
  for (int i = 0; i < gridSize; i++) {
    if (func(intGrids_[intGridName].get()[i])) {
      voxelSet.push_back(reverseHash(i));
    }
  }
}

void VoxelGrid::fillVoxelSetFromFloatGrid(std::vector<Mn::Vector3i>& voxelSet,
                                          const std::string& floatGridName,
                                          bool (*func)(float)) {
  assert(floatGrids_.find(floatGridName) != floatGrids_.end());
  int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                 m_voxelGridDimensions[2];
  for (int i = 0; i < gridSize; i++) {
    if ((*func)(floatGrids_[floatGridName].get()[i])) {
      voxelSet.push_back(reverseHash(i));
    }
  }
}

void VoxelGrid::fillVoxelSetFromVector3Grid(std::vector<Mn::Vector3i>& voxelSet,
                                            const std::string& vector3GridName,
                                            bool (*func)(Mn::Vector3)) {
  assert(vector3Grids_.find(vector3GridName) != vector3Grids_.end());
  int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                 m_voxelGridDimensions[2];
  for (int i = 0; i < gridSize; i++) {
    if (func(vector3Grids_[vector3GridName].get()[i])) {
      voxelSet.push_back(reverseHash(i));
    }
  }
}

// 6D SDF - labels each cell as interior (-inf), exterior (+inf), or boundary
// (0)
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
  bool nX = false, pX = false, nY = false, pY = false, nZ = false, pZ = false;
  int hash = 0;
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
// computes SDF in terms of manhattan distance with double sweep approach
void VoxelGrid::generateManhattanDistanceSDF(const std::string& gridName) {
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
        int i_behind = INT_MAX, j_behind = INT_MAX, k_behind = INT_MAX;
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
        int closest = 0;
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
        int i_ahead = INT_MAX, j_ahead = INT_MAX, k_ahead = INT_MAX;
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

void VoxelGrid::generateEuclideanDistanceSDF(const std::string& gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  if (intGrids_.find("InteriorExterior") == intGrids_.end()) {
    generateInteriorExteriorVoxelGrid();
  }
  // create new vector3Grid and fill data from interior/exterior grid
  addVector3Grid("ClosestBoundaryCell");
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        int hashedIndex = hashVoxelIndex(Mn::Vector3i(i, j, k));
        int label = intGrids_["InteriorExterior"].get()[hashedIndex];
        if (label == 0) {
          vector3Grids_["ClosestBoundaryCell"].get()[hashedIndex] =
              Mn::Vector3(i, j, k);
        } else {
          // intializing the closest boundary cell to be very far / invalid, so
          // it is ensured to be overwritten in the SDF calculation sweeps.
          vector3Grids_["ClosestBoundaryCell"].get()[hashedIndex] =
              Mn::Vector3(m_voxelGridDimensions) * 2;
        }
      }
    }
  }

  // 1st sweep
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3 i_behind = Mn::Vector3(m_voxelGridDimensions) * 2,
                    j_behind = Mn::Vector3(m_voxelGridDimensions) * 2,
                    k_behind = Mn::Vector3(m_voxelGridDimensions) * 2;
        if (isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = getVector3VoxelByIndex(Mn::Vector3i(i - 1, j, k),
                                            "ClosestBoundaryCell");
        }
        if (isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = getVector3VoxelByIndex(Mn::Vector3i(i, j - 1, k),
                                            "ClosestBoundaryCell");
        }
        if (isValidIndex(Mn::Vector3i(i, j, k - 1))) {
          k_behind = getVector3VoxelByIndex(Mn::Vector3i(i, j, k - 1),
                                            "ClosestBoundaryCell");
        }
        Mn::Vector3 coords(i, j, k);

        // get the currently recorded closest boundary distance
        float cur_dist = (getVector3VoxelByIndex(Mn::Vector3i(i, j, k),
                                                 "ClosestBoundaryCell") -
                          coords)
                             .length();

        // get the current distances from each point's closest boundary and
        // the current coordinates.
        float i_dist = NAN, j_dist = NAN, k_dist = NAN;
        i_dist = (i_behind - coords).length();
        j_dist = (j_behind - coords).length();
        k_dist = (k_behind - coords).length();

        if (i_dist <= j_dist && i_dist <= k_dist && i_dist <= cur_dist) {
          setVector3VoxelByIndex(Mn::Vector3i(i, j, k), i_behind,
                                 "ClosestBoundaryCell");
        } else if (j_dist <= i_dist && j_dist <= k_dist && j_dist <= cur_dist) {
          setVector3VoxelByIndex(Mn::Vector3i(i, j, k), j_behind,
                                 "ClosestBoundaryCell");
        } else if (k_dist <= i_dist && k_dist <= j_dist && k_dist <= cur_dist) {
          setVector3VoxelByIndex(Mn::Vector3i(i, j, k), k_behind,
                                 "ClosestBoundaryCell");
        }
      }
    }
  }
  // create float grid for distances, will be filled in this sweep.
  addFloatGrid(gridName);
  // second sweep
  for (int i = m_voxelGridDimensions[0]; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1]; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2]; k >= 0; k--) {
        Mn::Vector3 i_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    j_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    k_ahead = Mn::Vector3(m_voxelGridDimensions) * 2;
        if (isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = getVector3VoxelByIndex(Mn::Vector3i(i + 1, j, k),
                                           "ClosestBoundaryCell");
        }
        if (isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = getVector3VoxelByIndex(Mn::Vector3i(i, j + 1, k),
                                           "ClosestBoundaryCell");
        }
        if (isValidIndex(Mn::Vector3i(i, j, k + 1))) {
          k_ahead = getVector3VoxelByIndex(Mn::Vector3i(i, j, k + 1),
                                           "ClosestBoundaryCell");
        }
        Mn::Vector3 coords(i, j, k);

        // get the currently recorded closest boundary distance
        float cur_dist = (getVector3VoxelByIndex(Mn::Vector3i(i, j, k),
                                                 "ClosestBoundaryCell") -
                          coords)
                             .length();
        // get whether the coord is considered interior or exterior
        int intOrExtVal =
            getIntVoxelByIndex(Mn::Vector3i(i, j, k), "InteriorExterior");
        int intOrExtSign = (intOrExtVal > 0) - (intOrExtVal < 0);

        // get the current distances from each point's closest boundary and
        // the current coordinates.
        float i_dist = NAN, j_dist = NAN, k_dist = NAN;
        i_dist = (i_ahead - coords).length();
        j_dist = (j_ahead - coords).length();
        k_dist = (k_ahead - coords).length();
        if (i_dist <= j_dist && i_dist <= k_dist && i_dist <= cur_dist) {
          setVector3VoxelByIndex(Mn::Vector3i(i, j, k), i_ahead,
                                 "ClosestBoundaryCell");
          setFloatVoxelByIndex(Mn::Vector3i(i, j, k), intOrExtSign * i_dist,
                               gridName);
        } else if (j_dist <= i_dist && j_dist <= k_dist && j_dist <= cur_dist) {
          setVector3VoxelByIndex(Mn::Vector3i(i, j, k), j_ahead,
                                 "ClosestBoundaryCell");
          setFloatVoxelByIndex(Mn::Vector3i(i, j, k), intOrExtSign * j_dist,
                               gridName);
        } else if (k_dist <= i_dist && k_dist <= j_dist && k_dist <= cur_dist) {
          setVector3VoxelByIndex(Mn::Vector3i(i, j, k), k_ahead,
                                 "ClosestBoundaryCell");
          setFloatVoxelByIndex(Mn::Vector3i(i, j, k), intOrExtSign * k_dist,
                               gridName);
        } else {
          setFloatVoxelByIndex(Mn::Vector3i(i, j, k), intOrExtSign * cur_dist,
                               gridName);
        }
      }
    }
  }
}

void VoxelGrid::generateDistanceFlowField(const std::string& gridName) {
  // generateEuclideanDistanceSDF();
  addVector3Grid(gridName);
  int gridSize = m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
                 m_voxelGridDimensions[2];
  for (int i = 0; i < gridSize; i++) {
    Mn::Vector3 index = Mn::Vector3(reverseHash(i));
    vector3Grids_[gridName].get()[i] =
        index - vector3Grids_["ClosestBoundaryCell"].get()[i];
  }
}

void VoxelGrid::addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                         std::vector<Mn::Vector3>& normals,
                                         std::vector<Mn::Color3>& colors,
                                         std::vector<Mn::UnsignedInt>& indices,
                                         const Mn::Vector3i& local_coords) {
  // Using the data of a cubeSolid to create the voxel cube
  Mn::Trade::MeshData cubeData = Mn::Primitives::cubeSolid();

  // add cube to mesh

  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(local_coords);

  auto cubePositions = cubeData.positions3DAsArray();
  auto cubeNormals = cubeData.normalsAsArray();

  for (int i = 0; i < 24; i++) {
    Mn::Vector3 vertOffset = cubePositions[i] * m_voxelSize / 2;
    positions.push_back(vertOffset + mid);
    // Set the normals to be weighted such that cubes look slightly curved
    normals.push_back(cubePositions[i].normalized() * 1 / 4 +
                      cubeNormals[i].normalized() * 3 / 4);
    colors.push_back(Mn::Color3(.4, .8, 1));
  }
  // cube faces
  unsigned int sz = positions.size() - 24;
  auto cubeIndices = cubeData.indices();
  for (int i = 0; i < 36; i++) {
    indices.push_back(sz + cubeIndices[i][0]);
  }
}

void VoxelGrid::addVectorToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                          std::vector<Mn::Vector3>& normals,
                                          std::vector<Mn::Color3>& colors,
                                          std::vector<Mn::UnsignedInt>& indices,
                                          const Mn::Vector3i& local_coords,
                                          const Mn::Vector3& vec) {
  Mn::Vector3 mid = getGlobalCoords(local_coords);
  Mn::Vector3 pos1 = vec.normalized() * m_voxelSize * 1 / 2 + mid;
  Mn::Vector3 orthog1 = Mn::Math::cross(vec, Mn::Vector3(0, 1, 0));
  if (orthog1 == Mn::Vector3(0, 0, 0)) {
    orthog1 = Mn::Vector3(1, 0, 0);
  }
  Mn::Vector3 orthog2 = Mn::Math::cross(vec, orthog1);

  Mn::Vector3 pos2 = mid + orthog1.normalized() * m_voxelSize * 1 / 20;
  Mn::Vector3 pos3 = mid + orthog2.normalized() * m_voxelSize * 1 / 20;
  Mn::Vector3 pos4 = mid - orthog1.normalized() * m_voxelSize * 1 / 20;
  Mn::Vector3 pos5 = mid - orthog2.normalized() * m_voxelSize * 1 / 20;

  positions.push_back(pos1);
  positions.push_back(pos2);
  positions.push_back(pos3);
  positions.push_back(pos4);
  positions.push_back(pos5);

  colors.push_back(Mn::Color3(1, 1, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));

  normals.push_back(vec.normalized());
  normals.push_back((pos1 - mid).normalized());
  normals.push_back((pos2 - mid).normalized());
  normals.push_back((pos3 - mid).normalized());
  normals.push_back((pos4 - mid).normalized());

  unsigned int sz = positions.size() - 5;
  indices.push_back(sz);
  indices.push_back(sz + 1);
  indices.push_back(sz + 2);

  indices.push_back(sz);
  indices.push_back(sz + 2);
  indices.push_back(sz + 3);

  indices.push_back(sz);
  indices.push_back(sz + 3);
  indices.push_back(sz + 4);

  indices.push_back(sz);
  indices.push_back(sz + 4);
  indices.push_back(sz + 1);
}

void VoxelGrid::generateMesh(const std::string& gridName, bool isVectorField) {
  if (isVectorField)
    assert(vector3Grids_.find(gridName) != vector3Grids_.end());
  else
    assert(boolGrids_.find(gridName) != boolGrids_.end());
  std::vector<Mn::UnsignedInt> indices;
  std::vector<Mn::Vector3> positions;
  std::vector<Mn::Vector3> normals;
  std::vector<Mn::Color3> colors;
  int num_filled = 0;

  // iterate through each voxel grid cell
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i local_coords(i, j, k);
        if (isVectorField) {
          Mn::Vector3 vec = getVector3VoxelByIndex(local_coords, gridName);
          if (vec != Mn::Vector3(0, 0, 0))
            addVectorToMeshPrimitives(positions, normals, colors, indices,
                                      local_coords, vec);
        } else {
          bool val = getBoolVoxelByIndex(local_coords, gridName);
          if (val) {
            num_filled++;
            addVoxelToMeshPrimitives(positions, normals, colors, indices,
                                     local_coords);
          }
        }
      }
    }
  }

  // If the mesh already exists, replace it. Otherwise, create a new entry in
  // the dict
  if (meshDataDict_.find(gridName) != meshDataDict_.end()) {
    Mn::Trade::MeshData positionMeshData{
        Mn::MeshPrimitive::Triangles,
        {},
        indices,
        Mn::Trade::MeshIndexData{indices},
        {},
        positions,
        {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                      Cr::Containers::arrayView(positions)}}};
    meshDataDict_[gridName] =
        std::make_shared<Mn::Trade::MeshData>(Mn::MeshTools::interleave(
            positionMeshData,
            {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Color,
                                          Cr::Containers::arrayView(colors)},
             Mn::Trade::MeshAttributeData{
                 Mn::Trade::MeshAttribute::Normal,
                 Cr::Containers::arrayView(normals)}}));
  } else {
    Mn::Trade::MeshData positionMeshData{
        Mn::MeshPrimitive::Triangles,
        {},
        indices,
        Mn::Trade::MeshIndexData{indices},
        {},
        positions,
        {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                      Cr::Containers::arrayView(positions)}}};
    meshDataDict_.insert(std::make_pair(
        gridName,
        std::make_shared<Mn::Trade::MeshData>(Mn::MeshTools::interleave(
            positionMeshData,
            {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Color,
                                          Cr::Containers::arrayView(colors)},
             Mn::Trade::MeshAttributeData{
                 Mn::Trade::MeshAttribute::Normal,
                 Cr::Containers::arrayView(normals)}}))));
  }

  // If the mesh already exists, replace it. Otherwise, create a new entry in
  // the dict
  if (meshGLDict_.find(gridName) != meshGLDict_.end()) {
    meshGLDict_[gridName] = Mn::MeshTools::compile(*meshDataDict_[gridName]);
  } else {
    meshGLDict_.insert(std::make_pair(
        gridName, Mn::MeshTools::compile(*meshDataDict_[gridName])));
  }
}

}  // namespace geo
}  // namespace esp
