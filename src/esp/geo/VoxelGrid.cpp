
#include <assert.h>
#include <limits.h>
#include <cmath>

#include <Corrade/Utility/Algorithms.h>
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
                     const std::string& renderAssetHandle,
                     int resolution)
    : m_renderAssetHandle(renderAssetHandle) {
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
  m_BBMaxOffset = m_offset + Mn::Vector3(m_voxelGridDimensions) * scale;

  // create empty VoxelGrid
  Corrade::Containers::Array<char> cr_grid{Corrade::Containers::ValueInit,
                                           gridSize() * sizeof(bool)};
  grids_.insert(std::make_pair(
      "Boundary", std::make_pair(VoxelGridType::Bool, std::move(cr_grid))));

  int num_filled = 0;
  // Transfer data from Volume to VoxelGrid
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        if (vhacdVolume->GetVoxel(i, j, k) >= 2) {
          num_filled++;
          setVoxel(Mn::Vector3i(i, j, k), "Boundary", true);
        } else {
          setVoxel(Mn::Vector3i(i, j, k), "Boundary", false);
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
  Corrade::Containers::Array<char> cr_grid{Corrade::Containers::ValueInit,
                                           gridSize() * sizeof(bool)};
  grids_.insert(std::make_pair(
      "Boundary", std::make_pair(VoxelGridType::Bool, std::move(cr_grid))));
}

template <>
VoxelGridType VoxelGrid::voxelGridTypeFor<bool>() {
  return VoxelGridType::Bool;
}
template <>
VoxelGridType VoxelGrid::voxelGridTypeFor<int>() {
  return VoxelGridType::Int;
}
template <>
VoxelGridType VoxelGrid::voxelGridTypeFor<float>() {
  return VoxelGridType::Float;
}
template <>
VoxelGridType VoxelGrid::voxelGridTypeFor<Mn::Vector3>() {
  return VoxelGridType::Vector3;
}

//  --== GETTERS AND SETTERS FOR VOXELS ==--

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
                                           const std::string& boolGridName,
                                           int startRange,
                                           int endRange) {
  assert(grids_.find(intGridName) != grids_.end());
  assert(grids_[intGridName].first == VoxelGridType::Int);
  int num_filled = 0;
  addGrid<bool>(boolGridName);
  auto boolGrid = getGrid<bool>(boolGridName);
  auto intGrid = getGrid<int>(intGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (intGrid[i][j][k] >= startRange && intGrid[i][j][k] <= endRange) {
          boolGrid[i][j][k] = true;
          num_filled++;
        }
      }
    }
  }
  return num_filled;
}

int VoxelGrid::generateBoolGridFromVector3Grid(
    const std::string& vector3GridName,
    const std::string& boolGridName,
    bool func(Mn::Vector3)) {
  assert(grids_.find(vector3GridName) != grids_.end());
  assert(grids_[vector3GridName].first == VoxelGridType::Vector3);
  int num_filled = 0;
  addGrid<bool>(boolGridName);
  auto boolGrid = getGrid<bool>(boolGridName);
  auto vecGrid = getGrid<Mn::Vector3>(vector3GridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (func(vecGrid[i][j][k])) {
          boolGrid[i][j][k] = true;
          num_filled++;
        }
      }
    }
  }
  return num_filled;
}

int VoxelGrid::generateBoolGridFromFloatGrid(const std::string& floatGridName,
                                             const std::string& boolGridName,
                                             float startRange,
                                             float endRange) {
  assert(grids_.find(floatGridName) != grids_.end());
  assert(grids_[floatGridName].first == VoxelGridType::Float);
  int num_filled = 0;
  addGrid<bool>(boolGridName);
  auto boolGrid = getGrid<bool>(boolGridName);
  auto floatGrid = getGrid<float>(floatGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (floatGrid[i][j][k] >= startRange &&
            floatGrid[i][j][k] <= endRange) {
          boolGrid[i][j][k] = true;
          num_filled++;
        }
      }
    }
  }

  return num_filled;
}

std::vector<Mn::Vector3i> VoxelGrid::fillVoxelSetFromBoolGrid(
    const std::string& boolGridName) {
  std::vector<Mn::Vector3i> voxelSet = std::vector<Mn::Vector3i>();
  assert(grids_.find(boolGridName) != grids_.end());
  assert(grids_[boolGridName].first == VoxelGridType::Bool);
  auto boolGrid = getGrid<bool>(boolGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (boolGrid[i][j][k]) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
  return voxelSet;
}

std::vector<Mn::Vector3i> VoxelGrid::fillVoxelSetFromIntGrid(
    const std::string& intGridName,
    int lb,
    int ub) {
  assert(grids_.find(intGridName) != grids_.end());
  std::vector<Mn::Vector3i> voxelSet = std::vector<Mn::Vector3i>();
  assert(grids_[intGridName].first == VoxelGridType::Int);
  auto intGrid = getGrid<int>(intGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (intGrid[i][j][k] >= lb && intGrid[i][j][k] <= ub) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
  return voxelSet;
}

std::vector<Mn::Vector3i> VoxelGrid::fillVoxelSetFromFloatGrid(
    const std::string& floatGridName,
    float lb,
    float ub) {
  std::vector<Mn::Vector3i> voxelSet = std::vector<Mn::Vector3i>();
  assert(grids_.find(floatGridName) != grids_.end());
  assert(grids_[floatGridName].first == VoxelGridType::Float);
  auto floatGrid = getGrid<float>(floatGridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        if (floatGrid[i][j][k] >= lb && floatGrid[i][j][k] <= ub) {
          voxelSet.push_back(Mn::Vector3i(i, j, k));
        }
      }
    }
  }
  return voxelSet;
}

// 6D SDF - labels each cell as interior (-inf), exterior (+inf), or boundary
void VoxelGrid::generateInteriorExteriorVoxelGrid() {
  // create 6 bool grids
  addGrid<bool>("negXShadow");
  addGrid<bool>("posXShadow");
  addGrid<bool>("negYShadow");
  addGrid<bool>("posYShadow");
  addGrid<bool>("negZShadow");
  addGrid<bool>("posZShadow");
  auto boundaryGrid = getGrid<bool>("Boundary");
  unsigned long dims[3]{static_cast<unsigned long>(m_voxelGridDimensions[0]),
                        static_cast<unsigned long>(m_voxelGridDimensions[1]),
                        static_cast<unsigned long>(m_voxelGridDimensions[2])};
  Corrade::Containers::Array<char> cr_grid{
      Corrade::Containers::ValueInit,
      gridSize() * sizeof(Mn::Math::BoolVector<6>)};
  auto shadowGrid_ =
      Cr::Containers::StridedArrayView<3, Mn::Math::BoolVector<6>>{
          Cr::Containers::arrayCast<Mn::Math::BoolVector<6>>(cr_grid), dims};

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
        hit = (hit || boundaryGrid[ind][j][k]);
        if (hit)
          shadowGrid_[ind][j][k].set(0, true);
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[0]) {
        hit = (hit || boundaryGrid[ind][j][k]);
        if (hit)
          shadowGrid_[ind][j][k].set(1, true);
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
        hit = (hit || boundaryGrid[i][ind][k]);
        if (hit)
          shadowGrid_[i][ind][k].set(2, true);
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[1]) {
        hit = (hit || boundaryGrid[i][ind][k]);
        if (hit)
          shadowGrid_[i][ind][k].set(3, true);
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
        hit = (hit || boundaryGrid[i][j][ind]);
        if (hit)
          shadowGrid_[i][j][ind].set(4, true);
      }
      // fill posX
      hit = false;
      ind = -1;
      while (++ind < m_voxelGridDimensions[2]) {
        hit = (hit || boundaryGrid[i][j][ind]);
        if (hit)
          shadowGrid_[i][j][ind].set(5, true);
      }
    }
  }

  // create int grid
  addGrid<int>(gridName);
  auto intExtGrid = getGrid<int>(gridName);
  bool nX = false, pX = false, nY = false, pY = false, nZ = false, pZ = false;
  // fill in int grid with voting approach
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        if (boundaryGrid[i][j][k]) {
          intExtGrid[i][j][k] = 0;
          continue;
        }
        nX = !shadowGrid_[i][j][k][0];
        pX = !shadowGrid_[i][j][k][1];
        nY = !shadowGrid_[i][j][k][2];
        pY = !shadowGrid_[i][j][k][3];
        nZ = !shadowGrid_[i][j][k][4];
        pZ = !shadowGrid_[i][j][k][5];
        // || ((nX || pX) && (nY || pY) && (nZ || pZ))
        if (((nX && pX) || (nY && pY) || (nZ && pZ)) ||
            ((nX || pX) && (nY || pY) && (nZ || pZ))) {
          // Exterior (+inf)
          intExtGrid[i][j][k] = INT_MAX;
        } else {
          // Interior (-inf)
          intExtGrid[i][j][k] = INT_MIN;
        }
      }
    }
  }
}

// Manhattan distance SDF - starting from the interior exterior voxel grid,
// computes SDF in terms of manhattan distance with double sweep approach
void VoxelGrid::generateManhattanDistanceSDF(const std::string& gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  if (grids_.find("InteriorExterior") == grids_.end()) {
    generateInteriorExteriorVoxelGrid();
  }
  // create new intGrid and copy data from interior/exterior grid
  addGrid<int>(gridName);
  auto intExtGrid = getGrid<int>("InteriorExterior");
  auto sdfGrid = getGrid<int>(gridName);

  Cr::Utility::copy(intExtGrid, sdfGrid);

  // 1st sweep
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        int i_behind = INT_MAX, j_behind = INT_MAX, k_behind = INT_MAX;
        if (isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = abs(std::max(sdfGrid[i - 1][j][k], -INT_MAX));
        }
        if (isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = abs(std::max(sdfGrid[i][j - 1][k], -INT_MAX));
        }
        if (isValidIndex(Mn::Vector3i(i, j, k - 1))) {
          k_behind = abs(std::max(sdfGrid[i][j][k - 1], -INT_MAX));
        }
        int curVal = sdfGrid[i][j][k];
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
                 std::min(abs(std::max(curVal, -INT_MAX)), closest + 1);
        sdfGrid[i][j][k] = curVal;
      }
    }
  }
  // second sweep
  for (int i = m_voxelGridDimensions[0] - 1; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1] - 1; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2] - 1; k >= 0; k--) {
        int curVal = sdfGrid[i][j][k];
        if (curVal == 0)
          continue;
        int i_ahead = INT_MAX, j_ahead = INT_MAX, k_ahead = INT_MAX;
        if (isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = abs(std::max(sdfGrid[i + 1][j][k], -INT_MAX));
        }
        if (isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = abs(std::max(sdfGrid[i][j + 1][k], -INT_MAX));
        }
        if (isValidIndex(Mn::Vector3i(i, j, k + 1))) {
          k_ahead = abs(std::max(sdfGrid[i][j][k + 1], -INT_MAX));
        }

        int closest = INT_MAX - 1;
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
                 std::min(abs(std::max(curVal, -INT_MAX)), closest + 1);
        sdfGrid[i][j][k] = curVal;
      }
    }
  }
}

void VoxelGrid::generateEuclideanDistanceSDF(const std::string& gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  if (grids_.find("InteriorExterior") == grids_.end()) {
    generateInteriorExteriorVoxelGrid();
  }
  // create new vector3Grid and fill data from interior/exterior grid
  addGrid<Mn::Vector3>("ClosestBoundaryCell");
  auto intExtGrid = getGrid<int>("InteriorExterior");
  auto closestCellGrid = getGrid<Mn::Vector3>("ClosestBoundaryCell");

  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        int label = intExtGrid[i][j][k];
        if (label == 0) {
          closestCellGrid[i][j][k] = Mn::Vector3(i, j, k);
        } else {
          // intializing the closest boundary cell to be very far / invalid, so
          // it is ensured to be overwritten in the SDF calculation sweeps.
          closestCellGrid[i][j][k] = Mn::Vector3(m_voxelGridDimensions) * 2;
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
          i_behind = closestCellGrid[i - 1][j][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = closestCellGrid[i][j - 1][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j, k - 1))) {
          k_behind = closestCellGrid[i][j][k - 1];
        }
        Mn::Vector3 coords(i, j, k);

        // get the currently recorded closest boundary distance
        float cur_dist = (closestCellGrid[i][j][k] - coords).length();

        // get the current distances from each point's closest boundary and
        // the current coordinates.
        float i_dist = NAN, j_dist = NAN, k_dist = NAN;
        i_dist = (i_behind - coords).length();
        j_dist = (j_behind - coords).length();
        k_dist = (k_behind - coords).length();

        if (i_dist <= j_dist && i_dist <= k_dist && i_dist <= cur_dist) {
          closestCellGrid[i][j][k] = i_behind;
        } else if (j_dist <= i_dist && j_dist <= k_dist && j_dist <= cur_dist) {
          closestCellGrid[i][j][k] = j_behind;
        } else if (k_dist <= i_dist && k_dist <= j_dist && k_dist <= cur_dist) {
          closestCellGrid[i][j][k] = k_behind;
        }
      }
    }
  }
  // create float grid for distances, will be filled in this sweep.
  addGrid<float>(gridName);
  auto sdfGrid = getGrid<float>(gridName);
  // second sweep
  for (int i = m_voxelGridDimensions[0] - 1; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1] - 1; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2] - 1; k >= 0; k--) {
        Mn::Vector3 i_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    j_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    k_ahead = Mn::Vector3(m_voxelGridDimensions) * 2;
        if (isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = closestCellGrid[i + 1][j][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = closestCellGrid[i][j + 1][k];
        }
        if (isValidIndex(Mn::Vector3i(i, j, k + 1))) {
          k_ahead = closestCellGrid[i][j][k + 1];
        }
        Mn::Vector3 coords(i, j, k);

        // get the currently recorded closest boundary distance
        float cur_dist = (closestCellGrid[i][j][k] - coords).length();
        // get whether the coord is considered interior or exterior
        int intOrExtVal = intExtGrid[i][j][k];
        int intOrExtSign = (intOrExtVal > 0) - (intOrExtVal < 0);

        // get the current distances from each point's closest boundary and
        // the current coordinates.
        float i_dist = NAN, j_dist = NAN, k_dist = NAN;
        i_dist = (i_ahead - coords).length();
        j_dist = (j_ahead - coords).length();
        k_dist = (k_ahead - coords).length();
        if (i_dist <= j_dist && i_dist <= k_dist && i_dist <= cur_dist) {
          closestCellGrid[i][j][k] = i_ahead;
          sdfGrid[i][j][k] = intOrExtSign * i_dist;
        } else if (j_dist <= i_dist && j_dist <= k_dist && j_dist <= cur_dist) {
          closestCellGrid[i][j][k] = j_ahead;
          sdfGrid[i][j][k] = intOrExtSign * j_dist;
        } else if (k_dist <= i_dist && k_dist <= j_dist && k_dist <= cur_dist) {
          closestCellGrid[i][j][k] = k_ahead;
          sdfGrid[i][j][k] = intOrExtSign * k_dist;
        } else {
          sdfGrid[i][j][k] = intOrExtSign * cur_dist;
        }
      }
    }
  }
}

void VoxelGrid::generateDistanceFlowField(const std::string& gridName) {
  // generate the ESDF if not already created
  if (grids_.find("ClosestBoundaryCell") != grids_.end())
    generateEuclideanDistanceSDF();

  addGrid<Mn::Vector3>(gridName);
  auto flowGrid = getGrid<Mn::Vector3>(gridName);
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        flowGrid[i][j][k] = Mn::Vector3(index) -
                            getVoxel<Mn::Vector3>(index, "ClosestBoundaryCell");
      }
    }
  }
}
void VoxelGrid::fillBoolGridNeighborhood(std::vector<bool>& neighbors,
                                         const std::string& gridName,
                                         const Mn::Vector3i& index) {
  for (int i = 0; i < 6; i++) {
    neighbors.push_back(false);
  }
  auto grid = getGrid<bool>(gridName);
  neighbors[0] = isValidIndex(index + Mn::Vector3i(0, 0, 1))
                     ? grid[index[0]][index[1]][index[2] + 1]
                     : false;
  neighbors[1] = isValidIndex(index + Mn::Vector3i(1, 0, 0))
                     ? grid[index[0] + 1][index[1]][index[2]]
                     : false;
  neighbors[2] = isValidIndex(index + Mn::Vector3i(0, 1, 0))
                     ? grid[index[0]][index[1] + 1][index[2]]
                     : false;
  neighbors[3] = isValidIndex(index + Mn::Vector3i(0, 0, -1))
                     ? grid[index[0]][index[1]][index[2] - 1]
                     : false;
  neighbors[4] = isValidIndex(index + Mn::Vector3i(0, -1, 0))
                     ? grid[index[0]][index[1] - 1][index[2]]
                     : false;
  neighbors[5] = isValidIndex(index + Mn::Vector3i(-1, 0, 0))
                     ? grid[index[0] - 1][index[1]][index[2]]
                     : false;
}

void VoxelGrid::addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                         std::vector<Mn::Vector3>& normals,
                                         std::vector<Mn::Color3>& colors,
                                         std::vector<Mn::UnsignedInt>& indices,
                                         const Mn::Vector3i& local_coords,
                                         const std::vector<bool>& neighbors,
                                         const Mn::Color3& color) {
  // Using the data of a cubeSolid to create the voxel cube
  assert(neighbors.size() >= 6);
  Mn::Trade::MeshData cubeData = Mn::Primitives::cubeSolid();

  // add cube to mesh
  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(local_coords);

  unsigned int sz = positions.size();
  auto cubePositions = cubeData.positions3DAsArray();
  auto cubeNormals = cubeData.normalsAsArray();
  auto cubeIndices = cubeData.indices();

  // TODO: Only add neccessary faces (based on neighborhood)
  for (int i = 0; i < 24; i++) {
    Mn::Vector3 vertOffset = cubePositions[i] * m_voxelSize / 2;
    positions.push_back(vertOffset + mid);
    // Set the normals to be weighted such that cubes look slightly curved
    normals.push_back(cubePositions[i].normalized() * 1 / 4 +
                      cubeNormals[i].normalized() * 3 / 4);
    colors.push_back(color);
  }

  for (int i = 0; i < 6; i++) {
    if (!neighbors[i]) {
      for (int j = 0; j < 6; j++) {
        indices.push_back(sz + cubeIndices[i * 6 + j][0]);
      }
    }
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

  // TODO: Consider using more for loops to reduce code
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
  // TODO Collapse into tetrehedron
  // look for a magnum tetrehedron
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

  indices.push_back(sz + 1);
  indices.push_back(sz + 2);
  indices.push_back(sz + 3);

  indices.push_back(sz + 1);
  indices.push_back(sz + 3);
  indices.push_back(sz + 4);
}

void VoxelGrid::generateMeshDataAndMeshGL(const std::string gridName,
                                          std::vector<Mn::UnsignedInt>& indices,
                                          std::vector<Mn::Vector3>& positions,
                                          std::vector<Mn::Vector3>& normals,
                                          std::vector<Mn::Color3>& colors) {
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

void VoxelGrid::generateMesh(const std::string& gridName, bool isVectorField) {
  assert(grids_.find(gridName) != grids_.end());
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
          Mn::Vector3 vec = getVoxel<Mn::Vector3>(local_coords, gridName);
          if (vec != Mn::Vector3(0, 0, 0))
            addVectorToMeshPrimitives(positions, normals, colors, indices,
                                      local_coords, vec);
        } else {
          bool val = getVoxel<bool>(local_coords, gridName);
          if (val) {
            num_filled++;
            std::vector<bool> neighbors{};
            fillBoolGridNeighborhood(neighbors, gridName, local_coords);
            addVoxelToMeshPrimitives(positions, normals, colors, indices,
                                     local_coords, neighbors);
          }
        }
      }
    }
  }

  generateMeshDataAndMeshGL(gridName, indices, positions, normals, colors);
}

}  // namespace geo
}  // namespace esp
