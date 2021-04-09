
#include "VoxelUtils.h"
#include <Corrade/Utility/Algorithms.h>

namespace esp {
namespace geo {

void generateInteriorExteriorVoxelGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper) {
  // create 6 bool grids
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto boundaryGrid = v_grid->getGrid<bool>("Boundary");

  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();
  // Create a temporary grid (unregistered) to hold 6 booleans for each cell -
  // each for a specified direction of raycasts
  std::size_t dims[3]{static_cast<std::size_t>(m_voxelGridDimensions[0]),
                      static_cast<std::size_t>(m_voxelGridDimensions[1]),
                      static_cast<std::size_t>(m_voxelGridDimensions[2])};
  Corrade::Containers::Array<char> cr_grid{
      Corrade::Containers::ValueInit,
      v_grid->gridSize() * sizeof(Mn::Math::BoolVector<6>)};
  auto shadowGrid_ =
      Cr::Containers::StridedArrayView<3, Mn::Math::BoolVector<6>>{
          Cr::Containers::arrayCast<Mn::Math::BoolVector<6>>(cr_grid), dims};

  // fill each grid with ray cast
  bool hit = false;
  int ind = 0;
  int increment = 0;
  Mn::Vector3i indices;
  int shadowGridIndex = 0;
  for (int castAxis = 0; castAxis < 3; ++castAxis) {
    int a1 = castAxis ? 0 : 1;
    int a2 = castAxis == 2 ? 1 : 2;
    for (int j = 0; j < m_voxelGridDimensions[a1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[a2]; k++) {
        indices[a1] = j;
        indices[a2] = k;
        // fill from front and back of each 1D slice
        for (int direction = -1; direction < 2; direction += 2) {  //-1 and 1
          hit = false;
          ind = (direction > 0 ? 0 : m_voxelGridDimensions[castAxis] - 1);
          while (ind >= 0 && ind < m_voxelGridDimensions[castAxis]) {
            indices[castAxis] = ind;
            hit = (hit || boundaryGrid[indices[0]][indices[1]][indices[2]]);
            if (hit)
              shadowGrid_[indices[0]][indices[1]][indices[2]].set(
                  castAxis * 2 + (direction < 0 ? 0 : 1), true);
            ind += direction;
          }
        }
      }
    }
  }

  // create int grid
  std::string gridName = "InteriorExterior";
  v_grid->addGrid<int>(gridName);
  auto intExtGrid = v_grid->getGrid<int>(gridName);
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

void generateManhattanDistanceSDF(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& gridName) {
  // check to see if Interior/Exterior grid exists, if not, generate it
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();

  if (!v_grid->gridExists("InteriorExterior")) {
    generateInteriorExteriorVoxelGrid(voxelWrapper);
  }

  // create new intGrid and copy data from interior/exterior grid
  v_grid->addGrid<int>(gridName);
  auto intExtGrid = v_grid->getGrid<int>("InteriorExterior");
  auto sdfGrid = v_grid->getGrid<int>(gridName);

  Cr::Utility::copy(intExtGrid, sdfGrid);

  // 1st sweep
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        int i_behind = INT_MAX, j_behind = INT_MAX, k_behind = INT_MAX;
        if (v_grid->isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = abs(std::max(sdfGrid[i - 1][j][k], -INT_MAX));
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = abs(std::max(sdfGrid[i][j - 1][k], -INT_MAX));
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j, k - 1))) {
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
        if (v_grid->isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = abs(std::max(sdfGrid[i + 1][j][k], -INT_MAX));
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = abs(std::max(sdfGrid[i][j + 1][k], -INT_MAX));
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j, k + 1))) {
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

void generateEuclideanDistanceSDF(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& gridName) {
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();

  // check to see if Interior/Exterior grid exists, if not, generate it
  if (!v_grid->gridExists("InteriorExterior")) {
    generateInteriorExteriorVoxelGrid(voxelWrapper);
  }

  // create new vector3Grid and fill data from interior/exterior grid
  v_grid->addGrid<Mn::Vector3>("ClosestBoundaryCell");
  auto intExtGrid = v_grid->getGrid<int>("InteriorExterior");
  auto closestCellGrid = v_grid->getGrid<Mn::Vector3>("ClosestBoundaryCell");

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
        if (v_grid->isValidIndex(Mn::Vector3i(i - 1, j, k))) {
          i_behind = closestCellGrid[i - 1][j][k];
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j - 1, k))) {
          j_behind = closestCellGrid[i][j - 1][k];
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j, k - 1))) {
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
  v_grid->addGrid<float>(gridName);
  auto sdfGrid = v_grid->getGrid<float>(gridName);
  // second sweep
  for (int i = m_voxelGridDimensions[0] - 1; i >= 0; i--) {
    for (int j = m_voxelGridDimensions[1] - 1; j >= 0; j--) {
      for (int k = m_voxelGridDimensions[2] - 1; k >= 0; k--) {
        Mn::Vector3 i_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    j_ahead = Mn::Vector3(m_voxelGridDimensions) * 2,
                    k_ahead = Mn::Vector3(m_voxelGridDimensions) * 2;
        if (v_grid->isValidIndex(Mn::Vector3i(i + 1, j, k))) {
          i_ahead = closestCellGrid[i + 1][j][k];
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j + 1, k))) {
          j_ahead = closestCellGrid[i][j + 1][k];
        }
        if (v_grid->isValidIndex(Mn::Vector3i(i, j, k + 1))) {
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
  !Mn::Debug();
}

void generateScalarGradientField(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& scalarGridName,
    const std::string& gradientGridName) {
  auto v_grid = voxelWrapper->getVoxelGrid();
  VoxelGridType type = v_grid->getGridType(scalarGridName);
  if (type == VoxelGridType::Int) {
    generateScalarGradientField<int>(voxelWrapper, scalarGridName,
                                     gradientGridName);
  } else if (type == VoxelGridType::Float) {
    generateScalarGradientField<float>(voxelWrapper, scalarGridName,
                                       gradientGridName);
  }
}

std::vector<Mn::Vector3i> getVoxelSetFromBoolGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& boolGridName) {
  std::vector<Mn::Vector3i> voxelSet = std::vector<Mn::Vector3i>();
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();

  assert(v_grid->gridExists(boolGridName));
  auto boolGrid = v_grid->getGrid<bool>(boolGridName);
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

std::vector<Mn::Vector3i> getVoxelSetFromIntGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& intGridName,
    int lb,
    int ub) {
  std::vector<Mn::Vector3i> voxelSet = std::vector<Mn::Vector3i>();
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();

  assert(v_grid->gridExists(intGridName));
  auto intGrid = v_grid->getGrid<int>(intGridName);
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

std::vector<Mn::Vector3i> getVoxelSetFromFloatGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& floatGridName,
    float lb,
    float ub) {
  std::vector<Mn::Vector3i> voxelSet = std::vector<Mn::Vector3i>();
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();

  assert(v_grid->gridExists(floatGridName));
  auto floatGrid = v_grid->getGrid<float>(floatGridName);
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

}  // namespace geo
}  // namespace esp
