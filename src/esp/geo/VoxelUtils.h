
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_VOXEL_UTILITY_H_
#define ESP_GEO_VOXEL_UTILITY_H_

#include "VoxelWrapper.h"
#include "esp/core/esp.h"
#include "esp/geo/geo.h"

namespace esp {
namespace geo {

/**
 * @brief Generates an integer grid registered under "InteriorExterior" which
 * stores +inf for exterior cells, -inf for interior cells, and 0 for Boundary
 * cells.
 * @param voxelWrapper The voxelization for the SDF.
 */
void generateInteriorExteriorVoxelGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper);

/**
 * @brief Generates a signed distance field using manhattan distance as a
 * distance metric.
 * @param voxelWrapper The voxelization for the SDF.
 * @param gridName The name underwhich to register the newly created manhattan
 * SDF.
 */
void generateManhattanDistanceSDF(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& gridName);

/**
 * @brief Generates a signed distance field using euclidean distance as a
 * distance metric. Also created a "ClosestBoundaryCell" vector3 grid which
 * holds the index of the closest Boundary grid.
 * @param voxelWrapper The voxelization for the SDF.
 * @param gridName The name underwhich to register the newly created euclidean
 * SDF.
 */
void generateEuclideanDistanceSDF(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& gridName);

/**
 * @brief Generates a Vector3 field where each vector of a cell is the
 * gradient of how the scalar field changes.
 * @param voxelWrapper The voxelization for the gradient field.
 * @param scalarGridName The name of the scalar field for which the gradient
 * will be generated.
 * @param gradientGridName The name underwhich to register the newly created
 * scalar gradient field.
 */
void generateScalarGradientField(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& scalarGridName,
    const std::string& gradientGridName);

/**
 * @brief Generates a Vector3 field where each vector of a cell is the
 * gradient of how the scalar field changes.
 * @param voxelWrapper The voxelization for the gradient field.
 * @param scalarGridName The name of the scalar field for which the gradient
 * will be generated.
 * @param gradientGridName The name underwhich to register the newly created
 * scalar gradient field.
 */
template <typename T>
void generateScalarGradientField(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& scalarGridName,
    const std::string& gradientGridName) {
  auto v_grid = voxelWrapper->getVoxelGrid();
  auto m_voxelGridDimensions = v_grid->getVoxelGridDimensions();

  // generate the ESDF if not already created
  assert(v_grid->gridExists(scalarGridName));

  v_grid->addGrid<Mn::Vector3>(gradientGridName);
  auto gradientGrid = v_grid->getGrid<Mn::Vector3>(gradientGridName);
  auto scalarGrid = v_grid->getGrid<T>(scalarGridName);
  std::vector<Mn::Vector3i> neighbors{
      Mn::Vector3i(1, 0, 0),  Mn::Vector3i(-1, 0, 0), Mn::Vector3i(0, 1, 0),
      Mn::Vector3i(0, -1, 0), Mn::Vector3i(0, 0, 1),  Mn::Vector3i(0, 0, -1)};
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i index = Mn::Vector3i(i, j, k);
        Mn::Vector3 result(0, 0, 0);
        int validVectors = 0;
        for (auto neighbor : neighbors) {
          if (v_grid->isValidIndex(neighbor + index)) {
            float diff =
                scalarGrid[i + neighbor[0]][j + neighbor[1]][k + neighbor[2]] -
                scalarGrid[i][j][k];
            result += Mn::Vector3(neighbor) * diff;
            validVectors++;
          }
        }

        gradientGrid[i][j][k] = result / validVectors;
      }
    }
  }
}

/**
 * @brief Returns a vector of all filled/true voxels.
 * @param voxelWrapper The voxelization.
 * @param boolGridName The name of the boolean grid to be processed.
 * @return A vector of Vector3i's
 */
std::vector<Mn::Vector3i> getVoxelSetFromBoolGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& boolGridName);

/**
 * @brief Fills a vector with voxel indices that meet some criteria.
 * @param voxelWrapper The voxelization.
 * @param intGridName The name of the int grid to be processed.
 * @param lb The lower bound of voxel values to include.
 * @param ub The uppper bound of voxel values to include
 * @return A vector of Vector3i's
 */
std::vector<Mn::Vector3i> getVoxelSetFromIntGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& intGridName,
    int lb,
    int ub);

/**
 * @brief Fills a vector with voxel indices that meet some criteria.
 * @param voxelWrapper The voxelization.
 * @param floatGridName The name of the float grid to be processed.
 * @param lb The lower bound of voxel values to include.
 * @param ub The uppper bound of voxel values to include
 * @return A vector of Vector3i's
 */
std::vector<Mn::Vector3i> getVoxelSetFromFloatGrid(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& floatGridName,
    float lb,
    float ub);

}  // namespace geo
}  // namespace esp
#endif
