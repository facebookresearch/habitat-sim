
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
 * gradient of how the SDF changes.
 * @param voxelWrapper The voxelization for the gradient field.
 * @param gridName The name underwhich to register the newly created distance
 * gradient field.
 */
void generateDistanceGradientField(
    std::shared_ptr<esp::geo::VoxelWrapper>& voxelWrapper,
    const std::string& gridName);

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
 * @brief Fills a vector with voxel indices that contain a value imbetween the
 * specified lower_bound and upper_bound.
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
 * @brief Fills a vector with voxel indices that contain a value imbetween the
 * specified lower_bound and upper_bound.
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
