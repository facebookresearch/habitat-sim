// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_VOXEL_GRID_H_
#define ESP_GEO_VOXEL_GRID_H_

#include <vector>

#include "esp/assets/MeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/core/esp.h"
#include "esp/geo/geo.h"

#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Math/Range.h>
#include "esp/gfx/Drawable.h"
#include "esp/gfx/magnum.h"

#ifdef ESP_BUILD_WITH_VHACD
#include "VHACD.h"
#endif

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

class VoxelGrid {
 public:
#ifdef ESP_BUILD_WITH_VHACD
  /**
   * @brief Generates a boundary voxel grid using VHACD's voxelization
   * framework.
   * @param MeshData The mesh that will be voxelized
   * @param resolution The approximate number of voxels in the voxel grid.
   */
  VoxelGrid(const std::unique_ptr<assets::MeshData>& meshData, int resolution);
#endif

  /**
   * @brief Generates an empty voxel grid given some voxel size and voxel
   * dimensions..
   * @param voxelSize The size of a single voxel
   * @param VoxelGridDimensions The dimensions of the voxel grid.
   */
  VoxelGrid(const Mn::Vector3& voxelSize,
            const Mn::Vector3i& voxelGridDimensions);

  /**
   * @brief Loads a Voxel grid in from a file path. If the filepath points to a
   * directory, all .svx files will be loaded. If file path points to a file,
   * this file must represent a boolean voxel grid which will represent the
   * voxel grid's boundary grid.
   * @param filepath Can either be the path to a directory or a single .svx
   * file.
   */
  VoxelGrid(const std::string filepath);

  /**
   * @brief Generates a new empty boolean voxel grid.
   * @param name The key underwhich the grid will be registered and accessed.
   */
  void addBoolGrid(std::string name);

  /**
   * @brief Generates a new empty integer voxel grid.
   * @param name The key underwhich the grid will be registered and accessed.
   */
  void addIntGrid(std::string name);

  /**
   * @brief Generates a new empty float voxel grid.
   * @param name The key underwhich the grid will be registered and accessed.
   */
  void addFloatGrid(std::string name);

  /**
   * @brief Generates a new empty Magnum Vector3 voxel grid.
   * @param name The key underwhich the grid will be registered and accessed.
   */
  void addVector3Grid(std::string name);

  /**
   * @brief Linearizes 3D voxel coordinates to a single value in order to
   * directly access a voxel grid.
   * @param coords The 3D voxel index to be linearized.
   * @return The has value of the voxel index.
   */
  int hashVoxelIndex(const Mn::Vector3i& coords);

  /**
   * @brief Converts a hash value into a 3 dimensional coordinate.
   * @param hash The hash value.
   * @return The voxel index derived from the given hash value.
   */
  Mn::Vector3i reverseHash(const int hash);

  /**
   * @brief Checks to see if a given 3D voxel index is valid and does not go out
   * of bounds.
   * @param coords The voxel index.
   * @return True if the voxel index is valid, false otherwise.
   */
  bool isValidIndex(const Mn::Vector3i& coords) const {
    return coords[0] >= 0 && coords[1] >= 0 && coords[2] >= 0 &&
           coords[0] < m_voxelGridDimensions[0] &&
           coords[1] < m_voxelGridDimensions[1] &&
           coords[2] < m_voxelGridDimensions[2];
  }
  //  --== GETTERS AND SETTERS FOR VOXELS ==--

  /**
   * @brief Retrieves the boolean value at a given index for a specific voxel
   * grid.
   * @param coords The voxel index.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return The boolean value located at the index.
   */
  bool getBoolVoxelByIndex(const Mn::Vector3i& coords,
                           std::string gridName = "boundary");

  /**
   * @brief Sets the boolean value at a given index for a specific voxel grid.
   * @param coords The voxel index.
   * @param val The new value to be stored.
   * @param gridName The key underwhich the desired voxel grid is registered.
   */
  void setBoolVoxelByIndex(const Mn::Vector3i& coords,
                           bool val,
                           std::string gridName = "boundary");

  /**
   * @brief Retrieves the integer value at a given index for a specific voxel
   * grid.
   * @param coords The voxel index.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return The integer value located at the index.
   */
  int getIntVoxelByIndex(const Mn::Vector3i& coords, std::string gridName);

  /**
   * @brief Sets the integer value at a given index for a specific voxel grid.
   * @param coords The voxel index.
   * @param val The new value to be stored.
   * @param gridName The key underwhich the desired voxel grid is registered.
   */
  void setIntVoxelByIndex(const Mn::Vector3i& coords,
                          int val,
                          std::string gridName);

  /**
   * @brief Retrieves the float value at a given index for a specific voxel
   * grid.
   * @param coords The voxel index.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return The float value located at the index.
   */
  float getFloatVoxelByIndex(const Mn::Vector3i& coords, std::string gridName);

  /**
   * @brief Sets the float value at a given index for a specific voxel grid.
   * @param coords The voxel index.
   * @param val The new value to be stored.
   * @param gridName The key underwhich the desired voxel grid is registered.
   */
  void setFloatVoxelByIndex(const Mn::Vector3i& coords,
                            float val,
                            std::string gridName);

  /**
   * @brief Retrieves the Vector3 value at a given index for a specific voxel
   * grid.
   * @param coords The voxel index.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return The Vector3 value located at the index.
   */
  Mn::Vector3 getVector3VoxelByIndex(const Mn::Vector3i& coords,
                                     std::string gridName);

  /**
   * @brief Sets the Vector3 value at a given index for a specific voxel grid.
   * @param coords The voxel index.
   * @param val The new value to be stored.
   * @param gridName The key underwhich the desired voxel grid is registered.
   */
  void setVector3VoxelByIndex(const Mn::Vector3i& coords,
                              Mn::Vector3 val,
                              std::string gridName);

  /**
   * @brief Returns the dimensions of the voxel grid.
   * @return The Vector3i value representing the dimensions.
   */
  Mn::Vector3i getVoxelGridDimensions() { return m_voxelGridDimensions; }

  /**
   * @brief Returns the size of a voxel.
   * @return The Vector3 value representing the size of a voxel.
   */
  Mn::Vector3 getVoxelSize() { return m_voxelSize; }

  /**
   * @brief Returns the bounding box minimum offset used for generating an
   * aligned mesh.
   * @return The Vector3 value representing the offset.
   */
  Mn::Vector3 getOffset() { return m_offset; }

  /**
   * @brief Retrieves the MeshData for a particular voxelGrid. If it does not
   * exist, it will generate the mesh for that grid.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return A shared pointer to the MeshData.
   */
  std::shared_ptr<Mn::Trade::MeshData> getMeshData(
      std::string gridName = "boundary") {
    if (meshDataDict_[gridName] == nullptr)
      generateMesh(gridName);
    return meshDataDict_[gridName];
  }

  /**
   * @brief Retrieves the MeshGL used for rendering for a particular voxelGrid.
   * If it does not exist, it will generate the mesh for that grid.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return A shared pointer to the MeshGL.
   */
  Mn::GL::Mesh& getMeshGL(std::string gridName = "boundary") {
    if (meshDataDict_[gridName] == nullptr)
      generateMesh(gridName);
    return meshGLDict_[gridName];
  }

  /**
   * @brief Converts a voxel index into global coords by applying the offset and
   * multiplying by the real voxel size. Does not apply any transformation made
   * to the object the voxle grid is a part of.
   * @param coords The voxel index to be converted.
   * @return A Vector3 value representing the global coordinates of the voxel
   * index.
   */
  Mn::Vector3 getGlobalCoords(const Mn::Vector3i& coords);

  /**
   * @brief Sets the offset of the voxel grid.
   * @param coords The new offset.
   */
  void setOffset(const Mn::Vector3& coords);

  // --== BUILT-IN VOXELGRID GENERATORS ==--

  /**
   * @brief Generates a boolean voxel grid based on an integer grid. Values in
   * the boolean grid are true if the value for a voxel in the integer grid
   * falls between startRange and endRange.
   * @param intGridName The name of the integer grid.
   * @param boolGridName The name of the newly created boolean grid.
   * @param startRange The lowerbound in the true range.
   * @param endRange The upperbound in the true range.
   * @return An integer representing the number of cells that fell imbetween
   * startRange and endRange and were set to true in the boolean grid.
   */
  int generateBoolGridFromIntGrid(std::string intGridName,
                                  std::string boolGridName,
                                  int startRange,
                                  int endRange);

  /**
   * @brief Generates a boolean voxel grid based on an float grid. Values in the
   * boolean grid are true if the value for a voxel in the float grid falls
   * between startRange and endRange.
   * @param floatGridName The name of the float grid.
   * @param boolGridName The name of the newly created boolean grid.
   * @param startRange The lowerbound in the true range.
   * @param endRange The upperbound in the true range.
   * @return An integer representing the number of cells that fell imbetween
   * startRange and endRange and were set to true in the boolean grid.
   */
  int generateBoolGridFromFloatGrid(std::string floatGridName,
                                    std::string boolGridName,
                                    float startRange,
                                    float endRange);

  /**
   * @brief Generates a boolean voxel grid based on an Vector3 grid. Values in
   * the boolean grid are true if the value for a voxel in the Vector3 grid
   * returns true when evaluated by the function func which is passed in.
   * @param vector3GridName The name of the vector3 grid.
   * @param boolGridName The name of the newly created boolean grid.
   * @param func A pointer to a function used for evaluating whether a voxel
   * value should be true or not.
   * @return An integer representing the number of cells that were set to true
   * in the boolean grid.
   */
  int generateBoolGridFromVector3Grid(std::string vector3GridName,
                                      std::string boolGridName,
                                      bool func(Mn::Vector3));

  /**
   * @brief Fills a vector with voxel indices that meet some criteria.
   * @param[in,out] voxelSet The vector in which the indices will be inserted.
   * @param boolGridName The name of the boolean grid to be processed.
   * @param func A pointer to a function used for evaluating whether a voxel
   * value should be included in the set or not.
   */
  void fillVoxelSetFromBoolGrid(std::vector<Mn::Vector3i>& voxelSet,
                                std::string boolGridName,
                                bool func(bool));

  /**
   * @brief Fills a vector with voxel indices that meet some criteria.
   * @param[in,out] voxelSet The vector in which the indices will be inserted.
   * @param intGridName The name of the int grid to be processed.
   * @param func A pointer to a function used for evaluating whether a voxel
   * value should be included in the set or not.
   */
  void fillVoxelSetFromIntGrid(std::vector<Mn::Vector3i>& voxelSet,
                               std::string intGridName,
                               bool func(int));

  /**
   * @brief Fills a vector with voxel indices that meet some criteria.
   * @param[in,out] voxelSet The vector in which the indices will be inserted.
   * @param floatGridName The name of the float grid to be processed.
   * @param func A pointer to a function used for evaluating whether a voxel
   * value should be included in the set or not.
   */
  void fillVoxelSetFromFloatGrid(std::vector<Mn::Vector3i>& voxelSet,
                                 std::string floatGridName,
                                 bool func(float));

  /**
   * @brief Fills a vector with voxel indices that meet some criteria.
   * @param[in,out] voxelSet The vector in which the indices will be inserted.
   * @param vector3GridName The name of the Vector3 grid to be processed.
   * @param func A pointer to a function used for evaluating whether a voxel
   * value should be included in the set or not.
   */
  void fillVoxelSetFromVector3Grid(std::vector<Mn::Vector3i>& voxelSet,
                                   std::string vector3GridName,
                                   bool func(Mn::Vector3));

  /**
   * @brief Generates an integer grid registered under "InteriorExterior" which
   * stores +inf for exterior cells, -inf for interior cells, and 0 for boundary
   * cells.
   */
  void generateInteriorExteriorVoxelGrid();

  /**
   * @brief Generates a signed distance field using manhattan distance as a
   * distance metric.
   * @param gridName The name underwhich to register the newly created manhattan
   * SDF.
   */
  void generateManhattanDistanceSDF(
      std::string gridName = "MSignedDistanceField");

  /**
   * @brief Generates a signed distance field using euclidean distance as a
   * distance metric. Also created a "ClosestBoundaryCell" vector3 grid which
   * holds the index of the closest boundary grid.
   * @param gridName The name underwhich to register the newly created euclidean
   * SDF.
   */
  void generateEuclideanDistanceSDF(
      std::string gridName = "ESignedDistanceField");

  /**
   * @brief Generates a Vector3 field where each vector of a cell points away
   * from it's closest boundary cell.
   * @param gridName The name underwhich to register the newly created distance
   * flow field.
   */
  void generateDistanceFlowField(std::string gridName = "DistanceFlowField");

  /**
   * @brief Saves a particular grid to a svx file at a specified directory.
   * @param filepath The directory to which the svx file will be saved.
   * @param gridName The name of the voxel grid to be saved.
   */
  bool saveToSVXFile(const std::string& filepath, const std::string& gridName);

  /**
   * @brief Generates both a MeshData and MeshGL for a particular voxelGrid.
   * @param gridName The name of the voxel grid to be converted into a mesh.
   * @param isVectorField If set to true, a vector field mesh will be generated.
   */
  void generateMesh(std::string gridName = "boundary",
                    bool isVectorField = false);

  ESP_SMART_POINTERS(VoxelGrid)
 protected:
  /**
   * @brief Helper function for generate mesh. Adds a cube voxel to a mesh.
   * @param positions A vector of vertices for the mesh.
   * @param colors A vector of per-pertice colors
   * @param indices A vector of indicies for the faces on the mesh.
   * @param local_coords A voxel index specifying the location of the voxel.
   */
  void addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                std::vector<Mn::Vector3>& normals,
                                std::vector<Mn::Color3>& colors,
                                std::vector<Mn::UnsignedInt>& indices,
                                Mn::Vector3i local_coords);

  /**
   * @brief Helper function for generate mesh. Adds a vector voxel to a mesh
   * which points in a specified direction.
   * @param positions A vector of vertices for the mesh.
   * @param colors A vector of per-pertice colors
   * @param indices A vector of indicies for the faces on the mesh.
   * @param local_coords A voxel index specifying the location of the voxel.
   * @param vec The vector to be converted into a mesh.
   */
  void addVectorToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                 std::vector<Mn::Vector3>& normals,
                                 std::vector<Mn::Color3>& colors,
                                 std::vector<Mn::UnsignedInt>& indices,
                                 Mn::Vector3i& local_coords,
                                 Mn::Vector3& vec);

 private:
  // The number of voxels on the x, y, and z dimensions of the grid
  Mn::Vector3i m_voxelGridDimensions;

  // The unit lengths for each voxel dimension
  Mn::Vector3 m_voxelSize;

  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize). m_offset is in
  // world coordinates (not voxel).
  Mn::Vector3 m_offset;

  // The MeshData dictionary of various voxelizations, used for visualization
  std::map<std::string, std::shared_ptr<Mn::Trade::MeshData> > meshDataDict_;

  // The GL Mesh dictionary for visualizing the voxel.
  std::map<std::string, Mn::GL::Mesh> meshGLDict_;

  std::map<std::string, std::shared_ptr<bool> > boolGrids_;

  std::map<std::string, std::shared_ptr<int> > intGrids_;

  std::map<std::string, std::shared_ptr<float> > floatGrids_;

  std::map<std::string, std::shared_ptr<Mn::Vector3> > vector3Grids_;
};

}  // namespace geo
}  // namespace esp

#endif
