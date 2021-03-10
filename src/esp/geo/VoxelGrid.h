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

#include "VHACD.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

class VoxelGrid {
 public:
  // computes a voxel grid for a meshMetaData based on a given resolution (# of
  // voxels) and returns result directly from VHACD O(resolution)
  VoxelGrid(const std::unique_ptr<assets::MeshData>& meshData, int resolution);

  // Creates an empty voxel grid with specified voxel size and dimensions.
  VoxelGrid(const Mn::Vector3& voxelSize,
            const Mn::Vector3i& voxelGridDimensions);

  // Loads a voxel grid from an existing file.
  VoxelGrid(const std::string filepath);

  void addBoolGrid(std::string name);

  void addIntGrid(std::string name);

  void addFloatGrid(std::string name);

  void addVector3Grid(std::string name);

  // (coords.y * x.size * z.size + coords.z * x.size + coords.x)
  int hashVoxelIndex(const Mn::Vector3i& coords);

  Mn::Vector3i reverseHash(const int hash);

  bool isValidIndex(const Mn::Vector3i& coords) const {
    return coords[0] >= 0 && coords[1] >= 0 && coords[2] >= 0 &&
           coords[0] < m_voxelGridDimensions[0] &&
           coords[1] < m_voxelGridDimensions[1] &&
           coords[2] < m_voxelGridDimensions[2];
  }
  //  --== GETTERS AND SETTERS FOR VOXELS ==--

  // Getter and setter for bool value voxel grids
  bool getBoolVoxelByIndex(const Mn::Vector3i& coords,
                           std::string gridName = "boundary");

  void setBoolVoxelByIndex(const Mn::Vector3i& coords,
                           bool val,
                           std::string gridName = "boundary");
  // Getter and setter for int value voxel grids
  int getIntVoxelByIndex(const Mn::Vector3i& coords, std::string gridName);

  void setIntVoxelByIndex(const Mn::Vector3i& coords,
                          int val,
                          std::string gridName);

  // Getter and setter for Float value voxel grids
  float getFloatVoxelByIndex(const Mn::Vector3i& coords, std::string gridName);

  void setFloatVoxelByIndex(const Mn::Vector3i& coords,
                            float val,
                            std::string gridName);

  // Getter and setter for Vector3 value voxel grids
  Mn::Vector3 getVector3VoxelByIndex(const Mn::Vector3i& coords,
                                     std::string gridName);

  void setVector3VoxelByIndex(const Mn::Vector3i& coords,
                              Mn::Vector3 val,
                              std::string gridName);

  Mn::Vector3i getVoxelGridDimensions() { return m_voxelGridDimensions; }
  // The unit lengths for each voxel dimension
  Mn::Vector3 getVoxelSize() { return m_voxelSize; }

  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize)
  Mn::Vector3 getOffset() { return m_offset; }

  std::shared_ptr<Mn::Trade::MeshData> getMeshData(
      std::string gridName = "boundary") {
    if (meshDataDict_[gridName] == nullptr)
      generateMesh(gridName);
    return meshDataDict_[gridName];
  }

  // The GL Mesh for visualizing the voxel.
  Mn::GL::Mesh& getMeshGL(std::string gridName = "boundary") {
    if (meshDataDict_[gridName] == nullptr)
      generateMesh(gridName);
    return meshGLDict_[gridName];
  }

  Mn::Vector3 getGlobalCoords(const Mn::Vector3i& coords);

  // Convert coords to voxel coordinates
  void setOffset(const Mn::Vector3& coords);

  // built in VoxelGrid Generators

  // convert Integer mesh to bool mesh (useful for visualization)
  int generateBoolGridFromIntGrid(std::string intGridName,
                                  std::string boolGridName,
                                  int startRange,
                                  int endRange);

  // convert Float grid to bool grid (useful for visualization, currently only
  // bool grids can produce meshes)
  int generateBoolGridFromFloatGrid(std::string floatGridName,
                                    std::string boolGridName,
                                    float startRange,
                                    float endRange);

  int generateBoolGridFromVector3Grid(std::string floatGridName,
                                      std::string boolGridName,
                                      bool func(Mn::Vector3));

  void fillVoxelSetFromBoolGrid(std::vector<Mn::Vector3i>& voxelSet,
                                std::string boolGridName,
                                bool func(bool));

  void fillVoxelSetFromIntGrid(std::vector<Mn::Vector3i>& voxelSet,
                               std::string intGridName,
                               bool func(int));

  void fillVoxelSetFromFloatGrid(std::vector<Mn::Vector3i>& voxelSet,
                                 std::string floatGridName,
                                 bool func(float));

  void fillVoxelSetFromVector3Grid(std::vector<Mn::Vector3i>& voxelSet,
                                   std::string vector3GridName,
                                   bool func(Mn::Vector3));

  // 6D SDF - labels each cell as interior, exterior, or boundary
  void generateInteriorExteriorVoxelGrid();

  // Manhattan distance SDF - starting from the interior exterior voxel grid,
  // computes SDF with double sweep approach
  void generateManhattanDistanceSDF(
      std::string gridName = "MSignedDistanceField");

  void generateEuclideanDistanceSDF(
      std::string gridName = "ESignedDistanceField");

  void generateDistanceFlowField(std::string gridName = "DistanceFlowField");

  // found file format: svx - https://abfab3d.com/svx-format/
  bool saveToSVXFile(const std::string& filepath, const std::string& filename);

  void addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                std::vector<Mn::UnsignedInt>& indices,
                                Mn::Vector3i local_coords);

  // insert voxel information into a mesh which will be used for visualization.
  void generateMesh(std::string gridName = "boundary");

  ESP_SMART_POINTERS(VoxelGrid)

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

  // The MeshData of the voxelization, used for visualization
  std::shared_ptr<Mn::Trade::MeshData> meshData_;

  std::map<std::string, std::shared_ptr<Mn::Trade::MeshData> > meshDataDict_;

  // The GL Mesh for visualizing the voxel.
  std::unique_ptr<Mn::GL::Mesh> meshGL_;

  std::map<std::string, Mn::GL::Mesh> meshGLDict_;

  // std::map<std::string, bool > vfRecentlyUpdated_;

  /* a pointer to an array of pointers to Voxels.
  Alternatives:
    Octtree: Pros - memory efficient. Voxels can possibly be of varying sizes?
  Cons - can't represent SDF well, complicated
  Bitmask: Pros - maybe very fast collision detection? Cons - Can't represent
  any other info, complicated implementation
*/

  std::map<std::string, std::shared_ptr<bool> > boolGrids_;

  std::map<std::string, std::shared_ptr<int> > intGrids_;

  std::map<std::string, std::shared_ptr<float> > floatGrids_;

  std::map<std::string, std::shared_ptr<Mn::Vector3> > vector3Grids_;

  // std::map<std::string, std::shared_ptr<bool> > boolGrids_;
  // Voxel** m_grid;
};

}  // namespace geo
}  // namespace esp

#endif

// OPEN QUESTIONS
// How will this be consumed in the Simulator (Could make this a property of
// MeshMetaData? Could make a voxelgridDict_ where each VoxelGrid is registered
// under it's corresponding MeshMetaData key in resourceDict_) How to represent
// rotations of a voxel field? If given some rotation, I suppose it just rotates
// around the voxel field center? If this is the case, I need to make sure

// Pseudo code for running in python
// obj_template
// sim.computeVoxelGrid(obj_template)
// sim.sampleForCollision(object1, object2, position of object2) (Find the
// habitat tutorial where this is specified
