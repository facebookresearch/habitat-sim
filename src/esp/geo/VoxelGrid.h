// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_VOXEL_GRID_H_
#define ESP_GEO_VOXEL_GRID_H_

#include <vector>

#include "esp/assets/ResourceManager.h"
#include "esp/core/esp.h"

#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Math/Range.h>
#include "esp/gfx/magnum.h"

#include "VHACD.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

struct Voxel {
  float m_distance;
  bool is_filled = false;
  Voxel(bool filled) { is_filled = filled; }
};

class VoxelGrid {
 public:
  // default constructor
  VoxelGrid();
  // computes a voxel grid for a meshMetaData based on a given resolution (# of
  // voxels) and returns result directly from VHACD O(resolution)
  VoxelGrid(const Mn::Trade::MeshData& meshData, int resolution);

  // computes the meshMetaData's bounding box size and creates a Voxel Grid
  // large enough to fit the mesh. O( (bb.size()/voxelSize)^3 )
  VoxelGrid(const assets::MeshMetaData& meshMetaData,
            const Mn::Vector3 voxelSize);

  // Creates an empty voxel grid with specified voxel size and dimensions.
  VoxelGrid(const Mn::Vector3& voxelSize,
            const Mn::Vector3i& voxelGridDimensions);

  // Loads a voxel grid from an existing file.
  VoxelGrid(const std::string filepath);

  // (coords.y * x.size * z.size + coords.z * x.size + coords.x)
  int hashVoxelIndex(const Mn::Vector3i& coords);

  // Gets a voxel pointer based on local coords (coords.y * x.size * z.size +
  // coords.z * x.size + coords.x) O(1) access. Returns nullptr if invalid
  // coordinates.
  Voxel* getVoxelByIndex(const Mn::Vector3i& coords);

  // First convert coords to integer voxel coordinates, then apply globalOffset
  // * rotation
  Mn::Vector3i getVoxelIndex(const Mn::Vector3& coords);

  // multiply coords by m_voxelSize, apply globalOffset
  Mn::Vector3 getGlobalCoords(const Mn::Vector3i& coords);

  // just converts coords to local coords, then getVoxelByLocalCoords. Returns
  // nullptr if invalid coordinates.
  Voxel* getVoxelByGlobalCoords(const Mn::Vector3& coords);

  // Sets voxel by local voxel index.
  void setVoxelByIndex(const Mn::Vector3i& coords, Voxel* voxel);

  // iterates through each voxel of smaller VoxelGrid, applies relevant offset,
  // and returns true if the VoxelGrids have a shared filled voxel. (Q: Should
  // this work if the VoxelGrids have different voxelSizes?) O(N^3) <-- Q: can
  // we do better?
  bool checkForCollision(const VoxelGrid& v_grid);

  Mn::Vector3i getVoxelGridDimensions();
  // The unit lengths for each voxel dimension
  Mn::Vector3 getVoxelSize();

  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize)
  Mn::Vector3i getGlobalOffset();

  // Convert coords to voxel coordinates
  void setGlobalOffset(const Mn::Vector3& coords);

  // found file format: svx - https://abfab3d.com/svx-format/
  bool saveToSVXFile(const std::string& filepath, const std::string& filename);

  void addVoxelToMeshPrimitives(std::vector<Mn::Vector3>& positions,
                                std::vector<Mn::UnsignedShort>& indices,
                                Mn::Vector3i local_coords);

  // insert voxel information into a mesh.
  void fillVoxelMeshData(Cr::Containers::Optional<Mn::Trade::MeshData>& mesh);

 private:
  // The number of voxels on the x, y, and z dimensions of the grid
  Mn::Vector3i m_voxelGridDimensions;
  // The unit lengths for each voxel dimension
  Mn::Vector3 m_voxelSize;

  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize). m_globalOffset is in
  // world coordinates (not voxel).
  Mn::Vector3 m_globalOffset;

  /* a pointer to an array of pointers to Voxels.
  Alternatives:
    Octtree: Pros - memory efficient. Voxels can possibly be of varying sizes?
  Cons - can't represent SDF well, complicated
  Bitmask: Pros - maybe very fast collision detection? Cons - Can't represent
  any other info, complicated implementation
*/
  Voxel** m_grid;
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
