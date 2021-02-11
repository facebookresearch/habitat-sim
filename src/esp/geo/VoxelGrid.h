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

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

struct Voxel {
  int m_distance_field;
};

class VoxelGrid {
 public:
  // computes a voxel grid for a meshMetaData based on a given resolution (# of
  // voxels) and returns result directly from VHACD O(resolution)
  VoxelGrid(const assets::MeshMetaData& meshMetaData, int resolution);

  // computes the meshMetaData's bounding box size and creates a Voxel Grid
  // large enough to fit the mesh. O( (bb.size()/voxelSize)^3 )
  VoxelGrid(const assets::MeshMetaData& meshMetaData, vec3f voxelSize);

  // Creates an empty voxel grid with specified voxel size and dimensions.
  VoxelGrid(vec3f voxelSize, vec3f voxelGridDimensions);

  // Gets a voxel pointer based on local coords (coords.y * x.size * z.size +
  // coords.z * x.size + coords.x) O(1) access
  Voxel* getVoxelByLocalCoords(vec3i coords);

  // First convert coords to integer voxel coordinates, then apply globalOffset
  vec3i convertGlobalCoordsToLocal(vec3f coords);

  // Apply globalOffset, multiply each
  vec3i convertLocalCoordsToGlobal(vec3i coords);

  // just converts coords to local coords, then getVoxelByLocalCoords
  Voxel* getVoxelByGlobalCoords(vec3f);

  // iterates through each voxel of smaller VoxelGrid, applies relevant offset,
  // and returns true if the VoxelGrids have a shared filled voxel. (Q: Should
  // this work if the VoxelGrids have different voxelSizes?) O(N^3) <-- Q: can
  // we do better?
  bool checkForCollision(VoxelGrid);

  vec3i getVoxelGridDimensions();
  // The unit lengths for each voxel dimension
  vec3f getVoxelSize();
  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize)
  vec3i getGlobalOffset();

  // Convert coords to voxel coordinates
  void setGlobalOffset(vec3f coords);

 private:
  // The number of voxels on the x, y, and z dimensions of the grid
  vec3i voxelGridDimensions;
  // The unit lengths for each voxel dimension
  vec3f voxelSize;
  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize)
  vec3i globalOffset;

  /* a pointer to an array of pointers to Voxels.
  Alternatives:
    Octtree: Pros - memory efficient. Voxels can possibly be of varying sizes?
  Cons - can't represent SDF well, complicated Bitmask: Pros - maybe very fast
  collision detection? Cons - Can't represent any other info, complicated
  implementation
*/
  Voxel** grid[];
};

}  // namespace geo
}  // namespace esp

#endif

// Pseudo code for running in python
// obj_template
// sim.computeVoxelGrid(obj_template)
// sim.sampleForCollision(object1, object2, position of object2) (Find the
// habitat tutorial where this is specified
