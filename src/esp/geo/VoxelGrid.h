// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_VOXEL_GRID_H_
#define ESP_GEO_VOXEL_GRID_H_

#include <vector>

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Math/Range.h>
#include <Magnum/PixelFormat.h>

#include "esp/assets/MeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/magnum.h"

#ifdef ESP_BUILD_WITH_VHACD
#include "VHACD.h"
#endif

namespace esp {
namespace geo {

enum class VoxelGridType { Bool, Int, Float, Vector3 };

// Used for generating voxel meshes
struct VoxelVertex {
  Mn::Vector3 position;
  Mn::Vector3 normal;
  Mn::Vector3 color;
};

class VoxelGrid {
  struct GridEntry {
    VoxelGridType type;
    Corrade::Containers::Array<char> data;
    Corrade::Containers::StridedArrayView3D<void> view;
  };

 public:
#ifdef ESP_BUILD_WITH_VHACD
  /**
   * @brief Generates a Boundary voxel grid using VHACD's voxelization
   * framework.
   * @param meshData The mesh that will be voxelized
   * @param renderAssetHandle The handle for the render asset.
   * @param resolution The approximate number of voxels in the voxel grid.
   */
  VoxelGrid(const assets::MeshData& meshData,
            const std::string& renderAssetHandle,
            int resolution);
#endif

  /**
   * @brief Generates an empty voxel grid given some voxel size and voxel
   * dimensions..
   * @param voxelSize The size of a single voxel
   * @param voxelGridDimensions The dimensions of the voxel grid.
   */
  VoxelGrid(const Magnum::Vector3& voxelSize,
            const Magnum::Vector3i& voxelGridDimensions);

  /**
   * @brief Gets the enumerated type of a particular voxel grid type.
   * @return The enumerated type.
   */
  template <typename T>
  VoxelGridType voxelGridTypeFor();

  /**
   * @brief Returns the type of the specified voxel grid.
   * @param gridName The name of the grid.
   * @return The enumerated type.
   *
   */
  VoxelGridType getGridType(const std::string& gridName) {
    assert(grids_.find(gridName) != grids_.end());
    return grids_[gridName].type;
  }

  /**
   * @brief Generates a new empty voxel grid of a specified type.
   * @param gridName The key under which the grid will be registered and
   * accessed.
   */
  template <typename T>
  void addGrid(const std::string& gridName) {
    VoxelGridType type = voxelGridTypeFor<T>();

    std::size_t dims[3]{static_cast<std::size_t>(m_voxelGridDimensions[0]),
                        static_cast<std::size_t>(m_voxelGridDimensions[1]),
                        static_cast<std::size_t>(m_voxelGridDimensions[2])};

    Corrade::Containers::StridedDimensions<3, std::ptrdiff_t> strides{
        static_cast<std::ptrdiff_t>(m_voxelGridDimensions[2] *
                                    m_voxelGridDimensions[1] * sizeof(T)),
        static_cast<std::ptrdiff_t>(m_voxelGridDimensions[2] * sizeof(T)),
        static_cast<std::ptrdiff_t>(sizeof(T))};

    if (grids_.find(gridName) != grids_.end()) {
      // grid exists, simply overwrite
      ESP_DEBUG() << gridName << "exists, overwriting.";

      grids_[gridName].data = Corrade::Containers::Array<char>(
          Corrade::ValueInit, gridSize() * sizeof(T));

      Corrade::Containers::StridedArrayView3D<void> view{grids_[gridName].data,
                                                         dims, strides};
      grids_[gridName].view = view;
      grids_[gridName].type = type;
      return;
    }

    GridEntry new_grid;
    new_grid.data = Corrade::Containers::Array<char>(Corrade::ValueInit,
                                                     gridSize() * sizeof(T));

    Corrade::Containers::StridedArrayView3D<void> view{new_grid.data, dims,
                                                       strides};

    new_grid.view = view;
    new_grid.type = type;
    grids_.insert(std::make_pair(gridName, std::move(new_grid)));
  }

  /**
   * @brief Returns a list of existing voxel grids and their types.
   * @return A vector of pairs, where the first element is the voxel grid's
   * name, and the second element is the string of the object's type.
   */
  std::vector<std::pair<std::string, VoxelGridType>> getExistingGrids();

  /**
   * @brief Returns a bool true if a grid exists, false otherwise.
   * @param gridName The name of the grid.
   * @return A bool representing whether or not the specified grid exists.
   */
  bool gridExists(const std::string& gridName) {
    return grids_.find(gridName) != grids_.end();
  }

  /**
   * @brief Removes a grid and frees up memory.
   * @param gridName The name of the grid to be removed.
   */
  void removeGrid(const std::string& gridName) {
    assert(grids_.find(gridName) != grids_.end());
    grids_.erase(gridName);
  }

  /**
   * @brief Returns a StridedArrayView3D of a grid for easy index access and
   * manipulation.
   * @param gridName The name of the grid to be retrieved.
   * @return A StridedArrayView3D of the specified grid.
   */
  template <typename T>
  Corrade::Containers::StridedArrayView3D<T> getGrid(
      const std::string& gridName) {
    VoxelGridType type = voxelGridTypeFor<T>();
    CORRADE_ASSERT(grids_[gridName].type == type,
                   "VoxelGrid::getGrid(\"" + gridName +
                       "\") - Error: incorrect grid type cast requested.",
                   {});
    return Corrade::Containers::arrayCast<T>(grids_[gridName].view);
  }

  /**
   * @brief Checks to see if a given 3D voxel index is valid and does not go out
   * of bounds.
   * @param coords The voxel index.
   * @return True if the voxel index is valid, false otherwise.
   */
  bool isValidIndex(const Mn::Vector3i& coords) const {
    return bool((coords >= Mn::Vector3i()).all() &&
                (coords < m_voxelGridDimensions).all());
  }

  //  --== GETTERS AND SETTERS FOR VOXELS ==--

  /**
   * @brief Sets a voxel at a specified index for a specified grid to a value.
   * @param index The index of the voxel
   * @param gridName The voxel grid.
   * @param value The new value.
   */
  template <typename T>
  void setVoxel(const Magnum::Vector3i& index,
                const std::string& gridName,
                const T& value) {
    Corrade::Containers::StridedArrayView3D<T> arrayView3D =
        getGrid<T>(gridName);
    arrayView3D[index[0]][index[1]][index[2]] = value;
  }

  /**
   * @brief Retrieves the voxel value from a grid of a specified type (bool,
   * int, float, Magnum::Vector3).
   * @param index The index of the voxel
   * @param gridName The voxel grid.
   * @return The value from the specified voxel grid.
   */
  template <typename T>
  T getVoxel(const Magnum::Vector3i& index, const std::string& gridName) {
    Corrade::Containers::StridedArrayView3D<T> arrayView3D =
        getGrid<T>(gridName);
    return arrayView3D[index[0]][index[1]][index[2]];
  }

  /**
   * @brief Returns the dimensions of the voxel grid.
   * @return The Vector3i value representing the dimensions.
   */
  Magnum::Vector3i getVoxelGridDimensions() { return m_voxelGridDimensions; }

  /**
   * @brief Returns the size of a voxel.
   * @return The Vector3 value representing the size of a voxel.
   */
  Magnum::Vector3 getVoxelSize() { return m_voxelSize; }

  /**
   * @brief Gets the length of the voxel grid.
   * @return The length of the 1 dimensional array voxel grid.
   */
  int gridSize() {
    return m_voxelGridDimensions[0] * m_voxelGridDimensions[1] *
           m_voxelGridDimensions[2];
  }

  /**
   * @brief Returns the bounding box minimum offset used for generating an
   * aligned mesh.
   * @return The Vector3 value representing the offset.
   */
  Magnum::Vector3 getOffset() { return m_offset; }

  /**
   * @brief Returns the bounding box maximum offset used for generating an
   * aligned mesh.
   * @return The Vector3 value representing the offset.
   */
  Magnum::Vector3 getMaxOffset() { return m_BBMaxOffset; }

  /**
   * @brief Retrieves the MeshData for a particular voxelGrid. If it does not
   * exist, it will generate the mesh for that grid.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return A shared pointer to the MeshData.
   */
  std::shared_ptr<Magnum::Trade::MeshData> getMeshData(
      const std::string& gridName = "Boundary");

  /**
   * @brief Retrieves the MeshGL used for rendering for a particular voxelGrid.
   * If it does not exist, it will generate the mesh for that grid.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return A reference to the MeshGL.
   */
  Magnum::GL::Mesh& getMeshGL(const std::string& gridName = "Boundary");

  /**
   * @brief Converts a voxel index into global coords by applying the offset and
   * multiplying by the real voxel size. Does not apply any transformation made
   * to the object the voxle grid is a part of.
   * @param coords The voxel index to be converted.
   * @return A Vector3 value representing the global coordinates of the voxel
   * index.
   */
  Magnum::Vector3 getGlobalCoords(const Magnum::Vector3i& coords);

  /**
   * @brief Sets the offset of the voxel grid.
   * @param coords The new offset.
   */
  void setOffset(const Magnum::Vector3& coords);

  /**
   * @brief Generates both a MeshData and MeshGL for a particular voxelGrid.
   * @param gridName The name of the voxel grid to be converted into a mesh.
   */
  void generateMesh(const std::string& gridName = "Boundary");

  /**
   * @brief Generates a colored slice of a mesh.
   * @param gridName The name of the voxel grid to be converted into a mesh
   * slice.
   * @param ind The index value the x axis for the slicing plane.
   * @param minVal The minimum value of the grid. Used for determining heatmap
   * colors.
   * @param maxVal The maximum value of the grid. Used for determining heatmap
   * colors.
   */
  template <typename T>
  void generateSliceMesh(const std::string& gridName = "Boundary",
                         int ind = 0,
                         T minVal = 0,
                         T maxVal = 1) {
    assert(grids_.find(gridName) != grids_.end());
    assert(minVal != maxVal);
    Corrade::Containers::Array<VoxelVertex> vertices;
    Corrade::Containers::Array<Mn::UnsignedInt> indices;
    Corrade::Containers::StridedArrayView3D<T> grid = getGrid<T>(gridName);

    // iterate through each voxel grid cell
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        T val = clamp(grid[ind][j][k], minVal, maxVal);
        val -= minVal;
        float colorVal = float(val) / float(maxVal - minVal);
        Magnum::Vector3i local_coords(ind, j, k);
        Magnum::Color3 col = Magnum::Color3(1 - colorVal, colorVal, 0);
        std::vector<bool> neighbors{false, false, false, false, false, false};
        addVoxelToMeshPrimitives(vertices, indices, local_coords, neighbors,
                                 col);
      }
    }
    generateMeshDataAndMeshGL(gridName, vertices, indices);
  }

  ESP_SMART_POINTERS(VoxelGrid)
 protected:
  /**
   * @brief Fills vector neighbors with 6 booleans representing the top (y+1),
   * bottom (x+1), right (y+1), left (y-1), back (z-1) and front (x-1)
   * neighboring voxel's status.
   * @param [in] neighbors The vector of booleans to be filled.
   * @param gridName The name of the boolean grid to be checked.
   * @param index The index of the voxel.
   */
  void fillBoolGridNeighborhood(std::vector<bool>& neighbors,
                                const std::string& gridName,
                                const Magnum::Vector3i& index);

  /**
   * @brief Generates the Magnum MeshData and MeshGL given indices, positions,
   * normals, and colors.
   * @param gridName The grid name corresponding to the mesh's grid.
   * @param vertexData A Corrade Array of VoxelVertex which each contain a
   * vertex's position, normal, and color
   * @param indexData A Corrade Array of indicies for the faces on the mesh.
   */
  void generateMeshDataAndMeshGL(
      const std::string& gridName,
      Corrade::Containers::Array<VoxelVertex>& vertexData,
      Corrade::Containers::Array<Mn::UnsignedInt>&
          indexData  // note the &&s (!)
  );

  /**
   * @brief Helper function for generate mesh. Adds a cube voxel to a mesh.
   * @param vertexData A Corrade Array of VoxelVertex which each contain a
   * vertex's position, normal, and color
   * @param indexData A Corrade Array of indicies for the faces on the mesh.
   * @param localCoords A voxel index specifying the location of the voxel.
   * @param neighbors A boolean with 6 booleans representing whether the
   * voxel on the top (y+1), bottom (x+1), right (y+1), left (y-1), back (z-1)
   * and front (x-1) are filled.
   * @param color A Magnum::Color3 object specifying the color for a particular
   * voxel. Used primarily for generating the heatmap slices.
   */
  void addVoxelToMeshPrimitives(
      Corrade::Containers::Array<VoxelVertex>& vertexData,
      Corrade::Containers::Array<Mn::UnsignedInt>& indexData,
      const Magnum::Vector3i& localCoords,
      const std::vector<bool>& neighbors,
      const Magnum::Color3& color = Magnum::Color3(.4, .8, 1));

  /**
   * @brief Helper function for generate mesh. Adds a vector voxel to a mesh
   * which points in a specified direction.
   * @param vertexData A Corrade Array of VoxelVertex which each contain a
   * vertex's position, normal, and color
   * @param indexData A Corrade Array of indicies for the faces on the mesh.
   * @param localCoords A voxel index specifying the location of the voxel.
   * @param vec The vector to be converted into a mesh.
   */
  void addVectorToMeshPrimitives(
      Corrade::Containers::Array<VoxelVertex>& vertexData,
      Corrade::Containers::Array<Mn::UnsignedInt>& indexData,
      const Magnum::Vector3i& localCoords,
      const Magnum::Vector3& vec);

 private:
  // The number of voxels on the x, y, and z dimensions of the grid
  Magnum::Vector3i m_voxelGridDimensions;

  // The unit lengths for each voxel dimension
  Magnum::Vector3 m_voxelSize;

  // The relative positioning of the voxel grid to the simulation (May not
  // need). VoxelGrid corner is anchored to the world origin, so grid[0] is at
  // global position VoxelSize/2 + offset.dot(VoxelSize). m_offset is in
  // world coordinates (not voxel).
  Magnum::Vector3 m_offset;

  Magnum::Vector3 m_BBMaxOffset;

  // The underlying render asset handle the asset is tied to
  std::string m_renderAssetHandle;

  // The MeshData dictionary of various voxelizations, used for visualization
  std::map<std::string, std::shared_ptr<Magnum::Trade::MeshData>> meshDataDict_;

  // The GL Mesh dictionary for visualizing the voxel.
  std::map<std::string, Magnum::GL::Mesh> meshGLDict_;

  std::map<std::string, GridEntry> grids_;
};

}  // namespace geo
}  // namespace esp

#endif
