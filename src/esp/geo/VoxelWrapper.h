#ifndef ESP_GEO_VOXEL_WRAPPER_H_
#define ESP_GEO_VOXEL_WRAPPER_H_

#include "VoxelGrid.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/DrawableGroup.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace geo {

/**
 * @brief When an object is voxelized, it is given a VoxelWrapper which points
 * to an underlying VoxelGrid and the scene node of the object. This class
 * handles object-specific functionality. Multiple objects will each have there
 * own VoxelWrapper, but each of those VoxelWrappers could share a VoxelGrid if
 * they have the same underlying RenderAssetHandle and Voxel resolution.
 */
class VoxelWrapper {
 private:
  esp::scene::SceneNode* SceneNode;
  std::shared_ptr<VoxelGrid> voxelGrid;

 public:
#ifdef ESP_BUILD_WITH_VHACD
  /**
   * @brief Generates (using VHACD's voxelization functionality) or retrieves a
   * voxelization of a render asset mesh depending on whether it exists or not.
   * @param renderAssetHandle The handle for the render asset to which the voxel
   * grid corresponds.
   * @param sceneNode The scene node the voxel wrapper will be pointing to.
   * @param resourceManager Used for retrieving or registering the voxel grid.
   * @param resolution The approximate number of voxels for the voxelization.
   */
  VoxelWrapper(const std::string& renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager,
               int resolution);
#endif

  /**
   * @brief Generates a voxelization with a specified size and dimensions. The
   * voxelization is corner aligned with the object's cumulative bounding box's
   * corner.
   * @param renderAssetHandle The handle for the render asset to which the voxel
   * grid corresponds.
   * @param sceneNode The scene node the voxel wrapper will be pointing to.
   * @param resourceManager Used for registering the voxel grid.
   * @param voxelSize The size of an individual voxel cell.
   * @param voxelDimensions The dimensions of the voxel grid.
   */
  VoxelWrapper(const std::string& renderAssetHandle,
               esp::scene::SceneNode* sceneNode,
               esp::assets::ResourceManager& resourceManager,
               Mn::Vector3& voxelSize,
               Mn::Vector3i& voxelDimensions);

  /**
   * @brief Converts a global coordinate into voxel coordinates.
   * @param coords The global coordinate.
   * @return The voxel index corresponding to the global coordinate.
   */
  Mn::Vector3i getVoxelIndexFromGlobalCoords(const Mn::Vector3& coords);

  /**
   * @brief Converts a voxel coordinate into the global coordinates of the
   * middle of the specified voxel cell.
   * @param index The voxel index.
   * @return The global coordinate of the middle of the specified voxel cell.
   */
  Mn::Vector3 getGlobalCoordsFromVoxelIndex(const Mn::Vector3i& index);

  /**
   * @brief Returns the underlying voxel grid for direct manipulation.
   * @return The underlying voxel grid.
   */
  std::shared_ptr<VoxelGrid> getVoxelGrid() { return voxelGrid; }

  /**
   * @brief Returns a list of existing voxel grids and their types.
   * @return A vector of pairs, where the first element is the voxel grid's
   * name, and the second element is the string of the object's type.
   */
  std::vector<std::pair<std::string, esp::geo::VoxelGridType>>
  getExistingGrids() {
    return voxelGrid->getExistingGrids();
  }

  /**
   * @brief Returns a bool true if a grid exists, false otherwise.
   * @param gridName The name of the grid.
   * @return A bool representing whether or not the specified grid exists.
   */
  bool gridExists(const std::string& gridName) {
    return voxelGrid->gridExists(gridName);
  }

  // --== FUNCTIONS FROM VOXELGRID ==--
  /**
   * @brief Generates a new empty voxel grid of a specified type.
   * @param gridName The key underwhich the grid will be registered and
   * accessed.
   * @return a StridedArrayView3D for the newly created grid.
   */
  template <typename T>
  void addGrid(const std::string& gridName) {
    voxelGrid->addGrid<T>(gridName);
  }

  /**
   * @brief Removes a grid and frees up memory.
   * @param gridName The name of the grid to be removed.
   */
  void removeGrid(const std::string& gridName) {
    voxelGrid->removeGrid(gridName);
  }

  /**
   * @brief Returns a StridedArrayView3D of a grid for easy index access and
   * manipulation.
   * @param gridName The name of the grid to be removed.
   * @return A StridedArrayView3D of the specified grid.
   */
  template <typename T>
  Cr::Containers::StridedArrayView<3, T> getGrid(const std::string& gridName) {
    return voxelGrid->getGrid<T>(gridName);
  }

  //  --== GETTERS AND SETTERS FOR VOXELS ==--

  /**
   * @brief Sets a voxel at a specified index for a specified grid to a value.
   * @param index The index of the voxel
   * @param gridName The voxel grid.
   * @param value The new value.
   */
  template <typename T>
  void setVoxel(const Mn::Vector3i& index,
                const std::string& gridName,
                const T& value) {
    auto arrayView3D = getGrid<T>(gridName);
    arrayView3D[index[0]][index[1]][index[2]] = value;
  }

  /**
   * @brief Retrieves the voxel value from a grid of a specified type (bool,
   * int, float, Mn::Vector3).
   * @param index The index of the voxel
   * @param gridName The voxel grid.
   * @return The value from the specified voxel grid.
   */
  template <typename T>
  T getVoxel(const Mn::Vector3i& index, const std::string& gridName) {
    auto arrayView3D = getGrid<T>(gridName);
    return arrayView3D[index[0]][index[1]][index[2]];
  }

  /**
   * @brief Returns the dimensions of the voxel grid.
   * @return The Vector3i value representing the dimensions.
   */
  Mn::Vector3i getVoxelGridDimensions() {
    return voxelGrid->getVoxelGridDimensions();
  }

  /**
   * @brief Returns the size of a voxel.
   * @return The Vector3 value representing the size of a voxel.
   */
  Mn::Vector3 getVoxelSize() { return voxelGrid->getVoxelSize(); }

  /**
   * @brief Returns the bounding box minimum offset used for generating an
   * aligned mesh.
   * @return The Vector3 value representing the offset.
   */
  Mn::Vector3 getOffset() { return voxelGrid->getOffset(); }

  /**
   * @brief Returns the bounding box maximum offset used for generating an
   * aligned mesh.
   * @return The Vector3 value representing the offset.
   */
  Mn::Vector3 getMaxOffset() { return voxelGrid->getMaxOffset(); }

  /**
   * @brief Retrieves the MeshData for a particular voxelGrid. If it does not
   * exist, it will generate the mesh for that grid.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return A shared pointer to the MeshData.
   */
  std::shared_ptr<Mn::Trade::MeshData> getMeshData(
      const std::string& gridName = "Boundary") {
    return voxelGrid->getMeshData(gridName);
  }

  /**
   * @brief Retrieves the MeshGL used for rendering for a particular voxelGrid.
   * If it does not exist, it will generate the mesh for that grid.
   * @param gridName The key underwhich the desired voxel grid is registered.
   * @return A reference to the MeshGL.
   */
  Mn::GL::Mesh& getMeshGL(const std::string& gridName = "Boundary") {
    return voxelGrid->getMeshGL(gridName);
  }

  /**
   * @brief Sets the SceneNode of the voxel grid.
   * @param sceneNode_ The new sceneNode.
   */
  void setSceneNode(esp::scene::SceneNode* sceneNode_) {
    SceneNode = sceneNode_;
  }

  /**
   * @brief Sets the offset of the voxel grid.
   * @param coords The new offset.
   */
  void setOffset(const Mn::Vector3& coords) { voxelGrid->setOffset(coords); }

  /**
   * @brief Generates both a MeshData and MeshGL for a particular voxelGrid. Can
   * be a Bool grid or a Vector3 grid.
   * @param gridName The name of the voxel grid to be converted into a mesh.
   */
  void generateMesh(const std::string& gridName = "Boundary") {
    voxelGrid->generateMesh(gridName);
  }

  /**
   * @brief Generates a colored slice of a mesh.
   * @param gridName The name of the voxel grid to be converted into a mesh
   * slice.
   * @param ind The index long the x axis for the slicing plane.
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
    voxelGrid->generateSliceMesh(gridName, ind, minVal, maxVal);
  }

  ESP_SMART_POINTERS(VoxelWrapper)
};

}  // namespace geo
}  // namespace esp
#endif
