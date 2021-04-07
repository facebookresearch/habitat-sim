
#include <assert.h>
#include <limits.h>
#include <cmath>

#include <Corrade/Utility/Algorithms.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/Primitives/Cone.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cylinder.h>

#include "VoxelGrid.h"
#include "esp/assets/ResourceManager.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

#ifdef ESP_BUILD_WITH_VHACD
VoxelGrid::VoxelGrid(const assets::MeshData& meshData,
                     const std::string& renderAssetHandle,
                     int resolution)
    : m_renderAssetHandle(renderAssetHandle) {
  VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();

  Mn::Debug() << "Voxelizing mesh..";

  // run VHACD
  interfaceVHACD->computeVoxelField(&meshData.vbo[0][0], meshData.vbo.size(),
                                    &meshData.ibo[0], meshData.ibo.size() / 3,
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
  addGrid<bool>("Boundary");
  Cr::Containers::StridedArrayView3D<bool> boundaryGrid =
      getGrid<bool>("Boundary");

  int num_filled = 0;
  // Transfer data from Volume to VoxelGrid
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        if (vhacdVolume->GetVoxel(i, j, k) >= 2) {
          num_filled++;
          boundaryGrid[i][j][k] = true;
        }
      }
    }
  }
  if (num_filled == dims[0] * dims[1] * dims[2]) {
    // When VHACD is given too low of a resolution
    Mn::Debug() << "VOXELIZATION FAILED";
  }
}
#endif

VoxelGrid::VoxelGrid(const Mn::Vector3& voxelSize,
                     const Mn::Vector3i& voxelGridDimensions) {
  m_voxelSize = voxelSize;
  m_voxelGridDimensions = voxelGridDimensions;
  m_offset = Mn::Vector3(0.0, 0.0, 0.0);
  addGrid<bool>("Boundary");
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

std::vector<std::pair<std::string, esp::geo::VoxelGridType>>
VoxelGrid::getExistingGrids() {
  std::vector<std::pair<std::string, esp::geo::VoxelGridType>> existingGrids;
  std::map<std::string, GridEntry>::iterator it;
  for (it = grids_.begin(); it != grids_.end(); it++) {
    std::string typeName;
    VoxelGridType type = it->second.type;
    existingGrids.push_back(std::make_pair(it->first, type));
  }
  return existingGrids;
}

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
  return Mn::Vector3(coords) * m_voxelSize + m_offset;
}

void VoxelGrid::fillBoolGridNeighborhood(std::vector<bool>& neighbors,
                                         const std::string& gridName,
                                         const Mn::Vector3i& index) {
  Mn::Vector3i increments[] = {{0, 0, 1},  {1, 0, 0},  {0, 1, 0},
                               {0, 0, -1}, {0, -1, 0}, {-1, 0, 0}};
  Cr::Containers::StridedArrayView3D<bool> grid = getGrid<bool>(gridName);
  for (int i = 0; i < 6; i++) {
    auto n = index + increments[i];
    neighbors.push_back(isValidIndex(n) ? grid[n[0]][n[1]][n[2]] : false);
  }
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
  Mn::Trade::MeshData coneData = Mn::Primitives::coneSolid(1, 3, 1.0f);

  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(local_coords);

  // add cone to mesh (tip of arrow)
  unsigned int sz = positions.size();
  const auto&& conePositions = coneData.positions3DAsArray();
  const auto&& coneNormals = coneData.normalsAsArray();
  const auto&& coneIndices = coneData.indices();

  // Get rotation quaternion
  Mn::Rad angle{static_cast<float>(
      acos(Mn::Math::dot(vec.normalized(), Mn::Vector3(0, 1, 0))))};
  Mn::Vector3 crossProduct = Mn::Math::cross(vec, Mn::Vector3(0, 1, 0));
  Mn::Quaternion vecRotation{Mn::Math::IdentityInit};

  if (vec[0] == 0 && vec[2] == 0) {
    crossProduct = Mn::Vector3(0, 1, 0);
  }
  vecRotation = vecRotation.rotation(-angle, crossProduct.normalized());
  for (const auto& ind : conePositions) {
    positions.push_back(
        vecRotation.transformVector(ind * Mn::Vector3(0.02, 0.04, 0.02) +
                                    Mn::Vector3(0, 0.025, 0)) +
        mid);
    colors.push_back(Mn::Color3(0, .3, 1));
  }

  for (const auto& norm : coneNormals) {
    normals.push_back(norm);
  }

  for (const auto&& index : coneIndices) {
    indices.push_back(sz + index[0]);
  }

  // render cylinder (arrow stem)
  Mn::Trade::MeshData cylinderData = Mn::Primitives::cylinderSolid(1, 3, 1.0f);

  sz = positions.size();
  const auto&& cylinderPositions = cylinderData.positions3DAsArray();
  const auto&& cylinderNormals = cylinderData.normalsAsArray();
  const auto&& cylinderIndices = cylinderData.indices();

  for (const auto& ind : cylinderPositions) {
    positions.push_back(
        vecRotation.transformVector(ind * Mn::Vector3(0.01, 0.03, 0.01) -
                                    Mn::Vector3(0, 0.025, 0)) +
        mid);
    colors.push_back(Mn::Color3(0, .3, 1));
  }

  for (const auto& norm : cylinderNormals) {
    Mn::Vector3 transformedNorm =
        vecRotation.transformVector(norm * Mn::Vector3(0.01, 0.04, 0.01));
    normals.push_back(transformedNorm);
  }

  for (const auto&& index : cylinderIndices) {
    indices.push_back(sz + index[0]);
  }
}

void VoxelGrid::generateMeshDataAndMeshGL(const std::string& gridName,
                                          std::vector<Mn::UnsignedInt>& indices,
                                          std::vector<Mn::Vector3>& positions,
                                          std::vector<Mn::Vector3>& normals,
                                          std::vector<Mn::Color3>& colors) {
  // If the mesh already exists, replace it. Otherwise, create a new entry in
  // the dict
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
           Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Normal,
                                        Cr::Containers::arrayView(normals)}}));

  // If the mesh already exists, replace it. Otherwise, create a new entry in
  // the dict
  if (meshGLDict_.find(gridName) != meshGLDict_.end()) {
    meshGLDict_[gridName] = Mn::MeshTools::compile(*meshDataDict_[gridName]);
  } else {
    meshGLDict_.insert(std::make_pair(
        gridName, Mn::MeshTools::compile(*meshDataDict_[gridName])));
  }
}

void VoxelGrid::generateMesh(const std::string& gridName) {
  assert(grids_.find(gridName) != grids_.end());
  std::vector<Mn::UnsignedInt> indices;
  std::vector<Mn::Vector3> positions;
  std::vector<Mn::Vector3> normals;
  std::vector<Mn::Color3> colors;
  int num_filled = 0;
  VoxelGridType type = grids_[gridName].type;
  // iterate through each voxel grid cell
  for (int i = 0; i < m_voxelGridDimensions[0]; i++) {
    for (int j = 0; j < m_voxelGridDimensions[1]; j++) {
      for (int k = 0; k < m_voxelGridDimensions[2]; k++) {
        Mn::Vector3i local_coords(i, j, k);
        if (type == VoxelGridType::Vector3) {
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
