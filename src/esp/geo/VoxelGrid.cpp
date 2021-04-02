
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
VoxelGrid::VoxelGrid(const std::unique_ptr<assets::MeshData>& meshData,
                     const std::string& renderAssetHandle,
                     int resolution)
    : m_renderAssetHandle(renderAssetHandle) {
  VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();

  Mn::Debug() << "Voxelizing mesh..";

  // run VHACD
  interfaceVHACD->computeVoxelField(&meshData->vbo[0][0], meshData->vbo.size(),
                                    &meshData->ibo[0], meshData->ibo.size() / 3,
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
  auto boundaryGrid = getGrid<bool>("Boundary");

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

std::vector<std::pair<std::string, std::string>> VoxelGrid::getExistingGrids() {
  std::vector<std::pair<std::string, std::string>> existingGrids =
      std::vector<std::pair<std::string, std::string>>();
  std::map<std::string, GridEntry>::iterator it;
  for (it = grids_.begin(); it != grids_.end(); it++) {
    std::string typeName;
    VoxelGridType type = it->second.type;
    if (type == VoxelGridType::Bool) {
      typeName = "Bool";
    } else if (type == VoxelGridType::Int) {
      typeName = "Int";
    } else if (type == VoxelGridType::Float) {
      typeName = "Float";
    } else if (type == VoxelGridType::Vector3) {
      typeName = "Vector3";
    }
    existingGrids.push_back(std::make_pair(it->first, typeName));
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
  std::vector<Mn::Vector3i> increments = {{0, 0, 1},  {1, 0, 0},  {0, 1, 0},
                                          {0, 0, -1}, {0, -1, 0}, {-1, 0, 0}};
  auto grid = getGrid<bool>(gridName);
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

  // add cube to mesh
  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(local_coords);

  unsigned int sz = positions.size();
  const auto&& conePositions = coneData.positions3DAsArray();
  const auto&& coneNormals = coneData.normalsAsArray();
  const auto&& coneIndices = coneData.indices();

  // Get rotation quaternion
  Mn::Rad angle{acos(Mn::Math::dot(vec.normalized(), Mn::Vector3(0, 1, 0)))};
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

  // render cylinder
  Mn::Trade::MeshData cylinderData = Mn::Primitives::cylinderSolid(1, 3, 1.0f);

  // add cube to mesh
  // midpoint of a voxel

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

  /*Mn::Vector3 mid = getGlobalCoords(local_coords);
  Mn::Vector3 pos1 = vec.normalized() * m_voxelSize * 1 / 3 + mid;
  Mn::Vector3 orthog1 = Mn::Math::cross(vec, Mn::Vector3(0, 1, 0));
  if (orthog1 == Mn::Vector3(0, 0, 0)) {
    orthog1 = Mn::Vector3(1, 0, 0);
  }
  Mn::Vector3 orthog2 = Mn::Math::cross(vec, orthog1);

  Mn::Vector3 pos2 = mid + orthog1.normalized() * m_voxelSize * 1 / 15;
  Mn::Vector3 pos3 = mid + orthog2.normalized() * m_voxelSize * 1 / 15;
  Mn::Vector3 pos4 = mid - orthog1.normalized() * m_voxelSize * 1 / 15;
  Mn::Vector3 pos5 = mid - orthog2.normalized() * m_voxelSize * 1 / 15;

  unsigned int sz = positions.size();
  positions.push_back(pos1);
  positions.push_back(pos2);
  positions.push_back(pos3);
  positions.push_back(pos4);
  positions.push_back(pos5);

  colors.push_back(Mn::Color3(1, 1, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));
  colors.push_back(Mn::Color3(0, .3, 1));

  normals.push_back(vec.normalized());
  normals.push_back((pos1 - mid).normalized());
  normals.push_back((pos2 - mid).normalized());
  normals.push_back((pos3 - mid).normalized());
  normals.push_back((pos4 - mid).normalized());

  std::vector<unsigned int> inds{0, 1, 2, 0, 2, 3, 0, 3, 4,
                                 0, 4, 1, 1, 3, 2, 1, 4, 3};
  for (auto index : inds) {
    indices.push_back(sz + index);
  }

  // Add mesh information for the stem of the arrow

  // cube indices
  sz = positions.size();
  Mn::Vector3 pos6 = mid + orthog1.normalized() * m_voxelSize * 1 / 40;
  Mn::Vector3 pos7 = mid + orthog2.normalized() * m_voxelSize * 1 / 40;
  Mn::Vector3 pos8 = mid - orthog1.normalized() * m_voxelSize * 1 / 40;
  Mn::Vector3 pos9 = mid - orthog2.normalized() * m_voxelSize * 1 / 40;

  Mn::Vector3 pos10 = mid + orthog1.normalized() * m_voxelSize * 1 / 40 -
                      vec.normalized() * m_voxelSize * 1 / 4;
  Mn::Vector3 pos11 = mid + orthog2.normalized() * m_voxelSize * 1 / 40 -
                      vec.normalized() * m_voxelSize * 1 / 4;
  Mn::Vector3 pos12 = mid - orthog1.normalized() * m_voxelSize * 1 / 40 -
                      vec.normalized() * m_voxelSize * 1 / 4;
  Mn::Vector3 pos13 = mid - orthog2.normalized() * m_voxelSize * 1 / 40 -
                      vec.normalized() * m_voxelSize * 1 / 4;

  std::vector<Mn::Vector3> cubeVerts{pos6,  pos7,  pos8,  pos9,
                                     pos10, pos11, pos12, pos13};
  for (auto vert : cubeVerts) {
    positions.push_back(vert);
    colors.push_back(Mn::Color3(0, .3, 1));
    normals.push_back((vert - mid).normalized());
  }
  // add indices

  std::vector<unsigned int> cube_inds{4, 6, 5, 4, 7, 6, 6, 2, 5, 5,
                                      2, 1, 4, 5, 1, 4, 1, 0, 4, 0,
                                      3, 4, 3, 7, 2, 6, 7, 2, 7, 3};
  for (auto index : cube_inds) {
    indices.push_back(sz + index);
  }*/
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
  auto type = grids_[gridName].type;
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
