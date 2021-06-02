
#include <cassert>
#include <climits>
#include <cmath>

#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Algorithms.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/Math/Vector.h>
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
                     const Mn::Vector3i& voxelGridDimensions)
    : m_voxelSize(voxelSize),
      m_voxelGridDimensions(voxelGridDimensions),
      m_offset(Mn::Vector3(0.0, 0.0, 0.0)) {
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
    existingGrids.emplace_back(it->first, type);
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

void VoxelGrid::addVoxelToMeshPrimitives(
    Cr::Containers::Array<VoxelVertex>& vertexData,
    Cr::Containers::Array<Mn::UnsignedInt>& indexData,
    const Mn::Vector3i& localCoords,
    const std::vector<bool>& neighbors,
    const Mn::Color3& color) {
  // Using the data of a cubeSolid to create the voxel cube
  assert(neighbors.size() >= 6);
  Mn::Trade::MeshData cubeData = Mn::Primitives::cubeSolid();

  // add cube to mesh
  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(localCoords);
  unsigned int sz = vertexData.size();
  Corrade::Containers::StridedArrayView1D<const Magnum::Vector3> cubePositions =
      cubeData.attribute<Mn::Vector3>(Mn::Trade::MeshAttribute::Position);
  Corrade::Containers::StridedArrayView1D<const Magnum::Vector3> cubeNormals =
      cubeData.attribute<Mn::Vector3>(Mn::Trade::MeshAttribute::Normal);
  Cr::Containers::ArrayView<const Mn::UnsignedShort> cubeIndices =
      cubeData.indices<Mn::UnsignedShort>();
  for (std::size_t i = 0; i != cubeData.vertexCount(); ++i) {
    arrayAppend(vertexData, Cr::InPlaceInit,
                cubePositions[i] * m_voxelSize / 2 + mid,
                cubePositions[i].normalized() * 1 / 4 +
                    cubeNormals[i].normalized() * 3 / 4,
                color);
  }

  for (int i = 0; i < 6; i++) {
    if (!neighbors[i]) {
      for (int j = 0; j < 6; j++) {
        arrayAppend(indexData, sz + cubeIndices[i * 6 + j]);
      }
    }
  }
}

void VoxelGrid::addVectorToMeshPrimitives(
    Cr::Containers::Array<VoxelVertex>& vertexData,
    Cr::Containers::Array<Mn::UnsignedInt>& indexData,
    const Mn::Vector3i& localCoords,
    const Mn::Vector3& vec) {
  Mn::Trade::MeshData coneData = Mn::Primitives::coneSolid(1, 4, 1.0f);

  // midpoint of a voxel
  Mn::Vector3 mid = getGlobalCoords(localCoords);
  // add cone to mesh (tip of arrow)
  unsigned int sz = vertexData.size();
  Corrade::Containers::StridedArrayView1D<const Magnum::Vector3> conePositions =
      coneData.attribute<Mn::Vector3>(Mn::Trade::MeshAttribute::Position);
  Corrade::Containers::StridedArrayView1D<const Magnum::Vector3> coneNormals =
      coneData.attribute<Mn::Vector3>(Mn::Trade::MeshAttribute::Normal);
  Cr::Containers::ArrayView<const Mn::UnsignedInt> coneIndices =
      coneData.indices<Mn::UnsignedInt>();
  // Get rotation quaternion
  Mn::Rad angle = Mn::Math::angle(vec.normalized(), Mn::Vector3(0, 1, 0));

  // Cross product
  Mn::Vector3 crossProduct =
      Mn::Math::cross(vec.normalized(), Mn::Vector3(0, 1, 0));
  Mn::Quaternion vecRotation{Mn::Math::IdentityInit};

  if (vec[0] == 0 && vec[2] == 0) {
    crossProduct = Mn::Vector3(1, 0, 0);
  }
  vecRotation = Mn::Quaternion::rotation(-angle, crossProduct.normalized());
  for (std::size_t i = 0; i != coneData.vertexCount(); ++i) {
    arrayAppend(vertexData, Cr::InPlaceInit,
                vecRotation.transformVector(conePositions[i] *
                                                Mn::Vector3(0.02, 0.035, 0.02) +
                                            Mn::Vector3(0, 0.025, 0)) +
                    mid,
                coneNormals[i], Mn::Color3{0.4f, 0.8f, 1.0f});
  }

  for (const Mn::UnsignedInt index : coneIndices) {
    arrayAppend(indexData, sz + index);
  }
  // render cylinder (arrow stem)
  Mn::Trade::MeshData cylinderData = Mn::Primitives::cylinderSolid(1, 3, 1.0f);

  sz = vertexData.size();
  Corrade::Containers::StridedArrayView1D<const Magnum::Vector3>
      cylinderPositions = cylinderData.attribute<Mn::Vector3>(
          Mn::Trade::MeshAttribute::Position);
  Corrade::Containers::StridedArrayView1D<const Magnum::Vector3>
      cylinderNormals =
          cylinderData.attribute<Mn::Vector3>(Mn::Trade::MeshAttribute::Normal);
  Cr::Containers::ArrayView<const Mn::UnsignedInt> cylinderIndices =
      cylinderData.indices<Mn::UnsignedInt>();

  for (std::size_t i = 0; i != cylinderData.vertexCount(); ++i) {
    arrayAppend(vertexData, Cr::InPlaceInit,
                vecRotation.transformVector(
                    cylinderPositions[i] * Mn::Vector3(0.007, 0.025, 0.007) -
                    Mn::Vector3(0, 0.025, 0)) +
                    mid,
                cylinderNormals[i], Mn::Color3{0.3f, 0.7f, 0.9f});
  }

  for (const Mn::UnsignedInt index : cylinderIndices) {
    arrayAppend(indexData, sz + index);
  }
}

void VoxelGrid::generateMeshDataAndMeshGL(
    const std::string& gridName,
    Cr::Containers::Array<VoxelVertex>& vertexData,
    Cr::Containers::Array<Mn::UnsignedInt>& indexData) {
  // need to make the index/attribute views first, before calling std::move() on
  // the data
  Mn::Trade::MeshIndexData indices{indexData};
  Mn::Trade::MeshAttributeData positions{
      Mn::Trade::MeshAttribute::Position,
      Cr::Containers::stridedArrayView(vertexData)
          .slice(&VoxelVertex::position)};  // view on just the position member
  Mn::Trade::MeshAttributeData colors{
      Mn::Trade::MeshAttribute::Color,
      Cr::Containers::stridedArrayView(vertexData)
          .slice(&VoxelVertex::color)};  // view on just the color member
  Mn::Trade::MeshAttributeData normals{
      Mn::Trade::MeshAttribute::Normal,
      Cr::Containers::stridedArrayView(vertexData)
          .slice(&VoxelVertex::normal)};  // view on just the normal member

  // now move the data into a MeshData, together with all metadata describing
  // indices and attributes
  Mn::Trade::MeshData data{
      Mn::MeshPrimitive::Triangles,
      // the cast takes an Array<UnsignedInt> and turns it into an Array<char>
      // without having to copy everything again
      Cr::Containers::arrayAllocatorCast<char>(std::move(indexData)),
      indices,
      Cr::Containers::arrayAllocatorCast<char>(std::move(vertexData)),
      {positions, colors, normals}};

  // and save those, again move because the neither the Array nor MeshData are
  // copyable
  meshDataDict_[gridName] =
      std::make_shared<Mn::Trade::MeshData>(std::move(data));

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
  Cr::Containers::Array<VoxelVertex> vertices;
  Cr::Containers::Array<Mn::UnsignedInt> indices;
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
            addVectorToMeshPrimitives(vertices, indices, local_coords, vec);
        } else {
          bool val = getVoxel<bool>(local_coords, gridName);
          if (val) {
            num_filled++;
            std::vector<bool> neighbors{};
            fillBoolGridNeighborhood(neighbors, gridName, local_coords);
            addVoxelToMeshPrimitives(vertices, indices, local_coords,
                                     neighbors);
          }
        }
      }
    }
  }

  generateMeshDataAndMeshGL(gridName, vertices, indices);
}

}  // namespace geo
}  // namespace esp
