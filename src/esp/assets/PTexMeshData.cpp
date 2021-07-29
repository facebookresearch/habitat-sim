// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PTexMeshData.h"

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/GL/BufferTextureFormat.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#include "esp/core/esp.h"
#include "esp/gfx/PTexMeshShader.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

static constexpr int ROTATION_SHIFT = 30;
static constexpr int FACE_MASK = 0x3FFFFFFF;

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace assets {

void PTexMeshData::load(const std::string& meshFile,
                        const std::string& atlasFolder) {
  if (!io::exists(meshFile)) {
    Cr::Utility::Fatal{-1} << "PTexMeshData::load: Mesh file" << meshFile
                           << "does not exist.";
  }
  if (!io::exists(atlasFolder)) {
    Cr::Utility::Fatal{-1} << "PTexMeshData::load: The atlasFolder"
                           << atlasFolder << "does not exist.";
  }

  // Parse parameters
  const auto& paramsFile = atlasFolder + "/parameters.json";
  if (!io::exists(paramsFile)) {
    Cr::Utility::Fatal{-1} << "PTexMeshData::load: The parameter file"
                           << paramsFile << "does not exist.";
  }

  const io::JsonDocument json = io::parseJsonFile(paramsFile);
  splitSize_ = json["splitSize"].GetDouble();
  tileSize_ = json["tileSize"].GetInt();
  atlasFolder_ = atlasFolder;

  loadMeshData(meshFile);
}

float PTexMeshData::exposure() const {
  return exposure_;
}

void PTexMeshData::setExposure(float val) {
  exposure_ = val;
}

float PTexMeshData::gamma() const {
  return gamma_;
}

void PTexMeshData::setGamma(float val) {
  gamma_ = val;
}

float PTexMeshData::saturation() const {
  return saturation_;
}

void PTexMeshData::setSaturation(float val) {
  saturation_ = val;
}

const std::vector<PTexMeshData::MeshData>& PTexMeshData::meshes() const {
  return submeshes_;
}

std::string PTexMeshData::atlasFolder() const {
  return atlasFolder_;
}
// this is to break the quad into 2 triangles
// we need this triangle mesh to do object picking
void computeTriangleMeshIndices(uint64_t numFaces,
                                PTexMeshData::MeshData& currentSubMesh) {
  for (size_t jFace = 0; jFace < numFaces; ++jFace) {
    size_t offset = jFace * 4;
    // 1st triangle is (0, 1, 2)
    currentSubMesh.ibo_tri.push_back(currentSubMesh.ibo[offset + 0]);
    currentSubMesh.ibo_tri.push_back(currentSubMesh.ibo[offset + 1]);
    currentSubMesh.ibo_tri.push_back(currentSubMesh.ibo[offset + 2]);
    // 2nd triangle is (0, 2, 3)
    currentSubMesh.ibo_tri.push_back(currentSubMesh.ibo[offset + 0]);
    currentSubMesh.ibo_tri.push_back(currentSubMesh.ibo[offset + 2]);
    currentSubMesh.ibo_tri.push_back(currentSubMesh.ibo[offset + 3]);
  }
}

// split the original ptex mesh into sub-meshes.
//
// WARNING:
// This function is NOT used in the current simulator
//
// Why?
//
// the original mesh is cut into pieces (sub-meshes), each of which contains
// a number of faces. A key step of the algorithm is to sort them based on a
// code. ReplicaSDK uses std::sort to do this job, where the original relative
// order of the semantically equivalent values are *not* preserved.
// (ideally, std::stable_sort should be applied here.)
//
// the consequence is we cannot reproduce such face order in our simulator
// using std::sort. (clang and gcc may have different implementations of
// std::sort).
// So this function, splitMesh is disabled, and the sorted faces are now
// directly dumped from ReplicaSDK, and loaded to our simulator. See
// loadSubMeshes(...) for more details;

std::vector<PTexMeshData::MeshData> splitMesh(
    const PTexMeshData::MeshData& mesh,
    const float splitSize) {
  std::vector<uint32_t> verts;
  verts.resize(mesh.vbo.size());

  auto Part1By2 = [](uint64_t x) {
    x &= 0x1fffff;  // mask off lower 21 bits
    x = (x | (x << 32)) & 0x1f00000000ffff;
    x = (x | (x << 16)) & 0x1f0000ff0000ff;
    x = (x | (x << 8)) & 0x100f00f00f00f00f;
    x = (x | (x << 4)) & 0x10c30c30c30c30c3;
    x = (x | (x << 2)) & 0x1249249249249249;
    return x;
  };

  auto EncodeMorton3 = [&Part1By2](const vec3i& v) {
    return (Part1By2(v(2)) << 2) + (Part1By2(v(1)) << 1) + Part1By2(v(0));
  };

  box3f boundingBox;

  for (size_t i = 0; i < mesh.vbo.size(); i++) {
    boundingBox.extend(mesh.vbo[i].head<3>());
  }

// calculate vertex grid position and code
#pragma omp parallel for
  for (size_t i = 0; i < mesh.vbo.size(); i++) {
    const vec3f p = mesh.vbo[i].head<3>();
    vec3f pi = (p - boundingBox.min()) / splitSize;
    verts[i] = EncodeMorton3(pi.cast<int>());
  }

  // data structure for sorting faces
  struct SortFace {
    uint32_t index[4];
    uint32_t code;
    // TODO:
    // Do we need the originalFace anywhere?
    // If not, remove it in the future PR
    size_t originalFace;
  };

  // fill per-face data structures (including codes)
  size_t numFaces = mesh.ibo.size() / 4;
  std::vector<SortFace> faces;
  faces.resize(numFaces);

#pragma omp parallel for
  for (size_t i = 0; i < numFaces; i++) {
    faces[i].originalFace = i;
    faces[i].code = std::numeric_limits<uint32_t>::max();
    for (int j = 0; j < 4; j++) {
      faces[i].index[j] = mesh.ibo[i * 4 + j];

      // face code is minimum of referenced vertices codes
      faces[i].code = std::min(faces[i].code, verts[faces[i].index[j]]);
    }
  }

  // sort faces by code
  std::sort(faces.begin(), faces.end(),
            [](const SortFace& f1, const SortFace& f2) -> bool {
              return (f1.code < f2.code);
            });

  // find face chunk start indices
  std::vector<uint32_t> chunkStart;
  chunkStart.push_back(0);
  uint32_t prevCode = faces[0].code;
  for (size_t i = 1; i < faces.size(); i++) {
    if (faces[i].code != prevCode) {
      chunkStart.push_back(i);
      prevCode = faces[i].code;
    }
  }

  chunkStart.push_back(faces.size());
  size_t numChunks = chunkStart.size() - 1;

  // TODO:
  // Do we need the maxFaces anywhere?
  // If not, remove it in the future PR
  size_t maxFaces = 0;
  for (size_t i = 0; i < numChunks; i++) {
    uint32_t chunkSize = chunkStart[i + 1] - chunkStart[i];
    if (chunkSize > maxFaces)
      maxFaces = chunkSize;
  }

  // create new mesh for each chunk of faces
  std::vector<PTexMeshData::MeshData> subMeshes;

  for (size_t i = 0; i < numChunks; i++) {
    subMeshes.emplace_back();
  }

#pragma omp parallel for
  for (size_t i = 0; i < numChunks; i++) {
    uint32_t chunkSize = chunkStart[i + 1] - chunkStart[i];

    std::vector<uint32_t> refdVerts;
    // it maps indices from original mesh to the new ones in the chunk
    std::unordered_map<uint32_t, uint32_t> refdVertsMap;
    subMeshes[i].ibo.resize(chunkSize * 4);

    for (size_t j = 0; j < chunkSize; j++) {
      size_t faceIdx = chunkStart[i] + j;
      for (int k = 0; k < 4; k++) {
        uint32_t vertIndex = faces[faceIdx].index[k];
        uint32_t newIndex = 0;

        auto it = refdVertsMap.find(vertIndex);

        if (it == refdVertsMap.end()) {
          // vertex not found, add
          newIndex = refdVerts.size();
          refdVerts.push_back(vertIndex);
          refdVertsMap[vertIndex] = newIndex;
        } else {
          // found, use existing index
          newIndex = it->second;
        }
        subMeshes[i].ibo[j * 4 + k] = newIndex;
      }
    }

    computeTriangleMeshIndices(chunkSize, subMeshes[i]);

    // add referenced vertices to submesh
    subMeshes[i].vbo.resize(refdVerts.size());
    subMeshes[i].nbo.resize(refdVerts.size());
    for (size_t j = 0; j < refdVerts.size(); j++) {
      uint32_t index = refdVerts[j];
      subMeshes[i].vbo[j] = mesh.vbo[index];
      subMeshes[i].nbo[j] = mesh.nbo[index];
      // Careful:
      // for Ptex mesh we never ever set the "cbo"
    }
  }

  return subMeshes;
}

// =========== the input file format =======================
// a uint64_t, N, the number of sub-meshes;

// it follows by N chunks, each of which contains:
// a uint64_t, M, the number of faces within the chunk;
// M uint32_t, face indices (sorted) in the *original* mesh;
// =========================================================

//  this binary can be generated by modifying the splitMesh() in file
//  PTexLib.cpp in ReplicaSDK.
//  and the name is hard-coded as "sorted_faces.bin" in the simulator.

// Put it in the sub-folder, "habitat".

std::vector<PTexMeshData::MeshData> loadSubMeshes(
    const PTexMeshData::MeshData& mesh,
    const std::string& filename) {
  // sanity checks
  CORRADE_ASSERT(!filename.empty(),
                 "PTexMeshData::loadSubMeshes: filename cannot be empty.", {});
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);
  CORRADE_ASSERT(
      file.good(),
      "PTexMeshData::loadSubMeshes: cannot open the file " << filename, {});

  uint64_t numSubMeshes = 0;
  file.read(reinterpret_cast<char*>(&numSubMeshes), sizeof(uint64_t));

  std::vector<PTexMeshData::MeshData> subMeshes(numSubMeshes);

  size_t totalFaces = 0;  // used in sanity check
  for (uint64_t iMesh = 0; iMesh < numSubMeshes; ++iMesh) {
    uint64_t numFaces = 0;
    file.read(reinterpret_cast<char*>(&numFaces), sizeof(uint64_t));

    std::vector<uint32_t> originalFaces(numFaces);
    // load the face indices in the *original* mesh
    file.read(reinterpret_cast<char*>(originalFaces.data()),
              sizeof(uint32_t) * numFaces);
    // a *vertex* lookup table:
    // global index of the original mesh --> local index in sub-meshes
    // (note: this table cannot be defined outside of the for loop, as a vertex
    // in original mesh may appear in different sub-meshes.)
    std::unordered_map<uint32_t, uint32_t> globalToLocal;

    // Another *vertex* lookup table:
    // local index of current sub-mesh --> global index of the original mesh
    std::vector<uint32_t> localToGlobal;

    // compute the two lookup tables
    for (size_t jFace = 0; jFace < numFaces; ++jFace) {
      uint32_t f = originalFaces[jFace];  // face index in original mesh
      for (size_t v = 0; v < 4; ++v) {
        uint32_t global = mesh.ibo[f * 4 + v];
        if (globalToLocal.find(global) == globalToLocal.end()) {
          globalToLocal[global] = localToGlobal.size();
          localToGlobal.push_back(global);
        }
      }
    }

    // compute the ibo for the current sub-mesh
    auto& ibo = subMeshes[iMesh].ibo;
    ibo.resize(numFaces * 4);
    uint64_t idx = 0;
    for (size_t jFace = 0; jFace < numFaces; ++jFace) {
      uint32_t f = originalFaces[jFace];
      for (size_t v = 0; v < 4; ++v) {
        uint32_t global = mesh.ibo[f * 4 + v];
        CORRADE_ASSERT(globalToLocal.find(global) != globalToLocal.end(),
                       "PTexMeshData::loadSubMeshes: vertex "
                           << global << " is not in the sub-mesh " << iMesh,
                       {});
        uint32_t local = globalToLocal[global];
        ibo[idx++] = local;
      }
    }  // for jFace

    // this is to break the quad into 2 triangles
    // we need this triangle mesh to do object picking
    computeTriangleMeshIndices(numFaces, subMeshes[iMesh]);

    // compute the vbo, nbo for the current sub-mesh
    uint64_t numVertices = localToGlobal.size();
    subMeshes[iMesh].vbo.resize(numVertices);
    subMeshes[iMesh].nbo.resize(numVertices);
    for (size_t jLocal = 0; jLocal < numVertices; ++jLocal) {
      uint32_t global = localToGlobal[jLocal];
      subMeshes[iMesh].vbo[jLocal] = mesh.vbo[global];
      subMeshes[iMesh].nbo[jLocal] = mesh.nbo[global];
    }

    // Careful:
    // for Ptex mesh we never ever set the "cbo"

    totalFaces += numFaces;
  }  // for iMesh
  file.close();
  CORRADE_ASSERT(totalFaces == mesh.ibo.size() / 4,
                 "PTexMeshData::loadSubMeshes: the number of faces loaded from "
                 "the file does not "
                 "match it from the ptex mesh.",
                 {});

  ESP_DEBUG() << "The number of quads:" << totalFaces << ", which equals to"
              << totalFaces * 2 << "triangles.";

  return subMeshes;
}

void PTexMeshData::calculateAdjacency(const PTexMeshData::MeshData& mesh,
                                      std::vector<uint32_t>& adjFaces) {
  struct EdgeData {
    int face;
    int edge;
  };

  std::unordered_map<uint64_t, std::vector<EdgeData>> edgeMap;

  size_t numFaces = mesh.ibo.size() / 4;

  typedef std::unordered_map<uint64_t, std::vector<EdgeData>>::iterator
      EdgeIter;
  std::vector<EdgeIter> edgeIterators(numFaces * 4);

  // for each face
  for (int f = 0; f < numFaces; f++) {
    // for each edge
    for (int e = 0; e < 4; e++) {
      // add to edge to face map
      const int e_index = f * 4 + e;
      const uint32_t i0 = mesh.ibo[e_index];
      const uint32_t i1 = mesh.ibo[f * 4 + ((e + 1) % 4)];
      const uint64_t key =
          static_cast<uint64_t>(std::min(i0, i1)) << 32 | std::max(i0, i1);

      const EdgeData edgeData{f, e};

      auto it = edgeMap.find(key);

      if (it == edgeMap.end()) {
        it = edgeMap.emplace(key, std::vector<EdgeData>()).first;
        it->second.reserve(4);
        it->second.push_back(edgeData);
      } else {
        it->second.push_back(edgeData);
      }

      edgeIterators[e_index] = it;
    }
  }

  adjFaces.resize(numFaces * 4);

  for (int f = 0; f < numFaces; f++) {
    for (int e = 0; e < 4; e++) {
      const int e_index = f * 4 + e;
      auto it = edgeIterators[e_index];
      const std::vector<EdgeData>& adj = it->second;

      // find adjacent face
      int adjFace = -1;
      for (size_t i = 0; i < adj.size(); i++) {
        if (adj[i].face != f)
          adjFace = adj[i].face;
      }

      // find number of 90 degree rotation steps between faces
      int rot = 0;
      if (adj.size() == 2) {
        int edge0 = 0, edge1 = 0;
        if (adj[0].edge == e) {
          edge0 = adj[0].edge;
          edge1 = adj[1].edge;
        } else if (adj[1].edge == e) {
          edge0 = adj[1].edge;
          edge1 = adj[0].edge;
        }

        rot = (edge0 - edge1 + 2) & 3;
      }

      // pack adjacent face and rotation into 32-bit int
      adjFaces[f * 4 + e] = (rot << ROTATION_SHIFT) | (adjFace & FACE_MASK);
    }
  }
}

void PTexMeshData::loadMeshData(const std::string& meshFile) {
  PTexMeshData::MeshData originalMesh;
  parsePLY(meshFile, originalMesh);

  computeTriangleMeshIndices(originalMesh.ibo.size() / 4, originalMesh);
  collisionMeshData_.primitive = Mn::MeshPrimitive::Triangles;

  submeshes_.clear();
  if (splitSize_ > 0.0f) {
    ESP_DEBUG() << "Splitting mesh...";

    collisionVbo_ = Cr::Containers::Array<Mn::Vector3>(originalMesh.vbo.size());
    Cr::Utility::copy(Cr::Containers::arrayCast<Mn::Vector3>(
                          Cr::Containers::arrayView(originalMesh.vbo)),
                      collisionVbo_);
    collisionIbo_ =
        Cr::Containers::Array<Mn::UnsignedInt>(originalMesh.ibo_tri.size());
    Cr::Utility::copy(originalMesh.ibo_tri, collisionIbo_);

    collisionMeshData_.positions = collisionVbo_;
    collisionMeshData_.indices = collisionIbo_;

    // In this version, we load the sorted faces directly from an external
    // binary file dumped out from ReplicaSDK, and disable the function
    // splitMesh(...)

    // See detailed comments in front of the splitMesh(...)
    std::string subMeshesFilename = Corrade::Utility::Directory::join(
        atlasFolder_, "../habitat/sorted_faces.bin");
    submeshes_ = loadSubMeshes(originalMesh, subMeshesFilename);

    // TODO:
    // re-activate the following function after the bug is fixed in ReplicaSDK.
    // submeshes_ = splitMesh(originalMesh, splitSize_);
    // ESP_DEBUG() << "done" << std::endl;
  } else {
    submeshes_.emplace_back(std::move(originalMesh));
    collisionMeshData_.positions = Cr::Containers::arrayCast<Mn::Vector3>(
        Cr::Containers::arrayView(submeshes_.back().vbo));
    collisionMeshData_.indices = Cr::Containers::arrayCast<Mn::UnsignedInt>(
        Cr::Containers::arrayView(submeshes_.back().ibo_tri));
  }
}

void PTexMeshData::parsePLY(const std::string& filename,
                            PTexMeshData::MeshData& meshData) {
  std::vector<std::string> comments;
  std::vector<std::string> objInfo;

  std::string lastElement;
  std::string lastProperty;

  enum Properties { POSITION = 0, NORMAL, COLOR, NUM_PROPERTIES };

  size_t numVertices = 0;

  size_t positionDimensions = 0;
  size_t normalDimensions = 0;
  size_t colorDimensions = 0;

  std::vector<Properties> vertexLayout;

  size_t numFaces = 0;

  std::ifstream file(filename, std::ios::binary);

  // Header parsing
  {
    std::string line;

    while (std::getline(file, line)) {
      std::istringstream ls(line);
      std::string token;
      ls >> token;

      if (token == "ply" || token == "PLY" || token == "") {
        // Skip preamble line
        continue;
      } else if (token == "comment") {
        // Just store these incase
        comments.push_back(line.erase(0, 8));
      } else if (token == "format") {
        // We can only parse binary data, so check that's what it is
        std::string s;
        ls >> s;
        CORRADE_ASSERT(s == "binary_little_endian",
                       "PTexMeshData::parsePLY: the file is not a binary file "
                       "in little endian byte order", );
      } else if (token == "element") {
        std::string name;
        size_t size = 0;
        ls >> name >> size;

        if (name == "vertex") {
          // Pull out the number of vertices
          numVertices = size;
        } else if (name == "face") {
          // Pull out number of faces
          numFaces = size;
          CORRADE_ASSERT(numFaces > 0,
                         "PTexMeshData::parsePLY: number of faces is not "
                         "greater than 0.", );
        } else {
          CORRADE_ASSERT(
              false, "PTexMeshData::parsePLY: Cannot parse element" << name, );
        }

        // Keep track of what element we parsed last to associate the
        // properties that follow
        lastElement = name;
      } else if (token == "property") {
        std::string type, name;
        ls >> type;

        // Special parsing for list properties (e.g. faces)
        bool isList = false;

        if (type == "list") {
          isList = true;

          std::string countType;
          ls >> countType >> type;

          CORRADE_ASSERT(countType == "uchar" || countType == "uint8",
                         "PTexMeshData::parsePLY: Don't understand count type"
                             << countType, );

          CORRADE_ASSERT(
              type == "int",
              "PTexMeshData::parsePLY: Don't understand index type" << type, );

          CORRADE_ASSERT(lastElement == "face",
                         "PTexMeshData::parsePLY: Only expecting list after "
                         "face element, not after"
                             << lastElement, );
        }

        CORRADE_ASSERT(
            type == "float" || type == "int" || type == "uchar" ||
                type == "uint8",
            "PTexMeshData::parsePLY: Don't understand type" << type, );

        ls >> name;

        // Collecting vertex property information
        if (lastElement == "vertex") {
          CORRADE_ASSERT(type != "int",
                         "PTexMeshData::parsePLY: Don't support 32-bit integer "
                         "properties", );

          // Position information
          if (name == "x") {
            positionDimensions = 1;
            vertexLayout.push_back(Properties::POSITION);
            CORRADE_ASSERT(type == "float",
                           "PTexMeshData::parsePLY: Don't support 8-bit "
                           "integer positions", );
          } else if (name == "y") {
            CORRADE_ASSERT(lastProperty == "x",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "x, y, z, (w) order", );
            positionDimensions = 2;
          } else if (name == "z") {
            CORRADE_ASSERT(lastProperty == "y",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "x, y, z, (w) order", );
            positionDimensions = 3;
          } else if (name == "w") {
            CORRADE_ASSERT(lastProperty == "z",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "x, y, z, (w) order", );
            positionDimensions = 4;
          }

          // Normal information
          if (name == "nx") {
            normalDimensions = 1;
            vertexLayout.push_back(Properties::NORMAL);
            CORRADE_ASSERT(type == "float",
                           "PTexMeshData::parsePLY: Don't support 8-bit "
                           "integer normals", );
          } else if (name == "ny") {
            CORRADE_ASSERT(lastProperty == "nx",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "nx, ny, nz order", );
            normalDimensions = 2;
          } else if (name == "nz") {
            CORRADE_ASSERT(lastProperty == "ny",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "nx, ny, nz order", );
            normalDimensions = 3;
          }

          // Color information
          if (name == "red") {
            colorDimensions = 1;
            vertexLayout.push_back(Properties::COLOR);
            CORRADE_ASSERT(type == "uchar" || type == "uint8",
                           "PTexMeshData::parsePLY: Don't support non-8-bit "
                           "integer colors", );
          } else if (name == "green") {
            CORRADE_ASSERT(lastProperty == "red",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "red, green, blue, (alpha) order", );
            colorDimensions = 2;
          } else if (name == "blue") {
            CORRADE_ASSERT(lastProperty == "green",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "red, green, blue, (alpha) order", );
            colorDimensions = 3;
          } else if (name == "alpha") {
            CORRADE_ASSERT(lastProperty == "blue",
                           "PTexMeshData::parsePLY: Properties should follow "
                           "red, green, blue, (alpha) order", );
            colorDimensions = 4;
          }
        } else if (lastElement == "face") {
          CORRADE_ASSERT(isList,
                         "PTexMeshData::parsePLY: No idea what to do with "
                         "properties following faces", );
        } else {
          // No idea what to do with properties before elements
          CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        }

        lastProperty = name;
      } else if (token == "obj_info") {
        // Just store these incase
        objInfo.push_back(line.erase(0, 9));
      } else if (token == "end_header") {
        // Done reading!
        break;
      } else {
        // Something unrecognised
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      }
    }

    // Check things make sense.
    CORRADE_ASSERT(
        numVertices > 0,
        "PTexMeshData::parsePLY: number of vertices is not greater than 0", );
    CORRADE_ASSERT(positionDimensions > 0,
                   "PTexMeshData::parsePLY: the dimensions of the position is "
                   "not greater than 0", );
    CORRADE_ASSERT(
        positionDimensions == 3,
        "PTexMeshData::parsePLY: the dimensions of the position must be 3.", );
  }

  meshData.vbo.resize(numVertices, vec3f(0, 0, 0));

  if (normalDimensions != 0u) {
    meshData.nbo.resize(numVertices, vec4f(0, 0, 0, 1));
  }

  if (colorDimensions != 0u) {
    meshData.cbo.resize(numVertices, vec4uc(0, 0, 0, 255));
  }

  // Can only be FLOAT32 or UINT8
  const size_t positionBytes = positionDimensions * sizeof(float);  // floats
  const size_t normalBytes = normalDimensions * sizeof(float);      // floats
  const size_t colorBytes = colorDimensions * sizeof(uint8_t);      // bytes

  const size_t vertexPacketSizeBytes = positionBytes + normalBytes + colorBytes;

  size_t positionOffsetBytes = 0;
  size_t normalOffsetBytes = 0;
  size_t colorOffsetBytes = 0;

  size_t offsetSoFarBytes = 0;

  for (size_t i = 0; i < vertexLayout.size(); i++) {
    if (vertexLayout[i] == Properties::POSITION) {
      positionOffsetBytes = offsetSoFarBytes;
      offsetSoFarBytes += positionBytes;
    } else if (vertexLayout[i] == Properties::NORMAL) {
      normalOffsetBytes = offsetSoFarBytes;
      offsetSoFarBytes += normalBytes;
    } else if (vertexLayout[i] == Properties::COLOR) {
      colorOffsetBytes = offsetSoFarBytes;
      offsetSoFarBytes += colorBytes;
    } else {
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
    }
  }

  // Close after parsing header and re-open memory mapped
  const size_t postHeader = file.tellg();

  file.close();

  Cr::Containers::Array<const char, Cr::Utility::Directory::MapDeleter>
      mmappedData = Cr::Utility::Directory::mapRead(filename);

  const size_t fileSize = io::fileSize(filename);

  // Parse each vertex packet and unpack
  const char* bytes = mmappedData + postHeader;

  for (size_t i = 0; i < numVertices; i++) {
    const char* nextBytes = bytes + vertexPacketSizeBytes * i;

    memcpy(meshData.vbo[i].data(), &nextBytes[positionOffsetBytes],
           positionBytes);

    if (normalDimensions != 0u)
      memcpy(meshData.nbo[i].data(), &nextBytes[normalOffsetBytes],
             normalBytes);

    if (colorDimensions != 0u)
      memcpy(meshData.cbo[i].data(), &nextBytes[colorOffsetBytes], colorBytes);
  }

  const size_t bytesSoFar = postHeader + vertexPacketSizeBytes * numVertices;

  bytes = mmappedData + postHeader + vertexPacketSizeBytes * numVertices;

  // Read first face to get number of indices;
  const uint8_t faceDimensions = *bytes;

  CORRADE_ASSERT(faceDimensions == 3 || faceDimensions == 4,
                 "PTexMeshData::parsePLY: the dimension of a face is neither "
                 "3 nor 4.", );

  const size_t countBytes = 1;
  const size_t faceBytes = faceDimensions * sizeof(uint32_t);  // uint32_t
  const size_t facePacketSizeBytes = countBytes + faceBytes;

  const size_t predictedFaces = (fileSize - bytesSoFar) / facePacketSizeBytes;

  // Not sure what to do here
  //    if(predictedFaces < numFaces)
  //    {
  //        ESP_DEBUG()  << "Skipping" << numFaces - predictedFaces  << "missing
  //        faces" << std::endl;
  //    }
  //    else if(numFaces < predictedFaces)
  //    {
  //        ESP_DEBUG()  << "Ignoring" << predictedFaces - numFaces  << "extra
  //        faces" << std::endl;
  //    }

  numFaces = std::min(numFaces, predictedFaces);

  meshData.ibo.resize(numFaces * faceDimensions);

  for (size_t i = 0; i < numFaces; i++) {
    const char* nextBytes = bytes + facePacketSizeBytes * i;

    memcpy(&meshData.ibo[i * faceDimensions], &nextBytes[countBytes],
           faceBytes);
  }
}

void PTexMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  for (int iMesh = 0; iMesh < submeshes_.size(); ++iMesh) {
    ESP_DEBUG() << "Loading mesh" << iMesh + 1 << "/" << submeshes_.size()
                << "...";

    renderingBuffers_.emplace_back(
        std::make_unique<PTexMeshData::RenderingBuffer>());

    auto& currentMesh = renderingBuffers_.back();
    currentMesh->vertexBuffer.setData(submeshes_[iMesh].vbo,
                                      Magnum::GL::BufferUsage::StaticDraw);
    currentMesh->indexBuffer.setData(submeshes_[iMesh].ibo,
                                     Magnum::GL::BufferUsage::StaticDraw);

    // Will it increase the memory usage on GPU? Would it be a big concern?
    //
    // Yes, it will increase the memory footprint, however, the effect is
    // trivial, compared to the volume of the HDR textures.
    //
    // We measured the actual size of this index buffer for every model in the
    // Replica dataset, which ranged from 9MB to 104MB. We also found that, for
    // any Replica model, the volume of the textures was roughly 70x of it.
    // (see the measurement here, in the comments:
    // https://github.com/facebookresearch/habitat-sim/pull/745/files)
    currentMesh->triangleMeshIndexBuffer.setData(
        submeshes_[iMesh].ibo_tri, Magnum::GL::BufferUsage::StaticDraw);
  }
#ifndef CORRADE_TARGET_APPLE
  ESP_DEBUG() << "Calculating mesh adjacency...";

  std::vector<std::vector<uint32_t>> adjFaces(submeshes_.size());

#pragma omp parallel for
  for (int iMesh = 0; iMesh < submeshes_.size(); ++iMesh) {
    calculateAdjacency(submeshes_[iMesh], adjFaces[iMesh]);
  }
#endif

  for (int iMesh = 0; iMesh < submeshes_.size(); ++iMesh) {
    auto& currentMesh = renderingBuffers_[iMesh];

#ifndef CORRADE_TARGET_APPLE
    currentMesh->adjFacesBufferTexture.setBuffer(
        Magnum::GL::BufferTextureFormat::R32UI, currentMesh->adjFacesBuffer);
    currentMesh->adjFacesBuffer.setData(adjFaces[iMesh],
                                        Magnum::GL::BufferUsage::StaticDraw);
#endif
    GLintptr offset = 0;
    currentMesh->mesh
        .setPrimitive(Magnum::GL::MeshPrimitive::LinesAdjacency)
        // Warning:
        // CANNOT use currentMesh->indexBuffer.size() when calling
        // setCount because that returns the number of bytes of the buffer,
        // NOT the index counts
        .setCount(submeshes_[iMesh].ibo.size())
        .addVertexBuffer(currentMesh->vertexBuffer, offset,
                         gfx::PTexMeshShader::Position{})
        .setIndexBuffer(currentMesh->indexBuffer, offset,
                        Magnum::GL::MeshIndexType::UnsignedInt);

    // this triangle mesh will be used for object picking
    currentMesh->triangleMesh
        .setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
        // Warning:
        // CANNOT use currentMesh->indexBuffer.size() when calling
        // setCount because that returns the number of bytes of the buffer,
        // NOT the index counts
        .setCount(submeshes_[iMesh].ibo_tri.size())
        .addVertexBuffer(currentMesh->vertexBuffer, offset,
                         gfx::PTexMeshShader::Position{})
        .setIndexBuffer(currentMesh->triangleMeshIndexBuffer, offset,
                        Magnum::GL::MeshIndexType::UnsignedInt);
  }

  // load atlas data and upload them to GPU
  ESP_DEBUG() << "loading atlas textures:";
  for (size_t iMesh = 0; iMesh < renderingBuffers_.size(); ++iMesh) {
    const std::string hdrFile = Cr::Utility::Directory::join(
        atlasFolder_, std::to_string(iMesh) + "-color-ptex.hdr");

    CORRADE_ASSERT(io::exists(hdrFile),
                   "PTexMeshData::uploadBuffersToGPU: Cannot find the .hdr file"
                       << hdrFile, );

    ESP_DEBUG() << "Loading atlas" << iMesh + 1 << "/"
                << renderingBuffers_.size() << "from" << hdrFile << ".";

    Cr::Containers::Array<const char, Cr::Utility::Directory::MapDeleter> data =
        Cr::Utility::Directory::mapRead(hdrFile);
    // divided by 6, since there are 3 channels, R, G, B, each of which takes
    // 1 half_float (2 bytes)
    const int dim = static_cast<int>(std::sqrt(data.size() / 6));  // square
    CORRADE_ASSERT(dim * dim * 6 == data.size(),
                   "PTexMeshData::uploadBuffersToGPU: the atlas texture is not "
                   "a square", );

    // atlas
    // the size of each image is dim x dim x 3 (RGB) x 2 (half_float), which
    // equals to numBytes
    Magnum::ImageView2D image(Magnum::PixelFormat::RGB16F, {dim, dim}, data);

    renderingBuffers_[iMesh]
        ->atlasTexture.setWrapping(Magnum::GL::SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(Magnum::GL::SamplerFilter::Linear)
        .setMinificationFilter(Magnum::GL::SamplerFilter::Linear)
        .setStorage(
            Magnum::Math::log2(image.size().min()) + 1,  // mip level count
            Magnum::GL::TextureFormat::RGB16F,
            image.size())
        .setSubImage(0,   // mipLevel
                     {},  // offset
                     image)
        .generateMipmap();
  }

  buffersOnGPU_ = true;
}

PTexMeshData::RenderingBuffer* PTexMeshData::getRenderingBuffer(int submeshID) {
  CORRADE_ASSERT(submeshID >= 0 && submeshID < renderingBuffers_.size(),
                 "PTexMeshData::getRenderingBuffer: the submesh ID"
                     << submeshID << "is out of range.",
                 nullptr);
  return renderingBuffers_[submeshID].get();
}

Magnum::GL::Mesh* PTexMeshData::getMagnumGLMesh(int submeshID) {
  CORRADE_ASSERT(submeshID >= 0 && submeshID < renderingBuffers_.size(),
                 "PTexMeshData::getMagnumGLMesh: the submesh ID"
                     << submeshID << "is out of range.",
                 nullptr);
  return &(renderingBuffers_[submeshID]->mesh);
}

}  // namespace assets
}  // namespace esp
