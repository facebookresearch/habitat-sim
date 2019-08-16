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

namespace esp {
namespace assets {

void PTexMeshData::load(const std::string& meshFile,
                        const std::string& atlasFolder) {
  ASSERT(io::exists(meshFile));
  ASSERT(io::exists(atlasFolder));

  // Parse parameters
  // careful: when using "join", no leading forward slash in the file name.
  // otherwise the corrade would think it is absolute path
  const auto& paramsFile =
      Corrade::Utility::Directory::join(atlasFolder, "parameters.json");
  ASSERT(io::exists(paramsFile));
  const io::JsonDocument json = io::parseJsonFile(paramsFile);
  splitSize_ = json["splitSize"].GetDouble();
  tileSize_ = json["tileSize"].GetInt();
  atlasFolder_ = atlasFolder;

  loadMeshData(meshFile);
}

float PTexMeshData::exposure() const {
  return exposure_;
}

void PTexMeshData::setExposure(const float& val) {
  exposure_ = val;
}

float PTexMeshData::gamma() const {
  return gamma_;
}

void PTexMeshData::setGamma(const float& val) {
  gamma_ = val;
}

float PTexMeshData::saturation() const {
  return saturation_;
}

void PTexMeshData::setSaturation(const float& val) {
  saturation_ = val;
}

const std::vector<PTexMeshData::MeshData>& PTexMeshData::meshes() const {
  return submeshes_;
}

std::string PTexMeshData::atlasFolder() const {
  return atlasFolder_;
}

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

    // add referenced vertices to submesh
    subMeshes[i].vbo.resize(refdVerts.size());
    subMeshes[i].nbo.resize(refdVerts.size());
    for (size_t j = 0; j < refdVerts.size(); j++) {
      uint32_t index = refdVerts[j];
      subMeshes[i].vbo[j] = mesh.vbo[index];
      subMeshes[i].nbo[j] = mesh.nbo[index];
    }
  }

  return subMeshes;
}

void PTexMeshData::calculateAdjacency(const PTexMeshData::MeshData& mesh,
                                      std::vector<uint32_t>& adjFaces) {
  struct EdgeData {
    int face;
    int edge;
  };

  std::unordered_map<uint64_t, std::vector<EdgeData>> edgeMap;

  // only works on quad meshes
  const int polygonStride = 4;
  size_t numFaces = mesh.ibo.size() / polygonStride;

  typedef std::unordered_map<uint64_t, std::vector<EdgeData>>::iterator
      EdgeIter;
  std::vector<EdgeIter> edgeIterators(numFaces * polygonStride);

  // for each face
  for (int f = 0; f < numFaces; f++) {
    // for each edge
    for (int e = 0; e < polygonStride; e++) {
      // add to edge to face map
      const int e_index = f * polygonStride + e;
      const uint32_t i0 = mesh.ibo[e_index];
      const uint32_t i1 =
          mesh.ibo[f * polygonStride + ((e + 1) % polygonStride)];
      const uint64_t key =
          (uint64_t)std::min(i0, i1) << 32 | (uint32_t)std::max(i0, i1);

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

  adjFaces.resize(numFaces * polygonStride);

  for (int f = 0; f < numFaces; f++) {
    for (int e = 0; e < polygonStride; e++) {
      const int e_index = f * polygonStride + e;
      auto it = edgeIterators[e_index];
      const std::vector<EdgeData>& adj = it->second;

      // find adjacent face
      int adjFace = -1;
      for (size_t i = 0; i < adj.size(); i++) {
        if (adj[i].face != (int)f)
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
      adjFaces[f * polygonStride + e] =
          (rot << ROTATION_SHIFT) | (adjFace & FACE_MASK);
    }
  }
}

void PTexMeshData::loadMeshData(const std::string& meshFile) {
  PTexMeshData::MeshData originalMesh;

  LOG(INFO) << "Start parsing PLY... ";
  parsePLY(meshFile, originalMesh);
  LOG(INFO) << "Parsing PLY: Done";

  submeshes_.clear();
  if (splitSize_ > 0.0f) {
    LOG(INFO) << "Splitting mesh... ";
    submeshes_ = splitMesh(originalMesh, splitSize_);
    LOG(INFO) << "Splitting mesh: Done";
  } else {
    submeshes_.emplace_back(std::move(originalMesh));
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
        ASSERT(s == "binary_little_endian");
      } else if (token == "element") {
        std::string name;
        size_t size;
        ls >> name >> size;

        if (name == "vertex") {
          // Pull out the number of vertices
          numVertices = size;
        } else if (name == "face") {
          // Pull out number of faces
          numFaces = size;
          ASSERT(numFaces > 0);
        } else {
          ASSERT(false, "Can't parse element (%)", name);
        }

        // Keep track of what element we parsed last to associate the properties
        // that follow
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

          ASSERT(countType == "uchar" || countType == "uint8",
                 "Don't understand count type (%)", countType);

          ASSERT(type == "int", "Don't understand index type (%)", type);

          ASSERT(lastElement == "face",
                 "Only expecting list after face element, not after (%)",
                 lastElement);
        }

        ASSERT(type == "float" || type == "int" || type == "uchar" ||
                   type == "uint8",
               "Don't understand type (%)", type);

        ls >> name;

        // Collecting vertex property information
        if (lastElement == "vertex") {
          ASSERT(type != "int", "Don't support 32-bit integer properties");

          // Position information
          if (name == "x") {
            positionDimensions = 1;
            vertexLayout.push_back(Properties::POSITION);
            ASSERT(type == "float", "Don't support 8-bit integer positions");
          } else if (name == "y") {
            ASSERT(lastProperty == "x",
                   "Properties should follow x, y, z, (w) order");
            positionDimensions = 2;
          } else if (name == "z") {
            ASSERT(lastProperty == "y",
                   "Properties should follow x, y, z, (w) order");
            positionDimensions = 3;
          } else if (name == "w") {
            ASSERT(lastProperty == "z",
                   "Properties should follow x, y, z, (w) order");
            positionDimensions = 4;
          }

          // Normal information
          if (name == "nx") {
            normalDimensions = 1;
            vertexLayout.push_back(Properties::NORMAL);
            ASSERT(type == "float", "Don't support 8-bit integer normals");
          } else if (name == "ny") {
            ASSERT(lastProperty == "nx",
                   "Properties should follow nx, ny, nz order");
            normalDimensions = 2;
          } else if (name == "nz") {
            ASSERT(lastProperty == "ny",
                   "Properties should follow nx, ny, nz order");
            normalDimensions = 3;
          }

          // Color information
          if (name == "red") {
            colorDimensions = 1;
            vertexLayout.push_back(Properties::COLOR);
            ASSERT(type == "uchar" || type == "uint8",
                   "Don't support non-8-bit integer colors");
          } else if (name == "green") {
            ASSERT(lastProperty == "red",
                   "Properties should follow red, green, blue, (alpha) order");
            colorDimensions = 2;
          } else if (name == "blue") {
            ASSERT(lastProperty == "green",
                   "Properties should follow red, green, blue, (alpha) order");
            colorDimensions = 3;
          } else if (name == "alpha") {
            ASSERT(lastProperty == "blue",
                   "Properties should follow red, green, blue, (alpha) order");
            colorDimensions = 4;
          }
        } else if (lastElement == "face") {
          ASSERT(isList, "No idea what to do with properties following faces");
        } else {
          ASSERT(false, "No idea what to do with properties before elements");
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
        ASSERT(false);
      }
    }

    // Check things make sense.
    ASSERT(numVertices > 0);
    ASSERT(positionDimensions > 0);
  }

  meshData.vbo.resize(numVertices, vec4f(0, 0, 0, 1));

  if (normalDimensions) {
    meshData.nbo.resize(numVertices, vec4f(0, 0, 0, 1));
  }

  if (colorDimensions) {
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
      ASSERT(false);
    }
  }

  // Close after parsing header and re-open memory mapped
  const size_t postHeader = file.tellg();

  file.close();

  Corrade::Containers::Array<const char,
                             Corrade::Utility::Directory::MapDeleter>
      mmappedData = Corrade::Utility::Directory::mapRead(filename);

  const size_t fileSize = io::fileSize(filename);

  // Parse each vertex packet and unpack
  const char* bytes = mmappedData + postHeader;

  for (size_t i = 0; i < numVertices; i++) {
    const char* nextBytes = bytes + vertexPacketSizeBytes * i;

    memcpy(meshData.vbo[i].data(), &nextBytes[positionOffsetBytes],
           positionBytes);

    if (normalDimensions)
      memcpy(meshData.nbo[i].data(), &nextBytes[normalOffsetBytes],
             normalBytes);

    if (colorDimensions)
      memcpy(meshData.cbo[i].data(), &nextBytes[colorOffsetBytes], colorBytes);
  }

  const size_t bytesSoFar = postHeader + vertexPacketSizeBytes * numVertices;

  bytes = mmappedData + postHeader + vertexPacketSizeBytes * numVertices;

  // Read first face to get number of indices;
  const uint8_t faceDimensions = *bytes;

  ASSERT(faceDimensions == 3 || faceDimensions == 4);

  const size_t countBytes = 1;
  const size_t faceBytes = faceDimensions * sizeof(uint32_t);  // uint32_t
  const size_t facePacketSizeBytes = countBytes + faceBytes;

  const size_t predictedFaces = (fileSize - bytesSoFar) / facePacketSizeBytes;

  // Not sure what to do here
  //    if(predictedFaces < numFaces)
  //    {
  //        LOG(INFO) << "Skipping " << numFaces - predictedFaces << " missing
  //        faces" << std::endl;
  //    }
  //    else if(numFaces < predictedFaces)
  //    {
  //        LOG(INFO) << "Ignoring " << predictedFaces - numFaces << " extra
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

bool PTexMeshData::loadAdjacency(const std::string& filename,
                                 std::vector<std::vector<uint32_t>>& adjFaces) {
  if (!io::exists(filename)) {
    return false;
  }
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  uint64_t numSubMeshes = 0;
  file.read((char*)&numSubMeshes, sizeof(uint64_t));
  adjFaces.resize(numSubMeshes);

  for (uint64_t iMesh = 0; iMesh < numSubMeshes; ++iMesh) {
    uint64_t numAdjFaces = 0;
    file.read((char*)&numAdjFaces, sizeof(uint64_t));
    adjFaces[iMesh].resize(numAdjFaces);
    uint32_t* data = adjFaces[iMesh].data();
    file.read((char*)data, sizeof(uint32_t) * numAdjFaces);
  }

  file.close();
  return true;
}

void PTexMeshData::saveAdjacency(const std::string& filename,
                                 std::vector<std::vector<uint32_t>>& adjFaces) {
  std::ofstream file;
  file.open(filename, std::ios::out | std::ios::binary);
  if (!file.good()) {
    LOG(INFO) << "Error: cannot open " << filename << " to save the adjacency.";
    return;
  }

  // storage:
  // a uint64_t, to save the number of submeshes (N);
  // followed by N items, each of which:
  // a uint64_t, to save the number of adjacent faces (M) of submesh_j, then
  // M uint32_t numbers, each of which is the adjacent face ID

  uint64_t numSubMeshes = adjFaces.size();
  file.write((char*)&numSubMeshes, sizeof(uint64_t));

  for (uint64_t iMesh = 0; iMesh < numSubMeshes; ++iMesh) {
    uint64_t numAdjFaces = adjFaces[iMesh].size();
    file.write((char*)&numAdjFaces, sizeof(uint64_t));
    file.write((char*)adjFaces[iMesh].data(), sizeof(uint32_t) * numAdjFaces);
  }

  file.close();
}

void PTexMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  for (int iMesh = 0; iMesh < submeshes_.size(); ++iMesh) {
    LOG(INFO) << "Loading mesh " << iMesh + 1 << "/" << submeshes_.size()
              << ". ";

    renderingBuffers_.emplace_back(
        std::make_unique<PTexMeshData::RenderingBuffer>());

    auto& currentMesh = renderingBuffers_.back();
    currentMesh->vbo.setData(submeshes_[iMesh].vbo,
                             Magnum::GL::BufferUsage::StaticDraw);
    currentMesh->ibo.setData(submeshes_[iMesh].ibo,
                             Magnum::GL::BufferUsage::StaticDraw);
  }

  LOG(INFO) << "Calculating mesh adjacency... ";
  std::vector<std::vector<uint32_t>> adjFaces(submeshes_.size());

  // load it if it is computed before.
  // otherwise compute it once and save it for future usage.
  const std::string adjFaceFilename =
      Corrade::Utility::Directory::join(atlasFolder_, "../adjFaces.bin");
  if (!loadAdjacency(adjFaceFilename, adjFaces)) {
#pragma omp parallel for
    for (int iMesh = 0; iMesh < submeshes_.size(); ++iMesh) {
      calculateAdjacency(submeshes_[iMesh], adjFaces[iMesh]);
    }
    // Warning: you should have enough disk space to store the info
    // it usually takes a couple of 100MB (usually 200+MB).
    saveAdjacency(adjFaceFilename, adjFaces);
    LOG(INFO) << "Done: it is computed and saved to: " << adjFaceFilename;
  } else {
    LOG(INFO) << "Done: Loaded it from " << adjFaceFilename;
  }

  for (int iMesh = 0; iMesh < submeshes_.size(); ++iMesh) {
    auto& currentMesh = renderingBuffers_[iMesh];

    currentMesh->adjFaces.setBuffer(Magnum::GL::BufferTextureFormat::R32UI,
                                    currentMesh->abo);
    currentMesh->abo.setData(adjFaces[iMesh],
                             Magnum::GL::BufferUsage::StaticDraw);

    // using GL_LINES_ADJACENCY here to send quads to geometry shader
    currentMesh->mesh.setPrimitive(Magnum::GL::MeshPrimitive::LinesAdjacency)
        .setCount(submeshes_[iMesh].ibo.size())
        .addVertexBuffer(currentMesh->vbo, 0, gfx::PTexMeshShader::Position{})
        .setIndexBuffer(currentMesh->ibo, 0,
                        Magnum::GL::MeshIndexType::UnsignedInt);
  }

  // load atlas data and upload them to GPU
  LOG(INFO) << "loading atlas textures: ";
  for (size_t iMesh = 0; iMesh < renderingBuffers_.size(); ++iMesh) {
    const std::string hdrFile = Corrade::Utility::Directory::join(
        atlasFolder_, std::to_string(iMesh) + "-color-ptex.hdr");

    ASSERT(io::exists(hdrFile), Error : Cannot find the.hdr file);

    LOG(INFO) << "Loading atlas " << iMesh + 1 << "/"
              << renderingBuffers_.size() << " from " << hdrFile << ". ";

    Corrade::Containers::Array<const char,
                               Corrade::Utility::Directory::MapDeleter>
        data = Corrade::Utility::Directory::mapRead(hdrFile);
    // divided by 6, since there are 3 channels, R, G, B, each of which takes 1
    // half_float (2 bytes)
    const int dim = static_cast<int>(std::sqrt(data.size() / 6));  // square

    // atlas
    // the size of each image is dim x dim x 3 (RGB) x 2 (half_float), which
    // equals to numBytes
    Magnum::ImageView2D image(Magnum::PixelFormat::RGB16F, {dim, dim}, data);
    const int mipLevelCount = 1;
    renderingBuffers_[iMesh]
        ->tex.setWrapping(Magnum::GL::SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(Magnum::GL::SamplerFilter::Linear)
        .setMinificationFilter(Magnum::GL::SamplerFilter::Linear)
        .setStorage(mipLevelCount, Magnum::GL::TextureFormat::RGB16F,
                    image.size())
        .setSubImage(0, {}, image);
  }
  buffersOnGPU_ = true;
  LOG(INFO) << "data are uploaded to GPU.";
}

PTexMeshData::RenderingBuffer* PTexMeshData::getRenderingBuffer(int submeshID) {
  ASSERT(submeshID >= 0 && submeshID < renderingBuffers_.size());
  return renderingBuffers_[submeshID].get();
}

Magnum::GL::Mesh* PTexMeshData::getMagnumGLMesh(int submeshID) {
  ASSERT(submeshID >= 0 && submeshID < renderingBuffers_.size());
  return &(renderingBuffers_[submeshID]->mesh);
}

}  // namespace assets
}  // namespace esp
