// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Mp3dInstanceMeshData.h"

#include <fstream>
#include <sstream>
#include <vector>

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/Trade.h>

#include "esp/core/Esp.h"
#include "esp/geo/Geo.h"
#include "esp/io/Io.h"

namespace esp {
namespace assets {

bool Mp3dInstanceMeshData::loadMp3dPLY(const std::string& plyFile) {
  std::ifstream ifs(plyFile);
  if (!ifs.good()) {
    ESP_ERROR() << "Cannot open file at" << plyFile;
    return false;
  }

  std::string line, token;
  std::istringstream iss;
  std::getline(ifs, line);
  if (line != "ply") {
    ESP_ERROR() << "Invalid ply file header";
    return false;
  }
  std::getline(ifs, line);
  if (line != "format binary_little_endian 1.0") {
    ESP_ERROR() << "Invalid ply file header";
    return false;
  }

  // element vertex nVertex
  std::getline(ifs, line);
  iss.str(line);
  iss >> token;
  if (token != "element") {
    ESP_ERROR() << "Invalid element vertex header line";
    return false;
  }
  int nVertex = 0;
  iss >> token >> nVertex;

  // we know the header is fixed so skip until face count
  do {
    std::getline(ifs, line);
  } while ((line.substr(0, 12) != "element face") && !ifs.eof());
  std::stringstream iss2;
  iss2.str(line);
  iss2 >> token;
  if (token != "element") {
    ESP_ERROR() << "Invalid element face header line";
    return false;
  }
  int nFace = 0;
  iss2 >> token >> nFace;

  // ignore rest of header
  do {
    std::getline(ifs, line);
  } while ((line != "end_header") && !ifs.eof());

  cpu_cbo_.clear();
  cpu_cbo_.reserve(nVertex);
  cpu_vbo_.clear();
  cpu_vbo_.reserve(nVertex);
  // per-face vert idxs
  perFaceIdxs_.clear();
  perFaceIdxs_.reserve(nFace);

  for (int i = 0; i < nVertex; ++i) {
    vec3f position;
    vec3f normal;
    vec2f texCoords;
    vec3uc rgb;

    ifs.read(reinterpret_cast<char*>(position.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(normal.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(texCoords.data()), 2 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(rgb.data()), 3 * sizeof(uint8_t));
    cpu_vbo_.emplace_back(position);
    cpu_cbo_.emplace_back(rgb);
  }

  for (int i = 0; i < nFace; ++i) {
    uint8_t nIndices = 0;
    vec3ui indices;
    int32_t materialId = 0;
    int32_t segmentId = 0;
    int32_t categoryId = 0;

    ifs.read(reinterpret_cast<char*>(&nIndices), sizeof(nIndices));
    CORRADE_INTERNAL_ASSERT(nIndices == 3);
    ifs.read(reinterpret_cast<char*>(indices.data()), 3 * sizeof(int));
    ifs.read(reinterpret_cast<char*>(&materialId), sizeof(materialId));
    ifs.read(reinterpret_cast<char*>(&segmentId), sizeof(segmentId));
    ifs.read(reinterpret_cast<char*>(&categoryId), sizeof(categoryId));
    perFaceIdxs_.emplace_back(indices);
    materialIds_.emplace_back(materialId);
    segmentIds_.emplace_back(segmentId);
    categoryIds_.emplace_back(categoryId);
  }

  return true;
}

bool Mp3dInstanceMeshData::saveSemMeshPLY(
    const std::string& plyFile,
    const std::unordered_map<int, int>& segmentIdToObjectIdMap) {
  const int nVertex = cpu_vbo_.size();
  const int nFace = perFaceIdxs_.size();

  std::ofstream f(plyFile, std::ios::out | std::ios::binary);
  f << "ply" << std::endl;
  f << "format binary_little_endian 1.0" << std::endl;
  f << "element vertex " << nVertex << std::endl;
  f << "property float x" << std::endl;
  f << "property float y" << std::endl;
  f << "property float z" << std::endl;
  f << "property uchar red" << std::endl;
  f << "property uchar green" << std::endl;
  f << "property uchar blue" << std::endl;
  f << "element face " << nFace << std::endl;
  f << "property list uchar int vertex_indices" << std::endl;
  f << "property int object_id" << std::endl;
  f << "end_header" << std::endl;

  for (int iVertex = 0; iVertex < nVertex; ++iVertex) {
    const vec3f& xyz = cpu_vbo_[iVertex].head<3>();
    const vec3uc& rgb = cpu_cbo_[iVertex];
    f.write(reinterpret_cast<const char*>(xyz.data()), 3 * sizeof(float));
    f.write(reinterpret_cast<const char*>(rgb.data()), 3 * sizeof(uint8_t));
  }

  for (int iFace = 0; iFace < perFaceIdxs_.size(); ++iFace) {
    const uint8_t nIndices = 3;
    const vec3ui& indices = perFaceIdxs_[iFace];
    // The materialId corresponds to the segmentId from the .house file
    const int32_t segmentId = materialIds_[iFace];
    int32_t objectId = ID_UNDEFINED;
    if (segmentId >= 0) {
      objectId = segmentIdToObjectIdMap.at(segmentId);
    }
    f.write(reinterpret_cast<const char*>(&nIndices), sizeof(nIndices));
    f.write(reinterpret_cast<const char*>(indices.data()),
            3 * sizeof(uint32_t));
    f.write(reinterpret_cast<const char*>(&objectId), sizeof(objectId));
  }
  f.close();

  return true;
}

}  // namespace assets
}  // namespace esp
