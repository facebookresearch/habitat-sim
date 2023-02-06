/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

/**
  \namespace nvh::gltf

  These utilities are for loading glTF models in a
  canonical scene representation. From this representation
  you would create the appropriate 3D API resources (buffers
  and textures).

  \code{.cpp}
  // Typical Usage
  // Load the GLTF Scene using TinyGLTF

  tinygltf::Model    gltfModel;
  tinygltf::TinyGLTF gltfContext;
  fileLoaded = gltfContext.LoadASCIIFromFile(&gltfModel, &error, &warn,
  m_filename);

  // Fill the data in the gltfScene
  gltfScene.getMaterials(tmodel);
  gltfScene.getDrawableNodes(tmodel, GltfAttributes::Normal |
  GltfAttributes::Texcoord_0);

  // Todo in App:
  //   create buffers for vertices and indices, from gltfScene.m_position,
  gltfScene.m_index
  //   create textures from images: using tinygltf directly
  //   create descriptorSet for material using directly gltfScene.m_materials
  \endcode

*/

#pragma once
#include <string.h>
#include <algorithm>
#include <cassert>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "../nvmath/nvmath.h"
#include "tiny_gltf.h"

#define KHR_LIGHTS_PUNCTUAL_EXTENSION_NAME "KHR_lights_punctual"

namespace nvh {

// https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_materials_pbrSpecularGlossiness/README.md
#define KHR_MATERIALS_PBRSPECULARGLOSSINESS_EXTENSION_NAME \
  "KHR_materials_pbrSpecularGlossiness"
struct KHR_materials_pbrSpecularGlossiness {
  nvmath::vec4f diffuseFactor{1.f, 1.f, 1.f, 1.f};
  int diffuseTexture{-1};
  nvmath::vec3f specularFactor{1.f, 1.f, 1.f};
  float glossinessFactor{1.f};
  int specularGlossinessTexture{-1};
};

// https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_specular/README.md
#define KHR_MATERIALS_SPECULAR_EXTENSION_NAME "KHR_materials_specular"
struct KHR_materials_specular {
  float specularFactor{1.f};
  int specularTexture{-1};
  nvmath::vec3f specularColorFactor{1.f, 1.f, 1.f};
  int specularColorTexture{-1};
};

// https://github.com/KhronosGroup/glTF/tree/master/extensions/2.0/Khronos/KHR_texture_transform
#define KHR_TEXTURE_TRANSFORM_EXTENSION_NAME "KHR_texture_transform"
struct KHR_texture_transform {
  nvmath::vec2f offset{0.f, 0.f};
  float rotation{0.f};
  nvmath::vec2f scale{1.f};
  int texCoord{0};
  nvmath::mat3f uvTransform{
      1};  // Computed transform of offset, rotation, scale
};

// https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_materials_clearcoat/README.md
#define KHR_MATERIALS_CLEARCOAT_EXTENSION_NAME "KHR_materials_clearcoat"
struct KHR_materials_clearcoat {
  float factor{0.f};
  int texture{-1};
  float roughnessFactor{0.f};
  int roughnessTexture{-1};
  int normalTexture{-1};
};

// https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_materials_sheen/README.md
#define KHR_MATERIALS_SHEEN_EXTENSION_NAME "KHR_materials_sheen"
struct KHR_materials_sheen {
  nvmath::vec3f colorFactor{0.f, 0.f, 0.f};
  int colorTexture{-1};
  float roughnessFactor{0.f};
  int roughnessTexture{-1};
};

// https://github.com/DassaultSystemes-Technology/glTF/tree/KHR_materials_volume/extensions/2.0/Khronos/KHR_materials_transmission
#define KHR_MATERIALS_TRANSMISSION_EXTENSION_NAME "KHR_materials_transmission"
struct KHR_materials_transmission {
  float factor{0.f};
  int texture{-1};
};

// https://github.com/KhronosGroup/glTF/tree/master/extensions/2.0/Khronos/KHR_materials_unlit
#define KHR_MATERIALS_UNLIT_EXTENSION_NAME "KHR_materials_unlit"
struct KHR_materials_unlit {
  int active{0};
};

// PBR Next : KHR_materials_anisotropy
#define KHR_MATERIALS_ANISOTROPY_EXTENSION_NAME "KHR_materials_anisotropy"
struct KHR_materials_anisotropy {
  float factor{0.f};
  nvmath::vec3f direction{1.f, 0.f, 0.f};
  int texture{-1};
};

// https://github.com/DassaultSystemes-Technology/glTF/tree/KHR_materials_ior/extensions/2.0/Khronos/KHR_materials_ior
#define KHR_MATERIALS_IOR_EXTENSION_NAME "KHR_materials_ior"
struct KHR_materials_ior {
  float ior{1.5f};
};

// https://github.com/DassaultSystemes-Technology/glTF/tree/KHR_materials_volume/extensions/2.0/Khronos/KHR_materials_volume
#define KHR_MATERIALS_VOLUME_EXTENSION_NAME "KHR_materials_volume"
struct KHR_materials_volume {
  float thicknessFactor{0};
  int thicknessTexture{-1};
  float attenuationDistance{std::numeric_limits<float>::max()};
  nvmath::vec3f attenuationColor{1.f, 1.f, 1.f};
};

// https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_texture_basisu/README.md
#define KHR_TEXTURE_BASISU_NAME "KHR_texture_basisu"
struct KHR_texture_basisu {
  int source{-1};
};

// https://github.com/KhronosGroup/glTF/issues/948
#define KHR_MATERIALS_DISPLACEMENT_NAME "KHR_materials_displacement"
struct KHR_materials_displacement {
  float displacementGeometryFactor{1.0f};
  float displacementGeometryOffset{0.0f};
  int displacementGeometryTexture{-1};
};

// https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#reference-material
struct GltfMaterial {
  int shadingModel{0};  // 0: metallic-roughness, 1: specular-glossiness

  // pbrMetallicRoughness
  nvmath::vec4f baseColorFactor{1.f, 1.f, 1.f, 1.f};
  int baseColorTexture{-1};
  float metallicFactor{1.f};
  float roughnessFactor{1.f};
  int metallicRoughnessTexture{-1};

  int emissiveTexture{-1};
  nvmath::vec3f emissiveFactor{0, 0, 0};
  int alphaMode{0};
  float alphaCutoff{0.5f};
  int doubleSided{0};

  int normalTexture{-1};
  float normalTextureScale{1.f};
  int occlusionTexture{-1};
  float occlusionTextureStrength{1};

  // Extensions
  KHR_materials_pbrSpecularGlossiness specularGlossiness;
  KHR_materials_specular specular;
  KHR_texture_transform textureTransform;
  KHR_materials_clearcoat clearcoat;
  KHR_materials_sheen sheen;
  KHR_materials_transmission transmission;
  KHR_materials_unlit unlit;
  KHR_materials_anisotropy anisotropy;
  KHR_materials_ior ior;
  KHR_materials_volume volume;
  KHR_materials_displacement displacement;

  // Tiny Reference
  const tinygltf::Material* tmaterial{nullptr};
};

struct GltfNode {
  nvmath::mat4f worldMatrix{1};
  int primMesh{0};
  const tinygltf::Node* tnode{nullptr};
};

struct GltfPrimMesh {
  uint32_t firstIndex{0};
  uint32_t indexCount{0};
  uint32_t vertexOffset{0};
  uint32_t vertexCount{0};
  int materialIndex{0};

  nvmath::vec3f posMin{0, 0, 0};
  nvmath::vec3f posMax{0, 0, 0};
  std::string name;

  // Tiny Reference
  const tinygltf::Mesh* tmesh{nullptr};
  const tinygltf::Primitive* tprim{nullptr};
};

struct GltfStats {
  uint32_t nbCameras{0};
  uint32_t nbImages{0};
  uint32_t nbTextures{0};
  uint32_t nbMaterials{0};
  uint32_t nbSamplers{0};
  uint32_t nbNodes{0};
  uint32_t nbMeshes{0};
  uint32_t nbLights{0};
  uint32_t imageMem{0};
  uint32_t nbUniqueTriangles{0};
  uint32_t nbTriangles{0};
};

struct GltfCamera {
  nvmath::mat4f worldMatrix{1};
  nvmath::vec3f eye{0, 0, 0};
  nvmath::vec3f center{0, 0, 0};
  nvmath::vec3f up{0, 1, 0};

  tinygltf::Camera cam;
};

// See:
// https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_lights_punctual/README.md
struct GltfLight {
  nvmath::mat4f worldMatrix{1};
  tinygltf::Light light;
};

enum class GltfAttributes : uint8_t {
  NoAttribs = 0,
  Position = 1,
  Normal = 2,
  Texcoord_0 = 4,
  Texcoord_1 = 8,
  Tangent = 16,
  Color_0 = 32,
  All = 0xFF
};

using GltfAttributes_t = std::underlying_type_t<GltfAttributes>;

inline GltfAttributes operator|(GltfAttributes lhs, GltfAttributes rhs) {
  return static_cast<GltfAttributes>(static_cast<GltfAttributes_t>(lhs) |
                                     static_cast<GltfAttributes_t>(rhs));
}

inline GltfAttributes operator&(GltfAttributes lhs, GltfAttributes rhs) {
  return static_cast<GltfAttributes>(static_cast<GltfAttributes_t>(lhs) &
                                     static_cast<GltfAttributes_t>(rhs));
}

//--------------------------------------------------------------------------------------------------
// Class to convert gltfScene in simple draw-able format
//
struct GltfScene {
  // Importing all materials in a vector of GltfMaterial structure
  void importMaterials(const tinygltf::Model& tmodel);

  // Import all Mesh and primitives in a vector of GltfPrimMesh,
  // - Reads all requested GltfAttributes and create them if `forceRequested`
  // contains it.
  // - Create a vector of GltfNode, GltfLight and GltfCamera
  void importDrawableNodes(const tinygltf::Model& tmodel,
                           GltfAttributes requestedAttributes,
                           GltfAttributes forceRequested = GltfAttributes::All);

  void exportDrawableNodes(tinygltf::Model& tmodel,
                           GltfAttributes requestedAttributes);

  // Compute the scene bounding box
  void computeSceneDimensions();

  // Removes everything
  void destroy();

  static GltfStats getStatistics(const tinygltf::Model& tinyModel);

  // Scene data
  std::vector<GltfMaterial> m_materials;   // Material for shading
  std::vector<GltfNode> m_nodes;           // Drawable nodes, flat hierarchy
  std::vector<GltfPrimMesh> m_primMeshes;  // Primitive promoted to meshes
  std::vector<GltfCamera> m_cameras;
  std::vector<GltfLight> m_lights;

  // Attributes, all same length if valid
  std::vector<nvmath::vec3f> m_positions;
  std::vector<uint32_t> m_indices;
  std::vector<nvmath::vec3f> m_normals;
  std::vector<nvmath::vec4f> m_tangents;
  std::vector<nvmath::vec2f> m_texcoords0;
  std::vector<nvmath::vec2f> m_texcoords1;
  std::vector<nvmath::vec4f> m_colors0;

  // #TODO - Adding support for Skinning
  // using vec4us = vector4<unsigned short>;
  // std::vector<vec4us>        m_joints0;
  // std::vector<nvmath::vec4f> m_weights0;

  // Size of the scene
  struct Dimensions {
    nvmath::vec3f min = nvmath::vec3f(std::numeric_limits<float>::max());
    nvmath::vec3f max = nvmath::vec3f(std::numeric_limits<float>::min());
    nvmath::vec3f size{0.f};
    nvmath::vec3f center{0.f};
    float radius{0};
  } m_dimensions;

 private:
  void processNode(const tinygltf::Model& tmodel,
                   int& nodeIdx,
                   const nvmath::mat4f& parentMatrix);
  void processMesh(const tinygltf::Model& tmodel,
                   const tinygltf::Primitive& tmesh,
                   GltfAttributes requestedAttributes,
                   GltfAttributes forceRequested,
                   const std::string& name);

  void createNormals(GltfPrimMesh& resultMesh);
  void createTexcoords(GltfPrimMesh& resultMesh);
  void createTangents(GltfPrimMesh& resultMesh);
  void createColors(GltfPrimMesh& resultMesh);

  // Temporary data
  std::unordered_map<int, std::vector<uint32_t>> m_meshToPrimMeshes;
  std::vector<uint32_t> primitiveIndices32u;
  std::vector<uint16_t> primitiveIndices16u;
  std::vector<uint8_t> primitiveIndices8u;

  std::unordered_map<std::string, GltfPrimMesh> m_cachePrimMesh;

  void computeCamera();
  void checkRequiredExtensions(const tinygltf::Model& tmodel);
  void findUsedMeshes(const tinygltf::Model& tmodel,
                      std::set<uint32_t>& usedMeshes,
                      int nodeIdx);
};

nvmath::mat4f getLocalMatrix(const tinygltf::Node& tnode);

// Return a vector of data for a tinygltf::Value
template <typename T>
static inline std::vector<T> getVector(const tinygltf::Value& value) {
  std::vector<T> result{0};
  if (!value.IsArray())
    return result;
  result.resize(value.ArrayLen());
  for (int i = 0; i < value.ArrayLen(); i++) {
    result[i] =
        static_cast<T>(value.Get(i).IsNumber() ? value.Get(i).Get<double>()
                                               : value.Get(i).Get<int>());
  }
  return result;
}

static inline void getFloat(const tinygltf::Value& value,
                            const std::string& name,
                            float& val) {
  if (value.Has(name)) {
    val = static_cast<float>(value.Get(name).Get<double>());
  }
}

static inline void getInt(const tinygltf::Value& value,
                          const std::string& name,
                          int& val) {
  if (value.Has(name)) {
    val = value.Get(name).Get<int>();
  }
}

static inline void getVec2(const tinygltf::Value& value,
                           const std::string& name,
                           nvmath::vec2f& val) {
  if (value.Has(name)) {
    auto s = getVector<float>(value.Get(name));
    val = nvmath::vec2f{s[0], s[1]};
  }
}

static inline void getVec3(const tinygltf::Value& value,
                           const std::string& name,
                           nvmath::vec3f& val) {
  if (value.Has(name)) {
    auto s = getVector<float>(value.Get(name));
    val = nvmath::vec3f{s[0], s[1], s[2]};
  }
}

static inline void getVec4(const tinygltf::Value& value,
                           const std::string& name,
                           nvmath::vec4f& val) {
  if (value.Has(name)) {
    auto s = getVector<float>(value.Get(name));
    val = nvmath::vec4f{s[0], s[1], s[2], s[3]};
  }
}

static inline void getTexId(const tinygltf::Value& value,
                            const std::string& name,
                            int& val) {
  if (value.Has(name)) {
    val = value.Get(name).Get("index").Get<int>();
  }
}

// Calls a function (such as a lambda function) for each (index, value) pair in
// a sparse accessor. It's only potentially called for indices from
// accessorFirstElement through accessorFirstElement + numElementsToProcess - 1.
template <class T>
void forEachSparseValue(const tinygltf::Model& tmodel,
                        const tinygltf::Accessor& accessor,
                        size_t accessorFirstElement,
                        size_t numElementsToProcess,
                        std::function<void(size_t index, const T* value)> fn) {
  if (!accessor.sparse.isSparse) {
    return;  // Nothing to do
  }

  const auto& idxs = accessor.sparse.indices;
  if (!(idxs.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE      //
        || idxs.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT  //
        || idxs.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)) {
    assert(!"Unsupported sparse accessor index type.");
    return;
  }

  const tinygltf::BufferView& idxBufferView =
      tmodel.bufferViews[idxs.bufferView];
  const unsigned char* idxBuffer =
      &tmodel.buffers[idxBufferView.buffer].data[idxBufferView.byteOffset];
  const size_t idxBufferByteStride =
      idxBufferView.byteStride
          ? idxBufferView.byteStride
          : tinygltf::GetComponentSizeInBytes(idxs.componentType);
  if (idxBufferByteStride == size_t(-1))
    return;  // Invalid

  const auto& vals = accessor.sparse.values;
  const tinygltf::BufferView& valBufferView =
      tmodel.bufferViews[vals.bufferView];
  const unsigned char* valBuffer =
      &tmodel.buffers[valBufferView.buffer].data[valBufferView.byteOffset];
  const size_t valBufferByteStride = accessor.ByteStride(valBufferView);
  if (valBufferByteStride == size_t(-1))
    return;  // Invalid

  // Note that this could be faster for lots of small copies, since we could
  // binary search for the first sparse accessor index to use (since the
  // glTF specification requires the indices be sorted)!
  for (size_t pairIdx = 0; pairIdx < accessor.sparse.count; pairIdx++) {
    // Read the index from the index buffer, converting its type
    size_t index = 0;
    const unsigned char* pIdx = idxBuffer + idxBufferByteStride * pairIdx;
    switch (idxs.componentType) {
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
        index = *reinterpret_cast<const uint8_t*>(pIdx);
        break;
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
        index = *reinterpret_cast<const uint16_t*>(pIdx);
        break;
      case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
        index = *reinterpret_cast<const uint32_t*>(pIdx);
        break;
    }

    // If it's not in range, skip it
    if (index < accessorFirstElement ||
        (index - accessorFirstElement) >= numElementsToProcess) {
      continue;
    }

    fn(index,
       reinterpret_cast<const T*>(valBuffer + valBufferByteStride * pairIdx));
  }
}

// Copies accessor elements accessorFirstElement through
// accessorFirstElement + numElementsToCopy - 1 to outData elements
// outFirstElement through outFirstElement + numElementsToCopy - 1.
// This handles sparse accessors correctly! It's intended as a replacement for
// what would be memcpy(..., &buffer.data[...], ...) calls.
//
// However, it performs no conversion: it assumes (but does not check) that
// accessor's elements are of type T. For instance, T should be a struct of two
// floats for a VEC2 float accessor.
//
// This is range-checked, so elements that would be out-of-bounds are not
// copied. We assume size_t overflow does not occur.
// Note that outDataSizeInT is the number of elements in the outDataBuffer,
// while numElementsToCopy is the number of elements to copy, not the number
// of elements in accessor.
template <class T>
void copyAccessorData(T* outData,
                      size_t outDataSizeInElements,
                      size_t outFirstElement,
                      const tinygltf::Model& tmodel,
                      const tinygltf::Accessor& accessor,
                      size_t accessorFirstElement,
                      size_t numElementsToCopy) {
  if (outFirstElement >= outDataSizeInElements) {
    assert(!"Invalid outFirstElement!");
    return;
  }

  if (accessorFirstElement >= accessor.count) {
    assert(!"Invalid accessorFirstElement!");
    return;
  }

  const tinygltf::BufferView& bufferView =
      tmodel.bufferViews[accessor.bufferView];
  const unsigned char* buffer =
      &tmodel.buffers[bufferView.buffer]
           .data[accessor.byteOffset + bufferView.byteOffset];

  const size_t maxSafeCopySize =
      std::min(accessor.count - accessorFirstElement,
               outDataSizeInElements - outFirstElement);
  numElementsToCopy = std::min(numElementsToCopy, maxSafeCopySize);

  if (bufferView.byteStride == 0) {
    memcpy(outData + outFirstElement,
           reinterpret_cast<const T*>(buffer) + accessorFirstElement,
           numElementsToCopy * sizeof(T));
  } else {
    // Must copy one-by-one
    for (size_t i = 0; i < numElementsToCopy; i++) {
      outData[outFirstElement + i] =
          *reinterpret_cast<const T*>(buffer + bufferView.byteStride * i);
    }
  }

  // Handle sparse accessors by overwriting already copied elements.
  forEachSparseValue<T>(
      tmodel, accessor, accessorFirstElement, numElementsToCopy,
      [&outData](size_t index, const T* value) { outData[index] = *value; });
}

// Same as copyAccessorData(T*, ...), but taking a vector.
template <class T>
void copyAccessorData(std::vector<T>& outData,
                      size_t outFirstElement,
                      const tinygltf::Model& tmodel,
                      const tinygltf::Accessor& accessor,
                      size_t accessorFirstElement,
                      size_t numElementsToCopy) {
  copyAccessorData<T>(outData.data(), outData.size(), outFirstElement, tmodel,
                      accessor, accessorFirstElement, numElementsToCopy);
}

// Appending to \p attribVec, all the values of \p accessor
// Return false if the accessor is invalid.
// T must be nvmath::vec2f, nvmath::vec3f, or nvmath::vec4f.
template <typename T>
static bool getAccessorData(const tinygltf::Model& tmodel,
                            const tinygltf::Accessor& accessor,
                            std::vector<T>& attribVec) {
  // Retrieving the data of the accessor
  const auto nbElems = accessor.count;

  const size_t oldNumElements = attribVec.size();
  attribVec.resize(oldNumElements + nbElems);

  // Copying the attributes
  if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
    copyAccessorData<T>(attribVec, oldNumElements, tmodel, accessor, 0,
                        accessor.count);
  } else {
    // The component is smaller than float and need to be converted
    const auto& bufView = tmodel.bufferViews[accessor.bufferView];
    const auto& buffer = tmodel.buffers[bufView.buffer];
    const unsigned char* bufferByte =
        &buffer.data[accessor.byteOffset + bufView.byteOffset];

    // 2, 3, 4 for VEC2, VEC3, VEC4
    const int nbComponents = tinygltf::GetNumComponentsInType(accessor.type);
    if (nbComponents == -1)
      return false;  // Invalid

    // Stride per element
    const size_t byteStride = accessor.ByteStride(bufView);
    if (byteStride == size_t(-1))
      return false;  // Invalid

    if (!(accessor.componentType == TINYGLTF_COMPONENT_TYPE_BYTE ||
          accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE ||
          accessor.componentType == TINYGLTF_COMPONENT_TYPE_SHORT ||
          accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)) {
      assert(!"Unhandled tinygltf component type!");
      return false;
    }

    const auto& copyElementFn = [&](size_t elementIdx,
                                    const unsigned char* pElement) {
      T vecValue;

      for (int c = 0; c < nbComponents; c++) {
        switch (accessor.componentType) {
          case TINYGLTF_COMPONENT_TYPE_BYTE:
            vecValue[c] = float(*(reinterpret_cast<const char*>(pElement) + c));
            if (accessor.normalized) {
              vecValue[c] = std::max(vecValue[c] / 127.f, -1.f);
            }
            break;
          case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            vecValue[c] =
                float(*(reinterpret_cast<const unsigned char*>(pElement) + c));
            if (accessor.normalized) {
              vecValue[c] = vecValue[c] / 255.f;
            }
            break;
          case TINYGLTF_COMPONENT_TYPE_SHORT:
            vecValue[c] =
                float(*(reinterpret_cast<const short*>(pElement) + c));
            if (accessor.normalized) {
              vecValue[c] = std::max(vecValue[c] / 32767.f, -1.f);
            }
            break;
          case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            vecValue[c] =
                float(*(reinterpret_cast<const unsigned short*>(pElement) + c));
            if (accessor.normalized) {
              vecValue[c] = vecValue[c] / 65535.f;
            }
            break;
        }
      }

      attribVec[oldNumElements + elementIdx] = vecValue;
    };

    for (size_t i = 0; i < nbElems; i++) {
      copyElementFn(i, bufferByte + byteStride * i);
    }

    forEachSparseValue<unsigned char>(tmodel, accessor, 0, nbElems,
                                      copyElementFn);
  }

  return true;
}

// Appending to \p attribVec, all the values of \p attribName
// Return false if the attribute is missing or invalid.
// T must be nvmath::vec2f, nvmath::vec3f, or nvmath::vec4f.
template <typename T>
static bool getAttribute(const tinygltf::Model& tmodel,
                         const tinygltf::Primitive& primitive,
                         std::vector<T>& attribVec,
                         const std::string& attribName) {
  const auto& it = primitive.attributes.find(attribName);
  if (it == primitive.attributes.end())
    return false;
  const auto& accessor = tmodel.accessors[it->second];
  return getAccessorData(tmodel, accessor, attribVec);
}

inline bool hasExtension(const tinygltf::ExtensionMap& extensions,
                         const std::string& name) {
  return extensions.find(name) != extensions.end();
}

// This is appending the incoming data to the binary buffer (just one)
// and return the amount in byte of data that was added.
template <class T>
uint32_t appendData(tinygltf::Buffer& buffer, const T& inData) {
  auto* pData = reinterpret_cast<const char*>(inData.data());
  uint32_t len = static_cast<uint32_t>(sizeof(inData[0]) * inData.size());
  buffer.data.insert(buffer.data.end(), pData, pData + len);
  return len;
}

}  // namespace nvh
