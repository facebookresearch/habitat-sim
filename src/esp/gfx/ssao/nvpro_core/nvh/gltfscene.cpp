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

#include "gltfscene.hpp"
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include "boundingbox.hpp"
#include "nvprint.hpp"

namespace nvh {

#define EXTENSION_ATTRIB_IRAY "NV_attributes_iray"

//--------------------------------------------------------------------------------------------------
// Collect the value of all materials
//
void GltfScene::importMaterials(const tinygltf::Model& tmodel) {
  m_materials.reserve(tmodel.materials.size());

  for (auto& tmat : tmodel.materials) {
    GltfMaterial gmat;
    gmat.tmaterial = &tmat;  // Reference

    gmat.alphaCutoff = static_cast<float>(tmat.alphaCutoff);
    gmat.alphaMode =
        tmat.alphaMode == "MASK" ? 1 : (tmat.alphaMode == "BLEND" ? 2 : 0);
    gmat.doubleSided = tmat.doubleSided ? 1 : 0;
    gmat.emissiveFactor =
        tmat.emissiveFactor.size() == 3
            ? nvmath::vec3f(tmat.emissiveFactor[0], tmat.emissiveFactor[1],
                            tmat.emissiveFactor[2])
            : nvmath::vec3f(0.f);
    gmat.emissiveTexture = tmat.emissiveTexture.index;
    gmat.normalTexture = tmat.normalTexture.index;
    gmat.normalTextureScale = static_cast<float>(tmat.normalTexture.scale);
    gmat.occlusionTexture = tmat.occlusionTexture.index;
    gmat.occlusionTextureStrength =
        static_cast<float>(tmat.occlusionTexture.strength);

    // PbrMetallicRoughness
    auto& tpbr = tmat.pbrMetallicRoughness;
    gmat.baseColorFactor =
        nvmath::vec4f(tpbr.baseColorFactor[0], tpbr.baseColorFactor[1],
                      tpbr.baseColorFactor[2], tpbr.baseColorFactor[3]);
    gmat.baseColorTexture = tpbr.baseColorTexture.index;
    gmat.metallicFactor = static_cast<float>(tpbr.metallicFactor);
    gmat.metallicRoughnessTexture = tpbr.metallicRoughnessTexture.index;
    gmat.roughnessFactor = static_cast<float>(tpbr.roughnessFactor);

    // KHR_materials_pbrSpecularGlossiness
    if (tmat.extensions.find(
            KHR_MATERIALS_PBRSPECULARGLOSSINESS_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      gmat.shadingModel = 1;

      const auto& ext =
          tmat.extensions
              .find(KHR_MATERIALS_PBRSPECULARGLOSSINESS_EXTENSION_NAME)
              ->second;
      getVec4(ext, "diffuseFactor", gmat.specularGlossiness.diffuseFactor);
      getFloat(ext, "glossinessFactor",
               gmat.specularGlossiness.glossinessFactor);
      getVec3(ext, "specularFactor", gmat.specularGlossiness.specularFactor);
      getTexId(ext, "diffuseTexture", gmat.specularGlossiness.diffuseTexture);
      getTexId(ext, "specularGlossinessTexture",
               gmat.specularGlossiness.specularGlossinessTexture);
    }

    // KHR_materials_specular
    if (tmat.extensions.find(KHR_MATERIALS_SPECULAR_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      // Since KHR_materials_specular affects the metallic-roughness model:
      gmat.shadingModel = 0;

      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_SPECULAR_EXTENSION_NAME)->second;
      getFloat(ext, "specularFactor", gmat.specular.specularFactor);
      getTexId(ext, "specularTexture", gmat.specular.specularTexture);
      getVec3(ext, "specularColorFactor", gmat.specular.specularColorFactor);
      getTexId(ext, "specularColorTexture", gmat.specular.specularColorTexture);
    }

    // KHR_texture_transform
    if (tpbr.baseColorTexture.extensions.find(
            KHR_TEXTURE_TRANSFORM_EXTENSION_NAME) !=
        tpbr.baseColorTexture.extensions.end()) {
      const auto& ext = tpbr.baseColorTexture.extensions
                            .find(KHR_TEXTURE_TRANSFORM_EXTENSION_NAME)
                            ->second;
      auto& tt = gmat.textureTransform;
      getVec2(ext, "offset", tt.offset);
      getVec2(ext, "scale", tt.scale);
      getFloat(ext, "rotation", tt.rotation);
      getInt(ext, "texCoord", tt.texCoord);

      // Computing the transformation
      auto translation =
          nvmath::mat3f(1, 0, tt.offset.x, 0, 1, tt.offset.y, 0, 0, 1);
      auto rotation =
          nvmath::mat3f(cos(tt.rotation), sin(tt.rotation), 0,
                        -sin(tt.rotation), cos(tt.rotation), 0, 0, 0, 1);
      auto scale = nvmath::mat3f(tt.scale.x, 0, 0, 0, tt.scale.y, 0, 0, 0, 1);
      tt.uvTransform = scale * rotation * translation;
    }

    // KHR_materials_unlit
    if (tmat.extensions.find(KHR_MATERIALS_UNLIT_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      gmat.unlit.active = 1;
    }

    // KHR_materials_anisotropy
    if (tmat.extensions.find(KHR_MATERIALS_ANISOTROPY_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_ANISOTROPY_EXTENSION_NAME)->second;
      getFloat(ext, "anisotropy", gmat.anisotropy.factor);
      getVec3(ext, "anisotropyDirection", gmat.anisotropy.direction);
      getTexId(ext, "anisotropyTexture", gmat.anisotropy.texture);
    }

    // KHR_materials_clearcoat
    if (tmat.extensions.find(KHR_MATERIALS_CLEARCOAT_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_CLEARCOAT_EXTENSION_NAME)->second;
      getFloat(ext, "clearcoatFactor", gmat.clearcoat.factor);
      getTexId(ext, "clearcoatTexture", gmat.clearcoat.texture);
      getFloat(ext, "clearcoatRoughnessFactor", gmat.clearcoat.roughnessFactor);
      getTexId(ext, "clearcoatRoughnessTexture",
               gmat.clearcoat.roughnessTexture);
      getTexId(ext, "clearcoatNormalTexture", gmat.clearcoat.normalTexture);
    }

    // KHR_materials_sheen
    if (tmat.extensions.find(KHR_MATERIALS_SHEEN_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_SHEEN_EXTENSION_NAME)->second;
      getVec3(ext, "sheenColorFactor", gmat.sheen.colorFactor);
      getTexId(ext, "sheenColorTexture", gmat.sheen.colorTexture);
      getFloat(ext, "sheenRoughnessFactor", gmat.sheen.roughnessFactor);
      getTexId(ext, "sheenRoughnessTexture", gmat.sheen.roughnessTexture);
    }

    // KHR_materials_transmission
    if (tmat.extensions.find(KHR_MATERIALS_TRANSMISSION_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_TRANSMISSION_EXTENSION_NAME)
              ->second;
      getFloat(ext, "transmissionFactor", gmat.transmission.factor);
      getTexId(ext, "transmissionTexture", gmat.transmission.texture);
    }

    // KHR_materials_ior
    if (tmat.extensions.find(KHR_MATERIALS_IOR_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_IOR_EXTENSION_NAME)->second;
      getFloat(ext, "ior", gmat.ior.ior);
    }

    // KHR_materials_volume
    if (tmat.extensions.find(KHR_MATERIALS_VOLUME_EXTENSION_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_VOLUME_EXTENSION_NAME)->second;
      getFloat(ext, "thicknessFactor", gmat.volume.thicknessFactor);
      getTexId(ext, "thicknessTexture", gmat.volume.thicknessTexture);
      getFloat(ext, "attenuationDistance", gmat.volume.attenuationDistance);
      getVec3(ext, "attenuationColor", gmat.volume.attenuationColor);
    };

    // KHR_materials_displacement
    if (tmat.extensions.find(KHR_MATERIALS_DISPLACEMENT_NAME) !=
        tmat.extensions.end()) {
      const auto& ext =
          tmat.extensions.find(KHR_MATERIALS_DISPLACEMENT_NAME)->second;
      getTexId(ext, "displacementGeometryTexture",
               gmat.displacement.displacementGeometryTexture);
      getFloat(ext, "displacementGeometryFactor",
               gmat.displacement.displacementGeometryFactor);
      getFloat(ext, "displacementGeometryOffset",
               gmat.displacement.displacementGeometryOffset);
    }

    m_materials.emplace_back(gmat);
  }

  // Make default
  if (m_materials.empty()) {
    GltfMaterial gmat;
    gmat.metallicFactor = 0;
    m_materials.emplace_back(gmat);
  }
}

//--------------------------------------------------------------------------------------------------
// Linearize the scene graph to world space nodes.
//
void GltfScene::importDrawableNodes(
    const tinygltf::Model& tmodel,
    GltfAttributes requestedAttributes,
    GltfAttributes forceRequested /*= GltfAttributes::All*/) {
  checkRequiredExtensions(tmodel);

  int defaultScene = tmodel.defaultScene > -1 ? tmodel.defaultScene : 0;
  const auto& tscene = tmodel.scenes[defaultScene];

  // Finding only the mesh that are used in the scene
  std::set<uint32_t> usedMeshes;
  for (auto nodeIdx : tscene.nodes) {
    findUsedMeshes(tmodel, usedMeshes, nodeIdx);
  }

  // Find the number of vertex(attributes) and index
  // uint32_t nbVert{0};
  uint32_t nbIndex{0};
  uint32_t primCnt{0};  //  "   "  "  "
  for (const auto& m : usedMeshes) {
    auto& tmesh = tmodel.meshes[m];
    std::vector<uint32_t> vprim;
    for (const auto& primitive : tmesh.primitives) {
      if (primitive.mode != 4)  // Triangle
        continue;
      const auto& posAccessor =
          tmodel.accessors[primitive.attributes.find("POSITION")->second];
      // nbVert += static_cast<uint32_t>(posAccessor.count);
      if (primitive.indices > -1) {
        const auto& indexAccessor = tmodel.accessors[primitive.indices];
        nbIndex += static_cast<uint32_t>(indexAccessor.count);
      } else {
        nbIndex += static_cast<uint32_t>(posAccessor.count);
      }
      vprim.emplace_back(primCnt++);
    }
    m_meshToPrimMeshes[m] =
        std::move(vprim);  // mesh-id = { prim0, prim1, ... }
  }

  // Reserving memory
  m_indices.reserve(nbIndex);

  // Convert all mesh/primitives+ to a single primitive per mesh
  for (const auto& m : usedMeshes) {
    auto& tmesh = tmodel.meshes[m];
    for (const auto& tprimitive : tmesh.primitives) {
      processMesh(tmodel, tprimitive, requestedAttributes, forceRequested,
                  tmesh.name);
      m_primMeshes.back().tmesh = &tmesh;
      m_primMeshes.back().tprim = &tprimitive;
    }
  }

  // Transforming the scene hierarchy to a flat list
  for (auto nodeIdx : tscene.nodes) {
    processNode(tmodel, nodeIdx, nvmath::mat4f(1));
  }

  computeSceneDimensions();
  computeCamera();

  m_meshToPrimMeshes.clear();
  primitiveIndices32u.clear();
  primitiveIndices16u.clear();
  primitiveIndices8u.clear();
}

//--------------------------------------------------------------------------------------------------
//
//
void GltfScene::processNode(const tinygltf::Model& tmodel,
                            int& nodeIdx,
                            const nvmath::mat4f& parentMatrix) {
  const auto& tnode = tmodel.nodes[nodeIdx];

  nvmath::mat4f matrix = getLocalMatrix(tnode);
  nvmath::mat4f worldMatrix = parentMatrix * matrix;

  if (tnode.mesh > -1) {
    const auto& meshes =
        m_meshToPrimMeshes[tnode.mesh];  // A mesh could have many primitives
    for (const auto& mesh : meshes) {
      GltfNode node;
      node.primMesh = mesh;
      node.worldMatrix = worldMatrix;
      node.tnode = &tnode;
      m_nodes.emplace_back(node);
    }
  } else if (tnode.camera > -1) {
    GltfCamera camera;
    camera.worldMatrix = worldMatrix;
    camera.cam = tmodel.cameras[tmodel.nodes[nodeIdx].camera];

    // If the node has the Iray extension, extract the camera information.
    if (hasExtension(tnode.extensions, EXTENSION_ATTRIB_IRAY)) {
      auto& iray_ext = tnode.extensions.at(EXTENSION_ATTRIB_IRAY);
      auto& attributes = iray_ext.Get("attributes");
      for (size_t idx = 0; idx < attributes.ArrayLen(); idx++) {
        auto& attrib = attributes.Get((int)idx);
        std::string attName = attrib.Get("name").Get<std::string>();
        auto& attValue = attrib.Get("value");
        if (attValue.IsArray()) {
          auto vec = getVector<float>(attValue);
          if (attName == "iview:position")
            camera.eye = {vec[0], vec[1], vec[2]};
          else if (attName == "iview:interest")
            camera.center = {vec[0], vec[1], vec[2]};
          else if (attName == "iview:up")
            camera.up = {vec[0], vec[1], vec[2]};
        }
      }
    }

    m_cameras.emplace_back(camera);
  } else if (tnode.extensions.find(KHR_LIGHTS_PUNCTUAL_EXTENSION_NAME) !=
             tnode.extensions.end()) {
    GltfLight light;
    const auto& ext =
        tnode.extensions.find(KHR_LIGHTS_PUNCTUAL_EXTENSION_NAME)->second;
    auto lightIdx = ext.Get("light").GetNumberAsInt();
    light.light = tmodel.lights[lightIdx];
    light.worldMatrix = worldMatrix;
    m_lights.emplace_back(light);
  }

  // Recursion for all children
  for (auto child : tnode.children) {
    processNode(tmodel, child, worldMatrix);
  }
}

//--------------------------------------------------------------------------------------------------
// Extracting the values to a linear buffer
//
void GltfScene::processMesh(const tinygltf::Model& tmodel,
                            const tinygltf::Primitive& tmesh,
                            GltfAttributes requestedAttributes,
                            GltfAttributes forceRequested,
                            const std::string& name) {
  // Only triangles are supported
  // 0:point, 1:lines, 2:line_loop, 3:line_strip, 4:triangles, 5:triangle_strip,
  // 6:triangle_fan
  if (tmesh.mode != 4)
    return;

  GltfPrimMesh resultMesh;
  resultMesh.name = name;
  resultMesh.materialIndex = std::max(0, tmesh.material);
  resultMesh.vertexOffset = static_cast<uint32_t>(m_positions.size());
  resultMesh.firstIndex = static_cast<uint32_t>(m_indices.size());

  // Create a key made of the attributes, to see if the primitive was already
  // processed. If it is, we will re-use the cache, but allow the material and
  // indices to be different.
  std::stringstream o;
  for (auto& a : tmesh.attributes) {
    o << a.first << a.second;
  }
  std::string key = o.str();
  bool primMeshCached = false;

  // Found a cache - will not need to append vertex
  auto it = m_cachePrimMesh.find(key);
  if (it != m_cachePrimMesh.end()) {
    primMeshCached = true;
    GltfPrimMesh cacheMesh = it->second;
    resultMesh.vertexCount = cacheMesh.vertexCount;
    resultMesh.vertexOffset = cacheMesh.vertexOffset;
  }

  // INDICES
  if (tmesh.indices > -1) {
    const tinygltf::Accessor& indexAccessor = tmodel.accessors[tmesh.indices];
    resultMesh.indexCount = static_cast<uint32_t>(indexAccessor.count);

    switch (indexAccessor.componentType) {
      case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT: {
        primitiveIndices32u.resize(indexAccessor.count);
        copyAccessorData(primitiveIndices32u, 0, tmodel, indexAccessor, 0,
                         indexAccessor.count);
        m_indices.insert(m_indices.end(), primitiveIndices32u.begin(),
                         primitiveIndices32u.end());
        break;
      }
      case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT: {
        primitiveIndices16u.resize(indexAccessor.count);
        copyAccessorData(primitiveIndices16u, 0, tmodel, indexAccessor, 0,
                         indexAccessor.count);
        m_indices.insert(m_indices.end(), primitiveIndices16u.begin(),
                         primitiveIndices16u.end());
        break;
      }
      case TINYGLTF_PARAMETER_TYPE_UNSIGNED_BYTE: {
        primitiveIndices8u.resize(indexAccessor.count);
        copyAccessorData(primitiveIndices8u, 0, tmodel, indexAccessor, 0,
                         indexAccessor.count);
        m_indices.insert(m_indices.end(), primitiveIndices8u.begin(),
                         primitiveIndices8u.end());
        break;
      }
      default:
        LOGE("Index component type %i not supported!\n",
             indexAccessor.componentType);
        return;
    }
  } else {
    // Primitive without indices, creating them
    const auto& accessor =
        tmodel.accessors[tmesh.attributes.find("POSITION")->second];
    for (auto i = 0; i < accessor.count; i++)
      m_indices.push_back(i);
    resultMesh.indexCount = static_cast<uint32_t>(accessor.count);
  }

  if (primMeshCached == false)  // Need to add this primitive
  {
    // POSITION
    {
      const bool hadPosition =
          getAttribute<nvmath::vec3f>(tmodel, tmesh, m_positions, "POSITION");
      if (!hadPosition) {
        LOGE(
            "This glTF file is invalid: it had a primitive with no POSITION "
            "attribute.\n");
        return;
      }
      // Keeping the size of this primitive (Spec says this is required
      // information)
      const auto& accessor =
          tmodel.accessors[tmesh.attributes.find("POSITION")->second];
      resultMesh.vertexCount = static_cast<uint32_t>(accessor.count);
      if (!accessor.minValues.empty()) {
        resultMesh.posMin =
            nvmath::vec3f(accessor.minValues[0], accessor.minValues[1],
                          accessor.minValues[2]);
      } else {
        resultMesh.posMin = nvmath::vec3f(std::numeric_limits<float>::max());
        for (const auto& p : m_positions) {
          for (int i = 0; i < 3; i++) {
            if (p[i] < resultMesh.posMin[i])
              resultMesh.posMin[i] = p[i];
          }
        }
      }
      if (!accessor.maxValues.empty()) {
        resultMesh.posMax =
            nvmath::vec3f(accessor.maxValues[0], accessor.maxValues[1],
                          accessor.maxValues[2]);
      } else {
        resultMesh.posMax = nvmath::vec3f(-std::numeric_limits<float>::max());
        for (const auto& p : m_positions)
          for (int i = 0; i < 3; i++) {
            if (p[i] > resultMesh.posMax[i])
              resultMesh.posMax[i] = p[i];
          }
      }
    }

    // NORMAL
    if (hasFlag(requestedAttributes, GltfAttributes::Normal)) {
      bool normalCreated =
          getAttribute<nvmath::vec3f>(tmodel, tmesh, m_normals, "NORMAL");

      if (!normalCreated && hasFlag(forceRequested, GltfAttributes::Normal))
        createNormals(resultMesh);
    }

    // TEXCOORD_0
    if (hasFlag(requestedAttributes, GltfAttributes::Texcoord_0)) {
      bool texcoordCreated = getAttribute<nvmath::vec2f>(
          tmodel, tmesh, m_texcoords0, "TEXCOORD_0");
      if (!texcoordCreated)
        texcoordCreated = getAttribute<nvmath::vec2f>(tmodel, tmesh,
                                                      m_texcoords0, "TEXCOORD");
      if (!texcoordCreated &&
          hasFlag(forceRequested, GltfAttributes::Texcoord_0))
        createTexcoords(resultMesh);
    }

    // TANGENT
    if (hasFlag(requestedAttributes, GltfAttributes::Tangent)) {
      bool tangentCreated =
          getAttribute<nvmath::vec4f>(tmodel, tmesh, m_tangents, "TANGENT");

      if (!tangentCreated && hasFlag(forceRequested, GltfAttributes::Tangent))
        createTangents(resultMesh);
    }

    // COLOR_0
    if (hasFlag(requestedAttributes, GltfAttributes::Color_0)) {
      bool colorCreated =
          getAttribute<nvmath::vec4f>(tmodel, tmesh, m_colors0, "COLOR_0");
      if (!colorCreated && hasFlag(forceRequested, GltfAttributes::Color_0))
        createColors(resultMesh);
    }
  }

  // Keep result in cache
  m_cachePrimMesh[key] = resultMesh;

  // Append prim mesh to the list of all primitive meshes
  m_primMeshes.emplace_back(resultMesh);
}  // namespace nvh

void GltfScene::createNormals(GltfPrimMesh& resultMesh) {
  // Need to compute the normals
  std::vector<nvmath::vec3f> geonormal(resultMesh.vertexCount);
  for (size_t i = 0; i < resultMesh.indexCount; i += 3) {
    uint32_t ind0 = m_indices[resultMesh.firstIndex + i + 0];
    uint32_t ind1 = m_indices[resultMesh.firstIndex + i + 1];
    uint32_t ind2 = m_indices[resultMesh.firstIndex + i + 2];
    const auto& pos0 = m_positions[ind0 + resultMesh.vertexOffset];
    const auto& pos1 = m_positions[ind1 + resultMesh.vertexOffset];
    const auto& pos2 = m_positions[ind2 + resultMesh.vertexOffset];
    const auto v1 = nvmath::normalize(
        pos1 - pos0);  // Many normalize, but when objects are really small the
    const auto v2 = nvmath::normalize(
        pos2 -
        pos0);  // cross will go below nv_eps and the normal will be (0,0,0)
    const auto n = nvmath::cross(v1, v2);
    geonormal[ind0] += n;
    geonormal[ind1] += n;
    geonormal[ind2] += n;
  }
  for (auto& n : geonormal)
    n = nvmath::normalize(n);
  m_normals.insert(m_normals.end(), geonormal.begin(), geonormal.end());
}

void GltfScene::createTexcoords(GltfPrimMesh& resultMesh) {
  // Set them all to zero
  //      m_texcoords0.insert(m_texcoords0.end(), resultMesh.vertexCount,
  //      nvmath::vec2f(0, 0));

  // Cube map projection
  for (uint32_t i = 0; i < resultMesh.vertexCount; i++) {
    const auto& pos = m_positions[resultMesh.vertexOffset + i];
    float absX = fabs(pos.x);
    float absY = fabs(pos.y);
    float absZ = fabs(pos.z);

    int isXPositive = pos.x > 0 ? 1 : 0;
    int isYPositive = pos.y > 0 ? 1 : 0;
    int isZPositive = pos.z > 0 ? 1 : 0;

    float maxAxis{}, uc{},
        vc{};  // Zero-initialize in case pos = {NaN, NaN, NaN}

    // POSITIVE X
    if (isXPositive && absX >= absY && absX >= absZ) {
      // u (0 to 1) goes from +z to -z
      // v (0 to 1) goes from -y to +y
      maxAxis = absX;
      uc = -pos.z;
      vc = pos.y;
    }
    // NEGATIVE X
    if (!isXPositive && absX >= absY && absX >= absZ) {
      // u (0 to 1) goes from -z to +z
      // v (0 to 1) goes from -y to +y
      maxAxis = absX;
      uc = pos.z;
      vc = pos.y;
    }
    // POSITIVE Y
    if (isYPositive && absY >= absX && absY >= absZ) {
      // u (0 to 1) goes from -x to +x
      // v (0 to 1) goes from +z to -z
      maxAxis = absY;
      uc = pos.x;
      vc = -pos.z;
    }
    // NEGATIVE Y
    if (!isYPositive && absY >= absX && absY >= absZ) {
      // u (0 to 1) goes from -x to +x
      // v (0 to 1) goes from -z to +z
      maxAxis = absY;
      uc = pos.x;
      vc = pos.z;
    }
    // POSITIVE Z
    if (isZPositive && absZ >= absX && absZ >= absY) {
      // u (0 to 1) goes from -x to +x
      // v (0 to 1) goes from -y to +y
      maxAxis = absZ;
      uc = pos.x;
      vc = pos.y;
    }
    // NEGATIVE Z
    if (!isZPositive && absZ >= absX && absZ >= absY) {
      // u (0 to 1) goes from +x to -x
      // v (0 to 1) goes from -y to +y
      maxAxis = absZ;
      uc = -pos.x;
      vc = pos.y;
    }

    // Convert range from -1 to 1 to 0 to 1
    float u = 0.5f * (uc / maxAxis + 1.0f);
    float v = 0.5f * (vc / maxAxis + 1.0f);

    m_texcoords0.emplace_back(u, v);
  }
}

void GltfScene::createTangents(GltfPrimMesh& resultMesh) {
  // #TODO - Should calculate tangents using default MikkTSpace algorithms
  // See: https://github.com/mmikk/MikkTSpace

  std::vector<nvmath::vec3f> tangent(resultMesh.vertexCount);
  std::vector<nvmath::vec3f> bitangent(resultMesh.vertexCount);

  // Current implementation
  // http://foundationsofgameenginedev.com/FGED2-sample.pdf
  for (size_t i = 0; i < resultMesh.indexCount; i += 3) {
    // local index
    uint32_t i0 = m_indices[resultMesh.firstIndex + i + 0];
    uint32_t i1 = m_indices[resultMesh.firstIndex + i + 1];
    uint32_t i2 = m_indices[resultMesh.firstIndex + i + 2];
    assert(i0 < resultMesh.vertexCount);
    assert(i1 < resultMesh.vertexCount);
    assert(i2 < resultMesh.vertexCount);

    // global index
    uint32_t gi0 = i0 + resultMesh.vertexOffset;
    uint32_t gi1 = i1 + resultMesh.vertexOffset;
    uint32_t gi2 = i2 + resultMesh.vertexOffset;

    const auto& p0 = m_positions[gi0];
    const auto& p1 = m_positions[gi1];
    const auto& p2 = m_positions[gi2];

    const auto& uv0 = m_texcoords0[gi0];
    const auto& uv1 = m_texcoords0[gi1];
    const auto& uv2 = m_texcoords0[gi2];

    nvmath::vec3f e1 = p1 - p0;
    nvmath::vec3f e2 = p2 - p0;

    nvmath::vec2f duvE1 = uv1 - uv0;
    nvmath::vec2f duvE2 = uv2 - uv0;

    float r = 1.0F;
    float a = duvE1.x * duvE2.y - duvE2.x * duvE1.y;
    if (fabs(a) > 0)  // Catch degenerated UV
    {
      r = 1.0f / a;
    }

    nvmath::vec3f t = (e1 * duvE2.y - e2 * duvE1.y) * r;
    nvmath::vec3f b = (e2 * duvE1.x - e1 * duvE2.x) * r;

    tangent[i0] += t;
    tangent[i1] += t;
    tangent[i2] += t;

    bitangent[i0] += b;
    bitangent[i1] += b;
    bitangent[i2] += b;
  }

  for (uint32_t a = 0; a < resultMesh.vertexCount; a++) {
    const auto& t = tangent[a];
    const auto& b = bitangent[a];
    const auto& n = m_normals[resultMesh.vertexOffset + a];

    // Gram-Schmidt orthogonalize
    nvmath::vec3f otangent = nvmath::normalize(t - (nvmath::dot(n, t) * n));

    // In case the tangent is invalid
    if (otangent == nvmath::vec3f(0, 0, 0)) {
      if (abs(n.x) > abs(n.y))
        otangent = nvmath::vec3f(n.z, 0, -n.x) / sqrt(n.x * n.x + n.z * n.z);
      else
        otangent = nvmath::vec3f(0, -n.z, n.y) / sqrt(n.y * n.y + n.z * n.z);
    }

    // Calculate handedness
    float handedness =
        (nvmath::dot(nvmath::cross(n, t), b) < 0.0F) ? 1.0F : -1.0F;
    m_tangents.emplace_back(otangent.x, otangent.y, otangent.z, handedness);
  }
}

void GltfScene::createColors(GltfPrimMesh& resultMesh) {
  // Set them all to one
  m_colors0.insert(m_colors0.end(), resultMesh.vertexCount,
                   nvmath::vec4f(1, 1, 1, 1));
}

//--------------------------------------------------------------------------------------------------
// Return the matrix of the node
//
nvmath::mat4f getLocalMatrix(const tinygltf::Node& tnode) {
  nvmath::mat4f mtranslation{1};
  nvmath::mat4f mscale{1};
  nvmath::mat4f mrot{1};
  nvmath::mat4f matrix{1};
  nvmath::quatf mrotation;

  if (!tnode.translation.empty())
    mtranslation.as_translation(nvmath::vec3f(
        tnode.translation[0], tnode.translation[1], tnode.translation[2]));
  if (!tnode.scale.empty())
    mscale.as_scale(
        nvmath::vec3f(tnode.scale[0], tnode.scale[1], tnode.scale[2]));
  if (!tnode.rotation.empty()) {
    mrotation[0] = static_cast<float>(tnode.rotation[0]);
    mrotation[1] = static_cast<float>(tnode.rotation[1]);
    mrotation[2] = static_cast<float>(tnode.rotation[2]);
    mrotation[3] = static_cast<float>(tnode.rotation[3]);
    mrotation.to_matrix(mrot);
  }
  if (!tnode.matrix.empty()) {
    for (int i = 0; i < 16; ++i)
      matrix.mat_array[i] = static_cast<float>(tnode.matrix[i]);
  }
  return mtranslation * mrot * mscale * matrix;
}

void GltfScene::destroy() {
  m_materials.clear();
  m_nodes.clear();
  m_primMeshes.clear();
  m_cameras.clear();
  m_lights.clear();

  m_positions.clear();
  m_indices.clear();
  m_normals.clear();
  m_tangents.clear();
  m_texcoords0.clear();
  m_texcoords1.clear();
  m_colors0.clear();
  m_cameras.clear();
  // m_joints0.clear();
  // m_weights0.clear();
  m_dimensions = {};
  m_meshToPrimMeshes.clear();
  primitiveIndices32u.clear();
  primitiveIndices16u.clear();
  primitiveIndices8u.clear();
  m_cachePrimMesh.clear();
}

//--------------------------------------------------------------------------------------------------
// Get the dimension of the scene
//
void GltfScene::computeSceneDimensions() {
  Bbox scnBbox;

  for (const auto& node : m_nodes) {
    const auto& mesh = m_primMeshes[node.primMesh];

    Bbox bbox(mesh.posMin, mesh.posMax);
    bbox = bbox.transform(node.worldMatrix);
    scnBbox.insert(bbox);
  }

  if (scnBbox.isEmpty() || !scnBbox.isVolume()) {
    LOGE("glTF: Scene bounding box invalid, Setting to: [-1,-1,-1], [1,1,1]");
    scnBbox.insert({-1.0f, -1.0f, -1.0f});
    scnBbox.insert({1.0f, 1.0f, 1.0f});
  }

  m_dimensions.min = scnBbox.min();
  m_dimensions.max = scnBbox.max();
  m_dimensions.size = scnBbox.extents();
  m_dimensions.center = scnBbox.center();
  m_dimensions.radius = scnBbox.radius();
}

static uint32_t recursiveTriangleCount(
    const tinygltf::Model& model,
    int nodeIdx,
    const std::vector<uint32_t>& meshTriangle) {
  auto& node = model.nodes[nodeIdx];
  uint32_t nbTriangles{0};
  for (const auto child : node.children) {
    nbTriangles += recursiveTriangleCount(model, child, meshTriangle);
  }

  if (node.mesh >= 0)
    nbTriangles += meshTriangle[node.mesh];

  return nbTriangles;
}

//--------------------------------------------------------------------------------------------------
// Retrieving information about the scene
//
GltfStats GltfScene::getStatistics(const tinygltf::Model& tinyModel) {
  GltfStats stats;

  stats.nbCameras = static_cast<uint32_t>(tinyModel.cameras.size());
  stats.nbImages = static_cast<uint32_t>(tinyModel.images.size());
  stats.nbTextures = static_cast<uint32_t>(tinyModel.textures.size());
  stats.nbMaterials = static_cast<uint32_t>(tinyModel.materials.size());
  stats.nbSamplers = static_cast<uint32_t>(tinyModel.samplers.size());
  stats.nbNodes = static_cast<uint32_t>(tinyModel.nodes.size());
  stats.nbMeshes = static_cast<uint32_t>(tinyModel.meshes.size());
  stats.nbLights = static_cast<uint32_t>(tinyModel.lights.size());

  // Computing the memory usage for images
  for (const auto& image : tinyModel.images) {
    stats.imageMem +=
        image.width * image.height * image.component * image.bits / 8;
  }

  // Computing the number of triangles
  std::vector<uint32_t> meshTriangle(tinyModel.meshes.size());
  uint32_t meshIdx{0};
  for (const auto& mesh : tinyModel.meshes) {
    for (const auto& primitive : mesh.primitives) {
      if (primitive.indices > -1) {
        const tinygltf::Accessor& indexAccessor =
            tinyModel.accessors[primitive.indices];
        meshTriangle[meshIdx] += static_cast<uint32_t>(indexAccessor.count) / 3;
      } else {
        const auto& posAccessor =
            tinyModel.accessors[primitive.attributes.find("POSITION")->second];
        meshTriangle[meshIdx] += static_cast<uint32_t>(posAccessor.count) / 3;
      }
    }
    meshIdx++;
  }

  stats.nbUniqueTriangles = std::accumulate(
      meshTriangle.begin(), meshTriangle.end(), 0, std::plus<>());
  for (auto& node : tinyModel.scenes[0].nodes) {
    stats.nbTriangles += recursiveTriangleCount(tinyModel, node, meshTriangle);
  }

  return stats;
}

//--------------------------------------------------------------------------------------------------
// Going through all cameras and find the position and center of interest.
// - The eye or position of the camera is found in the translation part of the
// matrix
// - The center of interest is arbitrary set in front of the camera to a
// distance equivalent
//   to the eye and the center of the scene. If the camera is pointing toward
//   the middle of the scene, the camera center will be equal to the scene
//   center.
// - The up vector is always Y up for now.
//
void GltfScene::computeCamera() {
  for (auto& camera : m_cameras) {
    if (camera.eye ==
        camera.center)  // Applying the rule only for uninitialized camera.
    {
      camera.worldMatrix.get_translation(camera.eye);
      float distance = nvmath::length(m_dimensions.center - camera.eye);
      auto rotMat = camera.worldMatrix.get_rot_mat3();
      camera.center = {0, 0, -distance};
      camera.center = camera.eye + (rotMat * camera.center);
      camera.up = {0, 1, 0};
    }
  }
}

void GltfScene::checkRequiredExtensions(const tinygltf::Model& tmodel) {
  std::set<std::string> supportedExtensions{
      KHR_LIGHTS_PUNCTUAL_EXTENSION_NAME,
      KHR_TEXTURE_TRANSFORM_EXTENSION_NAME,
      KHR_MATERIALS_PBRSPECULARGLOSSINESS_EXTENSION_NAME,
      KHR_MATERIALS_SPECULAR_EXTENSION_NAME,
      KHR_MATERIALS_UNLIT_EXTENSION_NAME,
      KHR_MATERIALS_ANISOTROPY_EXTENSION_NAME,
      KHR_MATERIALS_IOR_EXTENSION_NAME,
      KHR_MATERIALS_VOLUME_EXTENSION_NAME,
      KHR_MATERIALS_TRANSMISSION_EXTENSION_NAME,
      KHR_TEXTURE_BASISU_NAME,
  };

  for (auto& e : tmodel.extensionsRequired) {
    if (supportedExtensions.find(e) == supportedExtensions.end()) {
      LOGE(
          "\n---------------------------------------\n"
          "The extension %s is REQUIRED and not supported \n",
          e.c_str());
    }
  }
}

void GltfScene::findUsedMeshes(const tinygltf::Model& tmodel,
                               std::set<uint32_t>& usedMeshes,
                               int nodeIdx) {
  const auto& node = tmodel.nodes[nodeIdx];
  if (node.mesh >= 0)
    usedMeshes.insert(node.mesh);
  for (const auto& c : node.children)
    findUsedMeshes(tmodel, usedMeshes, c);
}

//--------------------------------------------------------------------------------------------------
//
//
void GltfScene::exportDrawableNodes(tinygltf::Model& tmodel,
                                    GltfAttributes requestedAttributes) {
  // We are rewriting buffer 0, but we will keep the information accessing the
  // other buffers. Information will be added at the end of the function.

  // Copy the buffer view and its position, such that we have a reference from
  // the accessors
  std::unordered_map<uint32_t, tinygltf::BufferView> tempBufferView;
  for (uint32_t i = 0; i < (uint32_t)tmodel.bufferViews.size(); i++) {
    if (tmodel.bufferViews[i].buffer > 0)
      tempBufferView[i] = tmodel.bufferViews[i];
  }
  // Copy all accessors, referencing to one of the temp BufferView
  std::vector<tinygltf::Accessor> tempAccessors;
  for (auto& accessor : tmodel.accessors) {
    if (tempBufferView.find(accessor.bufferView) != tempBufferView.end()) {
      tempAccessors.push_back(accessor);
    }
  }

  // Clear what will the rewritten
  tmodel.bufferViews.clear();
  tmodel.accessors.clear();
  tmodel.scenes[0].nodes.clear();

  // Default assets
  tmodel.asset.copyright = "NVIDIA Corporation";
  tmodel.asset.generator = "glTF exporter";
  tmodel.asset.version = "2.0";  // glTF version 2.0

  bool hasExt =
      std::find(tmodel.extensionsUsed.begin(), tmodel.extensionsUsed.end(),
                EXTENSION_ATTRIB_IRAY) != tmodel.extensionsUsed.end();
  if (!hasExt)
    tmodel.extensionsUsed.emplace_back(EXTENSION_ATTRIB_IRAY);

  // Reset all information in the buffer 0
  tmodel.buffers[0] = {};
  auto& tBuffer = tmodel.buffers[0];
  tBuffer.data.reserve(1000000000);  // Reserving 1 GB to make the allocations
                                     // faster (avoid re-allocations)

  struct OffsetLen {
    uint32_t offset{0};
    uint32_t len{0};
  };
  OffsetLen olIdx, olPos, olNrm, olTan, olCol, olTex;
  olIdx.len = appendData(tBuffer, m_indices);
  olPos.offset = olIdx.offset + olIdx.len;
  olPos.len = appendData(tBuffer, m_positions);
  olNrm.offset = olPos.offset + olPos.len;
  if (hasFlag(requestedAttributes, GltfAttributes::Normal) &&
      !m_normals.empty()) {
    olNrm.len = appendData(tBuffer, m_normals);
  }
  olTex.offset = olNrm.offset + olNrm.len;
  if (hasFlag(requestedAttributes, GltfAttributes::Texcoord_0) &&
      !m_texcoords0.empty()) {
    olTex.len = appendData(tBuffer, m_texcoords0);
  }
  olTan.offset = olTex.offset + olTex.len;
  if (hasFlag(requestedAttributes, GltfAttributes::Tangent) &&
      !m_tangents.empty()) {
    olTan.len = appendData(tBuffer, m_tangents);
  }
  olCol.offset = olTan.offset + olTan.len;
  if (hasFlag(requestedAttributes, GltfAttributes::Color_0) &&
      !m_colors0.empty()) {
    olCol.len = appendData(tBuffer, m_colors0);
  }

  nvmath::matrix4<double> identityMat;
  identityMat.identity();

  // Nodes
  std::vector<tinygltf::Node> tnodes;
  uint32_t nodeID = 0;
  for (auto& n : m_nodes) {
    auto* node = &tnodes.emplace_back();
    node->name = "";
    if (n.tnode) {
      node->name = n.tnode->name;
    }

    // Matrix -- Adding it only if not identity
    {
      bool isIdentity{true};
      std::vector<double> matrix(16);
      for (int i = 0; i < 16; i++) {
        isIdentity &=
            (identityMat.get_value()[i] == n.worldMatrix.get_value()[i]);
        matrix[i] = n.worldMatrix.get_value()[i];
      }
      if (!isIdentity)
        node->matrix = matrix;
    }
    node->mesh = n.primMesh;

    // Adding node to the scene
    tmodel.scenes[0].nodes.push_back(nodeID++);
  }

  // Camera
  if (!m_cameras.empty()) {
    tmodel.cameras.clear();
    auto& camera = tmodel.cameras.emplace_back();
    // Copy the tiny camera
    camera = m_cameras[0].cam;

    // Add camera node
    auto& node = tnodes.emplace_back();
    node.name = "Camera";
    node.camera = 0;
    node.matrix.resize(16);
    for (int i = 0; i < 16; i++)
      node.matrix[i] = m_cameras[0].worldMatrix.get_value()[i];

    // Add extension with pos, interest and up
    auto fct = [](const std::string& name, nvmath::vec3f val) {
      tinygltf::Value::Array tarr;
      tarr.emplace_back(val.x);
      tarr.emplace_back(val.y);
      tarr.emplace_back(val.z);

      tinygltf::Value::Object oattrib;
      oattrib["name"] = tinygltf::Value(name);
      oattrib["type"] = tinygltf::Value(std::string("Float32<3>"));
      oattrib["value"] = tinygltf::Value(tarr);
      return oattrib;
    };

    tinygltf::Value::Array vattrib;
    vattrib.emplace_back(fct("iview:position", m_cameras[0].eye));
    vattrib.emplace_back(fct("iview:interest", m_cameras[0].center));
    vattrib.emplace_back(fct("iview:up", m_cameras[0].up));
    tinygltf::Value::Object oattrib;
    oattrib["attributes"] = tinygltf::Value(vattrib);
    node.extensions[EXTENSION_ATTRIB_IRAY] = tinygltf::Value(oattrib);

    // Add camera to scene
    tmodel.scenes[0].nodes.push_back((int)tnodes.size() - 1);
  }
  tmodel.nodes = tnodes;

  // Mesh/Primitive
  std::vector<tinygltf::Mesh> tmeshes;
  for (const auto& pm : m_primMeshes) {
    Bbox bb;
    {
      for (size_t v_ctx = 0; v_ctx < pm.vertexCount; v_ctx++) {
        size_t idx = pm.vertexOffset + v_ctx;
        bb.insert(m_positions[idx]);
      }
    }

    // Adding a new mesh
    tmeshes.emplace_back();
    auto& tMesh = tmeshes.back();
    tMesh.name = pm.name;

    if (pm.tmesh) {
      tMesh.name = pm.tmesh->name;
      tMesh.extensions = pm.tmesh->extensions;
      tMesh.extensions_json_string = pm.tmesh->extensions_json_string;
    }

    //  primitive
    tMesh.primitives.emplace_back();
    auto& tPrim = tMesh.primitives.back();
    tPrim.mode = TINYGLTF_MODE_TRIANGLES;

    if (pm.tprim) {
      tPrim.extensions = pm.tprim->extensions;
      tPrim.extensions_json_string = pm.tprim->extensions_json_string;
    }

    // Material reference
    tPrim.material = pm.materialIndex;

    // Attributes
    auto& attributes = tPrim.attributes;

    {
      // Buffer View (INDICES)
      tmodel.bufferViews.emplace_back();
      auto& tBufferView = tmodel.bufferViews.back();
      tBufferView.buffer = 0;
      tBufferView.byteOffset = olIdx.offset + pm.firstIndex * sizeof(uint32_t);
      tBufferView.byteStride = 0;  // "bufferView.byteStride must not be defined
                                   // for indices accessor." ;
      tBufferView.byteLength = sizeof(uint32_t) * pm.indexCount;

      // Accessor (INDICES)
      tmodel.accessors.emplace_back();
      auto& tAccessor = tmodel.accessors.back();
      tAccessor.bufferView = static_cast<int>(tmodel.bufferViews.size() - 1);
      tAccessor.byteOffset = 0;
      tAccessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
      tAccessor.count = pm.indexCount;
      tAccessor.type = TINYGLTF_TYPE_SCALAR;

      // The accessor for the indices
      tPrim.indices = static_cast<int>(tmodel.accessors.size() - 1);
    }

    {
      // Buffer View (POSITION)
      tmodel.bufferViews.emplace_back();
      auto& tBufferView = tmodel.bufferViews.back();
      tBufferView.buffer = 0;
      tBufferView.byteOffset =
          olPos.offset + pm.vertexOffset * sizeof(nvmath::vec3f);
      tBufferView.byteStride = 3 * sizeof(float);
      tBufferView.byteLength = pm.vertexCount * tBufferView.byteStride;

      // Accessor (POSITION)
      tmodel.accessors.emplace_back();
      auto& tAccessor = tmodel.accessors.back();
      tAccessor.bufferView = static_cast<int>(tmodel.bufferViews.size() - 1);
      tAccessor.byteOffset = 0;
      tAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      tAccessor.count = pm.vertexCount;
      tAccessor.type = TINYGLTF_TYPE_VEC3;
      tAccessor.minValues = {bb.min()[0], bb.min()[1], bb.min()[2]};
      tAccessor.maxValues = {bb.max()[0], bb.max()[1], bb.max()[2]};
      assert(tAccessor.count > 0);

      attributes["POSITION"] = static_cast<int>(tmodel.accessors.size() - 1);
    }

    if (hasFlag(requestedAttributes, GltfAttributes::Normal) &&
        !m_normals.empty()) {
      // Buffer View (NORMAL)
      tmodel.bufferViews.emplace_back();
      auto& tBufferView = tmodel.bufferViews.back();
      tBufferView.buffer = 0;
      tBufferView.byteOffset =
          olNrm.offset + pm.vertexOffset * sizeof(nvmath::vec3f);
      tBufferView.byteStride = 3 * sizeof(float);
      tBufferView.byteLength = pm.vertexCount * tBufferView.byteStride;

      // Accessor (NORMAL)
      tmodel.accessors.emplace_back();
      auto& tAccessor = tmodel.accessors.back();
      tAccessor.bufferView = static_cast<int>(tmodel.bufferViews.size() - 1);
      tAccessor.byteOffset = 0;
      tAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      tAccessor.count = pm.vertexCount;
      tAccessor.type = TINYGLTF_TYPE_VEC3;

      attributes["NORMAL"] = static_cast<int>(tmodel.accessors.size() - 1);
    }

    if (hasFlag(requestedAttributes, GltfAttributes::Texcoord_0) &&
        !m_texcoords0.empty()) {
      // Buffer View (TEXCOORD_0)
      tmodel.bufferViews.emplace_back();
      auto& tBufferView = tmodel.bufferViews.back();
      tBufferView.buffer = 0;
      tBufferView.byteOffset =
          olTex.offset + pm.vertexOffset * sizeof(nvmath::vec2f);
      tBufferView.byteStride = 2 * sizeof(float);
      tBufferView.byteLength = pm.vertexCount * tBufferView.byteStride;

      // Accessor (TEXCOORD_0)
      tmodel.accessors.emplace_back();
      auto& tAccessor = tmodel.accessors.back();
      tAccessor.bufferView = static_cast<int>(tmodel.bufferViews.size() - 1);
      tAccessor.byteOffset = 0;
      tAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      tAccessor.count = pm.vertexCount;
      tAccessor.type = TINYGLTF_TYPE_VEC2;

      attributes["TEXCOORD_0"] = static_cast<int>(tmodel.accessors.size() - 1);
    }

    if (hasFlag(requestedAttributes, GltfAttributes::Tangent) &&
        !m_tangents.empty()) {
      // Buffer View (TANGENT)
      tmodel.bufferViews.emplace_back();
      auto& tBufferView = tmodel.bufferViews.back();
      tBufferView.buffer = 0;
      tBufferView.byteOffset =
          olTan.offset + pm.vertexOffset * sizeof(nvmath::vec4f);
      tBufferView.byteStride = 4 * sizeof(float);
      tBufferView.byteLength = pm.vertexCount * tBufferView.byteStride;

      // Accessor (TANGENT)
      tmodel.accessors.emplace_back();
      auto& tAccessor = tmodel.accessors.back();
      tAccessor.bufferView = static_cast<int>(tmodel.bufferViews.size() - 1);
      tAccessor.byteOffset = 0;
      tAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      tAccessor.count = pm.vertexCount;
      tAccessor.type = TINYGLTF_TYPE_VEC4;

      attributes["TANGENT"] = static_cast<int>(tmodel.accessors.size() - 1);
    }

    if (hasFlag(requestedAttributes, GltfAttributes::Color_0) &&
        !m_colors0.empty()) {
      // Buffer View (COLOR_O)
      tmodel.bufferViews.emplace_back();
      auto& tBufferView = tmodel.bufferViews.back();
      tBufferView.buffer = 0;
      tBufferView.byteOffset =
          olCol.offset + pm.vertexOffset * sizeof(nvmath::vec4f);
      tBufferView.byteStride = 4 * sizeof(float);
      tBufferView.byteLength = pm.vertexCount * tBufferView.byteStride;

      // Accessor (COLOR_O)
      tmodel.accessors.emplace_back();
      auto& tAccessor = tmodel.accessors.back();
      tAccessor.bufferView = static_cast<int>(tmodel.bufferViews.size() - 1);
      tAccessor.byteOffset = 0;
      tAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
      tAccessor.count = pm.vertexCount;
      tAccessor.type = TINYGLTF_TYPE_VEC4;

      attributes["COLOR_O"] = static_cast<int>(tmodel.accessors.size() - 1);
    }
  }
  tmodel.meshes = tmeshes;

  // Add back accessors for extra buffers (see collection at start of function)
  std::unordered_map<uint32_t, uint32_t> correspondance;
  for (auto& t : tempBufferView) {
    correspondance[t.first] =
        (uint32_t)
            tmodel.bufferViews.size();  // remember position of buffer view
    tmodel.bufferViews.push_back(t.second);
  }
  for (auto& t : tempAccessors) {
    t.bufferView = correspondance[t.bufferView];  // Find from previous buffer
                                                  // view, the new index
    tmodel.accessors.push_back(t);
  }
}

}  // namespace nvh
