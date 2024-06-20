// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

MaterialTextureAttributes::MaterialTextureAttributes(int index,
                                                     const std::string& handle,
                                                     const std::string& type)
    : AbstractAttributes(type, handle) {
  // index is required
  setIndex(index);
  // set defaults as defined in spec
  setTexCoord(0);
}

void MaterialTextureAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write MaterialTextureAttributes values to json
  writeValueToJson("index", jsonObj, allocator);
  writeValueToJson("texCoord", jsonObj, allocator);
  // handle instance-class specific writing
  writeValuesToJsonInternal(jsonObj, allocator);
}

std::string MaterialTextureAttributes::getObjectInfoHeaderInternal() const {
  return "Texture Index,texCoord Index," +
         getMaterialTextureInfoHeaderInternal();
}

std::string MaterialTextureAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString("{},{},{}", getAsString("index"),
                                   getAsString("texCoord"),
                                   getMaterialTextureInfoInternal());
}  // MaterialTextureAttributes::getObjectInfoInternal()

NormalTextureAttributes::NormalTextureAttributes(int index,
                                                 const std::string& handle)
    : MaterialTextureAttributes(index, handle, "NormalTextureAttributes") {
  // set defaults as defined in spec
  // normalTexture attributes scale of texture normals
  setScale(1.0);
}

OcclusionTextureAttributes::OcclusionTextureAttributes(
    int index,
    const std::string& handle)
    : MaterialTextureAttributes(index, handle, "OcclusionTextureAttributes") {
  // set defaults as defined in spec
  // Ignored except for occlusionTexture attributes - scales red channel of
  // occlusion texture
  setStrength(1.0);
}

PBRMetallicRoughnessAttributes::PBRMetallicRoughnessAttributes(
    const std::string& handle)
    : AbstractAttributes("PBRMetallicRoughnessAttributes", handle) {
  // set defaults
  setBaseColorFactor({1.0f, 1.0f, 1.0f, 1.0f});
  setMetallicFactor(1.0f);
  setRoughnessFactor(1.0f);
}

void PBRMetallicRoughnessAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  if (hasSubconfig("baseColorTexture")) {
    MaterialTextureAttributes::cptr bctSubConfig = getBaseColorTexture();
    auto bctJsonObj = bctSubConfig->writeToJsonObject(allocator);
    jsonObj.AddMember("baseColorTexture", bctJsonObj, allocator);
  }
  writeValueToJson("baseColorFactor", jsonObj, allocator);
  if (hasSubconfig("metallicRoughnessTexture")) {
    MaterialTextureAttributes::cptr mrSubConfig = getMetallicRoughnessTexture();
    auto mrJsonObj = mrSubConfig->writeToJsonObject(allocator);
    jsonObj.AddMember("metallicRoughnessTexture", mrJsonObj, allocator);
  }
  writeValueToJson("metallicFactor", jsonObj, allocator);
  writeValueToJson("roughnessFactor", jsonObj, allocator);
}  // PBRMetallicRoughnessAttributes::writeValuesToJson

std::string PBRMetallicRoughnessAttributes::getObjectInfoInternal() const {
  std::string res("");
  if (hasSubconfig("baseColorTexture")) {
    MaterialTextureAttributes::cptr bctSubConfig = getBaseColorTexture();
    Cr::Utility::formatInto(res, res.size(), "{},",
                            bctSubConfig->getObjectInfo());
  }
  Cr::Utility::formatInto(res, res.size(), "{},",
                          getAsString("baseColorFactor"));
  if (hasSubconfig("metallicRoughnessTexture")) {
    MaterialTextureAttributes::cptr mrSubConfig = getMetallicRoughnessTexture();
    Cr::Utility::formatInto(res, res.size(), "{},",
                            mrSubConfig->getObjectInfo());
  }
  Cr::Utility::formatInto(res, res.size(), "{},{}",
                          getAsString("metallicFactor"),
                          getAsString("roughnessFactor"));
  return res;
}  // PBRMetallicRoughnessAttributes::getObjectInfoInternal

std::string PBRMetallicRoughnessAttributes::getObjectInfoHeaderInternal()
    const {
  std::string res("");
  if (hasSubconfig("baseColorTexture")) {
    MaterialTextureAttributes::cptr bctSubConfig = getBaseColorTexture();
    Cr::Utility::formatInto(res, res.size(), "{},",
                            bctSubConfig->getObjectInfoHeader());
  }
  Cr::Utility::formatInto(res, res.size(), "Base Color Factor,");
  if (hasSubconfig("metallicRoughnessTexture")) {
    MaterialTextureAttributes::cptr mrSubConfig = getMetallicRoughnessTexture();
    Cr::Utility::formatInto(res, res.size(), "{},",
                            mrSubConfig->getObjectInfoHeader());
  }
  Cr::Utility::formatInto(res, res.size(), "Metallic Factor, Roughness Factor");
  return res;
}  // PBRMetallicRoughnessAttributes::getObjectInfoHeaderInternal

MaterialAttributes::MaterialAttributes(const std::string& handle)
    : AbstractAttributes("MaterialAttributes", handle) {
  // set defaults
  setAlphaCutoff(0.5);
  setDoubleSided(false);
  setAlphaMode("OPAQUE");
  setEmissiveFactor({0.0f, 0.0f, 0.0f});
}

void MaterialAttributes::writeValuesToJson(io::JsonGenericValue& jsonObj,
                                           io::JsonAllocator& allocator) const {
  if (hasSubconfig("pbrMetallicRoughness")) {
    PBRMetallicRoughnessAttributes::cptr pbrMetRoughAttr =
        getSubconfigCopy<const PBRMetallicRoughnessAttributes>(
            "pbrMetallicRoughness");
    auto pbrJsonObj = pbrMetRoughAttr->writeToJsonObject(allocator);
    jsonObj.AddMember("pbrMetallicRoughness", pbrJsonObj, allocator);
  }
  if (hasSubconfig("normalTexture")) {
    NormalTextureAttributes::cptr nrmlTxtrAttr =
        getSubconfigCopy<const NormalTextureAttributes>("normalTexture");

    auto nrmlTxtrJsonObj = nrmlTxtrAttr->writeToJsonObject(allocator);
    jsonObj.AddMember("normalTexture", nrmlTxtrJsonObj, allocator);
  }
  if (hasSubconfig("occlusionTexture")) {
    OcclusionTextureAttributes::cptr occlTxtrAttr =
        getSubconfigCopy<const OcclusionTextureAttributes>("occlusionTexture");

    auto oclTxtrJsonObj = occlTxtrAttr->writeToJsonObject(allocator);
    jsonObj.AddMember("occlusionTexture", oclTxtrJsonObj, allocator);
  }
  if (hasSubconfig("emissiveTexture")) {
    MaterialTextureAttributes::cptr emmTxtrAttr =
        getSubconfigCopy<const MaterialTextureAttributes>("emissiveTexture");

    auto emmTxtrJsonObj = emmTxtrAttr->writeToJsonObject(allocator);
    jsonObj.AddMember("emissiveTexture", emmTxtrJsonObj, allocator);
  }

  writeValueToJson("emissiveFactor", jsonObj, allocator);
  writeValueToJson("alphaMode", jsonObj, allocator);
  writeValueToJson("alphaCutoff", jsonObj, allocator);
  writeValueToJson("doubleSided", jsonObj, allocator);

}  // MaterialAttributes::writeValuesToJson

std::string MaterialAttributes::getObjectInfoHeaderInternal() const {
  std::string res("");

  if (hasSubconfig("pbrMetallicRoughness")) {
    PBRMetallicRoughnessAttributes::cptr pbrMetRoughAttr =
        getSubconfigCopy<const PBRMetallicRoughnessAttributes>(
            "pbrMetallicRoughness");
    Cr::Utility::formatInto(res, res.size(), "{},",
                            pbrMetRoughAttr->getObjectInfoHeader());
  }
  if (hasSubconfig("normalTexture")) {
    NormalTextureAttributes::cptr nrmlTxtrAttr =
        getSubconfigCopy<const NormalTextureAttributes>("normalTexture");

    Cr::Utility::formatInto(res, res.size(), "{},",
                            nrmlTxtrAttr->getObjectInfoHeader());
  }
  if (hasSubconfig("occlusionTexture")) {
    OcclusionTextureAttributes::cptr occlTxtrAttr =
        getSubconfigCopy<const OcclusionTextureAttributes>("occlusionTexture");

    Cr::Utility::formatInto(res, res.size(), "{},",
                            occlTxtrAttr->getObjectInfoHeader());
  }
  if (hasSubconfig("emissiveTexture")) {
    MaterialTextureAttributes::cptr emmTxtrAttr =
        getSubconfigCopy<const MaterialTextureAttributes>("emissiveTexture");

    Cr::Utility::formatInto(res, res.size(), "{},",
                            emmTxtrAttr->getObjectInfoHeader());
  }

  Cr::Utility::formatInto(
      res, res.size(),
      "Emissive Factor, Alpha Mode, Alpha Cutoff, Double Sided");

  return res;
}
std::string MaterialAttributes::getObjectInfoInternal() const {
  std::string res = "";

  if (hasSubconfig("pbrMetallicRoughness")) {
    PBRMetallicRoughnessAttributes::cptr pbrMetRoughAttr =
        getSubconfigCopy<const PBRMetallicRoughnessAttributes>(
            "pbrMetallicRoughness");
    Cr::Utility::formatInto(res, res.size(), "{},",
                            pbrMetRoughAttr->getObjectInfo());
  }
  if (hasSubconfig("normalTexture")) {
    NormalTextureAttributes::cptr nrmlTxtrAttr =
        getSubconfigCopy<const NormalTextureAttributes>("normalTexture");

    Cr::Utility::formatInto(res, res.size(), "{},",
                            nrmlTxtrAttr->getObjectInfo());
  }
  if (hasSubconfig("occlusionTexture")) {
    OcclusionTextureAttributes::cptr occlTxtrAttr =
        getSubconfigCopy<const OcclusionTextureAttributes>("occlusionTexture");

    Cr::Utility::formatInto(res, res.size(), "{},",
                            occlTxtrAttr->getObjectInfo());
  }
  if (hasSubconfig("emissiveTexture")) {
    MaterialTextureAttributes::cptr emmTxtrAttr =
        getSubconfigCopy<const MaterialTextureAttributes>("emissiveTexture");

    Cr::Utility::formatInto(res, res.size(), "{},",
                            emmTxtrAttr->getObjectInfo());
  }

  Cr::Utility::formatInto(res, res.size(), "{},{},{},{}",
                          getAsString("emissiveFactor"),
                          getAsString("alphaMode"), getAsString("alphaCutoff"),
                          getAsString("doubleSided"));

  return res;
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
