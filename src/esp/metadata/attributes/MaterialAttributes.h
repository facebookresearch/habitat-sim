// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_MATERIALATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_MATERIALATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief This class describes the texture references within a particular
 * material definition.  This follows the gltf 2.0 spec.
 */
class MaterialTextureAttributes : public AbstractAttributes {
 public:
  explicit MaterialTextureAttributes(
      int index,
      const std::string& handle,
      const std::string& type = "MaterialTextureAttributes");

  /** @brief Set the index of this texture. */
  void setIndex(int index) { set("index", index); }

  /** @brief Get the index of this texture. */
  int getIndex() const { return get<int>("index"); }

  /** @brief Set the texCoord mesh attribute index of this texture. */
  void setTexCoord(int texCoord) { set("texCoord", texCoord); }

  /** @brief Get the texCoord mesh attribute index of this texture.  */
  int getTexCoord() const { return get<int>("texCoord"); }

  /**
   * @brief Populate a Json object with all the first-level values held in this
   * MaterialTextureAttributes. Default is overridden to handle special
   * cases for MaterialTextureAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */
  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Write child-class-specific Json data.
   */
  virtual void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                         io::JsonAllocator& allocator) const {}

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object. To support normal and occlusion texture attributes
   */
  virtual std::string getMaterialTextureInfoInternal() const { return ""; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object. To support normal and occlusion texture attributes
   */
  virtual std::string getMaterialTextureInfoHeaderInternal() const {
    return "";
  }

 public:
  ESP_SMART_POINTERS(MaterialTextureAttributes)
};

/**
 * @brief This class describes the normalTexture references within a particular
 * material definition.  This follows the gltf 2.0 spec.
 */
class NormalTextureAttributes : public MaterialTextureAttributes {
 public:
  explicit NormalTextureAttributes(int index, const std::string& handle);

  /** @brief Set the scale to be used for this (normal) texture.
   * Texture-defined normals are scaled following this eq :
   * `scaledNormal =
   *    normalize((<sampled normal texture value> * 2.0 - 1.0) *
   *    vec3(<normal scale>, <normal scale>, 1.0))`
   */
  void setScale(double scale) { set("scale", scale); }

  /** @brief Get the scale to be used for this (normal) texture.
   * Texture-defined normals are scaled following this eq :
   * `scaledNormal =
   *    normalize((<sampled normal texture value> * 2.0 - 1.0) *
   *    vec3(<normal scale>, <normal scale>, 1.0))`
   */
  double getScale() const { return get<double>("scale"); }

 protected:
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object. To support normal and occlusion texture attributes
   */
  std::string getMaterialTextureInfoInternal() const override {
    return Cr::Utility::formatString("{},", getAsString("scale"));
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object. To support normal and occlusion texture attributes
   */
  std::string getMaterialTextureInfoHeaderInternal() const override {
    return "Normal XY Scale,";
  }

  /**
   * @brief Write NormalTextureAttributes-specific Json data.
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override {
    writeValueToJson("scale", jsonObj, allocator);
  }

 public:
  ESP_SMART_POINTERS(NormalTextureAttributes)
};  // class NormalTextureAttributes

/**
 * @brief This class describes the occlusionTexture references within a
 * particular material definition.  This follows the gltf 2.0 spec.
 */
class OcclusionTextureAttributes : public MaterialTextureAttributes {
 public:
  explicit OcclusionTextureAttributes(int index, const std::string& handle);

  /** @brief Set the strength to be used for this (occlusion) texture.
   * The occlusion values are linearly sampled from the R channel of the
   * texture. Higher values indicate areas that receive full indirect lighting
   * and lower values indicate no indirect lighting. Values in texture are
   * scaled by strength using this EQ :
   *
   * `1.0 + strength * (<sampled occlusion texture value> - 1.0)`
   */
  void setStrength(double strength) { set("strength", strength); }

  /** @brief Get the strength to be used for this (occlusion) texture.
   * The occlusion values are linearly sampled from the R channel of the
   * texture. Higher values indicate areas that receive full indirect lighting
   * and lower values indicate no indirect lighting.Values in texture are
   * scaled by strength using this EQ :
   *
   * `1.0 + strength * (<sampled occlusion texture value> - 1.0)`
   */
  double getStrength() const { return get<double>("strength"); }

 protected:
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object. To support normal and occlusion texture attributes
   */
  std::string getMaterialTextureInfoInternal() const override {
    return Cr::Utility::formatString("{},", getAsString("strength"));
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object. To support normal and occlusion texture attributes
   */
  std::string getMaterialTextureInfoHeaderInternal() const override {
    return "Occlusion Strength,";
  }

  /**
   * @brief Write OcclusionTextureAttributes-specific Json data.
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override {
    writeValueToJson("strength", jsonObj, allocator);
  }

 public:
  ESP_SMART_POINTERS(OcclusionTextureAttributes)
};  // class OcclusionTextureAttributes

/**
 * @brief This class defines the component of the material attributes relating
 * to the bpr base color, metalness, and roughness.  This corresponds to the
 * pbrMetallicRoughness Json object in the gltf 2.0 spec.
 */
class PBRMetallicRoughnessAttributes : public AbstractAttributes {
 public:
  explicit PBRMetallicRoughnessAttributes(const std::string& handle);

  /**
   * @brief Set the description of the @ref MaterialTextureAttributes for
   * the BaseColorTexture for this material. There will be only one
   * specified.
   */
  void setBaseColorTexture(MaterialTextureAttributes::ptr _baseColorTexture) {
    // force ID to be 0, there will be only 1 baseColorTexture defined for any
    // PBRMetallicRoughnessAttributes object.
    _baseColorTexture->setID(0);
    setSubconfigPtr<MaterialTextureAttributes>("baseColorTexture",
                                               _baseColorTexture);
  }

  /**
   * @brief Get a shared_pointer to the @ref MaterialTextureAttributes
   * descibing the BaseColorTexture specified for this material.
   */
  MaterialTextureAttributes::cptr getBaseColorTexture() const {
    return getSubconfigCopy<const MaterialTextureAttributes>(
        "baseColorTexture");
  }

  /**
   * @brief Set the description of the @ref MaterialTextureAttributes for the
   * MetallicRoughtnessTexture for this material. There will be only one
   * specified.
   */
  void setMetallicRoughnessTexture(
      MaterialTextureAttributes::ptr _metallicRoughnessTexture) {
    // force ID to be 0, there will be only 1 metallicRoughnessTexture defined
    // for any PBRMetallicRoughnessAttributes object.
    _metallicRoughnessTexture->setID(0);
    setSubconfigPtr<MaterialTextureAttributes>("metallicRoughnessTexture",
                                               _metallicRoughnessTexture);
  }

  /**
   * @brief Get a shared_pointer to the @ref MaterialTextureAttributes
   * descibing the MetallicRoughtnessTexture specified for this material.
   */
  MaterialTextureAttributes::cptr getMetallicRoughnessTexture() const {
    return getSubconfigCopy<const MaterialTextureAttributes>(
        "metallicRoughnessTexture");
  }

  /**
   * @brief Set the base color factor for this PBRMetallicRoughnessAttributes.
   * Default : [1.0f,1.0f,1.0f,1.0f]
   * @param baseColorFactor Linear multiplier of baseColorTexture. Treated as
   * object color if no texture exists
   */
  void setBaseColorFactor(const Mn::Color4& baseColorFactor) {
    set("baseColorFactor", baseColorFactor);
  }

  /**
   * @brief Get the base color factor for this PBRMetallicRoughnessAttributes.
   * Default : [1.0f,1.0f,1.0f,1.0f]
   * @return Linear multiplier of baseColorTexture. Treated as
   * object color if no texture exists
   */
  Mn::Color4 getBaseColorFactor() const {
    return get<Mn::Color4>("baseColorFactor");
  }

  /**
   * Set factor for the metalness of the material. This value defines a linear
   * multiplier for the sampled metalness values of the metallic-roughness
   * texture, if present. DEFAULT: 1.0
   */
  void setMetallicFactor(double metallicFactor) {
    set("metallicFactor", metallicFactor);
  }

  /**
   * Get factor for the metalness of the material. This value defines a linear
   * multiplier for the sampled metalness values of the metallic-roughness
   * texture, if present. DEFAULT: 1.0
   */
  double getMetallicFactor() const { return get<double>("metallicFactor"); }

  /**
   * Set factor for the roughness of the material. This value defines a linear
   * multiplier for the sampled roughness values of the metallic-roughness
   * texture, if present. DEFAULT: 1.0
   */
  void setRoughnessFactor(double roughnessFactor) {
    set("roughnessFactor", roughnessFactor);
  }

  /**
   * Get factor for the roughness of the material. This value defines a linear
   * multiplier for the sampled roughness values of the metallic-roughness
   * texture. DEFAULT: 1.0
   */
  double getRoughnessFactor() const { return get<double>("roughnessFactor"); }

  /**
   * @brief Populate a Json object with all the first-level values held in this
   * PBRMetallicRoughnessAttributes. Default is overridden to handle special
   * cases for PBRMetallicRoughnessAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */
  std::string getObjectInfoHeaderInternal() const override;

 public:
  ESP_SMART_POINTERS(PBRMetallicRoughnessAttributes)
};  // class PBRMetallicRoughnessAttributes

class MaterialAttributes : public AbstractAttributes {
 public:
  explicit MaterialAttributes(const std::string& handle = "");

  /**
   * @brief Set the description of the @ref PBRMetallicRoughnessAttributes for
   * this material. There will be only one specified.
   */
  void setPBRMetallicRoughness(
      PBRMetallicRoughnessAttributes::ptr _pbrMetallicRoughness) {
    _pbrMetallicRoughness->setID(0);
    setSubconfigPtr<PBRMetallicRoughnessAttributes>("pbrMetallicRoughness",
                                                    _pbrMetallicRoughness);
  }

  /**
   * @brief Get a shared_pointer to the @ref PBRMetallicRoughnessAttributes
   * descibing the set of parameters describing the color and metallic-roughness
   * for this material.
   */
  PBRMetallicRoughnessAttributes::cptr getPBRMetallicRoughness() const {
    return getSubconfigCopy<const PBRMetallicRoughnessAttributes>(
        "pbrMetallicRoughness");
  }

  /**
   * @brief Set the description of the @ref OcclusionTextureAttributes for this
   * material. There will be only one specified.
   */
  void setOcclusionTexture(OcclusionTextureAttributes::ptr _occlusionTexture) {
    _occlusionTexture->setID(0);
    setSubconfigPtr<OcclusionTextureAttributes>("occlusionTexture",
                                                _occlusionTexture);
  }
  /**
   * @brief Get a shared_pointer to the @ref OcclusionTextureAttributes
   * descibing the nature of the occlusionTexture for this material.
   */
  OcclusionTextureAttributes::cptr getOcclusionTexture() const {
    return getSubconfigCopy<const OcclusionTextureAttributes>(
        "occlusionTexture");
  }

  /**
   * @brief Set the description of the @ref NormalTextureAttributes for this
   * material. There will be only one specified.
   */
  void setNormalTexture(NormalTextureAttributes::ptr _normalTexture) {
    _normalTexture->setID(0);
    setSubconfigPtr<NormalTextureAttributes>("normalTexture", _normalTexture);
  }
  /**
   * @brief Get a shared_pointer to the @ref NormalTextureAttributes
   * descibing the nature of the occlusionTexture for this material.
   */
  NormalTextureAttributes::cptr getNormalTexture() const {
    return getSubconfigCopy<const NormalTextureAttributes>("normalTexture");
  }

  /**
   * @brief Set the description of the emissiveTexture @ref
   * MaterialTextureAttributes for this material. There will be only one
   * specified.
   */
  void setEmissiveTexture(MaterialTextureAttributes::ptr _emissiveTexture) {
    _emissiveTexture->setID(0);
    setSubconfigPtr<MaterialTextureAttributes>("emissiveTexture",
                                               _emissiveTexture);
  }
  /**
   * @brief Get a shared_pointer to the emissiveTexture @ref
   * MaterialTextureAttributes descibing the nature of the emissiveTexture for
   * this material.
   */
  MaterialTextureAttributes::cptr getEmissiveTexture() const {
    return getSubconfigCopy<const MaterialTextureAttributes>("emissiveTexture");
  }

  /**
   * @brief Set a value representing the alphaMode value for this material. This
   * value should correspond to an enum value defined in MaterialAlphaMode.
   */
  void setAlphaMode(const std::string& alphaMode) {
    // force to uppercase before setting
    const std::string alphaModeUC = Cr::Utility::String::uppercase(alphaMode);

    auto mapIter = AlphaModeNamesMap.find(alphaModeUC);

    ESP_CHECK(mapIter != AlphaModeNamesMap.end(),
              "Illegal alphaMode value"
                  << alphaMode << "attempted to be set in MaterialAttributes :"
                  << getHandle() << ". Aborting.");
    set("alphaMode", alphaModeUC);
  }  // setAlphaMode

  /**
   * @brief Get the value representing the alphaMode value for this material.
   * Should map to an enum value in AlphaModeNamesMap
   */
  MaterialAlphaMode getAlphaMode() const {
    const std::string val =
        Cr::Utility::String::uppercase(get<std::string>("alphaMode"));
    auto mapIter = AlphaModeNamesMap.find(val);
    if (mapIter != AlphaModeNamesMap.end()) {
      return mapIter->second;
    }
    // Opaque is default value
    return MaterialAlphaMode::Opaque;
  }  // getAlphaMode

  /**
   * Set cutoff when alphaMode == MASK.  Ignored otherwise. DEFAULT: [0,0,0]
   */
  void setEmissiveFactor(const Mn::Vector3& emissiveFactor) {
    set("emissiveFactor", emissiveFactor);
  }

  /**
   * Get cutoff when alphaMode == MASK.  Ignored otherwise. DEFAULT: [0,0,0]
   */
  Mn::Vector3 getEmissiveFactor() const {
    return get<Mn::Vector3>("emissiveFactor");
  }

  /**
   * Set cutoff when alphaMode == MASK.  Ignored otherwise. DEFAULT: 0.5
   */
  void setAlphaCutoff(double alphaCutoff) { set("alphaCutoff", alphaCutoff); }

  /**
   * Get cutoff when alphaMode == MASK.  Ignored otherwise. DEFAULT: 0.5
   */
  double getAlphaCutoff() const { return get<double>("alphaCutoff"); }

  /**
   * Set whether material is double sided, when false, back-face culling is
   * enabled, otherwise back-face culling is disabled, 2x-sided lighting is
   * enabled.
   */
  void setDoubleSided(bool doubleSided) { set("doubleSided", doubleSided); }

  /**
   * Get whether material is double sided, when false, back-face culling is
   * enabled, otherwise back-face culling is disabled, 2x-sided lighting is
   * enabled.
   */
  bool getDoubleSided() const { return get<bool>("doubleSided"); }

  /**
   * @brief Populate a Json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * MaterialAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific. The individual light
   * instances return a header for this.
   */
  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(MaterialAttributes)
};  // namespace attributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_MATERIALATTRIBUTES_H_
