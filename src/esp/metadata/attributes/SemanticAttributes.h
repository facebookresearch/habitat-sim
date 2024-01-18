// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SEMANTICATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SEMANTICATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

///////////////////////////////////////
// Semantic attributes

/**
 * @brief This class describes the semantic attributes for a specific region
 * annotation.
 */
class SemanticRegionAttributes : public AbstractAttributes {
 public:
  explicit SemanticRegionAttributes(const std::string& handle);

  /**
   * @brief Get the label assigned to this semantic region.
   */
  std::string getLabel() const { return get<std::string>("label"); }

  /**
   * @brief Set the label assigned to this semantic region.
   */
  void setLabel(const std::string& _label) { set("label", _label); }

  /**
   * @brief Get the height of the floor upon which the flat poly loop's points
   * lie.
   */
  double getFloorHeight() const { return get<double>("floor_height"); }

  /**
   * @brief Set the height of the floor upon which the flat poly loop's points
   * lie.
   */
  void setFloorHeight(double _floor_height) {
    set("floor_height", _floor_height);
  }

  /**
   * @brief Get the height of the extrusion above the plane of the poly loop.
   */
  double getExtrusionHeight() const { return get<double>("extrusion_height"); }

  /**
   * @brief Set the height of the extrusion above the plane of the poly loop.
   * lie.
   */
  void setExtrusionHeight(double _extrusion_height) {
    set("extrusion_height", _extrusion_height);
  }

  /**
   * @brief Get the minimum bounds point for the region annotation
   */
  Magnum::Vector3 getMinBounds() const {
    return get<Magnum::Vector3>("min_bounds");
  }

  /**
   * @brief Set the minimum bounds point for the region annotation
   */
  void setMinBounds(const Magnum::Vector3& _min_bounds) {
    set("min_bounds", _min_bounds);
  }

  /**
   * @brief Get the maximum bounds point for the region annotation
   */
  Magnum::Vector3 getMaxBounds() const {
    return get<Magnum::Vector3>("max_bounds");
  }

  /**
   * @brief Set the maximum bounds point for the region annotation
   */
  void setMaxBounds(const Magnum::Vector3& _max_bounds) {
    set("max_bounds", _max_bounds);
  }

  ////  TODO : Replace vector with configuration?

  /**
   * @brief retrieve a const reference to the vector holding the poly loop
   * points.
   */
  const std::vector<Magnum::Vector3>& getPolyLoop() const { return polyLoop_; }
  /**
   * @brief Set the vector holding the poly loop points.
   */
  void setPolyLoop(std::vector<Magnum::Vector3> _polyLoop) {
    polyLoop_ = std::move(_polyLoop);
  }

  /**
   * @brief Populate a JSON object with all the first-level values held in this
   * SemanticRegionAttributes.  Default is overridden to handle special
   * cases for SemanticRegionAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */
  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

  /**
   * @brief Vector of points making up the poly loop that describes the
   * extrusion base.
   */
  std::vector<Magnum::Vector3> polyLoop_{};

 public:
  ESP_SMART_POINTERS(SemanticRegionAttributes)

};  // class SemanticAttributes

/**
 * @brief This class describes the semantic attributes for a specific scene.
 * This includes semantic region description and annotation.
 */
class SemanticAttributes : public AbstractAttributes {
 public:
  explicit SemanticAttributes(const std::string& handle);

  SemanticAttributes(const SemanticAttributes& otr);
  SemanticAttributes(SemanticAttributes&& otr) noexcept;

  SemanticAttributes& operator=(const SemanticAttributes& otr);
  SemanticAttributes& operator=(SemanticAttributes&& otr) noexcept;

  /**
   * @brief Set the filename to the text file that describes the hierharchy of
   * semantic information embedded in the Semantic Asset mesh.  May be
   * overridden by value specified in Scene Instance Attributes.
   */
  void setSemanticDescriptorFilename(
      const std::string& semantic_descriptor_filename) {
    set("semantic_descriptor_filename", semantic_descriptor_filename);
  }
  /**
   * @brief Get the filename to the text file that describes the hierharchy of
   * semantic information embedded in the Semantic Asset mesh.  May be
   * overridden by value specified in Scene Instance Attributes.
   */
  std::string getSemanticDescriptorFilename() const {
    return get<std::string>("semantic_descriptor_filename");
  }

  /**
   * @brief Set the Filename to the semantic texture mesh, if one exists.
   */
  void setSemanticAssetHandle(const std::string& semanticAssetHandle) {
    set("semantic_asset", semanticAssetHandle);
  }

  /**
   * @brief Get the Filename to the semantic texture mesh, if one exists.
   */
  std::string getSemanticAssetHandle() const {
    return get<std::string>("semantic_asset");
  }

  /**
   * @brief Add an object instance attributes to this scene instance.
   */
  void addRegionInstanceAttrs(SemanticRegionAttributes::ptr _regionInstance) {
    setSubAttributesInternal<SemanticRegionAttributes>(
        _regionInstance, availableRegionInstIDs_, regionAnnotationConfig_,
        "region_desc_");
  }

  /**
   * @brief Get the object instance descriptions for this scene
   */
  std::vector<SemanticRegionAttributes::cptr> getRegionInstances() const {
    return getSubAttributesListInternal<SemanticRegionAttributes>(
        regionAnnotationConfig_);
  }
  /**
   * @brief Return the number of defined @ref SemanticRegionAttributes
   * subconfigs in this scene instance.
   */
  int getNumRegionInstances() const {
    return getNumSubAttributesInternal("region_desc_", regionAnnotationConfig_);
  }
  /**
   * @brief Clears current regionAnnotationConfig_ values.
   */
  void clearRegionInstances() {
    this->removeSubconfig("region_annotations");
    regionAnnotationConfig_ =
        editSubconfig<Configuration>("region_annotations");
  }

  /**
   * @brief Populate a JSON object with all the first-level values held in this
   * SemanticRegionAttributes.  Default is overridden to handle special
   * cases for SemanticRegionAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

  /**
   * @brief Populate a JSON object with all the data from the subconfigurations,
   * held in JSON sub-objects, for this SceneInstance. Have special handling for
   * ao instances and object instances before handling other subConfigs.
   */
  void writeSubconfigsToJson(io::JsonGenericValue& jsonObj,
                             io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific. Don't use this
   * method, since we have both SemanticAttributes data to save and the
   * individual SemanticRegionAttributes data to save.
   */
  std::string getObjectInfoHeaderInternal() const override { return ""; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

  /**
   * @brief Smartpointer to created region instance collection configuration.
   * The configuration is created on SemanticAttributes construction.
   */
  std::shared_ptr<Configuration> regionAnnotationConfig_{};

  /**
   * @brief Deque holding all released IDs to consume for region instances
   * when one is deleted, before using size of regionAnnotationConfig_
   * container.
   */
  std::deque<int> availableRegionInstIDs_;

 public:
  ESP_SMART_POINTERS(SemanticAttributes)

};  // class SemanticAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SEMANTICATTRIBUTES_H_
