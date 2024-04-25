// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#ifndef ESP_METADATA_ATTRIBUTES_MARKERSETS_H_
#define ESP_METADATA_ATTRIBUTES_MARKERSETS_H_

#include "esp/core/Configuration.h"
namespace esp {
namespace metadata {
namespace attributes {
/** @file
 * @brief Class @ref esp::metadata::attributes::Markersets,
 * Class @ref esp::metadata::attributes::MarkerSet,
 * Class @ref esp::metadata::attributes::LinkMarkerSets,
 * Class @ref esp::metadata::attributes::LinkMarkerSubset
 */

/**
 * @brief This class provides an alias for a single configuration holding 1 or
 * more marker points for a particular link.
 */
class LinkMarkerSubset : public esp::core::config::Configuration {
 public:
  LinkMarkerSubset() : Configuration() {}

  ESP_SMART_POINTERS(LinkMarkerSubset)
};  // class LinkMarkerSubset

/**
 * @brief This class provides an alias for the nested configuration tree used
 * for a single link's 1 or more marker subsets that should be attached to the
 * named link.
 */
class LinkMarkerSets : public esp::core::config::Configuration {
 public:
  LinkMarkerSets() : Configuration() {}

  /**
   * @brief Returns the number of existing LinkMarkerSubset in this collection.
   */
  int getNumLinkMarkeSubsets() const { return getNumSubconfigs(); }

  /**
   * @brief whether the given @p linkSubsetName exists as a LinkMarkerSubset in
   * this collection.
   *
   * @param linkSubsetName The desired marker set's name.
   * @return whether the name is found as a LinkMarkerSubset subconfiguration.
   */
  bool hasNamedLinkMarkeSubset(const std::string& linkSubsetName) const {
    return hasSubconfig(linkSubsetName);
  }

  /**
   * @brief Retrieve a listing of all the LinkMarkerSubset handles in this
   * collection.
   */
  std::vector<std::string> getAllLinkMarkeSubsetNames() const {
    return getSubconfigKeys();
  }

  /**
   * @brief Retrivess a copy of the named LinkMarkerSubset, if it exists, and
   * nullptr if it does not.
   */
  LinkMarkerSubset::ptr getNamedLinkMarkeSubsetCopy(
      const std::string& linkSubsetName) {
    return getSubconfigCopy<LinkMarkerSubset>(linkSubsetName);
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created)
   * LinkMarkerSubset with the given @p linkSubsetName , which can be modified
   * and the modifications will be retained.
   *
   * @param linkSubsetName The desired LinkMarkerSubset set's name.
   * @return a reference to the LinkMarkerSubset set.
   */
  LinkMarkerSubset::ptr editNamedLinkMarkeSubset(
      const std::string& linkSubsetName) {
    return editSubconfig<LinkMarkerSubset>(linkSubsetName);
  }

  ESP_SMART_POINTERS(LinkMarkerSets)
};  // class LinkMarkerSets

/**
 * @brief This class provides an alias for the nested configuration tree used
 * for a single markerset, covering 1 or more links
 */
class MarkerSet : public esp::core::config::Configuration {
 public:
  MarkerSet() : Configuration() {}

  /**
   * @brief Returns the number of existing LinkMarkerSets in this collection.
   */
  int getNumLinkMarkerSets() const { return getNumSubconfigs(); }

  /**
   * @brief whether the given @p linkSetName exists as a LinkMarkerSets in this
   * collection.
   *
   * @param linkSetName The desired LinkMarkerSets' name.
   * @return whether the name is found as a LinkMarkerSets subconfiguration.
   */
  bool hasNamedLinkMarkerSets(const std::string& linkSetName) const {
    return hasSubconfig(linkSetName);
  }

  /**
   * @brief Retrieve a listing of all the LinkMarkerSets handles in this
   * collection.
   */
  std::vector<std::string> getAllLinkMarkerSetsNames() const {
    return getSubconfigKeys();
  }

  /**
   * @brief Retrivess a copy of the named LinkMarkerSets, if it exists, and
   * nullptr if it does not.
   */
  LinkMarkerSets::ptr getNamedLinkMarkerSetsCopy(
      const std::string& linkSetName) {
    return getSubconfigCopy<LinkMarkerSets>(linkSetName);
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created)
   * LinkMarkerSets with the given @p linkSetName , which can be modified and
   * the modifications will be retained.
   *
   * @param linkSetName The desired marker set's name.
   * @return a reference to the marker set.
   */
  LinkMarkerSets::ptr editNamedLinkMarkerSets(const std::string& linkSetName) {
    return editSubconfig<LinkMarkerSets>(linkSetName);
  }

  ESP_SMART_POINTERS(MarkerSet)
};  // class MarkerSet

/**
 * @brief This class provides an alias for the nested configuration tree used
 * for markersets.
 */
class MarkerSets : public esp::core::config::Configuration {
 public:
  MarkerSets() : Configuration() {}

  /**
   * @brief Returns the number of existing MarkerSets in this collection.
   */
  int getNumMarkerSets() const { return getNumSubconfigs(); }

  /**
   * @brief whether the given @p markerSetName exists as a markerSet in this
   * collection.
   *
   * @param markerSetName The desired marker set's name.
   * @return whether the name is found as a MarkerSet subconfiguration.
   */
  bool hasNamedMarkerSet(const std::string& markerSetName) const {
    return hasSubconfig(markerSetName);
  }

  /**
   * @brief Retrieve a listing of all the markerset handles in this collection.
   */
  std::vector<std::string> getAllMarkerSetNames() const {
    return getSubconfigKeys();
  }

  /**
   * @brief Retrivess a copy of the named markerset, if it exists, and nullptr
   * if it does not.
   */
  MarkerSet::ptr getNamedMarkerSetCopy(const std::string& markerSetName) {
    return getSubconfigCopy<MarkerSet>(markerSetName);
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created) MarkerSet
   * with the given @p markerSetName , which can be modified and the
   * modifications will be retained.
   *
   * @param markerSetName The desired marker set's name.
   * @return a reference to the marker set.
   */
  MarkerSet::ptr editNamedMarkerSet(const std::string& markerSetName) {
    return editSubconfig<MarkerSet>(markerSetName);
  }

  ESP_SMART_POINTERS(MarkerSets)
};  // class MarkerSets

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_MARKERSETS_H_
