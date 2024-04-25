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
  /**
   * @brief Returns the number of existing markers in this LinkMarkerSubset.
   */
  int getNumMarkers() const {
    return getSubconfigView("markers")->getNumValues();
  }

  /**
   * @brief whether the given @p markerName exists as a marker in
   * this LinkMarkerSubset.
   *
   * @param markerName The desired marker set's name.
   * @return whether the name is found as a marker value.
   */
  bool hasNamedMarker(const std::string& markerName) const {
    return getSubconfigView("markers")->hasValue(markerName);
  }

  /**
   * @brief Retrieve the marker point specified by the given @p markerName
   */
  Mn::Vector3 getNamedMarker(const std::string& markerName) const {
    return getSubconfigView("markers")->get<Mn::Vector3>(markerName);
  }

  /**
   * @brief Adds passed marker. Uses naming convention from load - key for this
   * marker will be "markers_{numCurrentMarkers}"
   */
  void addMarker(const Magnum::Vector3& marker) {
    auto markersPtr = editSubconfig<Configuration>("markers");
    const std::string markerKey = Cr::Utility::formatString(
        "markers_{:.02d}", markersPtr->getNumValues());
    markersPtr->set(markerKey, marker);
  }

  /**
   * @brief Retrieve a listing of all the marker handles in this
   * LinkMarkerSubset.
   */
  std::vector<std::string> getAllMarkerNames() const {
    return getSubconfigView("markers")->getKeys();
  }

  /**
   * @brief Returns a list of all markers in this LinkMarkerSubset
   */
  std::vector<Mn::Vector3> getAllMarkers() const {
    const auto markersPtr = getSubconfigView("markers");
    std::vector<std::string> markerTags = markersPtr->getKeys();
    std::vector<Mn::Vector3> res;
    res.reserve(markerTags.size());
    for (const auto& tag : markerTags) {
      res.emplace_back(std::move(markersPtr->get<Mn::Vector3>(tag)));
    }
    return res;
  }

  /**
   * @brief Remove a named marker.
   */
  Mn::Vector3 removeMarker(const std::string& markerKey) {
    return editSubconfig<Configuration>("markers")->remove<Mn::Vector3>(
        markerKey);
  }

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

  /**
   * @brief Removes named LinkMarkerSubset. Does nothing if DNE.
   */
  void removeNamedMarkerSet(const std::string& linkSubsetName) {
    removeSubconfig(linkSubsetName);
  }

  ESP_SMART_POINTERS(LinkMarkerSets)
};  // class LinkMarkerSets

/**
 * @brief This class provides an alias for the nested configuration tree used
 * for a single MarkerSet, covering 1 or more links
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

  /**
   * @brief Removes named LinkMarkerSets. Does nothing if DNE.
   */
  void removeNamedMarkerSet(const std::string& linkSetName) {
    removeSubconfig(linkSetName);
  }

  ESP_SMART_POINTERS(MarkerSet)
};  // class MarkerSet

/**
 * @brief This class provides an alias for the nested configuration tree used
 * to hold multiple MarkerSets.
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
   * @brief Retrieve a listing of all the MarkerSet handles in this collection.
   */
  std::vector<std::string> getAllMarkerSetNames() const {
    return getSubconfigKeys();
  }

  /**
   * @brief Retrivess a copy of the named MarkerSet, if it exists, and nullptr
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

  /**
   * @brief Removes named MarkerSet. Does nothing if DNE.
   */
  void removeNamedMarkerSet(const std::string& markerSetName) {
    removeSubconfig(markerSetName);
  }

  /**
   * @brief Remove the specified MarkerSet
   */

  ESP_SMART_POINTERS(MarkerSets)
};  // class MarkerSets

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_MARKERSETS_H_
