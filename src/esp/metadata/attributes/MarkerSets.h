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
 * Class @ref esp::metadata::attributes::LinkMarkerSet,
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
   * @brief Returns a list of all markers in this LinkMarkerSubset
   */
  std::vector<Mn::Vector3> getMarkers() const {
    const auto markersPtr = getSubconfigView("markers");
    // Get all vector3 keys from subconfig in sorted vector
    std::vector<std::string> markerTags = markersPtr->getKeysByType(
        esp::core::config::ConfigValType::MagnumVec3, true);
    std::vector<Mn::Vector3> res;
    res.reserve(markerTags.size());
    for (const auto& tag : markerTags) {
      res.emplace_back(markersPtr->get<Mn::Vector3>(tag));
    }
    return res;
  }

  /**
   * @brief Set the list of marker points
   */
  void setMarkers(const std::vector<Mn::Vector3>& markers) {
    auto markersPtr = editSubconfig<Configuration>("markers");
    // Remove all existing markers
    markersPtr->_clearAllValues();
    for (std::size_t i = 0; i < markers.size(); ++i) {
      const std::string& key = Cr::Utility::formatString("{:.03d}", i);
      markersPtr->set(key, markers[i]);
    }
  }

  /**
   * @brief Rekeys all markers to have vector IDXs as string keys
   * @return returns how many markers have been processed with new keys.
   */
  int rekeyAllMarkers() { return rekeySubconfigValues("markers"); }

  ESP_SMART_POINTERS(LinkMarkerSubset)
};  // class LinkMarkerSubset

/**
 * @brief This class provides an alias for the nested configuration tree used
 * for a single link's 1 or more marker subsets that should be attached to the
 * named link.
 */
class LinkMarkerSet : public esp::core::config::Configuration {
 public:
  LinkMarkerSet() : Configuration() {}

  /**
   * @brief Returns the number of existing LinkSubset in this collection.
   */
  int getNumLinkSubsets() const { return getNumSubconfigs(); }

  /**
   * @brief whether the given @p linkSubsetName exists as a LinkSubset in
   * this collection.
   *
   * @param linkSubsetName The desired marker set's name.
   * @return whether the name is found as a LinkSubset subconfiguration.
   */
  bool hasNamedLinkSubset(const std::string& linkSubsetName) const {
    return hasSubconfig(linkSubsetName);
  }

  /**
   * @brief Retrieve a listing of all the LinkSubset handles in this
   * collection.
   */
  std::vector<std::string> getAllLinkSubsetNames() const {
    return getSubconfigKeys(true);
  }

  /**
   * @brief Retrivess a copy of the named LinkSubset, if it exists, and
   * nullptr if it does not.
   */
  LinkMarkerSubset::ptr getNamedLinkSubsetCopy(
      const std::string& linkSubsetName) {
    return getSubconfigCopy<LinkMarkerSubset>(linkSubsetName);
  }

  /**
   * @brief Retrieve a view of the naamed LinkSubset, if it exists, and
   * nullptr if it does not.
   */
  LinkMarkerSubset::cptr getNamedLinkSubsetView(
      const std::string& linkSubsetName) const {
    return std::static_pointer_cast<const LinkMarkerSubset>(
        getSubconfigView(linkSubsetName));
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created)
   * LinkMarkerSubset with the given @p linkSubsetName , which can be modified
   * and the modifications will be retained.
   *
   * @param linkSubsetName The desired LinkMarkerSubset set's name.
   * @return a reference to the LinkMarkerSubset set.
   */
  LinkMarkerSubset::ptr editNamedLinkSubset(const std::string& linkSubsetName) {
    return editSubconfig<LinkMarkerSubset>(linkSubsetName);
  }

  /**
   * @brief Removes named LinkMarkerSubset. Does nothing if DNE.
   */
  void removeNamedLinkSet(const std::string& linkSubsetName) {
    removeSubconfig(linkSubsetName);
  }

  /**
   * @brief Set the specified name's subset markers to the given marker values.
   */
  void setLinkSubsetMarkers(const std::string& linkSubsetName,
                            const std::vector<Mn::Vector3>& markers) {
    editNamedLinkSubset(linkSubsetName)->setMarkers(markers);
  }

  /**
   * @brief Set the markers of all the subsets specified by name in the passed
   * map.
   */
  void setAllMarkers(
      const std::unordered_map<std::string, std::vector<Mn::Vector3>>&
          markerMap) {
    for (const auto& markers : markerMap) {
      setLinkSubsetMarkers(markers.first, markers.second);
    }
  }

  /**
   * @brief Retrieve all the markers for the named link subset for this link
   */
  std::vector<Mn::Vector3> getLinkSubsetMarkers(const std::string& key) const {
    return getNamedLinkSubsetView(key)->getMarkers();
  }

  /**
   * @brief this retrieves all the markers across all subests for this link
   */
  std::unordered_map<std::string, std::vector<Mn::Vector3>> getAllMarkers()
      const {
    std::unordered_map<std::string, std::vector<Mn::Vector3>> resMap;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      resMap[key] = getLinkSubsetMarkers(key);
    }
    return resMap;
  }  // getAllMarkers

  /**
   * @brief Rekeys all marker collections to have vector IDXs as string keys
   * @return returns how many markers have been processed with new keys in this
   * link's marker subsets.
   */
  int rekeyAllMarkers() {
    int res = 0;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      res += editNamedLinkSubset(key)->rekeyAllMarkers();
    }
    return res;
  }

  ESP_SMART_POINTERS(LinkMarkerSet)
};  // class LinkMarkerSet

/**
 * @brief This class provides an alias for the nested configuration tree used
 * for a single MarkerSet, covering 1 or more links
 */
class MarkerSet : public esp::core::config::Configuration {
 public:
  MarkerSet() : Configuration() {}

  /**
   * @brief Returns the number of existing LinkMarkerSet in this collection.
   */
  int getNumLinkSets() const { return getNumSubconfigs(); }

  /**
   * @brief whether the given @p linkSetName exists as a LinkMarkerSet in this
   * collection.
   *
   * @param linkSetName The desired LinkMarkerSet' name.
   * @return whether the name is found as a LinkMarkerSet subconfiguration.
   */
  bool hasNamedLinkSet(const std::string& linkSetName) const {
    return hasSubconfig(linkSetName);
  }

  /**
   * @brief Retrieve a listing of all the LinkMarkerSet handles in this
   * collection.
   */
  std::vector<std::string> getAllLinkSetNames() const {
    return getSubconfigKeys(true);
  }

  /**
   * @brief Retrivess a copy of the named LinkMarkerSet, if it exists, and
   * nullptr if it does not.
   */
  LinkMarkerSet::ptr getNamedLinkSetCopy(const std::string& linkSetName) {
    return getSubconfigCopy<LinkMarkerSet>(linkSetName);
  }

  /**
   * @brief Retrieve a view of the naamed LinkMarkerSet, if it exists, and
   * nullptr if it does not.
   */
  LinkMarkerSet::cptr getNamedLinkSetView(
      const std::string& linkSetName) const {
    return std::static_pointer_cast<const LinkMarkerSet>(
        getSubconfigView(linkSetName));
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created)
   * LinkMarkerSet with the given @p linkSetName , which can be modified and
   * the modifications will be retained.
   *
   * @param linkSetName The desired marker set's name.
   * @return a reference to the marker set.
   */
  LinkMarkerSet::ptr editNamedLinkSet(const std::string& linkSetName) {
    return editSubconfig<LinkMarkerSet>(linkSetName);
  }

  /**
   * @brief Removes named LinkMarkerSet. Does nothing if DNE.
   */
  void removeNamedLinkSet(const std::string& linkSetName) {
    removeSubconfig(linkSetName);
  }

  /**
   * @brief Set a specified link's specified subset's markers.
   */
  void setLinkSetSubsetMarkers(const std::string& linkSetName,
                               const std::string& linkSubsetName,
                               const std::vector<Mn::Vector3>& markers) {
    editNamedLinkSet(linkSetName)
        ->setLinkSubsetMarkers(linkSubsetName, markers);
  }

  /**
   * @brief Sets all the specified name's link markers to the given marker
   * values specified by the link subset name.
   */
  void setLinkSetMarkers(
      const std::string& linkSetName,
      const std::unordered_map<std::string, std::vector<Mn::Vector3>>&
          markers) {
    editNamedLinkSet(linkSetName)->setAllMarkers(markers);
  }

  /**
   * @brief Set the markers of all the links specified by name in the passed
   * map.
   */
  void setAllMarkers(const std::unordered_map<
                     std::string,
                     std::unordered_map<std::string, std::vector<Mn::Vector3>>>&
                         markerMap) {
    for (const auto& markers : markerMap) {
      setLinkSetMarkers(markers.first, markers.second);
    }
  }

  /**
   * @brief Get the markers for a specified link's specified subset.
   */
  std::vector<Mn::Vector3> getLinkSetSubsetMarkers(
      const std::string& linkName,
      const std::string& linkSubsetName) const {
    return getNamedLinkSetView(linkName)->getLinkSubsetMarkers(linkSubsetName);
  }

  /**
   * @brief Retrieve all the markers for the named link within this markerset
   */
  std::unordered_map<std::string, std::vector<Mn::Vector3>> getLinkSetMarkers(
      const std::string& linkName) const {
    return getNamedLinkSetView(linkName)->getAllMarkers();
  }

  /**
   * @brief this retrieves all the markers for across all links in this
   * markerset
   */
  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::vector<Mn::Vector3>>>
  getAllMarkers() const {
    std::unordered_map<
        std::string, std::unordered_map<std::string, std::vector<Mn::Vector3>>>
        resMap;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& linkName : subsetKeys) {
      resMap[linkName] = getLinkSetMarkers(linkName);
    }
    return resMap;
  }  // getAllMarkers

  /**
   * @brief Rekeys all marker collections to have vector IDXs as string keys
   * @return returns how many markers have been processed with new keys in this
   * markerset.
   */
  int rekeyAllMarkers() {
    int res = 0;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      res += editNamedLinkSet(key)->rekeyAllMarkers();
    }
    return res;
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
    return getSubconfigKeys(true);
  }

  /**
   * @brief Retrivess a copy of the named MarkerSet, if it exists, and nullptr
   * if it does not.
   */
  MarkerSet::ptr getNamedMarkerSetCopy(const std::string& markerSetName) {
    return getSubconfigCopy<MarkerSet>(markerSetName);
  }

  /**
   * @brief Retrieve a view of the naamed MarkerSet, if it exists, and
   * nullptr if it does not.
   */
  MarkerSet::cptr getNamedMarkerSetView(
      const std::string& markerSetName) const {
    return std::static_pointer_cast<const MarkerSet>(
        getSubconfigView(markerSetName));
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
   * @brief Set a specified MarkerSet's specified link's specified subset's
   * markers.
   */
  void setMarkerSetLinkSetSubsetMarkers(
      const std::string& markerSetName,
      const std::string& linkSetName,
      const std::string& linkSubsetName,
      const std::vector<Mn::Vector3>& markers) {
    editNamedMarkerSet(markerSetName)
        ->setLinkSetSubsetMarkers(linkSetName, linkSubsetName, markers);
  }

  /**
   * @brief Sets all the specified marker's specified link's subsets' markers to
   * the given marker values specified in the map.
   */
  void setMarkerSetLinkSetMarkers(
      const std::string& markerSetName,
      const std::string& linkSetName,
      const std::unordered_map<std::string, std::vector<Mn::Vector3>>&
          markers) {
    editNamedMarkerSet(markerSetName)->setLinkSetMarkers(linkSetName, markers);
  }

  /**
   * @brief Sets all the specified MarkerSet's links' subset markers to the
   * given marker values specified in the map.
   */
  void setMarkerSetMarkers(
      const std::string& markerSetName,
      const std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>& markers) {
    editNamedMarkerSet(markerSetName)->setAllMarkers(markers);
  }

  /**
   * @brief Set the markers of all the links specified by name in the passed
   * map.
   */
  void setAllMarkers(
      const std::unordered_map<
          std::string,
          std::unordered_map<
              std::string,
              std::unordered_map<std::string, std::vector<Mn::Vector3>>>>&
          markerMap) {
    for (const auto& markers : markerMap) {
      setMarkerSetMarkers(markers.first, markers.second);
    }
  }

  /**
   * @brief Return a single MarkerSet's Link's Subset of markers
   */
  std::vector<Mn::Vector3> getMarkerSetLinkSetSubsetMarkers(
      const std::string& markerSetName,
      const std::string& linkSetName,
      const std::string& linkSubsetName) const {
    return getNamedMarkerSetView(markerSetName)
        ->getLinkSetSubsetMarkers(linkSetName, linkSubsetName);
  }
  /**
   * @brief Return all of a MarkerSet's Link's Subsets of markers
   */

  std::unordered_map<std::string, std::vector<Mn::Vector3>>
  getMarkerSetLinkSetMarkers(const std::string& markerSetName,
                             const std::string& linkSetName) const {
    return getNamedMarkerSetView(markerSetName)->getLinkSetMarkers(linkSetName);
  }

  /**
   * @brief Retrieve all the markers for the named markerset
   */
  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::vector<Mn::Vector3>>>
  getMarkerSetMarkers(const std::string& markerSetName) const {
    return getNamedMarkerSetView(markerSetName)->getAllMarkers();
  }

  /**
   * @brief this retrieves all the markers across all the markersets, keyed by
   * MarkerSet name, Link Id and Link subset name
   */
  std::unordered_map<
      std::string,
      std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
  getAllMarkers() const {
    std::unordered_map<
        std::string,
        std::unordered_map<
            std::string,
            std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
        resMap;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& markerSetName : subsetKeys) {
      resMap[markerSetName] = getMarkerSetMarkers(markerSetName);
    }
    return resMap;
  }  // getAllMarkers

  /**
   * @brief Rekeys all marker collections to have vector IDXs as string keys
   * @return returns how many markers have been processed with new keys.
   */
  int rekeyAllMarkers() {
    int res = 0;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      res += editNamedMarkerSet(key)->rekeyAllMarkers();
    }
    return res;
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
