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
 * Class @ref esp::metadata::attributes::TaskSet,
 * Class @ref esp::metadata::attributes::LinkSet,
 * Class @ref esp::metadata::attributes::MarkerSet
 */

/**
 * @brief This class provides an alias for a single Configuration holding 1 or
 * more marker points within a particular LinkSet.
 */
class MarkerSet : public esp::core::config::Configuration {
 public:
  MarkerSet() : Configuration() {}
  /**
   * @brief Returns the number of existing marker points in this MarkerSet.
   */
  int getNumPoints() {
    // the 'markers' subconfig may not exist due to how the MarkerSet hierarchy
    // is loaded from JSON.
    if (!hasSubconfig("markers")) {
      return 0;
    }
    return getSubconfigView("markers")->getNumValues();
  }

  /**
   * @brief Returns a list of all the marker points in this MarkerSet
   */
  std::vector<Mn::Vector3> getAllPoints() const {
    // the 'markers' subconfig may not exist due to how the MarkerSet hierarchy
    // is loaded from JSON.
    if (!hasSubconfig("markers")) {
      return {};
    }
    return getSubconfigValsOfTypeInVector<Mn::Vector3>("markers");
  }

  /**
   * @brief Set the list of all the marker points for this MarkerSet
   */
  void setAllPoints(const std::vector<Mn::Vector3>& markers) {
    setSubconfigValsOfTypeInVector("markers", markers);
  }

  /**
   * @brief Rekeys all marker points to have vector IDXs as string keys,
   * retaining their original keys' natural ordering.
   * @return returns how many marker points have been processed with new keys.
   */
  int rekeyAllMarkers() { return rekeySubconfigValues("markers"); }

 public:
  ESP_SMART_POINTERS(MarkerSet)
};  // class MarkerSet

/**
 * @brief This class provides an alias for the nested Configuration tree used
 * for a single link's 1 or more MarkerSets that should be attached to the
 * named link.
 */
class LinkSet : public esp::core::config::Configuration {
 public:
  LinkSet() : Configuration() {}

  /**
   * @brief Returns the number of existing MarkerSets in this LinkSet.
   */
  int getNumMarkerSets() const { return getNumSubconfigs(); }

  /**
   * @brief whether the given @p markerSetName exists as a MarkerSet in
   * this LinkSet.
   *
   * @param markerSetName The desired marker set's name.
   * @return whether the name is found as a MarkerSet subConfiguration.
   */
  bool hasMarkerSet(const std::string& markerSetName) const {
    return hasSubconfig(markerSetName);
  }

  /**
   * @brief Retrieve a listing of all the MarkerSet handles in this
   * LinkSet.
   */
  std::vector<std::string> getAllMarkerSetNames() const {
    return getSubconfigKeys(true);
  }

  /**
   * @brief Retrivess a copy of the named MarkerSet, if it exists, and
   * nullptr if it does not.
   */
  MarkerSet::ptr getMarkerSetCopy(const std::string& markerSetName) {
    return getSubconfigCopy<MarkerSet>(markerSetName);
  }

  /**
   * @brief Retrieve a view of the named MarkerSet, if it exists, and
   * nullptr if it does not.
   */
  MarkerSet::cptr getMarkerSetView(const std::string& markerSetName) const {
    return std::static_pointer_cast<const MarkerSet>(
        getSubconfigView(markerSetName));
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created)
   * MarkerSet with the given @p markerSetName , which can be modified
   * and the modifications will be retained.
   *
   * @param markerSetName The desired MarkerSet name.
   * @return a reference to the MarkerSet.
   */
  MarkerSet::ptr editMarkerSet(const std::string& markerSetName) {
    MarkerSet::ptr ptr = editSubconfig<MarkerSet>(markerSetName);
    if (!ptr->hasSubconfig("markers")) {
      ptr->editSubconfig<Configuration>("markers");
    }
    return ptr;
  }

  /**
   * @brief Removes named MarkerSet. Does nothing if DNE.
   */
  void removeMarkerSet(const std::string& markerSetName) {
    removeSubconfig(markerSetName);
  }

  /**
   * @brief Set the specified MarkerSet's points to the given values.
   * @param markerSetName the name of the MarkerSet
   * @param markerList the list of the specified MarkerSet's points.
   */
  void setMarkerSetPoints(const std::string& markerSetName,
                          const std::vector<Mn::Vector3>& markerList) {
    editMarkerSet(markerSetName)->setAllPoints(markerList);
  }

  /**
   * @brief Set the marker points of all the MarkerSets specified by name in the
   * passed map.
   * @param markerMap a map holding all the MarkerSet points within this
   * LinkSet, with MarkerSet name as the key, referncing a vector of 3d points,
   */
  void setAllMarkerPoints(
      const std::unordered_map<std::string, std::vector<Mn::Vector3>>&
          markerMap) {
    for (const auto& markers : markerMap) {
      setMarkerSetPoints(markers.first, markers.second);
    }
  }

  /**
   * @brief Retrieve all the marker points for the specified MarkerSet in this
   * LinkSet
   * @return a vector of 3d points
   */
  std::vector<Mn::Vector3> getMarkerSetPoints(const std::string& key) const {
    return getMarkerSetView(key)->getAllPoints();
  }

  /**
   * @brief Retrieve all the marker points across all MarkerSets for this link,
   * as a map.
   * @return a map holding all the MarkerSet points within this LinkSet, with
   * MarkerSet name as the key, referncing a vector of 3d points
   */
  std::unordered_map<std::string, std::vector<Mn::Vector3>> getAllMarkerPoints()
      const {
    std::unordered_map<std::string, std::vector<Mn::Vector3>> resMap;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      resMap[key] = getMarkerSetPoints(key);
    }
    return resMap;
  }  // getAllMarkerPoints

  /**
   * @brief Rekeys all marker collections to have vector IDXs as string keys
   * @return returns how many markers have been processed with new keys in this
   * LinkSet's MarkerSets.
   */
  int rekeyAllMarkers() {
    int res = 0;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      res += editMarkerSet(key)->rekeyAllMarkers();
    }
    return res;
  }

  ESP_SMART_POINTERS(LinkSet)
};  // class LinkSet

/**
 * @brief This class provides an alias for the nested Configuration tree used
 * for a single TaskSet, holding 1 or more LinkSets
 */
class TaskSet : public esp::core::config::Configuration {
 public:
  TaskSet() : Configuration() {}

  /**
   * @brief Returns the number of existing LinkSets in this collection.
   */
  int getNumLinkSets() const { return getNumSubconfigs(); }

  /**
   * @brief Returns the number of existing MarkerSets in this collection.
   */
  int getNumLinkMarkerSets() const {
    const auto& subsetKeys = getSubconfigKeys();
    int count = 0;
    for (const auto& linkSetName : subsetKeys) {
      count += getLinkSetView(linkSetName)->getNumMarkerSets();
    }
    return count;
  }

  /**
   * @brief Whether the given @p linkSetName exists as a LinkSet in this
   * collection.
   *
   * @param linkSetName The desired LinkSet' name.
   * @return whether the name is found as a LinkSet subConfiguration.
   */
  bool hasLinkSet(const std::string& linkSetName) const {
    return hasSubconfig(linkSetName);
  }

  /**
   * @brief Whether the given @p markerSetName and @p linkSetName exist in this
   * TaskSet.
   *
   * @param linkSetName The desired LinkSet' name.
   * @return whether the named MarkerSet exists within the named LinkSet.
   */
  bool hasLinkMarkerSet(const std::string& linkSetName,
                        const std::string& markerSetName) const {
    if (hasSubconfig(linkSetName)) {
      return getLinkSetView(linkSetName)->hasMarkerSet(markerSetName);
    }
    return false;
  }

  /**
   * @brief Retrieve a listing of all the LinkSet handles in this
   * collection.
   */
  std::vector<std::string> getAllLinkSetNames() const {
    return getSubconfigKeys(true);
  }

  /**
   * @brief Retrivess a copy of the named LinkSet, if it exists, and
   * nullptr if it does not.
   */
  LinkSet::ptr getLinkSetCopy(const std::string& linkSetName) {
    return getSubconfigCopy<LinkSet>(linkSetName);
  }

  /**
   * @brief Retrieve a view of the named LinkSet, if it exists, and
   * nullptr if it does not.
   */
  LinkSet::cptr getLinkSetView(const std::string& linkSetName) const {
    return std::static_pointer_cast<const LinkSet>(
        getSubconfigView(linkSetName));
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created)
   * LinkSet with the given @p linkSetName , which can be modified and
   * the modifications will be retained.
   *
   * @param linkSetName The desired LinkSet's name.
   * @return a reference to the LinkSet.
   */
  LinkSet::ptr editLinkSet(const std::string& linkSetName) {
    return editSubconfig<LinkSet>(linkSetName);
  }

  /**
   * @brief Removes named LinkSet. Does nothing if DNE.
   */
  void removeLinkSet(const std::string& linkSetName) {
    removeSubconfig(linkSetName);
  }

  /**
   * @brief Initialize a link/markerset hierarchy with the passed names
   *
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @param markerSetName the name of the MarkerSet within @p linkSetName
   */

  void initLinkMarkerSet(const std::string& linkSetName,
                         const std::string& markerSetName) {
    editLinkSet(linkSetName)->editMarkerSet(markerSetName);
  }

  /**
   * @brief Set a specified LinkSet's specified MarkerSet's points to the given
   * list of points.
   * @param linkSetName the name of the LinkSet
   * @param markerSetName the name of the MarkerSet within @p linkSetName
   * @param markerList the list of the specified MarkerSet's points.
   */
  void setLinkMarkerSetPoints(const std::string& linkSetName,
                              const std::string& markerSetName,
                              const std::vector<Mn::Vector3>& markerList) {
    editLinkSet(linkSetName)->setMarkerSetPoints(markerSetName, markerList);
  }

  /**
   * @brief Sets all the MarkerSet points in the specified LinkSet to the given
   * marker values specified in the map.
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @param markerMap a map holding all the MarkerSet points within the
   * specified LinkSet, with MarkerSet name as the key, referncing a vector of
   * 3d points,
   */
  void setLinkSetPoints(
      const std::string& linkSetName,
      const std::unordered_map<std::string, std::vector<Mn::Vector3>>&
          markers) {
    editLinkSet(linkSetName)->setAllMarkerPoints(markers);
  }

  /**
   * @brief Sets all the LinkSet's MarkerSets' points in this TaskSet
   * to the given marker values specified in the map.
   * @param markerMap an unordered map keyed by LinkSet name of unordered maps,
   * each keyed by MarkerSet name of Markers as a vector of 3d points.
   */
  void setAllMarkerPoints(
      const std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>&
          markerMap) {
    for (const auto& markers : markerMap) {
      setLinkSetPoints(markers.first, markers.second);
    }
  }

  /**
   * @brief Retrieve the specified LinkSet's MarkerSet as a vector of 3d
   * points.
   * @param linkSetName the name of the LinkSet
   * @param markerSetName the name of the MarkerSet within @p linkSetName
   * @return a vector of 3d points
   */
  std::vector<Mn::Vector3> getLinkMarkerSetPoints(
      const std::string& linkSetName,
      const std::string& markerSetName) const {
    return getLinkSetView(linkSetName)->getMarkerSetPoints(markerSetName);
  }

  /**
   * @brief Retrieve all the MarkerSet points for the specified LinkSet within
   * this TaskSet.
   * @param linkSetName the name of the LinkSet
   * @return  a map holding all the MarkerSet points within the
   * specified LinkSet, with MarkerSet name as the key, referncing a vector of
   * 3d points.
   */
  std::unordered_map<std::string, std::vector<Mn::Vector3>> getLinkSetPoints(
      const std::string& linkSetName) const {
    return getLinkSetView(linkSetName)->getAllMarkerPoints();
  }

  /**
   * @brief this retrieves all the marker points across all the LinkSets in this
   * TaskSet.
   * @return an unordered map keyed by LinkSet name of unordered maps, each
   * keyed by MarkerSet name of Markers as a vector of 3d points.
   */
  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::vector<Mn::Vector3>>>
  getAllMarkerPoints() const {
    std::unordered_map<
        std::string, std::unordered_map<std::string, std::vector<Mn::Vector3>>>
        resMap;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& linkSetName : subsetKeys) {
      resMap[linkSetName] = getLinkSetPoints(linkSetName);
    }
    return resMap;
  }  // getAllMarkerPoints

  /**
   * @brief Rekeys all marker collections to have vector IDXs as string keys
   * @return returns how many markers have been processed with new keys in this
   * TaskSet.
   */
  int rekeyAllMarkers() {
    int res = 0;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      res += editLinkSet(key)->rekeyAllMarkers();
    }
    return res;
  }

  ESP_SMART_POINTERS(TaskSet)
};  // class TaskSet

/**
 * @brief This class provides an alias for the nested Configuration tree used
 * to hold multiple TaskSets.
 */
class MarkerSets : public esp::core::config::Configuration {
 public:
  MarkerSets() : Configuration() {}

  /**
   * @brief Returns the number of existing TaskSets in this collection.
   */
  int getNumTaskSets() const { return getNumSubconfigs(); }

  /**
   * @brief Returns the number of existing LinkSets in this collection.
   */
  int getNumTaskLinkSets() const {
    const auto& subsetKeys = getSubconfigKeys();
    int count = 0;
    for (const auto& taskSetName : subsetKeys) {
      count += getTaskSetView(taskSetName)->getNumLinkSets();
    }
    return count;
  }
  /**
   * @brief Returns the number of existing MarkerSets in this collection.
   */
  int getNumTaskLinkMarkerSets() const {
    const auto& subsetKeys = getSubconfigKeys();
    int count = 0;
    for (const auto& taskSetName : subsetKeys) {
      count += getTaskSetView(taskSetName)->getNumLinkMarkerSets();
    }
    return count;
  }

  /**
   * @brief whether the given @p taskSetName exists as a TaskSet in this
   * collection.
   *
   * @param taskSetName The desired TaskSet's name.
   * @return whether the name is found as a TaskSet subConfiguration.
   */
  bool hasTaskSet(const std::string& taskSetName) const {
    return hasSubconfig(taskSetName);
  }

  /**
   * @brief Whether the given @p linkSetName and @p taskSetName exist in this
   * collection.
   *
   * @param taskSetName The desired TaskSet's name.
   * @param linkSetName The desired LinkSet's name, to be found in the given
   * TaskSet.
   * @return wwhether the names are found as expected.
   */
  bool hasTaskLinkSet(const std::string& taskSetName,
                      const std::string& linkSetName) const {
    if (hasSubconfig(taskSetName)) {
      return getTaskSetView(taskSetName)->hasLinkSet(linkSetName);
    }
    return false;
  }

  /**
   * @brief Whether the given hierarchy of @p markerSetName, @p linkSetName and
   * @p taskSetName all exist in this collection.
   *
   * @param taskSetName The desired TaskSet's name.
   * @param linkSetName The desired LinkSet's name, to be found in the given
   * TaskSet.
   * @param markerSetName The desired MarkerSet's name, to be found in the given
   * LinkSet.
   * @return wwhether the names are found as expected.
   */
  bool hasTaskLinkMarkerSet(const std::string& taskSetName,
                            const std::string& linkSetName,
                            const std::string& markerSetName) const {
    if (hasSubconfig(taskSetName)) {
      return getTaskSetView(taskSetName)
          ->hasLinkMarkerSet(linkSetName, markerSetName);
    }
    return false;
  }

  /**
   * @brief Retrieve a listing of all the TaskSet handles in this
   * collection.
   */
  std::vector<std::string> getAllTaskSetNames() const {
    return getSubconfigKeys(true);
  }

  /**
   * @brief Retrivess a copy of the named TaskSet, if it exists, and nullptr
   * if it does not.
   */
  TaskSet::ptr getTaskSetCopy(const std::string& taskSetName) {
    return getSubconfigCopy<TaskSet>(taskSetName);
  }

  /**
   * @brief Retrieve a view of the named TaskSet, if it exists, and
   * nullptr if it does not.
   */
  TaskSet::cptr getTaskSetView(const std::string& taskSetName) const {
    return std::static_pointer_cast<const TaskSet>(
        getSubconfigView(taskSetName));
  }

  /**
   * @brief Retrieves a reference to a (potentially newly created) TaskSet
   * with the given @p taskSetName , which can be modified and the
   * modifications will be retained.
   *
   * @param taskSetName The desired TaskSet's name.
   * @return a reference to the TaskSet.
   */
  TaskSet::ptr editTaskSet(const std::string& taskSetName) {
    return editSubconfig<TaskSet>(taskSetName);
  }

  /**
   * @brief Removes named TaskSet. Does nothing if DNE.
   */
  void removeTaskSet(const std::string& taskSetName) {
    removeSubconfig(taskSetName);
  }

  /**
   * @brief Initialize a task/link/markerset hierarchy with the passed names
   *
   * @param taskSetName the name of the TaskSet
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @param markerSetName the name of the MarkerSet within @p linkSetName
   */
  void initTaskLinkMarkerSet(const std::string& taskSetName,
                             const std::string& linkSetName,
                             const std::string& markerSetName) {
    editTaskSet(taskSetName)->initLinkMarkerSet(linkSetName, markerSetName);
  }

  /**
   * @brief Set the specified TaskSet's specified LinkSet's specified
   * MarkerSet's marker points.
   * @param taskSetName the name of the TaskSet
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @param markerSetName the name of the MarkerSet within @p linkSetName
   * @param markerList the list of the specified MarkerSet' points.
   */
  void setTaskLinkMarkerSetPoints(const std::string& taskSetName,
                                  const std::string& linkSetName,
                                  const std::string& markerSetName,
                                  const std::vector<Mn::Vector3>& markerList) {
    editTaskSet(taskSetName)
        ->setLinkMarkerSetPoints(linkSetName, markerSetName, markerList);
  }

  /**
   * @brief Sets all the MarkerSet points in the specified TaskSet's specified
   * LinkSet to the given marker values specified in the map.
   * @param taskSetName the name of the TaskSet
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @param markerMap a map holding all the MarkerSet points within the
   * specified LinkSet, with MarkerSet name as the key, referncing a vector of
   * 3d points,
   */
  void setTaskLinkSetPoints(
      const std::string& taskSetName,
      const std::string& linkSetName,
      const std::unordered_map<std::string, std::vector<Mn::Vector3>>&
          markerMap) {
    editTaskSet(taskSetName)->setLinkSetPoints(linkSetName, markerMap);
  }

  /**
   * @brief Sets all the LinkSet's MarkerSets' points in the specified TaskSet
   * to the given marker values specified in the map.
   * @param taskSetName the name of the TaskSet
   * @param markerMap an unordered map keyed by LinkSet name of unordered
   * maps, each keyed by MarkerSet name of Markers as a vector of 3d points.
   */
  void setTaskSetPoints(
      const std::string& taskSetName,
      const std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>&
          markerMap) {
    auto taskSetPtr = editTaskSet(taskSetName);
    for (const auto& markers : markerMap) {
      taskSetPtr->setLinkSetPoints(markers.first, markers.second);
    }
  }

  /**
   * @brief Set all the marker points across every TaskSet using the values in
   * the passed map.
   * @param markerMap an unordered map keyed by TaskSet name, of unordered
   * maps, each keyed by LinkSet name, of unordered maps, each keyed by
   * MarkerSet name of Markers as a vector of 3d points.
   */
  void setAllMarkerPoints(
      const std::unordered_map<
          std::string,
          std::unordered_map<
              std::string,
              std::unordered_map<std::string, std::vector<Mn::Vector3>>>>&
          markerMap) {
    for (const auto& markers : markerMap) {
      setTaskSetPoints(markers.first, markers.second);
    }
  }

  /**
   * @brief Retrieve a single TaskSet's LinkSet's MarkerSet as a vector of 3d
   * points.
   * @param taskSetName the name of the TaskSet
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @param markerSetName the name of the MarkerSet within @p linkSetName
   * @return a vector of 3d points
   */
  std::vector<Mn::Vector3> getTaskLinkMarkerSetPoints(
      const std::string& taskSetName,
      const std::string& linkSetName,
      const std::string& markerSetName) const {
    return getTaskSetView(taskSetName)
        ->getLinkMarkerSetPoints(linkSetName, markerSetName);
  }

  /**
   * @brief Retrieve all of the MarkerSets for a particular LinkSet within the
   * specified TaskSet, as an unordered map keyed by MarkerSet name of
   * Markers as a vector of 3d points.
   * @param taskSetName the name of the TaskSet
   * @param linkSetName the name of the LinkSet within @p taskSetName
   * @return a map holding all the MarkerSet points within the specified
   * LinkSet, with MarkerSet name as the key, referncing a vector of 3d
   * points,
   */

  std::unordered_map<std::string, std::vector<Mn::Vector3>>
  getTaskLinkSetPoints(const std::string& taskSetName,
                       const std::string& linkSetName) const {
    return getTaskSetView(taskSetName)->getLinkSetPoints(linkSetName);
  }

  /**
   * @brief Retrieve all the marker points for the named TaskSet, as an
   * unordered map keyed by LinkSet name of unordered maps, each keyed by
   * MarkerSet name of Markers as a vector of 3d points.
   * @param taskSetName the name of the TaskSet
   * @return an unordered map keyed by LinkSet name of unordered maps, each
   * keyed by MarkerSet name of Markers as a vector of 3d points.
   */
  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::vector<Mn::Vector3>>>
  getTaskSetPoints(const std::string& taskSetName) const {
    return getTaskSetView(taskSetName)->getAllMarkerPoints();
  }

  /**
   * @brief Retrieve all the MarkerSet points across all the TaskSets.
   * @return an unordered map keyed by TaskSet name, of unordered maps, each
   * keyed by LinkSet name, of unordered maps, each keyed by MarkerSet name of
   * Markers as a vector of 3d points.
   */
  std::unordered_map<
      std::string,
      std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
  getAllMarkerPoints() const {
    std::unordered_map<
        std::string,
        std::unordered_map<
            std::string,
            std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
        resMap;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& taskSetName : subsetKeys) {
      resMap[taskSetName] = getTaskSetPoints(taskSetName);
    }
    return resMap;
  }  // getAllMarkerPoints

  /**
   * @brief Rekeys all marker collections to have vector IDXs as string keys
   * @return returns how many marker points have been processed with new keys.
   */
  int rekeyAllMarkers() {
    int res = 0;
    const auto& subsetKeys = getSubconfigKeys();
    for (const auto& key : subsetKeys) {
      res += editTaskSet(key)->rekeyAllMarkers();
    }
    return res;
  }

  ESP_SMART_POINTERS(MarkerSets)
};  // class MarkerSets

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_MARKERSETS_H_
