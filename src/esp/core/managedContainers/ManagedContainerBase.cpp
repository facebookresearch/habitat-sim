// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ManagedContainerBase.h"
#include <Corrade/Utility/FormatStl.h>
#include <algorithm>

namespace Cr = Corrade;

namespace esp {
namespace core {

namespace managedContainers {
bool ManagedContainerBase::setLock(const std::string& objectHandle, bool lock) {
  // if managed object does not currently exist then do not attempt to modify
  // its lock state
  if (!checkExistsWithMessage(objectHandle,
                              "<" + this->objectType_ + ">::setLock")) {
    return false;
  }
  // if setting lock else clearing lock
  if (lock) {
    userLockedObjectNames_.insert(objectHandle);
  } else {
    // if clearing, attempt to erase
    userLockedObjectNames_.erase(objectHandle);
  }
  return true;
}  // ManagedContainerBase::setLock
std::string ManagedContainerBase::getRandomObjectHandlePerType(
    const std::unordered_map<int, std::string>& mapOfHandles,
    const std::string& type) const {
  std::size_t numVals = mapOfHandles.size();
  if (numVals == 0) {
    ESP_ERROR() << "Attempting to get a random" << type << objectType_
                << "managed object handle but none are loaded, so no handles "
                   "will be returned.";
    return "";
  }
  int randIDX = rand() % numVals;

  std::string res;
  for (std::pair<std::unordered_map<int, std::string>::const_iterator, int>
           iter(mapOfHandles.begin(), 0);
       (iter.first != mapOfHandles.end() && iter.second <= randIDX);
       ++iter.first, ++iter.second) {
    res = iter.first->second;
  }
  return res;
}  // ManagedContainerBase::getRandomObjectHandlePerType

namespace {
/**
 * @brief This function will find the strings in the @p mapOfHandles that match,
 * or strictly do not match, the passed @p subStr search criteria.
 * @tparam The index in the pair of map elements to be searched. (non-type
 * param)
 * @tparam The class of the keys in the passed map
 * @tparam The class of the values in the passed map
 * @param mapOfHandles The map to search
 * @param subStr The substring to find within the requested index (key or value)
 * in the map elements
 * @param contains Whether @p subStr should be found or excluded in the results
 * returned
 * @param sorted Whether the results should be sorted.
 * @return A vector of strings that either match, or explicitly do not match,
 * the passed @p subStr in the passed @p mapOfHandles.
 */

template <int Idx, class T, class U>
std::vector<std::string> getHandlesBySubStringPerTypeInternal(
    const std::unordered_map<T, U>& mapOfHandles,
    const std::string& subStr,
    bool contains,
    bool sorted) {
  std::vector<std::string> res;
  // if empty return empty vector
  if (mapOfHandles.empty()) {
    return res;
  }
  res.reserve(mapOfHandles.size());
  // if search string is empty, return all values
  if (subStr.length() == 0) {
    for (const auto& elem : mapOfHandles) {
      res.emplace_back(std::get<Idx>(elem));
    }
  } else {
    // build search criteria for reverse map
    std::string strToLookFor = Cr::Utility::String::lowercase(subStr);
    std::size_t strSize = strToLookFor.length();
    for (typename std::unordered_map<T, U>::const_iterator iter =
             mapOfHandles.begin();
         iter != mapOfHandles.end(); ++iter) {
      std::string rawKey = std::get<Idx>(*iter);
      std::string key = Cr::Utility::String::lowercase(rawKey);
      // be sure that key is big enough to search in (otherwise find has
      // undefined behavior)
      if (key.length() < strSize) {
        continue;
      }
      bool found = (std::string::npos != key.find(strToLookFor));
      if (found == contains) {
        // if found and searching for contains, or not found and searching for
        // not contains
        res.emplace_back(std::move(rawKey));
      }
    }
  }
  if (sorted) {
    std::sort(res.begin(), res.end());
  }
  return res;
}
}  // namespace

std::vector<std::string>
ManagedContainerBase::getObjectHandlesBySubStringPerType(
    const std::unordered_map<int, std::string>& mapOfHandles,
    const std::string& subStr,
    bool contains,
    bool sorted) const {
  // get second element of pair in map entries (string)
  return getHandlesBySubStringPerTypeInternal<1>(mapOfHandles, subStr, contains,
                                                 sorted);
}  // ManagedContainerBase::getObjectHandlesBySubStringPerType

std::vector<std::string>
ManagedContainerBase::getObjectHandlesBySubStringPerType(
    const std::unordered_map<std::string, std::set<std::string>>& mapOfHandles,
    const std::string& subStr,
    bool contains,
    bool sorted) const {
  // get first element of pair in map entries (string)
  return getHandlesBySubStringPerTypeInternal<0>(mapOfHandles, subStr, contains,
                                                 sorted);
}  // ManagedContainerBase::getObjectHandlesBySubStringPerType

std::vector<std::string> ManagedContainerBase::getObjectInfoStrings(
    const std::string& subStr,
    bool contains) const {
  // get all handles that match query elements first
  std::vector<std::string> handles = getHandlesBySubStringPerTypeInternal<1>(
      objectLibKeyByID_, subStr, contains, true);
  std::vector<std::string> res(handles.size() + 1);
  if (handles.empty()) {
    res[0] = "No " + objectType_ + " constructs available.";
    return res;
  }
  int idx = 0;
  for (const std::string& objectHandle : handles) {
    // get the object
    auto objPtr = this->getObjectInternal<AbstractManagedObject>(objectHandle);
    if (idx == 0) {
      Cr::Utility::formatInto(res[idx], res[idx].size(),
                              "{} Full name,Can delete?,Is locked?,{}",
                              objectType_, objPtr->getObjectInfoHeader());
      ++idx;
    }
    Cr::Utility::formatInto(
        res[idx], res[idx].size(), "{},{},{},{}", objectHandle,
        (this->getIsUndeletable(objectHandle) ? "False" : "True"),
        (this->getIsUserLocked(objectHandle) ? "True" : "False"),
        objPtr->getObjectInfo());
    ++idx;
  }
  return res;
}  // ManagedContainerBase::getObjectInfoStrings

int ManagedContainerBase::getObjectIDByHandleOrNew(
    const std::string& objectHandle,
    bool getNext) {
  if (getObjectLibHasHandle(objectHandle)) {
    return this->getObjectInternal<AbstractManagedObject>(objectHandle)
        ->getID();
  }
  if (!getNext) {
    ESP_ERROR(Magnum::Debug::Flag::NoSpace)
        << "<" << this->objectType_ << "> : No " << objectType_
        << " managed object with handle " << objectHandle
        << " exists, so aborting ID query.";
    return ID_UNDEFINED;
  }
  return getUnusedObjectID();
}  // ManagedContainerBase::getObjectIDByHandle

std::string ManagedContainerBase::getObjectInfoCSVString(
    const std::string& subStr,
    bool contains) const {
  std::vector<std::string> infoAra = getObjectInfoStrings(subStr, contains);
  std::string res;
  for (std::string& s : infoAra) {
    Cr::Utility::formatInto(res, res.size(), "{}\n", s);
  }
  return res;
}  // ManagedContainerBase::getObjectInfoCSVString

std::string ManagedContainerBase::getUniqueHandleFromCandidatePerType(
    const std::unordered_map<int, std::string>& mapOfHandles,
    const std::string& name) const {
  // find all existing values with passed name - these should be a list
  // of existing instances of this object name.  We are going to go through
  // all of these names and find the "last" instance, meaning the highest count.
  std::vector<std::string> objHandles =
      this->getObjectHandlesBySubStringPerType(mapOfHandles, name, true, true);

  int incr = 0;
  // use this as pivot character - the last instance of this character in any
  // object instance name will be the partition between the name and the count.
  char pivotChar = ':';

  if (!objHandles.empty()) {
    // handles exist with passed substring.  Find highest handle increment, add
    // 1 and use for new name 1, build new handle
    for (const std::string& objName : objHandles) {
      // make sure pivot character is found in name
      if (objName.find(pivotChar) == std::string::npos) {
        continue;
      }
      // split string on underscore, last value will be string of highest incr
      // value existing.
      std::vector<std::string> vals =
          Cr::Utility::String::split(objName, pivotChar);
      // if any exist, check that the last element in split list is a string rep
      // of a valid integer count (count is always positive)
      const std::string digitStr = vals.back();
      if (digitStr.empty() ||
          (std::find_if(digitStr.begin(), digitStr.end(), [](unsigned char c) {
             return std::isdigit(c) == 0;
           }) != digitStr.end())) {
        // if string is not a valid representation of an integer, skip this name
        // candidate
        continue;
      }
      // by here, all are expected to have an integer count as the component
      // past the final pivot.
      int new_incr = std::stoi(digitStr);

      if (new_incr >= incr) {
        incr = new_incr + 1;
      }
    }
  }  // returned results with passed substring
  // build new name with appropriate handle increment
  const std::string handleIncrement =
      Cr::Utility::formatString("_{}{:.04d}", std::string(1, pivotChar), incr);
  return name + handleIncrement;
}  // ManagedContainerBase::getUniqueHandleFromCandidatePerType

}  // namespace managedContainers
}  // namespace core
}  // namespace esp
