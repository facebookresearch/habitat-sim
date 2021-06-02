// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ManagedContainerBase.h"
#include <Corrade/Utility/FormatStl.h>
namespace esp {
namespace core {

bool ManagedContainerBase::setLock(const std::string& objectHandle, bool lock) {
  // if managed object does not currently exist then do not attempt to modify
  // its lock state
  if (!checkExistsWithMessage(objectHandle, "ManagedContainerBase::setLock")) {
    return false;
  }
  // if setting lock else clearing lock
  if (lock) {
    userLockedObjectNames_.insert(objectHandle);
  } else if (userLockedObjectNames_.count(objectHandle) > 0) {
    // if clearing, verify exists
    userLockedObjectNames_.erase(objectHandle);
  }
  return true;
}  // ManagedContainer::setLock
std::string ManagedContainerBase::getRandomObjectHandlePerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& type) const {
  std::size_t numVals = mapOfHandles.size();
  if (numVals == 0) {
    LOG(ERROR) << "Attempting to get a random " << type << objectType_
               << " managed object handle but none are loaded; Aboring";
    return "";
  }
  int randIDX = rand() % numVals;

  std::string res;
  for (std::pair<std::map<int, std::string>::const_iterator, int> iter(
           mapOfHandles.begin(), 0);
       (iter.first != mapOfHandles.end() && iter.second <= randIDX);
       ++iter.first, ++iter.second) {
    res = iter.first->second;
  }
  return res;
}  // ManagedContainer::getRandomObjectHandlePerType

std::vector<std::string>
ManagedContainerBase::getObjectHandlesBySubStringPerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& subStr,
    bool contains) const {
  std::vector<std::string> res;
  // if empty return empty vector
  if (mapOfHandles.size() == 0) {
    return res;
  }
  // if search string is empty, return all values
  if (subStr.length() == 0) {
    for (const auto& elem : mapOfHandles) {
      res.push_back(elem.second);
    }
    return res;
  }
  // build search criteria
  std::string strToLookFor = Cr::Utility::String::lowercase(subStr);

  std::size_t strSize = strToLookFor.length();

  for (std::map<int, std::string>::const_iterator iter = mapOfHandles.begin();
       iter != mapOfHandles.end(); ++iter) {
    std::string key = Cr::Utility::String::lowercase(iter->second);
    // be sure that key is big enough to search in (otherwise find has undefined
    // behavior)
    if (key.length() < strSize) {
      continue;
    }
    bool found = (std::string::npos != key.find(strToLookFor));
    if (found == contains) {
      // if found and searching for contains, or not found and searching for not
      // contains
      res.push_back(iter->second);
    }
  }
  return res;
}  // ManagedContainerBase::getObjectHandlesBySubStringPerType

std::vector<std::string>
ManagedContainerBase::getObjectHandlesBySubStringPerType(
    const std::map<std::string, std::set<std::string>>& mapOfHandles,
    const std::string& subStr,
    bool contains) const {
  std::vector<std::string> res;
  // if empty return empty vector
  if (mapOfHandles.size() == 0) {
    return res;
  }
  // if search string is empty, return all values
  if (subStr.length() == 0) {
    for (const auto& elem : mapOfHandles) {
      res.push_back(elem.first);
    }
    return res;
  }
  // build search criteria
  std::string strToLookFor = Cr::Utility::String::lowercase(subStr);

  std::size_t strSize = strToLookFor.length();

  for (std::map<std::string, std::set<std::string>>::const_iterator iter =
           mapOfHandles.begin();
       iter != mapOfHandles.end(); ++iter) {
    std::string key = Cr::Utility::String::lowercase(iter->first);
    // be sure that key is big enough to search in (otherwise find has undefined
    // behavior)
    if (key.length() < strSize) {
      continue;
    }
    bool found = (std::string::npos != key.find(strToLookFor));
    if (found == contains) {
      // if found and searching for contains, or not found and searching for not
      // contains
      res.push_back(iter->first);
    }
  }
  return res;
}  // ManagedContainerBase::getObjectHandlesBySubStringPerType

std::string ManagedContainerBase::getUniqueHandleFromCandidatePerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& name) const {
  // find all existing values with passed name - this
  std::vector<std::string> resVals =
      this->getObjectHandlesBySubStringPerType(mapOfHandles, name, true);

  int incr = 0;
  // default to illegal apple/windows character
  char pivotChar = ':';
#if __linux__
  pivotChar = '/';
#endif
  if (resVals.size() != 0) {
    // handles exist with passed substring.  Find highest handle increment, add
    // 1 and use for new name 1, build new handle
    for (const std::string& s : resVals) {
      // split string on underscore, last value will be string of highest incr
      // value existing.
      std::vector<std::string> vals = Cr::Utility::String::split(s, pivotChar);
      // if any exist, all are expected to end
      int new_incr = std::stoi(vals.back());

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

}  // namespace core
}  // namespace esp
