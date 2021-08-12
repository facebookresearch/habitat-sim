// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ManagedContainerBase.h"
#include <Corrade/Utility/FormatStl.h>
#include <algorithm>

namespace Cr = Corrade;

namespace esp {
namespace core {

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
  } else if (userLockedObjectNames_.count(objectHandle) > 0) {
    // if clearing, verify exists
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
                << "managed object handle but none are loaded; Aboring";
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

std::vector<std::string>
ManagedContainerBase::getObjectHandlesBySubStringPerType(
    const std::unordered_map<int, std::string>& mapOfHandles,
    const std::string& subStr,
    bool contains,
    bool sorted) const {
  std::vector<std::string> res;
  // if empty return empty vector
  if (mapOfHandles.size() == 0) {
    return res;
  }
  // if search string is empty, return all values
  if (subStr.length() == 0) {
    res.reserve(mapOfHandles.size());
    for (const auto& elem : mapOfHandles) {
      res.push_back(elem.second);
    }
    if (sorted) {
      std::sort(res.begin(), res.end());
    }
    return res;
  }
  // build search criteria
  std::string strToLookFor = Cr::Utility::String::lowercase(subStr);

  std::size_t strSize = strToLookFor.length();

  for (std::unordered_map<int, std::string>::const_iterator iter =
           mapOfHandles.begin();
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
  if (sorted) {
    std::sort(res.begin(), res.end());
  }
  return res;
}  // ManagedContainerBase::getObjectHandlesBySubStringPerType

std::vector<std::string>
ManagedContainerBase::getObjectHandlesBySubStringPerType(
    const std::unordered_map<std::string, std::set<std::string>>& mapOfHandles,
    const std::string& subStr,
    bool contains,
    bool sorted) const {
  std::vector<std::string> res;
  // if empty return empty vector
  if (mapOfHandles.size() == 0) {
    return res;
  }
  // if search string is empty, return all values
  if (subStr.length() == 0) {
    res.reserve(mapOfHandles.size());
    for (const auto& elem : mapOfHandles) {
      res.push_back(elem.first);
    }
    if (sorted) {
      std::sort(res.begin(), res.end());
    }
    return res;
  }
  // build search criteria
  std::string strToLookFor = Cr::Utility::String::lowercase(subStr);

  std::size_t strSize = strToLookFor.length();

  for (std::unordered_map<std::string, std::set<std::string>>::const_iterator
           iter = mapOfHandles.begin();
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
  if (sorted) {
    std::sort(res.begin(), res.end());
  }
  return res;
}  // ManagedContainerBase::getObjectHandlesBySubStringPerType

std::vector<std::string> ManagedContainerBase::getObjectInfoStrings(
    const std::string& subStr,
    bool contains) const {
  // get all handles that match query elements first
  std::vector<std::string> handles =
      this->getObjectHandlesBySubstring(subStr, contains, true);
  std::vector<std::string> res(handles.size() + 1);
  if (handles.size() == 0) {
    res[0] = "No " + objectType_ + " constructs available.";
    return res;
  }
  int idx = 0;
  for (const std::string& objectHandle : handles) {
    // get the object
    auto objPtr = getObjectInternal<AbstractManagedObject>(objectHandle);
    if (idx == 0) {
      res[idx++]
          .append(objectType_)
          .append(" Full name, Can delete?, Is locked?, ")
          .append(objPtr->getObjectInfoHeader());
    }
    res[idx++]
        .append(objectHandle)
        .append(1, ',')
        .append(((this->getIsUndeletable(objectHandle)) ? "False, " : "True, "))
        .append(((this->getIsUserLocked(objectHandle)) ? "True, " : "False, "))
        .append(objPtr->getObjectInfo());
  }
  return res;
}  // ManagedContainerBase::getObjectInfoStrings

int ManagedContainerBase::getObjectIDByHandleOrNew(
    const std::string& objectHandle,
    bool getNext) {
  if (getObjectLibHasHandle(objectHandle)) {
    return getObjectInternal<AbstractManagedObject>(objectHandle)->getID();
  }
  if (!getNext) {
    ESP_ERROR() << "<" << Cr::Utility::Debug::nospace << this->objectType_
                << Cr::Utility::Debug::nospace << "> : No" << objectType_
                << "managed object with handle" << objectHandle
                << "exists. Aborting";
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
    res += s.append(1, '\n');
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

  if (objHandles.size() != 0) {
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

}  // namespace core
}  // namespace esp
