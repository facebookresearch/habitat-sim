// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ManagedContainerBase.h"

namespace esp {
namespace core {

std::string ManagedContainerBase::convertFilenameToJSON(
    const std::string& filename,
    const std::string& jsonTypeExt) {
  std::string strHandle = Cr::Utility::String::lowercase(filename);
  std::string resHandle(filename);
  if (std::string::npos ==
      strHandle.find(Cr::Utility::String::lowercase(jsonTypeExt))) {
    resHandle = Cr::Utility::Directory::splitExtension(filename).first + "." +
                jsonTypeExt;
    LOG(INFO) << "ManagedContainerBase::convertFilenameToJSON : Filename : "
              << filename
              << " changed to proposed JSON configuration filename : "
              << resHandle;
  } else {
    LOG(INFO) << "ManagedContainerBase::convertFilenameToJSON : Filename : "
              << filename << " is appropriate JSON configuration filename.";
  }
  return resHandle;
}  // ManagedContainerBase::convertFilenameToJSON

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

bool ManagedContainerBase::verifyLoadDocument(const std::string& filename,
                                              io::JsonDocument& jsonDoc) {
  if (isValidFileName(filename)) {
    try {
      jsonDoc = io::parseJsonFile(filename);
    } catch (...) {
      LOG(ERROR)
          << objectType_
          << "ManagedContainerBase::verifyLoadDocument : Failed to parse "
          << filename << " as JSON.";
      return false;
    }
    return true;
  } else {
    // by here always fail
    LOG(ERROR) << objectType_
               << "ManagedContainerBase::verifyLoadDocument : File " << filename
               << " does not exist";
    return false;
  }
}  // ManagedContainerBase::verifyLoadDocument

}  // namespace core
}  // namespace esp
