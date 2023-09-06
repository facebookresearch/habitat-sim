// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_ABSTRACTFILEBASEDMANAGEDOBJECT_H_
#define ESP_CORE_ABSTRACTFILEBASEDMANAGEDOBJECT_H_

#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/Utility/Path.h>
#include <string>
#include "AbstractManagedObject.h"
#include "esp/io/Json.h"

namespace esp {
namespace core {

namespace managedContainers {
class AbstractFileBasedManagedObject : public AbstractManagedObject {
 public:
  /**
   * @brief directory where files used to construct ManagedObject can be found.
   */
  virtual void setFileDirectory(const std::string& fileDirectory) = 0;
  virtual std::string getFileDirectory() const = 0;

  /**
   * @brief This will return a simplified version of the
   * AbstractFileBasedManagedObject handle, removing extensions and any parent
   * directories in name. Note : there's no guarantee this handle will be
   * sufficiently unique to identify this AbstractFileBasedManagedObject, so
   * this should only be used for logging, and not for attempts to search for
   * AbstractFileBasedManagedObjects.
   */
  virtual std::string getSimplifiedHandle() const {
    // first parse for file name, and then get rid of extension(s).
    return Corrade::Utility::Path::splitExtension(
               Corrade::Utility::Path::splitExtension(
                   Corrade::Utility::Path::split(getHandle()).second())
                   .first())
        .first();
  }

  /**
   * @brief Build and return a json object holding the pertinent data for this
   * AbstractFileBasedManagedObject.
   */
  virtual io::JsonGenericValue writeToJsonObject(
      io::JsonAllocator& allocator) const = 0;

 public:
  ESP_SMART_POINTERS(AbstractFileBasedManagedObject)
};  // class AbstractFileBasedManagedObject
}  // namespace managedContainers
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_ABSTRACTFILEBASEDMANAGEDOBJECT_H_
