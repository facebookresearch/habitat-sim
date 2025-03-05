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
   * @brief Set the directory where files used to construct ManagedObject can be
   * found.
   */
  virtual void setFileDirectory(const std::string& fileDirectory) = 0;

  /**
   * @brief Get directory where files used to construct ManagedObject can be
   * found.
   */
  virtual std::string getFileDirectory() const = 0;

  /**
   * @brief Set the fully qualified filename of the file used to create or most
   * recently save this ManagedObject.
   */
  virtual void setActualFilename(const std::string& fullFileName) = 0;

  /**
   * @brief Get the fully qualified filename of the file used to create or most
   * recently save this ManagedObject.
   */
  virtual std::string getActualFilename() const = 0;

  /**
   * @brief Get whether this ManagedObject has been saved to disk in its current
   * state. Only applicable to registered ManagedObjects
   */
  virtual bool isAttrSaved() const = 0;

  /**
   * @brief Set that this ManagedObject has values that are different than its
   * most recently saved-to-disk version. This is called when the ManagedObject
   * is registered.
   */

  void setAttrIsNotSaved() { setFileSaveStatus(false); }

  /**
   * @brief Set that this ManagedObject is the same as its saved-to-disk
   * version. This is called when the ManagedObject is saved to disk.
   */
  void setAttrIsSaved() { setFileSaveStatus(true); }

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
                   Corrade::Utility::Path::filename(getHandle()))
                   .first())
        .first();
  }

  /**
   * @brief Build and return a json object holding the pertinent data for this
   * AbstractFileBasedManagedObject.
   */
  virtual io::JsonGenericValue writeToJsonObject(
      io::JsonAllocator& allocator) const = 0;

 protected:
  /**
   * @brief Set this ManagedObject's save status (i.e. whether it matches its
   * version on disk or not)
   */
  virtual void setFileSaveStatus(bool _isSaved) = 0;

 public:
  ESP_SMART_POINTERS(AbstractFileBasedManagedObject)
};  // class AbstractFileBasedManagedObject
}  // namespace managedContainers
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_ABSTRACTFILEBASEDMANAGEDOBJECT_H_
