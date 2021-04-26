// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_MANAGEDFILEBASEDCONTAINER_H_
#define ESP_CORE_MANAGEDFILEBASEDCONTAINER_H_

/** @file
 * @brief Class Template @ref esp::core::ManagedFileBasedContainer : @ref
 * esp::core::ManagedContainer functionality specifically for file-based @ref
 * esp::core::AbstractManagedObject objects
 */

#include "ManagedContainer.h"

namespace esp {
namespace core {
/**
 * @brief Class template defining file-io-based responsibilities and
 * functionality for managing @ref esp::core::AbstractManagedObject constructs.
 * @tparam T the type of managed object a particular specialization of
 * this class works with.  Must inherit from @ref
 * esp::core::AbstractManagedObject.
 * @tparam Access Whether the default access (getters) for this
 * container provides copies of the objects held, or the actual objects
 * themselves.
 */
template <class T, ManagedObjectAccess Access>
class ManagedFileBasedContainer : public ManagedContainer<T, Access> {
 public:
  typedef std::shared_ptr<T> ManagedFileIOPtr;

  explicit ManagedFileBasedContainer(const std::string& metadataType)
      : ManagedContainer<T, Access>(metadataType) {}

  /**
   * @brief Creates an instance of a managed object from a JSON file.
   *
   * @param filename the name of the file describing the object managed object.
   * Assumes it exists and fails if it does not.
   * @param registerObject whether to add this managed object to the
   * library. If the user is going to edit this managed object, this should be
   * false - any subsequent editing will require re-registration. Defaults to
   * true.
   * @return a reference to the desired managed object, or nullptr if fails.
   */
  ManagedFileIOPtr createObjectFromJSONFile(const std::string& filename,
                                            bool registerObject = true) {
    io::JsonDocument docConfig = nullptr;
    bool success = this->verifyLoadDocument(filename, docConfig);
    if (!success) {
      LOG(ERROR) << "ManagedFileBasedContainer::createObjectFromFile ("
                 << this->objectType_
                 << ") : Failure reading document as JSON : " << filename
                 << ". Aborting.";
      return nullptr;
    }
    // convert doc to const value
    const io::JsonGenericValue config = docConfig.GetObject();
    ManagedFileIOPtr attr = this->buildManagedObjectFromDoc(filename, config);
    return this->postCreateRegister(attr, registerObject);
  }  // ManagedFileBasedContainer::createObjectFromJSONFile

  /**
   * @brief Method to load a Managed Object's data from a file.  If the file
   * type is not supported by specialization of this method, this method
   * executes and an error is thrown.
   * @tparam type of document to load.
   * @param filename name of file document to load from
   * @param config document to read for data
   * @return a shared pointer of the created Managed Object
   */
  template <typename U>
  ManagedFileIOPtr buildManagedObjectFromDoc(const std::string& filename,
                                             CORRADE_UNUSED const U& config) {
    LOG(ERROR)
        << "ManagedContainer::buildManagedObjectFromDoc (" << this->objectType_
        << ") : Failure loading attributes from document of unknown type : "
        << filename << ". Aborting.";
  }
  /**
   * @brief Method to load a Managed Object's data from a file.  This is the
   * JSON specialization, using type inference.
   * @param filename name of file document to load from
   * @param config JSON document to read for data
   * @return a shared pointer of the created Managed Object
   */
  ManagedFileIOPtr buildManagedObjectFromDoc(
      const std::string& filename,
      const io::JsonGenericValue& jsonConfig) {
    return this->buildObjectFromJSONDoc(filename, jsonConfig);
  }

  /**
   * @brief Parse passed JSON Document specifically for @ref ManagedPtr object.
   * It always returns a @ref ManagedPtr object.
   * @param filename The name of the file describing the @ref ManagedPtr,
   * used as managed object handle/name on create.
   * @param jsonConfig json document to parse - assumed to be legal JSON doc.
   * @return a reference to the desired managed object.
   */
  virtual ManagedFileIOPtr buildObjectFromJSONDoc(
      const std::string& filename,
      const io::JsonGenericValue& jsonConfig) = 0;

 protected:
  /**
   * @brief Get directory component of managed object handle and call @ref
   * esp::core::AbstractManagedObject::setFileDirectory if a legitimate
   * directory exists in handle.
   *
   * @param object pointer to managed object to set
   */
  void setFileDirectoryFromHandle(ManagedFileIOPtr object) {
    std::string handleName = object->getHandle();
    auto loc = handleName.find_last_of('/');
    if (loc != std::string::npos) {
      object->setFileDirectory(handleName.substr(0, loc));
    }
  }  // setFileDirectoryFromHandle

 public:
  ESP_SMART_POINTERS(ManagedFileBasedContainer<T, Access>)

};  // class ManagedFileBasedContainer

}  // namespace core
}  // namespace esp
#endif  // ESP_CORE_MANAGEDFILEBASEDCONTAINER_H_
