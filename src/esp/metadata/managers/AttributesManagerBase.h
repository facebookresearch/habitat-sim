// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_ATTRIBUTESMANAGERBASE_H_
#define ESP_METADATA_MANAGERS_ATTRIBUTESMANAGERBASE_H_

/** @file
 * @brief Class Template @ref esp::metadata::managers::AttributesManager
 */

#include "esp/metadata/attributes/AttributesBase.h"

#include "esp/core/ManagedContainerBase.h"
#include "esp/io/io.h"

namespace Cr = Corrade;

namespace esp {
namespace core {
class ManagedContainerBase;
}
namespace metadata {
namespace managers {
namespace Attrs = esp::metadata::attributes;
/**
 * @brief Class template defining responsibilities and functionality for
 * managing @ref esp::metadata::attributes::AbstractAttributes constructs.
 * @tparam T the type of managed attributes a particular specialization
 * of this class works with.  Must inherit from @ref
 * esp::metadata::attributes::AbstractAttributes.
 */
template <class T>
class AttributesManager : public esp::core::ManagedContainer<T> {
 public:
  static_assert(std::is_base_of<Attrs::AbstractAttributes, T>::value,
                "AttributesManager :: Managed object type must be derived from "
                "AbstractAttributes");

  typedef std::shared_ptr<T> AttribsPtr;

  AttributesManager(const std::string& attrType, const std::string& JSONTypeExt)
      : esp::core::ManagedContainer<T>::ManagedContainer(attrType),
        JSONTypeExt_(JSONTypeExt) {}
  virtual ~AttributesManager() = default;

  /**
   * @brief Check if currently configured primitive asset template library has
   * passed handle.
   * @param handle String name of primitive asset attributes desired
   * @return whether handle exists or not in asset attributes library
   */
  virtual bool isValidPrimitiveAttributes(const std::string& handle) = 0;

 protected:
  // ======== Typedefs and Instance Variables ========
  /**
   * @brief The string extension for json files for this manager's attributes
   * types
   */
  const std::string JSONTypeExt_;

 public:
  ESP_SMART_POINTERS(AttributesManager<AttribsPtr>)

};  // class AttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_MANAGERS_ATTRIBUTESMANAGERBASE_H_
