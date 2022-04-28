// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_MATERIALATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_MATERIALATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

class MaterialAttributes : public AbstractAttributes {
 public:
  explicit MaterialAttributes(const std::string& handle = "");

  MaterialAttributes(const MaterialAttributes& otr);
  MaterialAttributes(MaterialAttributes&& otr) noexcept;

  MaterialAttributes& operator=(const MaterialAttributes& otr);
  MaterialAttributes& operator=(MaterialAttributes&& otr) noexcept;

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * MaterialAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific. The individual light
   * instances return a header for this.
   */
  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(MaterialAttributes)
};  // class MaterialAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_MATERIALATTRIBUTES_H_
