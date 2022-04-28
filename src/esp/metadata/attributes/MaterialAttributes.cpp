// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

MaterialAttributes::MaterialAttributes(const std::string& handle)
    : AbstractAttributes("MaterialAttributes", handle) {}
MaterialAttributes::MaterialAttributes(const MaterialAttributes& otr)
    : AbstractAttributes(otr) {}
MaterialAttributes::MaterialAttributes(MaterialAttributes&& otr) noexcept
    : AbstractAttributes(std::move(static_cast<AbstractAttributes>(otr))) {}

void MaterialAttributes::writeValuesToJson(io::JsonGenericValue& jsonObj,
                                           io::JsonAllocator& allocator) const {
}  // MaterialAttributes::writeValuesToJson

MaterialAttributes& MaterialAttributes::operator=(
    const MaterialAttributes& otr) {
  if (this != &otr) {
    this->AbstractAttributes::operator=(otr);
  }
  return *this;
}
MaterialAttributes& MaterialAttributes::operator=(
    MaterialAttributes&& otr) noexcept {
  this->AbstractAttributes::operator=(
      std::move(static_cast<AbstractAttributes>(otr)));
  return *this;
}

std::string MaterialAttributes::getObjectInfoHeaderInternal() const {
  return "";
}
std::string MaterialAttributes::getObjectInfoInternal() const {
  std::string res = "\n";

  return res;
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
