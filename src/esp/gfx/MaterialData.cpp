// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialData.h"

namespace esp {
namespace gfx {

bool operator==(const PhongMaterialInfo& a, const PhongMaterialInfo& b) {
  return a.shininess == b.shininess && a.ambientColor == b.ambientColor &&
         a.diffuseColor == b.diffuseColor &&
         a.specularColor == b.specularColor && a.importName == b.importName;
}

bool operator!=(const PhongMaterialInfo& a, const PhongMaterialInfo& b) {
  return !(a == b);
}

}  // namespace gfx
}  // namespace esp
