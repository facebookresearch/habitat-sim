// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

SceneObjectInstanceAttributes::SceneObjectInstanceAttributes(
    const std::string& handle)
    : AbstractAttributes("SceneObjectInstanceAttributes", handle) {}
SceneAttributes::SceneAttributes(const std::string& handle)
    : AbstractAttributes("SceneAttributes", handle) {}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
