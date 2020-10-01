// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

const std::string LightAttributes::JSONConfigTestString = R"({
      "position": [2.45833,0.1,3.84271],
      "color_scale": -0.1,
      "color": [2,1,-1]
    })";

LightAttributes::LightAttributes(const std::string& handle)
    : AbstractAttributes("LightAttributes", handle) {
  setPosition({0.0, 0.0, 0.0});
  setColor({1.0, 1.0, 1.0});
  setColorScale(1.0);
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
