// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DepthUnprojection.h"

#include <Corrade/Containers/ArrayView.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/Matrix4.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

Mn::Vector2 calculateDepthUnprojection(const Mn::Matrix4& projectionMatrix) {
  return Mn::Vector2{(projectionMatrix[2][2] - 1.0f), projectionMatrix[3][2]} *
         0.5f;
}

void unprojectDepth(const Mn::Vector2& unprojection,
                    Cr::Containers::ArrayView<Mn::Float> depth) {
  for (std::size_t i = 0; i != depth.size(); ++i) {
    /* We can afford using == for comparison as 1.0f has an exact
       representation and the depth is cleared to exactly this value. */
    if (depth[i] == 1.0f) {
      depth[i] = 0.0f;
      continue;
    }

    depth[i] = unprojection[1] / (depth[i] + unprojection[0]);
  }
}

}  // namespace gfx
}  // namespace esp
