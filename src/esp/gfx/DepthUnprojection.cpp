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

Mn::Matrix2x2 calculateDepthUnprojection(const Mn::Matrix4& projectionMatrix) {
  /* Inverted projection matrix to unproject the depth value and chop the
     near plane off. We don't care about X/Y there and the corresponding
     parts of the matrix are zero as well so take just the lower-right part
     of it (denoted a, b, c, d).

      x 0 0 0
      0 y 0 0
      0 0 a b
      0 0 c d

     Doing an inverse of just the bottom right block is enough as well -- see
     https://en.wikipedia.org/wiki/Block_matrix#Block_diagonal_matrices for
     a proof.

     Taking a 2-component vector with the first component being Z and second
     1, the final calculation of unprojected Z is then

      | a b |   | z |   | az + b |
      | c d | * | 1 | = | cz + d |

  */
  auto unprojection =
      Mn::Matrix2x2{Mn::Math::swizzle<'z', 'w'>(projectionMatrix[2]),
                    Mn::Math::swizzle<'z', 'w'>(projectionMatrix[3])}
          .inverted();

  /* The Z value comes in range [0; 1], but we need it in the range [-1; 1].
     Instead of doing z = x*2 - 1 for every pixel, we add that to this
     matrix:

      az + b
      a(x*2 - 1) + b
      2ax - a + b
      (2a)x + (b - a)

    and similarly for c/d. Which means -- from the second component we
    subtract the first, and the first we multiply by 2. */
  unprojection[1] -= unprojection[0];
  unprojection[0] *= 2.0;

  /* Finally, because the output has Z going forward, not backward, we need
     to negate it. There's a perspective division happening, so we have to
     negate just the first row. */
  unprojection.setRow(0, -unprojection.row(0));

  return unprojection;
}

void unprojectDepth(const Mn::Matrix2x2& unprojection,
                    Cr::Containers::ArrayView<Mn::Float> depth) {
  for (std::size_t i = 0; i != depth.size(); ++i) {
    /* We can afford using == for comparison as 1.0f has an exact
       representation and the depth is cleared to exactly this value. */
    if (depth[i] == 1.0f) {
      depth[i] = 0.0f;
      continue;
    }

    /* The following is

        (az + b) / (cz + d)

       See the comment in draw() above for details. */
    depth[i] = Mn::Math::fma(unprojection[0][0], depth[i], unprojection[1][0]) /
               Mn::Math::fma(unprojection[0][1], depth[i], unprojection[1][1]);
  }
}

}  // namespace gfx
}  // namespace esp
