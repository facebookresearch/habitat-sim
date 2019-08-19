// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Containers.h>
#include <Magnum/Magnum.h>

namespace esp {
namespace gfx {

/** @brief Calculate depth unprojection coefficients */
Magnum::Matrix2x2 calculateDepthUnprojection(
    const Magnum::Matrix4& projectionMatrix);

/**
@brief Unproject depth values
@param[in] unprojection Unprojection coefficients from
    @ref calculateDepthUnprojection()
@param[in,out] depth    Depth values in range @f$ [ 0 ; 1 ] @f$

See @ref calculateDepthUnprojection() for the full algorithm explanation.
Additionally to applying that calculation, if the input depth is at the far
plane (of value @cpp 1.0f @ce), it's set to @cpp 0.0f @ce on output as
consumers expect zeros for things that are too far.
*/
void unprojectDepth(const Magnum::Matrix2x2& unprojection,
                    Corrade::Containers::ArrayView<Magnum::Float> depth);

}  // namespace gfx
}  // namespace esp
