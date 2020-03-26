// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>

namespace esp {
namespace gfx {

/**
@brief Triangle shader

Outputs unique ID values for each triangle in the mesh.
*/
class TriangleShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /** @brief Flag */
  enum class Flag {};

  /** @brief Flags */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /** @brief Constructor */
  explicit TriangleShader(Flags flags = {});

  /**
   * @brief The flags passed to the Constructor
   */
  Flags flags() const { return flags_; }

 private:
  const Flags flags_;
};

CORRADE_ENUMSET_OPERATORS(TriangleShader::Flags)

}  // namespace gfx
}  // namespace esp
