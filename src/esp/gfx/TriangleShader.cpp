// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "TriangleShader.h"

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/Matrix4.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace esp {
namespace gfx {

namespace {
enum { DepthTextureUnit = 1 };
}

TriangleShader::TriangleShader(Flags flags) : flags_{flags} {
  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  const Corrade::Utility::Resource rs{"default-shaders"};

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL410;
#endif

  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};
  frag.addSource(rs.get("triangle.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({frag}));

  attachShaders({frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());
}

/* Clang doesn't have target_clones yet: https://reviews.llvm.org/D51650 */
#if defined(CORRADE_TARGET_X86) && defined(__GNUC__) && __GNUC__ >= 6
__attribute__((target_clones("default", "sse4.2", "avx2")))
#endif

}  // namespace gfx
}  // namespace esp
