// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PrimitiveIDShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>

// This is to import the "resources" at runtime.
// When the resource is compiled into static library,
// it must be explicitly initialized via this macro, and should be called
// *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace Cr = Corrade;

namespace esp {
namespace gfx {

PrimitiveIDShader::PrimitiveIDShader() {
#ifndef MAGNUM_TARGET_WEBGL
  MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Magnum::GL::Version::GL410);
#endif

  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  // this is not the file name, but the group name in the config file
  const Corrade::Utility::Resource rs{"default-shaders"};

#ifdef MAGNUM_TARGET_WEBGL
  Magnum::GL::Version glVersion = Magnum::GL::Version::GLES300;
#else
  Magnum::GL::Version glVersion = Magnum::GL::Version::GL410;
#endif

  Magnum::GL::Shader vert{glVersion, Magnum::GL::Shader::Type::Vertex};
  Magnum::GL::Shader frag{glVersion, Magnum::GL::Shader::Type::Fragment};

  vert.addSource(rs.get("primitive-id-textured-gl410.vert"));
  frag.addSource(rs.get("primitive-id-textured-gl410.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Magnum::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  transformationProjectionMatrixUniform_ =
      uniformLocation("transformationProjectionMatrix");
}

}  // namespace gfx
}  // namespace esp
