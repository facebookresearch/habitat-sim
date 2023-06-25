// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DepthUnprojection.h"

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/Matrix4.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(GfxBatchShaderResources)
}

namespace esp {
namespace gfx_batch {

namespace {
enum { DepthTextureUnit = 1 };
}

DepthShader::DepthShader(Flags flags) : flags_{flags} {
  if (!Corrade::Utility::Resource::hasGroup("gfx-batch-shaders")) {
    importShaderResources();
  }
  const Corrade::Utility::Resource rs{"gfx-batch-shaders"};

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  if (flags & Flag::UnprojectExistingDepth) {
    vert.addSource("#define UNPROJECT_EXISTING_DEPTH\n");
    frag.addSource("#define UNPROJECT_EXISTING_DEPTH\n");
  }

  if (flags & Flag::NoFarPlanePatching)
    frag.addSource("#define NO_FAR_PLANE_PATCHING\n");

  vert.addSource(rs.getString("depth.vert"));
  frag.addSource(rs.getString("depth.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(vert.compile() && frag.compile());

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  if (flags & Flag::UnprojectExistingDepth) {
    projectionMatrixOrDepthUnprojectionUniform_ =
        uniformLocation("depthUnprojection");
    setUniform(uniformLocation("depthTexture"), DepthTextureUnit);
  } else {
    transformationMatrixUniform_ = uniformLocation("transformationMatrix");
    projectionMatrixOrDepthUnprojectionUniform_ =
        uniformLocation("projectionMatrix");
  }
}

DepthShader& DepthShader::setDepthUnprojection(
    const Mn::Vector2& depthUnprojection) {
  CORRADE_INTERNAL_ASSERT(flags_ & Flag::UnprojectExistingDepth);
  setUniform(projectionMatrixOrDepthUnprojectionUniform_, depthUnprojection);
  return *this;
}

DepthShader& DepthShader::setTransformationMatrix(const Mn::Matrix4& matrix) {
  CORRADE_INTERNAL_ASSERT(!(flags_ & Flag::UnprojectExistingDepth));
  setUniform(transformationMatrixUniform_, matrix);
  return *this;
}

DepthShader& DepthShader::setProjectionMatrix(const Mn::Matrix4& matrix) {
  if (flags_ & Flag::UnprojectExistingDepth) {
    setUniform(projectionMatrixOrDepthUnprojectionUniform_,
               calculateDepthUnprojection(matrix));
  } else {
    setUniform(projectionMatrixOrDepthUnprojectionUniform_, matrix);
  }
  return *this;
}

DepthShader& DepthShader::bindDepthTexture(Mn::GL::Texture2D& texture) {
  texture.bind(DepthTextureUnit);
  return *this;
}

Mn::Vector2 calculateDepthUnprojection(const Mn::Matrix4& projectionMatrix) {
  return Mn::Vector2{(projectionMatrix[2][2] - 1.0f), projectionMatrix[3][2]} *
         0.5f;
}

/* Clang doesn't have target_clones yet: https://reviews.llvm.org/D51650 */
#if defined(CORRADE_TARGET_X86) && defined(__GNUC__) && __GNUC__ >= 6
__attribute__((target_clones("default", "sse4.2", "avx2")))
#endif
void unprojectDepth(
    const Mn::Vector2& unprojection,
    const Cr::Containers::StridedArrayView2D<Mn::Float>& depth) {
  for (Cr::Containers::StridedArrayView1D<Mn::Float> row : depth) {
    for (Mn::Float& d : row) {
      d = unprojection[1] / (d + unprojection[0]);
    }
  }

  /* Change pixels on the far plane to be 0. Done in a separate loop to allow
     the optimizer to vectorize the above better.  */
  const Mn::Float farDepth = unprojection[1] / (1.0f + unprojection[0]);
  for (Cr::Containers::StridedArrayView1D<Mn::Float> row : depth) {
    for (Mn::Float& d : row) {
      /* We can afford using == for comparison as 1.0f has an exact
         representation, the depth was cleared to exactly this value and the
         calculation is done exactly the same way in both cases -- thus the
         result should be bit-exact. */
      if (d == farDepth)
        d = 0.0f;
    }
  }
}

}  // namespace gfx_batch
}  // namespace esp
