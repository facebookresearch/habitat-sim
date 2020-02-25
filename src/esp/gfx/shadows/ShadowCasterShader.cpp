/*

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include "ShadowCasterShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Matrix4.h>

// This is to import the "resources" at runtime.
// When the resource is compiled into static library,
// it must be explicitly initialized via this macro, and should be called
// *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace esp {
namespace gfx {

namespace Mn = Magnum;

ShadowCasterShader::ShadowCasterShader() {
  MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Mn::GL::Version::GL330);

  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  const Mn::Utility::Resource rs{"default-shaders"};

  Mn::GL::Shader vert{Mn::GL::Version::GL330, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{Mn::GL::Version::GL330, Mn::GL::Shader::Type::Fragment};

  vert.addSource(rs.get("ShadowCaster.vert"));
  frag.addSource(rs.get("ShadowCaster.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  _transformationMatrixUniform = uniformLocation("transformationMatrix");
}

ShadowCasterShader& ShadowCasterShader::setTransformationMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(_transformationMatrixUniform, matrix);
  return *this;
}

}  // namespace gfx
}  // namespace esp
