/*
    Originally written by Vladimir Vondrus as part of the Magnum Library

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019
              Vladimír Vondruš <mosra@centrum.cz>
*/

#include "PhongShadowReceiverShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Corrade/Containers/EnumSet.hpp>

#include "Magnum/GL/Context.h"
#include "Magnum/GL/Extensions.h"
#include "Magnum/GL/Shader.h"
#include "Magnum/GL/Texture.h"
#include "Magnum/GL/TextureArray.h"
#include "Magnum/Math/Color.h"
#include "Magnum/Math/Matrix4.h"

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
namespace Cr = Corrade;

namespace {
enum : Magnum::Int {
  AmbientTextureLayer = 0,
  DiffuseTextureLayer = 1,
  SpecularTextureLayer = 2,
  NormalTextureLayer = 3,
  ShadowmapTextureLayer = 4,
};
}

PhongShadowReceiverShader::PhongShadowReceiverShader(
    const Flags flags,
    const Magnum::UnsignedInt lightCount,
    const Magnum::UnsignedInt numShadowMapLayers)
    : _flags{flags},
      _lightCount{lightCount},
      numShadowMapLayers_{numShadowMapLayers} {
#ifndef MAGNUM_TARGET_WEBGL
  MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Magnum::GL::Version::GL410);
#endif
  /* Import resources on static build, if not already */
  if (!Cr::Utility::Resource::hasGroup("default-shaders"))
    importShaderResources();
  Cr::Utility::Resource rs("default-shaders");

#ifdef MAGNUM_TARGET_WEBGL
  Magnum::GL::Version version = Magnum::GL::Version::GLES300;
#else
  Magnum::GL::Version version = Magnum::GL::Version::GL410;
#endif

  Mn::GL::Shader vert{version, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{version, Mn::GL::Shader::Type::Fragment};

  std::string lightInitializer;
  if (lightCount) {
    /* Initializer for the light color array -- we need a list of vec4(1.0)
       joined by commas. For GLES we'll simply upload the values directly. */
    constexpr const char lightInitializerPreamble[] =
        "#define LIGHT_COLOR_INITIALIZER ";
    constexpr std::size_t lightInitializerPreambleSize =
        Cr::Containers::arraySize(lightInitializerPreamble) - 1;
    constexpr const char lightInitializerItem[] = "vec4(1.0), ";
    constexpr std::size_t lightInitializerItemSize =
        Cr::Containers::arraySize(lightInitializerItem) - 1;
    lightInitializer.reserve(
        Cr::Containers::arraySize(lightInitializerPreamble) - 1 +
        lightCount * lightInitializerItemSize);
    lightInitializer.append(lightInitializerPreamble,
                            lightInitializerPreambleSize);
    for (std::size_t i = 0; i != lightCount; ++i)
      lightInitializer.append(lightInitializerItem, lightInitializerItemSize);

    /* Drop the last comma and add a newline at the end */
    lightInitializer[lightInitializer.size() - 2] = '\n';
    lightInitializer.resize(lightInitializer.size() - 1);
  }

  std::string shadowMapLevelsInitializer = "#define NUM_SHADOW_MAP_LEVELS " +
                                           std::to_string(numShadowMapLayers_) +
                                           "\n";

  vert.addSource(flags & (Flag::AmbientTexture | Flag::DiffuseTexture |
                          Flag::SpecularTexture | Flag::NormalTexture)
                     ? "#define TEXTURED\n"
                     : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags & Flag::VertexColor ? "#define VERTEX_COLOR\n" : "")
      .addSource(
          Cr::Utility::formatString("#define LIGHT_COUNT {}\n", lightCount))
      .addSource(shadowMapLevelsInitializer)
      .addSource(rs.get("phong-shadow-receiver.vert"));
  frag.addSource(flags & Flag::AmbientTexture ? "#define AMBIENT_TEXTURE\n"
                                              : "")
      .addSource(flags & Flag::DiffuseTexture ? "#define DIFFUSE_TEXTURE\n"
                                              : "")
      .addSource(flags & Flag::SpecularTexture ? "#define SPECULAR_TEXTURE\n"
                                               : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags & Flag::VertexColor ? "#define VERTEX_COLOR\n" : "")
      .addSource(flags & Flag::AlphaMask ? "#define ALPHA_MASK\n" : "")
      .addSource(flags & Flag::ObjectId ? "#define OBJECT_ID\n" : "")
      .addSource(shadowMapLevelsInitializer)
      .addSource(
          Cr::Utility::formatString("#define LIGHT_COUNT {}\n", lightCount));
  if (lightCount)
    frag.addSource(std::move(lightInitializer));
  frag.addSource(rs.get("phong-shadow-receiver.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  _transformationMatrixUniform = uniformLocation("transformationMatrix");
  _projectionMatrixUniform = uniformLocation("projectionMatrix");
  _ambientColorUniform = uniformLocation("ambientColor");
  if (lightCount) {
    _normalMatrixUniform = uniformLocation("normalMatrix");
    _diffuseColorUniform = uniformLocation("diffuseColor");
    _specularColorUniform = uniformLocation("specularColor");
    _shininessUniform = uniformLocation("shininess");
    _lightPositionsUniform = uniformLocation("lightPositions");
    _lightColorsUniform = uniformLocation("lightColors");
    // shadows
    modelMatrixUniform_ = uniformLocation("modelMatrix");
    shadowLightDirectionUniform_ = uniformLocation("shadowLightDirection");
    shadowBiasUniform_ = uniformLocation("shadowBias");
    shadowMapMatrixUniform_ = uniformLocation("shadowmapMatrix");
    shadeFacesFacingAwayFromLight_ =
        uniformLocation("shadeFacesFacingAwayFromLight");
  }
  if (flags & Flag::AlphaMask)
    _alphaMaskUniform = uniformLocation("alphaMask");
  if (flags & Flag::ObjectId)
    _objectIdUniform = uniformLocation("objectId");

  if (flags & Flag::AmbientTexture)
    setUniform(uniformLocation("ambientTexture"), AmbientTextureLayer);
  if (lightCount) {
    if (flags & Flag::DiffuseTexture)
      setUniform(uniformLocation("diffuseTexture"), DiffuseTextureLayer);
    if (flags & Flag::SpecularTexture)
      setUniform(uniformLocation("specularTexture"), SpecularTextureLayer);
    if (flags & Flag::NormalTexture)
      setUniform(uniformLocation("normalTexture"), NormalTextureLayer);

    setUniform(uniformLocation("shadowmapTexture"), ShadowmapTextureLayer);
  }
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setAmbientColor(
    const Magnum::Color4& color) {
  setUniform(_ambientColorUniform, color);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::bindAmbientTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(_flags & Flag::AmbientTexture,
                 "PhongShadowReceiverShader::bindAmbientTexture(): "
                 "the shader was not "
                 "created with ambient texture enabled",
                 *this);
  texture.bind(AmbientTextureLayer);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setDiffuseColor(
    const Magnum::Color4& color) {
  if (_lightCount)
    setUniform(_diffuseColorUniform, color);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::bindDiffuseTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(_flags & Flag::DiffuseTexture,
                 "PhongShadowReceiverShader::bindDiffuseTexture(): "
                 "the shader was not "
                 "created with diffuse texture enabled",
                 *this);
  if (_lightCount)
    texture.bind(DiffuseTextureLayer);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setSpecularColor(
    const Magnum::Color4& color) {
  if (_lightCount)
    setUniform(_specularColorUniform, color);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::bindSpecularTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(_flags & Flag::SpecularTexture,
                 "PhongShadowReceiverShader::bindSpecularTexture(): "
                 "the shader was not "
                 "created with specular texture enabled",
                 *this);
  if (_lightCount)
    texture.bind(SpecularTextureLayer);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::bindNormalTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(_flags & Flag::NormalTexture,
                 "PhongShadowReceiverShader::bindNormalTexture(): the "
                 "shader was not "
                 "created with normal texture enabled",
                 *this);
  if (_lightCount)
    texture.bind(NormalTextureLayer);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::bindTextures(
    Mn::GL::Texture2D* ambient,
    Mn::GL::Texture2D* diffuse,
    Mn::GL::Texture2D* specular,
    Mn::GL::Texture2D* normal) {
  CORRADE_ASSERT(_flags & (Flag::AmbientTexture | Flag::DiffuseTexture |
                           Flag::SpecularTexture | Flag::NormalTexture),
                 "PhongShadowReceiverShader::bindTextures(): the "
                 "shader was not created "
                 "with any textures enabled",
                 *this);
  Mn::GL::AbstractTexture::bind(AmbientTextureLayer,
                                {ambient, diffuse, specular, normal});
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setShininess(
    Mn::Float shininess) {
  if (_lightCount)
    setUniform(_shininessUniform, shininess);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setAlphaMask(
    Mn::Float mask) {
  CORRADE_ASSERT(_flags & Flag::AlphaMask,
                 "PhongShadowReceiverShader::setAlphaMask(): the "
                 "shader was not created "
                 "with alpha mask enabled",
                 *this);
  setUniform(_alphaMaskUniform, mask);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setObjectId(
    Magnum::UnsignedInt id) {
  CORRADE_ASSERT(_flags & Flag::ObjectId,
                 "PhongShadowReceiverShader::setObjectId(): the "
                 "shader was not created "
                 "with object ID enabled",
                 *this);
  setUniform(_objectIdUniform, id);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setTransformationMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(_transformationMatrixUniform, matrix);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setNormalMatrix(
    const Mn::Matrix3x3& matrix) {
  if (_lightCount)
    setUniform(_normalMatrixUniform, matrix);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setProjectionMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(_projectionMatrixUniform, matrix);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setLightPositions(
    const Cr::Containers::ArrayView<const Mn::Vector3> positions) {
  CORRADE_ASSERT(_lightCount == positions.size(),
                 "PhongShadowReceiverShader::setLightPositions(): expected"
                     << _lightCount << "items but got" << positions.size(),
                 *this);
  if (_lightCount)
    setUniform(_lightPositionsUniform, positions);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setLightPosition(
    Magnum::UnsignedInt id,
    const Mn::Vector3& position) {
  CORRADE_ASSERT(id < _lightCount,
                 "PhongShadowReceiverShader::setLightPosition(): light ID"
                     << id << "is out of bounds for" << _lightCount << "lights",
                 *this);
  setUniform(_lightPositionsUniform + id, position);
  return *this;
}

/* It's light, but can't be in the header because MSVC needs to know the size
   of Mn::Vector3 for the initializer list use */
PhongShadowReceiverShader& PhongShadowReceiverShader::setLightPositions(
    std::initializer_list<Mn::Vector3> lights) {
  return setLightPositions({lights.begin(), lights.size()});
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setLightColors(
    const Cr::Containers::ArrayView<const Magnum::Color4> colors) {
  CORRADE_ASSERT(_lightCount == colors.size(),
                 "PhongShadowReceiverShader::setLightColors(): expected"
                     << _lightCount << "items but got" << colors.size(),
                 *this);
  if (_lightCount)
    setUniform(_lightColorsUniform, colors);
  return *this;
}

/* It's light, but can't be in the header because MSVC needs to know the size
   of Color for the initializer list use */
PhongShadowReceiverShader& PhongShadowReceiverShader::setLightColors(
    std::initializer_list<Magnum::Color4> colors) {
  return setLightColors({colors.begin(), colors.size()});
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setLightColor(
    Magnum::UnsignedInt id,
    const Magnum::Color4& color) {
  CORRADE_ASSERT(id < _lightCount,
                 "PhongShadowReceiverShader::setLightColor(): light ID"
                     << id << "is out of bounds for" << _lightCount << "lights",
                 *this);
  setUniform(_lightColorsUniform + id, color);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setModelMatrix(
    const Magnum::Matrix4& matrix) {
  setUniform(modelMatrixUniform_, matrix);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setShadowmapMatrices(
    Corrade::Containers::ArrayView<const Magnum::Matrix4> matrices) {
  setUniform(shadowMapMatrixUniform_, matrices);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setShadowLightDirection(
    const Magnum::Vector3& vector) {
  setUniform(shadowLightDirectionUniform_, vector);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setShadowmapTexture(
    Magnum::GL::Texture2DArray& texture) {
  texture.bind(ShadowmapTextureLayer);
  return *this;
}

PhongShadowReceiverShader& PhongShadowReceiverShader::setShadowBias(
    const Magnum::Float bias) {
  if (_lightCount)
    setUniform(shadowBiasUniform_, bias);
  return *this;
}

PhongShadowReceiverShader&
PhongShadowReceiverShader::setShadeFacesFacingAwayFromLight(bool shadeFaces) {
  if (_lightCount)
    setUniform(shadeFacesFacingAwayFromLight_, shadeFaces);
  return *this;
}

}  // namespace gfx
}  // namespace esp
