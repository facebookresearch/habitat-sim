// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Resource.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/Magnum.h>

#include "TextRenderer.h"
#include "esp/core/esp.h"

// This is to import the "resources" at runtime. When the resource is compiled
// into static library, it must be explicitly initialized via this macro, and
// should be called *outside* of any namespace.
static void importFontResources() {
  CORRADE_RESOURCE_INITIALIZE(FontResources)
}

namespace esp {
namespace gfx {
TextRenderer::TextRenderer(float aspectRatio,
                           float fontSize,
                           int cacheSize,
                           const std::string& characters)
    : cache_(Mn::Vector2i{cacheSize}) {
  if (!Cr::Utility::Resource::hasGroup("fonts")) {
    importFontResources();
  }
  CORRADE_ASSERT(
      fontSize > 0,
      "TextRenderer::TextRenderer: font size" << fontSize << "is illegal.", );
  CORRADE_ASSERT(
      cacheSize > 0,
      "TextRenderer::TextRenderer: cache size" << cacheSize << "is illegal.", );

  font_ = fontManager_.loadAndInstantiate("StbTrueTypeFont");
  CORRADE_ASSERT(
      font_,
      "TextRenderer::TextRenderer: cannot load and instantiate the font", );

  Cr::Utility::Resource rs("fonts");
  std::string fontName = "SourceSansPro-SemiBold.ttf";
  if (!font_->openData(rs.getRaw(fontName), fontSize)) {
    Cr::Utility::Fatal{-1} << "TextRenderer::TextRenderer: The font" << fontName
                           << "does not exist.";
  }

  font_->fillGlyphCache(cache_, characters);

  updateAspectRatio(aspectRatio);
}

size_t TextRenderer::createRenderer(float onScreenCharacterSize,
                                    size_t glyphCount,
                                    Mn::Text::Alignment alignment) {
  CORRADE_ASSERT(onScreenCharacterSize > 0,
                 "TextRenderer::createRenderer: the character size"
                     << onScreenCharacterSize << "is illegal.",
                 0);

  CORRADE_ASSERT(glyphCount > 0,
                 "TextRenderer::createRenderer: the capacity" << glyphCount
                                                              << "is illegal.",
                 0);

  text_.emplace_back(
      std::move(Cr::Containers::Pointer<Mn::Text::Renderer2D>()));
  size_t rendererId = text_.size() - 1;
  text_[rendererId].reset(new Mn::Text::Renderer2D(
      *font_, cache_, onScreenCharacterSize, alignment));
  text_[rendererId]->reserve(glyphCount, Mn::GL::BufferUsage::DynamicDraw,
                             Mn::GL::BufferUsage::StaticDraw);

  return rendererId;
}

void TextRenderer::updateText(const std::string& text, size_t rendererId) {
  CORRADE_ASSERT(rendererId < text_.size(),
                 "TextRenderer::updateText: the rendererId"
                     << rendererId << "is out of range.", );
  text_[rendererId]->render(text);
}

TextRenderer& TextRenderer::draw(const Mn::Color3& color, size_t rendererId) {
  CORRADE_ASSERT(
      rendererId < text_.size(),
      "TextRenderer::draw: the rendererId" << rendererId << "is out of range.",
      *this);
  textShader_
      .setTransformationProjectionMatrix(
          textProjection_ *
          Mn::Matrix3::translation(
              1.0f / textProjection_.rotationScaling().diagonal()))
      .setColor(color)
      .bindVectorTexture(cache_.texture());
  text_[rendererId]->mesh().draw(textShader_);
  return *this;
}

TextRenderer& TextRenderer::updateAspectRatio(float viewportAspectRatio) {
  textProjection_ =
      Mn::Matrix3::scaling(Mn::Vector2::yScale(viewportAspectRatio));

  return *this;
}

}  // namespace gfx
}  // namespace esp
