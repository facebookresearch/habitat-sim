// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "SensorInfoVisualizer.h"

#include <Corrade/Utility/Assert.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector4.h>
#include <Magnum/Shaders/Generic.h>

#include "DepthVisualizerShader.h"

namespace esp {
namespace gfx {

namespace Mn = Magnum;
namespace Cr = Corrade;

SensorInfoVisualizer::SensorInfoVisualizer() {
  // prepare a big triangle mesh to cover the screen
  mesh_ = Mn::GL::Mesh{};
  mesh_.setCount(3);
}
void SensorInfoVisualizer::resetFramebufferRenderbuffer(
    Mn::Vector2i framebufferSize) {
  if (frameBuffer_.viewport() != Mn::Range2Di{{}, {framebufferSize}}) {
    frameBuffer_ = Mn::GL::Framebuffer{{{}, framebufferSize}};
    colorBuffer_.setStorage(Mn::GL::RenderbufferFormat::RGBA8, framebufferSize);
    frameBuffer_.attachRenderbuffer(Mn::GL::Framebuffer::ColorAttachment{0},
                                    colorBuffer_);
    // depth buffer is 24-bit integer pixel
    depthBuffer_.setStorage(Mn::GL::RenderbufferFormat::DepthComponent24,
                            framebufferSize);
    frameBuffer_.attachRenderbuffer(
        Mn::GL::Framebuffer::BufferAttachment::Depth, depthBuffer_);
  }
}

void SensorInfoVisualizer::prepareToDraw(Magnum::Vector2i framebufferSize) {
  resetFramebufferRenderbuffer(framebufferSize);

  frameBuffer_.bind();
  frameBuffer_.mapForDraw({
      {Mn::Shaders::Generic3D::ColorOutput,
       Mn::GL::Framebuffer::ColorAttachment{0}},
  });
  frameBuffer_.clearDepth(1.0f).clearColor(0,                // color attachment
                                           Mn::Vector4ui{0}  // clear color
  );

  CORRADE_INTERNAL_ASSERT(
      frameBuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
      Mn::GL::Framebuffer::Status::Complete);
}

void SensorInfoVisualizer::blitRgbaToDefault() {
  frameBuffer_.mapForRead(Mn::GL::Framebuffer::ColorAttachment{0});
  CORRADE_ASSERT(
      frameBuffer_.viewport() == Mn::GL::defaultFramebuffer.viewport(),
      "SensorInfoVisualizer::blitRgbaToDefault(): a mismatch on viewport"
      "between current framebuffer and the default framebuffer.", );

  Mn::GL::AbstractFramebuffer::blit(
      frameBuffer_, Mn::GL::defaultFramebuffer, frameBuffer_.viewport(),
      Mn::GL::defaultFramebuffer.viewport(), Mn::GL::FramebufferBlit::Color,
      Mn::GL::FramebufferBlitFilter::Nearest);
}

void SensorInfoVisualizer::draw(Mn::ResourceKey shaderKey) {
  Magnum::Resource<Magnum::GL::AbstractShaderProgram> shader =
      shaderManager_.get<Magnum::GL::AbstractShaderProgram>(shaderKey);

  CORRADE_ASSERT(shader,
                 "SensorInfoVisualizer::draw(): Shader does not exist.", );
  shader->draw(mesh_);
}

Mn::ResourceKey SensorInfoVisualizer::getShaderKey(SensorInfoType type) {
  switch (type) {
    case SensorInfoType::Depth:
      return "Depth";
      break;
  }
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

}  // namespace gfx
}  // namespace esp
