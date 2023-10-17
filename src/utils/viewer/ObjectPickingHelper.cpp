// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectPickingHelper.h"
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Assert.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/GenericGL.h>
#include "esp/gfx/DrawableConfiguration.h"

namespace Cr = Corrade;
namespace Mn = Magnum;
using Mn::Math::Literals::operator""_rgbf;
using Mn::Math::Literals::operator""_rgbaf;

ObjectPickingHelper::ObjectPickingHelper(Mn::Vector2i viewportSize) {
  // create the framebuffer and set the color attachment
  recreateFramebuffer(viewportSize);
  mapForDraw();
  CORRADE_INTERNAL_ASSERT(
      selectionFramebuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
      Mn::GL::Framebuffer::Status::Complete);

  shader_.setViewportSize(Mn::Vector2{viewportSize});
  shader_.setColor(0x2f83cc7f_rgbaf)
      .setWireframeColor(0xdcdcdc_rgbf)
      .setWireframeWidth(2.0);
}

void ObjectPickingHelper::recreateFramebuffer(Mn::Vector2i viewportSize) {
  // setup an offscreen frame buffer for object selection
  selectionDepth_.setStorage(Mn::GL::RenderbufferFormat::DepthComponent24,
                             viewportSize);
  selectionDrawableId_.setStorage(Mn::GL::RenderbufferFormat::R32UI,
                                  viewportSize);
  selectionFramebuffer_ = Mn::GL::Framebuffer{{{}, viewportSize}};
  selectionFramebuffer_
      .attachRenderbuffer(Mn::GL::Framebuffer::BufferAttachment::Depth,
                          selectionDepth_)
      .attachRenderbuffer(Mn::GL::Framebuffer::ColorAttachment{1},
                          selectionDrawableId_);
}

ObjectPickingHelper& ObjectPickingHelper::prepareToDraw() {
  selectionFramebuffer_.bind();
  mapForDraw();
  selectionFramebuffer_.clearDepth(1.0f).clearColor(1, Mn::Vector4ui{0xffff});
  CORRADE_INTERNAL_ASSERT(
      selectionFramebuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
      Mn::GL::Framebuffer::Status::Complete);

  // remove any visualized object that is picked before
  if (meshVisualizerDrawable_) {
    delete meshVisualizerDrawable_;
    meshVisualizerDrawable_ = nullptr;
  }

  return *this;
}

ObjectPickingHelper& ObjectPickingHelper::mapForDraw() {
  selectionFramebuffer_.mapForDraw({{Mn::Shaders::GenericGL3D::ColorOutput,
                                     Mn::GL::Framebuffer::DrawAttachment::None},
                                    {Mn::Shaders::GenericGL3D::ObjectIdOutput,
                                     Mn::GL::Framebuffer::ColorAttachment{1}}});
  return *this;
}

ObjectPickingHelper& ObjectPickingHelper::handleViewportChange(
    Mn::Vector2i viewportSize) {
  recreateFramebuffer(viewportSize);
  selectionFramebuffer_.setViewport({{}, viewportSize});

  shader_.setViewportSize(Mn::Vector2{viewportSize});
  return *this;
}

unsigned int ObjectPickingHelper::getObjectId(
    const Mn::Vector2i& mouseEventPosition,
    const Mn::Vector2i& windowSize) {
  selectionFramebuffer_.mapForRead(Mn::GL::Framebuffer::ColorAttachment{1});
  CORRADE_INTERNAL_ASSERT(
      selectionFramebuffer_.checkStatus(Mn::GL::FramebufferTarget::Read) ==
      Mn::GL::Framebuffer::Status::Complete);

  // First scale the position from being relative to window size to being
  // relative to framebuffer size as those two can be different on HiDPI
  // systems
  const Mn::Vector2i position =
      mouseEventPosition *
      Mn::Vector2{selectionFramebuffer_.viewport().size()} /
      Mn::Vector2{windowSize};

  const Mn::Vector2i fbPosition{
      position.x(),
      selectionFramebuffer_.viewport().sizeY() - position.y() - 1};
  const Mn::Range2Di area = Mn::Range2Di::fromSize(fbPosition, Mn::Vector2i{1});

  const Mn::UnsignedInt pickedObject =
      selectionFramebuffer_.read(area, {Mn::PixelFormat::R32UI})
          .pixels<Mn::UnsignedInt>()[0][0];

  return pickedObject;
}

void ObjectPickingHelper::createPickedObjectVisualizer(
    esp::gfx::Drawable* pickedObject) {
  if (meshVisualizerDrawable_) {
    delete meshVisualizerDrawable_;
    meshVisualizerDrawable_ = nullptr;
  }

  if (!pickedObject) {
    return;
  }

  // default configuration to pass pickedObjectDrawbles_
  esp::gfx::DrawableConfiguration cfg{
      esp::NO_LIGHT_KEY,
      esp::WHITE_MATERIAL_KEY,
      esp::metadata::attributes::ObjectInstanceShaderType::Unspecified,
      &pickedObjectDrawbles_,
      nullptr,
      nullptr,
      nullptr};

  // magnum scene graph will handle the garbage collection even we did not
  // recycle it by the end of the simulation
  meshVisualizerDrawable_ = new esp::gfx::MeshVisualizerDrawable(
      static_cast<esp::scene::SceneNode&>(pickedObject->object()), shader_,
      pickedObject->getVisualizerMesh(), cfg);

  return;
}
