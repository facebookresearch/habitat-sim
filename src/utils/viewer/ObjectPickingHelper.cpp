#include "ObjectPickingHelper.h"
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Assert.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/Generic.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

ObjectPickingHelper::ObjectPickingHelper(Mn::Vector2i viewportSize) {
  recreateFramebuffer(viewportSize).mapForDraw();
  CORRADE_INTERNAL_ASSERT(
      selectionFramebuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
      Mn::GL::Framebuffer::Status::Complete);
}

ObjectPickingHelper& ObjectPickingHelper::recreateFramebuffer(
    Mn::Vector2i viewportSize) {
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
  return *this;
}

ObjectPickingHelper& ObjectPickingHelper::prepareToDraw() {
  selectionFramebuffer_.bind();
  mapForDraw();
  selectionFramebuffer_.clearDepth(1.0f).clearColor(1, Mn::Vector4ui{0xffff});
  CORRADE_INTERNAL_ASSERT(
      selectionFramebuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
      Mn::GL::Framebuffer::Status::Complete);
  return *this;
}

ObjectPickingHelper& ObjectPickingHelper::mapForDraw() {
  selectionFramebuffer_.mapForDraw({{Mn::Shaders::Generic3D::ColorOutput,
                                     Mn::GL::Framebuffer::DrawAttachment::None},
                                    {Mn::Shaders::Generic3D::ObjectIdOutput,
                                     Mn::GL::Framebuffer::ColorAttachment{1}}});
  return *this;
}

ObjectPickingHelper& ObjectPickingHelper::setViewport(
    Mn::Vector2i viewportSize) {
  recreateFramebuffer(viewportSize);
  selectionFramebuffer_.setViewport({{}, viewportSize});
  return *this;
}

unsigned int ObjectPickingHelper::getObjectId(
    Magnum::Vector2i mouseEventPosition,
    Magnum::Vector2i windowSize) {
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

  const Mn::UnsignedInt selectedObject =
      selectionFramebuffer_.read(area, {Mn::PixelFormat::R32UI})
          .pixels<Mn::UnsignedInt>()[0][0];

  return selectedObject;
}
