// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include "magnum.h"

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>

using namespace Magnum;

namespace esp {
namespace gfx {

struct Renderer::Impl {
  Impl(int width, int height)
      : framebufferSize_(width, height),
        colorBuffer_(),
        objectIdBuffer_(),
        depthRenderbuffer_(),
        framebuffer_({{}, framebufferSize_}) {
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    setSize(width, height);
  }
  ~Impl() { LOG(INFO) << "Deconstructing Renderer"; }

  void setSize(int width, int height) {
    framebufferSize_[0] = width;
    framebufferSize_[1] = height;
    colorBuffer_.setStorage(GL::RenderbufferFormat::SRGB8Alpha8,
                            framebufferSize_);
    objectIdBuffer_.setStorage(GL::RenderbufferFormat::R32UI, framebufferSize_);
    depthRenderbuffer_.setStorage(GL::RenderbufferFormat::DepthComponent32F,
                                  framebufferSize_);
    framebuffer_ = GL::Framebuffer{{{}, framebufferSize_}};
    framebuffer_
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer_)
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1},
                            objectIdBuffer_)
        .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth,
                            depthRenderbuffer_)
        .mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}},
                     {1, GL::Framebuffer::ColorAttachment{1}}});
    CORRADE_INTERNAL_ASSERT(
        framebuffer_.checkStatus(GL::FramebufferTarget::Draw) ==
        GL::Framebuffer::Status::Complete);
  }

  inline void renderEnter() {
    framebuffer_.clearDepth(1.0);
    framebuffer_.clearColor(0, Color4{});
    framebuffer_.clearColor(1, Vector4ui{});
    framebuffer_.bind();
  }

  inline void renderExit() {}

  void draw(RenderCamera& camera, MagnumDrawableGroup& drawables) {
    renderEnter();
    camera.getMagnumCamera().setViewport(framebufferSize_);

    /* Inverted projection matrix to unproject the depth value and chop the
       near plane off. We don't care about X/Y there and the corresponding
       parts of the matrix are zero as well so take just the lower-right part
       of it (denoted a, b, c, d).

        x 0 0 0
        0 y 0 0
        0 0 a b
        0 0 c d

       Doing an inverse of just the bottom right block is enough as well -- see
       https://en.wikipedia.org/wiki/Block_matrix#Block_diagonal_matrices for
       a proof.

       Taking a 2-component vector with the first component being Z and second
       1, the final calculation of unprojected Z is then

        | a b |   | z |   | az + b |
        | c d | * | 1 | = | cz + d |

    */
    const Matrix4 projection = camera.getMagnumCamera().projectionMatrix();
    depthUnprojection_ = Matrix2x2{Math::swizzle<'z', 'w'>(projection[2]),
                                   Math::swizzle<'z', 'w'>(projection[3])}
                             .inverted();

    /* The Z value comes in range [0; 1], but we need it in the range [-1; 1].
       Instead of doing z = x*2 - 1 for every pixel, we add that to this
       matrix:

        az + b
        a(x*2 - 1) + b
        2ax - a + b
        (2a)x + (b - a)

      and similarly for c/d. Which means -- from the second component we
      subtract the first, and the first we multiply by 2. */
    depthUnprojection_[1] -= depthUnprojection_[0];
    depthUnprojection_[0] *= 2.0;

    /* Finally, because the output has Z going forward, not backward, we need
       to negate it. There's a perspective division happening, so we have to
       negate just the first row. */
    depthUnprojection_.setRow(0, -depthUnprojection_.row(0));

    camera.draw(drawables);
    renderExit();
  }

  void draw(sensor::Sensor& visualSensor, scene::SceneGraph& sceneGraph) {
    ASSERT(visualSensor.isVisualSensor());

    // set the modelview matrix, projection matrix of the render camera;
    sceneGraph.setDefaultRenderCamera(visualSensor);

    draw(sceneGraph.getDefaultRenderCamera(), sceneGraph.getDrawables());
  }

  void readFrameRgba(uint8_t* ptr) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{0});
    Image2D rgbaImage =
        framebuffer_.read(Range2Di::fromSize({0, 0}, framebufferSize_),
                          {PixelFormat::RGBA8Unorm});
    std::memcpy(ptr, rgbaImage.data(), rgbaImage.data().size());
  }

  void readFrameDepth(float* ptr) {
    Image2D depthImage = framebuffer_.read(
        Range2Di::fromSize({0, 0}, framebufferSize_),
        {GL::PixelFormat::DepthComponent, GL::PixelType::Float});

    /* Unproject the Z */
    Containers::ArrayView<const Float> data =
        Containers::arrayCast<const Float>(depthImage.data());
    for (std::size_t i = 0; i != data.size(); ++i) {
      const Float z = data[i];

      /* If a fragment has a depth of 1, it's due to a hole in the mesh. The
         consumers expect 0 for things that are too far, so be nice to them.
         We can afford using == for comparison as 1.0f has an exact
         representation and the depth is cleared to exactly this value. */
      if (z == 1.0f) {
        ptr[i] = 0.0f;
        continue;
      }

      /* The following is

          (az + b) / (cz + d)

         See the comment in draw() above for details. */
      ptr[i] =
          Math::fma(depthUnprojection_[0][0], z, depthUnprojection_[1][0]) /
          Math::fma(depthUnprojection_[0][1], z, depthUnprojection_[1][1]);
    }
  }

  void readFrameObjectId(uint32_t* ptr) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{1});
    Image2D objectImage = framebuffer_.read(
        Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::R32UI});
    std::memcpy(ptr, objectImage.data(), objectImage.data().size());
  }

  Magnum::Vector2i framebufferSize_;
  GL::Renderbuffer colorBuffer_;
  GL::Renderbuffer objectIdBuffer_;
  GL::Renderbuffer depthRenderbuffer_;
  GL::Framebuffer framebuffer_;

  Matrix2x2 depthUnprojection_;
};

Renderer::Renderer(int width, int height)
    : pimpl_(spimpl::make_unique_impl<Impl>(width, height)) {}

void Renderer::draw(RenderCamera& camera, scene::SceneGraph& sceneGraph) {
  pimpl_->draw(camera, sceneGraph.getDrawables());
}

void Renderer::draw(sensor::Sensor& visualSensor,
                    scene::SceneGraph& sceneGraph) {
  pimpl_->draw(visualSensor, sceneGraph);
}

void Renderer::setSize(int width, int height) {
  pimpl_->setSize(width, height);
}

void Renderer::readFrameRgba(uint8_t* ptr) {
  pimpl_->readFrameRgba(ptr);
}

void Renderer::readFrameDepth(float* ptr) {
  pimpl_->readFrameDepth(ptr);
}

void Renderer::readFrameObjectId(uint32_t* ptr) {
  pimpl_->readFrameObjectId(ptr);
}

vec3i Renderer::getSize() {
  return vec3i(pimpl_->framebufferSize_[0], pimpl_->framebufferSize_[1], 4);
}

}  // namespace gfx
}  // namespace esp
