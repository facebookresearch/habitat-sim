// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include "magnum.h"

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
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
  Impl() {
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
  }
  ~Impl() { LOG(INFO) << "Deconstructing Renderer"; }

  void draw(RenderCamera& camera, MagnumDrawableGroup& drawables) {
    camera.draw(drawables);
  }

  void draw(sensor::Sensor& visualSensor, scene::SceneGraph& sceneGraph) {
    ASSERT(visualSensor.isVisualSensor());

    // set the modelview matrix, projection matrix of the render camera;
    sceneGraph.setDefaultRenderCamera(visualSensor);
    sceneGraph.getDefaultRenderCamera().getMagnumCamera().setViewport(
        visualSensor.resolution());

    draw(sceneGraph.getDefaultRenderCamera(), sceneGraph.getDrawables());
  }
};

Renderer::Renderer() : pimpl_(spimpl::make_unique_impl<Impl>()) {}

void Renderer::draw(RenderCamera& camera, scene::SceneGraph& sceneGraph) {
  pimpl_->draw(camera, sceneGraph.getDrawables());
}

void Renderer::draw(sensor::Sensor& visualSensor,
                    scene::SceneGraph& sceneGraph) {
  pimpl_->draw(visualSensor, sceneGraph);
}

}  // namespace gfx
}  // namespace esp
