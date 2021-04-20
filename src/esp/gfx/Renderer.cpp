// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include <Corrade/Containers/StridedArrayView.h>
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
#ifdef CORRADE_TARGET_APPLE
#include <Magnum/ImageView.h>
#endif
#include <Magnum/PixelFormat.h>
#include <Magnum/ResourceManager.h>

#include "esp/core/Check.h"
#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/TextureVisualizerShader.h"
#include "esp/gfx/magnum.h"
#include "esp/sensor/VisualSensor.h"
#include "esp/sim/Simulator.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

struct Renderer::Impl {
  explicit Impl(Flags flags)
      : depthShader_{nullptr}, flags_{flags}, mesh_{Cr::Containers::NullOpt} {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  }
  ~Impl() { LOG(INFO) << "Deconstructing Renderer"; }

  void draw(RenderCamera& camera,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags) {
    for (auto& it : sceneGraph.getDrawableGroups()) {
      // TODO: remove || true
      if (it.second.prepareForDraw(camera) || true) {
        camera.draw(it.second, flags);
      }
    }
  }

  void draw(sensor::VisualSensor& visualSensor, sim::Simulator& sim) {
    if (visualSensor.specification()->sensorType ==
        sensor::SensorType::Semantic) {
      ESP_CHECK(sim.semanticSceneExists(),
                "Renderer::Impl::draw(): SemanticSensor observation requested "
                "but no SemanticScene is loaded");
    }
    visualSensor.drawObservation(sim);
  }

  void visualize(sensor::VisualSensor& visualSensor,
                 float colorMapOffset,
                 float colorMapScale) {
    sensor::SensorType& type = visualSensor.specification()->sensorType;
    if (type == sensor::SensorType::Depth ||
        type == sensor::SensorType::Semantic) {
      Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
      gfx::RenderTarget& tgt = visualSensor.renderTarget();
      if (!mesh_) {
        // prepare a big triangle mesh to cover the screen
        mesh_ = Mn::GL::Mesh{};
        mesh_->setCount(3);
      }
      if (type == sensor::SensorType::Depth) {
        Magnum::Resource<Mn::GL::AbstractShaderProgram, TextureVisualizerShader>
            shader = getShader<TextureVisualizerShader>(
                esp::gfx::Renderer::Impl::RendererShaderType::
                    DepthTextureVisualizer);

#ifdef CORRADE_TARGET_APPLE
        Mn::Image2D image = tgt.getDepthTexture().image(
            0, {Mn::GL::PixelFormat::DepthComponent, Mn::GL::PixelType::Float});
        Mn::ImageView2D imgView{image.storage(), Mn::PixelFormat::R32F,
                                image.size(), image.data()};

        if ((visualizedTex_ == Cr::Containers::NullOpt) ||
            (visualizedTex_ != Cr::Containers::NullOpt &&
             visualizedTex_->imageSize(0) != image.size())) {
          visualizedTex_ = Mn::GL::Texture2D{};
          (*visualizedTex_)
              .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
              .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
              .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
              .setStorage(1, Mn::GL::TextureFormat::R32F, image.size());
        }
        (*visualizedTex_).setSubImage(0, {}, imgView);
        shader->bindDepthTexture(*visualizedTex_);
#else
        shader->bindDepthTexture(tgt.getDepthTexture());
#endif
        shader->setDepthUnprojection(*visualSensor.depthUnprojection());
        shader->setColorMapTransformation(colorMapOffset, colorMapScale);
        tgt.renderReEnter();
        shader->draw(*mesh_);
        tgt.renderExit();
      }

      // TODO object id
      Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
    }
  }

  void bindRenderTarget(sensor::VisualSensor& sensor, Flags bindingFlags) {
    auto depthUnprojection = sensor.depthUnprojection();
    CORRADE_ASSERT(depthUnprojection,
                   "Renderer::Impl::bindRenderTarget(): Sensor does not have a "
                   "depthUnprojection matrix", );

    if (!depthShader_) {
      depthShader_ = std::make_unique<DepthShader>(
          DepthShader::Flag::UnprojectExistingDepth);
    }

    RenderTarget::Flags renderTargetFlags = {};
    switch (sensor.specification()->sensorType) {
      case sensor::SensorType::Color:
        CORRADE_ASSERT(
            !(flags_ & Flag::NoTextures),
            "Renderer::Impl::bindRenderTarget(): Tried to setup a color "
            "render buffer while the simulator was initialized with "
            "requiresTextures = false", );
        renderTargetFlags |= RenderTarget::Flag::RgbaBuffer;
        break;

      case sensor::SensorType::Depth:
        if (bindingFlags & Flag::VisualizeTexture) {
          renderTargetFlags |= RenderTarget::Flag::RgbaBuffer;
        }
        renderTargetFlags |= RenderTarget::Flag::DepthTexture;
        break;

      case sensor::SensorType::Semantic:
        if (bindingFlags & Flag::VisualizeTexture) {
          renderTargetFlags |= RenderTarget::Flag::RgbaBuffer;
        }
        renderTargetFlags |= RenderTarget::Flag::ObjectIdBuffer;
        break;

      default:
        // I need this default, since sensor type list is long, and without
        // default clang-tidy will complain
        break;
    }

    sensor.bindRenderTarget(RenderTarget::create_unique(
        sensor.framebufferSize(), *depthUnprojection, depthShader_.get(),
        renderTargetFlags, &sensor));
  }

 private:
  // TODO: shall we use shader resource manager from now?
  std::unique_ptr<DepthShader> depthShader_;
  const Flags flags_;
  Cr::Containers::Optional<Mn::GL::Mesh> mesh_;
  Mn::ResourceManager<Mn::GL::AbstractShaderProgram> shaderManager_;
#ifdef CORRADE_TARGET_APPLE
  Cr::Containers::Optional<Mn::GL::Texture2D> visualizedTex_ =
      Cr::Containers::NullOpt;
#endif

  enum class RendererShaderType : uint8_t {
    DepthShader = 0,
    DepthTextureVisualizer = 1,
    // ObjectIdTextureVisualizer = 2,
  };
  template <typename T>
  Mn::Resource<Mn::GL::AbstractShaderProgram, T> getShader(
      RendererShaderType type) {
    Mn::ResourceKey key;
    switch (type) {
      case RendererShaderType::DepthShader:
        key = Mn::ResourceKey{"depthShader"};
        break;

      case RendererShaderType::DepthTextureVisualizer:
        key = Mn::ResourceKey{"depthVisualizer"};
        break;

      // TODO: object id
      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
    Mn::Resource<Mn::GL::AbstractShaderProgram, T> shader =
        shaderManager_.get<Mn::GL::AbstractShaderProgram, T>(key);

    if (!shader) {
      if (type == RendererShaderType::DepthShader) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(),
            new DepthShader{DepthShader::Flag::UnprojectExistingDepth},
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      } else if (type == RendererShaderType::DepthTextureVisualizer) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(),
            new TextureVisualizerShader{
                {TextureVisualizerShader::Flag::DepthTexture}},
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      }
    }
    CORRADE_INTERNAL_ASSERT(shader);

    return shader;
  }
};  // namespace gfx

Renderer::Renderer(Flags flags)
    : pimpl_(spimpl::make_unique_impl<Impl>(flags)) {}

void Renderer::draw(RenderCamera& camera,
                    scene::SceneGraph& sceneGraph,
                    RenderCamera::Flags flags) {
  pimpl_->draw(camera, sceneGraph, flags);
}

void Renderer::draw(sensor::VisualSensor& visualSensor, sim::Simulator& sim) {
  pimpl_->draw(visualSensor, sim);
}

void Renderer::bindRenderTarget(sensor::VisualSensor& sensor,
                                Flags bindingFlags) {
  pimpl_->bindRenderTarget(sensor, bindingFlags);
}

void Renderer::visualize(sensor::VisualSensor& sensor,
                         float colorMapOffset,
                         float colorMapScale) {
  pimpl_->visualize(sensor, colorMapOffset, colorMapScale);
}

}  // namespace gfx
}  // namespace esp
