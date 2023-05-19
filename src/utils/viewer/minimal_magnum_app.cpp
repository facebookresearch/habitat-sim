// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Platform/GLContext.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <cstdio>
#include <memory>

#include <GLFW/glfw3.h>

/* Setup the colored triangle */
using namespace Magnum::Math::Literals;

class MyApplication : public Magnum::Platform::Application {
 public:
  explicit MyApplication(const Arguments& arguments)
      : Magnum::Platform::Application{
            arguments,
            Configuration{}.setTitle("MyApplication").setSize({400, 300})} {
    // _ctx = std::make_unique<Magnum::Platform::GLContext>();

    // struct TriangleVertex {
    //   Magnum::Vector2 position;
    //   Magnum::Color3 color;
    // };
    // const TriangleVertex vertices[]{
    //     {{-0.5f, -0.5f}, 0xff0000_rgbf}, /* Left vertex, red color */
    //     {{0.5f, -0.5f}, 0x00ff00_rgbf},  /* Right vertex, green color */
    //     {{0.0f, 0.5f}, 0x0000ff_rgbf}    /* Top vertex, blue color */
    // };

    // _mesh.setCount(Corrade::Containers::arraySize(vertices))
    //     .addVertexBuffer(Magnum::GL::Buffer{vertices}, 0,
    //                      Magnum::Shaders::VertexColorGL2D::Position{},
    //                      Magnum::Shaders::VertexColorGL2D::Color3{});
  }

  void drawEvent() override {
    /* Render here */
    static float animFraction = 0.f;
    animFraction = fmodf(animFraction + 0.05, 1.0);

    Magnum::GL::Renderer::setClearColor(Magnum::Color4(animFraction, 0, 0, 1));
    Magnum::GL::defaultFramebuffer.clear(Magnum::GL::FramebufferClear::Color);
    // _shader.draw(_mesh);

    swapBuffers();
    redraw();
  }

 private:
  std::unique_ptr<Magnum::Platform::GLContext> _ctx;
  Magnum::GL::Mesh _mesh;
  Magnum::Shaders::VertexColorGL2D _shader;
};

MAGNUM_APPLICATION_MAIN(MyApplication)
