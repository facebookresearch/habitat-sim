/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
        2020, 2021, 2022 — Vladimír Vondruš <mosra@centrum.cz>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Platform/GLContext.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <GLFW/glfw3.h>

using namespace Magnum;

int main(int argc, char** argv) {
  /* Initialize the library */
  if (!glfwInit())
    return -1;

  auto myErrorFun = [](int errorCode, const char* msg) {
    printf("error: %s\n", msg);
  };

  glfwSetErrorCallback(myErrorFun);

  glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);

  /* Create a windowed mode window and its OpenGL context */
  GLFWwindow* const window = glfwCreateWindow(
      800, 600, "Magnum Plain GLFW Triangle Example", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  /* Make the window's context current */
  glfwMakeContextCurrent(window);

  {
    /* Create Magnum context in an isolated scope */
    Platform::GLContext ctx{argc, argv};

    /* Setup the colored triangle */
    using namespace Math::Literals;

    struct TriangleVertex {
      Vector2 position;
      Color3 color;
    };
    const TriangleVertex vertices[]{
        {{-0.5f, -0.5f}, 0xff0000_rgbf}, /* Left vertex, red color */
        {{0.5f, -0.5f}, 0x00ff00_rgbf},  /* Right vertex, green color */
        {{0.0f, 0.5f}, 0x0000ff_rgbf}    /* Top vertex, blue color */
    };

    GL::Mesh mesh;
    mesh.setCount(Containers::arraySize(vertices))
        .addVertexBuffer(GL::Buffer{vertices}, 0,
                         Shaders::VertexColorGL2D::Position{},
                         Shaders::VertexColorGL2D::Color3{});

    Shaders::VertexColorGL2D shader;

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window)) {
      /* Render here */
      GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);
      shader.draw(mesh);

      /* Swap front and back buffers */
      glfwSwapBuffers(window);

      /* Poll for and process events */
      glfwPollEvents();
    }
  }

  glfwTerminate();
}
