#include <gl_tensor_param.h>
#include <glad/glad.h>
#include <headless_gl_context.h>
#include <math.h>
#include <pybind11/pybind11.h>
#include <test_utils.h>
#include <torch/torch.h>

namespace py = pybind11;
using namespace gltensortest;

static const char *vShaderSource = R"(
#version 400 core

layout( location = 0 ) in vec3 vPosition;
layout (location = 1) in vec3 vColor;

out vec3 outColor;

void main()
{
    gl_Position = vec4(vPosition, 1.0f);
    outColor = vColor;
}
)";

static const char *fShaderSource = R"(
#version 450 core

in vec3 outColor;
out vec4 fColor;

void main()
{
    fColor = vec4(outColor, 1.0);
}
)";

struct GLTensorTest {
  static const GLuint kNumVertices = 3;
  const int kWidth = 1024;
  const int kHeight = 1024;

  GLContextPtr gl_context_;

  ShaderInfo shaders_[2] = {
      (ShaderInfo){
          .type_ = GL_VERTEX_SHADER, .source_ = vShaderSource, .name_ = 0},
      (ShaderInfo){
          .type_ = GL_FRAGMENT_SHADER, .source_ = fShaderSource, .name_ = 0},
  };
  GLuint shader_program_ = 0;

  GLfloat vertices_[kNumVertices][6] = {{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
                                        {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                                        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}};

  GLuint vao_ = 0;
  GLuint vbo_ = 0;

  GLuint frame_buffer_ = 0;
  GLuint image_ = 0;

  float degree_ = 0.0f;

  GLTensorTest() {
    gl_context_ = HeadlessGLContext::Create();

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glCreateBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_), vertices_, GL_STREAM_DRAW);

    shader_program_ = loadShaders(shaders_, 2);
    glUseProgram(shader_program_);
    EGL_CHECK_ERROR;

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);

    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glGenFramebuffers(1, &frame_buffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);

    // // The texture we're going to render to
    glGenTextures(1, &image_);

    // "Bind" the newly created texture : all future texture functions
    // will modify this texture
    glBindTexture(GL_TEXTURE_2D, image_);

    // Give an empty image to OpenGL ( the last "0" )
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kWidth, kHeight, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, 0);

    // Poor filtering. Needed !
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Set "renderedTexture" as our colour attachement #0
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           image_, 0);

    // Set the list of draw buffers.
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);  // "1" is the size of DrawBuffers
    glDisable(GL_DEPTH_TEST);

    glViewport(0, 0, kWidth, kHeight);

    // Always check that our framebuffer is ok
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      throw std::runtime_error("Renderer: GL_FRAMEBUFFER failure");
    }
  }

  ~GLTensorTest() {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);

    // detach shaders from program and release
    releaseShaders(shader_program_, shaders_, 2);

    glDeleteFramebuffers(1, &frame_buffer_);
    glDeleteTextures(1, &image_);
    // glDeleteRenderbuffers(1, &image_);
    EGL_CHECK_ERROR;
  }

  void* GetGLTensorParam() const {
    GLTensorParam *param = new GLTensorParam;
    param->device_id_ = gl_context_->GetDeviceId();
    param->image_ = image_;
    param->target_ = GL_TEXTURE_2D;
    param->width_ = kWidth;
    param->height_ = kHeight;
    param->format_ = GL_RGBA;
    return param;
  }

  void Frame(void* data) {
    float rad = degree_ * M_PI / 180.0f;
    float color = (float)sin(rad) / 2.0f + 0.5f;

    float bg_color[] = {color, color, color, 1.0f};
    glClearBufferfv(GL_COLOR, 0, bg_color);

    static float rad120 = 120.0f * M_PI / 180.0f;
    for (int i = 0; i < 3; ++i) {
      vertices_[i][0] = (float)cos(rad + i * rad120);
      vertices_[i][1] = (float)sin(rad + i * rad120);
    }
    degree_ += 1.0f;

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_), vertices_, GL_STREAM_DRAW);

    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, kNumVertices);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (data) {
      glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_UNSIGNED_BYTE, data);
    }

    EGL_CHECK_ERROR;
  }
};

PYBIND11_MODULE(gl_tensor_test, m) {
  m.doc() = R"pbdoc(
        Graphic-Cuda PyTorch Tensor plugin test
        -----------------------
        .. currentmodule:: gl_tensor_test
        .. autosummary::
           :toctree: _generate
    )pbdoc";

  py::class_<GLTensorTest>(m, "GLTensorTest")
      .def(py::init<>())
      .def("GetGLTensorParam", &GLTensorTest::GetGLTensorParam,
           py::return_value_policy::take_ownership)
      .def("Frame", &GLTensorTest::Frame);

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}