#ifndef GL_TENSOR_UTILS_H_
#define GL_TENSOR_UTILS_H_

#include <glad/glad.h>
#include <stdlib.h>
#include <chrono>

typedef std::chrono::high_resolution_clock ChronoClock;
#define CLOCK_INTERVAL(x)                                                      \
  ((double)(std::chrono::duration_cast<std::chrono::nanoseconds>(x).count()) / \
   1000000000L)

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

extern void EglErrorChecker(const char* file, const char* func, int line);

#define EGL_CHECK_ERROR                                       \
  do {                                                        \
    EglErrorChecker(__FILE__, __PRETTY_FUNCTION__, __LINE__); \
  } while (0)

const size_t kShaderLogLength = 2048;
struct ShaderInfo {
  GLenum type_;
  const char* source_;
  GLuint name_;
  char log_[kShaderLogLength];
};

GLuint loadShaders(ShaderInfo* shaders, size_t length);
void releaseShaders(GLuint shader_program, const ShaderInfo* shaders,
                    size_t length);

int savePng(char* filename, int width, int height, void* buffer,
            char* title = NULL);

#ifdef __cplusplus
};
#endif  // __cplusplus

#endif