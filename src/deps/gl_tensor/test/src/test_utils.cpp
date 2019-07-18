#include <stdio.h>
#include <stdlib.h>

#include <test_utils.h>
#include <glad/glad_egl.h>
#include <png.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void EglErrorChecker(const char* file, const char* func, int line) {
  EGLint err = eglGetError();
  if (err != EGL_SUCCESS) {
    printf("[%s:%d %s] got an EGL error\n", file, line, func);
  }
}

GLuint loadShaders(ShaderInfo* shaders, size_t length) {
  GLint compiled;
  GLsizei len;
  size_t i = 0;
  for (i = 0; i < length; ++i) {
    ShaderInfo& shaderInfo = shaders[i];
    shaderInfo.name_ = glCreateShader(shaderInfo.type_);
    if (!shaderInfo.name_) {
      break;
    }
    glShaderSource(shaderInfo.name_, 1, &shaderInfo.source_, NULL);
    glCompileShader(shaderInfo.name_);
    glGetShaderiv(shaderInfo.name_, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
      glGetShaderInfoLog(shaderInfo.name_, kShaderLogLength, &len,
                         shaderInfo.log_);
      break;
    }
  }

  if (i < length) {
    releaseShaders(0, shaders, length);
    return 0;
  }

  GLuint shader_program = glCreateProgram();
  if (!shader_program) {
    releaseShaders(0, shaders, length);
    return 0;
  }

  i = 0;
  for (i = 0; i < length; ++i) {
    glAttachShader(shader_program, shaders[i].name_);
  }

  glLinkProgram(shader_program);

  EGL_CHECK_ERROR;

  return shader_program;
}

void releaseShaders(GLuint shader_program, const ShaderInfo* shaders,
                    size_t length) {
  for (size_t i = 0; i < length; ++i) {
    const ShaderInfo& shaderInfo = shaders[i];
    if (!shaderInfo.name_) {
      continue;
    }
    if (shader_program) {
      glDetachShader(shader_program, shaderInfo.name_);
    }
    glDeleteShader(shaderInfo.name_);
  }
  if (shader_program) {
    glDeleteProgram(shader_program);
  }
}

int savePng(char* filename, int width, int height, void* buffer, char* title) {
  int code = 0;
  FILE* fp = NULL;
  png_structp png_ptr = NULL;
  png_infop info_ptr = NULL;
  png_bytep* row_pointers = NULL;

  // Open file for writing (binary mode)
  fp = fopen(filename, "wb");
  if (fp == NULL) {
    fprintf(stderr, "Could not open file %s for writing\n", filename);
    code = 1;
    goto finalise;
  }

  // Initialize write structure
  png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (png_ptr == NULL) {
    fprintf(stderr, "Could not allocate write struct\n");
    code = 1;
    goto finalise;
  }

  // Initialize info structure
  info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == NULL) {
    fprintf(stderr, "Could not allocate info struct\n");
    code = 1;
    goto finalise;
  }

  // Setup Exception handling
  if (setjmp(png_jmpbuf(png_ptr))) {
    fprintf(stderr, "Error during png creation\n");
    code = 1;
    goto finalise;
  }

  png_init_io(png_ptr, fp);

  // Write header (8 bit colour depth)
  png_set_IHDR(png_ptr, info_ptr, width, height, 8, PNG_COLOR_TYPE_RGB_ALPHA,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  // Set title
  // if (title != NULL) {
  //   png_text title_text;
  //   title_text.compression = PNG_TEXT_COMPRESSION_NONE;
  //   title_text.key = "Title";
  //   title_text.text = title;
  //   png_set_text(png_ptr, info_ptr, &title_text, 1);
  // }

  png_write_info(png_ptr, info_ptr);

  row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
  for (int i = 0; i < height; ++i) {
    row_pointers[i] = (png_bytep)buffer + i * 4 * width;
  }
  png_write_image(png_ptr, row_pointers);

  // End write
  png_write_end(png_ptr, NULL);

finalise:
  if (fp != NULL) fclose(fp);
  if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
  if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
  if (row_pointers != NULL) free(row_pointers);

  return code;
}

#ifdef __cplusplus
};
#endif  // __cplusplus
