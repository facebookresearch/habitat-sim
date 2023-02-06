/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#include "error_gl.hpp"

#include <algorithm>

namespace nvgl {

bool checkNamedFramebuffer(GLuint fbo) {
  GLenum status = glCheckNamedFramebufferStatus(fbo, GL_FRAMEBUFFER);
  switch (status) {
    case GL_FRAMEBUFFER_UNDEFINED:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_UNDEFINED");
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
      LOGE("OpenGL Error(%s)\n",
           "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER");
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER");
      break;
    case GL_FRAMEBUFFER_UNSUPPORTED:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_UNSUPPORTED");
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE");
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
      LOGE("OpenGL Error(%s)\n", "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS");
      break;
  }

  return status != GL_FRAMEBUFFER_COMPLETE;
}

bool checkGLVersion(GLint MajorVersionRequire, GLint MinorVersionRequire) {
  GLint MajorVersionContext = 0;
  GLint MinorVersionContext = 0;
  glGetIntegerv(GL_MAJOR_VERSION, &MajorVersionContext);
  glGetIntegerv(GL_MINOR_VERSION, &MinorVersionContext);
  return (MajorVersionContext * 100 + MinorVersionContext * 10) >=
         (MajorVersionRequire * 100 + MinorVersionRequire * 10);
}

bool checkExtension(char const* String) {
  GLint ExtensionCount = 0;
  glGetIntegerv(GL_NUM_EXTENSIONS, &ExtensionCount);
  for (GLint i = 0; i < ExtensionCount; ++i)
    if (std::string((char const*)glGetStringi(GL_EXTENSIONS, i)) ==
        std::string(String))
      return true;
  return false;
}

bool checkError(const char* Title) {
  int Error;
  if ((Error = glGetError()) != GL_NO_ERROR) {
    std::string ErrorString;
    switch (Error) {
      case GL_INVALID_ENUM:
        ErrorString = "GL_INVALID_ENUM";
        break;
      case GL_INVALID_VALUE:
        ErrorString = "GL_INVALID_VALUE";
        break;
      case GL_INVALID_OPERATION:
        ErrorString = "GL_INVALID_OPERATION";
        break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:
        ErrorString = "GL_INVALID_FRAMEBUFFER_OPERATION";
        break;
      case GL_OUT_OF_MEMORY:
        ErrorString = "GL_OUT_OF_MEMORY";
        break;
      default:
        ErrorString = "UNKNOWN";
        break;
    }
    LOGE("OpenGL Error(%s): %s\n", ErrorString.c_str(), Title);
  }
  return Error == GL_NO_ERROR;
}

bool checkBindings(int bindingBits, const char* marker, int num) {
  bool bound = false;
  GLint obj = 0;

#define GLERRCHECKBOUND(name, obj)     \
  glGetIntegerv(name, (obj));          \
  if (*(obj)) {                        \
    bound = true;                      \
    LOGW("OpenGL bound: %s\n", #name); \
  }

#define GLERRCHECKBOUNDFN(fn, what, obj) \
  fn;                                    \
  if (*(obj)) {                          \
    bound = true;                        \
    LOGW("OpenGL bound: %s\n", what);    \
  }

#define GLERRCHECKBOUNDUNIT(i, name, obj)    \
  glGetIntegerv(name, (obj));                \
  if (*(obj)) {                              \
    bound = true;                            \
    LOGW("OpenGL bound: %s %d\n", #name, i); \
  }

#define GLERRCHECKBOUNDINDEXED(name, i, obj) \
  glGetIntegeri_v(name, i, (obj));           \
  if (*(obj)) {                              \
    bound = true;                            \
    LOGW("OpenGL bound: %s %d\n", #name, i); \
  }

  if (bindingBits & CHECKBINDING_VAO_BIT) {
    GLERRCHECKBOUND(GL_VERTEX_ARRAY_BINDING, &obj);
  }
  if (bindingBits & CHECKBINDING_FBO_BIT) {
    GLERRCHECKBOUND(GL_DRAW_FRAMEBUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_READ_FRAMEBUFFER_BINDING, &obj);
  }
  if (bindingBits & CHECKBINDING_PROGRAM_BIT) {
    GLERRCHECKBOUND(GL_CURRENT_PROGRAM, &obj);
    GLERRCHECKBOUND(GL_PROGRAM_PIPELINE_BINDING, &obj);
  }
  if (bindingBits & CHECKBINDING_IMAGES_BIT) {
    GLint units;
    glGetIntegerv(GL_MAX_IMAGE_UNITS, &units);
    for (int i = 0; i < units; i++) {
      GLERRCHECKBOUNDINDEXED(GL_IMAGE_BINDING_NAME, i, &obj);
    }
  }
  if (bindingBits & CHECKBINDING_BUFFERS_BIT) {
    GLERRCHECKBOUND(GL_ARRAY_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_ELEMENT_ARRAY_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_PIXEL_PACK_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_PIXEL_UNPACK_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_UNIFORM_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_TRANSFORM_FEEDBACK_BINDING, &obj);
    GLERRCHECKBOUND(GL_SHADER_STORAGE_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_ATOMIC_COUNTER_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_TEXTURE_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_COPY_READ_BUFFER_BINDING, &obj);
    GLERRCHECKBOUND(GL_COPY_WRITE_BUFFER_BINDING, &obj);

    GLint units;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIB_BINDINGS, &units);
    for (int i = 0; i < units; i++) {
      GLERRCHECKBOUNDINDEXED(GL_VERTEX_BINDING_BUFFER, i, &obj);
    }

    glGetIntegerv(GL_MAX_UNIFORM_BUFFER_BINDINGS, &units);
    for (int i = 0; i < units; i++) {
      GLERRCHECKBOUNDINDEXED(GL_UNIFORM_BUFFER_BINDING, i, &obj);
    }

    glGetIntegerv(GL_MAX_TRANSFORM_FEEDBACK_BUFFERS, &units);
    for (int i = 0; i < units; i++) {
      GLERRCHECKBOUNDINDEXED(GL_TRANSFORM_FEEDBACK_BUFFER_BINDING, i, &obj);
    }

    glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &units);
    for (int i = 0; i < units; i++) {
      GLERRCHECKBOUNDINDEXED(GL_SHADER_STORAGE_BUFFER_BINDING, i, &obj);
    }

    glGetIntegerv(GL_MAX_ATOMIC_COUNTER_BUFFER_BINDINGS, &units);
    for (int i = 0; i < units; i++) {
      GLERRCHECKBOUNDINDEXED(GL_ATOMIC_COUNTER_BUFFER_BINDING, i, &obj);
    }
  }

  if (bindingBits & (CHECKBINDING_TEXTURES_BIT | CHECKBINDING_SAMPLERS_BIT)) {
    GLint currentUnit;
    GLint units;

    glGetIntegerv(GL_ACTIVE_TEXTURE, &currentUnit);
    glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &units);

    for (int i = 0; i < units; i++) {
      glActiveTexture(GL_TEXTURE0 + i);
      if (bindingBits & CHECKBINDING_TEXTURES_BIT) {
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_1D, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_2D, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_3D, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_1D_ARRAY, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_2D_ARRAY, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_CUBE_MAP, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_CUBE_MAP_ARRAY, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_2D_MULTISAMPLE, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_2D_MULTISAMPLE_ARRAY, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_CUBE_MAP, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_CUBE_MAP_ARRAY, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_BUFFER, &obj);
        GLERRCHECKBOUNDUNIT(i, GL_TEXTURE_BINDING_RECTANGLE, &obj);
      }
      if (bindingBits & CHECKBINDING_SAMPLERS_BIT) {
        GLERRCHECKBOUND(GL_SAMPLER_BINDING, &obj);
      }
    }

    glActiveTexture(currentUnit);
  }

  if (bindingBits & CHECKBINDING_XFB_BIT) {
    GLERRCHECKBOUND(GL_TRANSFORM_FEEDBACK_BINDING, &obj);
  }

  if (bindingBits & CHECKBINDING_VATTRIBS_BIT) {
    GLint units;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &units);
    GLint enabled;
    for (int i = 0; i < units; i++) {
      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &enabled);
      if (enabled) {
        bound = true;
        LOGW("OpenGL enabled: vertex %d\n", i);
      }
    }
  }

  if (bound) {
    if (marker) {
      LOGW("%s %d\n\n", marker, num);
    } else {
      LOGW("\n");
    }
  }
#undef GLERRCHECKBOUNDUNIT
#undef GLERRCHECKBOUND
#undef GLERRCHECKBOUNDINDEXED
#undef GLERRCHECKBOUNDFN

  return !bound;
}

bool checkTextureTarget(GLuint texture, GLenum target) {
  GLenum param;
  glGetTextureParameterIiv(texture, GL_TEXTURE_TARGET, (GLint*)&param);
  return param == target;
}

bool checkTextureTarget(GLuint texture,
                        GLenum target,
                        const char* name,
                        const char* marker,
                        int num) {
  if (!checkTextureTarget(texture, target)) {
    if (marker) {
      LOGW("OpenGL unexpected texture target (%s)\n - %s %d\n", name, marker,
           num);
    } else {
      LOGW("OpenGL unexpected texture target (%s)\n", name);
    }
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////

void CheckBufferResidency::addAddress(uint64_t address, uint64_t size) {
  Entry entry;
  entry.address = address;
  entry.size = size;

  m_entries.push_back(entry);

  m_dirty = true;
}

void CheckBufferResidency::removeAddress(uint64_t address) {
  size_t idx = find(address);

  m_entries[idx] = m_entries[m_entries.size() - 1];
  m_entries.pop_back();

  m_dirty = true;
}

bool CheckBufferResidency::checkAddress(uint64_t address) {
  size_t idx = find(address);

  return (idx != ~0);
}

size_t CheckBufferResidency::find(uint64_t address) {
  if (m_entries.empty())
    return ~0;

  if (m_dirty) {
    sort();
  }

  size_t count = m_entries.size();
  size_t begin = 0;

  while (count > 0) {
    size_t idx = begin;
    size_t step = count / 2;
    idx += step;

    if (m_entries[idx].address < address) {
      begin = idx + 1;
      count -= step + 1;
    } else {
      count = step;
    }
  }

  if (begin == m_entries.size() || m_entries[begin].address != address) {
    begin = std::max(begin - 1, size_t(0));
  }

  if (m_entries[begin].address <= address &&
      address < m_entries[begin].address + m_entries[begin].size) {
    return begin;
  } else {
    return ~0;
  }
}

void CheckBufferResidency::sort() {
  std::sort(m_entries.begin(), m_entries.end(), Entry_compare);
  m_dirty = false;
}
}  // namespace nvgl
