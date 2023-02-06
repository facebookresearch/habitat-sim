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

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#ifndef NV_BASEGL_H_INCLUDED
#define NV_BASEGL_H_INCLUDED

#include <cstdio>
#include "extensions_gl.hpp"

#define NV_BUFFER_OFFSET(i) ((char*)NULL + (i))

namespace nvgl {
/**
    # struct nvgl::BufferBinding
    Wraps buffer, offset, size, gpu address
  */

struct BufferBinding {
  GLuint buffer = 0;
  GLintptr offset = 0;
  GLsizeiptr size = 0;
  GLuint64 bufferADDR;

  BufferBinding() {}
  BufferBinding(GLuint inBuffer,
                GLintptr inOffset,
                GLsizeiptr inSize,
                GLuint64 inBufferADDR) {
    buffer = inBuffer;
    size = inSize;
    offset = inOffset;
    bufferADDR = inBufferADDR + inOffset;
  }
};

/**
    # struct nvgl::TextureBuffer
    A `GL_TEXTURE_BUFFER` that references the provided buffer.
    Wraps texture and bindless texture handle.
  */

struct TextureBuffer {
  GLuint tex = 0;
  GLuint64 texADDR = 0;

  void create(GLuint buffer, GLintptr offset, GLsizeiptr sz, GLenum format) {
    glCreateTextures(GL_TEXTURE_BUFFER, 1, &tex);
    glTextureBufferRange(tex, format, buffer, offset, sz);
    if (has_GL_ARB_bindless_texture) {
      texADDR = glGetTextureHandleARB(tex);
      glMakeTextureHandleResidentARB(texADDR);
    }
  }

  void destroy() {
    if (texADDR) {
      glMakeTextureHandleNonResidentARB(texADDR);
    }
    glDeleteTextures(1, &tex);
  }
};

/**
    # struct nvgl::Buffer
    Wraps buffer as well as optionally creates a `GL_TEXTURE_BUFFER` if
    a non-null `format` is provided. If bindless is available it will
    also create bindless handles for all resources and make them resident.

    If the `flags` contain `GL_MAP_PERSISTENT_BIT` it will also map
    the buffer and keep the host pointer.
  */

struct Buffer {
  GLuint buffer = 0;
  GLuint tex = 0;
  GLuint64 bufferADDR = 0;
  GLuint64 texADDR = 0;
  GLsizeiptr size = 0;
  void* mapped = nullptr;

  void create(GLsizeiptr sz, const void* data, GLuint flags, GLenum format) {
    size = sz;
    glCreateBuffers(1, &buffer);
    glNamedBufferStorage(buffer, sz, data, flags);
    if (has_GL_NV_shader_buffer_load) {
      glGetNamedBufferParameterui64vNV(buffer, GL_BUFFER_GPU_ADDRESS_NV,
                                       &bufferADDR);
      glMakeNamedBufferResidentNV(buffer, GL_READ_WRITE);
    }
    if (format) {
      glCreateTextures(GL_TEXTURE_BUFFER, 1, &tex);
      glTextureBuffer(tex, format, buffer);
      if (has_GL_ARB_bindless_texture) {
        texADDR = glGetTextureHandleARB(tex);
        glMakeTextureHandleResidentARB(texADDR);
      }
    }

    if (flags & GL_MAP_PERSISTENT_BIT) {
      mapped = glMapNamedBufferRange(
          buffer, 0, sz,
          flags & (GL_MAP_READ_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT |
                   GL_MAP_COHERENT_BIT));
    }
  }
  void destroy() {
    if (mapped) {
      glUnmapNamedBuffer(buffer);
    }
    if (tex) {
      if (texADDR) {
        glMakeTextureHandleNonResidentARB(texADDR);
      }
      glDeleteTextures(1, &tex);
    }
    if (bufferADDR) {
      glMakeNamedBufferNonResidentNV(buffer);
    }
    glDeleteBuffers(1, &buffer);
  }

  operator GLuint() const { return buffer; }
};

inline size_t uboAligned(size_t size) {
  return ((size + 255) / 256) * 256;
}

inline void bindMultiTexture(GLenum target, GLenum textarget, GLuint tex) {
  glActiveTexture(target);
  glBindTexture(textarget, tex);
}

/**
    # nvgl resource functions

    Functions that wrap glCreate/glDelete and operate on `GLuint& obj`.
    The "new" functions delete the existing object if non-null and create a new
   one. The "delete" functions delete non-null objects.

    * newBuffer / deleteBuffer
    * newTextureView
    * newTexture / deleteTexture
    * newFramebuffer / deleteFramebuffer
    * newSampler / deleteSampler
    * newQuery / deleteQuery
    * newVertexArray / deleteVertexArray

    \code{.cpp}
    // typical use-case
    FrameBuffer::resize(int with, int height){
      newFramebuffer(m_fbo);
      newTexture(m_color, GL_TEXTURE_2D);
      newTexture(m_depthStencil, GL_TEXTURE_2D);
      glTextureStorage2D(m_color, ...)
      glTextureStorage2D(m_depthStencil, ...)
      glNamedFramebufferTexture(m_fbo, GL_COLOR_ATTACHMENT0,        m_color, 0);
      glNamedFramebufferTexture(m_fbo, GL_DEPTH_STENCIL_ATTACHMENT,
   m_depthStencil, 0);
    }
    \endcode
  */

inline void newBuffer(GLuint& glid) {
  if (glid)
    glDeleteBuffers(1, &glid);
  glCreateBuffers(1, &glid);
}

inline void deleteBuffer(GLuint& glid) {
  if (glid)
    glDeleteBuffers(1, &glid);
  glid = 0;
}

inline void newTextureView(GLuint& glid) {
  if (glid)
    glDeleteTextures(1, &glid);
  glGenTextures(1, &glid);
}

inline void newTexture(GLuint& glid, GLenum target) {
  if (glid)
    glDeleteTextures(1, &glid);
  glCreateTextures(target, 1, &glid);
}

inline void deleteTexture(GLuint& glid) {
  if (glid)
    glDeleteTextures(1, &glid);
  glid = 0;
}

inline void newFramebuffer(GLuint& glid) {
  if (glid)
    glDeleteFramebuffers(1, &glid);
  glCreateFramebuffers(1, &glid);
}

inline void deleteFramebuffer(GLuint& glid) {
  if (glid)
    glDeleteFramebuffers(1, &glid);
  glid = 0;
}

inline void newSampler(GLuint& glid) {
  if (glid)
    glDeleteSamplers(1, &glid);
  glCreateSamplers(1, &glid);
}

inline void deleteSampler(GLuint& glid) {
  if (glid)
    glDeleteSamplers(1, &glid);
  glid = 0;
}

inline void newQuery(GLuint& glid, GLenum target) {
  if (glid)
    glDeleteQueries(1, &glid);
  glCreateQueries(target, 1, &glid);
}

inline void deleteQuery(GLuint& glid) {
  if (glid)
    glDeleteQueries(1, &glid);
  glid = 0;
}

inline void newVertexArray(GLuint& glid) {
  if (glid)
    glCreateVertexArrays(1, &glid);
  glGenVertexArrays(1, &glid);
}

inline void deleteVertexArray(GLuint& glid) {
  if (glid)
    glDeleteVertexArrays(1, &glid);
  glid = 0;
}
}  // namespace nvgl

#endif
