/*
 * Copyright (c) 2013-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2013-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
//--------------------------------------------------------------------

#include "../nvgl/contextwindow_gl.hpp"
#include "../nvgl/error_gl.hpp"
#include "../nvgl/extensions_gl.hpp"

#ifdef WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

#include <GL/wgl.h>

#include <windows.h>
#include <windowsx.h>
#include "resources.h"
#elif defined LINUX
#define GLFW_EXPOSE_NATIVE_GLX
#define GLFW_EXPOSE_NATIVE_X11
#include <GL/glx.h>
#include <GLFW/glfw3native.h>
#endif

//#include <fileformats/bmp.hpp"
#include "../nvh/nvprint.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace nvgl {

//------------------------------------------------------------------------------
// OGL callback
//------------------------------------------------------------------------------
#ifdef _DEBUG
static void APIENTRY myOpenGLCallback(GLenum source,
                                      GLenum type,
                                      GLuint id,
                                      GLenum severity,
                                      GLsizei length,
                                      const GLchar* message,
                                      const GLvoid* userParam) {
  ContextWindow* window = (ContextWindow*)userParam;

  GLenum filter = window->m_debugFilter;
  GLenum severitycmp = severity;
  // minor fixup for filtering so notification becomes lowest priority
  if (GL_DEBUG_SEVERITY_NOTIFICATION == filter) {
    filter = GL_DEBUG_SEVERITY_LOW_ARB + 1;
  }
  if (GL_DEBUG_SEVERITY_NOTIFICATION == severitycmp) {
    severitycmp = GL_DEBUG_SEVERITY_LOW_ARB + 1;
  }

  if (!filter || severitycmp <= filter) {
    // static std::map<GLuint, bool> ignoreMap;
    // if(ignoreMap[id] == true)
    //     return;
    const char* strSource = "0";
    const char* strType = strSource;
    switch (source) {
      case GL_DEBUG_SOURCE_API_ARB:
        strSource = "API";
        break;
      case GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB:
        strSource = "WINDOWS";
        break;
      case GL_DEBUG_SOURCE_SHADER_COMPILER_ARB:
        strSource = "SHADER COMP.";
        break;
      case GL_DEBUG_SOURCE_THIRD_PARTY_ARB:
        strSource = "3RD PARTY";
        break;
      case GL_DEBUG_SOURCE_APPLICATION_ARB:
        strSource = "APP";
        break;
      case GL_DEBUG_SOURCE_OTHER_ARB:
        strSource = "OTHER";
        break;
    }
    switch (type) {
      case GL_DEBUG_TYPE_ERROR_ARB:
        strType = "ERROR";
        break;
      case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB:
        strType = "Deprecated";
        break;
      case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB:
        strType = "Undefined";
        break;
      case GL_DEBUG_TYPE_PORTABILITY_ARB:
        strType = "Portability";
        break;
      case GL_DEBUG_TYPE_PERFORMANCE_ARB:
        strType = "Performance";
        break;
      case GL_DEBUG_TYPE_OTHER_ARB:
        strType = "Other";
        break;
    }
    switch (severity) {
      case GL_DEBUG_SEVERITY_HIGH_ARB:
        LOGE("ARB_debug : %s High - %s - %s : %s\n",
             window->m_debugTitle.c_str(), strSource, strType, message);
        break;
      case GL_DEBUG_SEVERITY_MEDIUM_ARB:
        LOGW("ARB_debug : %s Medium - %s - %s : %s\n",
             window->m_debugTitle.c_str(), strSource, strType, message);
        break;
      case GL_DEBUG_SEVERITY_LOW_ARB:
        LOGI("ARB_debug : %s Low - %s - %s : %s\n",
             window->m_debugTitle.c_str(), strSource, strType, message);
        break;
      default:
        // LOGI("ARB_debug : comment - %s - %s : %s\n", strSource, strType,
        // message);
        break;
    }
  }
}
#endif

// from GLFW 3.0
static int stringInExtensionString(const char* string, const char* exts) {
  const GLubyte* extensions = (const GLubyte*)exts;
  const GLubyte* start;
  GLubyte* where;
  GLubyte* terminator;

  // It takes a bit of care to be fool-proof about parsing the
  // OpenGL extensions string. Don't be fooled by sub-strings,
  // etc.
  start = extensions;
  for (;;) {
    where = (GLubyte*)strstr((const char*)start, string);
    if (!where)
      return GL_FALSE;

    terminator = where + strlen(string);
    if (where == start || *(where - 1) == ' ') {
      if (*terminator == ' ' || *terminator == '\0')
        break;
    }

    start = terminator;
  }

  return GL_TRUE;
}

#ifdef WIN32

//////////////////////////////////////////////////////////////////////////
// WIN 32

static HMODULE s_module = NULL;

struct ContextWindowInternalGL {
  HDC m_hDCaffinity = NULL;
  HDC m_hDC = NULL;
  HGLRC m_hRC = NULL;

  PFNWGLSWAPINTERVALEXTPROC m_wglSwapIntervalEXT = NULL;
  PFNWGLGETEXTENSIONSSTRINGARBPROC m_wglGetExtensionsStringARB = NULL;
  PFNWGLDELETEDCNVPROC m_wglDeleteDCNV = NULL;

  bool init(const ContextWindowCreateInfo& settings,
            GLFWwindow* sourcewindow,
            ContextWindow* ctxwindow) {
    GLuint PixelFormat;
    HWND hWnd = glfwGetWin32Window(sourcewindow);
    HINSTANCE hInstance = GetModuleHandle(NULL);

    PIXELFORMATDESCRIPTOR pfd;
    memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));

    pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = settings.depth;
    pfd.cStencilBits = settings.stencil;

    if (settings.stereo) {
      pfd.dwFlags |= PFD_STEREO;
    }

    if (settings.MSAA > 1) {
      HWND hWndDummy =
          CreateWindowEx(NULL, "Static", "Dummy", WS_OVERLAPPEDWINDOW, 0, 0, 10,
                         10, NULL, NULL, hInstance, NULL);

      m_hDC = GetDC(hWndDummy);
      PixelFormat = ChoosePixelFormat(m_hDC, &pfd);
      SetPixelFormat(m_hDC, PixelFormat, &pfd);
      m_hRC = wglCreateContext(m_hDC);
      wglMakeCurrent(m_hDC, m_hRC);

      PFNWGLCHOOSEPIXELFORMATARBPROC fn_wglChoosePixelFormatARB =
          (PFNWGLCHOOSEPIXELFORMATARBPROC)wglGetProcAddress(
              "wglChoosePixelFormatARB");

      ReleaseDC(hWndDummy, m_hDC);
      m_hDC = GetDC(hWnd);
      int attri[] = {WGL_DRAW_TO_WINDOW_ARB,
                     true,
                     WGL_PIXEL_TYPE_ARB,
                     WGL_TYPE_RGBA_ARB,
                     WGL_SUPPORT_OPENGL_ARB,
                     true,
                     WGL_ACCELERATION_ARB,
                     WGL_FULL_ACCELERATION_ARB,
                     WGL_DOUBLE_BUFFER_ARB,
                     true,
                     WGL_DEPTH_BITS_ARB,
                     settings.depth,
                     WGL_STENCIL_BITS_ARB,
                     settings.stencil,
                     WGL_SAMPLE_BUFFERS_ARB,
                     1,
                     WGL_SAMPLES_ARB,
                     settings.MSAA,
                     0,
                     0};
      GLuint nfmts;
      int fmt;

      if (!fn_wglChoosePixelFormatARB(m_hDC, attri, NULL, 1, &fmt, &nfmts)) {
        wglDeleteContext(m_hRC);
        return false;
      }
      wglDeleteContext(m_hRC);
      DestroyWindow(hWndDummy);
      if (!SetPixelFormat(m_hDC, fmt, &pfd))
        return false;
    } else {
      m_hDC = GetDC(hWnd);
      PixelFormat = ChoosePixelFormat(m_hDC, &pfd);
      SetPixelFormat(m_hDC, PixelFormat, &pfd);
    }
    m_hRC = wglCreateContext(m_hDC);
    wglMakeCurrent(m_hDC, m_hRC);

    HDC hdcContext = m_hDC;

    if (settings.device) {
      PFNWGLENUMGPUSNVPROC fn_wglEnumGpusNV =
          (PFNWGLENUMGPUSNVPROC)wglGetProcAddress("wglEnumGpusNV");
      PFNWGLENUMGPUDEVICESNVPROC fn_wglEnumGpuDevicesNV =
          (PFNWGLENUMGPUDEVICESNVPROC)wglGetProcAddress("wglEnumGpuDevicesNV");
      PFNWGLCREATEAFFINITYDCNVPROC fn_wglCreateAffinityDCNV =
          (PFNWGLCREATEAFFINITYDCNVPROC)wglGetProcAddress(
              "wglCreateAffinityDCNV");
      m_wglDeleteDCNV =
          (PFNWGLDELETEDCNVPROC)wglGetProcAddress("wglDeleteDCNV");

      if (fn_wglEnumGpusNV && fn_wglCreateAffinityDCNV && m_wglDeleteDCNV) {
        const uint32_t MAX_GPU = 4;
        HGPUNV hGPU[MAX_GPU];
        _GPU_DEVICE devices[MAX_GPU];
        HGPUNV GpuMask[2] = {0};
        uint32_t gpuIndex = 0;
        // Get a list of the first MAX_GPU GPUs in the system
        while ((gpuIndex < MAX_GPU) &&
               fn_wglEnumGpusNV(gpuIndex, &hGPU[gpuIndex])) {
          fn_wglEnumGpuDevicesNV(hGPU[gpuIndex], 0, &devices[gpuIndex]);
          gpuIndex++;
        }

        GpuMask[0] = hGPU[settings.device];

        // delete old
        wglMakeCurrent(NULL, NULL);
        wglDeleteContext(m_hRC);

        m_hDCaffinity = fn_wglCreateAffinityDCNV(GpuMask);
        hdcContext = m_hDCaffinity;

        PixelFormat = ChoosePixelFormat(m_hDCaffinity, &pfd);
        SetPixelFormat(m_hDCaffinity, PixelFormat, &pfd);
        m_hRC = wglCreateContext(m_hDCaffinity);

        wglMakeCurrent(m_hDC, m_hRC);
      }
    }

    PFNWGLCREATECONTEXTATTRIBSARBPROC fn_wglCreateContextAttribsARB =
        (PFNWGLCREATECONTEXTATTRIBSARBPROC)wglGetProcAddress(
            "wglCreateContextAttribsARB");
    if (fn_wglCreateContextAttribsARB) {
      HGLRC hRC = NULL;
      std::vector<int> attribList;
#define ADDATTRIB(a, b)      \
  {                          \
    attribList.push_back(a); \
    attribList.push_back(b); \
  }
      int maj = settings.major;
      int min = settings.minor;
      ADDATTRIB(WGL_CONTEXT_MAJOR_VERSION_ARB, maj)
      ADDATTRIB(WGL_CONTEXT_MINOR_VERSION_ARB, min)
      if (settings.core)
        ADDATTRIB(WGL_CONTEXT_PROFILE_MASK_ARB,
                  WGL_CONTEXT_CORE_PROFILE_BIT_ARB)
      else
        ADDATTRIB(WGL_CONTEXT_PROFILE_MASK_ARB,
                  WGL_CONTEXT_COMPATIBILITY_PROFILE_BIT_ARB)
      int ctxtflags = 0;
      if (settings.debug)
        ctxtflags |= WGL_CONTEXT_DEBUG_BIT_ARB;
      if (settings.robust)
        ctxtflags |= WGL_CONTEXT_ROBUST_ACCESS_BIT_ARB;
      if (settings.forward)  // use it if you want errors when compat options
                             // still used
        ctxtflags |= WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB;
      ADDATTRIB(WGL_CONTEXT_FLAGS_ARB, ctxtflags);
      ADDATTRIB(0, 0)
      int* p = &(attribList[0]);
      if (!(hRC = fn_wglCreateContextAttribsARB(hdcContext, 0, p))) {
        GLint MajorVersionContext = 0;
        GLint MinorVersionContext = 0;
        glGetIntegerv(GL_MAJOR_VERSION, &MajorVersionContext);
        glGetIntegerv(GL_MINOR_VERSION, &MinorVersionContext);
        if ((MajorVersionContext * 100 + MinorVersionContext * 10) <
            (maj * 100 + min * 10)) {
          LOGE("OpenGL version %d.%d not available. Only %d.%d found\n", maj,
               min, MajorVersionContext, MinorVersionContext);
        }
        LOGE("wglCreateContextAttribsARB() failed for OpenGL context.\n");
        return false;
      }
      if (!wglMakeCurrent(m_hDC, hRC)) {
        LOGE("wglMakeCurrent() failed for OpenGL context.\n");
      } else {
        wglDeleteContext(m_hRC);
        m_hRC = hRC;
#ifdef _DEBUG

        PFNGLDEBUGMESSAGECALLBACKARBPROC fn_glDebugMessageCallbackARB =
            (PFNGLDEBUGMESSAGECALLBACKARBPROC)wglGetProcAddress(
                "glDebugMessageCallbackARB");
        PFNGLDEBUGMESSAGECONTROLARBPROC fn_glDebugMessageControlARB =
            (PFNGLDEBUGMESSAGECONTROLARBPROC)wglGetProcAddress(
                "glDebugMessageControlARB");

        if (fn_glDebugMessageCallbackARB) {
          glEnable(GL_DEBUG_OUTPUT);
          glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB);
          fn_glDebugMessageControlARB(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE,
                                      0, NULL, GL_TRUE);
          fn_glDebugMessageCallbackARB(myOpenGLCallback, ctxwindow);
        }
#endif
      }
    }
    m_wglSwapIntervalEXT =
        (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
    m_wglGetExtensionsStringARB =
        (PFNWGLGETEXTENSIONSSTRINGARBPROC)wglGetProcAddress(
            "wglGetExtensionsStringARB");

    if (!s_module) {
      s_module = LoadLibraryA("opengl32.dll");
    }

    return true;
  }

  void deinit() {
    wglDeleteContext(m_hRC);
    if (m_hDCaffinity) {
      m_wglDeleteDCNV(m_hDCaffinity);
    }
  }
};

void ContextWindow::makeContextCurrent() {
  wglMakeCurrent(m_internal->m_hDC, m_internal->m_hRC);
}

void ContextWindow::makeContextNonCurrent() {
  wglMakeCurrent(0, 0);
}

int ContextWindow::extensionSupported(const char* name) {
  // we are not using the glew query, as glew will only report
  // those extension it knows about, not what the actual driver may support

  int i;
  GLint count;

  // Check if extension is in the modern OpenGL extensions string list
  // This should be safe to use since GL 3.0 is around for a long time :)

  glGetIntegerv(GL_NUM_EXTENSIONS, &count);

  for (i = 0; i < count; i++) {
    const char* en = (const char*)glGetStringi(GL_EXTENSIONS, i);
    if (!en) {
      return GL_FALSE;
    }

    if (strcmp(en, name) == 0)
      return GL_TRUE;
  }

  // Check platform specifc gets
  const char* exts = NULL;

  if (m_internal->m_wglGetExtensionsStringARB) {
    exts = m_internal->m_wglGetExtensionsStringARB(m_internal->m_hDC);
  }
  if (!exts) {
    return FALSE;
  }

  return stringInExtensionString(name, exts);
}

void* ContextWindow::sysGetProcAddress(const char* name) {
  void* p = (void*)wglGetProcAddress(name);
  if (p == 0 || (p == (void*)0x1) || (p == (void*)0x2) || (p == (void*)0x3) ||
      (p == (void*)-1)) {
    p = (void*)GetProcAddress(s_module, name);
  }

  return p;
}

void ContextWindow::swapInterval(int i) {
  m_internal->m_wglSwapIntervalEXT(i);
}

void ContextWindow::swapBuffers() {
  SwapBuffers(m_internal->m_hDC);
}
#else

struct ContextWindowInternalGL {
  GLFWwindow* m_glfwwindow = nullptr;

  bool init(const ContextWindowCreateInfo& settings,
            GLFWwindow* sourcewindow,
            ContextWindow* ctxwindow) {
    m_glfwwindow = sourcewindow;
    glfwMakeContextCurrent(m_glfwwindow);

#ifdef _DEBUG
    PFNGLDEBUGMESSAGECALLBACKARBPROC fn_glDebugMessageCallbackARB =
        (PFNGLDEBUGMESSAGECALLBACKARBPROC)glfwGetProcAddress(
            "glDebugMessageCallbackARB");
    PFNGLDEBUGMESSAGECONTROLARBPROC fn_glDebugMessageControlARB =
        (PFNGLDEBUGMESSAGECONTROLARBPROC)glfwGetProcAddress(
            "glDebugMessageControlARB");

    if (fn_glDebugMessageCallbackARB) {
      glEnable(GL_DEBUG_OUTPUT);
      glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB);
      fn_glDebugMessageControlARB(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0,
                                  NULL, GL_TRUE);
      fn_glDebugMessageCallbackARB(myOpenGLCallback, ctxwindow);
    }
#endif

    return true;
  }

  void deinit() {}
};

void ContextWindow::makeContextCurrent() {
  glfwMakeContextCurrent(m_internal->m_glfwwindow);
}

void ContextWindow::makeContextNonCurrent() {
  glfwMakeContextCurrent(NULL);
}

int ContextWindow::extensionSupported(const char* name) {
  // we are not using the glew query, as glew will only report
  // those extension it knows about, not what the actual driver may support

  int i;
  GLint count;

  // Check if extension is in the modern OpenGL extensions string list
  // This should be safe to use since GL 3.0 is around for a long time :)

  glGetIntegerv(GL_NUM_EXTENSIONS, &count);

  for (i = 0; i < count; i++) {
    const char* en = (const char*)glGetStringi(GL_EXTENSIONS, i);
    if (!en) {
      return GL_FALSE;
    }

    if (strcmp(en, name) == 0)
      return GL_TRUE;
  }

  // todo: port
  assert(false);

  return false;

  /*
  // Check platform specifc gets
  const char* exts = glXQueryExtensionsString(glfwGetX11Display(), 0);

  if(!exts)
  {
    return GL_FALSE;
  }

  return stringInExtensionString(name, exts);
  */
}

void* ContextWindow::sysGetProcAddress(const char* name) {
  void* p = (void*)glfwGetProcAddress(name);

  return p;
}

void ContextWindow::swapInterval(int i) {
  glfwSwapInterval(i);
}

void ContextWindow::swapBuffers() {
  glfwSwapBuffers(m_internal->m_glfwwindow);
}

#endif
//////////////////////////////////////////////////////////////////////////

ContextWindow::ContextWindow() {
  if (m_internal) {
    delete m_internal;
  }
}

//------------------------------------------------------------------------------
bool ContextWindow::init(const ContextWindowCreateInfo* cflags,
                         GLFWwindow* sourcewindow,
                         const char* dbgTitle) {
  if (!m_internal) {
    m_internal = new ContextWindowInternalGL;
  }

  ContextWindowCreateInfo settings;
  if (cflags) {
    settings = *cflags;
  }

  m_debugFilter = GL_DEBUG_SEVERITY_HIGH_ARB;
  m_debugTitle = dbgTitle;

  if (m_internal->init(settings, sourcewindow, this)) {
    load_GL(ContextWindow::sysGetProcAddress);

    const char* renderer = (const char*)glGetString(GL_RENDERER);

    GLint major = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    GLint minor = 0;
    glGetIntegerv(GL_MINOR_VERSION, &minor);

    if (major < settings.major ||
        (major == settings.major && minor < settings.minor)) {
      LOGE(" OpenGL context version too low: required %d.%d, found %d.%d\n",
           settings.major, settings.minor, major, minor);
      return false;
    }

    int version = major * 10 + minor;
    int success = 1;

#define HAS_VERSION(MA, MI)      \
  if (version >= (MA * 10 + MI)) \
    success = success && has_GL_VERSION_##MA##_##MI;
    HAS_VERSION(1, 1);
    HAS_VERSION(1, 2);
    HAS_VERSION(1, 3);
    HAS_VERSION(1, 4);
    HAS_VERSION(1, 5);
    HAS_VERSION(2, 0);
    HAS_VERSION(2, 1);
    HAS_VERSION(3, 0);
    HAS_VERSION(3, 1);
    HAS_VERSION(3, 2);
    HAS_VERSION(3, 3);
    HAS_VERSION(4, 0);
    HAS_VERSION(4, 1);
    HAS_VERSION(4, 2);
    HAS_VERSION(4, 3);
    HAS_VERSION(4, 4);
    HAS_VERSION(4, 5);
    HAS_VERSION(4, 6);
#undef HAS_VERSION

    if (!success) {
      LOGE(" OpenGL context version incomplete symbols\n");
      return false;
    }

    m_deviceName = std::string(renderer);

    return true;
  }

  return false;
}

void* ContextWindow::getProcAddress(const char* name) {
  return sysGetProcAddress(name);
}

void ContextWindow::screenshot(const char* filename,
                               int x,
                               int y,
                               int width,
                               int height,
                               unsigned char* data) {
  glFinish();
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glReadPixels(x, y, width, height, GL_BGRA, GL_UNSIGNED_BYTE, data);

  assert(false);  // todo: port
  // if(filename)
  // {
  //   saveBMP(filename, width, height, data);
  // }
}

void ContextWindow::deinit() {
  makeContextNonCurrent();
  m_internal->deinit();
}
}  // namespace nvgl
