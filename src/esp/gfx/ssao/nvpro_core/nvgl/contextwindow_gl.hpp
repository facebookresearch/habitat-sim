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
#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <stdint.h>
#include <string>

namespace nvgl {

/**
  # struct nvgl::ContextWindowCreateInfo

  Set up the context properties for a OpenGL ContextWindow.
  e.g. version, core/compatibiltiy etc.
*/

struct ContextWindowCreateInfo {
  int major;
  int minor;
  int device;

  int MSAA;
  int depth;
  int stencil;
  bool debug;
  bool robust;
  bool core;
  bool forward;
  bool stereo;
  class ContextWindow* share;

  ContextWindowCreateInfo(int _major = 4,
                          int _minor = 3,
                          bool _core = false,
                          int _MSAA = 0,
                          int _depth = 24,
                          int _stencil = 8,
                          bool _debug = false,
                          bool _robust = false,
                          bool _forward = false,
                          bool _stereo = false,
                          class ContextWindow* _share = 0) {
    major = _major;
    minor = _minor;
    MSAA = _MSAA;
    depth = _depth;
    stencil = _stencil;
    core = _core;
    debug = _debug;
    robust = _robust;
    forward = _forward;
    stereo = _stereo;
    share = _share;
    device = 0;
  }
};

/**
  \class nvgl::ContextWindow

  nvgl::ContextWindow sets up an OpenGL context from a provided `GLFWwindow`.
  Makes use of `glDebugMessageCallback` to hook up an error callback
  and loads all extensions provided by `extensions_gl.hpp`
*/

class ContextWindow {
 public:
  struct ContextWindowInternalGL* m_internal = nullptr;

  uint32_t m_debugFilter = 0;
  std::string m_debugTitle;
  std::string m_deviceName;

  ContextWindow(ContextWindow const&) = delete;
  ContextWindow& operator=(ContextWindow const&) = delete;

  ContextWindow();

  bool init(const ContextWindowCreateInfo* cflags,
            GLFWwindow* window,
            const char* dbgTitle = "test");
  void deinit();

  void swapInterval(int i);
  void swapBuffers();

  int extensionSupported(const char* name);
  void* getProcAddress(const char* name);

  void makeContextCurrent();
  void makeContextNonCurrent();

  void screenshot(const char* filename,
                  int x,
                  int y,
                  int width,
                  int height,
                  unsigned char* data);

  // TODO: check if this is really necessary : local method getProcAddressGL
  // could be enough
  static void* sysGetProcAddress(const char* name);
};

}  // namespace nvgl
