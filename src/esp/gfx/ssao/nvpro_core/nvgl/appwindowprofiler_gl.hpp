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

#ifndef NV_WINDOWPROFILER_GL_INCLUDED
#define NV_WINDOWPROFILER_GL_INCLUDED

#include "../nvh/appwindowprofiler.hpp"
#include "contextwindow_gl.hpp"
#include "profiler_gl.hpp"

//////////////////////////////////////////////////////////////////////////
/**
  \class nvgl::AppWindowProfilerGL

  nvgl::AppWindowProfilerGL derives from nvh::AppWindowProfiler
  and overrides the context and swapbuffer functions.

  To influence the context creation modify
  `m_contextInfo` prior running AppWindowProfiler::run,
  which triggers window, and context creation etc.

  The class comes with a nvgl::ProfilerGL instance that references the
  AppWindowProfiler::m_profiler's data.
*/

namespace nvgl {

#define NV_PROFILE_GL_SECTION(name) \
  nvgl::ProfilerGL::Section _tempTimer(m_profilerGL, name)
#define NV_PROFILE_GL_SPLIT() m_profilerGL.accumulationSplit()

class AppWindowProfilerGL : public nvh::AppWindowProfiler {
 public:
  AppWindowProfilerGL(bool singleThreaded = true)
      : nvh::AppWindowProfiler(singleThreaded), m_profilerGL(&m_profiler) {
    m_contextInfo.robust = false;
    m_contextInfo.core = false;
#ifdef NDEBUG
    m_contextInfo.debug = false;
#else
    m_contextInfo.debug = true;
#endif
    m_contextInfo.share = NULL;
    m_contextInfo.major = 4;
    m_contextInfo.minor = 5;
  }

  nvgl::ContextWindowCreateInfo m_contextInfo;
  ContextWindow m_contextWindow;

  nvgl::ProfilerGL m_profilerGL;

  int run(const std::string& name,
          int argc,
          const char** argv,
          int width,
          int height) {
    return AppWindowProfiler::run(name, argc, argv, width, height, true);
  }

  virtual void contextInit() override;
  virtual void contextDeinit() override;

  virtual void swapResize(int width, int height) override {
    m_windowState.m_swapSize[0] = width;
    m_windowState.m_swapSize[1] = height;
  }
  virtual void swapPrepare() override {}
  virtual void swapBuffers() override { m_contextWindow.swapBuffers(); }
  virtual void swapVsync(bool state) override {
    m_contextWindow.swapInterval(state ? 1 : 0);
  }
  virtual const char* contextGetDeviceName() override {
    return m_contextWindow.m_deviceName.c_str();
  }
};
}  // namespace nvgl

#endif
