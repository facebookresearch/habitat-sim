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

#ifndef NV_PROJECTBASE_INCLUDED
#define NV_PROJECTBASE_INCLUDED

#include <string.h>  // for memset
#include "../nvp/nvpwindow.hpp"

#include "parametertools.hpp"
#include "profiler.hpp"

namespace nvh {

/**
    \class nvh::AppWindowProfiler
    nvh::AppWindowProfiler provides an alternative utility wrapper class around
   NVPWindow. It is useful to derive single-window applications from and is used
   by some but not all nvpro-samples.

    Further functionality is provided :
    - built-in profiler/timer reporting to console
    - command-line argument parsing as well as config file parsing using the
   ParameterTools see AppWindowProfiler::setupParameters() for built-in commands
    - benchmark/automation mode using ParameterTools
    - screenshot creation
    - logfile based on devicename (depends on context)
    - optional context/swapchain interface
      the derived classes nvvk/appwindowprofiler_vk and
   nvgl/appwindowprofiler_gl make use of this
  */

#define NV_PROFILE_BASE_SECTION(name) \
  nvh::Profiler::Section _tempTimer(m_profiler, name)
#define NV_PROFILE_BASE_SPLIT() m_profiler.accumulationSplit()

class AppWindowProfiler : public NVPWindow {
 public:
  class WindowState {
   public:
    WindowState() : m_mouseButtonFlags(0), m_mouseWheel(0) {
      memset(m_keyPressed, 0, sizeof(m_keyPressed));
      memset(m_keyToggled, 0, sizeof(m_keyToggled));
    }

    int m_winSize[2];
    int m_swapSize[2];
    int m_mouseCurrent[2];
    int m_mouseButtonFlags;
    int m_mouseWheel;

    bool m_keyPressed[KEY_LAST + 1];
    bool m_keyToggled[KEY_LAST + 1];

    bool onPress(int key) { return m_keyPressed[key] && m_keyToggled[key]; }
  };

  //////////////////////////////////////////////////////////////////////////

  WindowState m_windowState;
  nvh::Profiler m_profiler;
  bool m_profilerPrint;
  bool m_hadProfilerPrint;
  bool m_timeInTitle;

  ParameterList m_parameterList;

  AppWindowProfiler(bool deprecated = true)
      : m_profilerPrint(true),
        m_vsync(false),
        m_active(false),
        m_timeInTitle(true),
        m_hadScreenshot(false) {
    setupParameters();
  }

  // Sample Related
  //////////////////////////////////////////////////////////////////////////

  // setup sample (this is executed after window/context creation)
  virtual bool begin() { return false; }
  // tear down sample (triggered by ESC/window close)
  virtual void end() {}
  // do primary logic/drawing etc. here
  virtual void think(double time) {}
  // react on swapchain resizes here
  // may be different to winWidth/winHeight!
  virtual void resize(int swapWidth, int swapHeight) {}

  // return true to prevent m_window state updates
  virtual bool mouse_pos(int x, int y) { return false; }
  virtual bool mouse_button(int button, int action) { return false; }
  virtual bool mouse_wheel(int wheel) { return false; }
  virtual bool key_button(int button, int action, int modifier) {
    return false;
  }
  virtual bool key_char(int button) { return false; }

  virtual void parseConfig(int argc,
                           const char** argv,
                           const std::string& path) {
    // if you want to handle parameters not represented in
    // m_parameterList then override this function accordingly.
    m_parameterList.applyTokens(argc, argv, "-", path.c_str());
    // This function is called before "begin" and provided with the commandline
    // used in "run". It can also be called by the benchmarking system, and
    // parseConfigFile.
  }
  virtual bool validateConfig() {
    // override if you want to test the state of app after parsing configs
    // returning false terminates app
    return true;
  }

  // additional special-purpose callbacks

  virtual void postProfiling() {}
  virtual void postEnd() {}
  virtual void postBenchmarkAdvance() {}
  virtual void postConfigPreContext(){};

  //////////////////////////////////////////////////////////////////////////

  // initial kickoff (typically called from main)
  int run(const std::string& name,
          int argc,
          const char** argv,
          int width,
          int height,
          bool requireGLContext);
  void leave();

  void parseConfigFile(const char* filename);

  // handles special strings (returns empty string if
  // could not do the replacement properly)
  // known specials:
  // $DEVICE$
  std::string specialStrings(const char* original);

  void setVsync(bool state);
  bool getVsync() const { return m_vsync; }

  //////////////////////////////////////////////////////////////////////////
  // Context Window (if desired, not mandatory )
  //
  // Used when deriving from this class for the purpose of providing 3D Api
  // contexts nvvk/appwindowprofiler_vk or nvgl/appwindowprofiler_gl make use of
  // this.

  virtual void contextInit() {}
  virtual void contextDeinit() {}
  virtual void contextSync() {}
  virtual const char* contextGetDeviceName() { return NULL; }

  virtual void swapResize(int winWidth, int winHeight) {
    m_windowState.m_swapSize[0] = winWidth;
    m_windowState.m_swapSize[1] = winHeight;
  }
  virtual void swapPrepare() {}
  virtual void swapBuffers() {}
  virtual void swapVsync(bool state) {}

  //////////////////////////////////////////////////////////////////////////

  // inherited from NVPWindow, don't use them directly, use the "Sample-related"
  // ones
  void onWindowClose() override;
  void onWindowResize(int w, int h) override;
  void onWindowRefresh() override {
  }  // leave empty, we call redraw ourselves in think
  void onMouseMotion(int x, int y) override;
  void onMouseWheel(int delta) override;
  void onMouseButton(MouseButton button,
                     ButtonAction action,
                     int mods,
                     int x,
                     int y) override;
  void onKeyboard(KeyCode key,
                  ButtonAction action,
                  int mods,
                  int x,
                  int y) override;
  void onKeyboardChar(unsigned char key, int mods, int x, int y) override;

 private:
  struct Benchmark {
    std::string filename;
    std::string content;
    nvh::ParameterSequence sequence;
    uint32_t frameLength = 256;
    uint32_t frame = 0;
  };

  struct Config {
    int32_t winpos[2];
    int32_t winsize[2];
    bool vsyncstate = true;
    bool quickexit = false;
    uint32_t intervalSeconds = 2;
    uint32_t frameLimit = 0;
    uint32_t timerLimit = 0;
    std::string dumpatexitFilename;
    std::string screenshotFilename;
    std::string logFilename;
    std::string configFilename;
    uint32_t clearColor[3] = {127, 0, 0};

    Config() {
      winpos[0] = 50;
      winpos[1] = 50;
      winsize[0] = 0;
      winsize[1] = 0;
    }
  };

  void parameterCallback(uint32_t param);

  void setupParameters();
  void exitScreenshot();

  void initBenchmark();
  void advanceBenchmark();

  bool m_activeContext = false;
  bool m_active = false;
  bool m_vsync;
  bool m_hadScreenshot;
  Config m_config;
  Benchmark m_benchmark;

  uint32_t m_paramWinsize;
  uint32_t m_paramVsync;
  uint32_t m_paramScreenshot;
  uint32_t m_paramLog;
  uint32_t m_paramCfg;
  uint32_t m_paramBat;
  uint32_t m_paramClear;
};
}  // namespace nvh

#endif
