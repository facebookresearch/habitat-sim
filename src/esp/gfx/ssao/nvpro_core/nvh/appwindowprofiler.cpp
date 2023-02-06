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

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "appwindowprofiler.hpp"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include "fileoperations.hpp"
#include "misc.hpp"
// #include <fileformats/bmp.hpp"

namespace nvh {

static void replace(std::string& str,
                    const std::string& from,
                    const std::string& to) {
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

static void fixDeviceName(std::string& deviceName) {
  replace(deviceName, "INTEL(R) ", "");
  replace(deviceName, "AMD ", "");
  replace(deviceName, "DRI ", "");
  replace(deviceName, "(TM) ", "");
  replace(deviceName, " Series", "");
  replace(deviceName, " Graphics", "");
  replace(deviceName, "/PCIe/SSE2", "");
  std::replace(deviceName.begin(), deviceName.end(), ' ', '_');

  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '/'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '\\'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), ':'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '?'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '*'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '<'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '>'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '|'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), '"'),
                   deviceName.end());
  deviceName.erase(std::remove(deviceName.begin(), deviceName.end(), ','),
                   deviceName.end());
}

void AppWindowProfiler::onMouseMotion(int x, int y) {
  AppWindowProfiler::WindowState& window = m_windowState;

  if (!window.m_mouseButtonFlags && mouse_pos(x, y))
    return;

  window.m_mouseCurrent[0] = x;
  window.m_mouseCurrent[1] = y;
}

void AppWindowProfiler::onMouseButton(MouseButton Button,
                                      ButtonAction Action,
                                      int mods,
                                      int x,
                                      int y) {
  AppWindowProfiler::WindowState& window = m_windowState;
  m_profiler.reset();

  if (mouse_button(Button, Action))
    return;

  switch (Action) {
    case BUTTON_PRESS: {
      switch (Button) {
        case MOUSE_BUTTON_LEFT: {
          window.m_mouseButtonFlags |= MOUSE_BUTTONFLAG_LEFT;
        } break;
        case MOUSE_BUTTON_MIDDLE: {
          window.m_mouseButtonFlags |= MOUSE_BUTTONFLAG_MIDDLE;
        } break;
        case MOUSE_BUTTON_RIGHT: {
          window.m_mouseButtonFlags |= MOUSE_BUTTONFLAG_RIGHT;
        } break;
      }
    } break;
    case BUTTON_RELEASE: {
      if (!window.m_mouseButtonFlags)
        break;

      switch (Button) {
        case MOUSE_BUTTON_LEFT: {
          window.m_mouseButtonFlags &= ~MOUSE_BUTTONFLAG_LEFT;
        } break;
        case MOUSE_BUTTON_MIDDLE: {
          window.m_mouseButtonFlags &= ~MOUSE_BUTTONFLAG_MIDDLE;
        } break;
        case MOUSE_BUTTON_RIGHT: {
          window.m_mouseButtonFlags &= ~MOUSE_BUTTONFLAG_RIGHT;
        } break;
      }
    } break;
  }
}

void AppWindowProfiler::onMouseWheel(int y) {
  AppWindowProfiler::WindowState& window = m_windowState;
  m_profiler.reset();

  if (mouse_wheel(y))
    return;

  window.m_mouseWheel += y;
}

void AppWindowProfiler::onKeyboard(KeyCode key,
                                   ButtonAction action,
                                   int mods,
                                   int x,
                                   int y) {
  AppWindowProfiler::WindowState& window = m_windowState;
  m_profiler.reset();

  if (key_button(key, action, mods))
    return;

  bool newState = false;

  switch (action) {
    case BUTTON_PRESS:
    case BUTTON_REPEAT: {
      newState = true;
      break;
    }
    case BUTTON_RELEASE: {
      newState = false;
      break;
    }
  }

  window.m_keyToggled[key] = window.m_keyPressed[key] != newState;
  window.m_keyPressed[key] = newState;
}

void AppWindowProfiler::onKeyboardChar(unsigned char key,
                                       int mods,
                                       int x,
                                       int y) {
  m_profiler.reset();

  if (key_char(key))
    return;
}

void AppWindowProfiler::parseConfigFile(const char* filename) {
  std::string result = loadFile(filename, false);
  if (result.empty()) {
    LOGW("file not found: %s\n", filename);
    return;
  }
  std::vector<const char*> args;
  ParameterList::tokenizeString(result, args);

  std::string path = getFilePath(filename);

  parseConfig(uint32_t(args.size()), args.data(), path);
}

void AppWindowProfiler::onWindowClose() {
  exitScreenshot();
}

void AppWindowProfiler::onWindowResize(int width, int height) {
  m_profiler.reset();

  if (width == 0 || height == 0) {
    return;
  }

  m_windowState.m_winSize[0] = width;
  m_windowState.m_winSize[1] = height;
  if (m_activeContext) {
    swapResize(width, height);
  }
  if (m_active) {
    resize(m_windowState.m_swapSize[0], m_windowState.m_swapSize[1]);
  }
}

void AppWindowProfiler::setVsync(bool state) {
  if (m_internal) {
    swapVsync(state);
    LOGI("vsync: %s\n", state ? "on" : "off");
  }
  m_config.vsyncstate = state;
  m_vsync = state;
}

int AppWindowProfiler::run(const std::string& title,
                           int argc,
                           const char** argv,
                           int width,
                           int height,
                           bool requireGLContext) {
  m_config.winsize[0] = m_config.winsize[0] ? m_config.winsize[0] : width;
  m_config.winsize[1] = m_config.winsize[1] ? m_config.winsize[1] : height;

  // skip first argument here (exe file)
  parseConfig(argc - 1, argv + 1, ".");
  if (!validateConfig()) {
    return EXIT_FAILURE;
  }

  if (!NVPWindow::open(m_config.winpos[0], m_config.winpos[1],
                       m_config.winsize[0], m_config.winsize[1], title.c_str(),
                       requireGLContext)) {
    LOGE("Could not create window\n");
    return EXIT_FAILURE;
  }
  m_windowState.m_winSize[0] = m_config.winsize[0];
  m_windowState.m_winSize[1] = m_config.winsize[1];

  postConfigPreContext();
  contextInit();
  m_activeContext = true;

  // hack to react on $DEVICE$ filename
  if (!m_config.logFilename.empty()) {
    parameterCallback(m_paramLog);
  }

  if (contextGetDeviceName()) {
    std::string deviceName = contextGetDeviceName();
    fixDeviceName(deviceName);
    LOGOK("DEVICE: %s\n", deviceName.c_str());
  }

  initBenchmark();

  setVsync(m_config.vsyncstate);

  bool Run = begin();
  m_active = true;

  bool quickExit = m_config.quickexit;
  if (m_config.frameLimit) {
    m_profilerPrint = false;
    quickExit = true;
  }

  double timeStart = getTime();
  double timeBegin = getTime();
  double frames = 0;

  bool lastVsync = m_vsync;

  m_hadProfilerPrint = false;

  double lastProfilerPrintTime = 0;

  if (Run) {
    while (pollEvents()) {
      bool wasClosed = false;
      while (!isOpen()) {
        NVPSystem::waitEvents();
        wasClosed = true;
      }
      if (wasClosed) {
        continue;
      }

      if (m_windowState.onPress(KEY_V)) {
        setVsync(!m_vsync);
      }

      std::string stats;
      {
        bool benchmarkActive = m_benchmark.sequence.isActive();
        double curTime = getTime();
        double printInterval = m_profilerPrint && !benchmarkActive
                                   ? float(m_config.intervalSeconds)
                                   : float(FLT_MAX);
        bool printStats = ((curTime - lastProfilerPrintTime) > printInterval);

        if (printStats) {
          lastProfilerPrintTime = curTime;
        }
        m_profiler.beginFrame();

        swapPrepare();
        {
          // const nvh::Profiler::Section profile(m_profiler, "App");
          think(getTime() - timeStart);
        }
        memset(m_windowState.m_keyToggled, 0,
               sizeof(m_windowState.m_keyToggled));
        swapBuffers();

        m_profiler.endFrame();
        if (printStats) {
          m_profiler.print(stats);
        }
      }

      m_hadProfilerPrint = false;

      if (m_profilerPrint && !stats.empty()) {
        if (!m_config.timerLimit || m_config.timerLimit == 1) {
          LOGI("%s\n", stats.c_str());
          m_hadProfilerPrint = true;
        }
        if (m_config.timerLimit == 1) {
          m_config.frameLimit = 1;
        }
        if (m_config.timerLimit) {
          m_config.timerLimit--;
        }
      }

      advanceBenchmark();
      postProfiling();

      frames++;

      double timeCurrent = getTime();
      double timeDelta = timeCurrent - timeBegin;
      if (timeDelta > double(m_config.intervalSeconds) ||
          lastVsync != m_vsync || m_config.frameLimit == 1) {
        std::ostringstream combined;

        if (lastVsync != m_vsync) {
          timeDelta = 0;
        }

        if (m_timeInTitle) {
          combined << title << ": " << (timeDelta * 1000.0 / (frames))
                   << " [ms]" << (m_vsync ? " (vsync on - V for toggle)" : "");
          setTitle(combined.str().c_str());
        }

        if (m_config.frameLimit == 1) {
          LOGI("frametime: %f ms\n", (timeDelta * 1000.0 / (frames)));
        }

        frames = 0;
        timeBegin = timeCurrent;
        lastVsync = m_vsync;
      }

      if (m_windowState.m_keyPressed[KEY_ESCAPE] || m_config.frameLimit == 1)
        break;

      if (m_config.frameLimit)
        m_config.frameLimit--;
    }
  }
  contextSync();
  exitScreenshot();

  if (quickExit) {
    exit(EXIT_SUCCESS);
    return EXIT_SUCCESS;
  }

  end();
  m_active = false;
  contextDeinit();
  postEnd();

  return Run ? EXIT_SUCCESS : EXIT_FAILURE;
}

void AppWindowProfiler::leave() {
  m_config.frameLimit = 1;
}

std::string AppWindowProfiler::specialStrings(const char* original) {
  std::string str(original);

  if (strstr(original, "$DEVICE$")) {
    if (contextGetDeviceName()) {
      std::string deviceName = contextGetDeviceName();
      fixDeviceName(deviceName);
      if (deviceName.empty()) {
        // no proper device name available
        return std::string();
      }

      // replace $DEVICE$
      replace(str, "$DEVICE$", deviceName);
    } else {
      // no proper device name available
      return std::string();
    }
  }
  return str;
}

void AppWindowProfiler::parameterCallback(uint32_t param) {
  if (param == m_paramLog) {
    std::string logfileName = specialStrings(m_config.logFilename.c_str());
    if (!logfileName.empty()) {
      nvprintSetLogFileName(logfileName.c_str());
    }
  } else if (param == m_paramCfg || param == m_paramBat) {
    parseConfigFile(m_config.configFilename.c_str());
  } else if (param == m_paramWinsize) {
    if (m_internal) {
      setWindowSize(m_config.winsize[0], m_config.winsize[1]);
    }
  }

  if (!m_active)
    return;

  if (param == m_paramVsync) {
    setVsync(m_config.vsyncstate);
  } else if (param == m_paramScreenshot) {
    std::string filename = specialStrings(m_config.screenshotFilename.c_str());
    if (!filename.empty()) {
      screenshot(filename.c_str());
    }
  } else if (param == m_paramClear) {
    clear(m_config.clearColor[0], m_config.clearColor[1],
          m_config.clearColor[2]);
  }
}

void AppWindowProfiler::setupParameters() {
  nvh::ParameterList::Callback callback = [&](uint32_t param) {
    parameterCallback(param);
  };

  m_paramWinsize =
      m_parameterList.add("winsize|Set window size (width and height)",
                          m_config.winsize, callback, 2);
  m_paramVsync = m_parameterList.add("vsync|Enable or disable vsync",
                                     &m_config.vsyncstate, callback);
  m_paramLog = m_parameterList.addFilename("logfile|Set logfile",
                                           &m_config.logFilename, callback);
  m_paramCfg =
      m_parameterList.addFilename(".cfg|load parameters from this config file",
                                  &m_config.configFilename, callback);
  m_paramBat =
      m_parameterList.addFilename(".bat|load parameters from this batch file",
                                  &m_config.configFilename, callback);
  m_parameterList.add("winpos|Set window position (x and y)", m_config.winpos,
                      nullptr, 2);
  m_parameterList.add("frames|Set number of frames to render before exit",
                      &m_config.frameLimit);
  m_parameterList.add(
      "timerprints|Set number of timerprints to do, before exit",
      &m_config.timerLimit);
  m_parameterList.add("timerinterval|Set interval of timer prints in seconds",
                      &m_config.intervalSeconds);
  m_parameterList.add(
      "bmpatexit|Set file to store a bitmap image of the last frame at exit",
      &m_config.dumpatexitFilename);
  m_parameterList.addFilename("benchmark|Set benchmark filename",
                              &m_benchmark.filename);
  m_parameterList.add("benchmarkframes|Set number of benchmarkframes",
                      &m_benchmark.frameLength);
  m_parameterList.add("quickexit|skips tear down", &m_config.quickexit);
  m_paramScreenshot =
      m_parameterList.add("screenshot|makes a screenshot into this file",
                          &m_config.screenshotFilename, callback);
  m_paramClear =
      m_parameterList.add("clear|clears window color (r,b,g in 0-255) using OS",
                          m_config.clearColor, callback, 3);
}

void AppWindowProfiler::exitScreenshot() {
  if (!m_config.dumpatexitFilename.empty() && !m_hadScreenshot) {
    screenshot(m_config.dumpatexitFilename.c_str());
    m_hadScreenshot = true;
  }
}

void AppWindowProfiler::initBenchmark() {
  if (m_benchmark.filename.empty())
    return;

  m_benchmark.content = loadFile(m_benchmark.filename.c_str(), false);
  if (!m_benchmark.content.empty()) {
    std::vector<const char*> tokens;
    ParameterList::tokenizeString(m_benchmark.content, tokens);

    std::string path = getFilePath(m_benchmark.filename.c_str());

    m_benchmark.sequence.init(&m_parameterList, tokens);

    // do first iteration manually, due to custom arg parsing
    uint32_t argBegin;
    uint32_t argCount;
    if (!m_benchmark.sequence.advanceIteration("benchmark", 1, argBegin,
                                               argCount)) {
      parseConfig(argCount, &tokens[argBegin], path);
    }

    m_profiler.reset(nvh::Profiler::CONFIG_DELAY);

    m_benchmark.frame = 0;
    m_profilerPrint = false;
  }
}

void AppWindowProfiler::advanceBenchmark() {
  if (!m_benchmark.sequence.isActive())
    return;

  m_benchmark.frame++;

  if (m_benchmark.frame > m_benchmark.frameLength +
                              nvh::Profiler::CONFIG_DELAY +
                              nvh::Profiler::FRAME_DELAY) {
    m_benchmark.frame = 0;

    std::string stats;
    m_profiler.print(stats);
    LOGI("BENCHMARK %d \"%s\" {\n", m_benchmark.sequence.getIteration(),
         m_benchmark.sequence.getSeparatorArg(0));
    LOGI("%s}\n\n", stats.c_str());

    bool done = m_benchmark.sequence.applyIteration("benchmark", 1, "-");
    m_profiler.reset(nvh::Profiler::CONFIG_DELAY);

    postBenchmarkAdvance();

    if (done) {
      leave();
    }
  }
}

}  // namespace nvh
