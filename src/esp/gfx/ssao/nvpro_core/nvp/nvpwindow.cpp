/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2018-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
//--------------------------------------------------------------------

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "nvpwindow.hpp"

#include <stdio.h>
#include <algorithm>
#include <string>

static_assert(NVPWindow::BUTTON_RELEASE == GLFW_RELEASE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::BUTTON_PRESS == GLFW_PRESS, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::BUTTON_REPEAT == GLFW_REPEAT,
              "glfw/nvpwindow mismatch");

static_assert(NVPWindow::MOUSE_BUTTON_LEFT == GLFW_MOUSE_BUTTON_LEFT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::MOUSE_BUTTON_RIGHT == GLFW_MOUSE_BUTTON_RIGHT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::MOUSE_BUTTON_MIDDLE == GLFW_MOUSE_BUTTON_MIDDLE,
              "glfw/nvpwindow mismatch");

static_assert(NVPWindow::KMOD_SHIFT == GLFW_MOD_SHIFT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KMOD_CONTROL == GLFW_MOD_CONTROL,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KMOD_ALT == GLFW_MOD_ALT, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KMOD_SUPER == GLFW_MOD_SUPER,
              "glfw/nvpwindow mismatch");

/*
for key in keysheader:gmatch("#define ([%w_]+)") do
  print("static_assert(NVPWindow::"..key:sub(6,-1).." == "..key..",
\"glfw/nvpwindow mismatch\");") end
*/

static_assert(NVPWindow::KEY_UNKNOWN == GLFW_KEY_UNKNOWN,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_SPACE == GLFW_KEY_SPACE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_APOSTROPHE == GLFW_KEY_APOSTROPHE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_COMMA == GLFW_KEY_COMMA,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_MINUS == GLFW_KEY_MINUS,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_PERIOD == GLFW_KEY_PERIOD,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_SLASH == GLFW_KEY_SLASH,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_0 == GLFW_KEY_0, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_1 == GLFW_KEY_1, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_2 == GLFW_KEY_2, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_3 == GLFW_KEY_3, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_4 == GLFW_KEY_4, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_5 == GLFW_KEY_5, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_6 == GLFW_KEY_6, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_7 == GLFW_KEY_7, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_8 == GLFW_KEY_8, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_9 == GLFW_KEY_9, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_SEMICOLON == GLFW_KEY_SEMICOLON,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_EQUAL == GLFW_KEY_EQUAL,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_A == GLFW_KEY_A, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_B == GLFW_KEY_B, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_C == GLFW_KEY_C, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_D == GLFW_KEY_D, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_E == GLFW_KEY_E, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F == GLFW_KEY_F, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_G == GLFW_KEY_G, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_H == GLFW_KEY_H, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_I == GLFW_KEY_I, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_J == GLFW_KEY_J, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_K == GLFW_KEY_K, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_L == GLFW_KEY_L, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_M == GLFW_KEY_M, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_N == GLFW_KEY_N, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_O == GLFW_KEY_O, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_P == GLFW_KEY_P, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_Q == GLFW_KEY_Q, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_R == GLFW_KEY_R, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_S == GLFW_KEY_S, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_T == GLFW_KEY_T, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_U == GLFW_KEY_U, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_V == GLFW_KEY_V, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_W == GLFW_KEY_W, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_X == GLFW_KEY_X, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_Y == GLFW_KEY_Y, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_Z == GLFW_KEY_Z, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LEFT_BRACKET == GLFW_KEY_LEFT_BRACKET,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_BACKSLASH == GLFW_KEY_BACKSLASH,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_RIGHT_BRACKET == GLFW_KEY_RIGHT_BRACKET,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_GRAVE_ACCENT == GLFW_KEY_GRAVE_ACCENT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_WORLD_1 == GLFW_KEY_WORLD_1,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_WORLD_2 == GLFW_KEY_WORLD_2,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_ESCAPE == GLFW_KEY_ESCAPE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_ENTER == GLFW_KEY_ENTER,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_TAB == GLFW_KEY_TAB, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_BACKSPACE == GLFW_KEY_BACKSPACE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_INSERT == GLFW_KEY_INSERT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_DELETE == GLFW_KEY_DELETE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_RIGHT == GLFW_KEY_RIGHT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LEFT == GLFW_KEY_LEFT, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_DOWN == GLFW_KEY_DOWN, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_UP == GLFW_KEY_UP, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_PAGE_UP == GLFW_KEY_PAGE_UP,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_PAGE_DOWN == GLFW_KEY_PAGE_DOWN,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_HOME == GLFW_KEY_HOME, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_END == GLFW_KEY_END, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_CAPS_LOCK == GLFW_KEY_CAPS_LOCK,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_SCROLL_LOCK == GLFW_KEY_SCROLL_LOCK,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_NUM_LOCK == GLFW_KEY_NUM_LOCK,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_PRINT_SCREEN == GLFW_KEY_PRINT_SCREEN,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_PAUSE == GLFW_KEY_PAUSE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F1 == GLFW_KEY_F1, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F2 == GLFW_KEY_F2, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F3 == GLFW_KEY_F3, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F4 == GLFW_KEY_F4, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F5 == GLFW_KEY_F5, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F6 == GLFW_KEY_F6, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F7 == GLFW_KEY_F7, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F8 == GLFW_KEY_F8, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F9 == GLFW_KEY_F9, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F10 == GLFW_KEY_F10, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F11 == GLFW_KEY_F11, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F12 == GLFW_KEY_F12, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F13 == GLFW_KEY_F13, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F14 == GLFW_KEY_F14, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F15 == GLFW_KEY_F15, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F16 == GLFW_KEY_F16, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F17 == GLFW_KEY_F17, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F18 == GLFW_KEY_F18, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F19 == GLFW_KEY_F19, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F20 == GLFW_KEY_F20, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F21 == GLFW_KEY_F21, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F22 == GLFW_KEY_F22, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F23 == GLFW_KEY_F23, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F24 == GLFW_KEY_F24, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_F25 == GLFW_KEY_F25, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_0 == GLFW_KEY_KP_0, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_1 == GLFW_KEY_KP_1, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_2 == GLFW_KEY_KP_2, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_3 == GLFW_KEY_KP_3, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_4 == GLFW_KEY_KP_4, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_5 == GLFW_KEY_KP_5, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_6 == GLFW_KEY_KP_6, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_7 == GLFW_KEY_KP_7, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_8 == GLFW_KEY_KP_8, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_9 == GLFW_KEY_KP_9, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_DECIMAL == GLFW_KEY_KP_DECIMAL,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_DIVIDE == GLFW_KEY_KP_DIVIDE,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_MULTIPLY == GLFW_KEY_KP_MULTIPLY,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_SUBTRACT == GLFW_KEY_KP_SUBTRACT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_ADD == GLFW_KEY_KP_ADD,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_ENTER == GLFW_KEY_KP_ENTER,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_KP_EQUAL == GLFW_KEY_KP_EQUAL,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LEFT_SHIFT == GLFW_KEY_LEFT_SHIFT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LEFT_CONTROL == GLFW_KEY_LEFT_CONTROL,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LEFT_ALT == GLFW_KEY_LEFT_ALT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LEFT_SUPER == GLFW_KEY_LEFT_SUPER,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_RIGHT_SHIFT == GLFW_KEY_RIGHT_SHIFT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_RIGHT_CONTROL == GLFW_KEY_RIGHT_CONTROL,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_RIGHT_ALT == GLFW_KEY_RIGHT_ALT,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_RIGHT_SUPER == GLFW_KEY_RIGHT_SUPER,
              "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_MENU == GLFW_KEY_MENU, "glfw/nvpwindow mismatch");
static_assert(NVPWindow::KEY_LAST == GLFW_KEY_LAST, "glfw/nvpwindow mismatch");

void NVPWindow::cb_windowrefreshfun(GLFWwindow* glfwwin) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->onWindowRefresh();
}

void NVPWindow::cb_windowsizefun(GLFWwindow* glfwwin, int w, int h) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->m_windowSize[0] = w;
  win->m_windowSize[1] = h;
  win->onWindowResize(w, h);
}
void NVPWindow::cb_windowclosefun(GLFWwindow* glfwwin) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  win->m_isClosing = true;
  win->onWindowClose();
}

void NVPWindow::cb_mousebuttonfun(GLFWwindow* glfwwin,
                                  int button,
                                  int action,
                                  int mods) {
  double x, y;
  glfwGetCursorPos(glfwwin, &x, &y);

  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->m_keyModifiers = mods;
  win->m_mouseX = int(x);
  win->m_mouseY = int(y);
  win->onMouseButton((NVPWindow::MouseButton)button,
                     (NVPWindow::ButtonAction)action, mods, win->m_mouseX,
                     win->m_mouseY);
}
void NVPWindow::cb_cursorposfun(GLFWwindow* glfwwin, double x, double y) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->m_mouseX = int(x);
  win->m_mouseY = int(y);
  win->onMouseMotion(win->m_mouseX, win->m_mouseY);
}
void NVPWindow::cb_scrollfun(GLFWwindow* glfwwin, double x, double y) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->m_mouseWheel += int(y);
  win->onMouseWheel(int(y));
}
void NVPWindow::cb_keyfun(GLFWwindow* glfwwin,
                          int key,
                          int scancode,
                          int action,
                          int mods) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->m_keyModifiers = mods;
  win->onKeyboard((NVPWindow::KeyCode)key, (NVPWindow::ButtonAction)action,
                  mods, win->m_mouseX, win->m_mouseY);
}
void NVPWindow::cb_charfun(GLFWwindow* glfwwin, unsigned int codepoint) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->onKeyboardChar(codepoint, win->m_keyModifiers, win->m_mouseX,
                      win->m_mouseY);
}

void NVPWindow::cb_dropfun(GLFWwindow* glfwwin, int count, const char** paths) {
  NVPWindow* win = (NVPWindow*)glfwGetWindowUserPointer(glfwwin);
  if (win->isClosing())
    return;
  win->onDragDrop(count, paths);
}

bool NVPWindow::isClosing() const {
  return m_isClosing || glfwWindowShouldClose(m_internal);
}

bool NVPWindow::isOpen() const {
  return glfwGetWindowAttrib(m_internal, GLFW_VISIBLE) == GLFW_TRUE &&
         glfwGetWindowAttrib(m_internal, GLFW_ICONIFIED) == GLFW_FALSE &&
         !isClosing();
}

bool NVPWindow::open(int posX,
                     int posY,
                     int width,
                     int height,
                     const char* title,
                     bool requireGLContext) {
  NV_ASSERT(NVPSystem::isInited() && "NVPSystem::Init not called");

  m_windowSize[0] = width;
  m_windowSize[1] = height;

  m_windowName = title ? title : "Sample";

#ifdef _WIN32
  (void)requireGLContext;
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
#else
  if (!requireGLContext) {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  } else {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    // Some samples make use of compatibility profile features
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
#ifdef _DEBUG
#ifdef GLFW_CONTEXT_DEBUG  // Since GLFW_CONTEXT_DEBUG is new in GLFW 3.4
    glfwWindowHint(GLFW_CONTEXT_DEBUG, 1);
#else
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, 1);
#endif
#endif
  }
#endif

  m_internal = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (!m_internal) {
    return false;
  }

  if (posX != 0 || posY != 0) {
    glfwSetWindowPos(m_internal, posX, posY);
  }
  glfwSetWindowUserPointer(m_internal, this);
  glfwSetWindowRefreshCallback(m_internal, cb_windowrefreshfun);
  glfwSetWindowCloseCallback(m_internal, cb_windowclosefun);
  glfwSetCursorPosCallback(m_internal, cb_cursorposfun);
  glfwSetMouseButtonCallback(m_internal, cb_mousebuttonfun);
  glfwSetKeyCallback(m_internal, cb_keyfun);
  glfwSetScrollCallback(m_internal, cb_scrollfun);
  glfwSetCharCallback(m_internal, cb_charfun);
  glfwSetWindowSizeCallback(m_internal, cb_windowsizefun);
  glfwSetDropCallback(m_internal, cb_dropfun);

  return true;
}

void NVPWindow::deinit() {
  glfwDestroyWindow(m_internal);
  m_internal = nullptr;
  m_windowSize[0] = 0;
  m_windowSize[1] = 0;
  m_windowName = std::string();
}

void NVPWindow::close() {
  glfwSetWindowShouldClose(m_internal, GLFW_TRUE);
}

void NVPWindow::setTitle(const char* title) {
  glfwSetWindowTitle(m_internal, title);
}

void NVPWindow::maximize() {
  glfwMaximizeWindow(m_internal);
}

void NVPWindow::restore() {
  glfwRestoreWindow(m_internal);
}

void NVPWindow::minimize() {
  glfwIconifyWindow(m_internal);
}

void NVPWindow::setWindowPos(int x, int y) {
  glfwSetWindowPos(m_internal, x, y);
}

void NVPWindow::setWindowSize(int w, int h) {
  glfwSetWindowSize(m_internal, w, h);
}

std::string NVPWindow::openFileDialog(const char* title, const char* exts) {
  return NVPSystem::windowOpenFileDialog(m_internal, title, exts);
}
std::string NVPWindow::saveFileDialog(const char* title, const char* exts) {
  return NVPSystem::windowSaveFileDialog(m_internal, title, exts);
}
void NVPWindow::screenshot(const char* filename) {
  NVPSystem::windowScreenshot(m_internal, filename);
}
void NVPWindow::clear(uint32_t r, uint32_t g, uint32_t b) {
  NVPSystem::windowClear(m_internal, r, g, b);
}

void NVPWindow::setFullScreen(bool bYes) {
  if (bYes == m_isFullScreen)
    return;

  GLFWmonitor* monitor = glfwGetWindowMonitor(m_internal);
  const GLFWvidmode* mode = glfwGetVideoMode(monitor);

  if (bYes) {
    glfwGetWindowPos(m_internal, &m_preFullScreenPos[0],
                     &m_preFullScreenPos[1]);
    glfwGetWindowSize(m_internal, &m_preFullScreenSize[0],
                      &m_preFullScreenSize[1]);
    glfwSetWindowMonitor(m_internal, monitor, 0, 0, mode->width, mode->height,
                         mode->refreshRate);
    glfwSetWindowAttrib(m_internal, GLFW_RESIZABLE, GLFW_FALSE);
    glfwSetWindowAttrib(m_internal, GLFW_DECORATED, GLFW_FALSE);
  } else {
    glfwSetWindowMonitor(m_internal, nullptr, m_preFullScreenPos[0],
                         m_preFullScreenPos[1], m_preFullScreenSize[0],
                         m_preFullScreenSize[1], 0);
    glfwSetWindowAttrib(m_internal, GLFW_RESIZABLE, GLFW_TRUE);
    glfwSetWindowAttrib(m_internal, GLFW_DECORATED, GLFW_TRUE);
  }

  m_isFullScreen = bYes;
}
