/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
//--------------------------------------------------------------------

#include "nvpsystem.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#ifdef NVP_SUPPORTS_SOCKETS
#include "socketSampleMessages.h"
#endif

#include <algorithm>

static bool s_sysInit = false;

static void cb_errorfun(int, const char* str) {
  LOGE("%s\n", str);
}

//---------------------------------------------------------------------------
// Message pump
void NVPSystem::pollEvents() {
#ifdef NVP_SUPPORTS_SOCKETS
  // check the stack of messages from remote connection, first
  processRemoteMessages();
#endif
  glfwPollEvents();
}

void NVPSystem::waitEvents() {
  glfwWaitEvents();
}

void NVPSystem::postTiming(float ms, int fps, const char* details) {
#ifdef NVP_SUPPORTS_SOCKETS
  ::postTiming(ms, fps, details);
#endif
}

double NVPSystem::getTime() {
  return glfwGetTime();
}

void NVPSystem::init(const char* projectName) {
  std::string logfile =
      std::string("log_") + std::string(projectName) + std::string(".txt");
  nvprintSetLogFileName(logfile.c_str());

  int result = glfwInit();
  if (!result) {
    LOGE("could not init glfw\n");
    exit(-1);
  }

  glfwSetErrorCallback(cb_errorfun);

  // initNSight();
#ifdef NVP_SUPPORTS_SOCKETS
  //
  // Socket init if needed
  //
  startSocketServer(/*port*/ 1056);
#endif

  s_sysInit = true;
  platformInit();
}

void NVPSystem::deinit() {
  platformDeinit();
  glfwTerminate();
}

bool NVPSystem::isInited() {
  return s_sysInit;
}
