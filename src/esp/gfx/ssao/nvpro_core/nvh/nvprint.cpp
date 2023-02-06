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

#include "nvprint.hpp"

#include <mutex>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

static char s_logFileNameDefault[] = "log_nvprosample.txt";
static char* s_logFileName = s_logFileNameDefault;
static size_t s_strBuffer_sz = 0;
static char* s_strBuffer = nullptr;
static FILE* s_fd = nullptr;
static bool s_bLogReady = false;
static bool s_bPrintLogging = true;
static uint32_t s_bPrintFileLogging = ~0;
static int s_printLevel = -1;  // <0 mean no level prefix
static PFN_NVPRINTCALLBACK s_printCallback = nullptr;
static std::mutex s_mutex;

void nvprintSetLogFileName(const char* name) {
  std::lock_guard<std::mutex> lockGuard(s_mutex);

  if (name == NULL || strcmp(s_logFileName, name) == 0)
    return;

  size_t l = strlen(name) + 1;
  s_logFileName = new char[l];
  strncpy(s_logFileName, name, l);

  if (s_fd) {
    fclose(s_fd);
    s_fd = nullptr;
    s_bLogReady = false;
  }
}
void nvprintSetCallback(PFN_NVPRINTCALLBACK callback) {
  s_printCallback = callback;
}
void nvprintSetLevel(int l) {
  s_printLevel = l;
}
int nvprintGetLevel() {
  return s_printLevel;
}
void nvprintSetLogging(bool b) {
  s_bPrintLogging = b;
}

void nvprintSetFileLogging(bool state, uint32_t mask) {
  std::lock_guard<std::mutex> lockGuard(s_mutex);

  if (state) {
    s_bPrintFileLogging |= mask;
  } else {
    s_bPrintFileLogging &= ~mask;
  }
}

void nvprintf2(va_list& vlist, const char* fmt, int level) {
  if (s_bPrintLogging == false) {
    return;
  }

  std::lock_guard<std::mutex> lockGuard(s_mutex);
  if (s_strBuffer_sz == 0) {
    s_strBuffer_sz = 1024;
    s_strBuffer = (char*)malloc(s_strBuffer_sz);
  }
  while ((vsnprintf(s_strBuffer, s_strBuffer_sz - 1, fmt, vlist)) <
         0)  // means there wasn't enough room
  {
    s_strBuffer_sz *= 2;
    char* tmp = (char*)realloc(s_strBuffer, s_strBuffer_sz);
    if (tmp == nullptr)  // !C6308
      return;
    s_strBuffer = tmp;
  }

  // Do nothing if allocating/reallocating s_strBuffer failed
  if (!s_strBuffer) {
    return;
  }

#ifdef WIN32
  OutputDebugStringA(s_strBuffer);
#endif

  if (s_bPrintFileLogging & (1 << level)) {
    if (s_bLogReady == false) {
      s_fd = fopen(s_logFileName, "wt");
      s_bLogReady = true;
    }
    if (s_fd) {
      fputs(s_strBuffer, s_fd);
    }
  }

  if (s_printCallback) {
    s_printCallback(level, s_strBuffer);
  }
  ::printf("%s", s_strBuffer);
}
void nvprintf(
#ifdef _MSC_VER
    _Printf_format_string_
#endif
    const char* fmt,
    ...) {
  //    int r = 0;
  va_list vlist;
  va_start(vlist, fmt);
  nvprintf2(vlist, fmt, s_printLevel);
  va_end(vlist);
}
void nvprintfLevel(int level,
#ifdef _MSC_VER
                   _Printf_format_string_
#endif
                   const char* fmt,
                   ...) {
  va_list vlist;
  va_start(vlist, fmt);
  nvprintf2(vlist, fmt, level);
  va_end(vlist);
}
