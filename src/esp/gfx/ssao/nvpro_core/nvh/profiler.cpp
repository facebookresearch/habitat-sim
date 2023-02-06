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

#include "profiler.hpp"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////

namespace nvh {

const uint32_t Profiler::CONFIG_DELAY;
const uint32_t Profiler::FRAME_DELAY;
const uint32_t Profiler::START_SECTIONS;
const uint32_t Profiler::MAX_NUM_AVERAGE;

Profiler::Profiler(Profiler* master) {
  m_data = master ? master->m_data : std::shared_ptr<Data>(new Data);
  grow(START_SECTIONS);
}

Profiler::Profiler(uint32_t startSections) {
  m_data = std::shared_ptr<Data>(new Data);
  grow(startSections);
}

void Profiler::setAveragingSize(uint32_t num) {
  assert(num <= MAX_NUM_AVERAGE);
  m_data->numAveraging = num;

  for (size_t i = 0; i < m_data->entries.size(); i++) {
    m_data->entries[i].cpuTime.init(num);
    m_data->entries[i].gpuTime.init(num);
  }
  m_data->cpuTime.init(num);
}

void Profiler::beginFrame() {
  m_data->level = 0;
  m_data->nextSection = 0;
  m_data->frameSections.clear();

  m_data->cpuCurrentTime = -m_clock.getMicroSeconds();
}

void Profiler::endFrame() {
  assert(m_data->level == 0);

  m_data->cpuCurrentTime += m_clock.getMicroSeconds();

  if (!m_data->frameSections.empty() &&
      ((uint32_t)m_data->frameSections.size() != m_data->numLastEntries)) {
    m_data->numLastEntries = (uint32_t)m_data->frameSections.size();
    m_data->numLastSections = m_data->frameSections.back() + 1;
    m_data->resetDelay = CONFIG_DELAY;
  }

  if (m_data->resetDelay) {
    m_data->resetDelay--;
    for (uint32_t i = 0; i < m_data->entries.size(); i++) {
      Entry& entry = m_data->entries[i];
      if (entry.level != LEVEL_SINGLESHOT) {
        entry.numTimes = 0;
        entry.cpuTime.reset();
        entry.gpuTime.reset();
      }
    }
    m_data->cpuTime.reset();
    m_data->numFrames = 0;
  }

  if (m_data->numFrames > FRAME_DELAY) {
    for (uint32_t i : m_data->frameSections) {
      Entry& entry = m_data->entries[i];

      if (entry.splitter)
        continue;

      uint32_t queryFrame = (m_data->numFrames + 1) % FRAME_DELAY;
      bool available =
          entry.api == nullptr ||
          entry.gpuTimeProvider(i, queryFrame, entry.gpuTimes[queryFrame]);

      if (available) {
        entry.cpuTime.add(entry.cpuTimes[queryFrame]);
        entry.gpuTime.add(entry.gpuTimes[queryFrame]);
        entry.numTimes++;
      }
    }

    for (uint32_t i : m_data->singleSections) {
      Entry& entry = m_data->entries[i];
      uint32_t queryFrame = entry.subFrame;

      // query once
      bool available =
          entry.cpuTime.numValid == 0 &&
          (entry.api == nullptr ||
           entry.gpuTimeProvider(i, queryFrame, entry.gpuTimes[queryFrame]));

      if (available) {
        entry.cpuTime.add(entry.cpuTimes[queryFrame]);
        entry.gpuTime.add(entry.gpuTimes[queryFrame]);
        entry.numTimes++;
      }
    }

    m_data->cpuTime.add(m_data->cpuCurrentTime);
  }

  m_data->numFrames++;
}

void Profiler::grow(uint32_t newsize) {
  size_t oldsize = m_data->entries.size();

  if (oldsize == newsize) {
    return;
  }

  m_data->entries.resize(newsize);

  for (size_t i = oldsize; i < newsize; i++) {
    m_data->entries[i].cpuTime.init(m_data->numAveraging);
    m_data->entries[i].gpuTime.init(m_data->numAveraging);
  }
}

void Profiler::clear() {
  m_data->entries.clear();
  m_data->singleSections.clear();
}

void Profiler::reset(uint32_t delay) {
  m_data->resetDelay = delay;
}

static std::string format(const char* msg, ...) {
  std::size_t const STRING_BUFFER(8192);
  char text[STRING_BUFFER];
  va_list list;

  if (msg == 0)
    return std::string();

  va_start(list, msg);
#ifdef _WIN32
  vsprintf_s(text, msg, list);
#else  // #ifdef _WIN32
  vsprintf(text, msg, list);
#endif
  va_end(list);

  return std::string(text);
}

bool Profiler::getTimerInfo(uint32_t i, TimerInfo& info) {
  Entry& entry = m_data->entries[i];

  if (!entry.numTimes || entry.accumulated) {
    return false;
  }

  info.gpu.average = entry.gpuTime.getAveraged();
  info.cpu.average = entry.cpuTime.getAveraged();
  info.cpu.absMinValue = entry.cpuTime.absMinValue;
  info.cpu.absMaxValue = entry.cpuTime.absMaxValue;
  info.gpu.absMinValue = entry.gpuTime.absMinValue;
  info.gpu.absMaxValue = entry.gpuTime.absMaxValue;
  bool found = false;
  for (uint32_t n = i + 1; n < m_data->numLastSections; n++) {
    Entry& otherentry = m_data->entries[n];
    if (otherentry.name == entry.name && otherentry.level == entry.level &&
        otherentry.api == entry.api && !otherentry.accumulated) {
      found = true;
      info.gpu.average += otherentry.gpuTime.getAveraged();
      info.cpu.average += otherentry.cpuTime.getAveraged();
      info.cpu.absMinValue += entry.cpuTime.absMinValue;
      info.cpu.absMaxValue += entry.cpuTime.absMaxValue;
      info.gpu.absMinValue += entry.gpuTime.absMinValue;
      info.gpu.absMaxValue += entry.gpuTime.absMaxValue;
      otherentry.accumulated = true;
    }

    if (otherentry.splitter && otherentry.level <= entry.level)
      break;
  }

  info.accumulated = found;
  info.numAveraged = entry.cpuTime.numValid;

  return true;
}

bool Profiler::getTimerInfo(const char* name, TimerInfo& info) {
  if (name == nullptr) {
    info = TimerInfo();
    if (!m_data->cpuTime.numValid) {
      return false;
    }
    info.cpu.average = m_data->cpuTime.getAveraged();
    info.cpu.absMaxValue = m_data->cpuTime.absMaxValue;
    info.cpu.absMinValue = m_data->cpuTime.absMinValue;
    info.numAveraged = m_data->cpuTime.numValid;

    return true;
  }

  for (uint32_t i = 0; i < m_data->numLastSections; i++) {
    Entry& entry = m_data->entries[i];

    entry.accumulated = false;
  }

  for (uint32_t i = 0; i < (uint32_t)m_data->entries.size(); i++) {
    Entry& entry = m_data->entries[i];

    if (!entry.name)
      continue;

    if (strcmp(name, entry.name))
      continue;

    return getTimerInfo(i, info);
  }

  return false;
}

void Profiler::print(std::string& stats) {
  stats.clear();

  for (uint32_t i = 0; i < m_data->numLastSections; i++) {
    Entry& entry = m_data->entries[i];
    entry.accumulated = false;
  }

  printf("Timer null;\t N/A %6d; CPU %6d;\n", 0,
         (uint32_t)m_data->cpuTime.getAveraged());

  for (uint32_t i = 0; i < m_data->numLastSections; i++) {
    static const char* spaces = "        ";  // 8
    Entry& entry = m_data->entries[i];

    if (entry.level == LEVEL_SINGLESHOT)
      continue;

    uint32_t level = 7 - (entry.level > 7 ? 7 : entry.level);

    TimerInfo info;
    if (!getTimerInfo(i, info))
      continue;

    const char* gpuname = entry.api ? entry.api : "N/A";

    if (info.accumulated) {
      stats += format(
          "%sTimer %s;\t %s %6d; CPU %6d; (microseconds, accumulated loop)\n",
          &spaces[level], entry.name, gpuname, (uint32_t)(info.gpu.average),
          (uint32_t)(info.cpu.average));
    } else {
      stats += format(
          "%sTimer %s;\t %s %6d; CPU %6d; (microseconds, avg %d)\n",
          &spaces[level], entry.name, gpuname, (uint32_t)(info.gpu.average),
          (uint32_t)(info.cpu.average), (uint32_t)entry.cpuTime.numValid);
    }
  }
}

uint32_t Profiler::getTotalFrames() const {
  return m_data->numFrames;
}

void Profiler::accumulationSplit() {
  SectionID sec = getSectionID(false, nullptr);
  if (sec >= m_data->entries.size()) {
    grow((uint32_t)(m_data->entries.size() * 2));
  }

  m_data->entries[sec].level = m_data->level;
  m_data->entries[sec].splitter = true;
}

Profiler::SectionID Profiler::getSectionID(bool singleShot, const char* name) {
  uint32_t numEntries = (uint32_t)m_data->entries.size();

  if (singleShot) {
    // find empty slot or with same name
    for (uint32_t i = 0; i < numEntries; i++) {
      Entry& entry = m_data->entries[i];
      if (entry.name == name || entry.name == nullptr) {
        m_data->singleSections.push_back(i);
        return i;
      }
    }
    m_data->singleSections.push_back(numEntries);
    return numEntries;
  } else {
    // find non-single shot slot
    while (m_data->nextSection < numEntries &&
           m_data->entries[m_data->nextSection].level == LEVEL_SINGLESHOT) {
      m_data->nextSection++;
    }

    m_data->frameSections.push_back(m_data->nextSection);
    return m_data->nextSection++;
  }
}

Profiler::SectionID Profiler::beginSection(const char* name,
                                           const char* api,
                                           gpuTimeProvider_fn gpuTimeProvider,
                                           bool singleShot) {
  uint32_t subFrame = m_data->numFrames % FRAME_DELAY;
  SectionID sec = getSectionID(singleShot, name);

  if (sec >= m_data->entries.size()) {
    grow((uint32_t)(m_data->entries.size() * 2));
  }

  Entry& entry = m_data->entries[sec];
  uint32_t level = singleShot ? LEVEL_SINGLESHOT : (m_data->level++);

  if (entry.name != name || entry.api != api || entry.level != level) {
    entry.name = name;
    entry.api = api;

    if (!singleShot) {
      m_data->resetDelay = CONFIG_DELAY;
    }
  }

  entry.subFrame = subFrame;
  entry.level = level;
  entry.splitter = false;
  entry.gpuTimeProvider = gpuTimeProvider;

#ifdef NVP_SUPPORTS_NVTOOLSEXT
  {
    nvtxEventAttributes_t eventAttrib = {0};
    eventAttrib.version = NVTX_VERSION;
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    eventAttrib.colorType = NVTX_COLOR_ARGB;

    unsigned char color[4];
    color[0] = 255;
    color[1] = 0;
    color[2] = sec % 2 ? 127 : 255;
    color[3] = 255;

    color[2] -= level * 16;
    color[3] -= level * 16;

    eventAttrib.color = *(uint32_t*)(color);
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii = name;
    nvtxRangePushEx(&eventAttrib);
  }
#endif

  entry.cpuTimes[subFrame] = -getMicroSeconds();
  entry.gpuTimes[subFrame] = 0;

  if (singleShot) {
    entry.cpuTime.init(1);
    entry.gpuTime.init(1);
  }

  return sec;
}

void Profiler::endSection(SectionID sec) {
  Entry& entry = m_data->entries[sec];

  entry.cpuTimes[entry.subFrame] += getMicroSeconds();

#ifdef NVP_SUPPORTS_NVTOOLSEXT
  nvtxRangePop();
#endif

  if (entry.level != LEVEL_SINGLESHOT) {
    m_data->level--;
  }
}

Profiler::Clock::Clock() {
  m_init = std::chrono::high_resolution_clock::now();
}

double Profiler::Clock::getMicroSeconds() const {
  return double(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now() - m_init)
                    .count()) /
         double(1000);
}
}  // namespace nvh
