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

#ifndef NV_PROFILER_INCLUDED
#define NV_PROFILER_INCLUDED

#include <float.h>  // DBL_MAX
#include <stdint.h>
#include <stdio.h>
#include <string.h>  //memset
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#ifdef NVP_SUPPORTS_NVTOOLSEXT
#define NVTX_STDINT_TYPES_ALREADY_DEFINED
#include "../nvtx3/nvToolsExt.h"
#endif

namespace nvh {

//////////////////////////////////////////////////////////////////////////
/**
    \class nvh::Profiler

    \brief The nvh::Profiler class is designed to measure timed sections.

    Each section has a cpu and gpu time. Gpu times are typically provided
    by derived classes for each individual api (e.g. OpenGL, Vulkan etc.).

    There is functionality to pretty print the sections with their nesting
   level. Multiple profilers can reference the same database, so one profiler
    can serve as master that they others contribute to. Typically the
    base class measuring only CPU time could be the master, and the api
    derived classes reference it to share the same database.

    Profiler::Clock can be used standalone for time measuring.
  */

class Profiler {
 public:
  /// if we detect a change in timers (api/name change we trigger a reset after
  /// that amount of frames)
  static const uint32_t CONFIG_DELAY = 8;
  /// gpu times are queried after that amount of frames
  static const uint32_t FRAME_DELAY = 4;
  /// by default we start with space for that many begin/end sections per-frame
  static const uint32_t START_SECTIONS = 64;
  /// cyclic window for averaging
  static const uint32_t MAX_NUM_AVERAGE = 128;

 public:
  typedef uint32_t SectionID;
  typedef uint32_t OnceID;

  class Clock {
    // generic utility class for measuring time
    // uses high resolution timer provided by OS
   public:
    Clock();
    double getMicroSeconds() const;

   private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_init;
  };

  //////////////////////////////////////////////////////////////////////////

  // utility class for automatic calling of begin/end within a local scope
  class Section {
   public:
    Section(Profiler& profiler, const char* name, bool singleShot = false)
        : m_profiler(profiler) {
      m_id = profiler.beginSection(name, nullptr, nullptr, singleShot);
    }
    ~Section() { m_profiler.endSection(m_id); }

   private:
    SectionID m_id;
    Profiler& m_profiler;
  };

  // recurring, must be within beginFrame/endFrame
  Section timeRecurring(const char* name) {
    return Section(*this, name, false);
  }

  // single shot, results are available after FRAME_DELAY many endFrame
  Section timeSingle(const char* name) { return Section(*this, name, true); }

  //////////////////////////////////////////////////////////////////////////

  // num <= MAX_NUM_AVERAGE
  void setAveragingSize(uint32_t num);

  //////////////////////////////////////////////////////////////////////////

  // gpu times for a section are queried at "endFrame" with the use of this
  // optional function. It returns true if the queried result was available, and
  // writes the microseconds into gpuTime.
  typedef std::function<bool(SectionID, uint32_t subFrame, double& gpuTime)>
      gpuTimeProvider_fn;

  // must be called every frame
  void beginFrame();
  void endFrame();

  // there are two types of sections
  //  singleShot = true, means the timer can exist outside begin/endFrame and is
  //  non-recurring
  //                     results of previous singleShot with same name will be
  //                     overwritten.
  // singleShot = false, sections can be nested, but must be within
  // begin/endFrame
  //

  SectionID beginSection(const char* name,
                         const char* api = nullptr,
                         gpuTimeProvider_fn gpuTimeProvider = nullptr,
                         bool singleShot = false);
  void endSection(SectionID slot);

  // When a section is used within a loop (same nesting level), and the the same
  // arguments for name and api are passed, we normally average the results of
  // those sections together when printing the stats or using the getAveraged
  // functions below. Calling the splitter (outside of a section) means we
  // insert a split point that the averaging will not pass.
  void accumulationSplit();

  inline double getMicroSeconds() const { return m_clock.getMicroSeconds(); }

  //////////////////////////////////////////////////////////////////////////

  // resets all stats
  void clear();

  // resets recurring sections
  // in case averaging should be reset after a few frames (warm-up cache, hide
  // early heavier frames after configuration changes) implicit resets are
  // triggered if the frame's configuration of timer section changes compared to
  // previous frame.
  void reset(uint32_t delay = CONFIG_DELAY);

  // pretty print current averaged timers
  void print(std::string& stats);

  // returns number of frames since reset
  uint32_t getTotalFrames() const;

  struct TimerStats {
    // time in microseconds
    double average = 0;
    double absMinValue = DBL_MAX;
    double absMaxValue = 0;
  };

  struct TimerInfo {
    // number of averaged values, <= MAX_NUM_AVERAGE
    uint32_t numAveraged = 0;

    // accumulation happens for example in loops:
    //   for (..) { auto scopeTimer = timeSection("blah"); ... }
    // then the reported values are the accumulated sum of all those timers.
    bool accumulated = false;

    TimerStats cpu;
    TimerStats gpu;
  };

  // query functions for current gathered cyclic averages ( <= MAX_NUM_AVERAGE)
  // use nullptr name to get the cpu timing of the outermost scope
  // (beginFrame/endFrame) returns true if found timer and it had valid values
  bool getTimerInfo(const char* name, TimerInfo& info);

  // simplified wrapper
  bool getAveragedValues(const char* name, double& cpuTime, double& gpuTime) {
    TimerInfo info;

    if (getTimerInfo(name, info)) {
      cpuTime = info.cpu.average;
      gpuTime = info.gpu.average;
      return true;
    } else {
      cpuTime = 0;
      gpuTime = 0;
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////////

  // if a master is provided we use its database
  // otherwise our own
  Profiler(Profiler* master = nullptr);

  Profiler(uint32_t startSections);

 protected:
  //////////////////////////////////////////////////////////////////////////

  // Utility functions for derived classes that provide gpu times.
  // We assume most apis use a big pool of api-specific events/timers,
  // the functions below help manage such pool.

  inline uint32_t getSubFrame(SectionID slot) const {
    return m_data->entries[slot].subFrame;
  }
  inline uint32_t getRequiredTimers() const {
    return (uint32_t)(m_data->entries.size() * FRAME_DELAY * 2);
  }

  static inline uint32_t getTimerIdx(SectionID slot,
                                     uint32_t subFrame,
                                     bool begin) {
    // must not change order of begin/end
    return ((slot * FRAME_DELAY) + subFrame) * 2 + (begin ? 0 : 1);
  }

  inline bool isSectionRecurring(SectionID slot) const {
    return m_data->entries[slot].level != LEVEL_SINGLESHOT;
  }

 private:
  //////////////////////////////////////////////////////////////////////////

  static const uint32_t LEVEL_SINGLESHOT = ~0;

  struct TimeValues {
    double times[MAX_NUM_AVERAGE] = {0};
    double valueTotal = 0;
    double absMinValue = DBL_MAX;
    double absMaxValue = 0;

    uint32_t index = 0;
    uint32_t numCycle = MAX_NUM_AVERAGE;
    uint32_t numValid = 0;

    TimeValues(uint32_t cycleSize = MAX_NUM_AVERAGE) { init(cycleSize); }

    void init(uint32_t cycleSize) {
      numCycle = std::min(cycleSize, MAX_NUM_AVERAGE);
      reset();
    }

    void reset() {
      valueTotal = 0;
      absMinValue = DBL_MAX;
      absMaxValue = 0;
      index = 0;
      numValid = 0;
      memset(times, 0, sizeof(times));
    }

    void add(double time) {
      valueTotal += time - times[index];
      times[index] = time;

      index = (index + 1) % numCycle;
      numValid = std::min(numValid + 1, numCycle);

      absMinValue = std::min(time, absMinValue);
      absMaxValue = std::max(time, absMaxValue);
    }

    double getAveraged() {
      if (numValid) {
        return valueTotal / double(numValid);
      } else {
        return 0;
      }
    }
  };

  struct Entry {
    const char* name = nullptr;
    const char* api = nullptr;
    gpuTimeProvider_fn gpuTimeProvider = nullptr;

    // level == ~0 used for "singleShot"
    uint32_t level = 0;
    uint32_t subFrame = 0;

#ifdef NVP_SUPPORTS_NVTOOLSEXT
    nvtxRangeId_t m_nvrange;
#endif
    double cpuTimes[FRAME_DELAY] = {0};
    double gpuTimes[FRAME_DELAY] = {0};

    // number of times summed since last reset
    uint32_t numTimes = 0;

    TimeValues gpuTime;
    TimeValues cpuTime;

    // splitter is used to prevent accumulated case below
    // when same depth level is used
    // {section("BLAH"); ... }
    // splitter
    // {section("BLAH"); ...}
    // now the result of "BLAH" is not accumulated

    bool splitter = false;

    // if the same timer name is used within a loop (same
    // depth level), e.g.:
    //
    // for () { section("BLAH"); ... }
    //
    // we accumulate the timing values of all of them

    bool accumulated = false;
  };

  struct Data {
    uint32_t numAveraging = MAX_NUM_AVERAGE;
    uint32_t resetDelay = 0;
    uint32_t numFrames = 0;

    uint32_t level = 0;
    uint32_t nextSection = 0;

    uint32_t numLastSections = 0;
    uint32_t numLastEntries = 0;

    std::vector<uint32_t> frameSections;
    std::vector<uint32_t> singleSections;

    double cpuCurrentTime = 0;
    TimeValues cpuTime;

    std::vector<Entry> entries;
  };

  std::shared_ptr<Data> m_data = nullptr;
  Clock m_clock;

  SectionID getSectionID(bool singleShot, const char* name);

  bool getTimerInfo(uint32_t i, TimerInfo& info);
  void grow(uint32_t newsize);
};
}  // namespace nvh

#endif
