/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
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

#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

namespace nvh {

using DefaultDelayClock = std::chrono::steady_clock;
using DefaultDelayDuration = std::chrono::nanoseconds;

/**
 * \class nvh::delayed_call class returned by delay_noreturn_for to track the
 * thread created and possibly reset the delay timer.
 */
template <class Clock = DefaultDelayClock,
          class Duration = std::chrono::duration<double>>
class delayed_call {
  template <class ClockT, class DurationT, class Function, class... Args>
  friend delayed_call<ClockT, DurationT> delay_noreturn_for(
      const DurationT& sleep_duration,
      Function&& f,
      Args&&... args);

 public:
  /** Update the thread to make the call sleep_duration from now
   *
   * \return True if the delay was updated before the callback was called. False
   * otherwise.
   */
  bool delay_for(const Duration& sleep_duration) {
    bool result = false;
    if (m_delay) {
      std::lock_guard<std::mutex> lock(m_delay->mutex);
      if (!m_delay->started) {
        auto prevUntil = m_delay->until;
        m_delay->until = Clock::now() + sleep_duration;

        // No need to wake up the other thread if the delay is longer. It'll
        // keep looping while dirty is set.
        if (prevUntil < m_delay->until)
          m_delay->dirty = true;
        else
          m_delay->cv.notify_all();
      }
      result = !m_delay->started;
    }
    return result;
  }

  /** Cancel a delayed call
   *
   * \return True if the call was cancelled before running. False otherwise.
   */
  bool cancel() {
    bool result = false;
    if (m_delay) {
      std::lock_guard<std::mutex> lock(m_delay->mutex);
      if (!m_delay->started) {
        m_delay->cancelled = true;
        m_delay->cv.notify_all();
      }
      result = !m_delay->started;
    }
    return result;
  }

  delayed_call() = default;
  delayed_call(delayed_call&& other) { *this = std::move(other); }
  ~delayed_call() = default;

  delayed_call& operator=(delayed_call&& other) {
    m_delay = std::move(other.m_delay);
    return *this;
  }

  // This class is movable only
  delayed_call(const delayed_call& other) = delete;
  delayed_call& operator=(const delayed_call& other) = delete;

 private:
  struct DelayData {
    std::chrono::time_point<Clock, Duration> until;
    std::thread thread;
    std::mutex mutex;
    std::condition_variable cv;
    bool dirty = true;
    bool cancelled = false;
    bool started = false;

    ~DelayData() {
      if (thread.joinable())
        thread.join();
    }
  };

  template <class Function, class... Args>
  static void delayEntry(DelayData* delay, Function&& f, Args&&... args) {
    {
      std::unique_lock<std::mutex> lock(delay->mutex);
      std::cv_status status = std::cv_status::no_timeout;
      while (!delay->cancelled &&
             (delay->dirty || status == std::cv_status::no_timeout)) {
        delay->dirty = false;
        status = delay->cv.wait_until(lock, delay->until);
      }
      if (delay->cancelled)
        return;
      delay->started = true;
    }

    // Ignore the return value. Need to keep a std::future object if not.
    (void)f(std::forward<Args>(args)...);
  }

  std::unique_ptr<DelayData> m_delay;

  template <class Function, class... Args>
  delayed_call(const Duration& sleep_duration, Function&& f, Args&&... args)
      : m_delay(std::make_unique<DelayData>()) {
    m_delay->until = Clock::now() + sleep_duration;
    m_delay->thread = std::thread(
        delayed_call::delayEntry<std::remove_reference_t<Function>&&,
                                 std::remove_reference_t<Args>&&...>,
        m_delay.get(), std::forward<Function>(f), std::forward<Args>(args)...);
  }
};

/**
 * Delay a call to a void function for sleep_duration.
 *
 * \return A delayed_call object that holds the running thread.
 *
 * Example:
 * \code
 * // Create or update a delayed call to callback. Useful to consolidate
 * multiple events into one call. if(!m_delayedCall.delay_for(delay))
 *   m_delayedCall = nvh::delay_noreturn_for(delay, callback);
 * \endcode
 */
template <class Clock = DefaultDelayClock,
          class Duration = DefaultDelayDuration,
          class Function,
          class... Args>
delayed_call<Clock, Duration> delay_noreturn_for(const Duration& sleep_duration,
                                                 Function&& f,
                                                 Args&&... args) {
  return delayed_call<Clock, Duration>(
      sleep_duration, std::forward<Function>(f), std::forward<Args>(args)...);
}

}  // namespace nvh
