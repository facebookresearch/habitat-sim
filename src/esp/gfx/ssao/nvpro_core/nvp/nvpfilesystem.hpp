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
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "../nvh/nvprint.hpp"
#include "../nvh/threading.hpp"

namespace nvp {

/**
  \class nvp::FileSystemMonitor

  Monitors files and/or directories for changes

  This cross-platform wrapper does not create any threads, but is designed for
  it. checkEvents() will block until either an event is generated, or cancel()
  is called. See ModifiedFilesMonitor for an example.
*/
class FileSystemMonitor {
 public:
  /** File events.
   *
   * \note Visual studio saves files to temporary filenames and then swaps them
   * with renames. Renames/moves are not exposed explicitly yet.
   */
  enum Event {
    FSM_CREATE = (1 << 0),
    FSM_DELETE = (1 << 1),
    FSM_MODIFY = (1 << 2),
  };

  struct EventData {
    Event event;
    const std::string path;
    void* userPtr;
  };

  using CallbackType = void(const EventData&);
  using Callback = std::function<CallbackType>;
  using PathID = int;

  static const PathID INVALID_PATH_ID = 0;

  static FileSystemMonitor* create();
  static void destroy(FileSystemMonitor* monitor);

  /** Add a file or directory to be monitored for events
   *
   *  It is not safe to call add() at the same time as checkEvents(). Call
   * cancel() first.
   *
   * \return PathID that can be passed to remove() on success, 0 on failure.
   */
  virtual PathID add(const std::string& path,
                     uint32_t eventMask,
                     void* userPtr = nullptr) = 0;

  /** Remove a file or directory from being monitored
   *
   *  It is not safe to call remove() at the same time as checkEvents(). Call
   * cancel() first. */
  virtual void remove(const PathID& pathID) = 0;

  /** Process the event queue, blocking until there is another event or cancel()
   * is called.
   *
   *  \return True if the thread should keep looping, i.e. there are no errors
   * and cancel() has not been called. It is expected this is called in a loop
   * by an event thread.
   */
  virtual bool checkEvents(const Callback& callback) = 0;

  /** Abort checkEvents, if it is currently being called. */
  virtual void cancel() = 0;

 protected:
  FileSystemMonitor() = default;
  virtual ~FileSystemMonitor() = default;
  PathID nextPathID() {
    while (++m_nextPathID == INVALID_PATH_ID)
      ;
    return m_nextPathID;
  }

 private:
  PathID m_nextPathID = 0;
};

/**
  \class nvp::FSMRunner

  Adds a thread to nvp::FileSystemMonitor that repeatedly calls
  nvp::FileSystemMonitor::checkEvents().
*/
class FSMRunner {
 public:
  FSMRunner(const FileSystemMonitor::Callback& callback)
      : m_callback(callback) {
    m_monitor = FileSystemMonitor::create();
  }
  ~FSMRunner() {
    stop();
    FileSystemMonitor::destroy(m_monitor);
  }

 protected:
  void start() {
    assert(!m_thread.joinable());
    m_thread = std::thread(&FSMRunner::entrypoint, this);
  }
  void stop() {
    if (m_thread.joinable()) {
      m_monitor->cancel();
      m_thread.join();
    }
  }

  FileSystemMonitor* m_monitor;

 private:
  void entrypoint() {
    for (;;) {
      if (!m_monitor->checkEvents(m_callback))
        break;
    }
  }

  std::thread m_thread;
  FileSystemMonitor::Callback m_callback;
};

/**
  \class nvp::FSMCallbacks

  Utility class to get per-path callbacks.

  Make sure PathSetCallback objects returned by add() do not outlive the
  FSMCallbacks. Be careful not to destroy a PathCallback during a callback.

  Example:
  \code
  FSMCallbacks callbacks;

  auto callbackFile1 = callbacks.add(std::vector<std::string>{"file1.txt"},
  nvp::FileSystemMonitor::FSM_MODIFY, [this](nvp::FileSystemMonitor::EventData
  ev) {
                                        // Do something with file1.txt
                                      });

  auto callbackFile2 = callbacks.add(std::vector<std::string>{"file2.txt"},
  nvp::FileSystemMonitor::FSM_MODIFY, [this](nvp::FileSystemMonitor::EventData
  ev) {
                                        // Do something with file2.txt
                                      });

  // When callbackFile1 goes out of scope, file1.txt stops being monitored
  callbackFile1.reset()
  \endcode
*/
class FSMCallbacks : public FSMRunner {
 public:
  using Clock = nvh::DefaultDelayClock;
  using Duration = nvh::DefaultDelayDuration;

  struct PathSetCallbackData {
    FSMCallbacks& owner;
    std::vector<FileSystemMonitor::PathID> ids;
    FileSystemMonitor::Callback callback;
    const Duration consolidateDelay;
    nvh::delayed_call<Clock, Duration> delayedCall;
    ~PathSetCallbackData() {
      if (!ids.empty()) {
        std::lock_guard<std::mutex> lock(owner.m_monitorMutex);
        owner.stop();
        for (const auto& id : ids)
          owner.m_monitor->remove(id);
        owner.start();
      }
    }
  };

  using PathSetCallback = std::shared_ptr<PathSetCallbackData>;

  FSMCallbacks() : FSMRunner(&FSMCallbacks::callback) {}

  template <class List>
  PathSetCallback add(const List& pathList,
                      uint32_t eventMask,
                      const FileSystemMonitor::Callback& callback,
                      const Duration& consolidateDelay = Duration::zero()) {
    std::lock_guard<std::mutex> lock(m_monitorMutex);
    stop();

    PathSetCallback pathSetCallback{
        new PathSetCallbackData{*this, {}, callback, consolidateDelay}};
    for (const auto& path : pathList) {
      auto pathID = m_monitor->add(path, eventMask, pathSetCallback.get());
      if (pathID == FileSystemMonitor::INVALID_PATH_ID) {
        LOGE("Failed to watch '%s' for changes\n", path.c_str());
      } else {
        pathSetCallback->ids.push_back(pathID);
      }
    }

    start();
    return pathSetCallback;
  }

 private:
  static void callback(const nvp::FileSystemMonitor::EventData& ev) {
    auto cbData = reinterpret_cast<PathSetCallbackData*>(ev.userPtr);

    if (cbData->consolidateDelay != Duration::zero()) {
      // This may block if the previous delayed call started and is still
      // running. Rather than do anything clever here, it is left to the user to
      // layer on functionality and make sure this callback loop is not blocked.
      if (!cbData->delayedCall.delay_for(cbData->consolidateDelay))
        cbData->delayedCall = nvh::delay_noreturn_for<Clock>(
            cbData->consolidateDelay, cbData->callback, ev);
    } else
      cbData->callback(ev);
  }

  // Protect against various threads racing FSMRunner::stop()/start() calls
  // during add() and ~PathSetCallbackData().
  std::mutex m_monitorMutex;
};

/**
  \class nvp::ModifiedFilesMonitor

  Monitors files and/or directories for changes.

  Starts a thread at creation. Be careful to keep it in scope while events are
  needed.

  This cross-platform wrapper does not create any threads, but is designed to be
  used with one. checkEvents() will block until either an event is generated, or
  cancel() is called.

  Example:
  \code
  std::vector<std::string> dirs = {"shaders_bin"};
  nvp::FileSystemMonitor::Callback callback =
  [](nvp::FileSystemMonitor::EventData ev){ g_reloadShaders = true;
  };
  auto fileMonitor = std::make_unique<nvp::ModifiedFilesMonitor>(dirs,
  callback); \endcode
*/
class ModifiedFilesMonitor : public FSMRunner {
 public:
  template <class List>
  ModifiedFilesMonitor(const List& paths,
                       const FileSystemMonitor::Callback& callback)
      : FSMRunner(callback) {
    for (const auto& path : paths) {
      if (m_monitor->add(path, FileSystemMonitor::FSM_MODIFY |
                                   FileSystemMonitor::FSM_CREATE) ==
          FileSystemMonitor::INVALID_PATH_ID) {
        LOGE("Failed to watch '%s' for changes\n", path.c_str());
      }
    }
    start();
  }
};

}  // namespace nvp
