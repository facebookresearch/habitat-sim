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

#include "nvpfilesystem.hpp"

#include <string.h>
#include <array>
#include <cassert>
#include <unordered_map>

using namespace nvp;

#if defined(_WIN32)
#include <Windows.h>
#include <filesystem>
#include <locale>

void logLastWindowError(std::string context) {
  LPVOID messageBuffer;
  DWORD dw = GetLastError();
  DWORD numChars = FormatMessage(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
          FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL, dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR)&messageBuffer, 0, NULL);
  assert(numChars);

#ifndef UNICODE
  std::string errorStr((char*)messageBuffer);
#else
#error Not implemented
#endif
  LOGE("%s, error %lu: %s\n", context.c_str(), dw, errorStr.c_str());

  LocalFree(messageBuffer);
}

struct PathKey {
  std::string path;
  uint32_t eventMask;
  bool operator==(const PathKey& other) const {
    return path == other.path && eventMask == other.eventMask;
  }
};

template <class A, class B>
size_t combine_hash(const A& a, const B& b) {
  return std::hash<A>()(a) ^ (std::hash<B>()(b) << 1);
}
template <class A, class B, class C>
size_t combine_hash(const A& a, const B& b, const C& c) {
  return (combine_hash(a, b) >> 1) ^ std::hash<C>()(c);
}

namespace std {

template <>
struct hash<PathKey> {
  std::size_t operator()(const PathKey& k) const {
    return ::combine_hash(k.path, k.eventMask);
  }
};

}  // namespace std

// Converts a UTF-8 string to a UTF-16 string. Avoids using <codecvt>, due to
// https://github.com/microsoft/STL/issues/443.
std::wstring utf8ToWideString(std::string utf8String) {
  if (utf8String.size() > std::numeric_limits<int>::max()) {
    assert(!"Too many characters for UTF8-to-UTF16 API!");
    return L"";
  }
  const int utf8Bytes = static_cast<int>(utf8String.size());
  const int utf16Characters =
      MultiByteToWideChar(CP_UTF8, 0, utf8String.data(), utf8Bytes, nullptr, 0);
  if (utf16Characters < 0) {
    assert(!"Error counting UTF-16 characters!");
    return L"";
  }
  std::wstring result(utf16Characters, 0);
  (void)MultiByteToWideChar(CP_UTF8, 0, utf8String.data(), utf8Bytes,
                            result.data(), utf16Characters);
  return result;
}

// Converts a UTF-16 string to a UTF-8 string. Avoids using <codecvt>, due to
// https://github.com/microsoft/STL/issues/443.
std::string wideToUTF8String(std::wstring utf16String) {
  if (utf16String.size() > std::numeric_limits<int>::max()) {
    assert(!"Too many characters for UTF16-to-UTF8 API!");
    return "";
  }
  const int utf16Characters = static_cast<int>(utf16String.size());
  const int utf8Bytes =
      WideCharToMultiByte(CP_UTF8, 0, utf16String.data(), utf16Characters,
                          nullptr, 0, nullptr, nullptr);
  if (utf8Bytes < 0) {
    assert(!"Error counting UTF-8 bytes!");
    return "";
  }
  std::string result(utf8Bytes, 0);
  (void)WideCharToMultiByte(CP_UTF8, 0, utf16String.data(), utf16Characters,
                            result.data(), utf8Bytes, nullptr, nullptr);
  return result;
}

struct PathInstance {
  FileSystemMonitor::PathID id;
  void* userPtr;

  bool operator==(const FileSystemMonitor::PathID& pathID) const {
    return id == pathID;
  }
};

/** Class to handle receiving per-directory filesystem events
 *
 * This class is reused to provide per-file events and distribute events to
 * multiple listeners. Each WindowsPathMonitor can only be a file monitor
 * (filtered events) or a directory monitor (all events), not both.
 *
 * Structure:
 * - WindowsPathMonitor - monitored directory, receiving OS events
 *   - Instances (PathID + userData) for multiple directory listeners of
 * per-directory monitoring
 *   - Sub-paths for per-file monitoring
 *     - Instances (PathID + userData) for multiple file listeners
 */
struct WindowsPathMonitor : PathKey {
  WindowsPathMonitor(PathKey key)
      : PathKey{key}, m_overlapped{}, m_eventsRequested{false} {
    // Translate the event mask
    m_winEventFilter = 0;
    if (eventMask &
        (FileSystemMonitor::FSM_CREATE | FileSystemMonitor::FSM_DELETE))
      m_winEventFilter |=
          FILE_NOTIFY_CHANGE_CREATION | FILE_NOTIFY_CHANGE_FILE_NAME;
    if (eventMask & FileSystemMonitor::FSM_MODIFY)
      m_winEventFilter |= FILE_NOTIFY_CHANGE_LAST_WRITE;

    // Open the path to receive events from it
    std::wstring pathW = utf8ToWideString(path);
    DWORD shareMode = FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE;
    DWORD flags = FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OVERLAPPED;
    m_dirHandle = CreateFileW(pathW.c_str(), GENERIC_READ, shareMode, NULL,
                              OPEN_EXISTING, flags, NULL);
    if (m_dirHandle == INVALID_HANDLE_VALUE)
      logLastWindowError(
          std::string("FileSystemMonitor: Error in CreateFileW for path ") +
          path);
  }

  ~WindowsPathMonitor() {
    if (m_eventsRequested)
      cancelAsync();

    if (m_dirHandle != INVALID_HANDLE_VALUE)
      CloseHandle(m_dirHandle);
  }

  bool getEventsAsync() {
    m_eventsRequested = true;
    BOOL result = ReadDirectoryChangesExW(
        m_dirHandle, m_eventBuffer.data(), (DWORD)m_eventBuffer.size(), TRUE,
        m_winEventFilter, NULL, &m_overlapped, NULL,
        ReadDirectoryNotifyExtendedInformation);
    if (result == FALSE)
      logLastWindowError(
          std::string(
              "FileSystemMonitor: Error in ReadDirectoryChangesW for path ") +
          path);
    return result == TRUE;
  }

  void cancelAsync() {
    assert(m_eventsRequested);
    m_eventsRequested = false;
    CancelIoEx(m_dirHandle, &m_overlapped);
  }

  // Avoid throwing an exception in the constructor with an is-valid method.
  bool isValid() { return m_dirHandle != INVALID_HANDLE_VALUE; }

  HANDLE m_dirHandle;
  DWORD m_winEventFilter;
  std::array<uint8_t, 63 * 1024> m_eventBuffer;
  OVERLAPPED m_overlapped;
  bool m_eventsRequested;
  std::unordered_map<std::string, std::vector<PathInstance>> m_fileInstances;
  std::vector<PathInstance> m_directoryInstances;
};

class FileSystemMonitorWindows : public FileSystemMonitor {
  friend class FileSystemMonitor;

  virtual PathID add(const std::string& path,
                     uint32_t eventMask,
                     void* userPtr) override {
    std::string monitorDir{path};

    // Workaround monitoring individual files by monitoring the entire directory
    std::filesystem::path pathObj{path};
    bool isDir = std::filesystem::is_directory(pathObj);
    if (!isDir) {
      monitorDir = pathObj.parent_path().string();
    }

    // Track the path if it isn't tracked already
    WindowsPathMonitor* monitor;
    auto key = PathKey{monitorDir, eventMask};
    auto it = m_paths.find(key);
    if (it != m_paths.end()) {
      monitor = it->second.get();
    } else {
      auto pathMonitor = std::make_unique<WindowsPathMonitor>(key);
      if (!pathMonitor->isValid())
        return false;

      // Get async events for this path on the main port
      ULONG_PTR objectPtr = reinterpret_cast<ULONG_PTR>(pathMonitor.get());
      if (CreateIoCompletionPort(pathMonitor->m_dirHandle, m_ioCompletionPort,
                                 objectPtr, 1) != m_ioCompletionPort) {
        logLastWindowError("CreateIoCompletionPort in path monitoring");
        return false;
      }

      if (!pathMonitor->getEventsAsync())
        return false;

      monitor = pathMonitor.get();
      m_paths[key] = std::move(pathMonitor);
    }

    auto id = nextPathID();
    PathInstance instance{id, userPtr};

    if (isDir)
      monitor->m_directoryInstances.push_back(instance);
    else
      monitor->m_fileInstances[pathObj.filename().string()].push_back(instance);

    m_idToMonitor[id] = monitor;
    return id;
  }

  virtual void remove(const PathID& pathID) override {
    auto& monitor = *m_idToMonitor[pathID];
    bool isDirMonitor = monitor.m_fileInstances.empty();
    bool instancesEmpty;
    // TODO: lots of slow linear searching here, although both monitoring many
    // files and remove() is expected to be uncommon.
    if (isDirMonitor) {
      // Find and remove the pathID instance in m_directoryInstances.
      auto it = std::find(monitor.m_directoryInstances.begin(),
                          monitor.m_directoryInstances.end(), pathID);
      monitor.m_directoryInstances.erase(it);
      instancesEmpty = monitor.m_directoryInstances.empty();
    } else {
      // Find and remove the pathID instance in m_fileInstances. It could be
      // inside any one of the monitored files (sub-paths).
      for (auto& file : monitor.m_fileInstances) {
        auto& instances = file.second;
        auto it = std::find(instances.begin(), instances.end(), pathID);
        if (it == instances.end())
          continue;
        instances.erase(it);
        if (instances.empty())
          monitor.m_fileInstances.erase(file.first);
        break;
      }
      instancesEmpty = monitor.m_fileInstances.empty();
    }

    // If there are no instances left, remove the monitor.
    if (instancesEmpty)
      m_paths.erase((PathKey)monitor);

    m_idToMonitor.erase(pathID);
  }

  void handleEvent(WindowsPathMonitor* pathMonitor,
                   const FILE_NOTIFY_EXTENDED_INFORMATION* notifyInfo,
                   const Callback& callback) {
    std::wstring subPathW(notifyInfo->FileName,
                          notifyInfo->FileNameLength / sizeof(WCHAR));
    std::string subPath = wideToUTF8String(subPathW);

    bool isDirMonitor = pathMonitor->m_fileInstances.empty();
    std::vector<PathInstance>* instances;
    if (!isDirMonitor) {
      // Filter out files not monitored in this directoriy
      auto it = pathMonitor->m_fileInstances.find(subPath);
      if (it == pathMonitor->m_fileInstances.end()) {
        return;
      }
      instances = &it->second;
    } else
      instances = &pathMonitor->m_directoryInstances;

    std::filesystem::path fullPath = pathMonitor->path;
    fullPath /= subPath;  // "/" adds the paths

    LOGI("FileSystemMonitor %p event (mask %lx) for '%s'\n", m_ioCompletionPort,
         notifyInfo->Action, fullPath.string().c_str());

    for (auto& instance : *instances) {
      switch (notifyInfo->Action) {
        case FILE_ACTION_ADDED:
        case FILE_ACTION_RENAMED_NEW_NAME:
          if (pathMonitor->eventMask & FSM_CREATE)
            callback(
                EventData{FSM_CREATE, fullPath.string(), instance.userPtr});
          break;
        case FILE_ACTION_REMOVED:
        case FILE_ACTION_RENAMED_OLD_NAME:
          if (pathMonitor->eventMask & FSM_DELETE)
            callback(
                EventData{FSM_DELETE, fullPath.string(), instance.userPtr});
          break;
        case FILE_ACTION_MODIFIED:
          if (pathMonitor->eventMask & FSM_MODIFY)
            callback(
                EventData{FSM_MODIFY, fullPath.string(), instance.userPtr});
          break;
        default:
          // TODO: don't throw away free information
          break;
      }
    }
  }

  virtual bool checkEvents(const Callback& callback) override {
    DWORD bytesTransferred = 0;  // FILE_NOTIFY_EXTENDED_INFORMATION struct size
    ULONG_PTR userObject = 0;
    OVERLAPPED* overlapped = NULL;
    if (GetQueuedCompletionStatus(m_ioCompletionPort, &bytesTransferred,
                                  &userObject, &overlapped,
                                  INFINITE) == FALSE) {
      return true;
    }

    // Handle the case that cancel() unblocked the call
    if (reinterpret_cast<ULONG_PTR>(this) == userObject) {
      bool cancelling = m_cancelling;
      m_cancelling = false;
      return !cancelling;
    }

    auto pathMonitor = reinterpret_cast<WindowsPathMonitor*>(userObject);
    const auto& buffer = pathMonitor->m_eventBuffer;

    size_t offset = 0;
    for (;;) {
      auto notifyInfo =
          reinterpret_cast<const FILE_NOTIFY_EXTENDED_INFORMATION*>(
              buffer.data() + offset);
      handleEvent(pathMonitor, notifyInfo, callback);

      // Re-arm the event
      if (!pathMonitor->getEventsAsync()) {
        return false;
      }

      if (!notifyInfo->NextEntryOffset)
        break;

      offset += notifyInfo->NextEntryOffset;
    }

    return true;
  }

  virtual void cancel() override {
    ULONG_PTR objectPtr = reinterpret_cast<ULONG_PTR>(this);
    m_cancelling = true;
    PostQueuedCompletionStatus(m_ioCompletionPort, 0, objectPtr, NULL);
  }

  FileSystemMonitorWindows() {
    m_ioCompletionPort =
        CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 1);
    assert(m_ioCompletionPort != NULL);
  }

  virtual ~FileSystemMonitorWindows() override {
    assert(!m_cancelling);
    CloseHandle(m_ioCompletionPort);
  }

  HANDLE m_ioCompletionPort;
  bool m_cancelling = false;

  std::unordered_map<PathKey, std::unique_ptr<WindowsPathMonitor>> m_paths;

  // Reuse directory monitors that are created to monitor specific files
  std::unordered_map<PathID, WindowsPathMonitor*> m_idToMonitor;
};
#endif

#if defined(LINUX)
#include <linux/limits.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/inotify.h>
#include <unistd.h>

/** Object to track monitored paths
 *
 * inotify will not create multiple watches if the same path is added twice.
 * To handle multiple FileSystemMonitor clients, this is faked with a list of
 * instances per path.
 */
struct InotifyPath {
  /** File or directory name */
  std::string path;

  /** Union of events that all instances request */
  uint32_t inotifyMaskAll;

  /** Per-instance structure requesting events for this path */
  struct MonitorInstance {
    void* userPtr;
    uint32_t inotifyMask;
  };

  std::unordered_map<FileSystemMonitor::PathID, MonitorInstance> instances;
};

class FileSystemMonitorInotify : public FileSystemMonitor {
  friend class FileSystemMonitor;

  virtual int add(const std::string& path,
                  uint32_t eventMask,
                  void* userPtr) override {
    // Translate the event mask
    uint32_t inotifyMask = 0;
    if (eventMask & FSM_CREATE)
      inotifyMask |= IN_CREATE;
    if (eventMask & FSM_MODIFY)
      inotifyMask |= IN_MODIFY;
    if (eventMask & FSM_DELETE)
      inotifyMask |= IN_DELETE;

    assert(inotifyMask);
    if (!inotifyMask)
      return INVALID_PATH_ID;

    int watchDescriptor =
        inotify_add_watch(m_inotifyFd, path.c_str(), inotifyMask);
    if (watchDescriptor == -1)
      return INVALID_PATH_ID;

    auto id = nextPathID();
    // LOGI("FileSystemMonitor %i added %i %s\n", m_inotifyFd, id,
    // path.c_str());

    auto it = m_paths.find(watchDescriptor);
    if (it != m_paths.end()) {
      // Path has already been added before. May need to combine the mask and
      // re-add.
      auto allBits = it->second.inotifyMaskAll | inotifyMask;
      if (allBits != inotifyMask)
        (void)inotify_add_watch(m_inotifyFd, path.c_str(), allBits);
      it->second.inotifyMaskAll = allBits;

      it->second.instances[id] = {userPtr, inotifyMask};
    } else {
      m_paths[watchDescriptor] = InotifyPath{
          path,
          inotifyMask,
          {{id, InotifyPath::MonitorInstance{userPtr, inotifyMask}}}};
    }
    m_idToWatchDescriptor[id] = watchDescriptor;
    return id;
  }

  virtual void remove(const PathID& pathID) override {
    int watchDescriptor = m_idToWatchDescriptor[pathID];
    auto& path = m_paths[watchDescriptor];
    // LOGI("FileSystemMonitor %i removed %i %s\n", m_inotifyFd, pathID,
    // path.path.c_str());
    m_idToWatchDescriptor.erase(pathID);
    path.instances.erase(pathID);
    if (path.instances.empty()) {
      // LOGI("FileSystemMonitor %i removing inotify watch\n", m_inotifyFd);
      int result = inotify_rm_watch(m_inotifyFd, watchDescriptor);
      assert(result == 0);
      m_paths.erase(watchDescriptor);
    }
  }

  virtual bool checkEvents(const Callback& callback) override {
    struct pollfd fds[] {
      {m_inotifyFd, POLLIN}, {m_cancelFd, POLLIN},
    };
    const auto nfds = sizeof(fds) / sizeof(fds[0]);

    // Block until there are inotify events, or cancel() is called.
    // LOGI("FileSystemMonitor %i poll enter\n", m_inotifyFd);
    int fdsReady = poll(fds, nfds, -1);
    // LOGI("FileSystemMonitor %i poll exit\n", m_inotifyFd);
    assert(fdsReady >= 0);
    if (fdsReady == 0) {
      return true;
    }
    if (fdsReady == -1) {
      if (errno == EINTR || errno == EAGAIN) {
        return true;
      }

      // Stop checking events because some error happened
      LOGE("Error in poll(inotify-fd)\n");
      return false;
    }
    if (fds[1].revents & POLLIN) {
      uint64_t val = 0;
      const ssize_t numBytes = read(m_cancelFd, &val, sizeof(val));
      assert(val == 1);
      assert(numBytes == sizeof(val));

      // Stop checking events because cancel() was called
      return false;
    }

    // There was no error and m_cancelFd wasn't set. Must be an inotify event.
    assert(fds[0].revents & POLLIN);

    // Read event data
    auto readFrom = m_eventBuffer.data() + m_eventBufferBytes;
    auto bytesLeft = m_eventBuffer.size() - m_eventBufferBytes;
    size_t bytesRead = read(m_inotifyFd, readFrom, bytesLeft);
    m_eventBufferBytes += bytesRead;

    // Process whole events in the buffer
    size_t offset = 0;
    while (offset + sizeof(inotify_event) <= m_eventBufferBytes) {
      const auto& event =
          *reinterpret_cast<inotify_event*>(m_eventBuffer.data() + offset);
      size_t eventSize = sizeof(inotify_event) + event.len;
      if (offset + eventSize > m_eventBufferBytes) {
        // Incomplete event read() into buffer
        break;
      }

      // If remove() is called, there may still be queued events. Ignore any for
      // unknown watch descriptors. IN_Q_OVERFLOW can also generate wd == -1.
      auto pathIt = m_paths.find(event.wd);
      if (pathIt != m_paths.end()) {
        const auto& path = pathIt->second;
        for (const auto& instanceIt : path.instances) {
          const auto& instance = instanceIt.second;
          auto reportMask = event.mask & instance.inotifyMask;

          // inotify only gives a name when watching directories, not files.
          auto filename = event.len ? std::string(event.name) : path.path;
          LOGI("FileSystemMonitor %i event (mask %x) for '%s'\n", m_inotifyFd,
               reportMask, filename.c_str());
          if (reportMask & IN_CREATE)
            callback(EventData{FSM_CREATE, filename, instance.userPtr});
          if (reportMask & IN_MODIFY)
            callback(EventData{FSM_MODIFY, filename, instance.userPtr});
          if (reportMask & IN_DELETE)
            callback(EventData{FSM_DELETE, filename, instance.userPtr});
        }
      }
      offset += eventSize;
    }

    // Shift any remainder to the start of the buffer
    m_eventBufferBytes -= offset;
    if (offset && m_eventBufferBytes) {
      memmove(m_eventBuffer.data(), m_eventBuffer.data() + offset,
              m_eventBufferBytes);
    }
    return true;
  }

  virtual void cancel() override {
    uint64_t val = 1;
    const ssize_t numBytes = write(m_cancelFd, &val, sizeof(val));
    assert(numBytes == sizeof(val));
  }

  FileSystemMonitorInotify() {
    m_inotifyFd = inotify_init();
    m_cancelFd = eventfd(0, 0);
  }

  virtual ~FileSystemMonitorInotify() override {
    close(m_cancelFd);
    close(m_inotifyFd);
  }

  int m_inotifyFd;
  int m_cancelFd;

  /** inotify provides a stream of event data. This buffer is here to hold
   * incomplete reads. Sized to hold at least one event for the max path length.
   */
  std::array<uint8_t, sizeof(inotify_event) + PATH_MAX> m_eventBuffer;

  size_t m_eventBufferBytes = 0;

  /** Monitored paths, indexed by the inotify watch descriptor */
  std::unordered_map<int, InotifyPath> m_paths;

  /* Path instance lookup, to provide multiple events for the same path */
  std::unordered_map<PathID, int> m_idToWatchDescriptor;
};
#endif

FileSystemMonitor* FileSystemMonitor::create() {
#if defined(_WIN32)
  return new FileSystemMonitorWindows();
#elif defined(LINUX)
  return new FileSystemMonitorInotify();
#else
  return nullptr;
#endif
}

void FileSystemMonitor::destroy(FileSystemMonitor* monitor) {
  delete monitor;
}
