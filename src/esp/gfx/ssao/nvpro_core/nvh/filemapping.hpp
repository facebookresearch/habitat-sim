/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2020-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#pragma once

#include <cstddef>
#include <utility>

namespace nvh {

class FileMapping {
 public:
  FileMapping(FileMapping&& other) noexcept {
    this->operator=(std::move(other));
  };

  FileMapping& operator=(FileMapping&& other) noexcept {
    m_isValid = other.m_isValid;
    m_fileSize = other.m_fileSize;
    m_mappingType = other.m_mappingType;
    m_mappingPtr = other.m_mappingPtr;
    m_mappingSize = other.m_mappingSize;
#ifdef _WIN32
    m_win32.file = other.m_win32.file;
    m_win32.fileMapping = other.m_win32.fileMapping;
    other.m_win32.file = nullptr;
    other.m_win32.fileMapping = nullptr;
#else
    m_unix.file = other.m_unix.file;
    other.m_unix.file = -1;
#endif
    other.m_isValid = false;
    other.m_mappingPtr = nullptr;

    return *this;
  }

  FileMapping(const FileMapping&) = delete;
  FileMapping& operator=(const FileMapping& other) = delete;
  FileMapping() {}

  ~FileMapping() { close(); }

  enum MappingType {
    MAPPING_READONLY,       // opens existing file for read-only access
    MAPPING_READOVERWRITE,  // creates new file with read/write access,
                            // overwriting existing files
  };

  // fileSize only for write access
  bool open(const char* filename, MappingType mappingType, size_t fileSize = 0);
  void close();

  const void* data() const { return m_mappingPtr; }
  void* data() { return m_mappingPtr; }
  size_t size() const { return m_mappingSize; }
  bool valid() const { return m_isValid; }

 protected:
  static size_t g_pageSize;

#ifdef _WIN32
  struct {
    void* file = nullptr;
    void* fileMapping = nullptr;
  } m_win32;
#else
  struct {
    int file = -1;
  } m_unix;
#endif

  bool m_isValid = false;
  size_t m_fileSize = 0;
  MappingType m_mappingType = MappingType::MAPPING_READONLY;
  void* m_mappingPtr = nullptr;
  size_t m_mappingSize = 0;
};

// convenience types
class FileReadMapping : private FileMapping {
 public:
  bool open(const char* filename) {
    return FileMapping::open(filename, MAPPING_READONLY, 0);
  }
  void close() { FileMapping::close(); }
  const void* data() const { return m_mappingPtr; }
  size_t size() const { return m_fileSize; }
  bool valid() const { return m_isValid; }
};

class FileReadOverWriteMapping : private FileMapping {
 public:
  bool open(const char* filename, size_t fileSize) {
    return FileMapping::open(filename, MAPPING_READOVERWRITE, fileSize);
  }
  void close() { FileMapping::close(); }
  void* data() { return m_mappingPtr; }
  size_t size() const { return m_fileSize; }
  bool valid() const { return m_isValid; }
};
}  // namespace nvh
