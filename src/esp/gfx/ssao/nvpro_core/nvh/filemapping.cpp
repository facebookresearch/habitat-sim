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

#include "filemapping.hpp"
#include <assert.h>

#if defined(LINUX)
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

inline DWORD HIDWORD(size_t x) {
  return (DWORD)(x >> 32);
}
inline DWORD LODWORD(size_t x) {
  return (DWORD)x;
}
#endif

namespace nvh {

bool FileMapping::open(const char* fileName,
                       MappingType mappingType,
                       size_t fileSize) {
  if (!g_pageSize) {
#if defined(_WIN32)
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    g_pageSize = (size_t)si.dwAllocationGranularity;
#elif defined(LINUX)
    g_pageSize = (size_t)getpagesize();
#endif
  }

  m_mappingType = mappingType;

  if (mappingType == MAPPING_READOVERWRITE) {
    assert(fileSize);
    m_fileSize = fileSize;
    m_mappingSize = ((fileSize + g_pageSize - 1) / g_pageSize) * g_pageSize;

    // check if the current process is allowed to save a file of that size
#if defined(_WIN32)
    TCHAR dir[MAX_PATH + 1];
    BOOL success = FALSE;
    ULARGE_INTEGER numFreeBytes;

    DWORD length = GetVolumePathName(fileName, dir, MAX_PATH + 1);

    if (length > 0) {
      success = GetDiskFreeSpaceEx(dir, NULL, NULL, &numFreeBytes);
    }

    m_isValid = (!!success) && (m_mappingSize <= numFreeBytes.QuadPart);
#elif defined(LINUX)
    struct rlimit rlim;
    getrlimit(RLIMIT_FSIZE, &rlim);
    m_isValid = (m_mappingSize <= rlim.rlim_cur);
#endif
    if (!m_isValid) {
      return false;
    }
  }

#if defined(_WIN32)
  m_win32.file =
      mappingType == MAPPING_READONLY
          ? CreateFile(fileName, GENERIC_READ, FILE_SHARE_READ, NULL,
                       OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, NULL)
          : CreateFile(fileName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                       CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

  m_isValid = (m_win32.file != INVALID_HANDLE_VALUE);
  if (m_isValid) {
    if (mappingType == MAPPING_READONLY) {
      DWORD sizeHi = 0;
      DWORD sizeLo = GetFileSize(m_win32.file, &sizeHi);
      m_mappingSize = (static_cast<size_t>(sizeHi) << 32) | sizeLo;
      m_fileSize = m_mappingSize;
    }

    m_win32.fileMapping = CreateFileMapping(
        m_win32.file, NULL,
        mappingType == MAPPING_READONLY ? PAGE_READONLY : PAGE_READWRITE,
        HIDWORD(m_mappingSize), LODWORD(m_mappingSize), NULL);

    m_isValid = (m_win32.fileMapping != NULL);
    if (m_isValid) {
      m_mappingPtr = MapViewOfFile(
          m_win32.fileMapping,
          mappingType == MAPPING_READONLY ? FILE_MAP_READ : FILE_MAP_ALL_ACCESS,
          HIDWORD(0), LODWORD(0), (SIZE_T)0);
      if (!m_mappingPtr) {
#if 0
      DWORD err = GetLastError();
#endif
        CloseHandle(m_win32.file);
        m_isValid = false;
      }
    } else {
      CloseHandle(m_win32.file);
    }
  }
#elif defined(LINUX)
  m_unix.file = mappingType == MAPPING_READONLY
                    ? ::open(fileName, O_RDONLY)
                    : ::open(fileName, O_RDWR | O_CREAT | O_TRUNC, 0666);

  m_isValid = (m_unix.file != -1);
  if (m_isValid) {
    if (mappingType == MAPPING_READONLY) {
      struct stat s;
      m_isValid &= (fstat(m_unix.file, &s) >= 0);
      m_mappingSize = s.st_size;
    } else {
      // make file large enough to hold the complete scene
      m_isValid &= (lseek(m_unix.file, m_mappingSize - 1, SEEK_SET) >= 0);
      m_isValid &= (write(m_unix.file, "", 1) >= 0);
      m_isValid &= (lseek(m_unix.file, 0, SEEK_SET) >= 0);
    }
    m_fileSize = m_mappingSize;
    if (m_isValid) {
      m_mappingPtr =
          mmap(0, m_mappingSize,
               mappingType == MAPPING_READONLY ? PROT_READ
                                               : (PROT_READ | PROT_WRITE),
               MAP_SHARED, m_unix.file, 0);
      m_isValid = (m_mappingPtr != MAP_FAILED);
    }
    if (!m_isValid) {
      ::close(m_unix.file);
      m_unix.file = -1;
    }
  }
#endif
  return m_isValid;
}

void FileMapping::close() {
  if (m_isValid) {
#if defined(_WIN32)
    assert((m_win32.file != INVALID_HANDLE_VALUE) &&
           (m_win32.fileMapping != NULL));

    UnmapViewOfFile(m_mappingPtr);
    CloseHandle(m_win32.fileMapping);

    if (m_mappingType == MAPPING_READOVERWRITE) {
      // truncate file to minimum size
      // To work with 64-bit file pointers, you can declare a LONG, treat it as
      // the upper half of the 64-bit file pointer, and pass its address in
      // lpDistanceToMoveHigh. This means you have to treat two different
      // variables as a logical unit, which is error-prone. The problems can be
      // ameliorated by using the LARGE_INTEGER structure to create a 64-bit
      // value and passing the two 32-bit values by means of the appropriate
      // elements of the union. (see msdn documentation on SetFilePointer)
      LARGE_INTEGER li;
      li.QuadPart = (__int64)m_fileSize;
      SetFilePointer(m_win32.file, li.LowPart, &li.HighPart, FILE_BEGIN);

      SetEndOfFile(m_win32.file);
    }
    CloseHandle(m_win32.file);

    m_mappingPtr = nullptr;
    m_win32.fileMapping = nullptr;
    m_win32.file = nullptr;

#elif defined(LINUX)
    assert(m_unix.file != -1);

    munmap(m_mappingPtr, m_mappingSize);
    ::close(m_unix.file);

    m_mappingPtr = nullptr;
    m_unix.file = -1;
#endif

    m_isValid = false;
  }
}

size_t FileMapping::g_pageSize = 0;

}  // namespace nvh
