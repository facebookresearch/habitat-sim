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

#ifndef NV_SHADERFILEMANAGER_INCLUDED
#define NV_SHADERFILEMANAGER_INCLUDED

#include <stdio.h>
#include <string>
#include <vector>

namespace nvh {

class ShaderFileManager {
  //////////////////////////////////////////////////////////////////////////
  /**
    \class nvh::ShaderFileManager

    The nvh::ShaderFileManager class is meant to be derived from to create the
    actual api-specific shader/program managers.

    The ShaderFileManager provides a system to find/load shader files.
    It also allows resolving #include instructions in HLSL/GLSL source files.
    Such includes can be registered before pointing to strings in memory.

    If m_handleIncludePasting is true, then `#include`s are replaced by
    the include file contents (recursively) before presenting the
    loaded shader source code to the caller. Otherwise, the include file
    loader is still available but `#include`s are left unchanged.

    Furthermore it handles injecting prepended strings (typically used
    for #defines) after the #version statement of GLSL files,
    regardless of m_handleIncludePasting's value.

  */

 public:
  enum FileType {
    FILETYPE_DEFAULT,
    FILETYPE_GLSL,
    FILETYPE_HLSL,
    FILETYPE_SPIRV,
  };

  struct IncludeEntry {
    std::string name;
    std::string filename;
    std::string content;
  };

  typedef std::vector<IncludeEntry> IncludeRegistry;

  static std::string format(const char* msg, ...);

 public:
  class IncludeID {
   public:
    size_t m_value;

    IncludeID() : m_value(size_t(~0)) {}

    IncludeID(size_t b) : m_value((uint32_t)b) {}

    IncludeID& operator=(size_t b) {
      m_value = b;
      return *this;
    }

    bool isValid() const { return m_value != size_t(~0); }

    operator bool() const { return isValid(); }
    operator size_t() const { return m_value; }

    friend bool operator==(const IncludeID& lhs, const IncludeID& rhs) {
      return rhs.m_value == lhs.m_value;
    }
  };

  struct Definition {
    Definition() {}
    Definition(uint32_t type,
               std::string const& prepend,
               std::string const& filename)
        : type(type), prepend(prepend), filename(filename) {}
    Definition(uint32_t type, std::string const& filename)
        : type(type), filename(filename) {}

    uint32_t type = 0;
    std::string filename;
    std::string prepend;
    std::string entry = "main";
    FileType filetype = FILETYPE_DEFAULT;
    std::string filenameFound;
    std::string content;
  };

  // optionally register files to be included, optionally provide content
  // directly rather than from disk
  //
  // name: name used within shader files
  // diskname = filename on disk (defaults to name if not set)
  // content = provide content as string rather than loading from disk

  IncludeID registerInclude(std::string const& name,
                            std::string const& diskname = std::string(),
                            std::string const& content = std::string());

  // Use m_prepend to pass global #defines
  // Derived api classes will use this as global prepend to the per-definition
  // prepends in combination with the source files actualSoure = m_prepend +
  // definition.prepend + definition.content
  std::string m_prepend;

  // per file state, used when FILETYPE_DEFAULT is provided in the Definition
  FileType m_filetype;

  // add search directories
  void addDirectory(const std::string& dir) { m_directories.push_back(dir); }

  ShaderFileManager(bool handleIncludePasting = true)
      : m_filetype(FILETYPE_GLSL),
        m_lineMarkers(true),
        m_forceLineFilenames(false),
        m_forceIncludeContent(false),
        m_supportsExtendedInclude(false),
        m_handleIncludePasting(handleIncludePasting) {
    m_directories.push_back(".");
  }

  //////////////////////////////////////////////////////////////////////////

  // in rare cases you may want to access the included content in detail
  // yourself

  IncludeID findInclude(std::string const& name) const;
  bool loadIncludeContent(IncludeID);
  const IncludeEntry& getIncludeEntry(IncludeID idx) const;

  std::string getProcessedContent(std::string const& filename,
                                  std::string& filenameFound);

 protected:
  std::string markerString(int line, std::string const& filename, int fileid);
  std::string getIncludeContent(IncludeID idx, std::string& filenameFound);
  std::string getContent(std::string const& filename,
                         std::string& filenameFound);
  std::string getContentWithRequestingSourceDirectory(
      std::string const& filename,
      std::string& filenameFound,
      std::string const& requestingSource);

  static std::string getDirectoryComponent(std::string filename);

  std::string manualInclude(std::string const& filename,
                            std::string& filenameFound,
                            std::string const& prepend,
                            bool foundVersion);
  std::string manualIncludeText(std::string const& sourceText,
                                std::string const& textFilename,
                                std::string const& prepend,
                                bool foundVersion);

  bool m_lineMarkers;
  bool m_forceLineFilenames;
  bool m_forceIncludeContent;
  bool m_supportsExtendedInclude;
  bool m_handleIncludePasting;

  std::vector<std::string> m_directories;
  IncludeRegistry m_includes;

  // Used as temporary storage in getContentWithRequestingSourceDirectory; saves
  // on dynamic allocation.
  std::vector<std::string> m_extendedDirectories;
};

}  // namespace nvh

#endif  // NV_PROGRAM_INCLUDED
