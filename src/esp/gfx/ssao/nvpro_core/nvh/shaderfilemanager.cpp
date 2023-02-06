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

/*
 * This file contains code derived from glf by Christophe Riccio, www.g-truc.net
 * Copyright (c) 2005 - 2015 G-Truc Creation (www.g-truc.net)
 * https://github.com/g-truc/ogl-samples/blob/master/framework/compiler.cpp
 */

#include "shaderfilemanager.hpp"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include "fileoperations.hpp"

namespace nvh {

std::string ShaderFileManager::format(const char* msg, ...) {
  char text[8192];
  va_list list;

  if (msg == 0)
    return std::string();

  va_start(list, msg);
  vsnprintf(text, sizeof(text), msg, list);
  va_end(list);

  return std::string(text);
}

inline std::string ShaderFileManager::markerString(int line,
                                                   std::string const& filename,
                                                   int fileid) {
  if (m_supportsExtendedInclude || m_forceLineFilenames) {
#if defined(_WIN32) && 1
    std::string fixedname;
    for (size_t i = 0; i < filename.size(); i++) {
      char c = filename[i];
      if (c == '/' || c == '\\') {
        fixedname.append("\\\\");
      } else {
        fixedname.append(1, c);
      }
    }
#else
    std::string fixedname = filename;
#endif
    return ShaderFileManager::format("#line %d \"", line) + fixedname +
           std::string("\"\n");
  } else {
    return ShaderFileManager::format("#line %d %d\n", line, fileid);
  }
}

std::string ShaderFileManager::getIncludeContent(IncludeID idx,
                                                 std::string& filename) {
  IncludeEntry& entry = m_includes[idx];

  filename = entry.filename;

  if (m_forceIncludeContent) {
    return entry.content;
  }

  if (!entry.content.empty() &&
      !findFile(entry.filename, m_directories).empty()) {
    return entry.content;
  }

  std::string content =
      loadFile(entry.filename, false, m_directories, filename, true);
  return content.empty() ? entry.content : content;
}

std::string ShaderFileManager::getContent(std::string const& filename,
                                          std::string& filenameFound) {
  if (filename.empty()) {
    return std::string();
  }

  IncludeID idx = findInclude(filename);

  if (idx.isValid()) {
    return getIncludeContent(idx, filenameFound);
  }

  // fall back
  filenameFound = filename;
  return loadFile(filename, false, m_directories, filenameFound, true);
}

std::string ShaderFileManager::getContentWithRequestingSourceDirectory(
    std::string const& filename,
    std::string& filenameFound,
    std::string const& requestingSource) {
  if (filename.empty()) {
    return std::string();
  }

  IncludeID idx = findInclude(filename);

  if (idx.isValid()) {
    return getIncludeContent(idx, filenameFound);
  }

  // fall back; check requestingSource's directory first.
  filenameFound = filename;
  m_extendedDirectories.resize(m_directories.size() + 1);
  m_extendedDirectories[0] = getDirectoryComponent(requestingSource);
  for (size_t i = 0; i < m_directories.size(); ++i) {
    m_extendedDirectories[i + 1] = m_directories[i];
  }
  return loadFile(filename, false, m_extendedDirectories, filenameFound, true);
}

std::string ShaderFileManager::getDirectoryComponent(std::string filename) {
  while (!filename.empty()) {
    auto popped = filename.back();
    filename.pop_back();
    switch (popped) {
      case '/':
        goto exitLoop;
#if defined(_WIN32)
      case '\\':
        goto exitLoop;
#endif
    }
  }
exitLoop:
  if (filename.empty())
    filename.push_back('.');
  return filename;
}

std::string ShaderFileManager::manualInclude(std::string const& filename,
                                             std::string& filenameFound,
                                             std::string const& prepend,
                                             bool foundVersion) {
  std::string source = getContent(filename, filenameFound);
  return manualIncludeText(source, filenameFound, prepend, foundVersion);
}

std::string ShaderFileManager::manualIncludeText(
    std::string const& sourceText,
    std::string const& textFilename,
    std::string const& prepend,
    bool foundVersion) {
  if (sourceText.empty()) {
    return std::string();
  }

  std::stringstream stream;
  stream << sourceText;
  std::string line, text;

  // Handle command line defines
  text += prepend;
  if (m_lineMarkers) {
    text += markerString(1, textFilename, 0);
  }

  int lineCount = 0;
  while (std::getline(stream, line)) {
    std::size_t offset = 0;
    lineCount++;

    // Version
    offset = line.find("#version");
    if (offset != std::string::npos) {
      std::size_t commentOffset = line.find("//");
      if (commentOffset != std::string::npos && commentOffset < offset)
        continue;

      if (foundVersion) {
        // someone else already set the version, so just comment out
        text += std::string("//") + line + std::string("\n");
      } else {
        // Reorder so that the #version line is always the first of a shader
        // text
        text = line + std::string("\n") + text + std::string("//") + line +
               std::string("\n");
        foundVersion = true;
      }
      continue;
    }

    // Handle replacing #include with text if configured to do so.
    // Otherwise just insert the #include command verbatim, for shaderc to
    // handle.
    if (m_handleIncludePasting) {
      offset = line.find("#include");
      if (offset != std::string::npos) {
        std::size_t commentOffset = line.find("//");
        if (commentOffset != std::string::npos && commentOffset < offset)
          continue;

        size_t firstQuote = line.find("\"", offset);
        size_t secondQuote = line.find("\"", firstQuote + 1);

        std::string include =
            line.substr(firstQuote + 1, secondQuote - firstQuote - 1);

        std::string includeFound;
        std::string includeContent =
            manualInclude(include, includeFound, std::string(), foundVersion);

        if (!includeContent.empty()) {
          text += includeContent;
          if (m_lineMarkers) {
            text += std::string("\n") +
                    markerString(lineCount + 1, textFilename, 0);
          }
        }
        continue;  // Skip adding the original #include line.
      }
    }

    text += line + "\n";
  }

  return text;
}

ShaderFileManager::IncludeID ShaderFileManager::registerInclude(
    std::string const& name,
    std::string const& filename,
    std::string const& content) {
  // find if already registered
  for (size_t i = 0; i < m_includes.size(); i++) {
    if (m_includes[i].name == name) {
      m_includes[i].content = content;
      return i;
    }
  }

  IncludeEntry entry;
  entry.name = name;
  entry.filename = filename.empty() ? name : filename;
  entry.content = content;

  m_includes.push_back(entry);

  return m_includes.size() - 1;
}

ShaderFileManager::IncludeID ShaderFileManager::findInclude(
    std::string const& name) const {
  // check registered includes first
  for (std::size_t i = 0; i < m_includes.size(); ++i) {
    if (m_includes[i].name == name) {
      return IncludeID(i);
    }
  }

  return IncludeID();
}

bool ShaderFileManager::loadIncludeContent(IncludeID idx) {
  std::string filenameFound;
  m_includes[idx].content = getIncludeContent(idx, filenameFound);
  return !m_includes[idx].content.empty();
}

const ShaderFileManager::IncludeEntry& ShaderFileManager::getIncludeEntry(
    IncludeID idx) const {
  return m_includes[idx];
}

std::string ShaderFileManager::getProcessedContent(std::string const& filename,
                                                   std::string& filenameFound) {
  return manualInclude(filename, filenameFound, "", false);
}

}  // namespace nvh
