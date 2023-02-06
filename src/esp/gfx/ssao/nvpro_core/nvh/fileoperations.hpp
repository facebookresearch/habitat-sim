/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

#include <algorithm>  // std::max
#include <fstream>
#include <sstream>
#include <vector>

#include "nvprint.hpp"

/**
 # functions in nvh

 - nvh::fileExists : check if file exists
 - nvh::findFile : finds filename in provided search directories
 - nvh::loadFile : (multiple overloads) loads file as std::string, binary or
 text, can also search in provided directories
 - nvh::getFileName : splits filename from filename with path
 - nvh::getFilePath : splits filepath from filename with path
 */

namespace nvh {

inline bool fileExists(const char* filename) {
  std::ifstream stream;
  stream.open(filename);
  return stream.is_open();
}

// returns first found filename (searches within directories provided)
inline std::string findFile(const std::string& infilename,
                            const std::vector<std::string>& directories,
                            bool warn = false) {
  std::ifstream stream;

  {
    stream.open(infilename.c_str());
    if (stream.is_open()) {
      // nvprintfLevel(LOGLEVEL_INFO, "Found: %s\n", infilename.c_str());
      return infilename;
    }
  }

  for (const auto& directory : directories) {
    std::string filename = directory + "/" + infilename;
    stream.open(filename.c_str());
    if (stream.is_open()) {
      // nvprintfLevel(LOGLEVEL_INFO, "Found: %s\n", filename.c_str());
      return filename;
    }
  }

  if (warn) {
    nvprintfLevel(LOGLEVEL_WARNING, "File not found: %s\n", infilename.c_str());
    nvprintfLevel(LOGLEVEL_WARNING, "In directories: \n");
    for (const auto& directory : directories) {
      nvprintfLevel(LOGLEVEL_WARNING, " - %s\n", directory.c_str());
    }
    nvprintfLevel(LOGLEVEL_WARNING, "\n");
  }

  return {};
}

inline std::string loadFile(const std::string& filename, bool binary) {
  std::string result;
  std::ifstream stream(
      filename,
      std::ios::ate | (binary ? std::ios::binary : std::ios_base::openmode(0)));

  if (!stream.is_open()) {
    return result;
  }

  result.reserve(stream.tellg());
  stream.seekg(0, std::ios::beg);

  result.assign((std::istreambuf_iterator<char>(stream)),
                std::istreambuf_iterator<char>());
  return result;
}

inline std::string loadFile(const char* filename, bool binary) {
  std::string name(filename);
  return loadFile(name, binary);
}

inline std::string loadFile(const std::string& filename,
                            bool binary,
                            const std::vector<std::string>& directories,
                            std::string& filenameFound,
                            bool warn = false) {
  filenameFound = findFile(filename, directories, warn);
  if (filenameFound.empty()) {
    return {};
  } else {
    return loadFile(filenameFound, binary);
  }
}

inline std::string loadFile(const std::string filename,
                            bool binary,
                            const std::vector<std::string>& directories,
                            bool warn = false) {
  std::string filenameFound;
  return loadFile(filename, binary, directories, filenameFound, warn);
}

// splits filename excluding path
inline std::string getFileName(std::string const& fullPath) {
  size_t istart;
  for (istart = fullPath.size() - 1;
       istart != -1 && fullPath[istart] != '\\' && fullPath[istart] != '/';
       istart--)
    ;
  return std::string(&fullPath[istart + 1]);
}

// splits path from filename
inline std::string getFilePath(const char* filename) {
  std::string path;
  // find path in filename
  {
    std::string filepath(filename);

    size_t pos0 = filepath.rfind('\\');
    size_t pos1 = filepath.rfind('/');

    pos0 = pos0 == std::string::npos ? 0 : pos0;
    pos1 = pos1 == std::string::npos ? 0 : pos1;

    path = filepath.substr(0, std::max(pos0, pos1));
  }

  if (path.empty()) {
    path = ".";
  }

  return path;
}

// Return true if the filename ends with ending. i.e. ".png"
inline bool endsWith(std::string const& value, std::string const& ending) {
  if (ending.size() > value.size())
    return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

}  // namespace nvh
