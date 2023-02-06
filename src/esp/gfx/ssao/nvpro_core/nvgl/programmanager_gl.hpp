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

#ifndef NV_PROGRAMMANAGER_INCLUDED
#define NV_PROGRAMMANAGER_INCLUDED

#include <stdio.h>
#include <string>
#include <vector>
#include "extensions_gl.hpp"

#include "../nvh/nvprint.hpp"
#include "../nvh/shaderfilemanager.hpp"

namespace nvgl {
//////////////////////////////////////////////////////////////////////////
/**
    \class nvgl::ProgramManager

    The nvgl::ProgramManager manages OpenGL programs generated from shader files
   (GLSL)

    Using ShaderFileManager it will find the files and resolve #include for
   GLSL. You must add include directories to the base-class for this.

    It also comes with some convenience functions to reload shaders etc.
    That is why we pass out the ProgramID rather than a GLuint directly.

    Example:

    \code{.cpp}
    ProgramManager mgr;

    // derived from ShaderFileManager
    mgr.addDirectory("/shaders/")

    // injected after #version directive
    mgr.m_prepend = "#define USE_NOISE 1\n";

    id = mgr.createProgram({{GL_VERTEX_SHADER,
   "object.vert.glsl"},{GL_FRAGMENT_SHADER, "object.frag.glsl"}}):

    glUseProgram(mgr.get(id));
    \endcode
  */

class ProgramID {
 public:
  size_t m_value;

  ProgramID() : m_value(size_t(~0)) {}
  ProgramID(size_t b) : m_value(b) {}

  ProgramID& operator=(size_t b) {
    m_value = b;
    return *this;
  }

  bool isValid() const { return m_value != size_t(~0); }

  operator bool() const { return isValid(); }
  operator size_t() const { return m_value; }

  friend bool operator==(const ProgramID& lhs, const ProgramID& rhs) {
    return rhs.m_value == lhs.m_value;
  }
};

class ProgramManager : public nvh::ShaderFileManager {
 public:
  static const uint32_t PREPROCESS_ONLY_PROGRAM = ~0;
  struct Program {
    Program() : program(0) {}

    uint32_t program;
    std::vector<Definition> definitions;
  };

  ProgramID createProgram(const std::vector<Definition>& definitions);
  ProgramID createProgram(const Definition& def0,
                          const Definition& def1 = Definition(),
                          const Definition& def2 = Definition(),
                          const Definition& def3 = Definition(),
                          const Definition& def4 = Definition());

  void destroyProgram(ProgramID idx);
  void reloadProgram(ProgramID idx);

  void reloadPrograms();
  void deletePrograms();
  bool areProgramsValid();

  bool isValid(ProgramID idx) const;
  unsigned int get(ProgramID idx) const;

  //////////////////////////////////////////////////////////////////////////
  // special purpose use, normally not required to touch

  // if not empty then we will store program binaries in files that use the
  // cachefile as prefix
  //   m_useCacheFile + "_"... implementation dependent
  std::string m_useCacheFile;

  // look for cachefiles first, otherwise look for original glsl files
  bool m_preferCache;
  // don't create actual program, only preprocess definition strings
  bool m_preprocessOnly;
  // don't create actual program, treat filename as raw
  bool m_rawOnly;

  ProgramManager(ProgramManager const&) = delete;
  ProgramManager& operator=(ProgramManager const&) = delete;

  ProgramManager()
      : m_preprocessOnly(false), m_preferCache(false), m_rawOnly(false) {
    m_filetype = FILETYPE_GLSL;
  }

 private:
  bool setupProgram(Program& prog);

  bool loadBinary(GLuint program,
                  const std::string& combinedPrepend,
                  const std::string& combinedFilenames);
  void saveBinary(GLuint program,
                  const std::string& combinedPrepend,
                  const std::string& combinedFilenames);
  std::string binaryName(const std::string& combinedPrepend,
                         const std::string& combinedFilenames);

  std::vector<Program> m_programs;
};

}  // namespace nvgl

#endif
