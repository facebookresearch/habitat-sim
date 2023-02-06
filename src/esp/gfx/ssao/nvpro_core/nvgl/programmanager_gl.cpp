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

#include "programmanager_gl.hpp"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../nvh/fileoperations.hpp"
#include "../nvh/nvprint.hpp"

namespace nvgl {

static constexpr int has_GL_VERSION_4_1 = 1;  // temp hack

static bool checkProgram(GLuint program) {
  if (!program)
    return false;

  GLint result = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &result);

  int infoLogLength;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLength);
  if (infoLogLength > 1
#ifdef NDEBUG
      && result == GL_FALSE
#endif
  ) {
    std::vector<char> buffer(infoLogLength);
    glGetProgramInfoLog(program, infoLogLength, NULL, &buffer[0]);
    LOGW("%s\n", &buffer[0]);
  }

  return result == GL_TRUE;
}

static bool checkShader(GLuint shader, std::string const& filename) {
  if (!shader)
    return false;

  GLint result = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &result);

  LOGI("%s ...\n", filename.c_str());
  int infoLogLength;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLength);
  if (infoLogLength > 1
#ifdef NDEBUG
      && result == GL_FALSE
#endif
  ) {
    std::vector<char> buffer(infoLogLength);
    glGetShaderInfoLog(shader, infoLogLength, NULL, &buffer[0]);
    LOGW("%s\n", &buffer[0]);
  }

  return result == GL_TRUE;
}

bool ProgramManager::setupProgram(Program& prog) {
  prog.program = 0;

  if (prog.definitions.empty())
    return false;

  m_supportsExtendedInclude =
      true;  // temp hack has_GL_ARB_shading_language_include != 0;

  std::string combinedPrepend = m_prepend;
  std::string combinedFilenames;
  for (size_t i = 0; i < prog.definitions.size(); i++) {
    combinedPrepend += prog.definitions[i].prepend;
    combinedFilenames += prog.definitions[i].filename;
  }

  bool allFound = true;
  for (size_t i = 0; i < prog.definitions.size(); i++) {
    Definition& definition = prog.definitions[i];
    if (definition.filetype == FILETYPE_DEFAULT) {
      definition.filetype = m_filetype;
    }

    if (m_rawOnly) {
      definition.content =
          getContent(definition.filename, definition.filenameFound);
    } else {
      char const* strDefine = "";

      switch (definition.type) {
        case GL_VERTEX_SHADER:
          strDefine = "#define _VERTEX_SHADER_ 1\n";
          break;
        case GL_FRAGMENT_SHADER:
          strDefine = "#define _FRAGMENT_SHADER_ 1\n";
          break;
        case GL_COMPUTE_SHADER:
          strDefine = "#define _COMPUTE_SHADER_ 1\n";
          break;
        case GL_GEOMETRY_SHADER:
          strDefine = "#define _GEOMETRY_SHADER_ 1\n";
          break;
        case GL_TESS_CONTROL_SHADER:
          strDefine = "#define _TESS_CONTROL_SHADER_ 1\n";
          break;
        case GL_TESS_EVALUATION_SHADER:
          strDefine = "#define _TESS_EVALUATION_SHADER_ 1\n";
          break;
#if GL_NV_mesh_shader
        case GL_MESH_SHADER_NV:
          strDefine = "#define _MESH_SHADER_ 1\n";
          break;
        case GL_TASK_SHADER_NV:
          strDefine = "#define _TASK_SHADER_ 1\n";
          break;
#endif
      }

      definition.content = manualInclude(
          definition.filename, definition.filenameFound,
          m_prepend + definition.prepend + std::string(strDefine), false);
    }
    allFound = allFound && !definition.content.empty();
  }

  if (m_preprocessOnly) {
    prog.program = PREPROCESS_ONLY_PROGRAM;
    return true;
  } else {
    prog.program = glCreateProgram();
    if (!m_useCacheFile.empty() && has_GL_VERSION_4_1) {
      glProgramParameteri(prog.program, GL_PROGRAM_BINARY_RETRIEVABLE_HINT,
                          GL_TRUE);
    }
  }

  bool loadedCache = false;
  if (!m_useCacheFile.empty() && (!allFound || m_preferCache) &&
      has_GL_VERSION_4_1) {
    // try cache
    loadedCache = loadBinary(prog.program, combinedPrepend, combinedFilenames);
  }
  if (!loadedCache) {
    for (size_t i = 0; i < prog.definitions.size(); i++) {
      Definition& definition = prog.definitions[i];
      GLuint shader = 0;
      if (!definition.content.empty()) {
        char const* sourcePointer = definition.content.c_str();
        shader = glCreateShader(definition.type);
        glShaderSource(shader, 1, &sourcePointer, NULL);
        glCompileShader(shader);
      }
      if (!shader || !checkShader(shader, definition.filename)) {
        glDeleteShader(shader);
        glDeleteProgram(prog.program);
        prog.program = 0;
        return false;
      }
      glAttachShader(prog.program, shader);
      glDeleteShader(shader);
    }
    glLinkProgram(prog.program);
  }

  if (checkProgram(prog.program)) {
    if (!m_useCacheFile.empty() && !loadedCache && has_GL_VERSION_4_1) {
      saveBinary(prog.program, combinedPrepend, combinedFilenames);
    }
    return true;
  }

  glDeleteProgram(prog.program);
  prog.program = 0;
  return false;
}

ProgramID ProgramManager::createProgram(
    const Definition& def0,
    const Definition& def1 /*= ShaderDefinition()*/,
    const Definition& def2 /*= ShaderDefinition()*/,
    const Definition& def3 /*= ShaderDefinition()*/,
    const Definition& def4 /*= ShaderDefinition()*/) {
  std::vector<ProgramManager::Definition> defs;
  defs.push_back(def0);
  if (def1.type)
    defs.push_back(def1);
  if (def2.type)
    defs.push_back(def2);
  if (def3.type)
    defs.push_back(def3);
  if (def4.type)
    defs.push_back(def4);

  return createProgram(defs);
}

ProgramID ProgramManager::createProgram(
    const std::vector<ProgramManager::Definition>& definitions) {
  Program prog;
  prog.definitions = definitions;

  setupProgram(prog);

  for (size_t i = 0; i < m_programs.size(); i++) {
    if (m_programs[i].definitions.empty()) {
      m_programs[i] = prog;
      return i;
    }
  }

  m_programs.push_back(prog);
  return m_programs.size() - 1;
}

bool ProgramManager::areProgramsValid() {
  bool valid = true;
  for (size_t i = 0; i < m_programs.size(); i++) {
    valid = valid && isValid((ProgramID)i);
  }
  return valid;
}

void ProgramManager::deletePrograms() {
  for (size_t i = 0; i < m_programs.size(); i++) {
    if (m_programs[i].program &&
        m_programs[i].program != PREPROCESS_ONLY_PROGRAM) {
      glDeleteProgram(m_programs[i].program);
    }
    m_programs[i].program = 0;
  }
}

void ProgramManager::reloadProgram(ProgramID i) {
  if (!isValid(i))
    return;

  bool old = m_preprocessOnly;

  if (m_programs[i].program &&
      m_programs[i].program != PREPROCESS_ONLY_PROGRAM) {
    glDeleteProgram(m_programs[i].program);
  }

  m_preprocessOnly = m_programs[i].program == PREPROCESS_ONLY_PROGRAM;
  m_programs[i].program = 0;
  if (!m_programs[i].definitions.empty()) {
    setupProgram(m_programs[i]);
  }
  m_preprocessOnly = old;
}

void ProgramManager::reloadPrograms() {
  LOGI("Reloading programs...\n");

  for (size_t i = 0; i < m_programs.size(); i++) {
    reloadProgram((ProgramID)i);
  }

  LOGI("done\n");
}

bool ProgramManager::isValid(ProgramID idx) const {
  return idx.isValid() &&
         (m_programs[idx].definitions.empty() || m_programs[idx].program != 0);
}

unsigned int ProgramManager::get(ProgramID idx) const {
  assert(m_programs[idx].program != PREPROCESS_ONLY_PROGRAM);
  return m_programs[idx].program;
}

void ProgramManager::destroyProgram(ProgramID idx) {
  if (m_programs[idx].program &&
      m_programs[idx].program != PREPROCESS_ONLY_PROGRAM) {
    glDeleteProgram(m_programs[idx].program);
  }
  m_programs[idx].program = 0;
  m_programs[idx].definitions.clear();
}

//-----------------------------------------------------------------------------
// MurmurHash2A, by Austin Appleby

// This is a variant of MurmurHash2 modified to use the Merkle-Damgard
// construction. Bulk speed should be identical to Murmur2, small-key speed
// will be 10%-20% slower due to the added overhead at the end of the hash.

// This variant fixes a minor issue where null keys were more likely to
// collide with each other than expected, and also makes the algorithm
// more amenable to incremental implementations. All other caveats from
// MurmurHash2 still apply.

#define mmix(h, k) \
  {                \
    k *= m;        \
    k ^= k >> r;   \
    k *= m;        \
    h *= m;        \
    h ^= k;        \
  }

static unsigned int strMurmurHash2A(const void* key,
                                    size_t len,
                                    unsigned int seed) {
  const unsigned int m = 0x5bd1e995;
  const int r = 24;
  unsigned int l = (unsigned int)len;

  const unsigned char* data = (const unsigned char*)key;

  unsigned int h = seed;
  unsigned int t = 0;

  while (len >= 4) {
    unsigned int k = *(unsigned int*)data;

    mmix(h, k);

    data += 4;
    len -= 4;
  }

  switch (len) {
    case 3:
      t ^= data[2] << 16;
    case 2:
      t ^= data[1] << 8;
    case 1:
      t ^= data[0];
  };

  mmix(h, t);
  mmix(h, l);

  h ^= h >> 13;
  h *= m;
  h ^= h >> 15;

  return h;
}
#undef mmix

static size_t strHexFromByte(char* buffer,
                             size_t bufferlen,
                             const void* data,
                             size_t len) {
  const char tostr[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  const unsigned char* d = (const unsigned char*)data;
  char* out = buffer;
  size_t i = 0;
  for (; i < len && (i * 2) + 1 < bufferlen; i++, d++, out += 2) {
    unsigned int val = *d;
    unsigned int hi = val / 16;
    unsigned int lo = val % 16;
    out[0] = tostr[hi];
    out[1] = tostr[lo];
  }

  return i * 2;
}

std::string ProgramManager::binaryName(const std::string& combinedPrepend,
                                       const std::string& combinedFilenames) {
  unsigned int hashCombine =
      combinedPrepend.empty()
          ? 0
          : strMurmurHash2A(&combinedPrepend[0], combinedPrepend.size(), 127);
  unsigned int hashFilenames =
      strMurmurHash2A(&combinedFilenames[0], combinedFilenames.size(), 129);

  std::string hexCombine;
  std::string hexFilenames;
  hexCombine.resize(8);
  hexFilenames.resize(8);
  strHexFromByte(&hexCombine[0], 8, &hashCombine, 4);
  strHexFromByte(&hexFilenames[0], 8, &hashFilenames, 4);

  return m_useCacheFile + "_" + hexCombine + "_" + hexFilenames + ".glp";
}

bool ProgramManager::loadBinary(GLuint program,
                                const std::string& combinedPrepend,
                                const std::string& combinedFilenames) {
  std::string filename = binaryName(combinedPrepend, combinedFilenames);
  std::string filenameFound;
  std::string binraw =
      nvh::loadFile(filename, true, m_directories, filenameFound);
  if (!binraw.empty()) {
    const char* bindata = &binraw[0];
    glProgramBinary(program, *(GLenum*)bindata, bindata + 4,
                    GLsizei(binraw.size() - 4));
    return checkProgram(program);
  }
  return false;
}

void ProgramManager::saveBinary(GLuint program,
                                const std::string& combinedPrepend,
                                const std::string& combinedFilenames) {
  std::string filename = binaryName(combinedPrepend, combinedFilenames);

  GLint datasize;
  GLint datasize2;
  glGetProgramiv(program, GL_PROGRAM_BINARY_LENGTH, &datasize);

  std::string binraw;
  binraw.resize(datasize + 4ULL);
  char* bindata = &binraw[0];
  glGetProgramBinary(program, datasize, &datasize2, (GLenum*)bindata,
                     bindata + 4);

  std::ofstream binfile;
  binfile.open(filename.c_str(), std::ios::binary | std::ios::out);
  if (binfile.is_open()) {
    binfile.write(bindata, datasize + 4ULL);
  }
}
}  // namespace nvgl
