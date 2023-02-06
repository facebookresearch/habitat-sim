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

#ifndef __NVPARMETERTOOLS_H__
#define __NVPARMETERTOOLS_H__

#include "../nvp/platform.h"

#include <climits>
#include <functional>
#include <string>
#include <vector>

namespace nvh {

//////////////////////////////////////////////////////////////////////////
/**
    \class nvh::ParameterList

    The nvh::ParameterList helps parsing commandline arguments
    or commandline arguments stored within ascii config files.

    Parameters always update the values they point to, and optionally
    can trigger a callback that can be provided per-parameter.

    \code{.cpp}
    ParameterList list;
    std::string   modelFilename;
    float         modelScale;

    list.addFilename(".gltf|model filename", &modelFilename);
    list.add("scale|model scale", &modelScale);

    list.applyTokens(3, {"blah.gltf","-scale","4"}, "-", "/assets/");
    \endcode

    Use in combination with the ParameterSequence class to iterate
    sequences of parameter changes for benchmarking/automation.
  */

class ParameterList {
 public:
  typedef std::function<void(uint32_t)> Callback;

  enum Type {
    TYPE_FLOAT,
    TYPE_INT,
    TYPE_UINT,
    TYPE_BOOL,
    TYPE_BOOL_VALUE,
    TYPE_STRING,
    TYPE_FILENAME,
    TYPE_TRIGGER,
  };

  ParameterList();

  uint32_t append(const ParameterList& list);

  // Add a parameter. The name can be given in format: name[|help text], for
  // example: "winsize|Set window size"
  uint32_t add(const char* name,
               float* destination,
               Callback callback = nullptr,
               uint32_t length = 1,
               float min = -FLT_MAX,
               float max = FLT_MAX);
  uint32_t add(const char* name,
               int32_t* destination,
               Callback callback = nullptr,
               uint32_t length = 1,
               int32_t min = -INT_MAX,
               int32_t max = +INT_MAX);
  uint32_t add(const char* name,
               uint32_t* destination,
               Callback callback = nullptr,
               uint32_t length = 1,
               uint32_t min = 0,
               uint32_t max = 0xFFFFFFFF);
  uint32_t add(const char* name,
               bool* destination,
               Callback callback = nullptr,
               uint32_t length = 1);
  uint32_t add(const char* name,
               bool* destination,
               bool value,
               Callback callback = nullptr,
               uint32_t length = 1);
  uint32_t add(const char* name,
               std::string* destination,
               Callback callback = nullptr,
               uint32_t length = 1);
  uint32_t add(const char* name, Callback callback, uint32_t length = 0);

  // if the parameter "name" starts with "." then we test the variable against
  // this file-ending rather than treating name as commandline option.  So an
  // argument that ends with ".blah" will trigger this parameter
  uint32_t addFilename(const char* name,
                       std::string* destination,
                       Callback callback = nullptr);

  // Set help of a parameter, returns the parameterIndex
  uint32_t setHelp(uint32_t parameterIndex, const char* helptext);

  // returns number of tokens found
  // paramPrefix is typically "-"
  // relative filenames get the defaultFilePath prepended
  uint32_t applyTokens(uint32_t argCount,
                       const char** argv,
                       const char* paramPrefix = nullptr,
                       const char* defaultFilePath = nullptr) const;
  // tests only single argument, increases arg by appropriate length on success
  // (returns true)
  bool applyParameters(uint32_t argCount,
                       const char** argv,
                       uint32_t& arg,
                       const char* paramPrefix = nullptr,
                       const char* defaultFilePath = nullptr) const;

  // prints all registered parameters and optional help strings
  void print() const;

  // separators are all space (tab, newline etc.) characters
  // preserves quotes based on "", converts backslashes, uses # as line comment
  // modifies content string by setting 0 at separators
  static void tokenizeString(std::string& content,
                             std::vector<const char*>& args);
  static const char* toString(Type typ);

 private:
  struct Parameter {
    Type type = TYPE_FLOAT;
    std::string name;
    uint32_t readLength = 0;
    uint32_t writeLength = 0;
    union {
      uint32_t u32;
      int32_t s32;
      float f32;
      bool b;
    } minmax[2] = {0, 0};
    union {
      uint32_t* u32;
      int32_t* s32;
      float* f32;
      bool* b;
      std::string* str;
      void* ptr;
    } destination = {nullptr};
    Callback callback = nullptr;
    std::string helptext;

    Parameter() {}
    Parameter(Type type,
              const char* name,
              Callback callback,
              void* destination,
              uint32_t readLength,
              uint32_t writeLength);
  };

  std::vector<Parameter> m_parameters;

  Parameter makeParam(Type type,
                      const char* name,
                      Callback callback,
                      void* destination,
                      uint32_t readLength,
                      uint32_t writeLength);
};

//////////////////////////////////////////////////////////////////////////
/**
    \class nvh::ParameterSequence

    The nvh::ParameterSequence processes provided tokens in sequences.
    The sequences are terminated by a special "separator" token.
    All tokens between the last iteration and the separator are applied
    to the provided ParameterList.
    Useful to process commands in sequences (automation, benchmarking etc.).

    Example:

    \code{.cpp}
    ParameterSequence sequence;
    ParameterList     list;
    int               mode;
    list.add("mode", &mode);

    std::vector<const char*> tokens;
    ParameterList::tokenizeString("benchmark simple -mode 10 benchmark complex
   -mode 20", tokens); sequence.init(&list, tokens);

       // 1 means our separator is followed by one argument (simple/complex)
       // "-" as parameters in the string are prefixed with -

    while(!sequence.advanceIteration("benchmark", 1, "-")) {
      printf("%d %s mode %d\n", sequence.getIteration(),
   sequence.getSeparatorArg(0), mode);
    }

    // would print:
    //   0 simple mode 10
    //   1 complex mode 20
    \endcode
  */

class ParameterSequence {
 public:
  ParameterSequence()
      : m_list(nullptr), m_index(0), m_iteration(0), m_separator(0) {}

  void init(const ParameterList* list, const std::vector<const char*>& tokens) {
    m_tokens = tokens;
    m_list = list;
  }

  // returns true if finished with all tokens, otherwise processes until next
  // separator token is found
  bool advanceIteration(const char* separator,
                        uint32_t separatorArgLength,
                        uint32_t& argBegin,
                        uint32_t& argCount);
  // also applies parameterlist
  bool applyIteration(const char* separator,
                      uint32_t separatorArgLength = 0,
                      const char* paramPrefix = nullptr,
                      const char* defaultFilePath = nullptr);
  // sets iteration to beginning
  void resetIteration();

  bool isActive() const { return m_list && m_index && m_iteration; }

  uint32_t getIteration() const { return m_iteration; }

  const char* getSeparatorArg(uint32_t offset) const {
    return m_separator != ~0 ? m_tokens[m_separator + offset + 1] : "";
  }

 private:
  const ParameterList* m_list;
  std::vector<const char*> m_tokens;
  size_t m_index;
  size_t m_separator;
  uint32_t m_iteration;
};

}  // namespace nvh

#endif
