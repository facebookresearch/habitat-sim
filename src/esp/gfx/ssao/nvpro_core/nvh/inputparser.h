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

//--------------------------------------------------------------------------------------------------
/**
  \class InputParser
  \brief InputParser is a Simple command line parser

  Example of usage for: test.exe -f name.txt -size 200 100

  Parsing the command line: mandatory '-f' for the filename of the scene

  \code{.cpp}
  nvh::InputParser parser(argc, argv);
  std::string filename = parser.getString("-f");
  if(filename.empty())  filename = "default.txt";
  if(parser.exist("-size") {
        auto values = parser.getInt2("-size");
  \endcode
*/

#pragma once
#include <string>
#include <vector>

class InputParser {
 public:
  InputParser(int& argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
      if (argv[i]) {
        m_tokens.emplace_back(argv[i]);
      }
    }
  }

  auto findOption(const std::string& option) const {
    return std::find(m_tokens.begin(), m_tokens.end(), option);
  }
  const std::string getString(const std::string& option,
                              std::string defaultString = "") const {
    if (exist(option)) {
      auto itr = findOption(option);
      if (itr != m_tokens.end() && ++itr != m_tokens.end()) {
        return *itr;
      }
    }

    return defaultString;
  }

  std::vector<std::string> getString(const std::string& option,
                                     uint32_t nbElem) const {
    auto itr = findOption(option);
    std::vector<std::string> items;
    while (itr != m_tokens.end() && ++itr != m_tokens.end() && nbElem-- > 0) {
      items.push_back((*itr));
    }
    return items;
  }

  int getInt(const std::string& option, int defaultValue = 0) const {
    if (exist(option))
      return std::stoi(getString(option));
    return defaultValue;
  }

  auto getInt2(const std::string& option,
               std::array<int, 2> defaultValues = {0, 0}) const {
    if (exist(option)) {
      auto items = getString(option, 2);
      if (items.size() == 2) {
        defaultValues[0] = std::stoi(items[0]);
        defaultValues[1] = std::stoi(items[1]);
      }
    }

    return defaultValues;
  }

  float getFloat(const std::string& option, float defaultValue = 0.0f) const {
    if (exist(option))
      return std::stof(getString(option));

    return defaultValue;
  }

  bool exist(const std::string& option) const {
    return findOption(option) != m_tokens.end();
  }

 private:
  std::vector<std::string> m_tokens;
};
