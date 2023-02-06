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
 * SPDX-FileCopyrightText: Copyright (c) 2014-2022 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "nvprint.hpp"

static constexpr int MAX_LINE_WIDTH = 60;

namespace nvh {
//--------------------------------------------------------------------------------------------------
// Command line parser.
//  std::string inFilename = "";
//  bool printHelp = false;
//  CommandLineParser args("Test Parser");
//  args.addArgument({"-f", "--filename"}, &inFilename, "Input filename");
//  args.addArgument({"-h", "--help"}, &printHelp, "Print Help");
//  bool result = args.parse(argc, argv);
//
class CommandLineParser {
 public:
  // These are the possible variables the options may point to. Bool and
  // std::string are handled in a special way, all other values are parsed
  // with a std::stringstream. This std::variant can be easily extended if
  // the stream operator>> is overloaded. If not, you have to add a special
  // case to the parse() method.
  using Value =
      std::variant<int32_t*, uint32_t*, double*, float*, bool*, std::string*>;

  // The description is printed as part of the help message.
  CommandLineParser(const std::string& description)
      : m_description(description) {}

  void addArgument(std::vector<std::string> const& flags,
                   Value const& value,
                   std::string const& help) {
    m_arguments.emplace_back(Argument{flags, value, help});
  }

  // Prints the description given to the constructor and the help for each
  // option.
  void printHelp(std::ostream& os = std::cout) const {
    // Print the general description.
    os << m_description << std::endl;

    // Find the argument with the longest combined flag length (in order to
    // align the help messages).
    uint32_t maxFlagLength = 0;
    for (auto const& argument : m_arguments) {
      uint32_t flagLength = 0;
      for (auto const& flag : argument.m_flags) {
        // Plus comma and space.
        flagLength += static_cast<uint32_t>(flag.size()) + 2;
      }

      maxFlagLength = std::max(maxFlagLength, flagLength);
    }

    // Now print each argument.
    for (auto const& argument : m_arguments) {
      std::string flags;
      for (auto const& flag : argument.m_flags) {
        flags += flag + ", ";
      }

      // Remove last comma and space and add padding according to the longest
      // flags in order to align the help messages.
      std::stringstream sstr;
      sstr << std::left << std::setw(maxFlagLength)
           << flags.substr(0, flags.size() - 2);

      // Print the help for each argument. This is a bit more involved since we
      // do line wrapping for long descriptions.
      size_t spacePos = 0;
      size_t lineWidth = 0;
      while (spacePos != std::string::npos) {
        size_t nextspacePos = argument.m_help.find_first_of(' ', spacePos + 1);
        sstr << argument.m_help.substr(spacePos, nextspacePos - spacePos);
        lineWidth += nextspacePos - spacePos;
        spacePos = nextspacePos;

        if (lineWidth > MAX_LINE_WIDTH) {
          os << sstr.str() << std::endl;
          sstr = std::stringstream();
          sstr << std::left << std::setw(maxFlagLength - 1) << " ";
          lineWidth = 0;
        }
      }
    }
  }

  // The command line arguments are traversed from start to end. That means,
  // if an option is set multiple times, the last will be the one which is
  // finally used. This call will throw a std::runtime_error if a value is
  // missing for a given option. Unknown flags will cause a warning on
  // std::cerr.
  bool parse(int argc, char* argv[]) {
    bool result = true;

    // Skip the first argument (name of the program).
    int i = 1;
    while (i < argc) {
      // First we have to identify whether the value is separated by a space or
      // a '='.
      std::string flag(argv[i]);
      std::string value;
      bool valueIsSeparate = false;

      // If there is an '=' in the flag, the part after the '=' is actually
      // the value.
      size_t equalPos = flag.find('=');
      if (equalPos != std::string::npos) {
        value = flag.substr(equalPos + 1);
        flag = flag.substr(0, equalPos);
      }
      // Else the following argument is the value.
      else if (i + 1 < argc) {
        value = argv[i + 1];
        valueIsSeparate = true;
      }

      // Search for an argument with the provided flag.
      bool foundArgument = false;

      for (auto const& argument : m_arguments) {
        if (std::find(argument.m_flags.begin(), argument.m_flags.end(), flag) !=
            std::end(argument.m_flags)) {
          foundArgument = true;

          // In the case of booleans, the value is not needed.
          if (std::holds_alternative<bool*>(argument.m_value)) {
            if (!value.empty() && value != "true" && value != "false") {
              valueIsSeparate = false;  // No value
            }
            *std::get<bool*>(argument.m_value) = (value != "false");
          }
          // In all other cases there must be a value.
          else if (value.empty()) {
            LOGE(
                "Failed to parse command line arguments. Missing value for "
                "argument %s\n",
                flag.c_str());
            return false;
          }
          // For a std::string, we take the entire value.
          else if (std::holds_alternative<std::string*>(argument.m_value)) {
            *std::get<std::string*>(argument.m_value) = value;
          }
          // In all other cases we use a std::stringstream to convert the value.
          else {
            std::visit(
                [&value](auto&& arg) {
                  std::stringstream sstr(value);
                  sstr >> *arg;
                },
                argument.m_value);
          }

          break;
        }
      }

      // Print a warning if there was an unknown argument.
      if (!foundArgument) {
        std::cerr << "Ignoring unknown command line argument \"" << flag
                  << "\"." << std::endl;
        result = false;
      }

      // Advance to the next flag.
      ++i;

      // If the value was separated, we have to advance our index once more.
      if (foundArgument && valueIsSeparate) {
        ++i;
      }
    }

    return result;
  }

 private:
  struct Argument {
    std::vector<std::string> m_flags;
    Value m_value;
    std::string m_help;
  };

  std::string m_description;
  std::vector<Argument> m_arguments;
};

}  // namespace nvh
