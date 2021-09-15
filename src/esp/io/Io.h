// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_IO_H_
#define ESP_IO_IO_H_

#include <string>
#include <vector>

namespace esp {
namespace io {

/**
 * @brief Replaces the extension of the given @p filename to the given @p ext.
 * @param filename The fully-qualified filename to change the extension to.
 * @param ext The extension to change to. If it is not prefixed wtih a dot, one
 * will be added.
 */
std::string changeExtension(const std::string& file, const std::string& ext);

/**
 * @brief This function will perform [glob-based pattern matching]
 * (https://en.wikipedia.org/wiki/Glob_(programming)) to find and return all the
 * files and directories that match the pattern.
 * @param pattern The pattern to match
 * @return a vector of the fully-qualified paths that match the pattern.
 */
std::vector<std::string> globDirs(const std::string& pattern);

/** @brief Tokenize input string by any delimiter char in delimiterCharList.
 *
 * @param delimiterCharList string containing all delimiter chars
 * @param limit > 0 indicates maximum number of times delimiter is applied
 * @param mergeAdjacentDelimiters whether to merge adjacent delimiters
 * @return std::vector<std::string>> of tokens
 */
std::vector<std::string> tokenize(const std::string& string,
                                  const std::string& delimiterCharList,
                                  int limit = 0,
                                  bool mergeAdjacentDelimiters = false);

}  // namespace io
}  // namespace esp

#endif  // ESP_IO_IO_H_
