// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
 * @brief This function will recursively remove any ellipsis in paths or path
 * patterns, as well as the parent directory the ellisis is bypassing. This is
 * so that if the ellipsis spans an OS link any function consuming the path does
 * not get lost. If the ellipsis is at the beginning of @p srcPath then it is
 * ignored.
 * @param srcPath The path to filter
 * @return The filtered path.
 */
std::string normalizePath(const std::string& srcPath);

/**
 * @brief This function will perform [glob-based pattern matching]
 * (https://en.wikipedia.org/wiki/Glob_(programming)) to find and return all the
 * files and directories that match the pattern.
 * @param pattern The pattern to match
 * @return a vector of the fully-qualified paths that match the pattern.
 */
std::vector<std::string> globDirs(const std::string& pattern);

}  // namespace io
}  // namespace esp

#endif  // ESP_IO_IO_H_
