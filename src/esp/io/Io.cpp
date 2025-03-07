// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Io.h"
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Path.h>
#include <Corrade/Utility/String.h>
#include <glob.h>

namespace Cr = Corrade;
namespace esp {
namespace io {

std::string changeExtension(const std::string& filename,
                            const std::string& ext) {
  const Cr::Containers::StringView filenameBase =
      Cr::Utility::Path::splitExtension(filename).first();
  return Cr::Utility::formatString(
      ext.empty() || ext[0] == '.' ? "{}{}" : "{}.{}", filenameBase, ext);

}  // changeExtension

std::string normalizePath(const std::string& srcPath) {
  std::size_t ellipsisLoc = srcPath.find("/../");
  if ((ellipsisLoc == std::string::npos) || (ellipsisLoc == 0)) {
    // not found
    return srcPath;
  }
  // ellipsisLoc is location of first instance of "/../"
  // Get rid of ellipsis inside a specified path, and the previous directory
  // it references, so that if the ellisis spans a link, consumers don't get
  // lost.

  // Get end of path/filename after location of ellipsis
  auto suffixString = srcPath.substr(ellipsisLoc + 4);
  // Get beginning of path up to directory before ellipsis
  std::size_t prevLoc = srcPath.find_last_of('/', ellipsisLoc - 1);
  // Get paths leading up to ellipsis-cancelled path
  auto prefixString = srcPath.substr(0, prevLoc);
  // recurses to get subsequent ellipses.
  auto filteredPath =
      Cr::Utility::formatString("{}/{}", prefixString, suffixString);
  return normalizePath(filteredPath);
}  // normalizePath

std::string getPathRelativeToAbsPath(const std::string& toRelPath,
                                     const std::string& absPathArg) {
  // Check if absPath is a path or filename - only use the path if it is a
  // filenaame
  const std::string absPath = Cr::Utility::Path::path(absPathArg);

  std::string result = "";
  const char* delim = "/";

  std::vector<std::string> absDirs =
                               Cr::Utility::String::splitWithoutEmptyParts(
                                   absPath, delim[0]),
                           relDirs =
                               Cr::Utility::String::splitWithoutEmptyParts(
                                   toRelPath, delim[0]);
  auto absIter = absDirs.cbegin();
  auto relIter = relDirs.cbegin();

  // find where both paths diverge - skip shared path components
  while (absIter != absDirs.cend() && relIter != relDirs.cend() &&
         *relIter == *absIter) {
    ++relIter;
    ++absIter;
  }

  // Add back-path components for each directory in abspath not found in
  // toRelPath
  while (absIter != absDirs.cend()) {
    Cr::Utility::formatInto(result, result.size(), "..{}", delim);

    ++absIter;
  }
  // Relative tail of path
  std::string relTail = "";
  while (relIter != relDirs.cend()) {
    if (*relIter == *relDirs.crbegin()) {
      Cr::Utility::formatInto(result, result.size(), "{}{}", relTail,
                              *relDirs.crbegin());
    } else {
      Cr::Utility::formatInto(relTail, relTail.size(), "{}{}", *relIter, delim);
    }
    ++relIter;
  }
  return result;
}  // getPathRelativeToAbsPath

std::vector<std::string> globDirs(const std::string& pattern) {
  // Check for ellipsis, if so process here.
  glob_t glob_result;

  glob(pattern.c_str(), GLOB_MARK, nullptr, &glob_result);
  std::vector<std::string> ret(glob_result.gl_pathc);
  for (int i = 0; i < glob_result.gl_pathc; ++i) {
    ret[i] = std::string(glob_result.gl_pathv[i]);
  }
  globfree(&glob_result);
  return ret;
}

}  // namespace io
}  // namespace esp
