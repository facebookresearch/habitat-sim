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

std::vector<std::string> globDirs(const std::string& pattern) {
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
