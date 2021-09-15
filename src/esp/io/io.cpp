// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "io.h"
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/FormatStl.h>
#include <glob.h>
#include <fstream>
#include <set>

namespace Cr = Corrade;
namespace esp {
namespace io {

#ifdef _WIN32
#include <io.h>
#define access _access_s
#else
#include <unistd.h>
#endif

std::string changeExtension(const std::string& filename,
                            const std::string& ext) {
  const std::string filenameBase =
      Cr::Utility::Directory::splitExtension(filename).first;
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

std::vector<std::string> tokenize(const std::string& string,
                                  const std::string& delimiterCharList,
                                  int limit /* = 0 */,
                                  bool mergeAdjDelims /* = false */) {
  std::vector<std::string> tokens;
  if (string.length() == 0) {
    return tokens;
  }
  const std::set<char> delimiterSet(delimiterCharList.begin(),
                                    delimiterCharList.end());
  int start = 0;
  int pos = 0;
  bool done = false;
  for (pos = 0; pos < string.length(); ++pos) {
    if (delimiterSet.count(string[pos]) != 0u) {
      if (pos > start) {
        tokens.push_back(string.substr(start, pos - start));
      } else if (!mergeAdjDelims || start == 0) {
        tokens.emplace_back("");
      }
      done = (limit > 1 && tokens.size() == limit - 1);
      start = pos + 1;
    }
    if (done) {
      break;
    }
  }
  // Done (collect remainder)
  pos = static_cast<int>(string.length());
  if (pos > start) {
    tokens.push_back(string.substr(start, pos - start));
  }
  return tokens;
}

}  // namespace io
}  // namespace esp
