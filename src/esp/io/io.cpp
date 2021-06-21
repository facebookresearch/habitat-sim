// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "io.h"
#include <glob.h>
#include <fstream>
#include <set>

namespace esp {
namespace io {

#ifdef _WIN32
#include <io.h>
#define access _access_s
#else
#include <unistd.h>
#endif

bool exists(const std::string& filename) {
  return (access(filename.c_str(), 0) == 0);
}

// returns the size of the file, if it exists and can be opened,
// otherwise returns 0
// TODO: consider uint64_t as the return value type
size_t fileSize(const std::string& filename) {
  if (!exists(filename)) {
    return 0;
  }

  std::ifstream file(filename.c_str(),
                     std::ifstream::in | std::ifstream::binary);

  if (!file.good()) {
    return 0;
  }

  file.seekg(0, std::ios::end);
  // tellg can return -1 if the stream does not
  // support the operation or it fails
  int size = file.tellg();
  file.close();

  return (size <= 0 ? 0 : size);
}

// TODO:
// a corner case it will fail to match the replace_extension in c++17:
// filename = "foo"
// ext  = "\\png"
// This function returns: foo.\png
// while replace_extension returns: foo.\\png

std::string changeExtension(const std::string& filename,
                            const std::string& ext) {
  std::string correctExt = ext;

  // this is the only requirement on an extension, that is,
  // if it is not empty, it must start with '.'
  if (ext != "" && ext[0] != '.') {
    correctExt = "." + ext;
  }

  // match the replace_extension in c++17 (GCC 8.1, C++2a)
  if (filename == "..") {
    return filename + correctExt;
  }

  size_t lastDot = filename.find_last_of('.');
  // in such case e.g., filename == ".png" or filename == "."
  if (lastDot == 0) {
    // match the replace_extension in c++17 (GCC 8.1, C++2a)
    return filename + correctExt;
  }

  if (lastDot == std::string::npos) {
    return (filename + correctExt);
  } else {
    return (filename.substr(0, lastDot) + correctExt);
  }
}

std::string removeExtension(const std::string& filename) {
  return changeExtension(filename, "");
}

/* The following implementation requires the support of C++17

// #include <filesystem>

namespace fs = std::filesystem;

bool exists(const std::string& file) {
  return fs::exists(file);
}

size_t fileSize(const std::string& file) {
  return fs::file_size(file);
}

std::string changeExtension(const std::string& file, const std::string& ext) {
  return fs::replace_extension(file, ext).string();
}

std::string removeExtension(const std::string& file) {
  return changeExtension(file, "");
}
*/

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
