// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/io/json.h"

#include <Corrade/Utility/String.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "esp/core/esp.h"

namespace Cr = Corrade;

namespace esp {
namespace io {

bool writeJsonToFile(const JsonDocument& document,
                     const std::string& filepath) {
  assert(!filepath.empty());
  std::string outFilePath = filepath;
  if (!Cr::Utility::String::endsWith(outFilePath, ".json")) {
    outFilePath += ".json";
  }

  auto* f = fopen(outFilePath.c_str(), "w");
  if (!f) {
    return false;
  }

  char writeBuffer[65536];
  rapidjson::FileWriteStream os(f, writeBuffer, sizeof(writeBuffer));

  rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(os);
  bool writeSuccess = document.Accept(writer);
  fclose(f);

  return writeSuccess;
}

JsonDocument parseJsonFile(const std::string& file) {
  FILE* pFile = fopen(file.c_str(), "rb");
  char buffer[65536];
  rapidjson::FileReadStream is(pFile, buffer, sizeof(buffer));
  JsonDocument d;
  d.ParseStream<0, rapidjson::UTF8<>, rapidjson::FileReadStream>(is);
  fclose(pFile);

  if (d.HasParseError()) {
    LOG(ERROR) << "Parse error reading " << file << " Error code "
               << d.GetParseError() << " at " << d.GetErrorOffset();
    throw std::runtime_error("JSON parse error");
  }
  return d;
}

JsonDocument parseJsonString(const std::string& jsonString) {
  JsonDocument d;
  d.Parse(jsonString.c_str());

  if (d.HasParseError()) {
    LOG(ERROR) << "Parse error parsing json string. Error code "
               << d.GetParseError() << " at " << d.GetErrorOffset();
    throw std::runtime_error("JSON parse error");
  }
  return d;
}

std::string jsonToString(const JsonDocument& d) {
  rapidjson::StringBuffer buffer{};
  rapidjson::Writer<rapidjson::StringBuffer> writer{buffer};
  d.Accept(writer);
  return buffer.GetString();
}

vec3f jsonToVec3f(const JsonGenericValue& jsonArray) {
  vec3f vec;
  size_t dim = 0;
  ASSERT(jsonArray.GetArray().Size() == vec.size());
  for (const auto& element : jsonArray.GetArray()) {
    vec[dim++] = element.GetFloat();
  }
  return vec;
}

}  // namespace io
}  // namespace esp
