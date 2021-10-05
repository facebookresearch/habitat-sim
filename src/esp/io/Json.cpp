// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/io/Json.h"
#include <Corrade/Containers/Containers.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/StringView.h>
#include <Corrade/Utility/String.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "esp/core/Esp.h"

namespace Cr = Corrade;

namespace esp {
namespace io {

bool writeJsonToFile(const JsonDocument& document,
                     const std::string& filepath,
                     bool usePrettyWriter,
                     int maxDecimalPlaces) {
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

  bool writeSuccess = false;
  if (usePrettyWriter) {
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(os);
    if (maxDecimalPlaces != -1) {
      writer.SetMaxDecimalPlaces(maxDecimalPlaces);
    }
    writeSuccess = document.Accept(writer);
  } else {
    rapidjson::Writer<rapidjson::FileWriteStream> writer(os);
    if (maxDecimalPlaces != -1) {
      writer.SetMaxDecimalPlaces(maxDecimalPlaces);
    }
    writeSuccess = document.Accept(writer);
  }
  fclose(f);

  return writeSuccess;
}

int loadJsonIntoConfiguration(
    const JsonGenericValue& jsonObj,
    const std::shared_ptr<core::config::Configuration>& config) {
  // count number of valid user config settings found

  int numConfigSettings = 0;
  for (rapidjson::Value::ConstMemberIterator it = jsonObj.MemberBegin();
       it != jsonObj.MemberEnd(); ++it) {
    // for each key, attempt to parse
    const std::string key = it->name.GetString();
    const auto& obj = it->value;
    // increment, assuming is valid object
    ++numConfigSettings;

    if (obj.IsDouble()) {
      config->set(key, obj.GetDouble());
    } else if (obj.IsNumber()) {
      config->set(key, obj.GetInt());
    } else if (obj.IsString()) {
      config->set(key, obj.GetString());
    } else if (obj.IsBool()) {
      config->set(key, obj.GetBool());
    } else if (obj.IsArray() && obj.Size() > 0 && obj[0].IsNumber()) {
      // numeric vector or quaternion
      if (obj.Size() == 3) {
        Magnum::Vector3 val{};
        if (io::fromJsonValue(obj, val)) {
          config->set(key, val);
        }
      } else if (obj.Size() == 4) {
        // assume is quaternion
        Magnum::Quaternion val{};
        if (io::fromJsonValue(obj, val)) {
          config->set(key, val);
        }
      } else {
        // decrement count for key:obj due to not being handled vector
        --numConfigSettings;
        // TODO support numeric array in JSON
        ESP_WARNING() << "Config cell in JSON document contains key" << key
                      << "referencing an unsupported numeric array of length :"
                      << obj.Size() << "so skipping.";
      }
    } else if (obj.IsObject()) {
      // support nested objects
      // create a new subgroup
      std::shared_ptr<core::config::Configuration> subGroupPtr =
          config->getSubconfigCopy<core::config::Configuration>(key);
      numConfigSettings += loadJsonIntoConfiguration(obj, subGroupPtr);
      // save subgroup's subgroup configuration in original config
      config->setSubconfigPtr<core::config::Configuration>(key, subGroupPtr);
      //
    } else {
      // TODO support other types?
      // decrement count for key:obj due to not being handled type
      --numConfigSettings;
      ESP_WARNING() << "Config cell in JSON document contains key" << key
                    << "referencing an unknown/unparsable value type, so "
                       "skipping this key.";
    }
  }
  return numConfigSettings;
}  // loadJsonIntoConfigSubgroup

JsonGenericValue configToJsonValue(
    const std::shared_ptr<core::config::Configuration>& configPtr,
    JsonAllocator& allocator) {
  JsonGenericValue jsonObj(rapidjson::kObjectType);
  // iterate through all values
  // pair of begin/end const iterators to all values
  auto valIterPair = configPtr->getValuesIterator();
  auto valBegin = valIterPair.first;
  auto valEnd = valIterPair.second;
  for (auto& valIter = valIterPair.first; valIter != valIterPair.second;
       ++valIter) {
    if (valIter->second.isValid()) {
      // make sure value is legal
      rapidjson::GenericStringRef<char> name{valIter->first.c_str()};
      auto jsonVal = cfgValToJsonValue(valIter->second, allocator);
      jsonObj.AddMember(name, jsonVal, allocator);
    } else {
      ESP_WARNING() << "Unitialized ConfigValue in Configuration @ key ["
                    << valIter->first
                    << "], so nothing will be written to JSON for this key.";
    }
  }  // iterate through all values

  // iterate through subconfigs
  // pair of begin/end const iterators to all subconfigurations
  auto cfgIterPair = configPtr->getSubconfigIterator();
  for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
       ++cfgIter) {
    rapidjson::GenericStringRef<char> name{cfgIter->first.c_str()};
    JsonGenericValue subObj = configToJsonValue(cfgIter->second, allocator);
    jsonObj.AddMember(name, subObj, allocator);
  }  // iterate through all configurations

  return jsonObj;
}  // configToJsonValue

JsonGenericValue cfgValToJsonValue(const core::config::ConfigValue& cfgVal,
                                   JsonAllocator& allocator) {
  // unknown is checked before this function is called, so does not need support
  switch (cfgVal.getType()) {
    case core::config::ConfigStoredType::Boolean: {
      return toJsonValue(cfgVal.get<bool>(), allocator);
    }
    case core::config::ConfigStoredType::Integer: {
      return toJsonValue(cfgVal.get<int>(), allocator);
    }
    case core::config::ConfigStoredType::Double: {
      return toJsonValue(cfgVal.get<double>(), allocator);
    }
    case core::config::ConfigStoredType::MagnumVec3: {
      return toJsonValue(cfgVal.get<Magnum::Vector3>(), allocator);
    }
    case core::config::ConfigStoredType::MagnumQuat: {
      return toJsonValue(cfgVal.get<Magnum::Quaternion>(), allocator);
    }
    case core::config::ConfigStoredType::MagnumRad: {
      auto r = cfgVal.get<Magnum::Rad>();
      return toJsonValue((r.operator float()), allocator);
    }
    case core::config::ConfigStoredType::String: {
      return toJsonValue(cfgVal.get<std::string>(), allocator);
    }
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in io::toJsonValue<ConfigValue>", {});
  }
}  // cfgValToJsonValue

bool writeConfigurationToJsonFile(
    const std::string& filename,
    const std::shared_ptr<esp::core::config::Configuration>& configPtr) {
  rapidjson::Document doc(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
  // build Json from passed Configuration
  auto configJson = configToJsonValue(configPtr, allocator);
  doc.Swap(configJson);

  // save to file
  bool success = writeJsonToFile(doc, filename, true, 7);
  return success;
}  // buildJsonFromConfiguration

JsonDocument parseJsonFile(const std::string& file) {
  FILE* pFile = fopen(file.c_str(), "rb");
  char buffer[65536];
  rapidjson::FileReadStream is(pFile, buffer, sizeof(buffer));
  JsonDocument d;
  d.ParseStream<0, rapidjson::UTF8<>, rapidjson::FileReadStream>(is);
  fclose(pFile);

  if (d.HasParseError()) {
    ESP_ERROR() << "Parse error reading" << file << "Error code"
                << d.GetParseError() << "at" << d.GetErrorOffset();
    throw std::runtime_error("JSON parse error");
  }
  return d;
}

JsonDocument parseJsonString(const std::string& jsonString) {
  JsonDocument d;
  d.Parse(jsonString.c_str());

  if (d.HasParseError()) {
    ESP_ERROR() << "Parse error parsing json string. Error code"
                << d.GetParseError() << "at" << d.GetErrorOffset();
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
  CORRADE_INTERNAL_ASSERT(jsonArray.GetArray().Size() == vec.size());
  for (const auto& element : jsonArray.GetArray()) {
    vec[dim++] = element.GetFloat();
  }
  return vec;
}

}  // namespace io
}  // namespace esp
