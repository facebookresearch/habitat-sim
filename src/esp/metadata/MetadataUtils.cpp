// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MetadataUtils.h"

#include <Corrade/Utility/String.h>

#include "esp/io/Io.h"
#include "esp/metadata/attributes/ObjectAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {
enum class ObjectInstanceShaderType;
}
namespace Cr = Corrade;
std::string getShaderTypeFromJsonDoc(const io::JsonGenericValue& jsonDoc) {
  // Check for shader type to use.  Default to unknown.
  std::string shader_type =
      getShaderTypeName(attributes::ObjectInstanceShaderType::Unknown);
  std::string tmpShaderType = "";
  if (io::readMember<std::string>(jsonDoc, "shader_type", tmpShaderType)) {
    // shader_type tag was found, perform check - first convert to
    // lowercase
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpShaderType);
    auto found = attributes::ShaderTypeNamesMap.find(strToLookFor);
    if (found != attributes::ShaderTypeNamesMap.end()) {
      shader_type = std::move(strToLookFor);
    } else {
      ESP_WARNING() << "`shader_type` value in json  : `" << tmpShaderType
                    << "` -> `" << strToLookFor
                    << "` does not map to a valid "
                       "ObjectInstanceShaderType value, so defaulting "
                       "shader type to ObjectInstanceShaderType::Unknown.";
    }
  }
  return shader_type;
}  // getShaderTypeFromJsonDoc

}  // namespace metadata
}  // namespace esp
