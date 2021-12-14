// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_METADATAUTILS_H_
#define ESP_METADATA_METADATAUTILS_H_

#include "esp/io/Json.h"

namespace esp {
namespace metadata {

/**
 * @brief This function is accessed by object and stage attributes loading, as
 * well as scene instance loading.  This function will process data held in a
 * json doc with the tag "shader_type", and return the int value found, or the
 * int value of @ref
 * esp::metadata::attributes::ObjectInstanceShaderType::Unknown if no data
 * found.
 * @param jsonDoc The json document to query for the tag of interest
 * @return The shader type specified in the document, or Unknown, casted to an
 * int.
 */
std::string getShaderTypeFromJsonDoc(const io::JsonGenericValue& jsonDoc);

}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_METADATAUTILS_H_
