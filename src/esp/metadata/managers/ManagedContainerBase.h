// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_MANAGEDCONTAINERBASE_H_
#define ESP_METADATA_MANAGERS_MANAGEDCONTAINERBASE_H_

/** @file
 * @brief Class Template @ref esp::metadata::managers::AttributesManager
 */

#include <deque>
#include <functional>
#include <map>
#include <set>

#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/metadata/AbstractManagedObject.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/metadata/attributes/ObjectAttributes.h"

#include "esp/io/json.h"

namespace Cr = Corrade;

namespace esp {
namespace assets {
class ResourceManager;
}

namespace metadata {
namespace managers {}  // namespace managers
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_MANAGERS_MANAGEDCONTAINERBASE_H_