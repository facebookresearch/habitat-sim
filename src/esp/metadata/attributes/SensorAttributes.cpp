// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SensorAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

AbstractSensorAttributes::AbstractSensorAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
}  // AbstractSensorAttributes ctor

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
