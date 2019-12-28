// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/io/json.h"
#include "esp/scene/SceneNode.h"

#include <string>

namespace esp {
namespace sensor {
class BaseSensor : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  explicit BaseSensor(scene::SceneNode& node, const esp::io::JsonDocument spec);
  std::string getType();
  std::string getUUID();
  std::vector<char> getObservation();
  std::vector<int> getObservationShape();
  std::string getObservationDataType();
};

class SensorFactory {
 public:
  static BaseSensor create(scene::SceneNode& node,
                           const esp::io::JsonDocument spec);
};
}  // namespace sensor
}  // namespace esp
