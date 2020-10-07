// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributes.h"
#include "esp/physics/RigidBase.h"
namespace esp {
namespace metadata {
namespace attributes {
// All keys must be lowercase
const std::map<std::string, esp::physics::MotionType>
    SceneObjectInstanceAttributes::MotionTypeNamesMap = {
        {"static", esp::physics::MotionType::STATIC},
        {"kinematic", esp::physics::MotionType::KINEMATIC},
        {"dynamic", esp::physics::MotionType::DYNAMIC},
};

const std::string SceneAttributes::JSONConfigTestString =
    R"({
      "stage_instance":{
          "template_name": "test_stage_template",
          "translation": [1,2,3],
          "rotation": [0.1, 0.2, 0.3, 0.4]
      },
      "object_instances": [
          {
              "template_name": "test_object_template0",
              "translation": [0,1,2],
              "rotation": [0.2, 0.3, 0.4, 0.5],
              "motion_type": "KINEMATIC"
          },
          {
              "template_name": "test_object_template1",
              "translation": [0,-1,-2],
              "rotation": [0.5, 0.6, 0.7, 0.8],
              "motion_type": "DYNAMIC"
          }
      ],
      "default_lighting":  "test_lighting_configuration",
      "navmesh_instance": "test_navmesh_path1",
      "semantic_scene_instance": "test_semantic_descriptor_path1"
     })";

SceneObjectInstanceAttributes::SceneObjectInstanceAttributes(
    const std::string& handle)
    : AbstractAttributes("SceneObjectInstanceAttributes", handle) {
  setMotionType(-1);  // defaults to unknown
}
SceneAttributes::SceneAttributes(const std::string& handle)
    : AbstractAttributes("SceneAttributes", handle) {}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
