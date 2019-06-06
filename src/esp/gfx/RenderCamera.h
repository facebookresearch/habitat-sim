// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "magnum.h"

#include "esp/core/esp.h"
#include "esp/scene/AttachedObject.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

class RenderCamera : public scene::AttachedObject {
 public:
  RenderCamera(scene::SceneNode& node);
  RenderCamera(scene::SceneNode& node,
               const vec3f& eye,
               const vec3f& target,
               const vec3f& up);
  virtual ~RenderCamera() {
    // do nothing, let magnum handle the camera
  }

  void setProjectionMatrix(int width,
                           int height,
                           float znear,
                           float zfar,
                           float hfov);

  mat4f getProjectionMatrix();

  mat4f getCameraMatrix();

  MagnumCamera& getMagnumCamera();

  void draw(MagnumDrawableGroup& drawables);

 protected:
  MagnumCamera* camera_ = nullptr;

  ESP_SMART_POINTERS(RenderCamera)
};

}  // namespace gfx
}  // namespace esp
