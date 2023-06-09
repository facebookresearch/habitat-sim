// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_SSAO_HBAOHELPER
#define ESP_GFX_SSAO_HBAOHELPER

#include "esp/core/Esp.h"

#include <string>

#include <Magnum/GL/GL.h>
#include <Magnum/Magnum.h>

// namespace Magnum { namespace GL {
// class Texture2D;
// class Framebuffer;
// }
// }

namespace esp {
namespace gfx {
namespace ssao {

class HBAOHelper2 {
 public:
  HBAOHelper2();

  ~HBAOHelper2() = default;

  // todo: is there a Magnum smart pointer type I should be using?
  void init(Magnum::GL::Texture2D* depthRenderTexture,
            Magnum::GL::Framebuffer* framebuffer);

  void tempCall(const std::string& name);

  void tempDrawLinearDepth(float projNearPlane,
                           float projFarPlane,
                           bool isProjOrtho,
                           int width,
                           int height,
                           int sampleIdx);

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(HBAOHelper2)
};

}  // namespace ssao
}  // namespace gfx
}  // namespace esp
#endif
