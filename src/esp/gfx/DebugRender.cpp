// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DebugRender.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/Math/Color.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

DebugRender::DebugRender(const std::size_t initialBufferCapacity)
    : _mesh{Mn::GL::MeshPrimitive::Lines} {
  _mesh.addVertexBuffer(_buffer, 0, Mn::Shaders::VertexColor3D::Position{},
                        Mn::Shaders::VertexColor3D::Color3{});
  arrayReserve(_bufferData, initialBufferCapacity * 4);
}

void DebugRender::drawLine(const Mn::Vector3& from,
                           const Mn::Vector3& to,
                           const Mn::Color3& color) {
  drawLine(from, to, color, color);
}

void DebugRender::drawLine(const Mn::Vector3& from,
                           const Mn::Vector3& to,
                           const Mn::Color3& fromColor,
                           const Mn::Color3& toColor) {
  arrayAppend(_bufferData,
              {from, Mn::Vector3(fromColor), to, Mn::Vector3(toColor)});
}

void DebugRender::flushLines() {
  /* Update buffer with new data */
  _buffer.setData(_bufferData, Mn::GL::BufferUsage::DynamicDraw);

  /* Update shader and draw */
  _mesh.setCount(_bufferData.size() / 2);
  _shader.setTransformationProjectionMatrix(_transformationProjectionMatrix)
      .draw(_mesh);

  /* Clear buffer to receive new data */
  arrayResize(_bufferData, 0);
}

void DebugRender::pushInputTransform(const Magnum::Matrix4& transform) {
  _inputTransformStack.push_back(transform);
  updateCachedInputTransform();
}

void DebugRender::popInputTransform() {
  CORRADE_INTERNAL_ASSERT(!_inputTransformStack.empty());
  _inputTransformStack.pop_back();
  updateCachedInputTransform();
}

void DebugRender::drawTransformedLine(const Magnum::Vector3& from,
                                      const Magnum::Vector3& to,
                                      const Magnum::Color3& color) {
  drawTransformedLine(from, to, color, color);
}

void DebugRender::drawTransformedLine(const Magnum::Vector3& from,
                                      const Magnum::Vector3& to,
                                      const Magnum::Color3& fromColor,
                                      const Magnum::Color3& toColor) {
  Mn::Vector3 fromTransformed = _cachedInputTransform.transformPoint(from);
  Mn::Vector3 toTransformed = _cachedInputTransform.transformPoint(to);
  drawLine(fromTransformed, toTransformed, fromColor, toColor);
}

void DebugRender::updateCachedInputTransform() {
  _cachedInputTransform = Mn::Matrix4{Magnum::Math::IdentityInit};
  for (const auto& item : _inputTransformStack) {
    _cachedInputTransform = _cachedInputTransform * item;
  }
}

}  // namespace gfx
}  // namespace esp
