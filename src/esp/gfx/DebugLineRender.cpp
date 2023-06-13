// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DebugLineRender.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>

#include "esp/core/Check.h"
#include "esp/core/Logging.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

namespace {

Mn::Color4 remapAlpha(const Mn::Color4& src) {
  // Because we render lines multiple times additively in flushLines, we need to
  // remap alpha (opacity). This is an approximation.
  constexpr float exponent = 2.f;
  return {src.rgb(), std::pow(src.a(), exponent)};
}

// return false if segment is entirely clipped
bool scissorSegmentToOutsideCircle(Mn::Vector3* pt0,
                                   Mn::Vector3* pt1,
                                   const Mn::Vector3& center,
                                   float radius) {
  float dist0 = (*pt0 - center).length();
  float dist1 = (*pt1 - center).length();
  if (dist0 < radius) {
    if (dist1 < radius) {
      return false;
    }
    const float lerpFraction = Mn::Math::lerpInverted(dist0, dist1, radius);
    CORRADE_INTERNAL_ASSERT(lerpFraction >= 0.f && lerpFraction <= 1.f);
    Mn::Vector3 clippedPt = Mn::Math::lerp(*pt0, *pt1, lerpFraction);
    *pt0 = clippedPt;
    return true;
  }

  if (dist1 < radius) {
    const float lerpFraction = Mn::Math::lerpInverted(dist1, dist0, radius);
    CORRADE_INTERNAL_ASSERT(lerpFraction >= 0.f && lerpFraction <= 1.f);
    Mn::Vector3 clippedPt = Mn::Math::lerp(*pt1, *pt0, lerpFraction);
    *pt1 = clippedPt;
    return true;
  }

  return true;
}

}  // namespace

DebugLineRender::DebugLineRender()
    : _glResources{std::make_unique<GLResourceSet>()} {
  _glResources->mesh.addVertexBuffer(_glResources->buffer, 0,
                                     Mn::Shaders::FlatGL3D::Position{},
                                     Mn::Shaders::FlatGL3D::Color4{});
}

void DebugLineRender::releaseGLResources() {
  _glResources = nullptr;
}

void DebugLineRender::drawLine(const Mn::Vector3& from,
                               const Mn::Vector3& to,
                               const Mn::Color4& fromColor,
                               const Mn::Color4& toColor) {
  VertexRecord v1{from, remapAlpha(fromColor)};
  VertexRecord v2{to, remapAlpha(toColor)};
  arrayAppend(_verts, {v1, v2});
}

void DebugLineRender::setLineWidth(float lineWidth) {
  // This is derived from experiments with glLineWidth on Nvidia hardware.
  const float maxLineWidth = 20.f;
  if (lineWidth > maxLineWidth) {
    ESP_WARNING() << "Requested lineWidth of" << lineWidth
                  << "is greater than max supported width of" << maxLineWidth;
    lineWidth = maxLineWidth;
  }
  _internalLineWidth = lineWidth / 2;  // see also DebugLineRender::flushLines
}

void DebugLineRender::flushLines(const Magnum::Matrix4& camMatrix,
                                 const Magnum::Matrix4& projMatrix,
                                 const Magnum::Vector2i& viewport) {
  flushLines(projMatrix * camMatrix, viewport);
}

void DebugLineRender::flushLines(const Magnum::Matrix4& projCamMatrix,
                                 const Magnum::Vector2i& viewport) {
  CORRADE_ASSERT(_glResources,
                 "DebugLineRender::flushLines: no GL resources; see "
                 "also releaseGLResources", );

  if (_verts.isEmpty()) {
    return;
  }

  bool doToggleBlend = !glIsEnabled(GL_BLEND);

  if (doToggleBlend) {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
    Mn::GL::Renderer::setBlendFunction(
        Mn::GL::Renderer::BlendFunction::SourceAlpha,
        Mn::GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    Mn::GL::Renderer::setBlendEquation(Mn::GL::Renderer::BlendEquation::Add);
  }

  // Update buffer with new data
  _glResources->buffer.setData(_verts, Mn::GL::BufferUsage::DynamicDraw);

  // Update shader
  _glResources->mesh.setCount(_verts.size());

  Mn::GL::Renderer::setLineWidth(_internalLineWidth);

  // We draw lines multiple times, with pixel offsets, to produce a single
  // thick, visually-appealing line. Todo: implement thick lines using
  // triangles, for example https://mattdesl.svbtle.com/drawing-lines-is-hard.
  // 1.2 is hand-tuned for Nvidia hardware to be just small enough so we don't
  // see gaps between the individual offset lines.
  const float x = _internalLineWidth * 1.2f;
  constexpr float sqrtOfTwo = Mn::Constants::sqrt2();
  // hard-coding 8 points around a circle
  const Mn::Vector3 offsets[] = {Mn::Vector3(x, x, 0),
                                 Mn::Vector3(-x, x, 0),
                                 Mn::Vector3(x, -x, 0),
                                 Mn::Vector3(-x, -x, 0),
                                 Mn::Vector3(x * sqrtOfTwo, 0, 0),
                                 Mn::Vector3(-x * sqrtOfTwo, 0, 0),
                                 Mn::Vector3(0, x * sqrtOfTwo, 0),
                                 Mn::Vector3(0, -x * sqrtOfTwo, 0)};

  const auto& projCam = projCamMatrix;

  auto submitLinesWithOffsets = [&]() {
    for (const auto& offset : offsets) {
      // We want to offset in viewport (pixel) space, but we need to specify our
      // offset transform in projection space. So divide by viewport dim.
      Magnum::Matrix4 offset0Matrix = Mn::Matrix4::translation(
          offset * Mn::Vector3(1.f / viewport.x(), 1.f / viewport.y(), 0.f));
      Magnum::Matrix4 transProj = offset0Matrix * projCam;
      _glResources->shader.setTransformationProjectionMatrix(transProj);
      _glResources->shader.draw(_glResources->mesh);
    }
  };

  _glResources->shader.setColor({1.0f, 1.0f, 1.0f, 1.0});

  submitLinesWithOffsets();

  // modify all colors to be semi-transparent
  static float opacity = 0.1;
  _glResources->shader.setColor({1.0f, 1.0f, 1.0f, opacity});

  // Here, we re-draw lines with a reversed depth function. This causes
  // occluded lines to be visualized as semi-transparent, which is useful for
  // UX and visually-appealing.
  Mn::GL::Renderer::setDepthFunction(Mn::GL::Renderer::DepthFunction::Greater);
  submitLinesWithOffsets();

  // restore to a reasonable default
  Mn::GL::Renderer::setDepthFunction(Mn::GL::Renderer::DepthFunction::Less);

  // Clear _verts to receive new data
  arrayResize(_verts, 0);

  // restore blending state if necessary
  if (doToggleBlend) {
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
    // Note: because we are disabling blending, we don't need to restore
    // BlendFunction or BlendEquation
  }

  // restore to a reasonable default
  Mn::GL::Renderer::setLineWidth(1.0);
}

void DebugLineRender::pushTransform(const Magnum::Matrix4& transform) {
  _inputTransformStack.push_back(transform);
  updateCachedInputTransform();
}

void DebugLineRender::popTransform() {
  CORRADE_INTERNAL_ASSERT(!_inputTransformStack.empty());
  _inputTransformStack.pop_back();
  updateCachedInputTransform();
}

void DebugLineRender::drawTransformedLine(const Magnum::Vector3& from,
                                          const Magnum::Vector3& to,
                                          const Magnum::Color4& fromColor,
                                          const Magnum::Color4& toColor) {
  Mn::Vector3 fromTransformed = _cachedInputTransform.transformPoint(from);
  Mn::Vector3 toTransformed = _cachedInputTransform.transformPoint(to);
  drawLine(fromTransformed, toTransformed, fromColor, toColor);
}

void DebugLineRender::drawBox(const Magnum::Vector3& min,
                              const Magnum::Vector3& max,
                              const Magnum::Color4& color) {
  // 4 lines along x axis
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), min.z()),
                      Mn::Vector3(max.x(), min.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), max.z()),
                      Mn::Vector3(max.x(), min.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), max.y(), min.z()),
                      Mn::Vector3(max.x(), max.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), max.y(), max.z()),
                      Mn::Vector3(max.x(), max.y(), max.z()), color);

  // 4 lines along y axis
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), min.z()),
                      Mn::Vector3(min.x(), max.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), min.y(), min.z()),
                      Mn::Vector3(max.x(), max.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), max.z()),
                      Mn::Vector3(min.x(), max.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), min.y(), max.z()),
                      Mn::Vector3(max.x(), max.y(), max.z()), color);

  // 4 lines along z axis
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), min.z()),
                      Mn::Vector3(min.x(), min.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), min.y(), min.z()),
                      Mn::Vector3(max.x(), min.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), max.y(), min.z()),
                      Mn::Vector3(min.x(), max.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), max.y(), min.z()),
                      Mn::Vector3(max.x(), max.y(), max.z()), color);
}

void DebugLineRender::drawCircle(const Magnum::Vector3& pos,
                                 float radius,
                                 const Magnum::Color4& color,
                                 int numSegments,
                                 const Magnum::Vector3& normal) {
  // https://stackoverflow.com/questions/11132681/what-is-a-formula-to-get-a-vector-perpendicular-to-another-vector
  auto randomPerpVec = normal.z() < normal.x()
                           ? Mn::Vector3(normal.y(), -normal.x(), 0)
                           : Mn::Vector3(0, -normal.z(), normal.y());

  pushTransform(Mn::Matrix4::lookAt(pos, pos + normal, randomPerpVec) *
                Mn::Matrix4::scaling(Mn::Vector3(radius, radius, 0.f)));

  Mn::Vector3 prevPt;
  for (int seg = 0; seg <= numSegments; ++seg) {
    Mn::Deg angle = Mn::Deg(360.f * float(seg) / numSegments);
    Mn::Vector3 pt(Mn::Math::cos(angle), Mn::Math::sin(angle), 0.f);
    if (seg > 0) {
      drawTransformedLine(prevPt, pt, color);
    }
    prevPt = pt;
  }

  popTransform();
}

void DebugLineRender::drawPathWithEndpointCircles(
    Mn::Containers::ArrayView<const Mn::Vector3> points,
    float radius,
    const Magnum::Color4& color,
    int numSegments,
    const Magnum::Vector3& normal) {
  ESP_CHECK(points.size() >= 2,
            "drawPathWithEndpointCircles requires at least two points");

  const auto& end0 = points.front();
  const auto& end1 = points.back();

  drawCircle(end0, radius, color, numSegments, normal);
  drawCircle(end1, radius, color, numSegments, normal);

  Mn::Vector3 prevPos;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& pos = points[i];
    if (i > 0) {
      if ((prevPos - end0).length() > radius &&
          (prevPos - end1).length() > radius &&
          (pos - end0).length() > radius && (pos - end1).length() > radius) {
        drawTransformedLine(prevPos, pos, color);
      } else {
        // we must scissor one or both points to lie on the end circles
        auto pt0 = prevPos;
        auto pt1 = pos;
        if (!scissorSegmentToOutsideCircle(&pt0, &pt1, end0, radius)) {
          continue;
        }
        if (!scissorSegmentToOutsideCircle(&pt0, &pt1, end1, radius)) {
          continue;
        }
        drawTransformedLine(pt0, pt1, color);
      }
    }
    prevPos = pos;
  }
}

void DebugLineRender::updateCachedInputTransform() {
  _cachedInputTransform = Mn::Matrix4{Magnum::Math::IdentityInit};
  for (const auto& item : _inputTransformStack) {
    _cachedInputTransform = _cachedInputTransform * item;
  }
}

}  // namespace gfx
}  // namespace esp
