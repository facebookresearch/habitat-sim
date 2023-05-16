// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DEBUGLINERENDER_H_
#define ESP_GFX_DEBUGLINERENDER_H_

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/Macros.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Shaders/FlatGL.h>

#include <memory>
#include <vector>

namespace esp {
namespace gfx {

/**
 * @brief Singleton utility class for on-the-fly rendering of lines (e.g. every
 * frame). This is intended for debugging or simple UX for prototype apps. The
 * API prioritizes ease-of-use over maximum runtime performance.
 *
 * It's easy to add new primitives here; see drawCircle as a reference. In
 * addition, if you're interested to integrate Magnum's line-based primitives,
 * see src/deps/magnum/doc/generated/primitives.cpp and also this discussion:
 * https://github.com/facebookresearch/habitat-sim/pull/1349#discussion_r660092144
 */
class DebugLineRender {
 public:
  /**
   * @brief Constructor. This allocates GPU resources so it should persist
   * frame-to-frame (don't recreate it every frame).
   */
  DebugLineRender();

  /** @brief Release GPU resources */
  void releaseGLResources();

  /** @brief Copying is not allowed */
  DebugLineRender(const DebugLineRender&) = delete;

  /** @brief Copying is not allowed */
  DebugLineRender& operator=(const DebugLineRender&) = delete;

  /**
   * @brief Set line width in pixels (approximate).
   */
  void setLineWidth(float lineWidth);

  /**
   * @brief Draw a line segment in world-space (ignores pushTransform).
   */
  void drawLine(const Magnum::Vector3& from,
                const Magnum::Vector3& to,
                const Magnum::Color4& color) {
    drawLine(from, to, color, color);
  }

  /**
   * @brief Draw a line segment in world-space.
   */
  void drawLine(const Magnum::Vector3& from,
                const Magnum::Vector3& to,
                const Magnum::Color4& fromColor,
                const Magnum::Color4& toColor);

  /**
   * @brief Submit lines to the GL renderer. Call this once per frame.
   * Because this uses transparency, you should ideally call this *after*
   * submitting opaque scene geometry.
   */
  void flushLines(const Magnum::Matrix4& camMatrix,
                  const Magnum::Matrix4& projMatrix,
                  const Magnum::Vector2i& viewport);

  void flushLines(const Magnum::Matrix4& projCamMatrix,
                  const Magnum::Vector2i& viewport);

  /**
   * @brief Push (multiply) a transform onto the transform stack, affecting all
   * line-drawing until popped. Must be paired with popTransform(). For example,
   * push an object's local-to-world transform and then use drawTransformedLine
   * to draw lines in the object's local space.
   */
  void pushTransform(const Magnum::Matrix4& transform);

  /**
   * @brief See also pushTransform.
   */
  void popTransform();

  /**
   * @brief Draw a line segment in world-space or local-space (see
   * pushTransform).
   */
  void drawTransformedLine(const Magnum::Vector3& from,
                           const Magnum::Vector3& to,
                           const Magnum::Color4& color) {
    drawTransformedLine(from, to, color, color);
  }

  /**
   * @brief Draw a line segment in world-space or local-space (see
   * pushTransform).
   */
  void drawTransformedLine(const Magnum::Vector3& from,
                           const Magnum::Vector3& to,
                           const Magnum::Color4& fromColor,
                           const Magnum::Color4& toColor);

  /**
   * @brief Draw a box in world-space or local-space (see pushTransform).
   */
  void drawBox(const Magnum::Vector3& min,
               const Magnum::Vector3& max,
               const Magnum::Color4& color);

  /**
   * @brief Draw a circle in world-space or local-space (see pushTransform).
   * The circle is an approximation; see numSegments.
   */
  void drawCircle(const Magnum::Vector3& pos,
                  float radius,
                  const Magnum::Color4& color,
                  int numSegments = 24,
                  const Magnum::Vector3& normal = Magnum::Vector3(0.0,
                                                                  1.0,
                                                                  0.0));

  /**
   * @brief Draw a sequence of line segments with circles at the two endpoints.
   * In world-space or local-space (see pushTransform).
   */
  void drawPathWithEndpointCircles(
      const std::vector<Magnum::Vector3>& points,
      float radius,
      const Magnum::Color4& color,
      int numSegments = 24,
      const Magnum::Vector3& normal = Magnum::Vector3(0.0, 1.0, 0.0)) {
    drawPathWithEndpointCircles(
        Magnum::Containers::ArrayView<const Magnum::Vector3>(points), radius,
        color, numSegments, normal);
  }

  /**
   * @brief Draw a sequence of line segments with circles at the two endpoints.
   * In world-space or local-space (see pushTransform).
   *
   * @param points Note that std::vector, std::array, or c-style array can be
   * passed here.
   *
   */
  void drawPathWithEndpointCircles(
      Magnum::Containers::ArrayView<const Magnum::Vector3> points,
      float radius,
      const Magnum::Color4& color,
      int numSegments = 24,
      const Magnum::Vector3& normal = Magnum::Vector3(0.0, 1.0, 0.0));

 private:
  void updateCachedInputTransform();

  struct VertexRecord {
    Magnum::Vector3 pos;
    Magnum::Color4 color;
  };

  struct GLResourceSet {
    Magnum::GL::Buffer buffer;
    Magnum::GL::Mesh mesh{Magnum::GL::MeshPrimitive::Lines};
    Magnum::Shaders::FlatGL3D shader{
        Magnum::Shaders::FlatGL3D::Configuration{}.setFlags(
            Magnum::Shaders::FlatGL3D::Flag::VertexColor)};
  };

  std::vector<Magnum::Matrix4> _inputTransformStack;
  Magnum::Matrix4 _cachedInputTransform{Magnum::Math::IdentityInit};
  float _internalLineWidth = 1.0f;
  std::unique_ptr<GLResourceSet> _glResources;
  Magnum::Containers::Array<VertexRecord> _verts;
};

}  // namespace gfx
}  // namespace esp

#endif
