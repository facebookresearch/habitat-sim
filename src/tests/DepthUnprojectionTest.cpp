// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/OpenGLTester.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Trade/MeshData.h>

#include "esp/gfx_batch/DepthUnprojection.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using Mn::Math::Literals::operator""_degf;
using Mn::Math::Literals::operator""_rgbf;

namespace {
using namespace esp::gfx_batch;

struct DepthUnprojectionTest : Mn::GL::OpenGLTester {
  explicit DepthUnprojectionTest();

  void testCpu();
  void testGpuDirect();
  void testGpuUnprojectExisting();

  void benchmarkBaseline();
  void benchmarkCpu();
  void benchmarkGpuDirect();
  void benchmarkGpuUnprojectExisting();
};

const struct {
  const char* name;
  float depth, expected;
  Mn::Matrix4 projection;
} TestData[]{
    {"z=4, near=0.01, far=100", 4.0f, 4.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=4, near=1.0, far=100", 4.0f, 4.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 1.0f, 100.0f)},
    {"z=4, near=0.01, far=10", 4.0f, 4.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 10.0f)},
    {"z=0.015, near=0.015, far=100", 0.015f, 0.015f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=10, near=0.01, far=100", 10.0f, 10.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=95, near=0.01, far=100", 95.0f, 95.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=near", 0.01f, 0.01f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=far", 100.0f, 0.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
};

/* Clang doesn't have target_clones yet: https://reviews.llvm.org/D51650 */
#if defined(CORRADE_TARGET_X86) && defined(__GNUC__) && __GNUC__ >= 6
#define FMV_SUPPORTED
#endif

#ifdef FMV_SUPPORTED
__attribute__((target("default"))) const char* targetName() {
  return "default";
}

__attribute__((target("sse4.2"))) const char* targetName() {
  return "sse4.2";
}

__attribute__((target("avx2"))) const char* targetName() {
  return "avx2";
}
#endif

#ifdef FMV_SUPPORTED
__attribute__((target_clones("default", "sse4.2", "avx2")))
#endif
CORRADE_NEVER_INLINE void
unprojectBaseline(const Mn::Matrix4& unprojection,
                  const Cr::Containers::StridedArrayView2D<float>& depth) {
  for (const Cr::Containers::StridedArrayView1D<float> row : depth) {
    for (float& d : row) {
      if (d == 1.0f) {
        d = 0.0f;
        continue;
      }
      d = -unprojection.transformPoint(Mn::Vector3::zAxis(-d)).z();
    }
  }
}

#ifdef FMV_SUPPORTED
__attribute__((target_clones("default", "sse4.2", "avx2")))
#endif
CORRADE_NEVER_INLINE void
unprojectBaselineNoBranch(
    const Mn::Matrix4& unprojection,
    const Cr::Containers::StridedArrayView2D<float>& depth) {
  for (const Cr::Containers::StridedArrayView1D<float> row : depth) {
    for (float& d : row) {
      d = -unprojection.transformPoint(Mn::Vector3::zAxis(-d)).z();
    }
  }
}

#ifdef FMV_SUPPORTED
__attribute__((target_clones("default", "sse4.2", "avx2")))
#endif
CORRADE_NEVER_INLINE void
unprojectDepthNoBranch(const Mn::Vector2& unprojection,
                       const Cr::Containers::StridedArrayView2D<float>& depth) {
  for (const Cr::Containers::StridedArrayView1D<float> row : depth) {
    for (float& d : row) {
      d = unprojection[1] / (d + unprojection[0]);
    }
  }
}

const struct {
  const char* name;
  void (*unprojectorFull)(const Mn::Matrix4&,
                          const Cr::Containers::StridedArrayView2D<float>&);
  void (*unprojectorOptimized)(
      const Mn::Vector2&,
      const Cr::Containers::StridedArrayView2D<float>&);
  DepthShader::Flags flags;
} UnprojectBenchmarkData[]{
    {"", unprojectBaseline, unprojectDepth, {}},
    {"no branch", unprojectBaselineNoBranch, unprojectDepthNoBranch,
     DepthShader::Flag::NoFarPlanePatching},
};

DepthUnprojectionTest::DepthUnprojectionTest() {
  addInstancedTests(
      {&DepthUnprojectionTest::testCpu, &DepthUnprojectionTest::testGpuDirect,
       &DepthUnprojectionTest::testGpuUnprojectExisting},
      Cr::Containers::arraySize(TestData));

  addInstancedBenchmarks({&DepthUnprojectionTest::benchmarkBaseline}, 10,
                         Cr::Containers::arraySize(UnprojectBenchmarkData));

  addInstancedBenchmarks({&DepthUnprojectionTest::benchmarkCpu}, 50,
                         Cr::Containers::arraySize(UnprojectBenchmarkData));

  addBenchmarks({&DepthUnprojectionTest::benchmarkGpuDirect}, 50,
                BenchmarkType::GpuTime);

  addInstancedBenchmarks(
      {&DepthUnprojectionTest::benchmarkGpuUnprojectExisting}, 50,
      Cr::Containers::arraySize(UnprojectBenchmarkData),
      BenchmarkType::GpuTime);
}

void DepthUnprojectionTest::testCpu() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Mn::Vector3 projected =
      data.projection.transformPoint({0.95f, -0.34f, -data.depth});
  CORRADE_COMPARE_WITH(
      -data.projection.inverted().transformPoint(projected).z(), data.depth,
      Cr::TestSuite::Compare::around(data.depth * 0.0006f));

  float depth[] = {Mn::Math::lerpInverted(-1.0f, 1.0f, projected.z())};
  unprojectDepth(calculateDepthUnprojection(data.projection),
                 Cr::Containers::stridedArrayView(depth));
  CORRADE_COMPARE_WITH(depth[0], data.expected,
                       Cr::TestSuite::Compare::around(data.depth * 0.0002f));
}

void DepthUnprojectionTest::testGpuDirect() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  /* This makes points that are too far set to 0, which is consistent with
     the CPU-side unprojection code */
  Mn::GL::Renderer::setClearColor(0x000000_rgbf);

  Mn::GL::Texture2D output;
  output.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::R32F, Mn::Vector2i{4});
  Mn::GL::Framebuffer framebuffer{{{}, Mn::Vector2i{4}}};
  framebuffer.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, output, 0)
      .clear(Mn::GL::FramebufferClear::Color)
      .bind();

  auto transformation =
      Mn::Matrix4::scaling({10000.0f, 10000.0f, 1.0f}) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(-data.depth));

  /* If we're testing for the far plane, don't draw anything, just use the
     clear color. */
  if (!(data.depth == 100.0f && data.expected == 0.0f)) {
    Mn::GL::Mesh mesh = Mn::MeshTools::compile(Mn::Primitives::planeSolid());
    DepthShader shader;
    shader.setTransformationMatrix(transformation)
        .setProjectionMatrix(data.projection)
        .draw(mesh);
  }

  MAGNUM_VERIFY_NO_GL_ERROR();

  Mn::Image2D image =
      framebuffer.read(framebuffer.viewport(), {Mn::PixelFormat::R32F});
  CORRADE_COMPARE_WITH(image.pixels<Mn::Float>()[2][2], data.expected,
                       Cr::TestSuite::Compare::around(data.depth * 0.0002f));
}

void DepthUnprojectionTest::testGpuUnprojectExisting() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Mn::Vector3 projected =
      data.projection.transformPoint({0.95f, -0.34f, -data.depth});
  Mn::GL::Renderer::setClearDepth(
      Mn::Math::lerpInverted(-1.0f, 1.0f, projected.z()));

  Mn::GL::Texture2D depth;
  depth.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, Mn::Vector2i{4});
  Mn::GL::Framebuffer framebuffer{{{}, Mn::Vector2i{4}}};
  framebuffer
      .attachTexture(Mn::GL::Framebuffer::BufferAttachment::Depth, depth, 0)
      .mapForDraw(Mn::GL::Framebuffer::DrawAttachment::None)
      .clear(Mn::GL::FramebufferClear::Depth)
      .bind();

  Mn::GL::Texture2D output;
  output.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::R32F, Mn::Vector2i{4});
  framebuffer.detach(Mn::GL::Framebuffer::BufferAttachment::Depth)
      .attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, output, 0)
      .mapForDraw(Mn::GL::Framebuffer::ColorAttachment{0})
      .clear(Mn::GL::FramebufferClear::Color);

  DepthShader shader{DepthShader::Flag::UnprojectExistingDepth};
  shader.setProjectionMatrix(data.projection)
      .bindDepthTexture(depth)
      .draw(Mn::GL::Mesh{}.setCount(3));

  MAGNUM_VERIFY_NO_GL_ERROR();

  Mn::Image2D image =
      framebuffer.read(framebuffer.viewport(), {Mn::PixelFormat::R32F});
  CORRADE_COMPARE_WITH(image.pixels<Mn::Float>()[2][2], data.expected,
                       Cr::TestSuite::Compare::around(data.depth * 0.0002f));
}

constexpr Mn::Vector2i BenchmarkSize{1536};

void DepthUnprojectionTest::benchmarkBaseline() {
  auto&& data = UnprojectBenchmarkData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  if (testCaseInstanceId() == 0 && testCaseRepeatId() == 0) {
#ifdef FMV_SUPPORTED
    Mn::Debug{} << "FMV target:" << targetName();
#else
    Mn::Debug{}
        << "FMV not supported by the compiler, relying on CXXFLAGS only.";
#endif
  }

  Mn::Matrix4 unprojection =
      Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.001f, 100.0f)
          .inverted();

  Cr::Containers::Array<float> depth{Cr::NoInit,
                                     std::size_t(BenchmarkSize.product())};
  for (std::size_t i = 0; i != depth.size(); ++i)
    depth[i] = float(2 * (i % 10000)) / float(10000) - 1.0f;

  CORRADE_BENCHMARK(1) {
    data.unprojectorFull(unprojection, stridedArrayView(depth));
  }

  CORRADE_COMPARE_AS(Mn::Math::max<float>(depth), 9.0f,
                     Cr::TestSuite::Compare::Greater);
}

void DepthUnprojectionTest::benchmarkCpu() {
  auto&& data = UnprojectBenchmarkData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Mn::Vector2 unprojection = calculateDepthUnprojection(
      Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.001f, 100.0f));

  Cr::Containers::Array<float> depth{Cr::NoInit,
                                     std::size_t(BenchmarkSize.product())};
  for (std::size_t i = 0; i != depth.size(); ++i)
    depth[i] = float(i % 10000) / float(10000);

  CORRADE_BENCHMARK(1) {
    data.unprojectorOptimized(unprojection, stridedArrayView(depth));
  }

  CORRADE_COMPARE_AS(Mn::Math::max<float>(depth), 9.0f,
                     Cr::TestSuite::Compare::Greater);
}

void DepthUnprojectionTest::benchmarkGpuDirect() {
  Mn::GL::Texture2D output{};
  output.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::R32F, BenchmarkSize);
  Mn::GL::Framebuffer framebuffer{{{}, BenchmarkSize}};
  framebuffer.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, output, 0)
      .clear(Mn::GL::FramebufferClear::Color)
      .bind();

  auto transformation = Mn::Matrix4::scaling({10000.0f, 10000.0f, 1.0f}) *
                        Mn::Matrix4::translation(Mn::Vector3::zAxis(-4.0f));

  Mn::GL::Mesh mesh = Mn::MeshTools::compile(Mn::Primitives::planeSolid());
  DepthShader shader;
  shader.setTransformationMatrix(transformation)
      .setProjectionMatrix(
          Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.001f, 100.0f));

  CORRADE_BENCHMARK(10) { shader.draw(mesh); }
}

void DepthUnprojectionTest::benchmarkGpuUnprojectExisting() {
  auto&& data = UnprojectBenchmarkData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Mn::Matrix4 projection =
      Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.001f, 100.0f);
  Mn::Vector3 projected = projection.transformPoint({0.95f, -0.34f, -4.0f});
  Mn::GL::Renderer::setClearDepth(
      Mn::Math::lerpInverted(-1.0f, 1.0f, projected.z()));

  Mn::GL::Texture2D depth;
  depth.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, BenchmarkSize);
  Mn::GL::Framebuffer framebuffer{{{}, BenchmarkSize}};
  framebuffer
      .attachTexture(Mn::GL::Framebuffer::BufferAttachment::Depth, depth, 0)
      .mapForDraw(Mn::GL::Framebuffer::DrawAttachment::None)
      .clear(Mn::GL::FramebufferClear::Depth)
      .bind();

  Mn::GL::Texture2D output;
  output.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::R32F, BenchmarkSize);
  framebuffer.detach(Mn::GL::Framebuffer::BufferAttachment::Depth)
      .attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, output, 0)
      .mapForDraw(Mn::GL::Framebuffer::ColorAttachment{0})
      .clear(Mn::GL::FramebufferClear::Color);

  DepthShader shader{DepthShader::Flag::UnprojectExistingDepth | data.flags};
  shader.setProjectionMatrix(projection).bindDepthTexture(depth);
  Mn::GL::Mesh mesh;
  mesh.setCount(3);

  CORRADE_BENCHMARK(10) { shader.draw(mesh); }
}

}  // namespace

CORRADE_TEST_MAIN(DepthUnprojectionTest)
