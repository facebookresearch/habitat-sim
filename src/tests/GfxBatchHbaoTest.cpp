// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/OpenGLTester.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Angle.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>

#include "esp/gfx_batch/Hbao.h"

#include "configure.h"

namespace {

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Mn::Math::Literals;

// Change this if we want to test on a different core mesh. Requires
// regenerating ground truth images.
constexpr const char* baseTestFilename = "van-gogh-room";

// Location of test scene mesh from habitat_test_secenes repository.
const Cr::Containers::String testPlySourceDir =
    Cr::Utility::Path::join(SCENE_DATASETS, "habitat-test-scenes");

// Validation images location (held in repository
// TODO : consider moving these images to habitat_test_secenes repository.
const Cr::Containers::String testHBAOImageDir =
    Cr::Utility::Path::join(TEST_ASSETS, "hbao_tests");

// Size of images to test for validation
constexpr Mn::Vector2i Size{320, 240};

// Aspect ratio of given size
const float AspectRatio = Mn::Vector2{Size}.aspectRatio();

struct Projection;
struct TestDataType;

struct GfxBatchHbaoTest : Mn::GL::OpenGLTester {
  explicit GfxBatchHbaoTest();
  void generateTestData();

  /// @brief Validation Tests ///

  /**
   * @brief Test standard orientation HBAO effect
   * @param data The pertinent test data for the test
   * @param filename The name of the validation file to test against
   * @param proj The projection matrix this test consumes
   */
  void testHBAOData(const TestDataType& data,
                    Cr::Containers::StringView filename,
                    const Projection& proj);

  /**
   * @brief Test HBAO effect on 90deg-rotated source data, to look for
   * orientation biases in the algorithm.
   * @param data The pertinent test data for the test
   * @param filename The name of the validation file to test against
   * @param proj The projection matrix this test consumes
   */
  void testFlippedHBAOData(const TestDataType& data,
                           Cr::Containers::StringView filename,
                           const Projection& proj);

  /**
   * @brief Test standard orientation perspective projection HBAO results
   */
  void testPerspective();

  /**
   * @brief Test 90deg-rotation perspective projection HBAO results
   */
  void testPerspectiveFlipped();

  /**
   * @brief Test standard orientation orthographic projection HBAO results
   */
  void testOrthographic();

  /// @brief Benchmarks ///
  /**
   * @brief Benchmark synthesizing HBAO effect
   * @param data The pertinent test data for the test
   * @param filename The name of the validation file to test against
   * @param proj The projection matrix this test consumes
   */
  void benchmarkHBAOData(const TestDataType& data, Mn::Matrix4 projMatrix);
  /**
   * @brief Benchmark the synthesis of the HBAO effect in a
   * perspective-projection environment.
   */
  void benchmarkPerspective();

  /**
   * @brief Benchmark the synthesis of the HBAO effect in a
   * orthographic-projection environment.
   */
  void benchmarkOrthographic();
};

/**
 * @brief Type defining a camera projection matrix and the source color and
 * depth images for that particular projection.
 */
struct Projection {
  Mn::Matrix4 projection;
  const Cr::Containers::String sourceColorFilename;
  const Cr::Containers::String sourceDepthFilename;
};

const Projection perspectiveData{
    Mn::Matrix4::perspectiveProjection(45.0_degf, AspectRatio, 0.1f, 100.0f),
    Cr::Utility::format("{}.color.png", baseTestFilename),
    Cr::Utility::format("{}.depth.exr", baseTestFilename)};

const Projection flippedPerspectiveData{
    Mn::Matrix4::perspectiveProjection(
        2.0f * Mn::Deg((Mn::Math::atan(Mn::Math::tan(Mn::Rad(45.0_degf) * 0.5) /
                                       AspectRatio))),
        1.0f / AspectRatio,
        0.1f,
        100.0f),
    Cr::Utility::format("{}.color.png", baseTestFilename),
    Cr::Utility::format("{}.depth.exr", baseTestFilename)};

const Projection orthographicData{
    Mn::Matrix4::orthographicProjection(
        Magnum::Vector2(4.0f * AspectRatio, 4.0f),
        0.1f,
        100.0f),
    Cr::Utility::format("{}.color-ortho.png", baseTestFilename),
    Cr::Utility::format("{}.depth-ortho.exr", baseTestFilename)};

/**
 * @brief Type defining test data used by various tests for validity and
 * benchmarking
 */
const struct TestDataType {
  const char* name;
  const char* filename;
  esp::gfx_batch::HbaoType algType;
  esp::gfx_batch::HbaoConfiguration config;

  Mn::Float maxThreshold = 0.0f;
  Mn::Float meanThreshold = 0.0f;
} TestData[]{
    // Perspective projection
    {"classic, defaults", "hbao-classic", esp::gfx_batch::HbaoType::Classic,
     esp::gfx_batch::HbaoConfiguration{}},
    {"classic, AO special blur", "hbao-classic-sblur",
     esp::gfx_batch::HbaoType::Classic,
     esp::gfx_batch::HbaoConfiguration{}.setUseSpecialBlur(true), 1.0f, 0.1f},
    {"classic, strong effect", "hbao-classic-strong",
     esp::gfx_batch::HbaoType::Classic,
     esp::gfx_batch::HbaoConfiguration{}
         .scaleIntensity(2.0f)
         .scaleRadius(1.5f)
         .scaleBlurSharpness(5.0f)},
    {"cache-aware, defaults", "hbao-cache",
     esp::gfx_batch::HbaoType::CacheAware, esp::gfx_batch::HbaoConfiguration{}},
    {"cache-aware, AO special blur", "hbao-cache-sblur",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setUseSpecialBlur(true), 1.0f, 0.1f},
    {"cache-aware, strong effect", "hbao-cache-strong",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}
         .scaleIntensity(2.0f)
         .scaleRadius(1.5f)
         .scaleBlurSharpness(5.0f)},
    {"cache-aware, strong effect, AO special blur", "hbao-cache-strong-sblur",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}
         .setUseSpecialBlur(true)
         .scaleIntensity(2.0f)
         .scaleRadius(1.5f)
         .scaleBlurSharpness(5.0f),
     1.0f, 0.1f},
    {"cache-aware, layered with image load store", "hbao-cache-layered",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setUseLayeredImageLoadStore(true)},
    {"cache-aware, layered with image load store, AO special blur",
     "hbao-cache-layered-sblur", esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}
         .setUseLayeredImageLoadStore(true)
         .setUseSpecialBlur(true),
     1.0f, 0.1f},
    {"cache-aware, layered with geometry shader", "hbao-cache-geom",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setUseLayeredGeometryShader(true)},
    {"cache-aware, layered with geometry shader, AO special blur",
     "hbao-cache-geom-sblur", esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}
         .setUseLayeredGeometryShader(true)
         .setUseSpecialBlur(true),
     1.0f, 0.1f},
};

TestDataType BenchData[]{
    {"classic, defaults", "hbao-classic", esp::gfx_batch::HbaoType::Classic,
     esp::gfx_batch::HbaoConfiguration{}},
    {"classic, no blur", "hbao-classic", esp::gfx_batch::HbaoType::Classic,
     esp::gfx_batch::HbaoConfiguration{}.setNoBlur(true)},
    {"cache-aware, defaults", "hbao-cache",
     esp::gfx_batch::HbaoType::CacheAware, esp::gfx_batch::HbaoConfiguration{}},
    {"cache-aware, no blur", "hbao-cache", esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setNoBlur(true)},
    {"cache-aware, AO special blur", "hbao-cache-sblur",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setUseSpecialBlur(true), 1.0f, 0.1f},
    {"cache-aware, layered with image load store", "hbao-cache-layered",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setUseLayeredImageLoadStore(true)},
    {"cache-aware, layered with image load store, AO special blur",
     "hbao-cache-layered-sblur", esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}
         .setUseLayeredImageLoadStore(true)
         .setUseSpecialBlur(true),
     1.0f, 0.1f},
    {"cache-aware, layered with geometry shader", "hbao-cache-geom",
     esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}.setUseLayeredGeometryShader(true)},
    {"cache-aware, layered with geometry shader, AO special blur",
     "hbao-cache-geom-sblur", esp::gfx_batch::HbaoType::CacheAware,
     esp::gfx_batch::HbaoConfiguration{}
         .setUseLayeredGeometryShader(true)
         .setUseSpecialBlur(true),
     1.0f, 0.1f},
};

const struct {
  const char* name;
  // Adding this in case we wish to do more tests on generation that do not map
  // directly to testCaseInstanceId 0 == perspective or testCaseInstanceId 1 ==
  // orthographic

  const Projection& projData;
} GenerateTestDataData[]{
    {"perspective", perspectiveData},
    {"orthographic", orthographicData},
};

GfxBatchHbaoTest::GfxBatchHbaoTest() {
  addInstancedTests({&GfxBatchHbaoTest::generateTestData},
                    Cr::Containers::arraySize(GenerateTestDataData));

  addInstancedTests({&GfxBatchHbaoTest::testPerspective,
                     &GfxBatchHbaoTest::testPerspectiveFlipped,
                     &GfxBatchHbaoTest::testOrthographic},
                    Cr::Containers::arraySize(TestData));

  addInstancedBenchmarks({&GfxBatchHbaoTest::benchmarkPerspective,
                          &GfxBatchHbaoTest::benchmarkOrthographic},
                         5, Cr::Containers::arraySize(BenchData),
                         BenchmarkType::GpuTime);
}

void GfxBatchHbaoTest::generateTestData() {
  auto&& data = GenerateTestDataData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> importerManager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      importerManager.loadAndInstantiate("AnySceneImporter");
  CORRADE_VERIFY(importer);

  /* magnum-sceneconverter <baseTestFilename>.glb --concatenate-meshes
   * <baseTestFilename>.mesh.ply */
  CORRADE_VERIFY(importer->openFile(Cr::Utility::Path::join(
      testPlySourceDir, Cr::Utility::format("{}.mesh.ply", baseTestFilename))));

  Cr::Containers::Optional<Mn::Trade::MeshData> meshData = importer->mesh(0);
  CORRADE_VERIFY(meshData);
  Mn::GL::Mesh mesh = Mn::MeshTools::compile(*meshData);

  Mn::GL::Texture2D colorTexture, depthTexture;
  colorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, Size);
  depthTexture.setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, Size);

  Mn::GL::Framebuffer framebuffer{{{}, Size}};
  framebuffer
      .attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, colorTexture, 0)
      .attachTexture(Mn::GL::Framebuffer::BufferAttachment::Depth, depthTexture,
                     0)
      .clear(Mn::GL::FramebufferClear::Depth | Mn::GL::FramebufferClear::Color)
      .bind();

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

  Mn::Shaders::PhongGL shader{
      Mn::Shaders::PhongGL::Configuration{}.setLightCount(3)};
  shader.setProjectionMatrix(data.projData.projection)
      .setTransformationMatrix(Mn::Matrix4::translation({0.0f, -1.0f, -7.5f}) *
                               Mn::Matrix4::rotationX(-80.0_degf) *
                               Mn::Matrix4::rotationZ(-90.0_degf))
      .setLightPositions({
          {10.0f, 10.0f, 10.0f, 0.0f},
          {-5.0f, -5.0f, 10.0f, 0.0f},
          {0.0f, 5.0f, -10.0f, 0.0f},
      })
      .setLightColors({0xffffff_rgbf, 0xffdddd_rgbf, 0xddddff_rgbf})
      .draw(mesh);

  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      framebuffer.read({{}, Size}, {Mn::PixelFormat::RGBA8Unorm}),
      Cr::Utility::Path::join(testHBAOImageDir,
                              data.projData.sourceColorFilename),
      (Mn::DebugTools::CompareImageToFile{}));
  CORRADE_COMPARE_WITH(
      framebuffer.read({{}, Size}, {Mn::PixelFormat::Depth32F}),
      Cr::Utility::Path::join(testHBAOImageDir,
                              data.projData.sourceDepthFilename),
      (Mn::DebugTools::CompareImageToFile{}));
}  // GfxBatchHbaoTest::generateTestData()

void GfxBatchHbaoTest::testHBAOData(const TestDataType& data,
                                    Cr::Containers::StringView filename,
                                    const Projection& projData) {
  if ((data.config.flags() & esp::gfx_batch::HbaoFlag::LayeredImageLoadStore) &&
      !(
#ifdef MAGNUM_TARGET_GLES
          Mn::GL::Context::current().isVersionSupported(
              Mn::GL::Version::GLES310)
#else
          Mn::GL::Context::current()
              .isExtensionSupported<
                  Mn::GL::Extensions::ARB::shader_image_load_store>()
#endif
              ))
    CORRADE_SKIP("Image load/store not supported");

  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> importerManager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      importerManager.loadAndInstantiate("AnyImageImporter");
  CORRADE_VERIFY(importer);

  if (!importer->openFile(Cr::Utility::Path::join(
          testHBAOImageDir, projData.sourceColorFilename))) {
    CORRADE_FAIL("Cannot load the color image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> color = importer->image2D(0);
  CORRADE_VERIFY(color);
  CORRADE_COMPARE(color->size(), Size);
  CORRADE_COMPARE(color->format(), Mn::PixelFormat::RGBA8Unorm);

  if (!importer->openFile(Cr::Utility::Path::join(
          testHBAOImageDir, projData.sourceDepthFilename))) {
    CORRADE_FAIL("Cannot load the depth image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> depth = importer->image2D(0);
  CORRADE_VERIFY(depth);
  CORRADE_COMPARE(depth->size(), Size);
  CORRADE_COMPARE(depth->format(), Mn::PixelFormat::Depth32F);

  Mn::GL::Texture2D inputDepthTexture;
  Mn::GL::Texture2D outputColorTexture;
  inputDepthTexture
      .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, Size)
      .setSubImage(0, {}, *depth);

  outputColorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, Size)
      .setSubImage(0, {}, *color);

  Mn::GL::Framebuffer output{{{}, Size}};
  output.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                       outputColorTexture, 0);

  /* No clear, that would kill the base image */

  MAGNUM_VERIFY_NO_GL_ERROR();

  esp::gfx_batch::Hbao hbao{
      esp::gfx_batch::HbaoConfiguration{data.config}.setSize(Size)};
  MAGNUM_VERIFY_NO_GL_ERROR();
  // test projection drawing for both classic and cache-aware algorithms

  hbao.drawEffect(projData.projection, data.algType, inputDepthTexture, output);

  MAGNUM_VERIFY_NO_GL_ERROR();

  CORRADE_COMPARE_WITH(output.read({{}, Size}, {Mn::PixelFormat::RGBA8Unorm}),
                       Cr::Utility::Path::join(testHBAOImageDir, filename),
                       (Mn::DebugTools::CompareImageToFile{
                           data.maxThreshold, data.meanThreshold}));

}  // GfxBatchHbaoTest::testHBAOData

void GfxBatchHbaoTest::testFlippedHBAOData(const TestDataType& data,
                                           Cr::Containers::StringView filename,
                                           const Projection& projData) {
  if ((data.config.flags() & esp::gfx_batch::HbaoFlag::LayeredImageLoadStore) &&
      !(
#ifdef MAGNUM_TARGET_GLES
          Mn::GL::Context::current().isVersionSupported(
              Mn::GL::Version::GLES310)
#else
          Mn::GL::Context::current()
              .isExtensionSupported<
                  Mn::GL::Extensions::ARB::shader_image_load_store>()
#endif
              ))
    CORRADE_SKIP("Image load/store not supported");

  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> importerManager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      importerManager.loadAndInstantiate("AnyImageImporter");
  CORRADE_VERIFY(importer);

  if (!importer->openFile(Cr::Utility::Path::join(
          testHBAOImageDir, projData.sourceColorFilename))) {
    CORRADE_FAIL("Cannot load the color image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> color = importer->image2D(0);
  CORRADE_VERIFY(color);
  CORRADE_COMPARE(color->size(), Size);
  CORRADE_COMPARE(color->format(), Mn::PixelFormat::RGBA8Unorm);

  if (!importer->openFile(Cr::Utility::Path::join(
          testHBAOImageDir, projData.sourceDepthFilename))) {
    CORRADE_FAIL("Cannot load the depth image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> depth = importer->image2D(0);
  CORRADE_VERIFY(depth);
  CORRADE_COMPARE(depth->size(), Size);
  CORRADE_COMPARE(depth->format(), Mn::PixelFormat::Depth32F);

  Mn::GL::Texture2D inputDepthTexture;
  Mn::Vector2i calcSize = Size.flipped();
  /* This rotates the depth image by 90° */
  Cr::Containers::Array<char> rotatedDepth{Cr::NoInit, depth->data().size()};
  Cr::Utility::copy(
      depth->pixels().flipped<1>().transposed<0, 1>(),
      Cr::Containers::StridedArrayView3D<char>{
          rotatedDepth,
          {std::size_t(Size.x()), std::size_t(Size.y()), depth->pixelSize()}});
  inputDepthTexture
      .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, calcSize)
      .setSubImage(0, {},
                   Mn::ImageView2D{depth->format(), calcSize, rotatedDepth});

  /* This rotates the color image by 90° */
  Cr::Containers::Array<char> rotatedColor{Cr::NoInit, color->data().size()};
  Cr::Utility::copy(
      color->pixels().flipped<1>().transposed<0, 1>(),
      Cr::Containers::StridedArrayView3D<char>{
          rotatedColor,
          {std::size_t(Size.x()), std::size_t(Size.y()), color->pixelSize()}});

  Mn::GL::Texture2D outputColorTexture;

  outputColorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, calcSize)
      .setSubImage(0, {},
                   Mn::ImageView2D{color->format(), calcSize, rotatedColor});

  Mn::GL::Framebuffer output{{{}, calcSize}};
  output.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                       outputColorTexture, 0);

  /* No clear, that would kill the base image */

  MAGNUM_VERIFY_NO_GL_ERROR();

  esp::gfx_batch::Hbao hbao{
      esp::gfx_batch::HbaoConfiguration{data.config}.setSize(calcSize)};

  MAGNUM_VERIFY_NO_GL_ERROR();
  // test projection drawing for both classic and cache-aware algorithms

  hbao.drawEffect(projData.projection, data.algType, inputDepthTexture, output);

  MAGNUM_VERIFY_NO_GL_ERROR();

  // This is expected to fail - the differeces from rotating and unrotating
  // are substantially greater than our current thresholds.
  CORRADE_EXPECT_FAIL(
      "The difference between these images is measurable and they should be "
      "identical, indicating a potential directional bias in the algorithm. "
      "This is to be investigated later.\n");
  CORRADE_COMPARE_WITH(
      (output.read({{}, calcSize}, {Mn::PixelFormat::RGBA8Unorm})
           .pixels<Mn::Color4ub>()
           .transposed<1, 0>()
           .flipped<1>()),
      Cr::Utility::Path::join(testHBAOImageDir, filename),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));

}  // GfxBatchHbaoTest::testFlippedHBAOData

void GfxBatchHbaoTest::testPerspective() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(Cr::Utility::format("{}, perspective", data.name));
  testHBAOData(
      data, Cr::Utility::format("{}.{}.png", baseTestFilename, data.filename),
      perspectiveData);

}  // GfxBatchHbaoTest::testPerspective()

void GfxBatchHbaoTest::testOrthographic() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(Cr::Utility::format("{}, orthographic", data.name));
  testHBAOData(
      data,
      Cr::Utility::format("{}.{}-ortho.png", baseTestFilename, data.filename),
      orthographicData);

}  // GfxBatchHbaoTest::testOrthographic()

void GfxBatchHbaoTest::testPerspectiveFlipped() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(
      Cr::Utility::format("{}, perspective, flipped", data.name));
  testFlippedHBAOData(
      data, Cr::Utility::format("{}.{}.png", baseTestFilename, data.filename),
      flippedPerspectiveData);

}  // GfxBatchHbaoTest::testPerspective()

void GfxBatchHbaoTest::benchmarkHBAOData(const TestDataType& data,
                                         Mn::Matrix4 projMatrix) {
  if ((data.config.flags() & esp::gfx_batch::HbaoFlag::LayeredImageLoadStore) &&
      !(
#ifdef MAGNUM_TARGET_GLES
          Mn::GL::Context::current().isVersionSupported(
              Mn::GL::Version::GLES310)
#else
          Mn::GL::Context::current()
              .isExtensionSupported<
                  Mn::GL::Extensions::ARB::shader_image_load_store>()
#endif
              ))
    CORRADE_SKIP("Image load/store not supported");

  // Scale image size to use for benchmark uniformly so projection matrix aspect
  // ration not affected.
  Mn::Vector2i BenchImageSize = Size * 4;
  const size_t arraySize = BenchImageSize.product() * 4;

  // For benchmarks source color and depth are just empty white images

  Cr::Containers::Array<char> depthData{Cr::DirectInit, arraySize, '\xff'};
  Mn::ImageView2D depth{Mn::PixelFormat::Depth32F, BenchImageSize, depthData};

  Mn::GL::Texture2D inputDepthTexture;
  inputDepthTexture
      .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, depth.size())
      .setSubImage(0, {}, depth);

  Cr::Containers::Array<char> colorData{Cr::DirectInit, arraySize, '\xff'};
  Mn::ImageView2D color{Mn::PixelFormat::RGBA8Unorm, BenchImageSize,
                        std::move(colorData)};
  Mn::GL::Texture2D outputColorTexture;
  outputColorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, color.size())
      .setSubImage(0, {}, color);

  Cr::Containers::Array<char> resColorData{Cr::DirectInit, arraySize, '\xff'};
  Mn::ImageView2D resultImage{Mn::PixelFormat::RGBA8Unorm, BenchImageSize,
                              std::move(resColorData)};

  Mn::GL::Framebuffer output{{{}, BenchImageSize}};
  output.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                       outputColorTexture, 0);

  /* No clear, that would kill the base image */

  MAGNUM_VERIFY_NO_GL_ERROR();

  esp::gfx_batch::Hbao hbao{
      esp::gfx_batch::HbaoConfiguration{data.config}.setSize(BenchImageSize)};
  MAGNUM_VERIFY_NO_GL_ERROR();
  // Call once to compile the shaders
  hbao.drawEffect(projMatrix, data.algType, inputDepthTexture, output);

  // benchmark projection drawing for both classic and cache-aware algorithms
  CORRADE_BENCHMARK(16) {
    hbao.drawEffect(projMatrix, data.algType, inputDepthTexture, output);
  }

  MAGNUM_VERIFY_NO_GL_ERROR();

  CORRADE_COMPARE_WITH(
      output.read({{}, BenchImageSize}, {Mn::PixelFormat::RGBA8Unorm}),
      resultImage,
      (Mn::DebugTools::CompareImage{data.maxThreshold, data.meanThreshold}));

}  // GfxBatchHbaoTest::testHBAOData

void GfxBatchHbaoTest::benchmarkPerspective() {
  auto&& data = BenchData[testCaseInstanceId()];
  setTestCaseDescription(Cr::Utility::format("{}, perspective.", data.name));

  // For benchmarks source color and depth are just empty white images
  benchmarkHBAOData(data, perspectiveData.projection);

}  // GfxBatchHbaoTest::benchmarkPerspective()

void GfxBatchHbaoTest::benchmarkOrthographic() {
  auto&& data = BenchData[testCaseInstanceId()];
  setTestCaseDescription(Cr::Utility::format("{}, orthographic.", data.name));

  // For benchmarks source color and depth are just empty white images

  benchmarkHBAOData(data, orthographicData.projection);
}  // GfxBatchHbaoTest::benchmarkPerspective()

}  // namespace

CORRADE_TEST_MAIN(GfxBatchHbaoTest)
