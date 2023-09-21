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

constexpr const char* baseTestFilename = "van-gogh-room";

const std::string SourceTestPlyDir =
    Cr::Utility::Path::join(SCENE_DATASETS, "habitat-test-scenes");

const std::string TestHBAOImageDir =
    Cr::Utility::Path::join(TEST_ASSETS, "hbao_tests");

constexpr Mn::Vector2i Size{320, 240};

constexpr const float aspectRatio = float(Size.x()) / Size.y();

struct TestDataType;
struct Projection;

struct GfxBatchHbaoTest : Mn::GL::OpenGLTester {
  explicit GfxBatchHbaoTest();
  void generateTestData();
  void testPerspective();

  void testPerspectiveFlipped();
  void testOrthographic();

  void testHBAOData(const TestDataType& data,
                    const std::string& filename,
                    const Projection& proj,
                    bool flipped);
};

struct Projection {
  Mn::Matrix4 projection;
  const std::string sourceColorFilename;
  const std::string sourceDepthFilename;
};

const Projection perspectiveData{
    Mn::Matrix4::perspectiveProjection(45.0_degf, aspectRatio, 0.1f, 100.0f),
    Cr::Utility::formatString("{}.color.png", baseTestFilename),
    Cr::Utility::formatString("{}.depth.exr", baseTestFilename)};

const Projection flippedPerspectiveData{
    Mn::Matrix4::perspectiveProjection(
        2.0f * Mn::Deg((Mn::Math::atan(Mn::Math::tan(Mn::Rad(45.0_degf) * 0.5) /
                                       aspectRatio))),
        1.0f / aspectRatio,
        0.1f,
        100.0f),
    Cr::Utility::formatString("{}.color.png", baseTestFilename),
    Cr::Utility::formatString("{}.depth.exr", baseTestFilename)};

const Projection orthographicData{
    Mn::Matrix4::orthographicProjection(
        Magnum::Vector2(4.0f * aspectRatio, 4.0f),
        0.1f,
        100.0f),
    Cr::Utility::formatString("{}.color-ortho.png", baseTestFilename),
    Cr::Utility::formatString("{}.depth-ortho.exr", baseTestFilename)};

const struct TestDataType {
  const char* name;
  const char* filename;
  bool classic;
  esp::gfx_batch::HbaoFlags flags;
  Mn::Float intensity, radius, blurSharpness;
  Mn::Float maxThreshold, meanThreshold;
} TestData[]{
    // Perspective projection
    {"classic, defaults", "hbao-classic", true,
     esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"classic, AO special blur", "hbao-classic-sblur", true,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 1.0f, 0.1f},
    {"classic, strong effect", "hbao-classic-strong", true,
     esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity() * 2.0f,
     esp::gfx_batch::HbaoConfiguration{}.radius() * 1.5f,
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness() * 5.0f, 0.0f, 0.0f},
    {"cache-aware, defaults", "hbao-cache", false,
     esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"cache-aware, AO special blur", "hbao-cache-sblur", false,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 1.0f, 0.1f},
    {"cache-aware, strong effect", "hbao-cache-strong", false,
     esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity() * 2.0f,
     esp::gfx_batch::HbaoConfiguration{}.radius() * 1.5f,
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness() * 5.0f, 0.0f, 0.0f},
    {"cache-aware, strong effect, AO special blur", "hbao-cache-strong-sblur",
     false, esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity() * 2.0f,
     esp::gfx_batch::HbaoConfiguration{}.radius() * 1.5f,
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness() * 5.0f, 1.0f, 0.1f},
    {"cache-aware, layered with image load store", "hbao-cache-layered", false,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::LayeredImageLoadStore,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"cache-aware, layered with image load store, AO special blur",
     "hbao-cache-layered-sblur", false,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         (esp::gfx_batch::HbaoFlag::LayeredImageLoadStore |
          esp::gfx_batch::HbaoFlag::UseAoSpecialBlur),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 1.0f, 0.1f},
    {"cache-aware, layered with geometry shader", "hbao-cache-geom", false,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::LayeredGeometryShader,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"cache-aware, layered with geometry shader, AO special blur",
     "hbao-cache-geom-sblur", false,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         (esp::gfx_batch::HbaoFlag::LayeredGeometryShader |
          esp::gfx_batch::HbaoFlag::UseAoSpecialBlur),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 1.0f, 0.1f},
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
      SourceTestPlyDir,
      Cr::Utility::formatString("{}.mesh.ply", baseTestFilename))));

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
      Cr::Utility::Path::join(TestHBAOImageDir,
                              data.projData.sourceColorFilename),
      (Mn::DebugTools::CompareImageToFile{}));
  CORRADE_COMPARE_WITH(
      framebuffer.read({{}, Size}, {Mn::PixelFormat::Depth32F}),
      Cr::Utility::Path::join(TestHBAOImageDir,
                              data.projData.sourceDepthFilename),
      (Mn::DebugTools::CompareImageToFile{}));
}

void GfxBatchHbaoTest::testHBAOData(const TestDataType& data,
                                    const std::string& filename,
                                    const Projection& projData,
                                    bool flipped) {
  if ((data.flags & esp::gfx_batch::HbaoFlag::LayeredImageLoadStore) &&
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
          TestHBAOImageDir, projData.sourceColorFilename))) {
    CORRADE_FAIL("Cannot load the color image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> color = importer->image2D(0);
  CORRADE_VERIFY(color);
  CORRADE_COMPARE(color->size(), Size);
  CORRADE_COMPARE(color->format(), Mn::PixelFormat::RGBA8Unorm);

  if (!importer->openFile(Cr::Utility::Path::join(
          TestHBAOImageDir, projData.sourceDepthFilename))) {
    CORRADE_FAIL("Cannot load the depth image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> depth = importer->image2D(0);
  CORRADE_VERIFY(depth);
  CORRADE_COMPARE(depth->size(), Size);
  CORRADE_COMPARE(depth->format(), Mn::PixelFormat::Depth32F);

  Mn::GL::Texture2D inputDepthTexture;
  Mn::GL::Texture2D outputColorTexture;
  Mn::Vector2i calcSize;
  if (flipped) {
    calcSize = Size.flipped();
    /* This rotates the depth image by 90° */
    Cr::Containers::Array<char> rotatedDepth{Cr::NoInit, depth->data().size()};
    Cr::Utility::copy(depth->pixels().flipped<1>().transposed<0, 1>(),
                      Cr::Containers::StridedArrayView3D<char>{
                          rotatedDepth,
                          {std::size_t(Size.x()), std::size_t(Size.y()),
                           depth->pixelSize()}});
    inputDepthTexture
        .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, calcSize)
        .setSubImage(0, {},
                     Mn::ImageView2D{depth->format(), calcSize, rotatedDepth});

    /* This rotates the color image by 90° */
    Cr::Containers::Array<char> rotatedColor{Cr::NoInit, color->data().size()};
    Cr::Utility::copy(color->pixels().flipped<1>().transposed<0, 1>(),
                      Cr::Containers::StridedArrayView3D<char>{
                          rotatedColor,
                          {std::size_t(Size.x()), std::size_t(Size.y()),
                           color->pixelSize()}});

    outputColorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, calcSize)
        .setSubImage(0, {},
                     Mn::ImageView2D{color->format(), calcSize, rotatedColor});

  } else {
    calcSize = Size;
    inputDepthTexture
        .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, calcSize)
        .setSubImage(0, {}, *depth);

    outputColorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, calcSize)
        .setSubImage(0, {}, *color);
  }
  Mn::GL::Framebuffer output{{{}, calcSize}};
  output.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                       outputColorTexture, 0);

  /* No clear, that would kill the base image */

  MAGNUM_VERIFY_NO_GL_ERROR();

  esp::gfx_batch::Hbao hbao{esp::gfx_batch::HbaoConfiguration{}
                                .setSize(calcSize)
                                .setFlags(data.flags)
                                .setIntensity(data.intensity)
                                .setRadius(data.radius)
                                .setBlurSharpness(data.blurSharpness)};
  MAGNUM_VERIFY_NO_GL_ERROR();
  // test projection drawing for both classic and cache-aware algorithms

  hbao.drawEffect(projData.projection, !data.classic, inputDepthTexture,
                  output);

  MAGNUM_VERIFY_NO_GL_ERROR();

  if (flipped) {
    // This is expected to fail - the differeces from rotating and unrotating
    // are substantially greater than our current thresholds.
    CORRADE_EXPECT_FAIL(
        "The difference between these images is beyond our "
        "current thresholds, to be investigated later.");
    CORRADE_COMPARE_WITH(
        (output.read({{}, calcSize}, {Mn::PixelFormat::RGBA8Unorm})
             .pixels<Mn::Color4ub>()
             .transposed<1, 0>()
             .flipped<1>()),
        Cr::Utility::Path::join(TestHBAOImageDir, filename),
        (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                            data.meanThreshold}));
  } else {
    CORRADE_COMPARE_WITH(
        output.read({{}, calcSize}, {Mn::PixelFormat::RGBA8Unorm}),
        Cr::Utility::Path::join(TestHBAOImageDir, filename),
        (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                            data.meanThreshold}));
  }
}  // GfxBatchHbaoTest::testHBAOData

void GfxBatchHbaoTest::testPerspective() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(
      Cr::Utility::formatString("{}, perspective", data.name));
  testHBAOData(
      data,
      Cr::Utility::formatString("{}.{}.png", baseTestFilename, data.filename),
      perspectiveData, false);

}  // GfxBatchHbaoTest::testPerspective()

void GfxBatchHbaoTest::testPerspectiveFlipped() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(
      Cr::Utility::formatString("{}, perspective, flipped", data.name));
  testHBAOData(
      data,
      Cr::Utility::formatString("{}.{}.png", baseTestFilename, data.filename),
      flippedPerspectiveData, true);

}  // GfxBatchHbaoTest::testPerspective()

void GfxBatchHbaoTest::testOrthographic() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(
      Cr::Utility::formatString("{}, orthographic", data.name));
  testHBAOData(data,
               Cr::Utility::formatString("{}.{}-ortho.png", baseTestFilename,
                                         data.filename),
               orthographicData, false);

}  // GfxBatchHbaoTest::testOrthographic()

}  // namespace

CORRADE_TEST_MAIN(GfxBatchHbaoTest)
