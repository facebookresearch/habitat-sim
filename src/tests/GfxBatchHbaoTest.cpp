#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
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

struct GfxBatchHbaoTest : Mn::GL::OpenGLTester {
  explicit GfxBatchHbaoTest();
  void generateTestData();
  void test();

  Mn::Matrix4 projMatrices[2] = {
      Mn::Matrix4::perspectiveProjection(45.0_degf, 1.0f, 0.1f, 100.0f),
      Mn::Matrix4::orthographicProjection(Magnum::Vector2(4.0f, 4.0f),
                                          0.1f,
                                          100.0f)};
  std::string srcRGBFileNames[2] = {"van-gogh-room.color.png",
                                    "van-gogh-room.color-ortho.png"};

  std::string srcDepthFileNames[2] = {"van-gogh-room.depth.exr",
                                      "van-gogh-room.depth-ortho.exr"};
};

const struct {
  const char* name;
  const char* filename;
  bool classic;
  int projIDX;
  esp::gfx_batch::HbaoFlags flags;
  Mn::Float intensity, radius, blurSharpness;
  Mn::Float maxThreshold, meanThreshold;
} TestData[]{
    // Perspective projection
    {"classic, perspective, defaults", "van-gogh-room.hbao-classic.png", true,
     0, esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"classic, perspective, AO special blur",
     "van-gogh-room.hbao-classic-sblur.png", true, 0,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 2.0f, 2.0f},
    {"classic, perspective, strong effect",
     "van-gogh-room.hbao-classic-strong.png", true, 0,
     esp::gfx_batch::HbaoConfiguration{}.flags(), 2.0f, 1.5f, 10.0f, 0.0f,
     0.0f},

    {"cache-aware, perspective, defaults", "van-gogh-room.hbao-cache.png",
     false, 0, esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"cache-aware, perspective, AO special blur",
     "van-gogh-room.hbao-cache-sblur.png", false, 0,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 2.0f, 2.0f},
    {"cache-aware, perspective, strong effect",
     "van-gogh-room.hbao-cache-strong.png", false, 0,
     esp::gfx_batch::HbaoConfiguration{}.flags(), 2.0f, 1.5f, 10.0f, 0.0f,
     0.0f},
    {"cache-aware, perspective, layered with image load store",
     "van-gogh-room.hbao-cache-layered.png", false, 0,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::LayeredImageLoadStore,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 4.5f, 0.11f},
    {"cache-aware, perspective, layered with geometry shader",
     "van-gogh-room.hbao-cache-geom.png", false, 0,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::LayeredGeometryShader,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 4.5f, 0.11f},

    // Orthographic projection
    {"classic, orthographic, defaults", "van-gogh-room.hbao-classic-ortho.png",
     true, 1, esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"classic, orthographic, AO special blur",
     "van-gogh-room.hbao-classic-sblur-ortho.png", true, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 2.0f, 2.0f},
    {"classic, orthographic, strong effect",
     "van-gogh-room.hbao-classic-strong-ortho.png", true, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags(), 2.0f, 1.5f, 10.0f, 0.0f,
     0.0f},

    {"cache-aware, orthographic, defaults",
     "van-gogh-room.hbao-cache-ortho.png", false, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags(),
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 0.0f, 0.0f},
    {"cache-aware, orthographic, AO special blur",
     "van-gogh-room.hbao-cache-sblur-ortho.png", false, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 2.0f, 2.0f},
    {"cache-aware, orthographic, strong effect",
     "van-gogh-room.hbao-cache-strong-ortho.png", false, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags(), 2.0f, 1.5f, 10.0f, 0.0f,
     0.0f},
    {"cache-aware, orthographic, layered with image load store",
     "van-gogh-room.hbao-cache-layered-ortho.png", false, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::LayeredImageLoadStore,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 4.5f, 0.11f},
    {"cache-aware, orthographic, layered with geometry shader",
     "van-gogh-room.hbao-cache-geom-ortho.png", false, 1,
     esp::gfx_batch::HbaoConfiguration{}.flags() |
         esp::gfx_batch::HbaoFlag::LayeredGeometryShader,
     esp::gfx_batch::HbaoConfiguration{}.intensity(),
     esp::gfx_batch::HbaoConfiguration{}.radius(),
     esp::gfx_batch::HbaoConfiguration{}.blurSharpness(), 4.5f, 0.11f},
};

const struct {
  const char* name;
  // Adding this in case we wish to do more tests on generation that do not map
  // directly to testCaseInstanceId 0 == perspective or testCaseInstanceId 1 ==
  // orthographic
  int projIDX;
} GenerateTestDataData[]{
    {"perspective", 0},
    {"orthographic", 1},
};

GfxBatchHbaoTest::GfxBatchHbaoTest() {
  addInstancedTests({&GfxBatchHbaoTest::generateTestData},
                    Cr::Containers::arraySize(GenerateTestDataData));

  addInstancedTests({&GfxBatchHbaoTest::test},
                    Cr::Containers::arraySize(TestData));
}

constexpr Mn::Vector2i Size{256, 256};
void GfxBatchHbaoTest::generateTestData() {
  auto&& data = GenerateTestDataData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> importerManager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      importerManager.loadAndInstantiate("AnySceneImporter");
  CORRADE_VERIFY(importer);

  /* magnum-sceneconverter van-gogh-room.glb --concatenate-meshes
   * van-gogh-room.mesh.ply */
  CORRADE_VERIFY(importer->openFile(Cr::Utility::Path::join(
      {TEST_ASSETS, "hbao_tests", "van-gogh-room.mesh.ply"})));

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
  shader.setProjectionMatrix(projMatrices[data.projIDX])
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
      Cr::Utility::Path::join(
          {TEST_ASSETS, "hbao_tests", srcRGBFileNames[data.projIDX]}),
      (Mn::DebugTools::CompareImageToFile{}));
  CORRADE_COMPARE_WITH(
      framebuffer.read({{}, Size}, {Mn::PixelFormat::Depth32F}),
      Cr::Utility::Path::join(
          {TEST_ASSETS, "hbao_tests", srcDepthFileNames[data.projIDX]}),
      (Mn::DebugTools::CompareImageToFile{}));
}

void GfxBatchHbaoTest::test() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

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
          {TEST_ASSETS, "hbao_tests", srcRGBFileNames[data.projIDX]}))) {
    CORRADE_FAIL("Cannot load the color image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> color = importer->image2D(0);
  CORRADE_VERIFY(color);
  CORRADE_COMPARE(color->size(), Size);
  CORRADE_COMPARE(color->format(), Mn::PixelFormat::RGBA8Unorm);

  if (!importer->openFile(Cr::Utility::Path::join(
          {TEST_ASSETS, "hbao_tests", srcDepthFileNames[data.projIDX]}))) {
    CORRADE_FAIL("Cannot load the depth image");
  }
  Cr::Containers::Optional<Mn::Trade::ImageData2D> depth = importer->image2D(0);
  CORRADE_VERIFY(depth);
  CORRADE_COMPARE(depth->size(), Size);
  CORRADE_COMPARE(depth->format(), Mn::PixelFormat::Depth32F);

  Mn::GL::Texture2D inputDepthTexture;
  inputDepthTexture
      .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, Size)
      .setSubImage(0, {}, *depth);

  Mn::GL::Texture2D outputColorTexture;
  outputColorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, Size)
      .setSubImage(0, {}, *color);
  Mn::GL::Framebuffer output{{{}, Size}};
  output.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                       outputColorTexture, 0);
  /* No clear, that would kill the base image */

  MAGNUM_VERIFY_NO_GL_ERROR();

  esp::gfx_batch::Hbao hbao{esp::gfx_batch::HbaoConfiguration{}
                                .setSize(Size)
                                .setFlags(data.flags)
                                .setIntensity(data.intensity)
                                .setRadius(data.radius)
                                .setBlurSharpness(data.blurSharpness)};
  MAGNUM_VERIFY_NO_GL_ERROR();
  // test projection drawing for both classic and cache-aware algorithms

  hbao.drawEffect(projMatrices[data.projIDX], !data.classic, inputDepthTexture,
                  output);
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      output.read({{}, Size}, {Mn::PixelFormat::RGBA8Unorm}),
      Cr::Utility::Path::join({TEST_ASSETS, "hbao_tests", data.filename}),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));
}

}  // namespace

CORRADE_TEST_MAIN(GfxBatchHbaoTest)
