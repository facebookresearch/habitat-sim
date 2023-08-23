#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/GL/OpenGLTester.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Compile.h>
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

struct GfxBatchHbaoTest: Mn::GL::OpenGLTester {
  explicit GfxBatchHbaoTest();

  void generateTestData();
  void test();
};

const struct {
  const char* name;
  const char* filename;
  bool classic;
  esp::gfx_batch::HbaoFlags flags;
  Mn::Float intensity, radius, blurSharpness;
  Mn::Float maxThreshold, meanThreshold;
  bool expectFail;
} TestData[]{
  {"classic, defaults", "van-gogh-room.hbao-classic.png",
    true,
    esp::gfx_batch::HbaoConfiguration{}.flags(),
    esp::gfx_batch::HbaoConfiguration{}.intensity(),
    esp::gfx_batch::HbaoConfiguration{}.radius(),
    esp::gfx_batch::HbaoConfiguration{}.blurSharpness(),
    0.0f, 0.0f, false},
  {"classic, AO special blur", "van-gogh-room.hbao-classic.png",
    true,
    esp::gfx_batch::HbaoConfiguration{}.flags()|esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
    esp::gfx_batch::HbaoConfiguration{}.intensity(),
    esp::gfx_batch::HbaoConfiguration{}.radius(),
    esp::gfx_batch::HbaoConfiguration{}.blurSharpness(),
    2.0f, 2.0f, true},
  {"classic, strong effect", "van-gogh-room.hbao-classic-strong.png",
    true, esp::gfx_batch::HbaoConfiguration{}.flags(),
    2.0f,
    1.5f,
    10.0f,
    0.0f, 0.0f, false},

  {"cache-aware, defaults", "van-gogh-room.hbao.png",
    false, esp::gfx_batch::HbaoConfiguration{}.flags(),
    esp::gfx_batch::HbaoConfiguration{}.intensity(),
    esp::gfx_batch::HbaoConfiguration{}.radius(),
    esp::gfx_batch::HbaoConfiguration{}.blurSharpness(),
    0.0f, 0.0f, false},
  {"cache-aware, AO special blur", "van-gogh-room.hbao.png",
    false,
    esp::gfx_batch::HbaoConfiguration{}.flags()|esp::gfx_batch::HbaoFlag::UseAoSpecialBlur,
    esp::gfx_batch::HbaoConfiguration{}.intensity(),
    esp::gfx_batch::HbaoConfiguration{}.radius(),
    esp::gfx_batch::HbaoConfiguration{}.blurSharpness(),
    2.0f, 2.0f, true},
  {"cache-aware, layered with image load store", "van-gogh-room.hbao.png",
    false,
    esp::gfx_batch::HbaoConfiguration{}.flags()|esp::gfx_batch::HbaoFlag::LayeredImageLoadStore,
    esp::gfx_batch::HbaoConfiguration{}.intensity(),
    esp::gfx_batch::HbaoConfiguration{}.radius(),
    esp::gfx_batch::HbaoConfiguration{}.blurSharpness(),
    4.5f, 0.11f, false},
  {"cache-aware, layered with geometry shader passthrough", "van-gogh-room.hbao.png",
    false,
    esp::gfx_batch::HbaoConfiguration{}.flags()|esp::gfx_batch::HbaoFlag::LayeredGeometryShaderPassthrough,
    esp::gfx_batch::HbaoConfiguration{}.intensity(),
    esp::gfx_batch::HbaoConfiguration{}.radius(),
    esp::gfx_batch::HbaoConfiguration{}.blurSharpness(),
    0.0f, 0.0f, false},
  {"cache-aware, strong effect", "van-gogh-room.hbao-strong.png",
    false, esp::gfx_batch::HbaoConfiguration{}.flags(),
    2.0f,
    1.5f,
    10.0f,
    0.0f, 0.0f, false},
};

GfxBatchHbaoTest::GfxBatchHbaoTest() {
  addTests({&GfxBatchHbaoTest::generateTestData});

  addInstancedTests({&GfxBatchHbaoTest::test},
    Cr::Containers::arraySize(TestData));
}

constexpr Mn::Vector2i Size{256, 256};
const Mn::Matrix4 Projection = Mn::Matrix4::perspectiveProjection(45.0_degf, 1.0f, 0.1f, 100.0f);

void GfxBatchHbaoTest::generateTestData() {
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> importerManager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = importerManager.loadAndInstantiate("AnySceneImporter");
  if(!importer)
    CORRADE_SKIP("Cannot load the importer");

  /* magnum-sceneconverter van-gogh-room.glb --concatenate-meshes van-gogh-room.mesh.ply */
  if(!importer->openFile(Cr::Utility::Path::join(SCENE_DATASETS, "habitat-test-scenes/van-gogh-room.mesh.ply")))
    CORRADE_SKIP("Cannot load the file");

  Cr::Containers::Optional<Mn::Trade::MeshData> meshData = importer->mesh(0);
  CORRADE_VERIFY(meshData);
  Mn::GL::Mesh mesh = Mn::MeshTools::compile(*meshData);

  Mn::GL::Texture2D colorTexture, depthTexture;
  colorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, Size);
  depthTexture.setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, Size);
  Mn::GL::Framebuffer framebuffer{{{}, Size}};
  framebuffer
    .attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, colorTexture, 0)
    .attachTexture(Mn::GL::Framebuffer::BufferAttachment::Depth, depthTexture, 0)
    .clear(Mn::GL::FramebufferClear::Depth|Mn::GL::FramebufferClear::Color)
    .bind();

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

  Mn::Shaders::PhongGL shader{Mn::Shaders::PhongGL::Configuration{}
    .setLightCount(3)};
  shader
    .setProjectionMatrix(Projection)
    .setTransformationMatrix(
      Mn::Matrix4::translation({0.0f, -1.0f, -7.5f})*
      Mn::Matrix4::rotationX(-80.0_degf)*
      Mn::Matrix4::rotationZ(-90.0_degf)
    )
    .setLightPositions({
      {10.0f, 10.0f, 10.0f, 0.0f},
      {-5.0f, -5.0f, 10.0f, 0.0f},
      {0.0f, 5.0f, -10.0f, 0.0f},
    })
    .setLightColors({
      0xffffff_rgbf,
      0xffdddd_rgbf,
      0xddddff_rgbf
    })
    .draw(mesh);

  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      framebuffer.read({{}, Size}, {Mn::PixelFormat::RGBA8Unorm}),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "van-gogh-room.color.png"),
      (Mn::DebugTools::CompareImageToFile{}));
  CORRADE_COMPARE_WITH(
      framebuffer.read({{}, Size}, {Mn::PixelFormat::Depth32F}),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "van-gogh-room.depth.exr"),
      (Mn::DebugTools::CompareImageToFile{}));
}

void GfxBatchHbaoTest::test() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // TODO expose this in Magnum::GL::Extensions, this is shitty
  if((data.flags & esp::gfx_batch::HbaoFlag::LayeredGeometryShaderPassthrough) && !(Mn::GL::Context::current().detectedDriver() & Mn::GL::Context::DetectedDriver::NVidia))
    CORRADE_SKIP("GL_NV_geometry_shader_passthrough not supported");

  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> importerManager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer = importerManager.loadAndInstantiate("AnyImageImporter");
  if(!importer)
    CORRADE_SKIP("Cannot load the importer");

  if(!importer->openFile(Cr::Utility::Path::join(TEST_ASSETS,
                              "van-gogh-room.color.png")))
    CORRADE_FAIL("Cannot load the color image");
  Cr::Containers::Optional<Mn::Trade::ImageData2D> color = importer->image2D(0);
  CORRADE_VERIFY(color);
  CORRADE_COMPARE(color->size(), Size);
  CORRADE_COMPARE(color->format(), Mn::PixelFormat::RGBA8Unorm);

  if(!importer->openFile(Cr::Utility::Path::join(TEST_ASSETS,
                              "van-gogh-room.depth.exr")))
    CORRADE_FAIL("Cannot load the depth image");
  Cr::Containers::Optional<Mn::Trade::ImageData2D> depth = importer->image2D(0);
  CORRADE_VERIFY(depth);
  CORRADE_COMPARE(depth->size(), Size);
  CORRADE_COMPARE(depth->format(), Mn::PixelFormat::Depth32F);

  Mn::GL::Texture2D inputDepthTexture;
  inputDepthTexture
    .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, Size)
    .setSubImage(0, {}, *depth);

  Mn::GL::Texture2D outputColorTexture;
  outputColorTexture
    .setStorage(1, Mn::GL::TextureFormat::RGBA8, Size)
    .setSubImage(0, {}, *color);
  Mn::GL::Framebuffer output{{{}, Size}};
  output.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, outputColorTexture, 0);
  /* No clear, that would kill the base image */

  MAGNUM_VERIFY_NO_GL_ERROR();

  esp::gfx_batch::Hbao hbao{esp::gfx_batch::HbaoConfiguration{}
    .setSize(Size)
    .setFlags(data.flags)
    .setIntensity(data.intensity)
    .setRadius(data.radius)
    .setBlurSharpness(data.blurSharpness)};
  MAGNUM_VERIFY_NO_GL_ERROR();

  if(data.classic)
    hbao.drawClassicPerspective(Projection, inputDepthTexture, output);
  else
    hbao.drawCacheAwarePerspective(Projection, inputDepthTexture, output);
  MAGNUM_VERIFY_NO_GL_ERROR();
  {
    CORRADE_EXPECT_FAIL_IF(data.expectFail, "This doesn't work as expected.");
    CORRADE_COMPARE_WITH(
        output.read({{}, Size}, {Mn::PixelFormat::RGBA8Unorm}),
        Cr::Utility::Path::join(TEST_ASSETS,
                                data.filename),
        (Mn::DebugTools::CompareImageToFile{data.maxThreshold, data.meanThreshold}));
  }
}

}

CORRADE_TEST_MAIN(GfxBatchHbaoTest)
