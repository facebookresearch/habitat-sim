// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/PluginManager/PluginMetadata.h>
#include <Corrade/TestSuite/Compare/File.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/GL/OpenGLTester.h> /* just for MAGNUM_VERIFY_NO_GL_ERROR() */
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Concatenate.h>
#include <Magnum/MeshTools/GenerateIndices.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Trade/AbstractImageConverter.h>
#include <Magnum/Trade/AbstractSceneConverter.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>

#include "esp/gfx_batch/RendererStandalone.h"

#ifdef ESP_BUILD_WITH_CUDA
#include <cuda_gl_interop.h>
#endif

#include "configure.h"

namespace {

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Mn::Math::Literals;

struct GfxBatchRendererTest : Cr::TestSuite::Tester {
  explicit GfxBatchRendererTest();

  void generateTestData();

  void defaults();

  void singleMesh();
  void meshHierarchy();
  void multipleMeshes();

  void multipleScenes();
  void clearScene();

  void cudaInterop();
};

// clang-format off
const struct {
  const char* name;
  esp::gfx_batch::RendererFlags flags;
  Mn::Float textureMultiplier;
  const char* filename;
} MeshHierarchyData[]{
  {"", {}, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"no textures",
    esp::gfx_batch::RendererFlag::NoTextures, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"}
};
// clang-format on

GfxBatchRendererTest::GfxBatchRendererTest() {
  // clang-format off
  addTests({&GfxBatchRendererTest::generateTestData,

            &GfxBatchRendererTest::defaults,

            &GfxBatchRendererTest::singleMesh});

  addInstancedTests({&GfxBatchRendererTest::meshHierarchy},
                    Cr::Containers::arraySize(MeshHierarchyData));

  addTests({&GfxBatchRendererTest::multipleMeshes,

            &GfxBatchRendererTest::multipleScenes,
            &GfxBatchRendererTest::clearScene,

            &GfxBatchRendererTest::cudaInterop});
  // clang-format on
}

#ifdef HAS_MAGNUM_GLTFSCENECONVERTER
/* These don't really need to have the same IDs as used by other code, only
   the name matters */
constexpr auto SceneFieldMeshViewIndexOffset = Mn::Trade::sceneFieldCustom(56);
constexpr auto SceneFieldMeshViewIndexCount = Mn::Trade::sceneFieldCustom(774);
constexpr auto SceneFieldMeshViewMaterial = Mn::Trade::sceneFieldCustom(23);
#endif

void GfxBatchRendererTest::generateTestData() {
#ifndef HAS_MAGNUM_GLTFSCENECONVERTER
  CORRADE_SKIP("GltfSceneConverter plugin not found");
#else
  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter>
      imageConverterManager;
  if (imageConverterManager.loadState("KtxImageConverter") ==
      Cr::PluginManager::LoadState::NotFound)
    CORRADE_SKIP("KtxImageConverter plugin not found");

  Cr::PluginManager::Manager<Mn::Trade::AbstractSceneConverter>
      converterManager;
  converterManager.registerExternalManager(imageConverterManager);
  Cr::Containers::Pointer<Mn::Trade::AbstractSceneConverter> converter =
      converterManager.loadAndInstantiate("GltfSceneConverter");
  if (!converter)
    CORRADE_SKIP("GltfSceneConverter plugin not found");

  converter->configuration().setValue("imageConverter", "KtxImageConverter");
  converter->configuration().setValue("imageExtension", "ktx2");
  /* To prevent the file from being opened by unsuspecting libraries */
  converter->configuration().addValue("extensionUsed", "MAGNUMX_mesh_views");
  converter->configuration().addValue("extensionRequired",
                                      "MAGNUMX_mesh_views");

  const Cr::Containers::String filename =
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR, "batch.gltf");

  /* Begin file conversion */
  converter->beginFile(filename);
  converter->setSceneFieldName(SceneFieldMeshViewIndexOffset,
                               "meshViewIndexOffset");
  converter->setSceneFieldName(SceneFieldMeshViewIndexCount,
                               "meshViewIndexCount");
  converter->setSceneFieldName(SceneFieldMeshViewMaterial, "meshViewMaterial");

  /* (Flat) plane, circle and triangle mesh. All made indexed and joined
     together. Important: offsets are in bytes. */
  Mn::Trade::MeshData plane =
      Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
          Mn::Primitives::PlaneFlag::TextureCoordinates));
  Mn::UnsignedInt planeIndexOffset = 0;

  Mn::Trade::MeshData circle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          32, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt circleIndexOffset = planeIndexOffset + 4 * plane.indexCount();

  Mn::Trade::MeshData triangle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          3, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt triangleIndexOffset =
      circleIndexOffset + 4 * circle.indexCount();

  Mn::Trade::MeshData mesh =
      Mn::MeshTools::concatenate({plane, circle, triangle});
  for (Mn::Vector2& i : mesh.mutableAttribute<Mn::Vector2>(
           Mn::Trade::MeshAttribute::TextureCoordinates)) {
    // TODO remmove this once GltfSceneConverter does that itself
    i.y() = 1.0f - i.y();
  }
  CORRADE_VERIFY(converter->add(mesh));

  /* Two-layer 4x4 texture. First layer is a grey/red checkerboard, second
     layer is a cyan, magenta, yellow and black-ish square. Having each channel
     non-zero to make it possible to distinguish "rendering broken" from
     "texture coordinates broken". */
  // clang-format off
  Mn::Color3ub image[4*4*2] {
    0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb,
    0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb,
    0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb,
    0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb,

    0x33cccc_rgb, 0x33cccc_rgb, 0xcc33cc_rgb, 0xcc33cc_rgb,
    0x33cccc_rgb, 0x33cccc_rgb, 0xcc33cc_rgb, 0xcc33cc_rgb,
    0xcccc33_rgb, 0xcccc33_rgb, 0x333333_rgb, 0x333333_rgb,
    0xcccc33_rgb, 0xcccc33_rgb, 0x333333_rgb, 0x333333_rgb
  };
  // clang-format on
  CORRADE_VERIFY(converter->add(
      Mn::ImageView3D{Mn::PixelFormat::RGB8Unorm, {4, 4, 2}, image}));

  /* A texture referencing the only image. Nearest neighbor filtering to have
     less noise in the output images. */
  CORRADE_VERIFY(converter->add(Mn::Trade::TextureData{
      Mn::Trade::TextureType::Texture2DArray, Mn::SamplerFilter::Nearest,
      Mn::SamplerFilter::Nearest, Mn::SamplerMipmap::Nearest,
      Mn::SamplerWrapping::Repeat, 0}));

  /* A (default, white) material spanning the whole first texture layer */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}), 0);

  /* A cyan / magenta / yellow material spanning the bottom left / top left /
     bottom right quadrant of second texture layer. I.e., nothing should be
     using the black-ish portion of the texture. When combined with the
     texture color, the output should have the corresponding red / green / blue
     channels zeroed out. */
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.5f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}), 3);
  /* Scene with
      - a square using the checkerboard material
      - a circle using the cyan material
      - a triangle using the magenta material
      - a subtree of four child meshes translated on X and Y, each being a
        square using one of the materials
    The top-level object is named and mesh assignments are always to its
    children, as that's currently what the renderer assumes. Similarly, the
    renderer only gets immediate children, not nested children.
  */
  struct Scene {
    struct Parent {
      Mn::UnsignedInt object;
      Mn::Int parent;
    } parents[11];
    struct Mesh {
      Mn::UnsignedInt object;
      Mn::UnsignedInt mesh;
      Mn::UnsignedInt meshViewIndexOffset;
      Mn::UnsignedInt meshViewIndexCount;
      Mn::Int meshViewMaterial;
    } meshes[7];
    struct Transformation {
      Mn::UnsignedInt object;
      Mn::Matrix4 trasformation;
    } transformations[4];
  } scene[]{{
    {{0, -1}, {1, 0}, /* square and its child mesh */
     {2, -1}, {3, 2}, /* circle and its child mesh */
     {4, -1}, {5, 4}, /* triangle and its child mesh */
     {6, -1}, {7, 6}, {8, 6}, /* four squares */
              {9, 6}, {10, 6}},
    {{ 1, 0, planeIndexOffset, plane.indexCount(), 0},
     { 3, 0, circleIndexOffset, circle.indexCount(), 1},
     { 5, 0, triangleIndexOffset, triangle.indexCount(), 2},
     { 7, 0, planeIndexOffset, plane.indexCount(), 0},
     { 8, 0, planeIndexOffset, plane.indexCount(), 1},
     { 9, 0, planeIndexOffset, plane.indexCount(), 2},
     {10, 0, planeIndexOffset, plane.indexCount(), 3}},
    {{ 7, Mn::Matrix4::translation({-0.5f, -0.5f, 0.0f})*
          Mn::Matrix4::scaling(Mn::Vector3{0.4f})},
     { 8, Mn::Matrix4::translation({+0.5f, -0.5f, 0.0f})*
          Mn::Matrix4::scaling(Mn::Vector3{0.4f})},
     { 9, Mn::Matrix4::translation({-0.5f, +0.5f, 0.0f})*
          Mn::Matrix4::scaling(Mn::Vector3{0.4f})},
     {10, Mn::Matrix4::translation({+0.5f, +0.5f, 0.0f})*
          Mn::Matrix4::scaling(Mn::Vector3{0.4f})}}
  }};
  converter->setObjectName(0, "square");
  converter->setObjectName(2, "circle");
  converter->setObjectName(4, "triangle");
  converter->setObjectName(6, "four squares");
  CORRADE_VERIFY(converter->add(Mn::Trade::SceneData{Mn::Trade::SceneMappingType::UnsignedInt, 11, {}, scene, {
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Parent,
      Cr::Containers::stridedArrayView(scene->parents).slice(&Scene::Parent::object),
      Cr::Containers::stridedArrayView(scene->parents).slice(&Scene::Parent::parent)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Mesh,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::mesh)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Transformation,
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::object),
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::trasformation)},
    /* Extras currently have to be last to avoid other fields being mixed with
       them */ // TODO fix properly!!!
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexOffset,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexOffset)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexCount,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexCount)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewMaterial,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewMaterial)},
  }}));
  // clang-format on

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
     test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
     executable). */
  CORRADE_COMPARE_AS(filename,
                     Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"),
                     Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR, "batch.bin"),
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.bin"),
      Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR, "batch.0.ktx2"),
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.0.ktx2"),
      Cr::TestSuite::Compare::File);
#endif
}

void GfxBatchRendererTest::defaults() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({48, 32}, {2, 3}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          /* No QuietLog, to verify at least once that the log *is* printed */
  };
  // clang-format on

  CORRADE_COMPARE(renderer.tileSize(), (Mn::Vector2i{48, 32}));
  CORRADE_COMPARE(renderer.tileCount(), (Mn::Vector2i{2, 3}));
  CORRADE_COMPARE(renderer.sceneCount(), 6);

  for (std::size_t i = 0; i != renderer.sceneCount(); ++i) {
    CORRADE_ITERATION(i);

    /* Implicit camera transform, no object transforms in any scene */
    CORRADE_COMPARE(renderer.camera(i), Mn::Matrix4{});
    CORRADE_COMPARE(renderer.transformations(i).size(), 0);
  }

  /* Add a file, because that's currently required */
  // TODO make it non-required (instantiate some empty shader if nothing)
  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));

  /* Nothing should be drawn, just the clear color */
  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  Mn::Image2D depth = renderer.depthImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Verify the color actually matches expectation so we don't need to manually
     color pick the image every time it changes */
  CORRADE_COMPARE_AS(
      color,
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestDefaults.png"),
      Mn::DebugTools::CompareImageToFile);
  CORRADE_COMPARE(color.size(), Mn::Vector2i{96});
  CORRADE_COMPARE(color.format(), Mn::PixelFormat::RGBA8Unorm);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[48][48], 0x1f1f1f_rgb);

  /* Verify the depth readout works as well. Also should be a single value. */
  CORRADE_COMPARE(depth.size(), Mn::Vector2i{96});
  CORRADE_COMPARE(depth.format(), Mn::PixelFormat::Depth32F);
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[48][48], 1.0f);
}

void GfxBatchRendererTest::singleMesh() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));

  /* Undo the aspect ratio, move camera back */
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "square"), 0);
  /* It adds one transformation for the top-level object and then one nested
     for the mesh, corresponding to the layout inside the glTF file */
  CORRADE_COMPARE(renderer.transformations(0).size(), 2);
  renderer.transformations(0)[0] = Mn::Matrix4::scaling(Mn::Vector3{0.8f});

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Check that texture coordinates, image data or whatever else didn't get
     flipped -- there should be a red pixel on the bottom left and grey pixel
     on the top left. */
  CORRADE_COMPARE_AS(
      renderer.colorImage(),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestSingleMesh.png"),
      Mn::DebugTools::CompareImageToFile);
  CORRADE_COMPARE(color.size(), (Mn::Vector2i{128, 96}));
  CORRADE_COMPARE(color.format(), Mn::PixelFormat::RGBA8Unorm);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[75][35], 0xcccccc_rgb);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[20][38], 0x990000_rgb);
}

void GfxBatchRendererTest::meshHierarchy() {
  auto&& data = MeshHierarchyData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1})
          .setFlags(data.flags),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));

  /* Undo the aspect ratio, move camera back */
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "four squares"), 0);
  /* It adds one transformation for the top-level object and then four nested
     for each mesh, corresponding to the layout inside the glTF file */
  CORRADE_COMPARE(renderer.transformations(0).size(), 5);
  renderer.transformations(0)[0] = Mn::Matrix4::scaling(Mn::Vector3{0.8f});

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* The square at bottom left should be the usual checkerboard, square at
     bottom right should be cyan, top left magenta and top right yellow */
  CORRADE_COMPARE_AS(
      renderer.colorImage(),
      Cr::Utility::Path::join({TEST_ASSETS, "screenshots", data.filename}),
      Mn::DebugTools::CompareImageToFile);
  CORRADE_COMPARE(color.size(), (Mn::Vector2i{128, 96}));
  CORRADE_COMPARE(color.format(), Mn::PixelFormat::RGBA8Unorm);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[29][87],
                  0x00ffff_rgb * data.textureMultiplier);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[68][50],
                  0xff00ff_rgb * data.textureMultiplier);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[67][78],
                  0xffff00_rgb * data.textureMultiplier);
}

void GfxBatchRendererTest::multipleMeshes() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "square"), 0);
  renderer.transformations(0)[0] =
      Mn::Matrix4::translation({0.0f, 0.5f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.4f});

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "circle"), 2);
  renderer.transformations(0)[2] =
      Mn::Matrix4::translation({-0.5f, -0.5f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.4f});

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "triangle"), 4);
  renderer.transformations(0)[4] =
      Mn::Matrix4::translation({0.5f, -0.5f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.4f});

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Square should be the usual checkerboard, circle should be cyan, triangle
     magenta */
  CORRADE_COMPARE_AS(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS, "screenshots/GfxBatchRendererTestMultipleMeshes.png"),
      Mn::DebugTools::CompareImageToFile);
  CORRADE_COMPARE(color.size(), (Mn::Vector2i{128, 96}));
  CORRADE_COMPARE(color.format(), Mn::PixelFormat::RGBA8Unorm);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[18][44], 0x00cccc_rgb);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[24][88], 0xcc00cc_rgb);
}

void GfxBatchRendererTest::multipleScenes() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({64, 48}, {2, 2}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{1.0f, 4.0f / 3.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  /* Scene 0 has one multi-mesh, scene 1 has two single-meshes, scene 2 is
     unused and scene 3 has a single triangle */
  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "four squares"), 0);
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "circle"), 0);
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "square"), 2);
  CORRADE_COMPARE(renderer.addMeshHierarchy(3, "triangle"), 0);

  /* Each camera is shifted differently on Y, each added mesh is shifted
     differently on X to test the right transformation is used each time */
  renderer.camera(0) = Mn::Matrix4::translation({0.0f, 0.0f, 1.0f}).inverted();
  renderer.transformations(0)[0] = Mn::Matrix4::translation({0.0f, 0.0f, 0.0f});

  renderer.camera(1) = Mn::Matrix4::translation({0.0f, 0.5f, 1.0f}).inverted();
  renderer.transformations(1)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});
  renderer.transformations(1)[2] =
      Mn::Matrix4::translation({-0.5f, 1.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  renderer.camera(3) = Mn::Matrix4::translation({0.0f, -0.5f, 1.0f}).inverted();
  renderer.transformations(3)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Just a visual test, the texture transformation and material aspects were
     tested well enough above */
  CORRADE_COMPARE_AS(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS, "screenshots/GfxBatchRendererTestMultipleScenes.png"),
      Mn::DebugTools::CompareImageToFile);
}

void GfxBatchRendererTest::clearScene() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({64, 48}, {2, 2}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{1.0f, 4.0f / 3.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  /* Like in multipleScenes(), except in different order, there's more stuff
     added to scene 1 and it isn't transformed in any way */
  CORRADE_COMPARE(renderer.addMeshHierarchy(3, "triangle"), 0);
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "square"), 0);
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "circle"), 2);
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "triangle"), 4);
  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "four squares"), 0);

  renderer.camera(0) = Mn::Matrix4::translation({0.0f, 0.0f, 1.0f}).inverted();
  renderer.transformations(0)[0] = Mn::Matrix4::translation({0.0f, 0.0f, 0.0f});

  renderer.camera(1) = Mn::Matrix4::translation({0.0f, 0.5f, 1.0f}).inverted();

  renderer.camera(3) = Mn::Matrix4::translation({0.0f, -0.5f, 1.0f}).inverted();
  renderer.transformations(3)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  /* Clearing scene 1 should have no effect on others */
  renderer.clear(1);
  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_AS(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS,
          "screenshots/GfxBatchRendererTestMultipleScenesClearScene1.png"),
      Mn::DebugTools::CompareImageToFile);

  /* Add things to scene 1 again, transform them */
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "circle"), 0);
  CORRADE_COMPARE(renderer.addMeshHierarchy(1, "square"), 2);
  renderer.transformations(1)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});
  renderer.transformations(1)[2] =
      Mn::Matrix4::translation({-0.5f, 1.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  /* Now it should match the output in multipleScenes() */
  renderer.draw();
  CORRADE_COMPARE_AS(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS, "screenshots/GfxBatchRendererTestMultipleScenes.png"),
      Mn::DebugTools::CompareImageToFile);
}

void GfxBatchRendererTest::cudaInterop() {
#ifndef ESP_BUILD_WITH_CUDA
  CORRADE_SKIP("ESP_BUILD_WITH_CUDA is not enabled");
#else
  {
    /* Initialize the count to 0, so it isn't left in a random state if the
       command fails, leading to a false impression that CUDA works, and a
       crash later */
    int count = 0;
    cudaGetDeviceCount(&count);
    if (!count)
      CORRADE_SKIP("No CUDA devices found");
  }

  /* Implicitly use device 0 */
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          // TODO this fails if using a GLX application and the GL device
          //  doesn't match the CUDA device, what to do?
          .setCudaDevice(0)
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  /* Mostly the same as singleMesh() */
  renderer.addFile(Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"));
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();
  renderer.addMeshHierarchy(0, "square");
  renderer.transformations(0)[0] = Mn::Matrix4::scaling(Mn::Vector3{0.8f});
  renderer.draw();

  /* Get CUDA image pointers */
  const void* cudaColorBuffer = renderer.colorCudaBufferDevicePointer();
  const void* cudaDepthBuffer = renderer.depthCudaBufferDevicePointer();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Copy them from the GPU */
  Mn::Image2D cudaColorImage{
      renderer.colorFramebufferFormat(),
      {128, 96},
      Cr::Containers::Array<char>{Cr::NoInit, 128 * 96 * 4}};
  Mn::Image2D cudaDepthImage{
      renderer.depthFramebufferFormat(),
      {128, 96},
      Cr::Containers::Array<char>{Cr::NoInit, 128 * 96 * 4}};
  cudaMemcpy(cudaColorImage.data(), cudaColorBuffer,
             cudaColorImage.data().size(), cudaMemcpyDeviceToHost);
  cudaMemcpy(cudaDepthImage.data(), cudaDepthBuffer,
             cudaDepthImage.data().size(), cudaMemcpyDeviceToHost);

  /* Should be the same as what singleMesh() rendered; depth should have *some*
     data also */
  CORRADE_COMPARE_AS(
      cudaColorImage,
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestSingleMesh.png"),
      Mn::DebugTools::CompareImageToFile);
  CORRADE_COMPARE(cudaDepthImage.pixels<Mn::Float>()[0][0], 1.0f);
  CORRADE_COMPARE(cudaDepthImage.pixels<Mn::Float>()[95][127], 1.0f);
  CORRADE_COMPARE(cudaDepthImage.pixels<Mn::Float>()[64][48], 0.0909091f);
#endif
}

}  // namespace

CORRADE_TEST_MAIN(GfxBatchRendererTest)
