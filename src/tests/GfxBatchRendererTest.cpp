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
  void generateTestDataMultipleMeshes();
  void generateTestDataMultipleTextures();
  void generateTestDataSquareCircleTriangle();
  void generateTestDataFourSquares();

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
  Cr::Containers::Array<const char*> gltfFilenames;
  Mn::UnsignedInt singleSceneBatchCount;
  Mn::UnsignedInt multipleScenesBatchCount[4];
} FileData[]{
  {"", Cr::Containers::array({"batch.gltf"}),
    1,
    {1, 1, 0, 1}
  },
  {"multiple meshes", Cr::Containers::array({"batch-multiple-meshes.gltf"}),
    /* Each has a separate mesh */
    3,
    {4, 2, 0, 1}},
  {"multiple textures", Cr::Containers::array({"batch-multiple-textures.gltf"}),
    /* Each has a separate texture */
    3,
    {4, 2, 0, 1}},
  {"multiple files", Cr::Containers::array({"batch-square-circle-triangle.gltf", "batch-four-squares.gltf"}),
    /* Square, circle and triangle are a single mesh but the hierarchical four
       squares are a separate file */
    2,
    {4, 2, 0, 1}},
};

const struct {
  const char* name;
  Cr::Containers::Array<const char*> gltfFilenames;
  esp::gfx_batch::RendererFlags flags;
  Mn::UnsignedInt batchCount;
  Mn::Float textureMultiplier;
  const char* filename;
} MeshHierarchyData[]{
  {"",
    Cr::Containers::array({"batch.gltf"}),
    {}, 1, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple meshes",
    Cr::Containers::array({"batch-multiple-meshes.gltf"}),
    {}, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple textures",
    Cr::Containers::array({"batch-multiple-textures.gltf"}),
    {}, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple files",
    Cr::Containers::array({"batch-square-circle-triangle.gltf", "batch-four-squares.gltf"}),
    {}, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"no textures",
    Cr::Containers::array({"batch.gltf"}),
    esp::gfx_batch::RendererFlag::NoTextures, 1, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"},
  {"multiple meshes, no textures",
    Cr::Containers::array({"batch-multiple-meshes.gltf"}),
    esp::gfx_batch::RendererFlag::NoTextures, 4, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"}
};
// clang-format on

GfxBatchRendererTest::GfxBatchRendererTest() {
  // clang-format off
  addTests({&GfxBatchRendererTest::generateTestData,
            &GfxBatchRendererTest::generateTestDataMultipleMeshes,
            &GfxBatchRendererTest::generateTestDataMultipleTextures,
            &GfxBatchRendererTest::generateTestDataSquareCircleTriangle});

  addTests({&GfxBatchRendererTest::generateTestDataFourSquares});

  addInstancedTests({&GfxBatchRendererTest::defaults,
                     &GfxBatchRendererTest::singleMesh},
      Cr::Containers::arraySize(FileData));

  addInstancedTests({&GfxBatchRendererTest::meshHierarchy},
      Cr::Containers::arraySize(MeshHierarchyData));

  addInstancedTests({&GfxBatchRendererTest::multipleMeshes,
                     &GfxBatchRendererTest::multipleScenes,
                     &GfxBatchRendererTest::clearScene},
      Cr::Containers::arraySize(FileData));

  addTests({&GfxBatchRendererTest::cudaInterop});
  // clang-format on
}

/* These don't really need to have the same IDs as used by other code, only
   the name matters */
constexpr auto SceneFieldMeshViewIndexOffset = Mn::Trade::sceneFieldCustom(56);
constexpr auto SceneFieldMeshViewIndexCount = Mn::Trade::sceneFieldCustom(774);
constexpr auto SceneFieldMeshViewMaterial = Mn::Trade::sceneFieldCustom(23);

void GfxBatchRendererTest::generateTestData() {
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

  converter->configuration().setValue("experimentalKhrTextureKtx", true);
  converter->configuration().setValue("imageConverter", "KtxImageConverter");
  /* Bundle images in the bin file to reduce the amount of test files */
  converter->configuration().setValue("bundleImages", true);
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

  /* (Flat) square, circle and triangle mesh. All made indexed and joined
     together. Important: offsets are in bytes. */
  Mn::Trade::MeshData square =
      Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
          Mn::Primitives::PlaneFlag::TextureCoordinates));
  Mn::UnsignedInt squareIndexOffset = 0;

  Mn::Trade::MeshData circle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          32, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt circleIndexOffset =
      squareIndexOffset + 4 * square.indexCount();

  Mn::Trade::MeshData triangle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          3, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt triangleIndexOffset =
      circleIndexOffset + 4 * circle.indexCount();

  CORRADE_VERIFY(
      converter->add(Mn::MeshTools::concatenate({square, circle, triangle})));

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
  CORRADE_VERIFY(converter->add(Mn::ImageView3D{
      Mn::PixelFormat::RGB8Unorm, {4, 4, 2}, image, Mn::ImageFlag3D::Array}));

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
  }}, "checkerboard"), 0);
  // clang-format on

  /* A cyan / magenta / yellow material spanning the bottom left / top left /
     bottom right quadrant of second texture layer. I.e., nothing should be
     using the black-ish portion of the texture. When combined with the
     texture color, the output should have the corresponding red / green / blue
     channels zeroed out. */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.5f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "yellow"), 3);
  // clang-format on

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
  // clang-format off
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
    {{ 1, 0, squareIndexOffset, square.indexCount(), 0},
     { 3, 0, circleIndexOffset, circle.indexCount(), 1},
     { 5, 0, triangleIndexOffset, triangle.indexCount(), 2},
     { 7, 0, squareIndexOffset, square.indexCount(), 0},
     { 8, 0, squareIndexOffset, square.indexCount(), 1},
     { 9, 0, squareIndexOffset, square.indexCount(), 2},
     {10, 0, squareIndexOffset, square.indexCount(), 3}},
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
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexOffset,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexOffset)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexCount,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexCount)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewMaterial,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewMaterial)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Transformation,
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::object),
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::trasformation)},
  }}));
  // clang-format on

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
     test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
     executable). Using a *.gltf to make it easier to see what's the
     batch-friendly glTF about. */
  CORRADE_COMPARE_AS(filename,
                     Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf"),
                     Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR, "batch.bin"),
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.bin"),
      Cr::TestSuite::Compare::File);
}

void GfxBatchRendererTest::generateTestDataMultipleMeshes() {
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

  converter->configuration().setValue("experimentalKhrTextureKtx", true);
  converter->configuration().setValue("imageConverter", "KtxImageConverter");
  /* Bundle images in the bin file to reduce the amount of test files */
  converter->configuration().setValue("bundleImages", true);

  const Cr::Containers::String filename = Cr::Utility::Path::join(
      MAGNUMRENDERERTEST_OUTPUT_DIR, "batch-multiple-meshes.gltf");

  /* Begin file conversion. No custom scene fields used in this case. */
  converter->beginFile(filename);

  /* Separate square, circle and triangle mesh */
  CORRADE_COMPARE(
      converter->add(Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
                         Mn::Primitives::PlaneFlag::TextureCoordinates)),
                     "square"),
      0);
  CORRADE_COMPARE(
      converter->add(
          Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
              32, Mn::Primitives::Circle3DFlag::TextureCoordinates)),
          "circle"),
      1);
  CORRADE_COMPARE(
      converter->add(
          Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
              3, Mn::Primitives::Circle3DFlag::TextureCoordinates)),
          "triangle"),
      2);

  /* Two-layer 4x4 texture, same as in generateTestData() */
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
  CORRADE_VERIFY(converter->add(Mn::ImageView3D{
      Mn::PixelFormat::RGB8Unorm, {4, 4, 2}, image, Mn::ImageFlag3D::Array}));

  /* A texture referencing the only image, same as in generateTestData() */
  CORRADE_VERIFY(converter->add(Mn::Trade::TextureData{
      Mn::Trade::TextureType::Texture2DArray, Mn::SamplerFilter::Nearest,
      Mn::SamplerFilter::Nearest, Mn::SamplerMipmap::Nearest,
      Mn::SamplerWrapping::Repeat, 0}));

  /* A (default, white) material and cyan / magenta / yellow material, same as
     in generateTestData() */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "checkerboard"), 0);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.5f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "yellow"), 3);
  // clang-format on

  /* Scene with
      - a square using the checkerboard material
      - a circle using the cyan material
      - a triangle using the magenta material
      - a subtree of four child meshes translated on X and Y, each being a
        square using one of the materials
    The only difference compared to generateTestData() is using separate mesh
    IDs instead of mesh views. */
  // clang-format off
  struct Scene {
    struct Parent {
      Mn::UnsignedInt object;
      Mn::Int parent;
    } parents[11];
    struct Mesh {
      Mn::UnsignedInt object;
      Mn::UnsignedInt mesh;
      Mn::Int meshMaterial;
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
    {{ 1, 0, 0},
     { 3, 1, 1},
     { 5, 2, 2},
     { 7, 0, 0},
     { 8, 0, 1},
     { 9, 0, 2},
     {10, 0, 3}},
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
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::MeshMaterial,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshMaterial)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Transformation,
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::object),
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::trasformation)},
  }}));
  // clang-format on

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
     test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
     executable) */
  CORRADE_COMPARE_AS(
      filename,
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch-multiple-meshes.gltf"),
      Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR,
                              "batch-multiple-meshes.bin"),
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch-multiple-meshes.bin"),
      Cr::TestSuite::Compare::File);
}

void GfxBatchRendererTest::generateTestDataMultipleTextures() {
  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter>
      imageConverterManager;
  if (imageConverterManager.loadState("PngImageConverter") ==
      Cr::PluginManager::LoadState::NotFound)
    CORRADE_SKIP("PngImageConverter plugin not found");

  Cr::PluginManager::Manager<Mn::Trade::AbstractSceneConverter>
      converterManager;
  converterManager.registerExternalManager(imageConverterManager);
  Cr::Containers::Pointer<Mn::Trade::AbstractSceneConverter> converter =
      converterManager.loadAndInstantiate("GltfSceneConverter");
  if (!converter)
    CORRADE_SKIP("GltfSceneConverter plugin not found");

  /* Using just plain PNGs, no need for KTX in this case */
  converter->configuration().setValue("imageConverter", "PngImageConverter");
  /* Bundle images in the bin file to reduce the amount of test files */
  converter->configuration().setValue("bundleImages", true);
  /* To prevent the file from being opened by unsuspecting libraries */
  converter->configuration().addValue("extensionUsed", "MAGNUMX_mesh_views");
  converter->configuration().addValue("extensionRequired",
                                      "MAGNUMX_mesh_views");

  const Cr::Containers::String filename = Cr::Utility::Path::join(
      MAGNUMRENDERERTEST_OUTPUT_DIR, "batch-multiple-textures.gltf");

  /* Begin file conversion */
  converter->beginFile(filename);
  converter->setSceneFieldName(SceneFieldMeshViewIndexOffset,
                               "meshViewIndexOffset");
  converter->setSceneFieldName(SceneFieldMeshViewIndexCount,
                               "meshViewIndexCount");
  converter->setSceneFieldName(SceneFieldMeshViewMaterial, "meshViewMaterial");

  /* (Flat) square, circle and triangle mesh. All made indexed and joined
     together. Important: offsets are in bytes. */
  Mn::Trade::MeshData square =
      Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
          Mn::Primitives::PlaneFlag::TextureCoordinates));
  Mn::UnsignedInt squareIndexOffset = 0;

  Mn::Trade::MeshData circle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          32, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt circleIndexOffset =
      squareIndexOffset + 4 * square.indexCount();

  Mn::Trade::MeshData triangle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          3, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt triangleIndexOffset =
      circleIndexOffset + 4 * circle.indexCount();

  CORRADE_VERIFY(
      converter->add(Mn::MeshTools::concatenate({square, circle, triangle})));

  /* Two-layer 4x4 image from the above tests, but compared to
     generateTestData() split into individual slices, each with its
     corresponding texture. The black portion was intentionally unused so it's
     not present here. */
  // clang-format off
  Mn::Color3ub checkerboard[4*4*2] {
    0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb,
    0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb,
    0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb,
    0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb
  };
  Mn::Color3ub cyan[]{
    0x33cccc_rgb, 0x33cccc_rgb,
    0x33cccc_rgb, 0x33cccc_rgb,
  };
  Mn::Color3ub magenta[]{
    0xcc33cc_rgb, 0xcc33cc_rgb,
    0xcc33cc_rgb, 0xcc33cc_rgb,
  };
  Mn::Color3ub yellow[]{
    0xcccc33_rgb, 0xcccc33_rgb,
    0xcccc33_rgb, 0xcccc33_rgb,
  };
  // clang-format on

  CORRADE_COMPARE(
      converter->add(
          Mn::ImageView2D{Mn::PixelFormat::RGB8Unorm, {4, 4}, checkerboard},
          "checkerboard"),
      0);
  CORRADE_COMPARE(
      converter->add(Mn::Trade::TextureData{Mn::Trade::TextureType::Texture2D,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerMipmap::Nearest,
                                            Mn::SamplerWrapping::Repeat, 0},
                     "checkerboard"),
      0);

  CORRADE_COMPARE(
      converter->add(Mn::ImageView2D{Mn::PixelStorage{}.setAlignment(1),
                                     Mn::PixelFormat::RGB8Unorm,
                                     {2, 2},
                                     cyan},
                     "cyan"),
      1);
  CORRADE_COMPARE(
      converter->add(Mn::Trade::TextureData{Mn::Trade::TextureType::Texture2D,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerMipmap::Nearest,
                                            Mn::SamplerWrapping::Repeat, 1},
                     "cyan"),
      1);

  CORRADE_COMPARE(
      converter->add(Mn::ImageView2D{Mn::PixelStorage{}.setAlignment(1),
                                     Mn::PixelFormat::RGB8Unorm,
                                     {2, 2},
                                     magenta},
                     "magenta"),
      2);
  CORRADE_COMPARE(
      converter->add(Mn::Trade::TextureData{Mn::Trade::TextureType::Texture2D,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerMipmap::Nearest,
                                            Mn::SamplerWrapping::Repeat, 2},
                     "magenta"),
      2);

  CORRADE_COMPARE(
      converter->add(Mn::ImageView2D{Mn::PixelStorage{}.setAlignment(1),
                                     Mn::PixelFormat::RGB8Unorm,
                                     {2, 2},
                                     yellow},
                     "yellow"),
      3);
  CORRADE_COMPARE(
      converter->add(Mn::Trade::TextureData{Mn::Trade::TextureType::Texture2D,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerFilter::Nearest,
                                            Mn::SamplerMipmap::Nearest,
                                            Mn::SamplerWrapping::Repeat, 3},
                     "yellow"),
      3);

  /* A (default, white) material with the checkerboard texture; cyan / magenta
     / yellow materials referencing the other three textures */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u}
  }}, "checkerboard"), 0);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 1u},
  }}, "cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 2u},
  }}, "magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 3u},
  }}, "yellow"), 3);
  // clang-format on

  /* Scene with
      - a square using the checkerboard material
      - a circle using the cyan material
      - a triangle using the magenta material
      - a subtree of four child meshes translated on X and Y, each being a
        square using one of the materials
    This is again the same as in generateTestData(). */
  // clang-format off
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
    {{ 1, 0, squareIndexOffset, square.indexCount(), 0},
     { 3, 0, circleIndexOffset, circle.indexCount(), 1},
     { 5, 0, triangleIndexOffset, triangle.indexCount(), 2},
     { 7, 0, squareIndexOffset, square.indexCount(), 0},
     { 8, 0, squareIndexOffset, square.indexCount(), 1},
     { 9, 0, squareIndexOffset, square.indexCount(), 2},
     {10, 0, squareIndexOffset, square.indexCount(), 3}},
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
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexOffset,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexOffset)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexCount,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexCount)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewMaterial,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewMaterial)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Transformation,
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::object),
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::trasformation)},
  }}));
  // clang-format on

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
     test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
     executable). */
  CORRADE_COMPARE_AS(filename,
                     Cr::Utility::Path::join(
                         TEST_ASSETS, "scenes/batch-multiple-textures.gltf"),
                     Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR,
                                             "batch-multiple-textures.bin"),
                     Cr::Utility::Path::join(
                         TEST_ASSETS, "scenes/batch-multiple-textures.bin"),
                     Cr::TestSuite::Compare::File);
}

void GfxBatchRendererTest::generateTestDataSquareCircleTriangle() {
  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter>
      imageConverterManager;
  if (imageConverterManager.loadState("PngImageConverter") ==
      Cr::PluginManager::LoadState::NotFound)
    CORRADE_SKIP("PngImageConverter plugin not found");

  Cr::PluginManager::Manager<Mn::Trade::AbstractSceneConverter>
      converterManager;
  converterManager.registerExternalManager(imageConverterManager);
  Cr::Containers::Pointer<Mn::Trade::AbstractSceneConverter> converter =
      converterManager.loadAndInstantiate("GltfSceneConverter");
  if(!converter)
    CORRADE_SKIP("GltfSceneConverter plugin not found");

  /* Bundle images in the bin file to reduce the amount of test files */
  converter->configuration().setValue("bundleImages", true);
  /* To prevent the file from being opened by unsuspecting libraries */
  converter->configuration().addValue("extensionUsed", "MAGNUMX_mesh_views");
  converter->configuration().addValue("extensionRequired",
                                      "MAGNUMX_mesh_views");

  const Cr::Containers::String filename = Cr::Utility::Path::join(
      MAGNUMRENDERERTEST_OUTPUT_DIR, "batch-square-circle-triangle.gltf");

  /* Begin file conversion */
  converter->beginFile(filename);
  converter->setSceneFieldName(SceneFieldMeshViewIndexOffset,
                               "meshViewIndexOffset");
  converter->setSceneFieldName(SceneFieldMeshViewIndexCount,
                               "meshViewIndexCount");
  converter->setSceneFieldName(SceneFieldMeshViewMaterial, "meshViewMaterial");

  /* (Flat) square, circle and triangle mesh, same as in generateTestData() */
  Mn::Trade::MeshData square =
      Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
          Mn::Primitives::PlaneFlag::TextureCoordinates));
  Mn::UnsignedInt squareIndexOffset = 0;

  Mn::Trade::MeshData circle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          32, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt circleIndexOffset =
      squareIndexOffset + 4 * square.indexCount();

  Mn::Trade::MeshData triangle =
      Mn::MeshTools::generateIndices(Mn::Primitives::circle3DSolid(
          3, Mn::Primitives::Circle3DFlag::TextureCoordinates));
  Mn::UnsignedInt triangleIndexOffset =
      circleIndexOffset + 4 * circle.indexCount();

  CORRADE_VERIFY(
      converter->add(Mn::MeshTools::concatenate({square, circle, triangle})));

  /* A checkerboard image, first layer of the image in generateTestData(), just
     as a 2D texture */
  // clang-format off
  Mn::Color3ub image[4*4*1] {
    0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb,
    0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb,
    0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb,
    0xcccccc_rgb, 0x990000_rgb, 0xcccccc_rgb, 0x990000_rgb
  };
  // clang-format on
  CORRADE_VERIFY(converter->add(Mn::ImageView2D{
      Mn::PixelFormat::RGB8Unorm, {4, 4}, image}));
  CORRADE_VERIFY(converter->add(Mn::Trade::TextureData{
      Mn::Trade::TextureType::Texture2D, Mn::SamplerFilter::Nearest,
      Mn::SamplerFilter::Nearest, Mn::SamplerMipmap::Nearest,
      Mn::SamplerWrapping::Repeat, 0}));

  /* Checkerboard / cyan / magenta material. The checkerboard is same as in
     generateTestData(), the other two materials are textureless. */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "checkerboard"), 0);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf*0x33ccccff_rgbaf},
  }}, "cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf*0xcc33ccff_rgbaf},
  }}, "magenta"), 2);
  // clang-format on

  /* Scene with
      - a square using the checkerboard material
      - a circle using the cyan material
      - a triangle using the magenta material
      Same as in generateTestData(), no transformation needed in this case. */
  // clang-format off
  struct Scene {
    struct Parent {
      Mn::UnsignedInt object;
      Mn::Int parent;
    } parents[6];
    struct Mesh {
      Mn::UnsignedInt object;
      Mn::UnsignedInt mesh;
      Mn::UnsignedInt meshViewIndexOffset;
      Mn::UnsignedInt meshViewIndexCount;
      Mn::Int meshViewMaterial;
    } meshes[3];
  } scene[]{{
    {{0, -1}, {1, 0},  /* square and its child mesh */
     {2, -1}, {3, 2},  /* circle and its child mesh */
     {4, -1}, {5, 4}}, /* triangle and its child mesh */
    {{ 1, 0, squareIndexOffset, square.indexCount(), 0},
     { 3, 0, circleIndexOffset, circle.indexCount(), 1},
     { 5, 0, triangleIndexOffset, triangle.indexCount(), 2}},
  }};
  converter->setObjectName(0, "square");
  converter->setObjectName(2, "circle");
  converter->setObjectName(4, "triangle");
  CORRADE_VERIFY(converter->add(Mn::Trade::SceneData{Mn::Trade::SceneMappingType::UnsignedInt, 6, {}, scene, {
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Parent,
      Cr::Containers::stridedArrayView(scene->parents).slice(&Scene::Parent::object),
      Cr::Containers::stridedArrayView(scene->parents).slice(&Scene::Parent::parent)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Mesh,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::mesh)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexOffset,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexOffset)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewIndexCount,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewIndexCount)},
    Mn::Trade::SceneFieldData{SceneFieldMeshViewMaterial,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshViewMaterial)},
    /* We don't need any transformation, using it just to properly mark the
        scene as 3D */
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Transformation,
      Mn::Trade::SceneMappingType::UnsignedInt, nullptr,
      Mn::Trade::SceneFieldType::Matrix4x4, nullptr
    },
  }}));
  // clang-format on

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
      test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
      executable) */
  CORRADE_COMPARE_AS(filename,
                      Cr::Utility::Path::join(
                          TEST_ASSETS, "scenes/batch-square-circle-triangle.gltf"),
                      Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR,
                                              "batch-square-circle-triangle.bin"),
                      Cr::Utility::Path::join(
                          TEST_ASSETS, "scenes/batch-square-circle-triangle.bin"),
                      Cr::TestSuite::Compare::File);
}

void GfxBatchRendererTest::generateTestDataFourSquares() {
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
  if(!converter)
    CORRADE_SKIP("GltfSceneConverter plugin not found");

  converter->configuration().setValue("imageConverter", "KtxImageConverter");
  converter->configuration().setValue("experimentalKhrTextureKtx", true);
  /* Bundle images in the bin file to reduce the amount of test files */
  converter->configuration().setValue("bundleImages", true);

  const Cr::Containers::String filename = Cr::Utility::Path::join(
      MAGNUMRENDERERTEST_OUTPUT_DIR, "batch-four-squares.gltf");

  /* Begin file conversion */
  converter->beginFile(filename);

  /* (Flat) square mesh. Used with four different materials so it gets
     duplicated in the glTF. */
  CORRADE_VERIFY(converter->add(Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
          Mn::Primitives::PlaneFlag::TextureCoordinates)), "square"));

  /* Two-layer 4x4 texture, same as in generateTestData() */
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
  CORRADE_VERIFY(converter->add(Mn::ImageView3D{
      Mn::PixelFormat::RGB8Unorm, {4, 4, 2}, image, Mn::ImageFlag3D::Array}));

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
  }}, "checkerboard"), 0);
  // clang-format on

  /* A cyan / magenta / yellow material spanning the bottom left / top left /
     bottom right quadrant of second texture layer. I.e., nothing should be
     using the black-ish portion of the texture. When combined with the
     texture color, the output should have the corresponding red / green / blue
     channels zeroed out. */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.5f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "yellow"), 3);
  // clang-format on

  /* Scene with
      - a square using the checkerboard material
      - a subtree of four child meshes translated on X and Y, each being a
        square using one of the materials
    A subset of what's in generateTestData() except the circle and triangle
    mesh that's in the first file instead. */
  // clang-format off
  struct Scene {
    struct Parent {
      Mn::UnsignedInt object;
      Mn::Int parent;
    } parents[5];
    struct Mesh {
      Mn::UnsignedInt object;
      Mn::UnsignedInt mesh;
      Mn::Int meshMaterial;
    } meshes[4];
    struct Transformation {
      Mn::UnsignedInt object;
      Mn::Matrix4 trasformation;
    } transformations[4];
  } scene[]{{
    {{0, -1}, {1, 0}, {2, 0}, /* four squares */
              {3, 0}, {4, 0}},
    {{1, 0, 0},
     {2, 0, 1},
     {3, 0, 2},
     {4, 0, 3}},
    {{1, Mn::Matrix4::translation({-0.5f, -0.5f, 0.0f})*
         Mn::Matrix4::scaling(Mn::Vector3{0.4f})},
     {2, Mn::Matrix4::translation({+0.5f, -0.5f, 0.0f})*
         Mn::Matrix4::scaling(Mn::Vector3{0.4f})},
     {3, Mn::Matrix4::translation({-0.5f, +0.5f, 0.0f})*
         Mn::Matrix4::scaling(Mn::Vector3{0.4f})},
     {4, Mn::Matrix4::translation({+0.5f, +0.5f, 0.0f})*
         Mn::Matrix4::scaling(Mn::Vector3{0.4f})}}
  }};
  converter->setObjectName(0, "four squares");
  CORRADE_VERIFY(converter->add(Mn::Trade::SceneData{Mn::Trade::SceneMappingType::UnsignedInt, 7, {}, scene, {
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Parent,
      Cr::Containers::stridedArrayView(scene->parents).slice(&Scene::Parent::object),
      Cr::Containers::stridedArrayView(scene->parents).slice(&Scene::Parent::parent)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Mesh,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::mesh)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::MeshMaterial,
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::object),
      Cr::Containers::stridedArrayView(scene->meshes).slice(&Scene::Mesh::meshMaterial)},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Transformation,
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::object),
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::trasformation)},
  }}));
  // clang-format on

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
      test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
      executable). */
  CORRADE_COMPARE_AS(
      filename,
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch-four-squares.gltf"),
      Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR,
                              "batch-four-squares.bin"),
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch-four-squares.bin"),
      Cr::TestSuite::Compare::File);
}

void GfxBatchRendererTest::defaults() {
  auto&& data = FileData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({48, 32}, {2, 3}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          /* No QuietLog for the first instance, to verify at least once that
             the log *is* printed */
          .setFlags(testCaseInstanceId() == 0 ? esp::gfx_batch::RendererStandaloneFlags{} : esp::gfx_batch::RendererStandaloneFlag::QuietLog)
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

    /* Nothing in the stats */
    esp::gfx_batch::SceneStats stats = renderer.sceneStats(i);
    CORRADE_COMPARE(stats.nodeCount, 0);
    CORRADE_COMPARE(stats.drawCount, 0);
    CORRADE_COMPARE(stats.drawBatchCount, 0);
  }

  /* Add a file, because that's currently required */
  // TODO make it non-required (instantiate some empty shader if nothing)
  for (const char* file : data.gltfFilenames)
    renderer.addFile(Cr::Utility::Path::join({TEST_ASSETS, "scenes", file}));

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
  auto&& data = FileData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  for (const char* file : data.gltfFilenames)
    renderer.addFile(Cr::Utility::Path::join({TEST_ASSETS, "scenes", file}));

  /* Undo the aspect ratio, move camera back */
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "square"), 0);

  /* Stats will show two nodes now -- it adds one transformation for the
     top-level object and then one nested for the mesh, corresponding to the
     layout inside the glTF file */
  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  CORRADE_COMPARE(stats.nodeCount, 2);
  CORRADE_COMPARE(stats.drawCount, 1);
  CORRADE_COMPARE(stats.drawBatchCount, 1);

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

  for (const char* file : data.gltfFilenames)
    renderer.addFile(Cr::Utility::Path::join({TEST_ASSETS, "scenes", file}));

  /* Undo the aspect ratio, move camera back */
  renderer.camera(0) =
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f) *
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted();

  CORRADE_COMPARE(renderer.addMeshHierarchy(0, "four squares"), 0);

  /* Stats will show five nodes now -- it adds one transformation for the
     top-level object and then four nested for each mesh, corresponding to the
     layout inside the glTF file. The batch count reflects how many separate
     meshes and textures there are. */
  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  CORRADE_COMPARE(stats.nodeCount, 5);
  CORRADE_COMPARE(stats.drawCount, 4);
  CORRADE_COMPARE(stats.drawBatchCount, data.batchCount);

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
  auto&& data = FileData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  for (const char* file : data.gltfFilenames)
    renderer.addFile(Cr::Utility::Path::join({TEST_ASSETS, "scenes", file}));

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

  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  CORRADE_COMPARE(stats.nodeCount, 6);
  CORRADE_COMPARE(stats.drawCount, 3);
  CORRADE_COMPARE(stats.drawBatchCount, data.singleSceneBatchCount);

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
  auto&& data = FileData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({64, 48}, {2, 2}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  for (const char* file : data.gltfFilenames)
    renderer.addFile(Cr::Utility::Path::join({TEST_ASSETS, "scenes", file}));

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

  esp::gfx_batch::SceneStats stats0 = renderer.sceneStats(0);
  CORRADE_COMPARE(stats0.nodeCount, 5);
  CORRADE_COMPARE(stats0.drawCount, 4);
  CORRADE_COMPARE(stats0.drawBatchCount, data.multipleScenesBatchCount[0]);

  esp::gfx_batch::SceneStats stats1 = renderer.sceneStats(1);
  CORRADE_COMPARE(stats1.nodeCount, 4);
  CORRADE_COMPARE(stats1.drawCount, 2);
  CORRADE_COMPARE(stats1.drawBatchCount, data.multipleScenesBatchCount[1]);

  esp::gfx_batch::SceneStats stats2 = renderer.sceneStats(2);
  CORRADE_COMPARE(stats2.nodeCount, 0);
  CORRADE_COMPARE(stats2.drawCount, 0);
  CORRADE_COMPARE(stats2.drawBatchCount, data.multipleScenesBatchCount[2]);

  esp::gfx_batch::SceneStats stats3 = renderer.sceneStats(3);
  CORRADE_COMPARE(stats3.nodeCount, 2);
  CORRADE_COMPARE(stats3.drawCount, 1);
  CORRADE_COMPARE(stats3.drawBatchCount, data.multipleScenesBatchCount[3]);

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
  auto&& data = FileData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({64, 48}, {2, 2}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  for (const char* file : data.gltfFilenames)
    renderer.addFile(Cr::Utility::Path::join({TEST_ASSETS, "scenes", file}));

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
  esp::gfx_batch::SceneStats stats1 = renderer.sceneStats(1);
  CORRADE_COMPARE(stats1.nodeCount, 0);
  CORRADE_COMPARE(stats1.drawCount, 0);

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
