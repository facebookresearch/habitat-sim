// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Triple.h>
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
#include <Magnum/Math/Range.h>
#include <Magnum/MeshTools/Combine.h>
#include <Magnum/MeshTools/Concatenate.h>
#include <Magnum/MeshTools/GenerateIndices.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/SceneTools/Hierarchy.h>
#include <Magnum/Trade/AbstractImageConverter.h>
#include <Magnum/Trade/AbstractSceneConverter.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>

#include <esp/gfx_batch/DepthUnprojection.h>
#include "Corrade/TestSuite/Compare/Numeric.h"
#include "esp/gfx_batch/RendererStandalone.h"

#ifdef ESP_BUILD_WITH_CUDA
#include <cuda_gl_interop.h>
#endif

#include "configure.h"

namespace {

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Cr::Containers::Literals;
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

  void renderNoFileAdded();
  void multipleScenes();
  void clearScene();

  void lights();
  void clearLights();

  void imageInto();
  void depthUnprojection();
  void cudaInterop();
};

// clang-format off
const struct {
  const char* name;
  bool wholeFile;
  bool scene;
  bool sceneDeepHierarchy;
  bool materials;
  const char* filenamePrefix;
} GenerateTestDataFourSquaresData[]{
  {"", false, true, false, true, "batch-four-squares"},
  {"no materials", false, true, false, false, "batch-four-squares-no-materials"},
  {"deep scene hierarchy, as a whole file", true, true, true, true, "batch-four-squares-deep-hierarchy-whole-file"},
  {"no scene, as a whole file", true, false, false, false, "batch-four-squares-no-scene"}
};
// clang-format on

// clang-format off
const struct {
  const char* name;
  /* Filename, flags and name corresponding to addFile() arguments */
  Cr::Containers::Array<Cr::Containers::Triple<const char*, esp::gfx_batch::RendererFileFlags, const char*>> gltfFilenames;
  Mn::UnsignedInt singleSceneBatchCount;
  Mn::UnsignedInt multipleScenesBatchCount[4];
  Mn::Float maxThreshold, meanThreshold;
} FileData[]{
  {"", {Cr::InPlaceInit, {
    {"batch.gltf", {}, nullptr}}},
    1,
    {1, 1, 0, 1},
    0.0f, 0.0f},
  /* Doesn't really verify that the right level count is generated, but at
     least checks that things don't crash */
  {"generate mipmap", {Cr::InPlaceInit, {
    {"batch.gltf", esp::gfx_batch::RendererFileFlag::GenerateMipmap, nullptr}}},
    1,
    {1, 1, 0, 1},
    0.0f, 0.0f},
  {"multiple meshes", {Cr::InPlaceInit, {
    {"batch-multiple-meshes.gltf", {}, nullptr}}},
    /* Each has a separate mesh */
    3,
    {4, 2, 0, 1},
    0.0f, 0.0f},
  {"multiple textures", {Cr::InPlaceInit, {
    {"batch-multiple-textures.gltf", {}, nullptr}}},
    /* Each has a separate texture */
    3,
    {4, 2, 0, 1},
    0.0f, 0.0f},
  /* Doesn't really verify that the right level count is generated, but at
     least checks that things don't crash */
  {"multiple textures, generate mipmap", {Cr::InPlaceInit, {
    {"batch-multiple-textures.gltf", esp::gfx_batch::RendererFileFlag::GenerateMipmap, nullptr}}},
    /* Each has a separate texture */
    3,
    {4, 2, 0, 1},
    0.0f, 0.0f},
  {"multiple files", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle.gltf", {}, nullptr},
    {"batch-four-squares.gltf", {}, nullptr}}},
    /* Square, circle and triangle are a single mesh but the hierarchical four
       squares are a separate file */
    2,
    {4, 2, 0, 1},
    0.0f, 0.0f},
  {"multiple files, compressed textures", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle-compressed.gltf", {}, nullptr},
    {"batch-four-squares-compressed.gltf", {}, nullptr}}},
    /* Square, circle and triangle are a single mesh but the hierarchical four
       squares are a separate file */
    2,
    {4, 2, 0, 1},
    /* DXT-compressed images have minor compression errors */
    1.5f, 0.5f},
  /* Mip level generation should do nothing for compressed textures */
  {"multiple files, compressed textures, generate mipmap", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle-compressed.gltf", esp::gfx_batch::RendererFileFlag::GenerateMipmap, nullptr},
    {"batch-four-squares-compressed.gltf", esp::gfx_batch::RendererFileFlag::GenerateMipmap, nullptr}}},
    2,
    {4, 2, 0, 1},
    1.5f, 0.5f},
  {"multiple files, some whole-file", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle.gltf", {}, nullptr},
    {"batch-four-squares-deep-hierarchy-whole-file.gltf", esp::gfx_batch::RendererFileFlag::Whole, "four squares"}}},
    /* Square, circle and triangle are a single mesh but the hierarchical four
       squares are a separate file */
    2,
    {4, 2, 0, 1},
    0.0f, 0.0f},
};

const struct {
  const char* name;
  /* Filename, flags and name corresponding to addFile() arguments */
  Cr::Containers::Array<Cr::Containers::Triple<const char*, esp::gfx_batch::RendererFileFlags, const char*>> gltfFilenames;
  esp::gfx_batch::RendererFlags flags;
  Mn::UnsignedInt nodeCount, drawCount, batchCount;
  Mn::Float textureMultiplier;
  const char* filename;
} MeshHierarchyData[]{
  {"", {Cr::InPlaceInit, {
    {"batch.gltf", {}, nullptr}}},
    {}, 5, 4, 1, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple meshes", {Cr::InPlaceInit, {
    {"batch-multiple-meshes.gltf", {}, nullptr}}},
    {}, 5, 4, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple textures", {Cr::InPlaceInit, {
    {"batch-multiple-textures.gltf", {}, nullptr}}},
    {}, 5, 4, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple files", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle.gltf", {}, nullptr},
    {"batch-four-squares.gltf", {}, nullptr}}},
    {}, 5, 4, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple files with deep hierarchy as a whole", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle.gltf",
      esp::gfx_batch::RendererFileFlag::Whole, nullptr},
    {"batch-four-squares-deep-hierarchy-whole-file.gltf",
      esp::gfx_batch::RendererFileFlag::Whole, "four squares"}}},
    {}, 5, 4, 4, 0xcc/255.0f,
    "GfxBatchRendererTestMeshHierarchy.png"},
  {"multiple files, no materials", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle.gltf", {}, nullptr},
    {"batch-four-squares-no-materials.gltf", {}, nullptr}}},
    {}, 5, 4, 4, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"},
  {"no textures", {Cr::InPlaceInit, {
    {"batch.gltf", {}, nullptr}}},
    esp::gfx_batch::RendererFlag::NoTextures, 5, 4, 1, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"},
  {"multiple meshes, no textures", {Cr::InPlaceInit, {
    {"batch-multiple-meshes.gltf", {}, nullptr}}},
    esp::gfx_batch::RendererFlag::NoTextures, 5, 4, 4, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"},
  {"multiple files, scene-less, as a whole", {Cr::InPlaceInit, {
    {"batch-square-circle-triangle.gltf",
      esp::gfx_batch::RendererFileFlag::Whole, nullptr},
    {"batch-four-squares-no-scene.gltf",
      esp::gfx_batch::RendererFileFlag::Whole, "four squares"}}},
    {}, 2, 1, 1, 1.0f,
    "GfxBatchRendererTestMeshHierarchyNoTextures.png"},
};

const struct {
  const char* name;
  Mn::UnsignedInt maxLightCount;
  Mn::UnsignedInt sceneLightCount;
  Mn::Deg rotation;
  const char* firstObject;
  const char* secondObject;
  const char* thirdObject;
  const char* expected;
} LightData[]{
  {"no lights enabled", 0, 0, 0.0_degf,
    "flat checkerboard sphere",
    "shaded yellow sphere",
    "shaded checkerboard sphere",
    "GfxBatchRendererTestLightsDisabled.png"},
  {"no lights added", 2, 0, 0.0_degf,
    "flat checkerboard sphere",
    "shaded yellow sphere",
    "shaded checkerboard sphere",
    "GfxBatchRendererTestLightsNone.png"},
  {"two lights", 2, 2, 0.0_degf,
    "flat checkerboard sphere",
    "shaded yellow sphere",
    "shaded checkerboard sphere",
    "GfxBatchRendererTestLights.png"},
  {"two lights, rotated", 2, 2, 180.0_degf,
    "flat checkerboard sphere",
    "shaded yellow sphere",
    "shaded checkerboard sphere",
    "GfxBatchRendererTestLights.png"},
  {"four lights used but two max", 2, 4, 0.0_degf,
    "flat checkerboard sphere",
    "shaded yellow sphere",
    "shaded checkerboard sphere",
    "GfxBatchRendererTestLights.png"},
  {"two lights used but four max", 4, 2, 0.0_degf,
    "flat checkerboard sphere",
    "shaded yellow sphere",
    "shaded checkerboard sphere",
    "GfxBatchRendererTestLights.png"},
  {"two lights but everything flat-shaded", 2, 2, 0.0_degf,
    "flat checkerboard sphere",
    "flat yellow sphere",
    "flat checkerboard sphere",
    "GfxBatchRendererTestLightsDisabled.png"},
};
// clang-format on

GfxBatchRendererTest::GfxBatchRendererTest() {
  // clang-format off
  addTests({&GfxBatchRendererTest::generateTestData,
            &GfxBatchRendererTest::generateTestDataMultipleMeshes,
            &GfxBatchRendererTest::generateTestDataMultipleTextures,
            &GfxBatchRendererTest::generateTestDataSquareCircleTriangle});

  addInstancedTests({&GfxBatchRendererTest::generateTestDataFourSquares},
      Cr::Containers::arraySize(GenerateTestDataFourSquaresData));

  addInstancedTests({&GfxBatchRendererTest::defaults,
                     &GfxBatchRendererTest::singleMesh},
      Cr::Containers::arraySize(FileData));

  addInstancedTests({&GfxBatchRendererTest::meshHierarchy},
      Cr::Containers::arraySize(MeshHierarchyData));

  addTests({&GfxBatchRendererTest::renderNoFileAdded});

  addInstancedTests({&GfxBatchRendererTest::multipleMeshes,
                     &GfxBatchRendererTest::multipleScenes,
                     &GfxBatchRendererTest::clearScene},
      Cr::Containers::arraySize(FileData));

  addInstancedTests({&GfxBatchRendererTest::lights},
      Cr::Containers::arraySize(LightData));

  addTests({&GfxBatchRendererTest::clearLights,
            &GfxBatchRendererTest::imageInto,
            &GfxBatchRendererTest::depthUnprojection,
            &GfxBatchRendererTest::cudaInterop});
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

  /* Square, circle, triangle, and a sphere mesh. All made indexed and joined
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

  Mn::Trade::MeshData sphere = Mn::Primitives::uvSphereSolid(
      8, 16, Mn::Primitives::UVSphereFlag::TextureCoordinates);
  Mn::UnsignedInt sphereIndexOffset =
      triangleIndexOffset + 4 * triangle.indexCount();

  CORRADE_VERIFY(converter->add(
      Mn::MeshTools::concatenate({square, circle, triangle, sphere})));

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

  /* A (default, white) flat material spanning the whole first texture layer */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "flat checkerboard"), 0);
  // clang-format on

  /* A cyan / magenta / yellow flat material spanning the bottom left / top
     left / bottom right quadrant of second texture layer. I.e., nothing should
     be using the black-ish portion of the texture. When combined with the
     texture color, the output should have the corresponding red / green / blue
     channels zeroed out. */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.5f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat yellow"), 3);
  // clang-format on

  /* A flat and non-flat yellowish checkerboard material used to render the
     sphere with lights */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff99ff_rgbaf}
  }}, "flat yellow"), 4);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff99ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "flat checkerboard"), 5);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff99ff_rgbaf}
  }}, "shaded yellow"), 6);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{{}, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff99ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "shaded checkerboard"), 7);
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
    } parents[19];
    struct Mesh {
      Mn::UnsignedInt object;
      Mn::UnsignedInt mesh;
      Mn::UnsignedInt meshViewIndexOffset;
      Mn::UnsignedInt meshViewIndexCount;
      Mn::Int meshViewMaterial;
    } meshes[11];
    struct Transformation {
      Mn::UnsignedInt object;
      Mn::Matrix4 transformation;
    } transformations[4];
  } scene[]{{
    {{0, -1}, {1, 0}, /* square and its child mesh */
     {2, -1}, {3, 2}, /* circle and its child mesh */
     {4, -1}, {5, 4}, /* triangle and its child mesh */
     {6, -1}, {7, 6}, {8, 6}, /* four squares */
              {9, 6}, {10, 6},
     {11, -1}, {12, 11}, /* flat yellow sphere and its child mesh */
     {13, -1}, {14, 13}, /* flat checkerboard sphere and its child mesh */
     {15, -1}, {16, 15}, /* shaded yellow sphere and its child mesh */
     {17, -1}, {18, 17}}, /* shaded checkerboard sphere and its child mesh */
    {{ 1, 0, squareIndexOffset, square.indexCount(), 0},
     { 3, 0, circleIndexOffset, circle.indexCount(), 1},
     { 5, 0, triangleIndexOffset, triangle.indexCount(), 2},
     { 7, 0, squareIndexOffset, square.indexCount(), 0},
     { 8, 0, squareIndexOffset, square.indexCount(), 1},
     { 9, 0, squareIndexOffset, square.indexCount(), 2},
     {10, 0, squareIndexOffset, square.indexCount(), 3},
     {12, 0, sphereIndexOffset, sphere.indexCount(), 4},
     {14, 0, sphereIndexOffset, sphere.indexCount(), 5},
     {16, 0, sphereIndexOffset, sphere.indexCount(), 6},
     {18, 0, sphereIndexOffset, sphere.indexCount(), 7}},
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
  converter->setObjectName(11, "flat yellow sphere");
  converter->setObjectName(13, "flat checkerboard sphere");
  converter->setObjectName(15, "shaded yellow sphere");
  converter->setObjectName(17, "shaded checkerboard sphere");
  CORRADE_VERIFY(converter->add(Mn::Trade::SceneData{Mn::Trade::SceneMappingType::UnsignedInt, Cr::Containers::arraySize(scene->parents), {}, scene, {
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
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::transformation)},
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

  /* Separate square... */
  CORRADE_COMPARE(
      converter->add(Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
                         Mn::Primitives::PlaneFlag::TextureCoordinates)),
                     "square"),
      0);

  /* ... circle, which is a non-indexed triangle fan ... */
  CORRADE_COMPARE(
      converter->add(Mn::Primitives::circle3DSolid(
                         32, Mn::Primitives::Circle3DFlag::TextureCoordinates),
                     "circle"),
      1);

  /* ... and a triangle with additional magenta vertex colors */
  {
    /* "Triangle", it's actually a circle, a fan, so 5 vertices instead of 3 */
    Mn::Vector3 colors[]{0xff00ff_rgbf, 0xff00ff_rgbf, 0xff00ff_rgbf,
                         0xff00ff_rgbf, 0xff00ff_rgbf};
    CORRADE_COMPARE(
        converter->add(
            Mn::MeshTools::generateIndices(Mn::MeshTools::interleave(
                Mn::Primitives::circle3DSolid(
                    3, Mn::Primitives::Circle3DFlag::TextureCoordinates),
                {Mn::Trade::MeshAttributeData{
                    Mn::Trade::MeshAttribute::Color,
                    Cr::Containers::arrayView(colors)}})),
            "triangle"),
        2);
  }

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
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "flat checkerboard"), 0);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.0f, 0.5f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat yellow"), 3);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
      Mn::Matrix3::translation({0.5f, 0.0f})*
      Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
  }}, "flat vertex-color magenta"), 4);
  // clang-format on

  /* Scene with
      - a square using the checkerboard material
      - a circle using the cyan material
      - a triangle using the vertex-color magenta material
      - a subtree of four child meshes translated on X and Y, each being a
        square using one of the first four materials
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
      Mn::Matrix4 transformation;
    } transformations[4];
  } scene[]{{
    {{0, -1}, {1, 0}, /* square and its child mesh */
     {2, -1}, {3, 2}, /* circle and its child mesh */
     {4, -1}, {5, 4}, /* triangle and its child mesh */
     {6, -1}, {7, 6}, {8, 6}, /* four squares */
              {9, 6}, {10, 6}},
    {{ 1, 0, 0},
     { 3, 1, 1},
     { 5, 2, 4}, /* using the vertex-color magenta material */
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
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::transformation)},
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
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u}
  }}, "flat checkerboard"), 0);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 1u},
  }}, "flat cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 2u},
  }}, "flat magenta"), 2);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 3u},
  }}, "flat yellow"), 3);
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
      Mn::Matrix4 transformation;
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
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::transformation)},
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
  if (!converter)
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
  CORRADE_VERIFY(converter->add(
      Mn::ImageView2D{Mn::PixelFormat::RGB8Unorm, {4, 4}, image}));
  CORRADE_VERIFY(converter->add(Mn::Trade::TextureData{
      Mn::Trade::TextureType::Texture2D, Mn::SamplerFilter::Nearest,
      Mn::SamplerFilter::Nearest, Mn::SamplerMipmap::Nearest,
      Mn::SamplerWrapping::Repeat, 0}));

  /* Checkerboard / cyan / magenta material. The checkerboard is same as in
     generateTestData(), the other two materials are textureless. */
  // clang-format off
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
    {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
  }}, "flat checkerboard"), 0);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf*0x33ccccff_rgbaf},
  }}, "flat cyan"), 1);
  CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
    {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf*0xcc33ccff_rgbaf},
  }}, "flat magenta"), 2);
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
  CORRADE_COMPARE_AS(
      filename,
      Cr::Utility::Path::join(TEST_ASSETS,
                              "scenes/batch-square-circle-triangle.gltf"),
      Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(
      Cr::Utility::Path::join(MAGNUMRENDERERTEST_OUTPUT_DIR,
                              "batch-square-circle-triangle.bin"),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "scenes/batch-square-circle-triangle.bin"),
      Cr::TestSuite::Compare::File);
}

void GfxBatchRendererTest::generateTestDataFourSquares() {
  auto&& data = GenerateTestDataFourSquaresData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

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
  converter->configuration().setValue("experimentalKhrTextureKtx", true);
  /* Bundle images in the bin file to reduce the amount of test files */
  converter->configuration().setValue("bundleImages", true);

  const Cr::Containers::String filename = Cr::Utility::Path::join(
      {MAGNUMRENDERERTEST_OUTPUT_DIR, data.filenamePrefix + ".gltf"_s});

  /* Begin file conversion */
  converter->beginFile(filename);

  /* File with materials */
  if (data.materials) {
    /* (Flat) square mesh. Used with four different materials so it gets
       duplicated in the glTF. */
    CORRADE_VERIFY(converter->add(
        Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
            Mn::Primitives::PlaneFlag::TextureCoordinates)),
        "square"));

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
    CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
      {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 0u}
    }}, "flat checkerboard"), 0);
    // clang-format on

    /* A cyan / magenta / yellow material spanning the bottom left / top left /
       bottom right quadrant of second texture layer. I.e., nothing should be
       using the black-ish portion of the texture. When combined with the
       texture color, the output should have the corresponding red / green /
       blue channels zeroed out. */
    // clang-format off
    CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
      {Mn::Trade::MaterialAttribute::BaseColor, 0x00ffffff_rgbaf},
      {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
        Mn::Matrix3::translation({0.0f, 0.0f})*
        Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
    }}, "flat cyan"), 1);
    CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
      {Mn::Trade::MaterialAttribute::BaseColor, 0xff00ffff_rgbaf},
      {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
        Mn::Matrix3::translation({0.5f, 0.0f})*
        Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
    }}, "flat magenta"), 2);
    CORRADE_COMPARE(converter->add(Mn::Trade::MaterialData{Mn::Trade::MaterialType::Flat, {
      {Mn::Trade::MaterialAttribute::BaseColor, 0xffff00ff_rgbaf},
      {Mn::Trade::MaterialAttribute::BaseColorTexture, 0u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureLayer, 1u},
      {Mn::Trade::MaterialAttribute::BaseColorTextureMatrix,
        Mn::Matrix3::translation({0.0f, 0.5f})*
        Mn::Matrix3::scaling(Mn::Vector2{0.5f})}
    }}, "flat yellow"), 3);
    // clang-format on

    /* Otherwise, if we have a scene, the colors get baked into the meshes. If
       we don't have a scene, it's handled below. */
  } else if (data.scene) {
    /* Each triangle has two faces */
    Mn::Color3 white[]{0xffffff_rgbf, 0xffffff_rgbf};
    Mn::Color3 cyan[]{0x00ffff_rgbf, 0x00ffff_rgbf};
    Mn::Color3 magenta[]{0xff00ff_rgbf, 0xff00ff_rgbf};
    Mn::Color3 yellow[]{0xffff00_rgbf, 0xffff00_rgbf};

    Mn::Trade::MeshData plane =
        Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
            Mn::Primitives::PlaneFlag::TextureCoordinates));
    CORRADE_COMPARE(
        converter->add(Mn::MeshTools::combineFaceAttributes(
                           plane, {Mn::Trade::MeshAttributeData{
                                      Mn::Trade::MeshAttribute::Color,
                                      Cr::Containers::arrayView(white)}}),
                       "white square"),
        0);
    CORRADE_COMPARE(
        converter->add(Mn::MeshTools::combineFaceAttributes(
                           plane, {Mn::Trade::MeshAttributeData{
                                      Mn::Trade::MeshAttribute::Color,
                                      Cr::Containers::arrayView(cyan)}}),
                       "cyan square"),
        1);
    CORRADE_COMPARE(
        converter->add(Mn::MeshTools::combineFaceAttributes(
                           plane, {Mn::Trade::MeshAttributeData{
                                      Mn::Trade::MeshAttribute::Color,
                                      Cr::Containers::arrayView(magenta)}}),
                       "magenta square"),
        2);
    CORRADE_COMPARE(
        converter->add(Mn::MeshTools::combineFaceAttributes(
                           plane, {Mn::Trade::MeshAttributeData{
                                      Mn::Trade::MeshAttribute::Color,
                                      Cr::Containers::arrayView(yellow)}}),
                       "yellow square"),
        3);
  }

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
      Mn::Matrix4 transformation;
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
  Mn::Trade::SceneData sceneData{Mn::Trade::SceneMappingType::UnsignedInt, 7, {}, scene, {
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
      Cr::Containers::stridedArrayView(scene->transformations).slice(&Scene::Transformation::transformation)},
  }};
  // clang-format on

  /* Override the above if we have no materials -- then the materials are -1
     and different meshes are used instead */
  if (data.scene && !data.materials) {
    scene->meshes[0].meshMaterial = -1;
    scene->meshes[1].meshMaterial = -1;
    scene->meshes[2].meshMaterial = -1;
    scene->meshes[3].meshMaterial = -1;
    scene->meshes[0].mesh = 0;
    scene->meshes[1].mesh = 1;
    scene->meshes[2].mesh = 2;
    scene->meshes[3].mesh = 3;
  }

  /* Override the above if deep hierarchy is requested -- then the "right"
     squares are children of the left, with relative transform */
  if (data.sceneDeepHierarchy) {
    scene->parents[2].parent = 1;
    scene->parents[4].parent = 3;
    scene->transformations[1].transformation =
        Mn::Matrix4::translation(Mn::Vector3::xAxis(1.0f / 0.4f));
    scene->transformations[3].transformation =
        Mn::Matrix4::translation(Mn::Vector3::xAxis(1.0f / 0.4f));
  }

  /* File with a scene */
  if (data.scene) {
    /* We need the name only if the file isn't treated as a whole */
    if (!data.wholeFile)
      converter->setObjectName(0, "four squares");

    CORRADE_VERIFY(converter->add(sceneData));

    /* File with a single mesh */
  } else {
    /* Use the scene to create a concatenated mesh */
    const Mn::Trade::MeshData squares[]{
        Mn::MeshTools::generateIndices(Mn::Primitives::planeSolid(
            Mn::Primitives::PlaneFlag::TextureCoordinates))};
    Cr::Containers::Array<Cr::Containers::Pair<
        Mn::UnsignedInt, Cr::Containers::Pair<Mn::UnsignedInt, Mn::Int>>>
        meshesMaterials = sceneData.meshesMaterialsAsArray();
    Cr::Containers::Array<Mn::Matrix4> transformations =
        Mn::SceneTools::absoluteFieldTransformations3D(
            sceneData, Mn::Trade::SceneField::Mesh);
    Cr::Containers::Array<Mn::Trade::MeshData> flattenedMeshes;
    for (std::size_t i = 0; i != meshesMaterials.size(); ++i) {
      arrayAppend(flattenedMeshes,
                  Mn::MeshTools::transform3D(
                      squares[meshesMaterials[i].second().first()],
                      transformations[i]));
    }
    const Mn::Trade::MeshData squaresJoined =
        Mn::MeshTools::concatenate(flattenedMeshes);

    /* Bake materials as per-face vertex colors */
    // clang-format off
    Mn::Color3 faceColors[]{
      /* Each triangle has two faces */
      0xffffff_rgbf, 0xffffff_rgbf,
      0x00ffff_rgbf, 0x00ffff_rgbf,
      0xff00ff_rgbf, 0xff00ff_rgbf,
      0xffff00_rgbf, 0xffff00_rgbf
    };
    // clang-format on
    Mn::Trade::MeshData squaresJoinedColored =
        Mn::MeshTools::combineFaceAttributes(
            squaresJoined, {Mn::Trade::MeshAttributeData{
                               Mn::Trade::MeshAttribute::Color,
                               Cr::Containers::arrayView(faceColors)}});

    CORRADE_VERIFY(converter->add(squaresJoinedColored));
  }

  CORRADE_VERIFY(converter->endFile());

  /* Test that the output matches. Mainly as a trigger to update the in-repo
      test data (pass `-S path/to/habitat_sim/data/test_scenes/` to the test
      executable). */
  CORRADE_COMPARE_AS(filename,
                     Cr::Utility::Path::join({TEST_ASSETS, "scenes",
                                              data.filenamePrefix + ".gltf"_s}),
                     Cr::TestSuite::Compare::File);
  CORRADE_COMPARE_AS(Cr::Utility::Path::join({MAGNUMRENDERERTEST_OUTPUT_DIR,
                                              data.filenamePrefix + ".bin"_s}),
                     Cr::Utility::Path::join({TEST_ASSETS, "scenes/",
                                              data.filenamePrefix + ".bin"_s}),
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

  CORRADE_VERIFY(!renderer.flags());
  CORRADE_COMPARE(!renderer.standaloneFlags(), testCaseInstanceId() == 0);
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
  for (const auto& file : data.gltfFilenames)
    CORRADE_VERIFY(renderer.addFile(
        Cr::Utility::Path::join({TEST_ASSETS, "scenes", file.first()}),
        file.second(), file.third()));

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

  for (const auto& file : data.gltfFilenames)
    CORRADE_VERIFY(renderer.addFile(
        Cr::Utility::Path::join({TEST_ASSETS, "scenes", file.first()}),
        file.second(), file.third()));

  /* Undo the aspect ratio, move camera back */
  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());

  CORRADE_VERIFY(renderer.hasNodeHierarchy("square"));
  CORRADE_VERIFY(!renderer.hasNodeHierarchy("squares"));
  CORRADE_VERIFY(!renderer.hasNodeHierarchy(""));
  CORRADE_COMPARE(
      renderer.addNodeHierarchy(0, "square",
                                /* Initial baked-in transformation, combined
                                   with what's set in transformations() below */
                                Mn::Matrix4::scaling(Mn::Vector3{0.4f})),
      0);

  /* Stats will show two nodes now -- it adds one transformation for the
     top-level object and then one nested for the mesh, corresponding to the
     layout inside the glTF file */
  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  CORRADE_COMPARE(stats.nodeCount, 2);
  CORRADE_COMPARE(stats.drawCount, 1);
  CORRADE_COMPARE(stats.drawBatchCount, 1);

  CORRADE_COMPARE(renderer.transformations(0).size(), 2);
  /* The initial baked-in transformation shouldn't appear here (that's why it's
     baked), combine it so it's a 0.8 scale in total */
  CORRADE_COMPARE(renderer.transformations(0)[0], Mn::Matrix4{});
  renderer.transformations(0)[0] = Mn::Matrix4::scaling(Mn::Vector3{2.0f});

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Check that texture coordinates, image data or whatever else didn't get
     flipped -- there should be a red pixel on the bottom left and grey pixel
     on the top left. */
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestSingleMesh.png"),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));
  CORRADE_COMPARE(color.size(), (Mn::Vector2i{128, 96}));
  CORRADE_COMPARE(color.format(), Mn::PixelFormat::RGBA8Unorm);
  /* Fuzzy-comparing a single packed value is *annoying*; don't */
  if (Mn::Math::equal(data.maxThreshold, 0.0f)) {
    CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[75][35], 0xcccccc_rgb);
    CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[20][38], 0x990000_rgb);
  }

  /* Depth should have *some* data also */
  Mn::Image2D depth = renderer.depthImage();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[0][0], 1.0f);
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[95][127], 1.0f);
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[64][48], 0.0909091f);
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

  for (const auto& file : data.gltfFilenames)
    CORRADE_VERIFY(renderer.addFile(
        Cr::Utility::Path::join({TEST_ASSETS, "scenes", file.first()}),
        file.second(), file.third()));

  /* Undo the aspect ratio, move camera back */
  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());

  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "four squares"), 0);

  /* Stats will show five nodes now -- it adds one transformation for the
     top-level object and then four nested for each mesh, corresponding to the
     layout inside the glTF file. The batch count reflects how many separate
     meshes and textures there are. */
  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  CORRADE_COMPARE(stats.nodeCount, data.nodeCount);
  CORRADE_COMPARE(stats.drawCount, data.drawCount);
  CORRADE_COMPARE(stats.drawBatchCount, data.batchCount);

  CORRADE_COMPARE(renderer.transformations(0).size(), data.nodeCount);
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

  for (const auto& file : data.gltfFilenames)
    CORRADE_VERIFY(renderer.addFile(
        Cr::Utility::Path::join({TEST_ASSETS, "scenes", file.first()}),
        file.second(), file.third()));

  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());

  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "square"), 0);
  renderer.transformations(0)[0] =
      Mn::Matrix4::translation({0.0f, 0.5f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.4f});

  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "circle"), 2);
  renderer.transformations(0)[2] =
      Mn::Matrix4::translation({-0.5f, -0.5f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.4f});

  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "triangle"), 4);
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
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS, "screenshots/GfxBatchRendererTestMultipleMeshes.png"),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));
  CORRADE_COMPARE(color.size(), (Mn::Vector2i{128, 96}));
  CORRADE_COMPARE(color.format(), Mn::PixelFormat::RGBA8Unorm);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[18][44], 0x00cccc_rgb);
  CORRADE_COMPARE(color.pixels<Mn::Color4ub>()[24][88], 0xcc00cc_rgb);
}

void GfxBatchRendererTest::renderNoFileAdded() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({64, 48}, {2, 2}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  renderer.draw();
  MAGNUM_VERIFY_NO_GL_ERROR();
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

  for (const auto& file : data.gltfFilenames)
    CORRADE_VERIFY(renderer.addFile(
        Cr::Utility::Path::join({TEST_ASSETS, "scenes", file.first()}),
        file.second(), file.third()));

  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{1.0f, 4.0f / 3.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());

  /* Scene 0 has one multi-mesh, scene 1 has two single-meshes, scene 2 is
     unused and scene 3 has a single triangle */
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "four squares"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "circle"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "square"), 2);
  CORRADE_COMPARE(renderer.addNodeHierarchy(3, "triangle"), 0);

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
  const auto identity = Mn::Matrix4{Mn::Math::IdentityInit};
  renderer.updateCamera(
      0, identity, Mn::Matrix4::translation({0.0f, 0.0f, 1.0f}).inverted());
  renderer.transformations(0)[0] = Mn::Matrix4::translation({0.0f, 0.0f, 0.0f});

  renderer.updateCamera(
      1, identity, Mn::Matrix4::translation({0.0f, 0.5f, 1.0f}).inverted());
  renderer.transformations(1)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});
  renderer.transformations(1)[2] =
      Mn::Matrix4::translation({-0.5f, 1.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  renderer.updateCamera(
      3, identity, Mn::Matrix4::translation({0.0f, -0.5f, 1.0f}).inverted());
  renderer.transformations(3)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  /* Just a visual test, the texture transformation and material aspects were
     tested well enough above */
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS, "screenshots/GfxBatchRendererTestMultipleScenes.png"),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));
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

  for (const auto& file : data.gltfFilenames)
    CORRADE_VERIFY(renderer.addFile(
        Cr::Utility::Path::join({TEST_ASSETS, "scenes", file.first()}),
        file.second(), file.third()));

  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{1.0f, 4.0f / 3.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());

  /* Like in multipleScenes(), except in different order, there's more stuff
     added to scene 1 and it isn't transformed in any way */
  CORRADE_COMPARE(renderer.addNodeHierarchy(3, "triangle"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "square"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "circle"), 2);
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "triangle"), 4);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "four squares"), 0);

  const auto identity = Mn::Matrix4{Mn::Math::IdentityInit};
  renderer.updateCamera(
      0, identity, Mn::Matrix4::translation({0.0f, 0.0f, 1.0f}).inverted());
  renderer.transformations(0)[0] = Mn::Matrix4::translation({0.0f, 0.0f, 0.0f});

  renderer.updateCamera(
      1, identity, Mn::Matrix4::translation({0.0f, 0.5f, 1.0f}).inverted());

  renderer.updateCamera(
      3, identity, Mn::Matrix4::translation({0.0f, -0.5f, 1.0f}).inverted());
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
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS,
          "screenshots/GfxBatchRendererTestMultipleScenesClearScene1.png"),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));

  /* Add things to scene 1 again, transform them */
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "circle"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(1, "square"), 2);
  renderer.transformations(1)[0] =
      Mn::Matrix4::translation({0.5f, 0.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});
  renderer.transformations(1)[2] =
      Mn::Matrix4::translation({-0.5f, 1.0f, 0.0f}) *
      Mn::Matrix4::scaling(Mn::Vector3{0.5f});

  /* Now it should match the output in multipleScenes() */
  renderer.draw();
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(
          TEST_ASSETS, "screenshots/GfxBatchRendererTestMultipleScenes.png"),
      (Mn::DebugTools::CompareImageToFile{data.maxThreshold,
                                          data.meanThreshold}));
}

void GfxBatchRendererTest::lights() {
  auto&& data = LightData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1})
          .setMaxLightCount(data.maxLightCount),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  CORRADE_VERIFY(renderer.addFile(
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf")));

  /* Undo the aspect ratio, move camera back */
  renderer.updateCamera(
      0,
      Mn::Matrix4::perspectiveProjection(60.0_degf, 4.0f / 3.0f, 0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(5.0f)).inverted());

  /* Add meshes, transform them in place */
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, data.firstObject), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, data.secondObject), 2);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, data.thirdObject), 4);
  renderer.transformations(0)[0] =
      Mn::Matrix4::translation({-1.5f, -1.0f, 0.0f});
  renderer.transformations(0)[2] = Mn::Matrix4::translation({0.0f, 1.0f, 0.0f});
  renderer.transformations(0)[4] =
      Mn::Matrix4::translation({1.5f, -1.0f, 0.0f});

  /* Add lights to new nodes */
  if (data.sceneLightCount) {
    CORRADE_COMPARE(renderer.addEmptyNode(0), 6);
    CORRADE_COMPARE(renderer.addEmptyNode(0), 7);
    renderer.transformations(0)[6] =
        Mn::Matrix4::translation({-1.0f, 1.5f, 1.0f});
    renderer.transformations(0)[7] =
        Mn::Matrix4::lookAt({}, {3.0f, 1.0f, 3.0f}, Mn::Vector3::xAxis());

    for (Mn::UnsignedInt i = 0; i != data.sceneLightCount; ++i) {
      CORRADE_COMPARE(renderer.addLight(
                          0, 6 + (i % 2),
                          i % 2 ? esp::gfx_batch::RendererLightType::Directional
                                : esp::gfx_batch::RendererLightType::Point),
                      i);
      renderer.lightColors(0)[i] =
          i % 2 ? 0xccffff_rgbf * 1.0f : 0x3333ff_rgbf * 3.0f;
      renderer.lightRanges(0)[i] = i % 2 ? Mn::Constants::inf() : 1.0f;
    }
  }

  /* There should be three draws, the textured flat + phong together in one
     batch and the untextured sphere in another. This might change in the
     future if it proves to be faster to draw flat-shaded meshes separately. */
  esp::gfx_batch::SceneStats stats = renderer.sceneStats(0);
  CORRADE_COMPARE(stats.nodeCount, data.sceneLightCount ? 8 : 6);
  CORRADE_COMPARE(stats.drawCount, 3);
  CORRADE_COMPARE(stats.drawBatchCount, 2);

  /* Render for the first time */
  renderer.draw();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join({TEST_ASSETS, "screenshots", data.expected}),
      (Mn::DebugTools::CompareImageToFile{0.75f, 0.005f}));

  /* Rotate if desired to test that the normal matrix is correctly propagated
     every frame, i.e. even if nothing is dirty, and render again */
  renderer.transformations(0)[0] =
      renderer.transformations(0)[0] * Mn::Matrix4::rotationY(data.rotation);
  renderer.transformations(0)[2] =
      renderer.transformations(0)[2] * Mn::Matrix4::rotationY(data.rotation);
  renderer.transformations(0)[4] =
      renderer.transformations(0)[4] * Mn::Matrix4::rotationY(data.rotation);
  renderer.draw();
  Mn::Image2D color = renderer.colorImage();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join({TEST_ASSETS, "screenshots", data.expected}),
      (Mn::DebugTools::CompareImageToFile{95.0f, 0.07f}));
}

void GfxBatchRendererTest::clearLights() {
  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1})
          .setMaxLightCount(2),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  CORRADE_VERIFY(renderer.addFile(
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf")));

  /* Undo the aspect ratio, move camera back */
  renderer.updateCamera(
      0,
      Mn::Matrix4::perspectiveProjection(60.0_degf, 4.0f / 3.0f, 0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(5.0f)).inverted());

  /* Add the same scene as in lights() */
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "flat checkerboard sphere"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "shaded yellow sphere"), 2);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "shaded checkerboard sphere"),
                  4);
  CORRADE_COMPARE(renderer.addEmptyNode(0), 6);
  CORRADE_COMPARE(renderer.addEmptyNode(0), 7);
  renderer.transformations(0)[0] =
      Mn::Matrix4::translation({-1.5f, -1.0f, 0.0f});
  renderer.transformations(0)[2] = Mn::Matrix4::translation({0.0f, 1.0f, 0.0f});
  renderer.transformations(0)[4] =
      Mn::Matrix4::translation({1.5f, -1.0f, 0.0f});
  renderer.transformations(0)[6] =
      Mn::Matrix4::translation({-1.0f, 1.5f, 1.0f});
  renderer.transformations(0)[7] =
      Mn::Matrix4::lookAt({}, {3.0f, 1.0f, 3.0f}, Mn::Vector3::yAxis());

  CORRADE_COMPARE(
      renderer.addLight(0, 6, esp::gfx_batch::RendererLightType::Point), 0);
  CORRADE_COMPARE(
      renderer.addLight(0, 7, esp::gfx_batch::RendererLightType::Directional),
      1);
  renderer.lightColors(0)[0] = 0x3333ff_rgbf * 3.0f;
  renderer.lightColors(0)[1] = 0xccffff_rgbf * 1.0f;
  renderer.lightRanges(0)[0] = 1.0f;
  renderer.lightRanges(0)[1] = Mn::Constants::inf();

  /* Verify the rendering is the same. This will upload the light uniform
     for the first time -- i.e., if it wasn't drawing there, the next draw()
     would be uploading the (empty) light uniform first and thus not testing
     that the clear happens correctly. */
  renderer.draw();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestLights.png"),
      (Mn::DebugTools::CompareImageToFile{0.75f, 0.005f}));

  /* Clearing the lights should not remove nodes */
  CORRADE_COMPARE(renderer.transformations(0).size(), 8);
  CORRADE_COMPARE(renderer.lightColors(0).size(), 2);
  renderer.clearLights(0);
  CORRADE_COMPARE(renderer.transformations(0).size(), 8);
  CORRADE_COMPARE(renderer.lightColors(0).size(), 0);

  /* Rendering now should result in the same as with no lights at all. The
     light uniform is not updated (as there are no lights, thus nothing to
     upload), nevertheless the rendering should not use them and respect the
     per-draw light count. This thus also test that variable light count among
     more than one scene works correctly. */
  renderer.draw();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestLightsNone.png"),
      (Mn::DebugTools::CompareImageToFile{0.75f, 0.005f}));

  renderer.addLight(0, 6, esp::gfx_batch::RendererLightType::Point);
  renderer.addLight(0, 7, esp::gfx_batch::RendererLightType::Directional);
  CORRADE_COMPARE(renderer.lightColors(0).size(), 2);

  /* Clearing everything should remove the lights as well */
  renderer.clear(0);
  CORRADE_COMPARE(renderer.transformations(0).size(), 0);
  CORRADE_COMPARE(renderer.lightColors(0).size(), 0);

  /* Add everything again to test that the internal state isn't corrupted in
     any way */
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "flat checkerboard sphere"), 0);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "shaded yellow sphere"), 2);
  CORRADE_COMPARE(renderer.addNodeHierarchy(0, "shaded checkerboard sphere"),
                  4);
  CORRADE_COMPARE(renderer.addEmptyNode(0), 6);
  CORRADE_COMPARE(renderer.addEmptyNode(0), 7);
  renderer.transformations(0)[0] =
      Mn::Matrix4::translation({-1.5f, -1.0f, 0.0f});
  renderer.transformations(0)[2] = Mn::Matrix4::translation({0.0f, 1.0f, 0.0f});
  renderer.transformations(0)[4] =
      Mn::Matrix4::translation({1.5f, -1.0f, 0.0f});
  renderer.transformations(0)[6] =
      Mn::Matrix4::translation({-1.0f, 1.5f, 1.0f});
  renderer.transformations(0)[7] =
      Mn::Matrix4::lookAt({}, {3.0f, 1.0f, 3.0f}, Mn::Vector3::xAxis());

  CORRADE_COMPARE(
      renderer.addLight(0, 6, esp::gfx_batch::RendererLightType::Point), 0);
  CORRADE_COMPARE(
      renderer.addLight(0, 7, esp::gfx_batch::RendererLightType::Directional),
      1);
  renderer.lightColors(0)[0] = 0x3333ff_rgbf * 3.0f;
  renderer.lightColors(0)[1] = 0xccffff_rgbf * 1.0f;
  renderer.lightRanges(0)[0] = 1.0f;
  renderer.lightRanges(0)[1] = Mn::Constants::inf();

  renderer.draw();
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_WITH(
      renderer.colorImage(),
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestLights.png"),
      (Mn::DebugTools::CompareImageToFile{0.75f, 0.005f}));
}

void GfxBatchRendererTest::imageInto() {
  /* Same as singleMesh(), just with a lot less checking and using *ImageInto()
     instead of *Image() */

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount({128, 96}, {1, 1}),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  /* Mostly the same as singleMesh() */
  CORRADE_VERIFY(renderer.addFile(
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf")));
  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());
  renderer.addNodeHierarchy(0, "square",
                            Mn::Matrix4::scaling(Mn::Vector3{0.8f}));
  renderer.draw();

  // TODO use the NoInit constructor once it exists
  Mn::Image2D color{
      Mn::PixelFormat::RGBA8Unorm, renderer.tileCount() * renderer.tileSize(),
      Cr::Containers::Array<char>{
          Cr::NoInit,
          std::size_t(
              (renderer.tileCount() * renderer.tileSize() * 4).product())}};
  renderer.colorImageInto({{}, color.size()}, color);
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE_AS(
      color,
      Cr::Utility::Path::join(TEST_ASSETS,
                              "screenshots/GfxBatchRendererTestSingleMesh.png"),
      Mn::DebugTools::CompareImageToFile);

  /* Depth should have *some* data also */
  // TODO use the NoInit constructor once it exists
  Mn::Image2D depth{
      Mn::PixelFormat::Depth32F, renderer.tileCount() * renderer.tileSize(),
      Cr::Containers::Array<char>{
          Cr::NoInit,
          std::size_t(
              (renderer.tileCount() * renderer.tileSize() * 4).product())}};
  renderer.depthImageInto({{}, depth.size()}, depth);
  MAGNUM_VERIFY_NO_GL_ERROR();
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[0][0], 1.0f);
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[95][127], 1.0f);
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[64][48], 0.0909091f);
}

void GfxBatchRendererTest::depthUnprojection() {
  constexpr Mn::Vector2i tileCount{2, 2};
  constexpr float near = 0.001f;
  constexpr float far = 10.0f;
  constexpr Mn::Vector2i tileSize(64, 64);

  // clang-format off
  esp::gfx_batch::RendererStandalone renderer{
      esp::gfx_batch::RendererConfiguration{}
          .setTileSizeCount(tileSize, tileCount),
      esp::gfx_batch::RendererStandaloneConfiguration{}
          .setFlags(esp::gfx_batch::RendererStandaloneFlag::QuietLog)
  };
  // clang-format on

  CORRADE_VERIFY(renderer.addFile(
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf")));

  // Place environment cameras at various distances from origin.
  const auto& projection =
      Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, near, far);
  renderer.updateCamera(
      0, projection,
      Mn::Matrix4::translation(Mn::Vector3::zAxis(2.5f)).inverted());
  renderer.updateCamera(
      1, projection,
      Mn::Matrix4::translation(Mn::Vector3::zAxis(5.0f)).inverted());
  renderer.updateCamera(
      2, projection,
      Mn::Matrix4::translation(Mn::Vector3::zAxis(7.5f)).inverted());
  renderer.updateCamera(
      3, projection,
      Mn::Matrix4::translation(Mn::Vector3::zAxis(20.0f)).inverted());

  // Spawn a plane in each environment, at origin, facing the camera.
  CORRADE_VERIFY(renderer.hasNodeHierarchy("square"));
  for (int i = 0; i < tileCount.product(); ++i) {
    CORRADE_COMPARE(renderer.addNodeHierarchy(i, "square"), 0);
  }

  // Render.
  renderer.draw();
  Mn::Image2D depth = renderer.depthImage();
  MAGNUM_VERIFY_NO_GL_ERROR();

  // Unproject each scene.
  for (int y = 0; y != tileCount.y(); ++y) {
    for (int x = 0; x != tileCount.x(); ++x) {
      const std::size_t sceneId = y * tileCount.x() + x;
      const Mn::Containers::Size2D offset(x * tileSize.x(), y * tileSize.y());
      const Mn::Containers::Size2D size(tileSize.x(), tileSize.y());
      esp::gfx_batch::unprojectDepth(
          renderer.cameraDepthUnprojection(sceneId),
          depth.pixels<Mn::Float>().sliceSize(offset, size));
    }
  }

  // Unprojected depth should read the distance between the cameras and the
  // plane, in meters. For each environment, the center pixel is compared with
  // the camera distance from origin.
  // Target 0 is at 2.5 meters from the camera
  CORRADE_COMPARE_WITH(depth.pixels<Mn::Float>()[32][32], 2.5f,
                       Corrade::TestSuite::Compare::around(0.01f));
  // Target 1 is at 5.0 meters from the camera
  CORRADE_COMPARE_WITH(depth.pixels<Mn::Float>()[32][96], 5.0f,
                       Corrade::TestSuite::Compare::around(0.01f));
  // Target 2 is at 7.5 meters from the camera
  CORRADE_COMPARE_WITH(depth.pixels<Mn::Float>()[96][32], 7.5f,
                       Corrade::TestSuite::Compare::around(0.01f));
  // Target 3 is beyond the far plane. Here, unprojected depth is set to 0.
  CORRADE_COMPARE(depth.pixels<Mn::Float>()[96][96], 0.0f);
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
  CORRADE_VERIFY(renderer.addFile(
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/batch.gltf")));
  renderer.updateCamera(
      0,
      Mn::Matrix4::orthographicProjection(2.0f * Mn::Vector2{4.0f / 3.0f, 1.0f},
                                          0.1f, 10.0f),
      Mn::Matrix4::translation(Mn::Vector3::zAxis(1.0f)).inverted());
  renderer.addNodeHierarchy(0, "square");
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
