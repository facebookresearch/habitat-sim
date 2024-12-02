# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(
  Magnum
  REQUIRED
  DebugTools
  AnyImageConverter
  AnySceneImporter
  AnyImageImporter
)
find_package(
  MagnumPlugins
  REQUIRED
  GltfImporter
  StbImageImporter
  KtxImporter
  StbImageConverter
  OPTIONAL_COMPONENTS GltfSceneConverter KtxImageConverter StanfordImporter
                      OpenExrImporter OpenExrImageConverter
)

if(BUILD_WITH_BACKGROUND_RENDERER)
  find_package(MagnumPlugins REQUIRED OpenExrImporter OpenExrImageConverter)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

corrade_add_test(AttributesConfigsTest AttributesConfigsTest.cpp LIBRARIES habsim)
target_include_directories(AttributesConfigsTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(AttributesManagersTest AttributesManagersTest.cpp LIBRARIES habsim)
target_include_directories(AttributesManagersTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  GfxBatchRendererTest
  GfxBatchRendererTest.cpp
  LIBRARIES
  gfx_batch
  Magnum::DebugTools
  Magnum::AnyImageConverter
  Magnum::AnySceneImporter
  MagnumPlugins::GltfImporter
  MagnumPlugins::KtxImporter
  MagnumPlugins::StbImageImporter
)
target_include_directories(GfxBatchRendererTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
if(BUILD_WITH_CUDA)
  target_include_directories(
    GfxBatchRendererTest PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  )
endif()
if(MagnumPlugins_GltfSceneConverter_FOUND)
  target_link_libraries(GfxBatchRendererTest PRIVATE MagnumPlugins::GltfSceneConverter)
endif()
if(MagnumPlugins_KtxImageConverter_FOUND)
  target_link_libraries(GfxBatchRendererTest PRIVATE MagnumPlugins::KtxImageConverter)
endif()

corrade_add_test(ConfigurationTest ConfigurationTest.cpp LIBRARIES habsim)

corrade_add_test(CullingTest CullingTest.cpp LIBRARIES habsim)
target_include_directories(CullingTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  DepthUnprojectionTest
  DepthUnprojectionTest.cpp
  LIBRARIES
  gfx_batch
  Magnum::MeshTools
  Magnum::OpenGLTester
  Magnum::Trade
  Magnum::Primitives
)

corrade_add_test(DrawableTest DrawableTest.cpp LIBRARIES habsim)
target_include_directories(DrawableTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(GeoTest GeoTest.cpp LIBRARIES habsim)

corrade_add_test(
  GfxBatchHbaoTest
  GfxBatchHbaoTest.cpp
  LIBRARIES
  gfx_batch
  Magnum::DebugTools
  Magnum::OpenGLTester
  Magnum::MeshTools
  Magnum::AnyImageConverter
  Magnum::AnyImageImporter
  Magnum::AnySceneImporter
  MagnumPlugins::OpenExrImageConverter
  MagnumPlugins::OpenExrImporter
  MagnumPlugins::StanfordImporter
  MagnumPlugins::StbImageImporter
  MagnumPlugins::StbImageConverter
)
target_include_directories(GfxBatchHbaoTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(GfxReplayTest GfxReplayTest.cpp LIBRARIES habsim)
target_include_directories(GfxReplayTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

if(BUILD_WITH_BACKGROUND_RENDERER)
  corrade_add_test(
    BatchReplayRendererTest
    BatchReplayRendererTest.cpp
    LIBRARIES
    habsim
    gfx_batch
    MagnumPlugins::OpenExrImageConverter
    MagnumPlugins::OpenExrImporter
  )
  target_include_directories(
    BatchReplayRendererTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
  )
endif()

corrade_add_test(GibsonSceneTest GibsonSceneTest.cpp LIBRARIES habsim)
target_include_directories(GibsonSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(HM3DSceneTest HM3DSceneTest.cpp LIBRARIES habsim)
target_include_directories(HM3DSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(IOTest IOTest.cpp LIBRARIES habsim)
target_include_directories(IOTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(LoggingTest LoggingTest.cpp LIBRARIES habsim)
set_tests_properties(LoggingTest PROPERTIES ENVIRONMENT HABITAT_SIM_LOG="")

corrade_add_test(MetadataMediatorTest MetadataMediatorTest.cpp LIBRARIES habsim)
target_include_directories(MetadataMediatorTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(Mp3dTest Mp3dTest.cpp LIBRARIES habsim)
target_include_directories(Mp3dTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(NavTest NavTest.cpp LIBRARIES habsim)
target_include_directories(NavTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(PathFinderTest PathFinderTest.cpp LIBRARIES habsim Corrade::Utility)
target_include_directories(PathFinderTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(PhysicsTest PhysicsTest.cpp LIBRARIES habsim)
target_include_directories(PhysicsTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(ReplicaSceneTest ReplicaSceneTest.cpp LIBRARIES habsim)
target_include_directories(ReplicaSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(ResourceManagerTest ResourceManagerTest.cpp LIBRARIES habsim)
target_include_directories(ResourceManagerTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(SceneGraphTest SceneGraphTest.cpp LIBRARIES habsim)

corrade_add_test(SemanticTest SemanticTest.cpp LIBRARIES habsim)
target_include_directories(SemanticTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(SensorTest SensorTest.cpp LIBRARIES habsim)

corrade_add_test(
  SimTest
  SimTest.cpp
  LIBRARIES
  habsim
  Magnum::DebugTools
  Magnum::AnyImageConverter
  MagnumPlugins::StbImageImporter
)
target_include_directories(SimTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Some tests are LOUD, we don't want to include their full log (but OTOH we
# want to have full log from others, so this is a compromise)
set_tests_properties(
  GfxReplayTest
  HM3DSceneTest
  MetadataMediatorTest
  Mp3dTest
  NavTest
  PhysicsTest
  ReplicaSceneTest
  ResourceManagerTest
  SimTest
  PROPERTIES ENVIRONMENT "HABITAT_SIM_LOG=quiet;MAGNUM_LOG=QUIET"
)
