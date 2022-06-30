find_package(Magnum REQUIRED DebugTools AnyImageConverter)
find_package(
  MagnumPlugins REQUIRED StbImageImporter KtxImporter
  OPTIONAL_COMPONENTS GltfSceneConverter KtxImageConverter
)

# If the GltfSceneConverter is present, the new Magnum SceneConverter APIs are
# as well. Which enables a data-generating test case in BatchRendererTest.
if(MagnumPlugins_GltfSceneConverter_FOUND)
  set(HAS_MAGNUM_GLTFSCENECONVERTER ON)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

corrade_add_test(
  AttributesConfigsTest
  AttributesConfigsTest.cpp
  LIBRARIES
  core
  assets
  metadata
)
target_include_directories(AttributesConfigsTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  AttributesManagersTest
  AttributesManagersTest.cpp
  LIBRARIES
  core
  assets
  metadata
)
target_include_directories(AttributesManagersTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  BatchRendererTest
  BatchRendererTest.cpp
  LIBRARIES
  gfx
  Magnum::DebugTools
  MagnumPlugins::KtxImporter
)
target_include_directories(BatchRendererTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(CoreTest CoreTest.cpp LIBRARIES core io)

corrade_add_test(CullingTest CullingTest.cpp LIBRARIES gfx)
target_include_directories(CullingTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  DepthUnprojectionTest
  DepthUnprojectionTest.cpp
  LIBRARIES
  gfx
  Magnum::MeshTools
  Magnum::OpenGLTester
  Magnum::Trade
  Magnum::Primitives
)

corrade_add_test(DrawableTest DrawableTest.cpp LIBRARIES gfx)
target_include_directories(DrawableTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(GeoTest GeoTest.cpp LIBRARIES geo)

corrade_add_test(
  GfxReplayTest
  GfxReplayTest.cpp
  LIBRARIES
  assets
  gfx
  sim
)
target_include_directories(GfxReplayTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(GibsonSceneTest GibsonSceneTest.cpp LIBRARIES scene sim)
target_include_directories(GibsonSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(HM3DSceneTest HM3DSceneTest.cpp LIBRARIES scene sim)
target_include_directories(HM3DSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(IOTest IOTest.cpp LIBRARIES io metadata)
target_include_directories(IOTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(LoggingTest LoggingTest.cpp LIBRARIES core)
set_tests_properties(LoggingTest PROPERTIES ENVIRONMENT HABITAT_SIM_LOG="")

corrade_add_test(
  MetadataMediatorTest MetadataMediatorTest.cpp LIBRARIES assets metadata
)
target_include_directories(MetadataMediatorTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(Mp3dTest Mp3dTest.cpp LIBRARIES scene)
target_include_directories(Mp3dTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  NavTest
  NavTest.cpp
  LIBRARIES
  core
  nav
  assets
)
target_include_directories(NavTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(PathFinderTest PathFinderTest.cpp LIBRARIES nav Corrade::Utility)
target_include_directories(PathFinderTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(PhysicsTest PhysicsTest.cpp LIBRARIES physics)
target_include_directories(PhysicsTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(
  ReplicaSceneTest
  ReplicaSceneTest.cpp
  LIBRARIES
  scene
  assets
  sim
)
target_include_directories(ReplicaSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(ResourceManagerTest ResourceManagerTest.cpp LIBRARIES assets)
target_include_directories(ResourceManagerTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(SceneGraphTest SceneGraphTest.cpp LIBRARIES scene)

corrade_add_test(SensorTest SensorTest.cpp LIBRARIES sensor sim)

corrade_add_test(
  SimTest
  SimTest.cpp
  LIBRARIES
  sim
  Magnum::DebugTools
  Magnum::AnyImageConverter
  MagnumPlugins::StbImageImporter
)
target_include_directories(SimTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

if(BUILD_WITH_VHACD)
  corrade_add_test(
    VoxelGridTest
    VoxelGridTest.cpp
    LIBRARIES
    sim
    assets
    geo
    Magnum::DebugTools
    Magnum::AnyImageConverter
    MagnumPlugins::StbImageImporter
  )
  target_include_directories(VoxelGridTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()

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
