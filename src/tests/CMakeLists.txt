find_package(Magnum REQUIRED DebugTools AnyImageConverter)
find_package(MagnumPlugins REQUIRED StbImageImporter)

macro(TEST TEST_NAME)
  add_executable(${TEST_NAME} "${TEST_NAME}.cpp")
  target_link_libraries(${TEST_NAME} core gtest_main)
  set(DEPENDENCIES "${ARGN}")
  foreach(DEPENDENCY IN LISTS DEPENDENCIES)
    target_link_libraries(${TEST_NAME} ${DEPENDENCY})
  endforeach()
  add_test(${TEST_NAME} ${TEST_NAME})
endmacro(TEST)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

test(AttributesManagersTest assets metadata)
target_include_directories(AttributesManagersTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(CoreTest io)

test(MetadataMediatorTest assets metadata)
target_include_directories(MetadataMediatorTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(NavTest nav assets)
target_include_directories(NavTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(IOTest io metadata)
target_include_directories(IOTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(Mp3dTest scene)
target_include_directories(Mp3dTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

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

corrade_add_test(GeoTest GeoTest.cpp LIBRARIES geo)

corrade_add_test(DrawableTest DrawableTest.cpp LIBRARIES gfx)
target_include_directories(DrawableTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(PhysicsTest physics)
target_include_directories(PhysicsTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(GfxReplayTest assets gfx sim)
target_include_directories(GfxReplayTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(ResourceManagerTest assets)
target_include_directories(ResourceManagerTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

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

corrade_add_test(CullingTest CullingTest.cpp LIBRARIES gfx)
target_include_directories(CullingTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(SuncgTest scene)
target_include_directories(SuncgTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

test(SceneGraphTest scene)
target_include_directories(SceneGraphTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(SensorTest SensorTest.cpp LIBRARIES sensor sim)
target_include_directories(SensorTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Some tests are LOUD, we don't want to include their full log (but OTOH we
# want to have full log from others, so this is a compromise)
set_tests_properties(
  NavTest Mp3dTest SuncgTest PROPERTIES ENVIRONMENT "HABITAT_SIM_LOG=nav,scene=quiet"
)
set_tests_properties(
  SimTest PROPERTIES ENVIRONMENT "HABITAT_SIM_LOG=quiet;MAGNUM_LOG=QUIET"
)
