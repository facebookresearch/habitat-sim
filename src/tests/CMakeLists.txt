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

test(CoreTest io)

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

test(SceneGraphTest scene)
target_include_directories(SceneGraphTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

corrade_add_test(SensorTest SensorTest.cpp LIBRARIES sensor sim)
target_include_directories(SensorTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Some tests are LOUD, we don't want to include their full log (but OTOH we
# want to have full log from others, so this is a compromise)
set_tests_properties(
  SimTest PROPERTIES ENVIRONMENT "GLOG_minloglevel=1;MAGNUM_LOG=QUIET"
)
