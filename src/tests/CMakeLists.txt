macro(TEST TEST_NAME)
  add_executable(${TEST_NAME} "${TEST_NAME}.cpp")
  target_link_libraries(${TEST_NAME} core gtest_main)
  set(DEPENDENCIES "${ARGN}")
  foreach(DEPENDENCY IN LISTS DEPENDENCIES)
    target_link_libraries(${TEST_NAME} ${DEPENDENCY})
  endforeach()
  add_test(${TEST_NAME} ${TEST_NAME})
endmacro(TEST)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/configure.h)

TEST(CoreTest io)

TEST(NavTest nav assets)
target_include_directories(NavTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

TEST(IOTest io)
target_include_directories(IOTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

TEST(GeoTest geo)

TEST(Mp3dTest scene)
target_include_directories(Mp3dTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

TEST(SimTest sim)
target_include_directories(SimTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

TEST(PhysicsTest physics)
target_include_directories(PhysicsTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

TEST(SuncgTest scene)
target_include_directories(SuncgTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Some tests are LOUD, we don't want to include their full log (but OTOH we
# want to have full log from others, so this is a compromise)
set_tests_properties(
  NavTest
  Mp3dTest
  SuncgTest
  PROPERTIES ENVIRONMENT GLOG_minloglevel=1
)
set_tests_properties(SimTest PROPERTIES
  ENVIRONMENT "GLOG_minloglevel=1;MAGNUM_LOG=QUIET")
