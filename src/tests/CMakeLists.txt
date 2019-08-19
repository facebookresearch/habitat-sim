macro(TEST TEST_NAME)
  add_executable(${TEST_NAME} "${TEST_NAME}.cpp")
  target_link_libraries(${TEST_NAME} core gtest_main)
  set(DEPENDENCIES "${ARGN}")
  foreach(DEPENDENCY IN LISTS DEPENDENCIES)
    target_link_libraries(${TEST_NAME} ${DEPENDENCY})
  endforeach()
  add_test(${TEST_NAME} ${TEST_NAME})
endmacro(TEST)

set(SCENE_DATASETS ${CMAKE_CURRENT_SOURCE_DIR}/../../data/scene_datasets)
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

TEST(SuncgTest scene)
target_include_directories(SuncgTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
