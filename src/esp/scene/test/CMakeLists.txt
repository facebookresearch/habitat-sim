include(GoogleTest)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

add_executable(GibsonSceneTest GibsonSceneTest.cpp)
target_link_libraries(
  GibsonSceneTest gtest_main scene sim
)

gtest_discover_tests(GibsonSceneTest)

find_package(Corrade REQUIRED Utility TestSuite)
corrade_add_test(
  ReplicaSceneTest
  ReplicaSceneTest.cpp
  LIBRARIES
  scene
  assets
  sim
  Corrade::Utility
)
target_include_directories(ReplicaSceneTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
