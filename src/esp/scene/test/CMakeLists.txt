include(GoogleTest)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/configure.h)

add_executable(GibsonSceneTest GibsonSceneTest.cpp)
target_link_libraries(GibsonSceneTest gtest_main scene)

gtest_discover_tests(GibsonSceneTest)
