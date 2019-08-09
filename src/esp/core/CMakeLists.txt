file(GLOB_RECURSE core_SOURCES "*.cpp")

if(NOT BUILD_GUI_VIEWERS AND CORRADE_TARGET_UNIX AND NOT CORRADE_TARGET_APPLE)
  set(ESP_BUILD_EGL_SUPPORT ON)
endif()

if(CORRADE_TARGET_EMSCRIPTEN)
  set(ESP_BUILD_GLOG_SHIM ON)
endif()

if(BUILD_PTEX_SUPPORT)
  set(ESP_BUILD_PTEX_SUPPORT ON)
endif()

if(BUILD_GPU_GPU)
  set(ESP_BUILD_GPU_GPU ON)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/configure.h)


find_package(Corrade REQUIRED Utility)

add_library(core STATIC ${core_SOURCES})

target_link_libraries(core
  PUBLIC
    Corrade::Utility
    glog
)

target_include_directories(core
  PUBLIC
    "${PROJECT_BINARY_DIR}")
