if(NOT BUILD_GUI_VIEWERS AND CORRADE_TARGET_UNIX AND NOT CORRADE_TARGET_APPLE)
  set(ESP_BUILD_EGL_SUPPORT ON)
endif()

if(CORRADE_TARGET_EMSCRIPTEN)
  set(ESP_BUILD_GLOG_SHIM ON)
endif()

if(BUILD_ASSIMP_SUPPORT)
  set(ESP_BUILD_ASSIMP_SUPPORT ON)
endif()

if(BUILD_PTEX_SUPPORT)
  set(ESP_BUILD_PTEX_SUPPORT ON)
endif()

if(BUILD_WITH_CUDA)
  set(ESP_BUILD_WITH_CUDA ON)
endif()

if(BUILD_WITH_BULLET)
  set(ESP_BUILD_WITH_BULLET ON)
endif()

if(BUILD_WITH_VHACD)
  set(ESP_BUILD_WITH_VHACD ON)
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

find_package(Corrade REQUIRED Utility)

add_library(
  core STATIC
  Buffer.cpp
  Buffer.h
  Check.cpp
  Check.h
  Configuration.h
  esp.cpp
  esp.h
  logging.h
  managedContainers/AbstractManagedObject.h
  managedContainers/ManagedContainer.h
  managedContainers/ManagedContainerBase.cpp
  managedContainers/ManagedContainerBase.h
  managedContainers/ManagedFileBasedContainer.h
  random.h
  spimpl.h
  Utility.h
)

target_link_libraries(
  core
  PUBLIC Corrade::Utility Magnum::Magnum glog
)

target_include_directories(core PUBLIC ${PROJECT_BINARY_DIR})
