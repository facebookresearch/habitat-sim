# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH ./data/default.pbr_config.json)
set(ESP_DEFAULT_PBRSHADER_CONFIG
    ${PROJECT_SOURCE_DIR}/.${ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h
)

set(
  gfx_SOURCES
  CubeMap.cpp
  CubeMap.h
  Drawable.cpp
  Drawable.h
  DrawableConfiguration.cpp
  DrawableConfiguration.h
  DrawableGroup.cpp
  DrawableGroup.h
  GenericDrawable.cpp
  GenericDrawable.h
  SkinData.h
  MeshVisualizerDrawable.cpp
  MeshVisualizerDrawable.h
  LightSetup.cpp
  LightSetup.h
  magnum.h
  RenderCamera.cpp
  RenderCamera.h
  CubeMapCamera.cpp
  CubeMapCamera.h
  CubeMap.cpp
  CubeMap.h
  DebugLineRender.cpp
  DebugLineRender.h
  Renderer.cpp
  Renderer.h
  replay/Keyframe.h
  replay/Player.cpp
  replay/Player.h
  replay/Recorder.cpp
  replay/Recorder.h
  replay/ReplayManager.h
  replay/ReplayManager.cpp
  WindowlessContext.cpp
  WindowlessContext.h
  RenderTarget.cpp
  RenderTarget.h
  ShaderManager.cpp
  ShaderManager.h
  PbrShader.cpp
  PbrShader.h
  PbrDrawable.cpp
  PbrDrawable.h
  TextureVisualizerShader.cpp
  TextureVisualizerShader.h
  CubeMapShaderBase.cpp
  CubeMapShaderBase.h
  DoubleSphereCameraShader.cpp
  DoubleSphereCameraShader.h
  EquirectangularShader.cpp
  EquirectangularShader.h
  PbrIBLHelper.cpp
  PbrIBLHelper.h
  PbrEquiRectangularToCubeMapShader.cpp
  PbrEquiRectangularToCubeMapShader.h
  PbrPrecomputedMapShader.cpp
  PbrPrecomputedMapShader.h
  PbrTextureUnit.h
  GaussianFilterShader.h
  GaussianFilterShader.cpp
)

if(BUILD_WITH_BACKGROUND_RENDERER)
  list(APPEND gfx_SOURCES BackgroundRenderer.h BackgroundRenderer.cpp)
endif()

find_package(
  Magnum
  REQUIRED
  AnyImageImporter
  AnySceneImporter
  GL
  MeshTools
  DebugTools
  SceneGraph
  Shaders
  Trade
  AnyImageConverter
)

find_package(MagnumPlugins REQUIRED GltfImporter StbImageImporter StbImageConverter)

find_package(MagnumIntegration REQUIRED Eigen)

find_package(Corrade REQUIRED Utility)
corrade_add_resource(GfxShaderResources ../../shaders/gfx/Shaders.conf)
list(APPEND gfx_SOURCES ${GfxShaderResources})

add_library(
  gfx STATIC
  ${gfx_SOURCES}
)

if(BUILD_WITH_CUDA)
  target_link_libraries(gfx PUBLIC ${CUDART_LIBRARY})
endif()

target_link_libraries(
  gfx
  PUBLIC assets
         core
         io
         gfx_batch
         physics
         Magnum::AnyImageImporter
         Magnum::AnySceneImporter
         Magnum::GL
         Magnum::Magnum
         Magnum::MeshTools
         Magnum::DebugTools
         Magnum::SceneGraph
         Magnum::Shaders
         Magnum::Trade
         MagnumPlugins::GltfImporter
         MagnumPlugins::StbImageImporter
         MagnumPlugins::StbImageConverter
         MagnumIntegration::Eigen
         Corrade::Utility
         Magnum::AnyImageConverter
)

if(BUILD_WITH_BACKGROUND_RENDERER)
  target_link_libraries(gfx PUBLIC atomic_wait)
endif()

# Link windowed application library if needed
if(BUILD_GUI_VIEWERS)
  if(CORRADE_TARGET_EMSCRIPTEN)
    find_package(Magnum REQUIRED EmscriptenApplication)
    target_link_libraries(gfx PUBLIC Magnum::EmscriptenApplication)
  else()
    find_package(Magnum REQUIRED GlfwApplication)
    target_link_libraries(gfx PUBLIC Magnum::GlfwApplication)
  endif()
endif()

# Link appropriate windowless library
if(MAGNUM_TARGET_EGL)
  # Includes also Emscripten
  find_package(Magnum REQUIRED WindowlessEglApplication)
elseif(CORRADE_TARGET_APPLE)
  find_package(Magnum REQUIRED WindowlessCglApplication)
elseif(CORRADE_TARGET_UNIX)
  # Mainly for builds with external Magnum that might not have TARGET_EGL
  # enabled
  find_package(Magnum REQUIRED WindowlessGlxApplication)
elseif(CORRADE_TARGET_WINDOWS)
  find_package(Magnum REQUIRED WindowlessWglApplication)
else()
  message(FATAL_ERROR "Unsupported platform")
endif()
target_link_libraries(
  gfx
  PUBLIC Magnum::WindowlessApplication
)
