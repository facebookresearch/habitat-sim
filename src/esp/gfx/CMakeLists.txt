set(
  gfx_SOURCES
  CubeMap.cpp
  CubeMap.h
  DepthUnprojection.cpp
  DepthUnprojection.h
  Drawable.cpp
  Drawable.h
  DrawableGroup.cpp
  DrawableGroup.h
  GenericDrawable.cpp
  GenericDrawable.h
  MeshVisualizerDrawable.cpp
  MeshVisualizerDrawable.h
  LightSetup.cpp
  LightSetup.h
  MaterialData.h
  MaterialUtil.cpp
  MaterialUtil.h
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
  PbrImageBasedLighting.cpp
  PbrImageBasedLighting.h
  PbrEquiRectangularToCubeMapShader.cpp
  PbrEquiRectangularToCubeMapShader.h
  PbrPrecomputedMapShader.cpp
  PbrPrecomputedMapShader.h
  PbrTextureUnit.h
  DepthMapDrawableBase.cpp
  DepthMapDrawableBase.h
  ShadowMapManager.h
  VarianceShadowMapShader.h
  VarianceShadowMapShader.cpp
  VarianceShadowMapDrawable.h
  VarianceShadowMapDrawable.cpp
  GaussianFilterShader.h
  GaussianFilterShader.cpp
)

# If ptex support is enabled add relevant source files
if(BUILD_PTEX_SUPPORT)
  list(
    APPEND
    gfx_SOURCES
    PTexMeshDrawable.cpp
    PTexMeshDrawable.h
    PTexMeshShader.cpp
    PTexMeshShader.h
  )
endif()

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
# TODO: enable the following flag and fix the compilation warnings
# set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
corrade_add_resource(ShaderResources ../../shaders/Shaders.conf)
list(APPEND gfx_SOURCES ${ShaderResources})

corrade_add_resource(PbrImageResources ../../../data/pbr/PbrImages.conf)
list(APPEND gfx_SOURCES ${PbrImageResources})

add_library(
  gfx STATIC
  ${gfx_SOURCES}
)

if(BUILD_WITH_CUDA)
  target_include_directories(
    gfx PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
                ${CMAKE_CURRENT_LIST_DIR}/cuda_helpers
  )

  target_link_libraries(gfx PUBLIC ${CUDART_LIBRARY})
endif()

target_link_libraries(
  gfx
  PUBLIC assets
         core
         io
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
if(APPLE)
  find_package(Magnum REQUIRED WindowlessCglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessCglApplication)
elseif(WIN32)
  find_package(Magnum REQUIRED WindowlessWglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessWglApplication)
elseif(CORRADE_TARGET_EMSCRIPTEN)
  find_package(Magnum REQUIRED WindowlessEglApplication)
  target_link_libraries(gfx PUBLIC Magnum::WindowlessEglApplication)
elseif(UNIX)
  if(BUILD_GUI_VIEWERS)
    find_package(Magnum REQUIRED WindowlessGlxApplication)
    target_link_libraries(gfx PUBLIC Magnum::WindowlessGlxApplication)
  else()
    find_package(Magnum REQUIRED WindowlessEglApplication)
    target_link_libraries(gfx PUBLIC Magnum::WindowlessEglApplication)
  endif()
endif()
